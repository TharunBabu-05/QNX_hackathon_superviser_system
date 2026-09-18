// safety_supervisor -- the one process every other process here answers to.
//
// It never touches hardware directly. Its only job is to sit on a QNX
// message-passing channel, receive sensor data and heartbeats from
// ultrasonic_task and imu_task, and turn that into a safety decision:
// is the vehicle's environment being sensed reliably right now, and if
// not, what should happen. That decision-making is deliberately kept in
// its own process, isolated by the microkernel's address-space
// separation from the sensor drivers -- a bug or hang in imu_task's I2C
// code cannot corrupt this process's state or take it down with it.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <pthread.h>
#include <sched.h>
#include <time.h>
#include <errno.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "protocol.h"

namespace {

// ---- Tunables -----------------------------------------------------------

constexpr unsigned SAFETY_TICK_PERIOD_MS = 50;

// A reading older than this can no longer be trusted for a safety
// decision, even though the sending process might still be alive.
constexpr uint64_t ULTRASONIC_DEADLINE_NS = 300ULL * 1000000; // task samples every 100ms
constexpr uint64_t IMU_DEADLINE_NS        = 300ULL * 1000000; // task samples every 50ms

// A process that misses this many heartbeat periods is declared dead,
// independent of whether its readings were otherwise still arriving.
constexpr uint64_t HEARTBEAT_TIMEOUT_NS = 250ULL * 1000000; // ~5 missed 50ms beats

constexpr float OBSTACLE_THRESHOLD_CM = 20.0f;

// How long a dead process gets to reconnect on its own before this logs
// a recovery prompt, and how many times it will repeat that before going
// quiet and staying in SAFE_STOP for a human to intervene.
//
// This deliberately does NOT call posix_spawn() to respawn anything.
// Earlier it did, and that wedged this process permanently: posix_spawn()
// talks to procnto to create the child, and this thread runs at the
// system's highest SCHED_FIFO priority with no scheduling attributes
// passed to posix_spawn() to override that for the child -- so the new
// process likely inherited it, and creating a second max-priority
// real-time thread while this one sat blocked waiting for procnto's
// reply produced a hang that not even SIGKILL could clear. The detection
// and timing logic below is unchanged; only the actual respawn action is.
constexpr uint64_t RECOVERY_GRACE_NS     = 2000ULL * 1000000;
constexpr unsigned MAX_RESPAWN_ATTEMPTS  = 3;

// ---- State ----------------------------------------------------------------

struct SubsystemState {
    bool     processAlive           = false;
    uint64_t lastHeartbeatNs        = 0;
    uint64_t lastMsgNs              = 0;
    SensorHealth dataHealth         = SensorHealth::Dead;
    uint64_t recoveryStartedNs      = 0; // 0 == not currently recovering
    unsigned respawnAttempts        = 0;
};

SubsystemState g_ultrasonic;
SubsystemState g_imu;

float g_lastDistanceCm    = -1.0f;
float g_lastAccelG[3]     = {0, 0, 0};

SystemState g_state       = SystemState::Recovery; // nothing has reported in yet
bool        g_overrideActive = false;

SafetyEvent g_events[MAX_EVENT_LOG];
unsigned    g_eventHead  = 0;
unsigned    g_eventCount = 0;

const char* stateName(SystemState s) {
    switch (s) {
    case SystemState::Normal:   return "NORMAL";
    case SystemState::Degraded: return "DEGRADED";
    case SystemState::SafeStop: return "SAFE_STOP";
    case SystemState::Recovery: return "RECOVERY";
    }
    return "?";
}

void logEvent(SafetyEventType type, float value) {
    g_events[g_eventHead] = { monotonicNs(), type, value };
    g_eventHead = (g_eventHead + 1) % MAX_EVENT_LOG;
    if (g_eventCount < MAX_EVENT_LOG) g_eventCount++;
}

// ---- Priority override ---------------------------------------------------

// Runs this process's main thread at the highest real-time priority in
// the system so its periodic safety tick and message dispatch always
// preempt the sensor tasks -- the concrete, scheduler-level meaning of
// "priority override" rather than just a naming convention.
void raisePriority() {
    struct sched_param sp;
    sp.sched_priority = sched_get_priority_max(SCHED_FIFO);
    if (pthread_setschedparam(pthread_self(), SCHED_FIFO, &sp) != 0) {
        perror("pthread_setschedparam(SCHED_FIFO) failed -- run as root for real-time priority");
    } else {
        printf("supervisor: running SCHED_FIFO priority %d\n", sp.sched_priority);
    }
}

// ---- Obstacle / override logic --------------------------------------------

void checkObstacle(const UltrasonicMsg& m) {
    if (m.status != SensorStatus::Connected) {
        return; // no trustworthy distance to react to
    }

    if (m.distanceCm < OBSTACLE_THRESHOLD_CM) {
        // Override latency: elapsed time from when ultrasonic_task
        // captured the reading to the moment this decision is made --
        // the concrete number for the "Override Latency" requirement.
        const double latencyMs = (monotonicNs() - m.timestampNs) / 1e6;
        logEvent(SafetyEventType::Obstacle, m.distanceCm);
        if (!g_overrideActive) {
            g_overrideActive = true;
            logEvent(SafetyEventType::OverrideEngaged, static_cast<float>(latencyMs));
            printf("[OVERRIDE] obstacle at %.1fcm, latency %.3fms -- navigation "
                   "commands would be suppressed here\n",
                   m.distanceCm, latencyMs);
        }
    } else if (g_overrideActive) {
        g_overrideActive = false;
        logEvent(SafetyEventType::OverrideCleared, m.distanceCm);
        printf("[OVERRIDE] cleared -- path clear at %.1fcm\n", m.distanceCm);
    }
}

// ---- Message handlers -------------------------------------------------

void onUltrasonicReading(const UltrasonicMsg& m) {
    g_ultrasonic.lastMsgNs = monotonicNs();
    if (m.status == SensorStatus::Connected) {
        g_lastDistanceCm = m.distanceCm;
    } else {
        logEvent(SafetyEventType::SensorTimeout, m.distanceCm);
    }
    checkObstacle(m);
}

void onImuReading(const ImuMsg& m) {
    g_imu.lastMsgNs = monotonicNs();
    if (m.valid) {
        for (int i = 0; i < 3; ++i) g_lastAccelG[i] = m.accelG[i];
    } else {
        logEvent(SafetyEventType::SensorTimeout, 0.0f);
    }
}

CliStatusReply buildStatusReply() {
    CliStatusReply r{};
    r.state = g_state;
    r.overrideActive = g_overrideActive ? 1 : 0;

    r.ultrasonicHealth = g_ultrasonic.processAlive ? g_ultrasonic.dataHealth : SensorHealth::Dead;
    r.lastDistanceCm   = g_lastDistanceCm;
    r.ultrasonicAgeMs  = g_ultrasonic.lastMsgNs ? (monotonicNs() - g_ultrasonic.lastMsgNs) / 1000000 : 0;

    r.imuHealth = g_imu.processAlive ? g_imu.dataHealth : SensorHealth::Dead;
    for (int i = 0; i < 3; ++i) r.lastAccelG[i] = g_lastAccelG[i];
    r.imuAgeMs = g_imu.lastMsgNs ? (monotonicNs() - g_imu.lastMsgNs) / 1000000 : 0;

    r.eventCount = g_eventCount;
    const unsigned start = (g_eventCount < MAX_EVENT_LOG) ? 0 : g_eventHead;
    for (unsigned i = 0; i < g_eventCount; ++i) {
        r.events[i] = g_events[(start + i) % MAX_EVENT_LOG];
    }
    return r;
}

// ---- Watchdog / safety deadline / recovery --------------------------------

void onHeartbeat(SubsystemState& s, uint64_t now, const char* name) {
    s.lastHeartbeatNs = now;
    if (!s.processAlive) {
        s.processAlive = true;
        s.recoveryStartedNs = 0;
        s.respawnAttempts = 0;
        logEvent(SafetyEventType::ProcessRecovered, 0.0f);
        printf("[WATCHDOG] %s heartbeat (re)established\n", name);
    }
}

void checkWatchdog(SubsystemState& s, uint64_t now, const char* name) {
    if (s.lastHeartbeatNs == 0) return; // never connected yet -- still starting up
    const uint64_t age = now - s.lastHeartbeatNs;
    if (age > HEARTBEAT_TIMEOUT_NS && s.processAlive) {
        s.processAlive = false;
        s.recoveryStartedNs = now;
        logEvent(SafetyEventType::ProcessDead, static_cast<float>(age / 1e6));
        printf("[WATCHDOG] %s missed heartbeat for %.0fms -- declaring dead, "
               "starting recovery timer\n", name, age / 1e6);
    }
}

void checkDeadline(SubsystemState& s, uint64_t now, uint64_t deadlineNs) {
    if (s.lastMsgNs == 0) {
        s.dataHealth = SensorHealth::Dead;
        return;
    }
    const uint64_t age = now - s.lastMsgNs;
    s.dataHealth = (age > deadlineNs) ? SensorHealth::Degraded : SensorHealth::Healthy;
}

// Reports a subsystem the watchdog declared dead, once it has been given
// RECOVERY_GRACE_NS to reconnect on its own. Bounded by MAX_RESPAWN_ATTEMPTS
// so a permanently broken sensor doesn't spam this forever -- past that
// limit this stays quiet in SAFE_STOP for a human to intervene. Does not
// respawn anything automatically -- see the comment on RECOVERY_GRACE_NS
// for why that was removed.
void tryRecover(SubsystemState& s, const char* name, uint64_t now) {
    if (s.processAlive || s.recoveryStartedNs == 0) return;
    if (now - s.recoveryStartedNs < RECOVERY_GRACE_NS) return;
    if (s.respawnAttempts >= MAX_RESPAWN_ATTEMPTS) return;

    s.respawnAttempts++;
    s.recoveryStartedNs = now;
    printf("[RECOVERY] %s still down after %llums -- attempt %u/%u -- "
           "restart it manually (automatic respawn is disabled)\n",
           name, static_cast<unsigned long long>(RECOVERY_GRACE_NS / 1000000),
           s.respawnAttempts, MAX_RESPAWN_ATTEMPTS);
}

void recomputeSystemState() {
    SystemState next;
    if (!g_ultrasonic.processAlive || !g_imu.processAlive) {
        next = SystemState::SafeStop;
    } else if (g_ultrasonic.dataHealth != SensorHealth::Healthy ||
               g_imu.dataHealth != SensorHealth::Healthy) {
        next = SystemState::Degraded;
    } else {
        next = SystemState::Normal;
    }
    if (next != g_state) {
        printf("[STATE] %s -> %s\n", stateName(g_state), stateName(next));
        g_state = next;
    }
}

void onSafetyTick(uint64_t now) {
    checkWatchdog(g_ultrasonic, now, "ultrasonic_task");
    checkWatchdog(g_imu, now, "imu_task");
    checkDeadline(g_ultrasonic, now, ULTRASONIC_DEADLINE_NS);
    checkDeadline(g_imu, now, IMU_DEADLINE_NS);
    tryRecover(g_ultrasonic, "ultrasonic_task", now);
    tryRecover(g_imu, "imu_task", now);
    recomputeSystemState();
}

// ---- IPC plumbing -----------------------------------------------------

union RecvMsg {
    struct _pulse      pulse;
    MsgHeader          hdr;
    UltrasonicMsg      ultrasonic;
    ImuMsg             imu;
    CliStatusRequest   cliReq;
};

void handlePulse(const struct _pulse& pulse, uint64_t now) {
    switch (pulse.code) {
    case PULSE_HEARTBEAT_ULTRASONIC: onHeartbeat(g_ultrasonic, now, "ultrasonic_task"); break;
    case PULSE_HEARTBEAT_IMU:        onHeartbeat(g_imu, now, "imu_task"); break;
    case PULSE_SAFETY_TICK:          onSafetyTick(now); break;
    default: break;
    }
}

void handleMessage(int rcvid, const RecvMsg& msg) {
    switch (msg.hdr.type) {
    case MsgType::UltrasonicReading: {
        onUltrasonicReading(msg.ultrasonic);
        AckReply reply{1};
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        break;
    }
    case MsgType::ImuReading: {
        onImuReading(msg.imu);
        AckReply reply{1};
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        break;
    }
    case MsgType::CliStatusRequest: {
        const CliStatusReply reply = buildStatusReply();
        MsgReply(rcvid, EOK, &reply, sizeof(reply));
        break;
    }
    }
}

// Arms a periodic CLOCK_MONOTONIC timer that delivers PULSE_SAFETY_TICK
// on this channel -- the standard QNX self-timer pattern: connect back to
// your own channel, then give that connection to timer_create() as the
// pulse's destination.
void startSafetyTickTimer(int chid) {
    const int coid = ConnectAttach(0, 0, chid, 0, 0);
    if (coid == -1) {
        perror("ConnectAttach(self) failed");
        exit(EXIT_FAILURE);
    }

    struct sigevent event;
    SIGEV_PULSE_INIT(&event, coid, SIGEV_PULSE_PRIO_INHERIT, PULSE_SAFETY_TICK, 0);

    timer_t timerId;
    if (timer_create(CLOCK_MONOTONIC, &event, &timerId) == -1) {
        perror("timer_create failed");
        exit(EXIT_FAILURE);
    }

    struct itimerspec its;
    its.it_value.tv_sec  = 0;
    its.it_value.tv_nsec = static_cast<long>(SAFETY_TICK_PERIOD_MS) * 1000000L;
    its.it_interval = its.it_value;
    if (timer_settime(timerId, 0, &its, nullptr) == -1) {
        perror("timer_settime failed");
        exit(EXIT_FAILURE);
    }
}

} // namespace

int main() {
    raisePriority();

    // Registers this channel under SUPERVISOR_NAME in QNX's name-locator
    // service so ultrasonic_task, imu_task, and cli_status can each find
    // it with name_open() -- none of them need to know this process's pid.
    name_attach_t* attach = name_attach(nullptr, SUPERVISOR_NAME, 0);
    if (!attach) {
        perror("name_attach failed");
        return EXIT_FAILURE;
    }
    const int chid = attach->chid;
    printf("safety_supervisor: listening as \"%s\" (chid=%d)\n", SUPERVISOR_NAME, chid);

    startSafetyTickTimer(chid);

    for (;;) {
        RecvMsg msg;
        const int rcvid = MsgReceive(chid, &msg, sizeof(msg), nullptr);
        if (rcvid == 0) {
            handlePulse(msg.pulse, monotonicNs());
        } else if (rcvid > 0) {
            handleMessage(rcvid, msg);
        }
        // rcvid < 0: interrupted or transient error -- just retry.
    }
}
