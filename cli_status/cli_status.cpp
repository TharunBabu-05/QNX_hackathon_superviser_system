// cli_status -- read-only observer of the safety_supervisor. Connects
// over the same named channel the sensor tasks use, but only ever sends
// a CliStatusRequest: it has no way to influence the control path, by
// design, so it can be run freely for monitoring without becoming a
// safety risk itself.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "protocol.h"

namespace {
const char* healthName(SensorHealth h) {
    switch (h) {
    case SensorHealth::Healthy:  return "HEALTHY";
    case SensorHealth::Degraded: return "DEGRADED";
    case SensorHealth::Dead:     return "DEAD";
    }
    return "?";
}

const char* stateName(SystemState s) {
    switch (s) {
    case SystemState::Normal:   return "NORMAL";
    case SystemState::Degraded: return "DEGRADED";
    case SystemState::SafeStop: return "SAFE_STOP";
    case SystemState::Recovery: return "RECOVERY";
    }
    return "?";
}

const char* eventName(SafetyEventType t) {
    switch (t) {
    case SafetyEventType::Obstacle:         return "OBSTACLE";
    case SafetyEventType::SensorTimeout:    return "SENSOR_TIMEOUT";
    case SafetyEventType::ProcessDead:      return "PROCESS_DEAD";
    case SafetyEventType::ProcessRecovered: return "PROCESS_RECOVERED";
    case SafetyEventType::OverrideEngaged:  return "OVERRIDE_ENGAGED";
    case SafetyEventType::OverrideCleared:  return "OVERRIDE_CLEARED";
    }
    return "?";
}

void printStatus(const CliStatusReply& r) {
    printf("=== Safety Supervisor Status ===\n");
    printf("System State : %s\n", stateName(r.state));
    printf("Override     : %s\n", r.overrideActive ? "ACTIVE" : "inactive");
    printf("Ultrasonic-1 : %-8s | last=%.1fcm, age=%llums\n",
           healthName(r.ultrasonicHealth[ULTRASONIC_ID_1]), r.lastDistanceCm[ULTRASONIC_ID_1],
           static_cast<unsigned long long>(r.ultrasonicAgeMs[ULTRASONIC_ID_1]));
    printf("Ultrasonic-2 : %-8s | last=%.1fcm, age=%llums\n",
           healthName(r.ultrasonicHealth[ULTRASONIC_ID_2]), r.lastDistanceCm[ULTRASONIC_ID_2],
           static_cast<unsigned long long>(r.ultrasonicAgeMs[ULTRASONIC_ID_2]));
    printf("IMU          : %-8s | accel=(%.2f,%.2f,%.2f)g, age=%llums\n",
           healthName(r.imuHealth), r.lastAccelG[0], r.lastAccelG[1], r.lastAccelG[2],
           static_cast<unsigned long long>(r.imuAgeMs));
    printf("Recent Safety Events (%u):\n", r.eventCount);
    for (unsigned i = 0; i < r.eventCount; ++i) {
        const SafetyEvent& e = r.events[i];
        printf("  [t=%llums] %-18s value=%.2f\n",
               static_cast<unsigned long long>(e.timestampNs / 1000000),
               eventName(e.type), e.value);
    }
    printf("\n");
}

bool queryOnce() {
    const int coid = name_open(SUPERVISOR_NAME, 0);
    if (coid == -1) {
        perror("name_open(safety_supervisor) failed -- is the supervisor running?");
        return false;
    }

    CliStatusRequest req{};
    req.hdr.type = MsgType::CliStatusRequest;
    CliStatusReply reply{};

    const bool ok = MsgSend(coid, &req, sizeof(req), &reply, sizeof(reply)) != -1;
    if (!ok) {
        perror("MsgSend(status request) failed");
    } else {
        printStatus(reply);
    }
    name_close(coid);
    return ok;
}
}

int main(int argc, char** argv) {
    const bool watch = (argc > 1 && strcmp(argv[1], "--watch") == 0);
    if (!watch) {
        return queryOnce() ? EXIT_SUCCESS : EXIT_FAILURE;
    }
    for (;;) {
        queryOnce();
        sleep(1);
    }
}
