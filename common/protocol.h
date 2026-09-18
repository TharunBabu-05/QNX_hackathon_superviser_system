#ifndef SAFETY_PROTOCOL_H
#define SAFETY_PROTOCOL_H

#include <cstdint>
#include <time.h>
#include <sys/neutrino.h>

// Name the safety_supervisor registers with name_attach() so unrelated
// processes can rendezvous with it by name (name_open()) instead of a
// hardcoded pid/coid -- QNX's lightweight alternative to writing a full
// resource manager when only a message-passing endpoint is needed.
#define SUPERVISOR_NAME "safety_supervisor"

// Pulse codes delivered on the supervisor's channel. Values below
// _PULSE_CODE_MINAVAIL are reserved by the kernel and other subsystems,
// so every application-defined pulse code must start from it.
//
// Both ultrasonic tasks share one heartbeat pulse code; MsgSendPulse()'s
// value parameter (landing in _pulse.value.sival_int on receipt) carries
// which ULTRASONIC_ID sent it, so the supervisor doesn't need a separate
// code per sensor.
enum {
    PULSE_HEARTBEAT_ULTRASONIC = _PULSE_CODE_MINAVAIL,
    PULSE_HEARTBEAT_IMU,
    PULSE_SAFETY_TICK, // supervisor's own periodic safety-check timer
};

// Identifies which physical ultrasonic sensor a message/heartbeat came
// from. ultrasonic_task-1 uses ULTRASONIC_ID_1, ultrasonic_task-2 uses
// ULTRASONIC_ID_2.
enum { ULTRASONIC_ID_1 = 0, ULTRASONIC_ID_2 = 1, ULTRASONIC_COUNT = 2 };

// Which subsystem a SafetyEvent is about. Ultrasonic sources reuse
// ULTRASONIC_ID_1/_2 so one field means the same thing whether the event
// came from a specific sensor or from a combined system-wide decision.
enum { SOURCE_IMU = 2, SOURCE_SYSTEM = 3 };

// HC-SR04 datasheet physical range. A real driver (see ultrasonic.cpp)
// never reports SensorStatus::Connected outside this window -- it reports
// OutOfRange instead. So the supervisor can treat "Connected" paired with
// a distance outside this range as proof the reading did not come from a
// real echo at all (a stuck/corrupted sensor, or injected fault data),
// not just an unusually near/far obstacle.
constexpr float ULTRASONIC_PHYSICAL_MIN_CM = 2.0f;
constexpr float ULTRASONIC_PHYSICAL_MAX_CM = 400.0f;

// Message types for synchronous MsgSend()/MsgReceive()/MsgReply() traffic.
// Every message starts with this header so the supervisor's receive loop
// can dispatch on hdr.type before interpreting the rest of the buffer.
enum class MsgType : uint16_t {
    UltrasonicReading,
    ImuReading,
    CliStatusRequest,
};

struct MsgHeader {
    MsgType type;
};

// Shared with ultrasonic_task's Ultrasonic::sense() so the supervisor can
// interpret UltrasonicMsg.status without depending on that driver's headers.
enum class SensorStatus : uint8_t { Connected, Disconnected, OutOfRange };

enum class SensorHealth    : uint8_t { Healthy, Degraded, Dead };
enum class SystemState     : uint8_t { Normal, Degraded, SafeStop, Recovery };
enum class SafetyEventType : uint8_t {
    Obstacle, SensorTimeout, ProcessDead, ProcessRecovered,
    OverrideEngaged, OverrideCleared,
    AnomalyDetected // a reading claimed valid but was physically impossible
};

// ultrasonic_task-1 / ultrasonic_task-2 -> supervisor
struct UltrasonicMsg {
    MsgHeader    hdr;
    uint64_t     timestampNs; // CLOCK_MONOTONIC, captured right before send:
                              // the start point for the override-latency measurement
    uint8_t      sensorId;    // ULTRASONIC_ID_1 or ULTRASONIC_ID_2
    SensorStatus status;
    float        distanceCm;
};

// imu_task -> supervisor
struct ImuMsg {
    MsgHeader hdr;
    uint64_t  timestampNs;
    uint8_t   valid;
    float     accelG[3];
    float     gyroDps[3];
};

// cli_status -> supervisor
struct CliStatusRequest {
    MsgHeader hdr;
};

// supervisor's reply to UltrasonicMsg/ImuMsg (the sender just needs to
// know the update was accepted).
struct AckReply {
    int32_t ok;
};

constexpr unsigned MAX_EVENT_LOG = 16;

struct SafetyEvent {
    uint64_t        timestampNs;
    SafetyEventType type;
    uint8_t         source; // ULTRASONIC_ID_1/_2, SOURCE_IMU, or SOURCE_SYSTEM
    float           value;  // context-dependent: distanceCm, latencyMs, ...
    // Snapshot of both ultrasonic sensors at the moment of this event, so
    // an obstacle/override entry shows full context, not just whichever
    // sensor triggered it.
    float           ultrasonic1Cm;
    float           ultrasonic2Cm;
};

// supervisor's reply to CliStatusRequest.
struct CliStatusReply {
    SystemState state;
    uint8_t     overrideActive;

    SensorHealth ultrasonicHealth[ULTRASONIC_COUNT];
    float        lastDistanceCm[ULTRASONIC_COUNT];
    uint64_t     ultrasonicAgeMs[ULTRASONIC_COUNT];

    SensorHealth imuHealth;
    float        lastAccelG[3];
    uint64_t     imuAgeMs;

    unsigned    eventCount;
    SafetyEvent events[MAX_EVENT_LOG];
};

inline uint64_t monotonicNs() {
    struct timespec ts;
    clock_gettime(CLOCK_MONOTONIC, &ts);
    return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
}

#endif // SAFETY_PROTOCOL_H
