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
enum {
    PULSE_HEARTBEAT_ULTRASONIC = _PULSE_CODE_MINAVAIL,
    PULSE_HEARTBEAT_IMU,
    PULSE_SAFETY_TICK, // supervisor's own periodic safety-check timer
};

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
    OverrideEngaged, OverrideCleared
};

// ultrasonic_task -> supervisor
struct UltrasonicMsg {
    MsgHeader    hdr;
    uint64_t     timestampNs; // CLOCK_MONOTONIC, captured right before send:
                              // the start point for the override-latency measurement
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
    float           value; // context-dependent: distanceCm, latencyMs, ...
};

// supervisor's reply to CliStatusRequest.
struct CliStatusReply {
    SystemState state;
    uint8_t     overrideActive;

    SensorHealth ultrasonicHealth;
    float        lastDistanceCm;
    uint64_t     ultrasonicAgeMs;

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
