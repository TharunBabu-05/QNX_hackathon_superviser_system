#include "ultrasonic.h"

#include <sys/neutrino.h>
#include <sys/syspage.h>

namespace {
// ClockCycles() returns raw CPU cycles; convert to microseconds using
// the board's calibrated cycle rate from the syspage (read once).
uint64_t cyclesPerUs() {
    static const uint64_t cps = SYSPAGE_ENTRY(qtime)->cycles_per_sec / 1000000ULL;
    return cps;
}

// HC-SR04 datasheet physical range: ~2cm to 400cm. A measurement outside
// this window did not come from a real echo off a real target.
constexpr float MIN_RANGE_CM = 2.0f;
constexpr float MAX_RANGE_CM = 400.0f;
}

Ultrasonic::Ultrasonic(Gpio& gpio, unsigned trigPin, unsigned echoPin)
    : gpio_(gpio), trigPin_(trigPin), echoPin_(echoPin) {
    gpio_.setMode(trigPin_, Gpio::Mode::Output);
    gpio_.setMode(echoPin_, Gpio::Mode::Input);
    gpio_.write(trigPin_, false);
}

Reading Ultrasonic::sense(unsigned echoTimeoutUs) const {
    const uint64_t cpu = cyclesPerUs();
    const uint64_t timeoutCycles = static_cast<uint64_t>(echoTimeoutUs) * cpu;

    // HC-SR04 datasheet: a >=10us high pulse on TRIG starts one ranging
    // cycle. nanospin_ns() busy-waits without yielding the CPU, which a
    // sleep/usleep call cannot guarantee at this granularity.
    gpio_.write(trigPin_, true);
    nanospin_ns(10000);
    gpio_.write(trigPin_, false);

    // No rising edge at all within the timeout means the sensor never
    // reacted to the trigger -- TRIG/ECHO wiring, power, or the module
    // itself is the problem, not "no target in range".
    const uint64_t waitStart = ClockCycles();
    while (!gpio_.read(echoPin_)) {
        if (ClockCycles() - waitStart > timeoutCycles) {
            return { SensorStatus::Disconnected, -1.0f };
        }
    }

    // Measure how long ECHO stays high: that duration is the round-trip
    // travel time of the pulse. A pulse that never ends is the same
    // "not really there" fault as never starting.
    const uint64_t pulseStart = ClockCycles();
    while (gpio_.read(echoPin_)) {
        if (ClockCycles() - pulseStart > timeoutCycles) {
            return { SensorStatus::Disconnected, -1.0f };
        }
    }
    const uint64_t pulseCycles = ClockCycles() - pulseStart;
    const float pulseUs = static_cast<float>(pulseCycles) / static_cast<float>(cpu);

    // distance = (time * speed_of_sound) / 2, halved for the round trip.
    // Speed of sound ~343 m/s == 0.0343 cm/us.
    const float cm = (pulseUs * 0.0343f) / 2.0f;

    if (cm < MIN_RANGE_CM || cm > MAX_RANGE_CM) {
        return { SensorStatus::OutOfRange, cm };
    }
    return { SensorStatus::Connected, cm };
}
