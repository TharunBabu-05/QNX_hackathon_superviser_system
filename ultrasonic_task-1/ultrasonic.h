#ifndef ULTRASONIC_H
#define ULTRASONIC_H

#include "gpio.h"
#include "protocol.h" // SensorStatus is shared with the supervisor's wire protocol

// HC-SR04 has no identification/handshake line, so "is it connected" can
// only be inferred from how it behaves when triggered:
//   Disconnected -- no echo pulse at all. TRIG never reached the sensor,
//                   ECHO never reached the GPIO, or the module has no power.
//   OutOfRange   -- an echo pulse did arrive, but its width falls outside
//                   the sensor's physical envelope (~2cm-400cm). That's not
//                   a real target; it's noise from a floating/miswired ECHO
//                   line, so it is reported as a fault, not a distance.
//   Connected    -- a plausible echo pulse arrived; distanceCm is valid.
struct Reading {
    SensorStatus status;
    float distanceCm; // meaningful only when status == Connected/OutOfRange
};

// Driver for an HC-SR04 ultrasonic distance sensor wired as:
//   TRIG ("TX" on the sensor, this MCU's output) -> GPIO23 / physical pin 16
//   ECHO ("RX" on the sensor, this MCU's input)  -> GPIO24 / physical pin 18
// ECHO is 5V logic; it must reach GPIO24 through the resistor divider
// (1k/2k) already wired, or it will damage the Pi's 3.3V-only input.
class Ultrasonic {
public:
    Ultrasonic(Gpio& gpio, unsigned trigPin, unsigned echoPin);

    // Triggers one ranging cycle and classifies the result -- callers
    // must check .status before trusting .distanceCm. Every wait is
    // bounded by echoTimeoutUs, so a disconnected/faulty sensor can
    // never hang the caller, which matters once this feeds a supervisor.
    Reading sense(unsigned echoTimeoutUs = 30000) const;

private:
    Gpio& gpio_;
    unsigned trigPin_;
    unsigned echoPin_;
};

#endif // ULTRASONIC_H
