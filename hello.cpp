#include <cstdio>
#include <cstdlib>
#include <unistd.h>

#include "gpio.h"
#include "ultrasonic.h"

namespace {
// Physical pin 16 = GPIO23 = TRIG ("TX": this MCU pulses it to fire a ping).
// Physical pin 18 = GPIO24 = ECHO ("RX": this MCU reads the return pulse).
constexpr unsigned TRIG_PIN = 23;
constexpr unsigned ECHO_PIN = 24;

// HC-SR04 needs >=60ms of quiet between pings or the previous echo can
// still be arriving when the next trigger fires.
constexpr unsigned SAMPLE_PERIOD_MS = 100;

// Confirms the sensor actually responds before anything downstream is
// allowed to trust its readings -- there is no point running a detection
// loop against a module that was never wired up correctly.
bool waitForSensor(const Ultrasonic& sensor, unsigned maxAttempts) {
    for (unsigned attempt = 1; attempt <= maxAttempts; ++attempt) {
        const Reading r = sensor.sense();
        if (r.status != SensorStatus::Disconnected) {
            return true;
        }
        printf("sensor check %u/%u: no response, retrying...\n", attempt, maxAttempts);
        usleep(SAMPLE_PERIOD_MS * 1000);
    }
    return false;
}
}

int main() {
    Gpio gpio;
    Ultrasonic sensor(gpio, TRIG_PIN, ECHO_PIN);

    printf("Checking HC-SR04 connection (TRIG=GPIO%u, ECHO=GPIO%u)...\n",
           TRIG_PIN, ECHO_PIN);
    if (!waitForSensor(sensor, 10)) {
        fprintf(stderr, "sensor not detected -- check TRIG/ECHO wiring and power\n");
        return EXIT_FAILURE;
    }
    printf("sensor connected. Starting distance monitoring.\n");

    for (;;) {
        const Reading r = sensor.sense();
        switch (r.status) {
        case SensorStatus::Connected:
            printf("sensor: OK   | distance: %.1f cm\n", r.distanceCm);
            break;
        case SensorStatus::OutOfRange:
            printf("sensor: WARN | reading outside physical range (%.1f cm) -- check wiring\n",
                   r.distanceCm);
            break;
        case SensorStatus::Disconnected:
            printf("sensor: FAIL | no echo response -- check connection\n");
            break;
        }
        usleep(SAMPLE_PERIOD_MS * 1000);
    }
}
