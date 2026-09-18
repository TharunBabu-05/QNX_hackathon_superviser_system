#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "gpio.h"
#include "ultrasonic.h"
#include "protocol.h"

namespace {
// Physical pin 16 = GPIO23 = TRIG ("TX": this MCU pulses it to fire a ping).
// Physical pin 18 = GPIO24 = ECHO ("RX": this MCU reads the return pulse).
constexpr unsigned TRIG_PIN = 23;
constexpr unsigned ECHO_PIN = 24;

// HC-SR04 needs >=60ms of quiet between pings or the previous echo can
// still be arriving when the next trigger fires.
constexpr unsigned SAMPLE_PERIOD_MS = 100;

// Independent of the sample rate: the supervisor's watchdog expects this
// on a fixed schedule to prove this process is still alive at all.
constexpr unsigned HEARTBEAT_PERIOD_MS = 50;

// Connection id to the supervisor's channel, shared by the sampling loop
// and the heartbeat thread. Set once in main() before either uses it.
int g_supervisorCoid = -1;

bool connectToSupervisor() {
    // name_open() resolves SUPERVISOR_NAME through QNX's name-locator
    // service and returns something usable directly as a coid for
    // MsgSend()/MsgSendPulse() -- neither side needs to know the other's pid.
    g_supervisorCoid = name_open(SUPERVISOR_NAME, 0);
    return g_supervisorCoid != -1;
}

// Runs on its own thread so a slow MsgSend() of sensor data can never
// delay the heartbeat: the watchdog must see this on schedule regardless
// of what the sampling loop is doing. This is the "fast context switch"
// the microkernel scheduler is doing constantly between this thread, the
// sampling loop, and the supervisor's receive loop.
void* heartbeatThread(void*) {
    for (;;) {
        MsgSendPulse(g_supervisorCoid, -1, PULSE_HEARTBEAT_ULTRASONIC, 0);
        usleep(HEARTBEAT_PERIOD_MS * 1000);
    }
    return nullptr;
}

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
    printf("sensor connected.\n");

    printf("Connecting to %s...\n", SUPERVISOR_NAME);
    while (!connectToSupervisor()) {
        perror("name_open(safety_supervisor) failed, retrying");
        sleep(1);
    }
    printf("connected to supervisor. Streaming readings.\n");

    pthread_t hbThread;
    pthread_create(&hbThread, nullptr, heartbeatThread, nullptr);

    for (;;) {
        const Reading r = sensor.sense();

        UltrasonicMsg msg{};
        msg.hdr.type = MsgType::UltrasonicReading;
        msg.timestampNs = monotonicNs(); // start of the override-latency clock
        msg.status = r.status;
        msg.distanceCm = r.distanceCm;

        AckReply reply{};
        if (MsgSend(g_supervisorCoid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            perror("MsgSend to supervisor failed");
        }

        usleep(SAMPLE_PERIOD_MS * 1000);
    }
}
