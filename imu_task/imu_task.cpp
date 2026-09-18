#include <cstdio>
#include <cstdlib>
#include <unistd.h>
#include <pthread.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "mpu6500.h"
#include "protocol.h"

namespace {
constexpr unsigned SAMPLE_PERIOD_MS    = 50; // 20Hz -- plenty for safety-level fault checks
constexpr unsigned HEARTBEAT_PERIOD_MS = 50;

// Connection id to the supervisor's channel, shared by the sampling loop
// and the heartbeat thread. Set once in main() before either uses it.
int g_supervisorCoid = -1;

bool connectToSupervisor() {
    // name_open() resolves SUPERVISOR_NAME through QNX's name-locator
    // service and returns something usable directly as a coid.
    g_supervisorCoid = name_open(SUPERVISOR_NAME, 0);
    return g_supervisorCoid != -1;
}

// Own thread so a slow/blocked I2C transaction can never delay the
// heartbeat the supervisor's watchdog is timing this process against.
void* heartbeatThread(void*) {
    for (;;) {
        MsgSendPulse(g_supervisorCoid, -1, PULSE_HEARTBEAT_IMU, 0);
        usleep(HEARTBEAT_PERIOD_MS * 1000);
    }
    return nullptr;
}
}

int main() {
    Mpu6500 imu;
    printf("Checking MPU6500 connection (SDA=GPIO2, SCL=GPIO3, /dev/i2c1)...\n");
    if (!imu.connect()) {
        fprintf(stderr, "IMU not detected -- check SDA/SCL wiring and power\n");
        return EXIT_FAILURE;
    }
    printf("IMU connected (WHO_AM_I confirmed MPU6500).\n");

    printf("Connecting to %s...\n", SUPERVISOR_NAME);
    while (!connectToSupervisor()) {
        perror("name_open(safety_supervisor) failed, retrying");
        sleep(1);
    }
    printf("connected to supervisor. Streaming samples.\n");

    pthread_t hbThread;
    pthread_create(&hbThread, nullptr, heartbeatThread, nullptr);

    for (;;) {
        Mpu6500::Sample sample{};
        const bool ok = imu.read(sample);

        ImuMsg msg{};
        msg.hdr.type = MsgType::ImuReading;
        msg.timestampNs = monotonicNs();
        msg.valid = ok ? 1 : 0;
        if (ok) {
            for (int i = 0; i < 3; ++i) {
                msg.accelG[i]  = sample.accelG[i];
                msg.gyroDps[i] = sample.gyroDps[i];
            }
        } else {
            fprintf(stderr, "IMU: read failed (bus error/NACK)\n");
        }

        AckReply reply{};
        if (MsgSend(g_supervisorCoid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            perror("MsgSend to supervisor failed");
        }

        usleep(SAMPLE_PERIOD_MS * 1000);
    }
}
