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

// How many samples to average at startup to find "resting" bias, and how
// strongly to smooth every sample after that (lower = smoother, slower to
// react). See calibrate() for why this matters.
constexpr unsigned CALIBRATION_SAMPLES = 50; // ~2.5s at SAMPLE_PERIOD_MS
constexpr float    FILTER_ALPHA        = 0.2f;

// Connection id to the supervisor's channel, shared by the sampling loop
// and the heartbeat thread. Set once in main() before either uses it.
int g_supervisorCoid = -1;

struct Calibration {
    float accelBiasG[3]  = {0, 0, 0};
    float gyroBiasDps[3] = {0, 0, 0};
};

// Averages CALIBRATION_SAMPLES readings taken right after connect(), while
// the board is assumed stationary, and treats that average as "zero" for
// every reading after. Without this, a resting accelerometer reports
// whatever component of gravity (~1g total) its current tilt projects onto
// each axis -- rarely all zero on any axis -- which looks like unexplained
// noise to anything expecting (0,0,0) at rest. This isn't discarding real
// signal: it's removing gravity plus the sensor's fixed bias, so only
// actual motion/impact shows up as a deviation from zero afterward.
Calibration calibrate(Mpu6500& imu) {
    Calibration c;
    unsigned ok = 0;
    for (unsigned i = 0; i < CALIBRATION_SAMPLES; ++i) {
        Mpu6500::Sample s;
        if (imu.read(s)) {
            for (int a = 0; a < 3; ++a) {
                c.accelBiasG[a]  += s.accelG[a];
                c.gyroBiasDps[a] += s.gyroDps[a];
            }
            ok++;
        }
        usleep(SAMPLE_PERIOD_MS * 1000);
    }
    if (ok > 0) {
        for (int a = 0; a < 3; ++a) {
            c.accelBiasG[a]  /= ok;
            c.gyroBiasDps[a] /= ok;
        }
    }
    return c;
}

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

    printf("Calibrating IMU -- keep it still for a moment...\n");
    const Calibration cal = calibrate(imu);
    printf("Calibration done (resting accel bias=(%.3f,%.3f,%.3f)g, "
           "gyro bias=(%.2f,%.2f,%.2f)dps)\n",
           cal.accelBiasG[0], cal.accelBiasG[1], cal.accelBiasG[2],
           cal.gyroBiasDps[0], cal.gyroBiasDps[1], cal.gyroBiasDps[2]);

    printf("Connecting to %s...\n", SUPERVISOR_NAME);
    while (!connectToSupervisor()) {
        perror("name_open(safety_supervisor) failed, retrying");
        sleep(1);
    }
    printf("connected to supervisor. Streaming samples.\n");

    pthread_t hbThread;
    pthread_create(&hbThread, nullptr, heartbeatThread, nullptr);

    // Exponential moving average, applied after bias correction, to smooth
    // out sample-to-sample jitter without lagging behind a real, sustained
    // motion by more than a couple of sample periods.
    float filtAccel[3] = {0, 0, 0};
    float filtGyro[3]  = {0, 0, 0};
    bool  filterPrimed = false;

    for (;;) {
        Mpu6500::Sample raw{};
        const bool ok = imu.read(raw);

        ImuMsg msg{};
        msg.hdr.type = MsgType::ImuReading;
        msg.timestampNs = monotonicNs();
        msg.valid = ok ? 1 : 0;

        if (ok) {
            float accel[3], gyro[3];
            for (int i = 0; i < 3; ++i) {
                accel[i] = raw.accelG[i]  - cal.accelBiasG[i];
                gyro[i]  = raw.gyroDps[i] - cal.gyroBiasDps[i];
            }
            if (!filterPrimed) {
                // Seed with the first real sample so the filter doesn't
                // spend its first several samples climbing up from a fake
                // zero start.
                for (int i = 0; i < 3; ++i) { filtAccel[i] = accel[i]; filtGyro[i] = gyro[i]; }
                filterPrimed = true;
            } else {
                for (int i = 0; i < 3; ++i) {
                    filtAccel[i] = FILTER_ALPHA * accel[i] + (1.0f - FILTER_ALPHA) * filtAccel[i];
                    filtGyro[i]  = FILTER_ALPHA * gyro[i]  + (1.0f - FILTER_ALPHA) * filtGyro[i];
                }
            }
            for (int i = 0; i < 3; ++i) {
                msg.accelG[i]  = filtAccel[i];
                msg.gyroDps[i] = filtGyro[i];
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
