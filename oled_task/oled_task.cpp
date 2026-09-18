// oled_task -- read-only status display, same role as cli_status but for
// an SSD1306 panel instead of a terminal. It only ever sends
// CliStatusRequest, same as cli_status, so it has no way to influence the
// control path; it exists purely to make the vehicle's current safety
// state visible without needing an SSH session open.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "protocol.h"
#include "ssd1306.h"

namespace {
constexpr unsigned POLL_PERIOD_MS = 250;

const char* sourceName(uint8_t source) {
    switch (source) {
    case ULTRASONIC_ID_1: return "US1";
    case ULTRASONIC_ID_2: return "US2";
    case SOURCE_IMU:      return "IMU";
    default:              return "SYS";
    }
}

// Scans the event log backward for the most recent reading-related fault
// (Obstacle or AnomalyDetected) so the screen can say *which* of the two
// it was and *which* sensor saw it, instead of a generic "stopped"
// message that doesn't tell a bystander what actually happened.
bool findLatestHazardEvent(const CliStatusReply& r, SafetyEvent& out) {
    for (unsigned i = r.eventCount; i-- > 0;) {
        const SafetyEvent& e = r.events[i];
        if (e.type == SafetyEventType::Obstacle || e.type == SafetyEventType::AnomalyDetected) {
            out = e;
            return true;
        }
    }
    return false;
}

void render(Ssd1306& oled, const CliStatusReply& r) {
    const bool stopped = r.overrideActive || r.state == SystemState::SafeStop;
    char line0[22], line1[22], line2[22], line3[22];

    if (!stopped) {
        snprintf(line0, sizeof(line0), "VEHICLE MOVING");
        snprintf(line1, sizeof(line1), "US1:%3.0fCM US2:%3.0fCM",
                  r.lastDistanceCm[ULTRASONIC_ID_1], r.lastDistanceCm[ULTRASONIC_ID_2]);
        snprintf(line2, sizeof(line2), "STATE: %s",
                  r.state == SystemState::Normal ? "NORMAL" : "DEGRADED");
    } else {
        SafetyEvent hazard{};
        const bool found = findLatestHazardEvent(r, hazard);
        const bool isAnomaly = found && hazard.type == SafetyEventType::AnomalyDetected;

        snprintf(line0, sizeof(line0), "VEHICLE STOPPED");
        snprintf(line1, sizeof(line1), "%s", isAnomaly ? "ANOMALY DETECTED" : "OBSTACLE DETECTED");
        if (found) {
            snprintf(line2, sizeof(line2), "SRC:%s VAL:%5.1f",
                      sourceName(hazard.source), hazard.value);
        } else {
            snprintf(line2, sizeof(line2), "US1:%3.0fCM US2:%3.0fCM",
                      r.lastDistanceCm[ULTRASONIC_ID_1], r.lastDistanceCm[ULTRASONIC_ID_2]);
        }
    }
    snprintf(line3, sizeof(line3), "IMU %4.2f %4.2f %4.2f",
              r.lastAccelG[0], r.lastAccelG[1], r.lastAccelG[2]);

    oled.drawText(0, 0, line0);
    oled.drawText(2, 0, line1);
    oled.drawText(4, 0, line2);
    oled.drawText(6, 0, line3);
}

bool queryStatus(int coid, CliStatusReply& out) {
    CliStatusRequest req{};
    req.hdr.type = MsgType::CliStatusRequest;
    return MsgSend(coid, &req, sizeof(req), &out, sizeof(out)) != -1;
}
}

int main(int argc, char** argv) {
    // GPIO0/1 is a different physical I2C controller than the one
    // imu_task uses -- which /dev/i2cN node it shows up as depends on
    // this board's QNX startup config, not something guessable from
    // here. Defaults to /dev/i2c0 (the conventional node for that bus);
    // override with `oled_task /dev/i2cN` if that's wrong on this Pi.
    const char* i2cPath = (argc > 1) ? argv[1] : "/dev/i2c0";

    Ssd1306 oled;
    printf("Checking SSD1306 OLED connection (SDA=GPIO0, SCL=GPIO1, %s)...\n", i2cPath);
    if (!oled.connect(i2cPath)) {
        fprintf(stderr,
                "OLED not detected on %s -- check wiring/power, or pass the "
                "correct device path as an argument (see `ls /dev/i2c*` on the Pi)\n",
                i2cPath);
        return EXIT_FAILURE;
    }
    printf("OLED connected.\n");

    printf("Connecting to %s...\n", SUPERVISOR_NAME);
    int coid = -1;
    while ((coid = name_open(SUPERVISOR_NAME, 0)) == -1) {
        perror("name_open(safety_supervisor) failed, retrying");
        sleep(1);
    }
    printf("connected to supervisor. Updating display.\n");

    for (;;) {
        CliStatusReply reply{};
        if (queryStatus(coid, reply)) {
            render(oled, reply);
        }
        usleep(POLL_PERIOD_MS * 1000);
    }
}
