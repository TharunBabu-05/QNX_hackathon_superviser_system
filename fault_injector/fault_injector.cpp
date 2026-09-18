// fault_injector -- deliberately feeds the supervisor a physically
// impossible ultrasonic reading, to demonstrate the "Fault Tolerance"
// requirement: the supervisor must not just react to real obstacles, it
// must also recognize when a sensor itself cannot be trusted and force a
// safe state rather than act on bad data.
//
// It does this the same way a real stuck/corrupted HC-SR04 module would
// look from the supervisor's side: it sends UltrasonicMsg frames claiming
// SensorStatus::Connected while carrying a distance outside the sensor's
// physical range (see ULTRASONIC_PHYSICAL_MIN_CM/MAX_CM in protocol.h).
// It does NOT replace ultrasonic_task-1/-2 -- run this alongside the real
// one. Both send frames for the same sensorId to the same supervisor;
// whichever arrives last is what the supervisor currently believes. Kill
// this process (Ctrl+C) and the real task's next valid frame clears the
// fault on its own -- that's the "Recovery" half of the demo, with no
// extra recovery step needed.
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <unistd.h>
#include <sys/neutrino.h>
#include <sys/dispatch.h>

#include "protocol.h"

namespace {
constexpr unsigned INJECT_PERIOD_MS = 100; // matches a real ultrasonic_task's sample rate

void usage(const char* argv0) {
    fprintf(stderr,
        "Usage: %s <1|2> [distanceCm]\n"
        "  1|2         -- which ultrasonic sensor to fake (matches ULTRASONIC_ID_1/_2)\n"
        "  distanceCm  -- fake reading to send, must be outside the sensor's physical\n"
        "                 range (%.0f..%.0fcm) to register as an anomaly rather than a\n"
        "                 normal obstacle/out-of-range reading. Default: 5000 (a stuck-\n"
        "                 high phantom reading no real echo could produce).\n"
        "Sends the fake reading every %ums until you press Ctrl+C. Run the real\n"
        "ultrasonic_task for the same sensor in another terminal at the same time --\n"
        "this does not replace it, it competes with it, exactly like a real fault would.\n",
        argv0, ULTRASONIC_PHYSICAL_MIN_CM, ULTRASONIC_PHYSICAL_MAX_CM, INJECT_PERIOD_MS);
}
}

int main(int argc, char** argv) {
    if (argc < 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }

    const int idArg = atoi(argv[1]);
    if (idArg != 1 && idArg != 2) {
        usage(argv[0]);
        return EXIT_FAILURE;
    }
    const uint8_t sensorId = (idArg == 1) ? ULTRASONIC_ID_1 : ULTRASONIC_ID_2;

    float fakeDistanceCm = 5000.0f;
    if (argc >= 3) {
        fakeDistanceCm = static_cast<float>(atof(argv[2]));
    }
    if (fakeDistanceCm >= ULTRASONIC_PHYSICAL_MIN_CM && fakeDistanceCm <= ULTRASONIC_PHYSICAL_MAX_CM) {
        fprintf(stderr,
            "warning: %.1fcm is inside the sensor's real physical range -- the "
            "supervisor will treat it as a normal obstacle reading, not a fault.\n",
            fakeDistanceCm);
    }

    printf("Connecting to %s...\n", SUPERVISOR_NAME);
    int coid = -1;
    while ((coid = name_open(SUPERVISOR_NAME, 0)) == -1) {
        perror("name_open(safety_supervisor) failed, retrying");
        sleep(1);
    }

    printf("fault_injector: injecting FAKE anomalous reading into ultrasonic-%d\n"
           "  (distance=%.1fcm, outside the %.0f..%.0fcm physical range) every %ums.\n"
           "  This runs alongside the real ultrasonic_task-%d -- it does not stop it.\n"
           "  Press Ctrl+C to stop the fault; the real sensor's next good reading\n"
           "  clears it automatically.\n",
           idArg, fakeDistanceCm, ULTRASONIC_PHYSICAL_MIN_CM, ULTRASONIC_PHYSICAL_MAX_CM,
           INJECT_PERIOD_MS, idArg);

    for (;;) {
        UltrasonicMsg msg{};
        msg.hdr.type     = MsgType::UltrasonicReading;
        msg.timestampNs  = monotonicNs();
        msg.sensorId     = sensorId;
        msg.status       = SensorStatus::Connected; // claims valid -- that's the fault
        msg.distanceCm   = fakeDistanceCm;

        AckReply reply{};
        if (MsgSend(coid, &msg, sizeof(msg), &reply, sizeof(reply)) == -1) {
            perror("MsgSend to supervisor failed");
        }

        usleep(INJECT_PERIOD_MS * 1000);
    }
}
