QNX RTOS microkernel-architecture topics now in use
Compared to before, this now actually exercises the microkernel model, not just one process:

Address-space process separation — ultrasonic_task, imu_task, supervisor, cli_status are four independent binaries. A hang or bad pointer in imu_task's I2C code cannot corrupt supervisor's state or take it down — they don't share memory. This is the concrete thing that a monolithic "four threads in one binary" version cannot claim.
Native QNX message passing — MsgSend()/MsgReceive()/MsgReply() for all sensor data and CLI queries. Synchronous, typed, kernel-arbitrated — not sockets or a queue library.
Pulses (MsgSendPulse(), SIGEV_PULSE_INIT) for heartbeats and the supervisor's own periodic timer — the lightweight, non-blocking QNX primitive, distinct from the synchronous messages carrying actual sensor data.
Name-service rendezvous (name_attach()/name_open()) — sensor tasks and the CLI find the supervisor by name; nobody hardcodes a pid.
Priority scheduling (SCHED_FIFO, pthread_setschedparam) — the supervisor runs at the highest real-time priority so its safety-tick and message dispatch always preempt the sensor tasks. This is "Priority Override" as an actual scheduler guarantee, not just a naming choice.
timer_create/SIGEV_PULSE — the supervisor's periodic watchdog/deadline check is driven by a CLOCK_MONOTONIC timer delivering a pulse to its own channel, the standard QNX self-timer idiom.
posix_spawn()-based recovery — bounded respawn of a subsystem the watchdog declares dead.
Everything from before (ThreadCtl, mmap_device_memory, ClockCycles, nanospin_ns, SYSPAGE_ENTRY) is still there inside ultrasonic_task.
Where each of your requested concepts lives
Requirement	Where
Message Passing	MsgSend/MsgReceive/MsgReply between all four processes
Heartbeat	MsgSendPulse every 50ms from each sensor task, on its own thread
Watchdog	checkWatchdog() in supervisor.cpp — declares a process dead after ~5 missed heartbeats
Safety Deadline	checkDeadline() — flags stale data (300ms) even if the process itself is alive
Recovery Timer	tryRecover() — 2s grace period, then bounded posix_spawn() retries (max 3)
Fault Tolerance	The Connected/Disconnected/OutOfRange classification (already built) feeding into watchdog + deadline + degrade logic
Priority Override	Supervisor's SCHED_FIFO priority + checkObstacle()'s override flag
Override Latency	Measured end-to-end from ultrasonic_task's capture timestamp to the supervisor's decision, logged in ms
Safety Events	16-entry ring buffer (g_events), exposed via CLI
Health Status	Per-subsystem SensorHealth + overall SystemState, exposed via CLI
CLI Mandatory	cli_status — ./cli_status for one snapshot, ./cli_status --watch to poll every second
Honest gaps you should know about
INT (GPIO17) is wired but unused — true interrupt-driven IMU sampling needs the exact GPIO→IRQ vector number your specific QNX Pi4 BSP assigns, which I don't have and won't guess. imu_task polls at 20Hz instead, which is correct but not "fast context switching via a real ISR." If you find that vector number in your BSP docs, this is the one part worth upgrading.
posix_spawn() respawn paths (/tmp/qnx/ultrasonic_task, /tmp/qnx/imu_task in supervisor.cpp) are placeholders — update them to wherever you actually deploy the binaries on the Pi, or respawn will just log a failed posix_spawn.
I2C devctl struct usage in mpu6500.cpp I verified against the real header on this machine (hw/i2c.h) but haven't tested against real hardware — if the very first WHO_AM_I read fails, that's the first thing to check with a scope/logic analyzer.
MPU6500 address assumes AD0 tied low (0x68); flip to 0x69 in mpu6500.cpp if your board wires AD0 high.
To run it: deploy all four binaries to the Pi, start supervisor first (it owns the name registration), then ultrasonic_task and imu_task in either order, then query with cli_status.