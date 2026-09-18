<div align="center">

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=6,11,20&height=200&section=header&text=Autonomous%20Vehicle%20Safety%20Supervisor&fontSize=36&fontColor=ffffff&animation=fadeIn&fontAlignY=35&desc=QNX%20Microkernel%20%C2%B7%20Multi-Process%20Safety%20Architecture%20%C2%B7%20Raspberry%20Pi%204B&descAlignY=55&descSize=17" width="100%"/>

<img src="https://readme-typing-svg.demolab.com/?font=Fira+Code&weight=600&size=20&duration=2800&pause=900&color=A855F7&center=true&vCenter=true&width=820&lines=5+independent+QNX+processes+%E2%80%94+not+5+threads;Native+message+passing+%2B+pulses%2C+not+sockets;SCHED_FIFO+priority+override+%E2%80%94+the+supervisor+always+wins;Heartbeat+watchdog+%2B+safety+deadlines+%2B+recovery+timer;2%C3%97+HC-SR04+%2B+MPU6500+IMU%2C+fused+into+one+decision" alt="Typing SVG"/>

<br/>

![QNX](https://img.shields.io/badge/RTOS-QNX%208.0-8A2BE2?style=for-the-badge)
![C++](https://img.shields.io/badge/C%2B%2B-17-00599C?style=for-the-badge&logo=cplusplus&logoColor=white)
![Raspberry Pi](https://img.shields.io/badge/Target-Raspberry%20Pi%204B-C51A4A?style=for-the-badge&logo=raspberrypi&logoColor=white)
![Architecture](https://img.shields.io/badge/Architecture-Microkernel-FF6B00?style=for-the-badge)
![IPC](https://img.shields.io/badge/IPC-Message%20Passing%20%2B%20Pulses-10B981?style=for-the-badge)
![Scheduling](https://img.shields.io/badge/Scheduling-SCHED__FIFO-EF4444?style=for-the-badge)
![Status](https://img.shields.io/badge/Status-Live%20on%20Target-brightgreen?style=for-the-badge)

</div>

<br/>

## What this actually is

A **safety supervisor for autonomous vehicles**, built as five independent QNX processes instead of one program with five functions. That's not a style choice — it's the entire point. On a monolithic OS, "isolating the safety logic from the sensor drivers" is a discipline you hope your team maintains. On QNX, it's **structural**: `imu_task` can segfault on a bad I²C read and `supervisor` will never know, because they don't share an address space. This repo is built to prove that difference, not just claim it.

<div align="center">

| | | |
|:---:|:---:|:---:|
| 🟣 **5 processes** | 🟠 **2× ultrasonic + 1× IMU** | 🔴 **1 supervisor at max RT priority** |
| Own address space each | Fused into one override decision | Watchdog + deadline + recovery for all three |

</div>

---

## 🏗️ Architecture

```mermaid
flowchart TB
    classDef hw fill:#F59E0B,stroke:#92400E,color:#111,stroke-width:2px
    classDef proc fill:#7C3AED,stroke:#4C1D95,color:#fff,stroke-width:2px
    classDef sup fill:#EF4444,stroke:#7F1D1D,color:#fff,stroke-width:3px
    classDef cli fill:#10B981,stroke:#065F46,color:#fff,stroke-width:2px

    HC1["HC-SR04 #1<br/>TRIG=GPIO23 (pin16)<br/>ECHO=GPIO24 (pin18)"]:::hw
    HC2["HC-SR04 #2<br/>TRIG=GPIO25 (pin22)<br/>ECHO=GPIO8 (pin24)"]:::hw
    IMUHW["MPU6500<br/>SDA=GPIO2 (pin3)<br/>SCL=GPIO3 (pin5)"]:::hw

    U1["ultrasonic_task-1<br/><sub>mmap_device_memory GPIO</sub>"]:::proc
    U2["ultrasonic_task-2<br/><sub>mmap_device_memory GPIO</sub>"]:::proc
    IMU["imu_task<br/><sub>devctl DCMD_I2C_SENDRECV</sub>"]:::proc

    SUP{{"safety_supervisor<br/>SCHED_FIFO, max priority<br/>watchdog · deadline · override · recovery"}}:::sup
    CLI["cli_status<br/><sub>read-only observer</sub>"]:::cli

    HC1 --> U1
    HC2 --> U2
    IMUHW --> IMU

    U1 -->|"MsgSend UltrasonicMsg"| SUP
    U1 -.->|"MsgSendPulse heartbeat"| SUP
    U2 -->|"MsgSend UltrasonicMsg"| SUP
    U2 -.->|"MsgSendPulse heartbeat"| SUP
    IMU -->|"MsgSend ImuMsg"| SUP
    IMU -.->|"MsgSendPulse heartbeat"| SUP

    CLI -->|"MsgSend CliStatusRequest"| SUP
    SUP -->|"MsgReply CliStatusReply"| CLI
```

Solid arrows are **synchronous `MsgSend`/`MsgReply`** (sensor data, CLI queries). Dashed arrows are **async `MsgSendPulse`** heartbeats — cheap, non-blocking, and never delayed by a slow sample cycle because each runs on its own thread. Every arrow into `supervisor` terminates at the *same* channel; `name_attach()`/`name_open()` let all four clients find it by name, with zero hardcoded pids.

### Safety state machine

```mermaid
stateDiagram-v2
    direction LR
    [*] --> RECOVERY
    RECOVERY --> NORMAL: all 3 processes alive<br/>+ all data fresh
    NORMAL --> DEGRADED: a sensor's data<br/>goes stale (>300ms)
    DEGRADED --> NORMAL: data fresh again
    DEGRADED --> SAFE_STOP: a process misses<br/>~5 heartbeats
    NORMAL --> SAFE_STOP: a process misses<br/>~5 heartbeats
    SAFE_STOP --> RECOVERY: heartbeat<br/>re-established

    classDef normal fill:#10B981,color:#fff
    classDef degraded fill:#F59E0B,color:#111
    classDef safestop fill:#EF4444,color:#fff
    classDef recovery fill:#8A2BE2,color:#fff
    class NORMAL normal
    class DEGRADED degraded
    class SAFE_STOP safestop
    class RECOVERY recovery
```

---

## ✅ Requirement → implementation

| Requirement | Where it lives | How |
|---|---|---|
| 🔀 **Message Passing** | all 5 processes | Native `MsgSend`/`MsgReceive`/`MsgReply` over a `name_attach()`-registered channel — no sockets, no queues |
| 💓 **Heartbeat** | every sensor task | Dedicated thread, `MsgSendPulse` every 50ms, independent of the sampling loop |
| 🐕 **Watchdog** | `supervisor.cpp::checkWatchdog` | ~5 missed heartbeats → process declared dead, logged, state transitions |
| ⏱️ **Safety Deadline** | `supervisor.cpp::checkDeadline` | Data older than 300ms is untrusted even if the process is alive — a *different* failure mode than a dead process |
| ⏲️ **Recovery Timer** | `supervisor.cpp::tryRecover` | 2s grace period, bounded to 3 logged attempts, then quiet in `SAFE_STOP` for a human |
| 🛡️ **Fault Tolerance** | driver + supervisor | `SensorStatus{Connected,Disconnected,OutOfRange}` classification feeds directly into health/state logic |
| 🚨 **Priority Override** | `supervisor.cpp::raisePriority` | Runs `SCHED_FIFO` at the system's max priority — always preempts sensor tasks, by the scheduler, not by convention |
| ⚡ **Override Latency** | `supervisor.cpp::updateOverride` | Measured end-to-end: sensor's `CLOCK_MONOTONIC` capture → supervisor's decision, in ms |
| 📋 **Safety Events** | 16-entry ring buffer | `Obstacle`, `SensorTimeout`, `ProcessDead/Recovered`, `OverrideEngaged/Cleared` |
| 💚 **Health Status** | per-subsystem + overall | `SensorHealth{Healthy,Degraded,Dead}` × 3 subsystems, rolled up into one `SystemState` |
| 🖥️ **CLI (mandatory)** | `cli_status` | `./cli_status` for one snapshot, `./cli_status --watch` to poll live — read-only, cannot influence the control path |

### 🎯 The override rule, precisely

Two ultrasonic sensors means one nuance worth stating explicitly: **override engages the instant either sensor sees an obstacle, and only clears once both report clear.** A vehicle covering front and rear shouldn't ignore a rear hazard just because the front happens to be clear.

---

## 🔌 Hardware wiring

<div align="center">

| Sensor | Signal | GPIO | Physical pin |
|:---|:---:|:---:|:---:|
| **HC-SR04 #1** | TRIG (TX) | GPIO23 | 16 |
| | ECHO (RX) | GPIO24 | 18 |
| **HC-SR04 #2** | TRIG (TX) | GPIO25 | 22 |
| | ECHO (RX) | GPIO8 † | 24 |
| **MPU6500** | SDA | GPIO2 | 3 |
| | SCL | GPIO3 | 5 |
| | INT ‡ | GPIO17 | 11 |

</div>

† GPIO8 is SPI0 CE0 under its ALT function — fine as a plain GPIO as long as SPI0 isn't enabled elsewhere on the board.
‡ INT is wired but **not yet used** — see [Known limitations](#-known-limitations-read-this).

---

## 📁 Repo layout

```
hello/
├── common/
│   └── protocol.h          # shared IPC wire format — the only thing every process depends on
├── ultrasonic_task-1/      # HC-SR04 #1: gpio.{h,cpp}, ultrasonic.{h,cpp}, ultrasonic_task-1.cpp
├── ultrasonic_task-2/      # HC-SR04 #2: same driver, different pins + sensor id
├── imu_task/               # MPU6500 over /dev/i2c1: mpu6500.{h,cpp}, imu_task.cpp
├── supervisor/             # the safety state machine — supervisor.cpp
└── cli_status/             # read-only status client — cli_status.cpp
```

Each subfolder is its own QNX recursive-make project (own `Makefile`/`common.mk`/`nto/`), building independent `aarch64le` + `x86_64` executables — five binaries, five processes, matching the diagram above exactly.

---

## 🛠️ Build

```bash
# from inside each of the 5 project folders:
make
```

Produces `nto/aarch64/o-le/<name>` (Pi target) and `nto/x86_64/o/<name>` (host, for sanity-checking the build).

## 🚀 Deploy & run

```bash
# 1. copy all five binaries to the Pi
scp -o MACs=hmac-sha2-256 \
    ultrasonic_task-1/nto/aarch64/o-le/ultrasonic_task-1 \
    ultrasonic_task-2/nto/aarch64/o-le/ultrasonic_task-2 \
    imu_task/nto/aarch64/o-le/imu_task \
    supervisor/nto/aarch64/o-le/supervisor \
    cli_status/nto/aarch64/o-le/cli_status \
    qnxuser@<PI_IP>:/tmp/

# 2. on the Pi
cd /tmp && chmod +x ultrasonic_task-1 ultrasonic_task-2 imu_task supervisor cli_status
```

Five terminals, **foreground**, in this order (GPIO access needs root; I²C doesn't):

| # | Command | Needs `sudo`? |
|:-:|---|:-:|
| A | `./supervisor` | no |
| B | `./imu_task` | no |
| C | `sudo ./ultrasonic_task-1` | **yes** |
| D | `sudo ./ultrasonic_task-2` | **yes** |
| E | `./cli_status --watch` | no |

### Sample output

```
=== Safety Supervisor Status ===
System State : NORMAL
Override     : inactive
Ultrasonic-1 : HEALTHY  | last=134.2cm, age=42ms
Ultrasonic-2 : HEALTHY  | last=87.6cm, age=38ms
IMU          : HEALTHY  | accel=(0.01,-0.02,0.99)g, age=21ms
Recent Safety Events (2):
  [t=104213ms] OBSTACLE           value=14.30
  [t=104213ms] OVERRIDE_ENGAGED   value=1.42
```

---

## ⚠️ Known limitations (read this)

Honesty over hackathon theater:

- **IMU sampling is polled (20Hz), not interrupt-driven.** The INT pin needs an exact GPIO→IRQ vector mapping specific to this BSP that wasn't available — polling is correct, just not the lowest-latency option.
- **Automatic process respawn was removed.** It briefly called `posix_spawn()` from `supervisor`'s max-priority `SCHED_FIFO` thread, which wedged the whole process (confirmed via `pidin -p` showing it blocked in `REPLY` state against `procnto`, unkillable even by `SIGKILL`). Recovery is now detected, timed, and logged — just not auto-executed. Restart the dead process manually when `[RECOVERY]` shows up.
- **GPIO access requires root** (`ThreadCtl(_NTO_TCTL_IO)`); I²C only requires group membership on `/dev/i2c1`. That's why the run table above has two different privilege levels.

<div align="center">

<sub>Built for a QNX hackathon around one idea: a microkernel's safety story isn't "we wrote careful code," it's "the OS itself won't let one process's bug become another's."</sub>

<img src="https://capsule-render.vercel.app/api?type=waving&color=gradient&customColorList=6,11,20&height=100&section=footer" width="100%"/>

</div>
