# Phase 4 independent ROS runtime validation report

Date: 2026-07-24  
Scope: propeller-off, no MAVROS, no PX4, no offboard  
Result: **BLOCKED BEFORE ROS STARTUP — NOT ACCEPTED**

## Safety boundary

- No MAVROS, PX4, offboard, rosbag, Livox driver, or FAST-LIO process was found
  during the explicit preflight process audit.
- No ROS node, Livox driver, FAST-LIO node, recorder, MAVROS, PX4, offboard
  publisher, flight controller connection, arming command, or motor command was
  started during this validation attempt.
- `EKF2_EV_CTRL` was not queried or modified because PX4 was not connected. The
  required value remains an external safety precondition of `11`.
- FAST-LIO source, tests, launch files, configuration, and build products were not
  modified after the Phase 3.5 freeze.

## Phase 3.5 freeze

Checkpoint:
`/home/robot/ws_offboard_control/phase_checkpoints/phase35_signoff_20260724_152912`

The checkpoint contains the design, source and direct dependencies, Phase
1/2/3/3.5 tests, five build products, and the last CTest log.

- `CHECKPOINT.sha256`: PASS
- `MANIFEST.sha256`: PASS for every frozen file
- Live source versus frozen source: PASS for all 18 checked files

Evidence:

- `evidence/phase35_checkpoint_selfcheck.txt`
- `evidence/live_source_vs_phase35_checkpoint.txt`

## Pre-runtime prerequisite audit

### Forbidden processes

PASS. No MAVROS, PX4, offboard, rosbag record/play, old `fastlio_mapping`, or
Livox driver process was present.

Evidence: `evidence/forbidden_process_matches.txt`

### LiDAR network

FAIL/BLOCKING.

- `enp86s0`: `carrier=0`, `operstate=down`
- `enx207bd51a27d1`: `carrier=0`, `operstate=down`
- Neither wired interface owns the Livox configuration host address
  `192.168.1.5/24`.
- Configured MID360 candidates `192.168.1.128` and `192.168.1.12` both returned
  100% packet loss.

Consequently, no real LiDAR or IMU input can be acquired and no valid
`/Odometry` baseline can be produced.

Evidence:

- `evidence/network_carrier.txt`
- `evidence/network_addresses.txt`
- `evidence/network_routes.txt`
- `evidence/ping_mid360_192.168.1.128.txt`
- `evidence/ping_mid360_192.168.1.12.txt`

### Required resource tool

FAIL/BLOCKING.

The mandatory `pidstat -t` collector is unavailable and the `sysstat` package is
not installed. No package installation was attempted because this run does not
authorize unrelated host changes or relaxed evidence collection.

Evidence: `evidence/tool_preflight.txt`

## Stop decision

The instruction for this run requires failures to be recorded and the validation
to stop without modifying FAST-LIO. Starting the stack with no sensor carrier
would only produce an invalid empty-data run, and substituting another CPU tool
would violate the explicit `pidstat -t` requirement. The run therefore stopped
before section A.

## Required acceptance stages

### A. Default-off baseline

NOT RUN.

The absence of `/Odometry/propagated` and `/high_rate_ev/diagnostics`, legacy
`/Odometry` rate/latency, CPU, RSS, and thread behavior were not measured.

### B. Independent enable and hand-held motion

NOT RUN.

No static, six-direction, yaw, or stop-to-zero measurements exist.

### C. Fault and recovery

NOT RUN.

No IMU timeout, LiDAR timeout, recovery wait, first recovered sequence, or
integer-nanosecond timestamp evidence exists.

### Repeated startup/shutdown

NOT RUN.

The required three startup/shutdown cycles were not attempted.

## Formal thresholds retained

No threshold was relaxed. The following formal gates from
`HIGH_RATE_EV_ODOM_DESIGN.md` remain unevaluated:

- propagated output rate: 30–50 Hz;
- source-age p95 `<25 ms`, maximum `<50 ms`;
- zero duplicate/backward stamps and every output tied to a newly integrated IMU;
- static speed-norm p95 `<0.03 m/s`;
- per-axis velocity/position-difference correlation `>0.9`;
- best apparent lag `<40 ms`;
- worker budget `0.010 s`;
- finite, symmetric, PSD, dynamic covariance;
- no legacy `/Odometry` regression;
- zero propagated output during FAULT and clean post-recovery advancement.

## Rosbag

No rosbag was created because validation stopped before ROS startup.

Marker:
`/home/robot/ws_offboard_control/phase4_validation_20260724_153046/NO_ROSBAG_CREATED.txt`

## Findings

- Critical: 0
- High: 1 — no wired LiDAR carrier/address; runtime validation cannot begin.
- Medium: 1 — mandatory `pidstat -t` is unavailable.

These are validation-environment findings, not confirmed FAST-LIO code defects.

## Signoff

`PHASE4_RUNTIME=BLOCKED_NOT_ACCEPTED`

No runtime acceptance claim can be made from this attempt.
