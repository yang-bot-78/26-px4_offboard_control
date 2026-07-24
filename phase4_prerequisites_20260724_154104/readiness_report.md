# Phase 4 prerequisite remediation

Date: 2026-07-24

## Host network configuration

- Intended MID360 interface: `enp86s0`
- NetworkManager connection: `有线连接 1`
- Persistent IPv4 method: manual
- Persistent address: `192.168.1.5/24`
- Default route: disabled for this connection
- IPv6: ignored
- Autoconnect: enabled
- NetworkManager connection activation: successful

The address is currently assigned, but the physical interface continues to report
`carrier=0`, `NO-CARRIER`, and `operstate=down`. MID360 address
`192.168.1.128` is unreachable and its neighbor entry is failed. Software
configuration is complete; physical power/cabling is not operational.

## pidstat

System-wide package installation was unavailable because sudo requires an
interactive password. The official Ubuntu `sysstat` package was therefore
downloaded and installed without privilege under:

`/home/robot/.local/opt/sysstat-12.5.2`

Executable:

`/home/robot/.local/bin/pidstat`

Validation:

- version: `12.5.2`
- `pidstat -t -p SELF 1 1`: PASS
- binary SHA-256:
  `bb48ce4dad4beaa9972eb78ba19baeb8338306aa31a1917032cab17f994c392b`

## Safety

No ROS, Livox, FAST-LIO, rosbag, MAVROS, PX4, or offboard process was started.
Phase 4 was not resumed.

## Readiness

`HOST_IP_CONFIGURED=YES`

`PIDSTAT_AVAILABLE=YES`

`MID360_PHYSICAL_CARRIER=NO`

`PHASE4_READY=NO`
