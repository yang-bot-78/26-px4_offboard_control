#!/usr/bin/env python3
"""离线分析无桨 EV 验证录包，全程不连接飞机。"""

from __future__ import annotations

import argparse
from collections import defaultdict
from datetime import datetime
import math
from pathlib import Path
import re
import statistics
from typing import Dict, Iterable, List, Optional, Sequence, Tuple

import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


Vector3 = Tuple[float, float, float]
TransformRow = Tuple[int, str, str, Vector3, Tuple[float, float, float, float]]


def stamp_seconds(message) -> Optional[float]:
    if not hasattr(message, "header"):
        return None
    stamp = message.header.stamp
    value = float(stamp.sec) + float(stamp.nanosec) * 1e-9
    return value if value > 0.0 else None


def position(message) -> Optional[Vector3]:
    if hasattr(message, "pose"):
        pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
        if hasattr(pose, "position"):
            return (pose.position.x, pose.position.y, pose.position.z)
    if all(hasattr(message, axis) for axis in ("x", "y", "z")):
        return (float(message.x), float(message.y), float(message.z))
    return None


def velocity(message) -> Optional[Vector3]:
    if hasattr(message, "twist"):
        twist = message.twist.twist if hasattr(message.twist, "twist") else message.twist
        if hasattr(twist, "linear"):
            return (twist.linear.x, twist.linear.y, twist.linear.z)
    if all(hasattr(message, axis) for axis in ("vx", "vy", "vz")):
        return (float(message.vx), float(message.vy), float(message.vz))
    return None


def orientation_yaw(message) -> Optional[float]:
    if not hasattr(message, "pose"):
        return None
    pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
    if not hasattr(pose, "orientation"):
        return None
    quaternion = pose.orientation
    values = (quaternion.x, quaternion.y, quaternion.z, quaternion.w)
    if not all(math.isfinite(value) for value in values):
        return None
    return quaternion_yaw(values)


def linear_velocity_covariance(message) -> Optional[Tuple[float, ...]]:
    if not hasattr(message, "twist") or not hasattr(message.twist, "covariance"):
        return None
    covariance = message.twist.covariance
    values = tuple(
        float(covariance[row * 6 + column])
        for row in range(3)
        for column in range(3)
    )
    return values if all(math.isfinite(value) for value in values) else None


def vector_subtract(right: Vector3, left: Vector3) -> Vector3:
    return tuple(r_value - l_value for r_value, l_value in zip(right, left))  # type: ignore[return-value]


def vector_error(left: Vector3, right: Vector3) -> float:
    return math.sqrt(sum((a - b) ** 2 for a, b in zip(left, right)))


def enu_to_ned(value: Vector3) -> Vector3:
    return (value[1], value[0], -value[2])


def quaternion_yaw(quaternion: Tuple[float, float, float, float]) -> float:
    x, y, z, w = quaternion
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z),
    )


def fmt_vector(value: Optional[Vector3]) -> str:
    if value is None:
        return "n/a"
    return "(" + ", ".join(f"{item:+.3f}" for item in value) + ")"


def fmt_seconds(value: float) -> str:
    return f"{value:.3f}s" if math.isfinite(value) else "n/a"


def percentile(values: Sequence[float], fraction: float) -> float:
    if not values:
        return math.nan
    ordered = sorted(values)
    index = min(len(ordered) - 1, max(0, round((len(ordered) - 1) * fraction)))
    return ordered[index]


def samples_between(samples, start_ns: int, end_ns: int):
    return [sample for sample in samples if start_ns <= sample[0] <= end_ns]


def phase_delta(samples, start_ns: int, end_ns: int) -> Optional[Vector3]:
    selected = samples_between(samples, start_ns, end_ns)
    if len(selected) < 2:
        return None
    return vector_subtract(selected[-1][1], selected[0][1])


def first_event_after(events, start_ns: int, predicate):
    return next((event for event in events if event[0] >= start_ns and predicate(event)), None)


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", type=Path, help="包含 metadata.yaml 的 rosbag 目录")
    parser.add_argument("--output", type=Path, help="Markdown 报告输出路径")
    parser.add_argument(
        "--strict-coordinate",
        action="store_true",
        help="要求 /race/odom、/tf、/tf_static 完整，并判定 base_link -> body TF",
    )
    args = parser.parse_args()

    bag_path = args.bag.resolve()
    if not (bag_path / "metadata.yaml").is_file():
        parser.error(f"metadata.yaml not found under {bag_path}")
    output_path = args.output or bag_path.parent / "validation_report.md"

    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag_path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {
        entry.name: entry.type for entry in reader.get_all_topics_and_types()
    }
    message_types = {name: get_message(type_name) for name, type_name in type_map.items()}

    counts: Dict[str, int] = defaultdict(int)
    receive_times: Dict[str, List[int]] = defaultdict(list)
    source_stamps: Dict[str, List[float]] = defaultdict(list)
    positions: Dict[str, List[Tuple[int, Vector3]]] = defaultdict(list)
    yaws: Dict[str, List[Tuple[int, float]]] = defaultdict(list)
    velocities: Dict[str, List[Tuple[int, Vector3]]] = defaultdict(list)
    velocity_covariances: Dict[str, List[Tuple[int, Tuple[float, ...]]]] = defaultdict(list)
    markers: List[Tuple[int, str]] = []
    health_states: List[Tuple[int, str]] = []
    health_faults: List[Tuple[int, bool]] = []
    diagnostics: List[Tuple[int, Dict[str, str]]] = []
    mavros_states: List[Tuple[int, bool, str]] = []
    transforms: Dict[str, List[TransformRow]] = defaultdict(list)

    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        counts[topic] += 1
        receive_times[topic].append(receive_ns)
        message_type = message_types.get(topic)
        if message_type is None:
            continue
        message = deserialize_message(serialized, message_type)
        source_stamp = stamp_seconds(message)
        if source_stamp is not None:
            source_stamps[topic].append(source_stamp)
        point = position(message)
        if point is not None and all(math.isfinite(item) for item in point):
            positions[topic].append((receive_ns, point))
        yaw = orientation_yaw(message)
        if yaw is not None and math.isfinite(yaw):
            yaws[topic].append((receive_ns, yaw))
        speed = velocity(message)
        if speed is not None and all(math.isfinite(item) for item in speed):
            velocities[topic].append((receive_ns, speed))
        covariance = linear_velocity_covariance(message)
        if covariance is not None:
            velocity_covariances[topic].append((receive_ns, covariance))

        if topic == "/ev_validation/marker":
            markers.append((receive_ns, str(message.data)))
        elif topic == "/ev_health/status":
            health_states.append((receive_ns, str(message.data)))
        elif topic == "/ev_health/fault":
            health_faults.append((receive_ns, bool(message.data)))
        elif topic == "/ev_health/diagnostics" and message.status:
            diagnostics.append(
                (receive_ns, {item.key: item.value for item in message.status[0].values})
            )
        elif topic == "/mavros/state":
            mavros_states.append((receive_ns, bool(message.armed), str(message.mode)))
        if topic in ("/tf", "/tf_static") and hasattr(message, "transforms"):
            for transform in message.transforms:
                translation = transform.transform.translation
                rotation = transform.transform.rotation
                transforms[topic].append(
                    (
                        receive_ns,
                        str(transform.header.frame_id),
                        str(transform.child_frame_id),
                        (translation.x, translation.y, translation.z),
                        (rotation.x, rotation.y, rotation.z, rotation.w),
                    )
                )

    marker_map = {label: time_ns for time_ns, label in markers}
    recovered_wall_markers = []
    wall_marker_path = bag_path.parent / "snapshot" / "markers_wall_time.txt"
    if wall_marker_path.is_file():
        for line in wall_marker_path.read_text(encoding="utf-8").splitlines():
            try:
                timestamp_text, label = line.split(maxsplit=1)
                timestamp_text = timestamp_text.replace(",", ".")
                timestamp_text = re.sub(
                    r"\.(\d{6})\d+([+-]\d{2}:\d{2}|Z)$",
                    r".\1\2",
                    timestamp_text,
                )
                marker_time_ns = int(
                    datetime.fromisoformat(timestamp_text).timestamp() * 1e9
                )
            except (ValueError, OverflowError):
                continue
            if label not in marker_map:
                marker_map[label] = marker_time_ns
                recovered_wall_markers.append(label)

    health_transitions = []
    previous_health_state = None
    for time_ns, state in health_states:
        if state == previous_health_state:
            continue
        health_transitions.append((time_ns, state))
        previous_health_state = state

    all_receive_times = [time_ns for times in receive_times.values() for time_ns in times]
    bag_end_ns = max(all_receive_times) if all_receive_times else 0

    def diagnostic_at_transition(time_ns: int, state: str) -> Dict[str, str]:
        matching = [
            (abs(diagnostic_time_ns - time_ns), values)
            for diagnostic_time_ns, values in diagnostics
            if values.get("state") == state
            and abs(diagnostic_time_ns - time_ns) <= int(0.5e9)
        ]
        return min(matching, key=lambda item: item[0])[1] if matching else {}

    lines = [
        "# Propeller-off EV validation report",
        "",
        f"Bag: `{bag_path}`",
        "",
        "## Safety evidence",
        "",
    ]

    armed_samples = sum(1 for _time, armed, _mode in mavros_states if armed)
    offboard_samples = sum(
        1 for _time, _armed, mode in mavros_states if mode.upper() == "OFFBOARD"
    )
    control_topics = (
        "/mavros/setpoint_raw/local",
        "/fmu/in/trajectory_setpoint",
        "/fmu/in/vehicle_command",
    )
    lines.extend(
        [
            f"- MAVROS armed=true samples: **{armed_samples}**",
            f"- MAVROS OFFBOARD samples: **{offboard_samples}**",
            "- Recorded control-topic message counts: "
            + ", ".join(f"`{topic}`={counts[topic]}" for topic in control_topics),
        ]
    )
    if recovered_wall_markers:
        lines.append(
            "- Markers recovered from the session wall-time snapshot: "
            + ", ".join(f"`{label}`" for label in recovered_wall_markers)
        )
    state_transitions = []
    previous_state = None
    for time_ns, armed, mode in mavros_states:
        state = (armed, mode)
        if state == previous_state:
            continue
        state_transitions.append((time_ns, armed, mode))
        previous_state = state
    if state_transitions:
        lines.append("MAVROS state transitions:")
        lines.append("")
        for time_ns, armed, mode in state_transitions:
            local_time = datetime.fromtimestamp(time_ns * 1e-9).astimezone()
            lines.append(
                f"- `{local_time.isoformat(timespec='milliseconds')}`: "
                f"armed={str(armed).lower()}, mode={mode}"
            )
        lines.append("")
    lines.extend(
        [
            "## Timing and frequency",
            "",
            "| Topic | Samples | Receive rate | Source rate | Non-monotonic stamps |",
            "|---|---:|---:|---:|---:|",
        ]
    )
    frequency_topics = (
        "/livox/lidar",
        "/livox/imu",
        "/Odometry",
        "/Odometry/healthy",
        "/mavros/odometry/out",
        "/mavros/local_position/odom",
        "/mavros/local_position/velocity_local",
    )
    for topic in frequency_topics:
        times = receive_times[topic]
        stamps = source_stamps[topic]
        receive_rate = (
            (len(times) - 1) / ((times[-1] - times[0]) * 1e-9)
            if len(times) > 1 and times[-1] > times[0]
            else math.nan
        )
        source_rate = (
            (len(stamps) - 1) / (stamps[-1] - stamps[0])
            if len(stamps) > 1 and stamps[-1] > stamps[0]
            else math.nan
        )
        non_monotonic = sum(
            1 for left, right in zip(stamps, stamps[1:]) if right <= left
        )
        lines.append(
            f"| `{topic}` | {counts[topic]} | {receive_rate:.2f} Hz | "
            f"{source_rate:.2f} Hz | {non_monotonic} |"
        )

    lines.extend([
        "",
        "### Recorded coverage",
        "",
        "| Topic | First sample | Last sample | Span | Silence before bag end |",
        "|---|---|---|---:|---:|",
    ])
    coverage_topics = (
        "/Odometry",
        "/Odometry/healthy",
        "/mavros/vision_pose/pose_cov",
        "/mavros/vision_speed/speed_twist_cov",
        "/mavros/local_position/odom",
        "/mavros/local_position/velocity_local",
        "/race/odom",
        "/race/control/status",
        "/ev_health/status",
    )
    for topic in coverage_topics:
        times = receive_times[topic]
        if not times:
            lines.append(f"| `{topic}` | n/a | n/a | n/a | n/a |")
            continue
        first_time = datetime.fromtimestamp(times[0] * 1e-9).astimezone()
        last_time = datetime.fromtimestamp(times[-1] * 1e-9).astimezone()
        span_s = (times[-1] - times[0]) * 1e-9
        end_silence_s = (bag_end_ns - times[-1]) * 1e-9
        lines.append(
            f"| `{topic}` | {first_time.isoformat(timespec='milliseconds')} | "
            f"{last_time.isoformat(timespec='milliseconds')} | {span_s:.3f}s | "
            f"{end_silence_s:.3f}s |"
        )

    strict_topics = (
        "/livox/lidar",
        "/livox/imu",
        "/Odometry",
        "/race/odom",
        "/tf",
        "/tf_static",
        "/mavros/vision_pose/pose_cov",
        "/mavros/local_position/odom",
        "/mavros/local_position/velocity_local",
    )
    strict_topics_present = all(counts[topic] > 0 for topic in strict_topics)
    strict_sensor_stamp_regressions = {
        topic: sum(
            1
            for left, right in zip(source_stamps[topic], source_stamps[topic][1:])
            if right <= left
        )
        for topic in ("/livox/lidar", "/livox/imu")
    }
    strict_sensor_time_ok = all(
        counts[topic] > 1 and regressions == 0
        for topic, regressions in strict_sensor_stamp_regressions.items()
    )
    base_body_tf = next(
        (
            row
            for row in transforms["/tf_static"]
            if row[1] == "base_link" and row[2] == "body"
        ),
        None,
    )
    map_base_tf_found = any(
        parent == "map" and child == "base_link"
        for _time_ns, parent, child, _translation, _rotation in transforms["/tf"]
    )
    base_body_yaw_deg = (
        math.degrees(quaternion_yaw(base_body_tf[4]))
        if base_body_tf is not None
        else math.nan
    )
    if args.strict_coordinate:
        lines.extend([
            "",
            "## Strict coordinate/TF evidence",
            "",
            "- Required topic counts: "
            + ", ".join(f"`{topic}`={counts[topic]}" for topic in strict_topics),
            "- Sensor source stamp regressions: "
            + ", ".join(
                f"`{topic}`={regressions}"
                for topic, regressions in strict_sensor_stamp_regressions.items()
            ),
            f"- Dynamic `map -> base_link` found: **{map_base_tf_found}**",
        ])
        if base_body_tf is None:
            lines.append("- Static `base_link -> body`: **missing**")
        else:
            lines.extend([
                f"- Static `base_link -> body` translation: "
                f"**{fmt_vector(base_body_tf[3])} m**",
                f"- Static `base_link -> body` yaw: **{base_body_yaw_deg:+.3f} deg**",
                "- This is the candidate TF recorded during the run; the marked "
                "body-velocity signs below decide whether its yaw is correct.",
            ])

    ages = []
    for _time_ns, values in diagnostics:
        try:
            value = float(values.get("input_age_s", "nan"))
        except ValueError:
            continue
        if math.isfinite(value):
            ages.append(value)
    lines.extend([
        "",
        "Source age from `/ev_health/diagnostics`: "
        + (
            f"min={min(ages):.4f}s, p50={percentile(ages, 0.50):.4f}s, "
            f"p95={percentile(ages, 0.95):.4f}s, "
            f"p99={percentile(ages, 0.99):.4f}s, max={max(ages):.4f}s"
            if ages
            else "n/a"
        ),
        "",
        "## 60-second static velocity acceptance",
        "",
    ])
    static_start_ns = marker_map.get("STATIC_60S_START")
    static_end_ns = marker_map.get("STATIC_60S_END")
    static_topic = (
        "/race/odom" if velocities["/race/odom"] else "/mavros/odometry/out"
    )
    if static_start_ns is not None and static_end_ns is not None:
        static_velocities = samples_between(
            velocities[static_topic], static_start_ns, static_end_ns
        )
        static_covariances = samples_between(
            velocity_covariances[static_topic], static_start_ns, static_end_ns
        )
        if static_velocities:
            axes = list(zip(*(value for _time_ns, value in static_velocities)))
            norms = [
                math.sqrt(sum(component * component for component in value))
                for _time_ns, value in static_velocities
            ]
            means = tuple(statistics.fmean(axis) for axis in axes)
            deviations = tuple(
                statistics.pstdev(axis) if len(axis) > 1 else 0.0 for axis in axes
            )
            rms = tuple(
                math.sqrt(statistics.fmean(value * value for value in axis))
                for axis in axes
            )
            lines.extend([
                f"- Samples: **{len(static_velocities)}**",
                f"- Axis mean FLU m/s: **{fmt_vector(means)}**",
                f"- Axis standard deviation m/s: **{fmt_vector(deviations)}**",
                f"- Axis RMS m/s: **{fmt_vector(rms)}**",
                f"- Speed norm p95/p99/max: **{percentile(norms, 0.95):.3f} / "
                f"{percentile(norms, 0.99):.3f} / {max(norms):.3f} m/s**",
                f"- Samples above 0.25 m/s: **{sum(value > 0.25 for value in norms)}**",
            ])
        else:
            lines.append(f"- **INCOMPLETE:** no `{static_topic}` velocity samples.")
        if static_covariances:
            for axis, index in zip("xyz", (0, 4, 8)):
                values = [value[index] for _time_ns, value in static_covariances]
                lines.append(
                    f"- Velocity variance {axis}: mean={statistics.fmean(values):.6g}, "
                    f"range=[{min(values):.6g}, {max(values):.6g}] m2/s2"
                )
            distinct_covariances = len({
                tuple(round(value, 12) for value in covariance)
                for _, covariance in static_covariances
            })
            lines.append(
                f"- Distinct 3x3 covariance matrices: **{distinct_covariances}**"
            )
    else:
        lines.append("- **INCOMPLETE:** STATIC_60S markers are missing.")

    lines.extend(["", "## Health transitions", ""])
    first_health_time = health_states[0][0] if health_states else None
    first_healthy = next(
        (item for item in health_transitions if item[1] == "HEALTHY"), None
    )
    startup_elapsed = (
        (first_healthy[0] - first_health_time) * 1e-9
        if first_healthy is not None and first_health_time is not None
        else math.nan
    )
    if health_states and health_states[0][1] == "HEALTHY":
        lines.append(
            "- Startup hysteresis: **not observable** (recording began after HEALTHY)."
        )
    else:
        lines.append(
            f"- First health status to first HEALTHY transition: **{fmt_seconds(startup_elapsed)}**"
        )

    raw_times = receive_times["/Odometry"]
    raw_gap = None
    if len(raw_times) >= 2:
        gap_candidates = [
            (right - left, left, right)
            for left, right in zip(raw_times, raw_times[1:])
            if right - left >= int(0.5e9)
        ]
        if gap_candidates:
            raw_gap = max(gap_candidates)
    if raw_gap is not None:
        gap_ns, last_raw_ns, first_raw_after_ns = raw_gap
        suspect = first_event_after(
            health_transitions,
            last_raw_ns,
            lambda item: item[1] == "SUSPECT",
        )
        fault = first_event_after(
            health_transitions,
            last_raw_ns,
            lambda item: item[1] == "FAULT",
        )
        recovered = first_event_after(
            health_transitions,
            first_raw_after_ns,
            lambda item: item[1] == "HEALTHY",
        )
        suspect_s = (suspect[0] - last_raw_ns) * 1e-9 if suspect else math.nan
        fault_s = (fault[0] - last_raw_ns) * 1e-9 if fault else math.nan
        recovery_s = (
            (recovered[0] - first_raw_after_ns) * 1e-9 if recovered else math.nan
        )
        healthy_during_fault = [
            time_ns
            for time_ns in receive_times["/Odometry/healthy"]
            if (fault[0] if fault else last_raw_ns) <= time_ns < first_raw_after_ns
        ]
        lines.append(f"- Largest raw Odometry gap: **{gap_ns * 1e-9:.3f}s**")
        lines.append(f"- Last raw sample to SUSPECT: **{fmt_seconds(suspect_s)}**")
        lines.append(f"- Last raw sample to FAULT: **{fmt_seconds(fault_s)}**")
        lines.append(
            "- `/Odometry/healthy` samples during FAULT before raw recovery: "
            f"**{len(healthy_during_fault)}**"
        )
        lines.append(
            f"- First raw sample after gap to HEALTHY: **{fmt_seconds(recovery_s)}**"
        )
    if raw_times and bag_end_ns > raw_times[-1]:
        terminal_gap_s = (bag_end_ns - raw_times[-1]) * 1e-9
        if terminal_gap_s >= 0.5:
            lines.append(
                f"- Terminal raw Odometry silence before bag end: **{terminal_gap_s:.3f}s**"
            )
    if health_states:
        final_state = health_states[-1][1]
        final_edge_ns = next(
            time_ns
            for time_ns, state in reversed(health_transitions)
            if state == final_state
        )
        lines.append(
            f"- Final recorded state: **{final_state}** for at least "
            f"**{(health_states[-1][0] - final_edge_ns) * 1e-9:.3f}s**; "
            "no later recovery is present in the bag."
        )
    lines.append("")
    lines.append("Observed health state edges and reasons:")
    lines.append("")
    for time_ns, state in health_transitions:
        local_time = datetime.fromtimestamp(time_ns * 1e-9).astimezone()
        transition_diagnostic = diagnostic_at_transition(time_ns, state)
        reason = transition_diagnostic.get("reason", "unknown")
        metric_details = []
        if reason.startswith("stale_input"):
            metric_details.append(
                f"input_age_s={transition_diagnostic.get('input_age_s', 'n/a')}"
            )
        if reason == "px4_velocity_unaligned":
            metric_details.append(
                "px4_alignment_miss_s="
                + transition_diagnostic.get("velocity_alignment_s", "n/a")
            )
        metric_suffix = f" ({', '.join(metric_details)})" if metric_details else ""
        lines.append(
            f"- `{local_time.isoformat(timespec='milliseconds')}`: {state}, "
            f"reason=`{reason}`{metric_suffix}"
        )

    lines.extend(
        [
            "",
            "## Axis/sign checks",
            "",
            "Raw/healthy positions use ENU-like world axes; ODOMETRY pose keeps that "
            "world convention while its twist is expressed in child/body FLU. "
            "PX4 NED is `(N,E,D)=(ENU y, ENU x, -ENU z)`.",
            "",
            "| Motion | Raw ENU delta | Healthy ENU delta | race/odom ENU delta | "
            "PX4 NED delta | Raw->NED error |",
            "|---|---|---|---|---|---:|",
        ]
    )
    axis_chain_results: Dict[str, bool] = {}
    expected_axis = {
        "FORWARD": (0, 1),
        "BACKWARD": (0, -1),
        "LEFT": (1, 1),
        "RIGHT": (1, -1),
        "UP": (2, 1),
        "DOWN": (2, -1),
    }
    for label, (axis, sign) in expected_axis.items():
        start_ns = marker_map.get(f"{label}_START")
        stop_ns = marker_map.get(f"{label}_STOPPED")
        if start_ns is None or stop_ns is None:
            continue
        raw_delta = phase_delta(positions["/Odometry"], start_ns, stop_ns)
        healthy_delta = phase_delta(positions["/Odometry/healthy"], start_ns, stop_ns)
        converted_topic = (
            "/race/odom"
            if positions["/race/odom"]
            else "/mavros/odometry/out"
        )
        vision_delta = phase_delta(positions[converted_topic], start_ns, stop_ns)
        px4_delta = phase_delta(
            positions["/fmu/out/vehicle_local_position"], start_ns, stop_ns
        )
        if px4_delta is None:
            # MAVROS 暴露的是被转回 ENU 的 PX4 局部位置。把它实测的位移增量转成
            # NED，作为可比对的备用值。
            mavros_delta = phase_delta(
                positions["/mavros/local_position/odom"], start_ns, stop_ns
            )
            px4_delta = enu_to_ned(mavros_delta) if mavros_delta else None
        expected_ned = enu_to_ned(raw_delta) if raw_delta else None
        mapping_error = (
            vector_error(px4_delta, expected_ned)
            if px4_delta is not None and expected_ned is not None
            else math.nan
        )
        sign_ok = (
            raw_delta is not None
            and abs(raw_delta[axis]) >= 0.05
            and raw_delta[axis] * sign > 0.0
        )
        chain_available = healthy_delta is not None and vision_delta is not None
        healthy_error = (
            vector_error(raw_delta, healthy_delta)
            if raw_delta is not None and healthy_delta is not None
            else math.nan
        )
        vision_error = (
            vector_error(raw_delta, vision_delta)
            if raw_delta is not None and vision_delta is not None
            else math.nan
        )
        chain_ok = (
            chain_available
            and sign_ok
            and healthy_error <= 0.05
            and vision_error <= 0.05
            and math.isfinite(mapping_error)
            and mapping_error <= 0.15
        )
        axis_chain_results[label] = chain_ok
        result = "PASS" if chain_ok else ("INCOMPLETE" if not chain_available else "FAIL")
        lines.append(
            f"| {label} {result} | {fmt_vector(raw_delta)} | "
            f"{fmt_vector(healthy_delta)} | {fmt_vector(vision_delta)} | "
            f"{fmt_vector(px4_delta)} | {mapping_error:.3f} m |"
        )

    lines.extend([
        "",
        "### Body-FLU velocity sign checks",
        "",
        "| Motion | Expected component | Peak signed component | Result |",
        "|---|---|---:|---|",
    ])
    expected_body_velocity = {
        "FORWARD": (0, 1),
        "BACKWARD": (0, -1),
        "LEFT": (1, 1),
        "RIGHT": (1, -1),
        "UP": (2, 1),
        "DOWN": (2, -1),
        "YAW90_FORWARD": (0, 1),
    }
    body_velocity_results: Dict[str, bool] = {}
    body_velocity_components: Dict[str, float] = {}
    for label, (axis, sign) in expected_body_velocity.items():
        start_ns = marker_map.get(f"{label}_START")
        stop_ns = marker_map.get(f"{label}_STOPPED")
        if start_ns is None or stop_ns is None:
            lines.append(f"| {label} | axis {axis}, sign {sign:+d} | n/a | INCOMPLETE |")
            continue
        body_velocity_topic = (
            "/race/odom"
            if velocities["/race/odom"]
            else "/mavros/odometry/out"
        )
        selected = samples_between(
            velocities[body_velocity_topic], start_ns, stop_ns
        )
        components = [value[axis] for _time_ns, value in selected]
        dominant = max(components, key=abs) if components else math.nan
        peak = dominant * sign if math.isfinite(dominant) else math.nan
        result = "PASS" if math.isfinite(peak) and peak >= 0.05 else "FAIL"
        body_velocity_results[label] = result == "PASS"
        body_velocity_components[label] = dominant
        component = ("x", "y", "z")[axis]
        expected_sign = "> 0" if sign > 0 else "< 0"
        lines.append(
            f"| {label} | `{component} {expected_sign}` | {peak:.3f} m/s | {result} |"
        )

    if args.strict_coordinate:
        forward_component = body_velocity_components.get("FORWARD", math.nan)
        left_component = body_velocity_components.get("LEFT", math.nan)
        lines.extend(["", "### Installation-yaw decision", ""])
        if forward_component > 0.05 and left_component > 0.05:
            lines.append(
                "- `/race/odom` reports forward `x>0` and left `y>0`: "
                "**retain the recorded installation yaw**."
            )
        elif forward_component < -0.05 and left_component < -0.05:
            current_installation_deg = (
                -base_body_yaw_deg if math.isfinite(base_body_yaw_deg) else math.nan
            )
            suggested_installation_deg = (
                (current_installation_deg + 180.0 + 180.0) % 360.0 - 180.0
                if math.isfinite(current_installation_deg)
                else math.nan
            )
            lines.append(
                "- `/race/odom` reports forward `x<0` and left `y<0`: the "
                "recorded installation yaw applies an extra 180 deg rotation."
            )
            if math.isfinite(suggested_installation_deg):
                lines.append(
                    f"- Suggested `body_to_fastlio_yaw_rad`: "
                    f"**{math.radians(suggested_installation_deg):+.6f} rad "
                    f"({suggested_installation_deg:+.1f} deg)**."
                )
        else:
            lines.append(
                "- Forward/left signs do not form a pure 0/180-deg mapping; "
                "do not modify TF from this run. Repeat the marked motions."
            )

    lines.extend([
        "",
        "### Vehicle Odom heading-to-motion checks",
        "",
        "The RViz `Vehicle Odom` arrow is `/race/odom` body `+X`. During a marked "
        "forward movement, its yaw must agree with the measured XY displacement.",
        "",
        "| Motion | XY displacement | Motion heading | Mean arrow yaw | Error | Result |",
        "|---|---:|---:|---:|---:|---|",
    ])
    heading_motion_results: Dict[str, bool] = {}
    heading_topic = "/race/odom" if positions["/race/odom"] else "/mavros/odometry/out"
    max_heading_error_rad = math.radians(20.0)
    for label in ("FORWARD", "YAW90_FORWARD"):
        start_ns = marker_map.get(f"{label}_START")
        stop_ns = marker_map.get(f"{label}_STOPPED")
        delta = (
            phase_delta(positions[heading_topic], start_ns, stop_ns)
            if start_ns is not None and stop_ns is not None
            else None
        )
        selected_yaws = (
            samples_between(yaws[heading_topic], start_ns, stop_ns)
            if start_ns is not None and stop_ns is not None
            else []
        )
        distance_xy = math.hypot(delta[0], delta[1]) if delta is not None else math.nan
        motion_heading = (
            math.atan2(delta[1], delta[0]) if delta is not None and distance_xy > 0.0
            else math.nan
        )
        mean_yaw = (
            math.atan2(
                sum(math.sin(value) for _time_ns, value in selected_yaws),
                sum(math.cos(value) for _time_ns, value in selected_yaws),
            )
            if selected_yaws else math.nan
        )
        heading_error = (
            math.atan2(
                math.sin(motion_heading - mean_yaw),
                math.cos(motion_heading - mean_yaw),
            )
            if math.isfinite(motion_heading) and math.isfinite(mean_yaw)
            else math.nan
        )
        passed = (
            distance_xy >= 0.20
            and math.isfinite(heading_error)
            and abs(heading_error) <= max_heading_error_rad
        )
        heading_motion_results[label] = passed
        result = "PASS" if passed else (
            "INCOMPLETE" if delta is None or not selected_yaws else "FAIL"
        )
        lines.append(
            f"| {label} | {distance_xy:.3f} m | "
            f"{math.degrees(motion_heading):+.1f} deg | "
            f"{math.degrees(mean_yaw):+.1f} deg | "
            f"{math.degrees(heading_error):+.1f} deg | {result} |"
        )

    lines.extend(["", "## Stop-to-zero checks", ""])
    for label in expected_axis:
        stopped_ns = marker_map.get(f"{label}_STOPPED")
        settled_ns = marker_map.get(f"{label}_SETTLED")
        if stopped_ns is None or settled_ns is None:
            continue
        selected = samples_between(
            velocities["/ev_health/velocity_ned"], stopped_ns, settled_ns
        )
        first_zero = next(
            (
                (time_ns - stopped_ns) * 1e-9
                for time_ns, value in selected
                if math.sqrt(sum(item * item for item in value)) <= 0.05
            ),
            math.nan,
        )
        lines.append(
            f"- {label}: speed <= 0.05 m/s after **{fmt_seconds(first_zero)}**"
        )

    lines.extend(
        [
            "",
            "## Interpretation thresholds",
            "",
            "- Startup/recovery HEALTHY should not occur before about 7.5 s in this validation launch.",
            "- Raw source stamps must be strictly increasing; normal source age should stay below 0.25 s.",
            "- FAST-LIO timeout should produce SUSPECT after about 0.5 s and FAULT about 0.3 s later.",
            "- No healthy position sample may be emitted during FAULT.",
            "- A stopped motion should fall below 0.05 m/s within 1.0 s.",
            "- Axis deltas must have the expected sign and ENU/NED mapping error should remain small.",
            "",
            "## EV velocity fusion readiness",
            "",
        ]
    )

    raw_stamps = source_stamps["/Odometry"]
    raw_source_rate = (
        (len(raw_stamps) - 1) / (raw_stamps[-1] - raw_stamps[0])
        if len(raw_stamps) > 1 and raw_stamps[-1] > raw_stamps[0]
        else math.nan
    )
    if math.isfinite(raw_source_rate) and raw_source_rate < 30.0:
        lines.append(
            f"- `FUSION_RATE_READINESS_FAIL`: real FAST-LIO source rate is "
            f"{raw_source_rate:.2f} Hz; no sample repetition is permitted."
        )
    lines.extend([
        "- PX4 console evidence (`vehicle_visual_odometry`, aid-source flags, and "
        "`cs_ev_vel=false`) must be attached separately; a ROS bag cannot prove all uORB states.",
        "- **READY_FOR_EV_VEL_FUSION = NO** until every static, hand-held, PX4-receive, "
        "and real-rate criterion passes.",
    ])

    rosbag_status_path = bag_path.parent / "snapshot" / "rosbag_status.txt"
    rosbag_saved_cleanly = not rosbag_status_path.is_file() or (
        "state=saved_cleanly" in rosbag_status_path.read_text(encoding="utf-8")
    )
    required_chain_topics = (
        "/Odometry/healthy",
        "/mavros/vision_pose/pose_cov",
        "/mavros/vision_speed/speed_twist_cov",
        "/mavros/local_position/odom",
        "/mavros/local_position/velocity_local",
    )
    chain_topics_present = all(counts[topic] > 0 for topic in required_chain_topics)
    no_control_messages = all(counts[topic] == 0 for topic in control_topics)
    final_health_healthy = bool(health_states) and health_states[-1][1] == "HEALTHY"
    static_complete = static_start_ns is not None and static_end_ns is not None
    required_axis_labels = (
        ("LEFT", "FORWARD", "UP", "DOWN")
        if args.strict_coordinate
        else tuple(expected_axis)
    )
    axis_complete = all(
        marker_map.get(f"{label}_START") is not None
        and marker_map.get(f"{label}_STOPPED") is not None
        for label in required_axis_labels
    )
    strict_motion_complete = (
        not args.strict_coordinate
        or (
            axis_complete
            and marker_map.get("YAW360_START") is not None
            and marker_map.get("YAW360_STOPPED") is not None
            and marker_map.get("YAW90_FORWARD_START") is not None
            and marker_map.get("YAW90_FORWARD_STOPPED") is not None
        )
    )
    strict_axis_pass = (
        not args.strict_coordinate
        or (
            all(axis_chain_results.get(label, False) for label in required_axis_labels)
            and all(
                body_velocity_results.get(label, False)
                for label in (*required_axis_labels, "YAW90_FORWARD")
            )
            and all(
                heading_motion_results.get(label, False)
                for label in ("FORWARD", "YAW90_FORWARD")
            )
        )
    )
    strict_tf_complete = (
        not args.strict_coordinate
        or (strict_topics_present and map_base_tf_found and base_body_tf is not None)
    )
    overall_pass = all((
        rosbag_saved_cleanly,
        chain_topics_present,
        armed_samples == 0,
        offboard_samples == 0,
        no_control_messages,
        final_health_healthy,
        static_complete,
        axis_complete,
        strict_motion_complete,
        strict_axis_pass,
        strict_tf_complete,
        not args.strict_coordinate or strict_sensor_time_ok,
    ))
    lines.extend([
        "",
        "## Run-level verdict",
        "",
        "| Criterion | Result |",
        "|---|---|",
        f"| Bag readable; clean-save status passes when present | {'PASS' if rosbag_saved_cleanly else 'FAIL'} |",
        f"| Required EV/MAVROS chain topics recorded | {'PASS' if chain_topics_present else 'FAIL'} |",
        f"| Unarmed and never OFFBOARD | {'PASS' if armed_samples == 0 and offboard_samples == 0 else 'FAIL'} |",
        f"| No control/setpoint messages | {'PASS' if no_control_messages else 'FAIL'} |",
        f"| EV health remained acceptable through recording end | {'PASS' if final_health_healthy else 'FAIL'} |",
        f"| Marked 60-second static test | {'PASS' if static_complete else 'INCOMPLETE'} |",
        f"| Marked physical axis/sign tests | {'PASS' if axis_complete else 'INCOMPLETE'} |",
        "",
        f"**OVERALL_PROP_OFF_VALIDATION = {'PASS' if overall_pass else 'NOT PASSED'}**",
    ])
    if args.strict_coordinate:
        lines[-2:-2] = [
            f"| Strict coordinate topics and TF captured | "
            f"{'PASS' if strict_tf_complete else 'FAIL'} |",
            f"| Livox LiDAR/IMU source stamps strictly increase | "
            f"{'PASS' if strict_sensor_time_ok else 'FAIL'} |",
            f"| Strict marked motion sequence completed | "
            f"{'PASS' if strict_motion_complete else 'INCOMPLETE'} |",
            f"| `/race/odom` axis/sign mapping | "
            f"{'PASS' if strict_axis_pass else 'FAIL'} |",
            f"| `Vehicle Odom` arrow agrees with forward displacement | "
            f"{'PASS' if all(heading_motion_results.get(label, False) for label in ('FORWARD', 'YAW90_FORWARD')) else 'FAIL'} |",
        ]

    output_path.write_text("\n".join(lines) + "\n", encoding="utf-8")
    print(output_path)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
