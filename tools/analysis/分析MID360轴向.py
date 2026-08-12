#!/usr/bin/env python3
"""分析带标记的 MID360/FAST-LIO 无桨轴向验证录包。"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
from typing import Dict, Iterable, List, Sequence, Tuple

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


POSE_TOPICS = (
    "/Odometry",
    "/Odometry/healthy",
    "/mavros/vision_pose/pose_cov",
    "/mavros/local_position/pose",
    "/mavros/local_position/odom",
)
REQUIRED_POSE_TOPICS = (
    "/Odometry",
    "/Odometry/healthy",
    "/mavros/vision_pose/pose_cov",
    "/mavros/local_position/odom",
)
MARKER_TOPIC = "/axis_validation/marker"
STATE_TOPIC = "/mavros/state"
HEALTH_TOPIC = "/ev_health/status"
HEALTH_DIAGNOSTIC_TOPIC = "/ev_health/diagnostics"
TF_STATIC_TOPIC = "/tf_static"


def stamp_ns(header) -> int:
    return int(header.stamp.sec) * 1_000_000_000 + int(header.stamp.nanosec)


def pose_from_message(message):
    pose = message.pose.pose if hasattr(message.pose, "pose") else message.pose
    position = pose.position
    orientation = pose.orientation
    return (
        np.array((position.x, position.y, position.z), dtype=float),
        np.array(
            (orientation.x, orientation.y, orientation.z, orientation.w),
            dtype=float,
        ),
    )


def quaternion_normalize(quaternion: np.ndarray) -> np.ndarray:
    norm = np.linalg.norm(quaternion, axis=-1, keepdims=True)
    return quaternion / norm


def quaternion_multiply(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    ax, ay, az, aw = np.moveaxis(a, -1, 0)
    bx, by, bz, bw = np.moveaxis(b, -1, 0)
    return np.stack(
        (
            aw * bx + ax * bw + ay * bz - az * by,
            aw * by - ax * bz + ay * bw + az * bx,
            aw * bz + ax * by - ay * bx + az * bw,
            aw * bw - ax * bx - ay * by - az * bz,
        ),
        axis=-1,
    )


def quaternion_conjugate(quaternion: np.ndarray) -> np.ndarray:
    output = np.array(quaternion, copy=True)
    output[..., :3] *= -1.0
    return output


def quaternion_angle(quaternion: np.ndarray) -> np.ndarray:
    normalized = quaternion_normalize(quaternion)
    scalar = np.clip(np.abs(normalized[..., 3]), 0.0, 1.0)
    return 2.0 * np.arccos(scalar)


def quaternion_to_rotation(quaternion: np.ndarray) -> np.ndarray:
    x, y, z, w = quaternion_normalize(quaternion)
    return np.array(
        (
            (1.0 - 2.0 * (y * y + z * z), 2.0 * (x * y - z * w), 2.0 * (x * z + y * w)),
            (2.0 * (x * y + z * w), 1.0 - 2.0 * (x * x + z * z), 2.0 * (y * z - x * w)),
            (2.0 * (x * z - y * w), 2.0 * (y * z + x * w), 1.0 - 2.0 * (x * x + y * y)),
        ),
        dtype=float,
    )


def quaternion_to_euler(quaternions: np.ndarray) -> np.ndarray:
    q = quaternion_normalize(quaternions)
    x, y, z, w = np.moveaxis(q, -1, 0)
    roll = np.arctan2(2.0 * (w * x + y * z), 1.0 - 2.0 * (x * x + y * y))
    pitch = np.arcsin(np.clip(2.0 * (w * y - z * x), -1.0, 1.0))
    yaw = np.arctan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
    return np.stack((roll, pitch, yaw), axis=-1)


def circular_difference(a: np.ndarray, b: np.ndarray) -> np.ndarray:
    return np.arctan2(np.sin(a - b), np.cos(a - b))


def median_pose_window(series: Dict[str, np.ndarray], begin: float, end: float):
    mask = (series["time"] >= begin) & (series["time"] <= end)
    if np.count_nonzero(mask) < 2:
        raise RuntimeError(f"insufficient pose data in window {begin:.3f}-{end:.3f}s")
    position = np.median(series["position"][mask], axis=0)
    quaternion = quaternion_normalize(np.median(series["quaternion"][mask], axis=0))
    return position, quaternion


def circle_fit(points: np.ndarray) -> Dict[str, float]:
    if len(points) < 6:
        raise RuntimeError("circle fit requires at least six samples")
    x = points[:, 0]
    y = points[:, 1]
    design = np.column_stack((2.0 * x, 2.0 * y, np.ones_like(x)))
    solution, _, _, _ = np.linalg.lstsq(design, x * x + y * y, rcond=None)
    center_x, center_y, constant = solution
    radius = math.sqrt(max(0.0, constant + center_x * center_x + center_y * center_y))
    radial = np.hypot(x - center_x, y - center_y)
    centroid = np.mean(points[:, :2], axis=0)
    centroid_radius = np.sqrt(np.mean(np.sum((points[:, :2] - centroid) ** 2, axis=1)))
    return {
        "center_x_m": float(center_x),
        "center_y_m": float(center_y),
        "radius_m": float(radius),
        "radial_rms_m": float(np.sqrt(np.mean((radial - radius) ** 2))),
        "centroid_rms_radius_m": float(centroid_radius),
        "xy_peak_to_peak_m": float(
            math.hypot(float(np.ptp(points[:, 0])), float(np.ptp(points[:, 1])))
        ),
    }


def find_rotation_window(series: Dict[str, np.ndarray], begin: float, end: float):
    mask = (series["time"] >= begin) & (series["time"] <= end)
    indexes = np.flatnonzero(mask)
    if len(indexes) < 10:
        raise RuntimeError("insufficient samples in rotation segment")
    time = series["time"][indexes]
    yaw = np.unwrap(quaternion_to_euler(series["quaternion"][indexes])[:, 2])
    yaw_rate = np.gradient(yaw, time)
    moving = np.flatnonzero(np.abs(yaw_rate) >= math.radians(5.0))
    if len(moving) < 5:
        raise RuntimeError("could not detect the 360-degree rotation window")
    first = max(0, int(moving[0]) - 2)
    last = min(len(indexes) - 1, int(moving[-1]) + 2)
    selected = indexes[first : last + 1]
    selected_yaw = np.unwrap(
        quaternion_to_euler(series["quaternion"][selected])[:, 2]
    )
    return selected, float(selected_yaw[-1] - selected_yaw[0])


def segment_motion(
    series: Dict[str, np.ndarray],
    begin: float,
    end: float,
    expected_axis: int,
) -> Dict[str, object]:
    baseline_position, baseline_quaternion = median_pose_window(
        series, max(0.0, begin - 2.5), max(0.0, begin - 0.3)
    )
    mask = (series["time"] >= begin) & (series["time"] <= end)
    positions = series["position"][mask]
    body_rotation_world = quaternion_to_rotation(baseline_quaternion).T
    body_delta = (body_rotation_world @ (positions - baseline_position).T).T
    peak_index = int(np.argmax(body_delta[:, expected_axis]))
    endpoint_position, _ = median_pose_window(
        series, max(begin, end - 3.0), max(begin, end - 0.3)
    )
    endpoint_body = body_rotation_world @ (endpoint_position - baseline_position)
    return {
        "peak_body_delta_m": body_delta[peak_index].tolist(),
        "endpoint_body_delta_m": endpoint_body.tolist(),
        "expected_axis_peak_m": float(body_delta[peak_index, expected_axis]),
        "opposite_axis_peak_m": float(-np.min(body_delta[:, expected_axis])),
        "samples": int(len(positions)),
    }


def attitude_segment(
    series: Dict[str, np.ndarray], begin: float, end: float, primary_axis: int
) -> Dict[str, float]:
    mask = (series["time"] >= begin) & (series["time"] <= end)
    euler = quaternion_to_euler(series["quaternion"][mask])
    euler = np.unwrap(euler, axis=0)
    position = series["position"][mask]
    baseline_position, _ = median_pose_window(
        series, max(0.0, begin - 2.5), max(0.0, begin - 0.3)
    )
    displacement = position - baseline_position
    return {
        "primary_range_deg": float(math.degrees(np.ptp(euler[:, primary_axis]))),
        "yaw_range_deg": float(math.degrees(np.ptp(euler[:, 2]))),
        "horizontal_excursion_m": float(np.max(np.linalg.norm(displacement[:, :2], axis=1))),
        "vertical_excursion_m": float(np.max(np.abs(displacement[:, 2]))),
    }


def relative_rotation_segment(
    series: Dict[str, np.ndarray], begin: float, end: float
) -> Dict[str, object]:
    _, baseline_quaternion = median_pose_window(
        series, max(0.0, begin - 2.5), max(0.0, begin - 0.3)
    )
    mask = (series["time"] >= begin) & (series["time"] <= end)
    quaternion = quaternion_normalize(series["quaternion"][mask])
    relative = quaternion_multiply(
        np.broadcast_to(quaternion_conjugate(baseline_quaternion), quaternion.shape),
        quaternion,
    )
    relative = quaternion_normalize(relative)
    relative[relative[:, 3] < 0.0] *= -1.0
    angle = 2.0 * np.arccos(np.clip(relative[:, 3], -1.0, 1.0))
    vector_norm = np.linalg.norm(relative[:, :3], axis=1)
    rotation_vector = np.zeros_like(relative[:, :3])
    valid = vector_norm > 1e-12
    rotation_vector[valid] = (
        relative[valid, :3] / vector_norm[valid, None] * angle[valid, None]
    )
    return {
        "xyz_range_deg": np.degrees(np.ptp(rotation_vector, axis=0)).tolist(),
        "xyz_max_abs_deg": np.degrees(
            np.max(np.abs(rotation_vector), axis=0)
        ).tolist(),
    }


def detect_attitude_window(
    series: Dict[str, np.ndarray],
    search_begin: float,
    search_end: float,
    primary_axis: int,
    threshold_deg: float = 8.0,
) -> Tuple[float, float]:
    """找出相对搜索起始姿态的大幅 roll/pitch 摆动区间。"""
    _, baseline_quaternion = median_pose_window(
        series, max(0.0, search_begin - 2.5), max(0.0, search_begin - 0.3)
    )
    search_mask = (series["time"] >= search_begin) & (series["time"] <= search_end)
    indexes = np.flatnonzero(search_mask)
    if len(indexes) < 5:
        raise RuntimeError("insufficient samples while detecting attitude motion")
    quaternion = quaternion_normalize(series["quaternion"][indexes])
    relative = quaternion_multiply(
        np.broadcast_to(quaternion_conjugate(baseline_quaternion), quaternion.shape),
        quaternion,
    )
    relative[relative[:, 3] < 0.0] *= -1.0
    angle = 2.0 * np.arccos(np.clip(relative[:, 3], -1.0, 1.0))
    vector_norm = np.linalg.norm(relative[:, :3], axis=1)
    rotation_vector = np.zeros_like(relative[:, :3])
    valid = vector_norm > 1e-12
    rotation_vector[valid] = (
        relative[valid, :3] / vector_norm[valid, None] * angle[valid, None]
    )
    active = np.flatnonzero(
        np.abs(rotation_vector[:, primary_axis]) >= math.radians(threshold_deg)
    )
    if len(active) < 3:
        raise RuntimeError("could not detect the requested attitude excursion")
    first = max(0, int(active[0]) - 2)
    last = min(len(indexes) - 1, int(active[-1]) + 2)
    return float(series["time"][indexes[first]]), float(series["time"][indexes[last]])


def position_excursion(
    series: Dict[str, np.ndarray], begin: float, end: float
) -> float:
    baseline_position, _ = median_pose_window(
        series, max(0.0, begin - 2.5), max(0.0, begin - 0.3)
    )
    mask = (series["time"] >= begin) & (series["time"] <= end)
    return float(np.max(np.linalg.norm(series["position"][mask] - baseline_position, axis=1)))


def fit_vertical_lever(
    raw: Dict[str, np.ndarray], windows: Sequence[Tuple[float, float]]
) -> Dict[str, float]:
    """拟合单一 Z 向杆臂，同时允许每个片段有各自的旋转中心。"""
    rows = []
    observations = []
    sample_count = 0
    parameter_count = 3 * len(windows) + 1
    for segment_index, (begin, end) in enumerate(windows):
        mask = (raw["time"] >= begin) & (raw["time"] <= end)
        for position, quaternion in zip(raw["position"][mask], raw["quaternion"][mask]):
            vertical_axis = quaternion_to_rotation(quaternion)[:, 2]
            for axis in range(3):
                row = np.zeros(parameter_count)
                row[3 * segment_index + axis] = 1.0
                row[-1] = vertical_axis[axis]
                rows.append(row)
                observations.append(position[axis])
            sample_count += 1
    design = np.asarray(rows)
    observation = np.asarray(observations)
    solution, _, _, _ = np.linalg.lstsq(design, observation, rcond=None)
    residual = observation - design @ solution
    residual_vectors = residual.reshape(-1, 3)
    return {
        "z_m": float(solution[-1]),
        "residual_rms_m": float(
            np.sqrt(np.mean(np.sum(residual_vectors * residual_vectors, axis=1)))
        ),
        "samples": sample_count,
    }


def fit_horizontal_lever(
    raw: Dict[str, np.ndarray], begin: float, end: float, fixed_z_m: float
) -> Dict[str, object]:
    """在 yaw 旋转区间拟合机体 X/Y 残余杆臂，Z 固定为实测值。"""
    mask = (raw["time"] >= begin) & (raw["time"] <= end)
    positions = raw["position"][mask]
    quaternions = raw["quaternion"][mask]
    rows = []
    observations = []
    adjusted_positions = []
    for position, quaternion in zip(positions, quaternions):
        rotation = quaternion_to_rotation(quaternion)
        adjusted = position - rotation[:, 2] * fixed_z_m
        adjusted_positions.append(adjusted)
        for axis in range(3):
            row = np.zeros(5)
            row[axis] = 1.0
            row[3] = rotation[axis, 0]
            row[4] = rotation[axis, 1]
            rows.append(row)
            observations.append(adjusted[axis])
    design = np.asarray(rows)
    observation = np.asarray(observations)
    solution, _, _, _ = np.linalg.lstsq(design, observation, rcond=None)
    residual = (observation - design @ solution).reshape(-1, 3)
    adjusted_positions = np.asarray(adjusted_positions)
    centered = adjusted_positions - np.mean(adjusted_positions, axis=0)
    residual_sum = float(np.sum(residual * residual))
    total_sum = float(np.sum(centered * centered))
    lever_xy = solution[3:5]
    return {
        "xy_m": lever_xy.tolist(),
        "norm_m": float(np.linalg.norm(lever_xy)),
        "fixed_z_m": fixed_z_m,
        "residual_rms_m": float(
            np.sqrt(np.mean(np.sum(residual * residual, axis=1)))
        ),
        "explained_fraction": 1.0 - residual_sum / total_sum if total_sum > 0.0 else 0.0,
        "samples": int(len(positions)),
    }


def compensated_candidate_series(
    raw: Dict[str, np.ndarray], lever_body: np.ndarray, installation_yaw: float
) -> Dict[str, np.ndarray]:
    c = math.cos(installation_yaw)
    s = math.sin(installation_yaw)
    rotation_fastlio_body = np.array(
        ((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0))
    )
    lever_fastlio = rotation_fastlio_body @ lever_body
    fixed_quaternion = np.array(
        (
            0.0,
            0.0,
            math.sin(installation_yaw / 2.0),
            math.cos(installation_yaw / 2.0),
        )
    )
    position = np.empty_like(raw["position"])
    for index, (raw_position, raw_quaternion) in enumerate(
        zip(raw["position"], raw["quaternion"])
    ):
        position[index] = (
            raw_position
            - quaternion_to_rotation(raw_quaternion) @ lever_fastlio
        )
    quaternion = quaternion_multiply(
        quaternion_normalize(raw["quaternion"]),
        np.broadcast_to(fixed_quaternion, raw["quaternion"].shape),
    )
    return {
        "record_ns": raw["record_ns"],
        "header_ns": raw["header_ns"],
        "time": raw["time"],
        "position": position,
        "quaternion": quaternion,
    }


def parse_config(path: Path) -> Dict[str, float]:
    values: Dict[str, float] = {}
    for raw_line in path.read_text(encoding="ascii").splitlines():
        line = raw_line.strip()
        if not line or line.startswith("#") or "=" not in line:
            continue
        name, value = line.split("=", 1)
        try:
            values[name] = float(value)
        except ValueError:
            continue
    return values


def load_bag(path: Path):
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    topic_types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    missing = [
        topic
        for topic in (*REQUIRED_POSE_TOPICS, MARKER_TOPIC, STATE_TOPIC, HEALTH_TOPIC)
        if topic not in topic_types
    ]
    if missing:
        raise RuntimeError(f"bag is missing required topics: {', '.join(missing)}")
    message_types = {name: get_message(type_name) for name, type_name in topic_types.items()}

    pose_rows: Dict[str, List[Tuple[int, int, np.ndarray, np.ndarray]]] = {
        topic: [] for topic in POSE_TOPICS
    }
    markers: List[Tuple[int, str]] = []
    states: List[Tuple[int, bool, str]] = []
    health: List[Tuple[int, str]] = []
    diagnostics: List[Tuple[int, Dict[str, str]]] = []
    static_transforms: List[object] = []
    first_record_ns = None
    last_record_ns = None

    while reader.has_next():
        topic, serialized, record_ns = reader.read_next()
        if first_record_ns is None:
            first_record_ns = record_ns
        last_record_ns = record_ns
        if topic not in message_types:
            continue
        if topic in pose_rows:
            message = deserialize_message(serialized, message_types[topic])
            position, quaternion = pose_from_message(message)
            pose_rows[topic].append(
                (record_ns, stamp_ns(message.header), position, quaternion)
            )
        elif topic == MARKER_TOPIC:
            message = deserialize_message(serialized, message_types[topic])
            markers.append((record_ns, message.data))
        elif topic == STATE_TOPIC:
            message = deserialize_message(serialized, message_types[topic])
            states.append((record_ns, bool(message.armed), str(message.mode)))
        elif topic == HEALTH_TOPIC:
            message = deserialize_message(serialized, message_types[topic])
            health.append((record_ns, str(message.data)))
        elif topic == HEALTH_DIAGNOSTIC_TOPIC:
            message = deserialize_message(serialized, message_types[topic])
            if message.status:
                diagnostics.append(
                    (
                        record_ns,
                        {item.key: item.value for item in message.status[0].values},
                    )
                )
        elif topic == TF_STATIC_TOPIC:
            message = deserialize_message(serialized, message_types[topic])
            static_transforms.extend(message.transforms)

    if first_record_ns is None or last_record_ns is None:
        raise RuntimeError("bag contains no messages")

    pose_series: Dict[str, Dict[str, np.ndarray]] = {}
    for topic, rows in pose_rows.items():
        if not rows:
            continue
        record_ns = np.array([row[0] for row in rows], dtype=np.int64)
        header_ns = np.array([row[1] for row in rows], dtype=np.int64)
        pose_series[topic] = {
            "record_ns": record_ns,
            "header_ns": header_ns,
            "time": (record_ns - first_record_ns) / 1e9,
            "position": np.stack([row[2] for row in rows]),
            "quaternion": np.stack([row[3] for row in rows]),
        }

    return {
        "start_ns": first_record_ns,
        "end_ns": last_record_ns,
        "poses": pose_series,
        "markers": [((time_ns - first_record_ns) / 1e9, label) for time_ns, label in markers],
        "states": [((time_ns - first_record_ns) / 1e9, armed, mode) for time_ns, armed, mode in states],
        "health": [((time_ns - first_record_ns) / 1e9, state) for time_ns, state in health],
        "diagnostics": [
            ((time_ns - first_record_ns) / 1e9, values)
            for time_ns, values in diagnostics
        ],
        "static_transforms": static_transforms,
    }


def marker_times(markers: Sequence[Tuple[float, str]]) -> Dict[str, float]:
    output: Dict[str, float] = {}
    for time_s, label in markers:
        if label != "标记通道初始化":
            output[label] = time_s
    return output


def bridge_contract(
    raw: Dict[str, np.ndarray],
    corrected: Dict[str, np.ndarray],
    lever_body: np.ndarray,
    installation_yaw: float,
) -> Dict[str, float]:
    raw_by_stamp = {int(stamp): index for index, stamp in enumerate(raw["header_ns"])}
    pairs = [
        (raw_by_stamp[int(stamp)], index)
        for index, stamp in enumerate(corrected["header_ns"])
        if int(stamp) in raw_by_stamp
    ]
    if not pairs:
        raise RuntimeError("raw and corrected poses have no matching measurement stamps")
    raw_index = np.array([pair[0] for pair in pairs], dtype=int)
    corrected_index = np.array([pair[1] for pair in pairs], dtype=int)
    raw_position = raw["position"][raw_index]
    raw_quaternion = quaternion_normalize(raw["quaternion"][raw_index])
    corrected_position = corrected["position"][corrected_index]
    corrected_quaternion = quaternion_normalize(corrected["quaternion"][corrected_index])

    c = math.cos(installation_yaw)
    s = math.sin(installation_yaw)
    rotation_fastlio_body = np.array(((c, -s, 0.0), (s, c, 0.0), (0.0, 0.0, 1.0)))
    lever_fastlio = rotation_fastlio_body @ lever_body
    expected_position = np.empty_like(raw_position)
    for index, (position, quaternion) in enumerate(zip(raw_position, raw_quaternion)):
        expected_position[index] = position - quaternion_to_rotation(quaternion) @ lever_fastlio
    position_error = np.linalg.norm(corrected_position - expected_position, axis=1)

    fixed_quaternion = np.array(
        (0.0, 0.0, math.sin(installation_yaw / 2.0), math.cos(installation_yaw / 2.0))
    )
    expected_quaternion = quaternion_multiply(
        raw_quaternion, np.broadcast_to(fixed_quaternion, raw_quaternion.shape)
    )
    error_quaternion = quaternion_multiply(
        quaternion_conjugate(expected_quaternion), corrected_quaternion
    )
    attitude_error = quaternion_angle(error_quaternion)
    return {
        "matched_samples": int(len(pairs)),
        "lever_fastlio_m": lever_fastlio.tolist(),
        "position_error_max_m": float(np.max(position_error)),
        "position_error_rms_m": float(np.sqrt(np.mean(position_error**2))),
        "attitude_error_max_deg": float(math.degrees(np.max(attitude_error))),
        "attitude_error_rms_deg": float(
            math.degrees(np.sqrt(np.mean(attitude_error**2)))
        ),
    }


def static_tf_result(transforms: Iterable[object]) -> Dict[str, object]:
    for transform in transforms:
        if transform.header.frame_id == "base_link" and transform.child_frame_id == "body":
            translation = transform.transform.translation
            rotation = transform.transform.rotation
            quaternion = np.array((rotation.x, rotation.y, rotation.z, rotation.w))
            yaw = quaternion_to_euler(quaternion[None, :])[0, 2]
            return {
                "found": True,
                "translation_m": [translation.x, translation.y, translation.z],
                "yaw_deg": math.degrees(yaw),
            }
    return {"found": False}


def fmt_vector(values: Sequence[float]) -> str:
    return "(" + ", ".join(f"{value:+.4f}" for value in values) + ")"


def analyze(path: Path, config_path: Path) -> Dict[str, object]:
    bag = load_bag(path)
    duration_s = (bag["end_ns"] - bag["start_ns"]) / 1e9
    markers = marker_times(bag["markers"])
    required_markers = (
        "静止开始",
        "原地旋转三百六十度",
        "向左平移",
        "向前平移",
        "向上平移",
        "俯仰验证",
        "静止结束",
    )
    missing = [label for label in required_markers if label not in markers]
    if missing:
        raise RuntimeError(f"missing action markers: {', '.join(missing)}")

    expected_order = [markers[label] for label in required_markers]
    if expected_order != sorted(expected_order):
        raise RuntimeError("action markers are not in the requested order")

    poses = bag["poses"]
    raw = poses["/Odometry"]
    corrected = poses["/mavros/vision_pose/pose_cov"]
    px4 = poses.get("/mavros/local_position/pose", poses["/mavros/local_position/odom"])
    config = parse_config(config_path)
    lever_body = np.array(
        (
            config["MID360_BODY_TO_SENSOR_X_M"],
            config["MID360_BODY_TO_SENSOR_Y_M"],
            config["MID360_BODY_TO_SENSOR_Z_M"],
        )
    )
    installation_yaw = config["MID360_BODY_TO_FASTLIO_YAW_RAD"]

    contract = bridge_contract(raw, corrected, lever_body, installation_yaw)
    rotation_end = markers["向左平移"] - 0.3
    rotation_indexes, rotation_delta = find_rotation_window(
        corrected, markers["原地旋转三百六十度"], rotation_end
    )
    rotation_begin = float(corrected["time"][rotation_indexes[0]])
    rotation_finish = float(corrected["time"][rotation_indexes[-1]])
    raw_rotation_mask = (raw["time"] >= rotation_begin) & (raw["time"] <= rotation_finish)
    px4_rotation_mask = (px4["time"] >= rotation_begin) & (px4["time"] <= rotation_finish)

    raw_circle = circle_fit(raw["position"][raw_rotation_mask, :2])
    corrected_circle = circle_fit(corrected["position"][rotation_indexes, :2])
    px4_circle = circle_fit(px4["position"][px4_rotation_mask, :2])
    horizontal_lever_fit = fit_horizontal_lever(
        raw, rotation_begin, rotation_finish, float(lever_body[2])
    )

    left = segment_motion(
        corrected,
        markers["向左平移"],
        markers["向前平移"] - 0.3,
        expected_axis=1,
    )
    forward = segment_motion(
        corrected,
        markers["向前平移"],
        markers["向上平移"] - 0.3,
        expected_axis=0,
    )

    roll_marker_present = "横滚验证" in markers
    roll_search_begin = (
        markers["横滚验证"]
        if roll_marker_present
        else markers["向上平移"] + 3.0
    )
    detected_roll_begin, detected_roll_end = detect_attitude_window(
        corrected,
        roll_search_begin,
        markers["俯仰验证"] - 0.3,
        primary_axis=0,
    )
    roll_begin = (
        markers["横滚验证"]
        if roll_marker_present
        else max(roll_search_begin, detected_roll_begin - 0.15)
    )
    roll_end = min(markers["俯仰验证"] - 0.3, detected_roll_end + 0.45)
    upward = segment_motion(
        corrected,
        markers["向上平移"],
        roll_begin - 0.3,
        expected_axis=2,
    )
    _, detected_pitch_end = detect_attitude_window(
        corrected,
        markers["俯仰验证"],
        markers["静止结束"] - 0.3,
        primary_axis=1,
    )
    pitch_begin = markers["俯仰验证"]
    pitch_end = max(detected_pitch_end, markers["静止结束"] - 1.2)
    roll = attitude_segment(
        corrected, roll_begin, roll_end, primary_axis=0
    )
    pitch = attitude_segment(
        corrected, pitch_begin, pitch_end, primary_axis=1
    )
    roll_relative_raw = relative_rotation_segment(raw, roll_begin, roll_end)
    roll_relative_current = relative_rotation_segment(corrected, roll_begin, roll_end)
    pitch_relative_raw = relative_rotation_segment(raw, pitch_begin, pitch_end)
    pitch_relative_current = relative_rotation_segment(corrected, pitch_begin, pitch_end)

    lever_fit_roll = fit_vertical_lever(raw, [(roll_begin, roll_end)])
    lever_fit_pitch = fit_vertical_lever(raw, [(pitch_begin, pitch_end)])
    lever_fit_combined = fit_vertical_lever(
        raw, [(roll_begin, roll_end), (pitch_begin, pitch_end)]
    )
    fitted_lever = np.array((lever_body[0], lever_body[1], lever_fit_combined["z_m"]))
    fitted_candidate = compensated_candidate_series(raw, fitted_lever, installation_yaw)
    attitude_excursions = {
        "roll_raw_m": position_excursion(raw, roll_begin, roll_end),
        "roll_configured_m": position_excursion(corrected, roll_begin, roll_end),
        "roll_fitted_m": position_excursion(fitted_candidate, roll_begin, roll_end),
        "pitch_raw_m": position_excursion(raw, pitch_begin, pitch_end),
        "pitch_configured_m": position_excursion(corrected, pitch_begin, pitch_end),
        "pitch_fitted_m": position_excursion(fitted_candidate, pitch_begin, pitch_end),
    }

    candidate_results: Dict[str, object] = {}
    for candidate_degrees in (-90, 0, 90):
        candidate = compensated_candidate_series(
            raw, lever_body, math.radians(candidate_degrees)
        )
        candidate_results[str(candidate_degrees)] = {
            "forward": segment_motion(
                candidate,
                markers["向前平移"],
                markers["向上平移"] - 0.3,
                expected_axis=0,
            ),
            "left": segment_motion(
                candidate,
                markers["向左平移"],
                markers["向前平移"] - 0.3,
                expected_axis=1,
            ),
            "rotation_circle": circle_fit(
                candidate["position"][rotation_indexes, :2]
            ),
        }

    static_begin = markers["静止开始"] + 1.0
    static_end = min(markers["原地旋转三百六十度"] - 1.0, static_begin + 10.0)
    static_mask = (corrected["time"] >= static_begin) & (corrected["time"] <= static_end)
    static_position = corrected["position"][static_mask]
    static_std = np.std(static_position, axis=0)
    static_span = np.ptp(static_position, axis=0)
    static_end_begin = markers["静止结束"] + 1.0
    static_end_finish = min(duration_s - 0.3, static_end_begin + 10.0)
    static_end_mask = (
        (corrected["time"] >= static_end_begin)
        & (corrected["time"] <= static_end_finish)
    )
    static_end_position = corrected["position"][static_end_mask]
    static_start_pose, static_start_quaternion = median_pose_window(
        corrected, static_begin, static_end
    )
    static_finish_pose, static_finish_quaternion = median_pose_window(
        corrected, static_end_begin, static_end_finish
    )
    recovery_world_delta = static_finish_pose - static_start_pose
    recovery_body_delta = (
        quaternion_to_rotation(static_start_quaternion).T @ recovery_world_delta
    )
    start_yaw = quaternion_to_euler(static_start_quaternion[None, :])[0, 2]
    finish_yaw = quaternion_to_euler(static_finish_quaternion[None, :])[0, 2]

    corrected_yaw = np.unwrap(
        quaternion_to_euler(corrected["quaternion"][rotation_indexes])[:, 2]
    )
    px4_rotation_yaw = np.unwrap(
        quaternion_to_euler(px4["quaternion"][px4_rotation_mask])[:, 2]
    )
    px4_yaw_delta = float(px4_rotation_yaw[-1] - px4_rotation_yaw[0])

    health_counts: Dict[str, int] = {}
    for _, state in bag["health"]:
        health_counts[state] = health_counts.get(state, 0) + 1
    action_health = [
        state
        for time_s, state in bag["health"]
        if markers["静止开始"] <= time_s <= markers["静止结束"]
    ]
    health_intervals = []
    interval_begin = None
    interval_states = set()
    for index, (time_s, state) in enumerate(bag["health"]):
        if state != "HEALTHY" and interval_begin is None:
            interval_begin = time_s
            interval_states = {state}
        elif state != "HEALTHY":
            interval_states.add(state)
        elif interval_begin is not None:
            health_intervals.append(
                {
                    "begin_s": interval_begin,
                    "end_s": time_s,
                    "states": sorted(interval_states),
                }
            )
            interval_begin = None
            interval_states = set()
    if interval_begin is not None:
        health_intervals.append(
            {
                "begin_s": interval_begin,
                "end_s": duration_s,
                "states": sorted(interval_states),
            }
        )
    for interval in health_intervals:
        raw_reasons = {
            values.get("reason", "")
            for time_s, values in bag["diagnostics"]
            if interval["begin_s"] <= time_s <= interval["end_s"]
        }
        reasons = sorted(
            reason
            for reason in raw_reasons
            if reason and not reason.startswith("recovering (")
        )
        if any(reason.startswith("recovering (") for reason in raw_reasons):
            reasons.append("recovering (2.00 s hysteresis)")
        interval["reasons"] = reasons
    armed_samples = sum(1 for _, armed, _ in bag["states"] if armed)
    modes = sorted({mode for _, _, mode in bag["states"]})

    input_delay = (raw["record_ns"] - raw["header_ns"]) / 1e9
    validation_delay = input_delay[
        (raw["time"] >= markers["静止开始"])
        & (raw["time"] <= markers["静止结束"])
    ]
    expected_horizontal_lever = float(np.linalg.norm(contract["lever_fastlio_m"][:2]))

    candidate_zero_forward = candidate_results["0"]["forward"]["peak_body_delta_m"]
    candidate_zero_left = candidate_results["0"]["left"]["peak_body_delta_m"]
    rotation_pivot_error = abs(raw_circle["radius_m"] - expected_horizontal_lever)
    configured_z_error = abs(float(lever_body[2]) - lever_fit_combined["z_m"])
    verdicts = {
        "bridge_math": contract["position_error_max_m"] < 1e-6
        and contract["attitude_error_max_deg"] < 1e-5,
        "rotation_completed": abs(math.degrees(rotation_delta)) >= 330.0,
        "configured_zero_degree_axis_mapping": forward["expected_axis_peak_m"] > 0.10
        and left["expected_axis_peak_m"] > 0.10
        and upward["expected_axis_peak_m"] > 0.10,
        "zero_degree_axis_candidate": candidate_zero_forward[0] > 0.10
        and abs(candidate_zero_forward[1]) < 0.10
        and candidate_zero_left[1] > 0.10
        and abs(candidate_zero_left[0]) < 0.10,
        "corrected_rotation_radius": corrected_circle["centroid_rms_radius_m"] <= 0.05,
        "configured_vertical_lever_supported": configured_z_error <= 0.03,
        "complete_marker_protocol": roll_marker_present,
        "health_during_marked_actions": set(action_health) == {"HEALTHY"},
        "health_entire_recording": set(health_counts) == {"HEALTHY"},
        "disarmed": armed_samples == 0,
        "non_offboard": "OFFBOARD" not in modes,
        "input_fresh": float(np.percentile(validation_delay, 95.0)) <= 0.25,
    }

    return {
        "bag": str(path),
        "duration_s": duration_s,
        "markers": bag["markers"],
        "bridge_contract": contract,
        "static_tf": static_tf_result(bag["static_transforms"]),
        "static": {
            "window_s": [static_begin, static_end],
            "position_std_m": static_std.tolist(),
            "position_peak_to_peak_m": static_span.tolist(),
            "end_window_s": [static_end_begin, static_end_finish],
            "end_position_std_m": np.std(static_end_position, axis=0).tolist(),
            "end_position_peak_to_peak_m": np.ptp(static_end_position, axis=0).tolist(),
            "recovery_body_delta_m": recovery_body_delta.tolist(),
            "recovery_position_norm_m": float(np.linalg.norm(recovery_world_delta)),
            "recovery_yaw_delta_deg": float(
                math.degrees(circular_difference(finish_yaw, start_yaw))
            ),
        },
        "rotation": {
            "detected_window_s": [rotation_begin, rotation_finish],
            "duration_s": rotation_finish - rotation_begin,
            "corrected_yaw_delta_deg": math.degrees(rotation_delta),
            "px4_ros_enu_yaw_delta_deg": math.degrees(px4_yaw_delta),
            "expected_horizontal_lever_m": expected_horizontal_lever,
            "raw_radius_vs_lever_error_m": rotation_pivot_error,
            "raw_circle": raw_circle,
            "corrected_circle": corrected_circle,
            "px4_circle": px4_circle,
            "horizontal_lever_fit": horizontal_lever_fit,
        },
        "forward": forward,
        "left": left,
        "upward": upward,
        "installation_candidates": candidate_results,
        "roll": roll,
        "pitch": pitch,
        "attitude_windows": {
            "roll_s": [roll_begin, roll_end],
            "roll_marker_present": roll_marker_present,
            "pitch_s": [pitch_begin, pitch_end],
        },
        "vertical_lever_fit": {
            "configured_z_m": float(lever_body[2]),
            "configured_error_vs_combined_fit_m": configured_z_error,
            "roll": lever_fit_roll,
            "pitch": lever_fit_pitch,
            "combined": lever_fit_combined,
            "position_excursions": attitude_excursions,
        },
        "relative_axes": {
            "roll_raw_fastlio": roll_relative_raw,
            "roll_configured_output": roll_relative_current,
            "pitch_raw_fastlio": pitch_relative_raw,
            "pitch_configured_output": pitch_relative_current,
        },
        "health": {
            "counts": health_counts,
            "marked_action_states": sorted(set(action_health)),
            "nonhealthy_intervals": health_intervals,
            "armed_samples": armed_samples,
            "modes": modes,
        },
        "timing": {
            "odometry_samples": int(len(raw["time"])),
            "odometry_rate_hz": float((len(raw["time"]) - 1) / np.ptp(raw["time"])),
            "input_delay_mean_s": float(np.mean(input_delay)),
            "input_delay_p95_s": float(np.percentile(input_delay, 95.0)),
            "input_delay_max_s": float(np.max(input_delay)),
            "validation_delay_p95_s": float(
                np.percentile(validation_delay, 95.0)
            ),
            "validation_delay_max_s": float(np.max(validation_delay)),
        },
        "verdicts": verdicts,
        "all_automatic_checks_pass": all(verdicts.values()),
    }


def markdown_report(result: Dict[str, object]) -> str:
    contract = result["bridge_contract"]
    rotation = result["rotation"]
    static = result["static"]
    forward = result["forward"]
    left = result["left"]
    upward = result["upward"]
    roll = result["roll"]
    pitch = result["pitch"]
    attitude_windows = result["attitude_windows"]
    lever_fit = result["vertical_lever_fit"]
    relative_axes = result["relative_axes"]
    candidates = result["installation_candidates"]
    health = result["health"]
    timing = result["timing"]
    verdicts = result["verdicts"]

    lines = [
        "# MID360 轴向手持验证报告",
        "",
        f"- Bag: `{result['bag']}`",
        f"- 时长: `{result['duration_s']:.3f} s`",
        f"- 自动检查: `{'PASS' if result['all_automatic_checks_pass'] else 'FAIL'}`",
        "",
        "## 标记",
        "",
    ]
    lines.extend(f"- `{time_s:.3f} s`: {label}" for time_s, label in result["markers"])
    lines.extend(
        [
            "",
            "## 桥接数学",
            "",
            f"- FAST-LIO 坐标杆臂: `{fmt_vector(contract['lever_fastlio_m'])} m`",
            f"- 匹配样本: `{contract['matched_samples']}`",
            f"- 位置公式最大误差: `{contract['position_error_max_m']:.9f} m`",
            f"- 姿态右乘最大误差: `{contract['attitude_error_max_deg']:.9f} deg`",
            f"- 判定: `{'PASS' if verdicts['bridge_math'] else 'FAIL'}`",
            "",
            "## 静止段",
            "",
            f"- 分析窗口: `{static['window_s'][0]:.3f}-{static['window_s'][1]:.3f} s`",
            f"- XYZ 标准差: `{fmt_vector(static['position_std_m'])} m`",
            f"- XYZ 峰峰值: `{fmt_vector(static['position_peak_to_peak_m'])} m`",
            f"- 结束静止 XYZ 标准差: `{fmt_vector(static['end_position_std_m'])} m`",
            f"- 开始到结束机体系位置差: `{fmt_vector(static['recovery_body_delta_m'])} m`",
            f"- 开始到结束位置差模长: `{static['recovery_position_norm_m']:.4f} m`",
            f"- 开始到结束 yaw 差: `{static['recovery_yaw_delta_deg']:+.3f} deg`",
            "",
            "## 360 度旋转",
            "",
            f"- 自动检测窗口: `{rotation['detected_window_s'][0]:.3f}-{rotation['detected_window_s'][1]:.3f} s`",
            f"- 修正后 ROS yaw 变化: `{rotation['corrected_yaw_delta_deg']:+.3f} deg`",
            f"- PX4 经 MAVROS 输出的 ENU yaw 变化: `{rotation['px4_ros_enu_yaw_delta_deg']:+.3f} deg`",
            f"- 理论水平杆臂: `{rotation['expected_horizontal_lever_m']:.4f} m`",
            f"- 原始 FAST-LIO 圆拟合半径: `{rotation['raw_circle']['radius_m']:.4f} m`",
            f"- 原始轨迹质心 RMS 半径: `{rotation['raw_circle']['centroid_rms_radius_m']:.4f} m`",
            f"- 杆臂修正后圆拟合半径: `{rotation['corrected_circle']['radius_m']:.4f} m`",
            f"- 杆臂修正后质心 RMS 半径: `{rotation['corrected_circle']['centroid_rms_radius_m']:.4f} m`",
            f"- PX4 位置质心 RMS 半径: `{rotation['px4_circle']['centroid_rms_radius_m']:.4f} m`",
            f"- 原始半径与实测水平杆臂之差: `{rotation['raw_radius_vs_lever_error_m']:.4f} m`",
            f"- 固定当前 Z 后反求水平杆臂 XY: `{fmt_vector(rotation['horizontal_lever_fit']['xy_m'])} m`",
            f"- 反求水平杆臂长度: `{rotation['horizontal_lever_fit']['norm_m']:.4f} m`",
            f"- 旋转模型解释比例: `{100.0 * rotation['horizontal_lever_fit']['explained_fraction']:.1f}%`",
            f"- 旋转模型残差 RMS: `{rotation['horizontal_lever_fit']['residual_rms_m']:.4f} m`",
            f"- <=0.05 m 判定: `{'PASS' if verdicts['corrected_rotation_radius'] else 'FAIL'}`",
            "",
            "## 平移轴向",
            "",
            f"- 左移峰值机体系位移: `{fmt_vector(left['peak_body_delta_m'])} m`",
            f"- 左移段终点机体系位移: `{fmt_vector(left['endpoint_body_delta_m'])} m`",
            f"- 前进峰值机体系位移: `{fmt_vector(forward['peak_body_delta_m'])} m`",
            f"- 前进段终点机体系位移: `{fmt_vector(forward['endpoint_body_delta_m'])} m`",
            f"- 上移峰值机体系位移: `{fmt_vector(upward['peak_body_delta_m'])} m`",
            f"- 当前 0 deg XYZ 轴向: `{'PASS' if verdicts['configured_zero_degree_axis_mapping'] else 'FAIL'}`",
            f"- 0 deg 候选前进终点: `{fmt_vector(candidates['0']['forward']['endpoint_body_delta_m'])} m`",
            f"- 0 deg 候选左移终点: `{fmt_vector(candidates['0']['left']['endpoint_body_delta_m'])} m`",
            f"- 0 deg 候选轴向: `{'PASS' if verdicts['zero_degree_axis_candidate'] else 'FAIL'}`",
            "",
            "## 横滚与俯仰",
            "",
            f"- 横滚窗口: `{attitude_windows['roll_s'][0]:.3f}-{attitude_windows['roll_s'][1]:.3f} s`",
            f"- 横滚标记: `{'已记录' if attitude_windows['roll_marker_present'] else '缺失；窗口由姿态数据推断'}`",
            f"- 横滚范围: `{roll['primary_range_deg']:.3f} deg`",
            f"- 横滚段 yaw 范围: `{roll['yaw_range_deg']:.3f} deg`",
            f"- 横滚段水平/垂直最大位置偏移: `{roll['horizontal_excursion_m']:.4f} / {roll['vertical_excursion_m']:.4f} m`",
            f"- 俯仰范围: `{pitch['primary_range_deg']:.3f} deg`",
            f"- 俯仰窗口: `{attitude_windows['pitch_s'][0]:.3f}-{attitude_windows['pitch_s'][1]:.3f} s`",
            f"- 俯仰段 yaw 范围: `{pitch['yaw_range_deg']:.3f} deg`",
            f"- 俯仰段水平/垂直最大位置偏移: `{pitch['horizontal_excursion_m']:.4f} / {pitch['vertical_excursion_m']:.4f} m`",
            f"- 原始 FAST-LIO 横滚相对 XYZ 范围: `{fmt_vector(relative_axes['roll_raw_fastlio']['xyz_range_deg'])} deg`",
            f"- 当前 0 deg 输出横滚相对 XYZ 范围: `{fmt_vector(relative_axes['roll_configured_output']['xyz_range_deg'])} deg`",
            f"- 原始 FAST-LIO 俯仰相对 XYZ 范围: `{fmt_vector(relative_axes['pitch_raw_fastlio']['xyz_range_deg'])} deg`",
            f"- 当前 0 deg 输出俯仰相对 XYZ 范围: `{fmt_vector(relative_axes['pitch_configured_output']['xyz_range_deg'])} deg`",
            "",
            "## 垂直杆臂拟合",
            "",
            f"- 录包时配置 Z: `{lever_fit['configured_z_m']:.5f} m`",
            f"- 横滚拟合 Z: `{lever_fit['roll']['z_m']:.5f} m`，残差 RMS `{lever_fit['roll']['residual_rms_m']:.4f} m`",
            f"- 俯仰拟合 Z: `{lever_fit['pitch']['z_m']:.5f} m`，残差 RMS `{lever_fit['pitch']['residual_rms_m']:.4f} m`",
            f"- 合并拟合 Z: `{lever_fit['combined']['z_m']:.5f} m`，残差 RMS `{lever_fit['combined']['residual_rms_m']:.4f} m`",
            f"- 配置值与合并拟合差: `{lever_fit['configured_error_vs_combined_fit_m']:.5f} m`",
            f"- 横滚位置最大偏移 raw/configured/fitted: `{lever_fit['position_excursions']['roll_raw_m']:.4f} / {lever_fit['position_excursions']['roll_configured_m']:.4f} / {lever_fit['position_excursions']['roll_fitted_m']:.4f} m`",
            f"- 俯仰位置最大偏移 raw/configured/fitted: `{lever_fit['position_excursions']['pitch_raw_m']:.4f} / {lever_fit['position_excursions']['pitch_configured_m']:.4f} / {lever_fit['position_excursions']['pitch_fitted_m']:.4f} m`",
            f"- 当前配置杆臂判定: `{'PASS' if verdicts['configured_vertical_lever_supported'] else 'FAIL'}`",
            "",
            "## 健康与安全",
            "",
            f"- EV 状态计数: `{health['counts']}`",
            f"- 标记动作期间 EV 状态: `{health['marked_action_states']}`",
            f"- armed=true 样本: `{health['armed_samples']}`",
            f"- 模式: `{health['modes']}`",
            f"- Odom 频率: `{timing['odometry_rate_hz']:.3f} Hz`",
            f"- Odom 延迟 mean/p95/max: `{timing['input_delay_mean_s']:.4f} / {timing['input_delay_p95_s']:.4f} / {timing['input_delay_max_s']:.4f} s`",
            f"- 验证动作期间延迟 p95/max: `{timing['validation_delay_p95_s']:.4f} / {timing['validation_delay_max_s']:.4f} s`",
            "",
            "### 非健康区间",
            "",
        ]
    )
    if health["nonhealthy_intervals"]:
        lines.extend(
            f"- `{interval['begin_s']:.3f}-{interval['end_s']:.3f} s`: "
            f"states={interval['states']}, reasons={interval['reasons']}"
            for interval in health["nonhealthy_intervals"]
        )
    else:
        lines.append("- 无")
    lines.extend(
        [
            "",
            "## 自动判定",
            "",
        ]
    )
    lines.extend(
        f"- {name}: `{'PASS' if passed else 'FAIL'}`"
        for name, passed in verdicts.items()
    )
    lines.append("")
    return "\n".join(lines)


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("bag", type=Path)
    parser.add_argument(
        "--config",
        type=Path,
    )
    parser.add_argument("--output", type=Path)
    args = parser.parse_args()

    config_path = args.config
    if config_path is None:
        bag_snapshot = args.bag / "analysis_config.conf"
        config_path = (
            bag_snapshot
            if bag_snapshot.exists()
            else Path("src/px4_ros_com/config/mid360_lever_arm.conf")
        )
    result = analyze(args.bag, config_path)
    report = markdown_report(result)
    if args.output:
        args.output.write_text(report, encoding="utf-8")
        json_path = args.output.with_suffix(".json")
        json_path.write_text(
            json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
        )
        print(args.output)
        print(json_path)
    else:
        print(report)
    return 0 if result["all_automatic_checks_pass"] else 2


if __name__ == "__main__":
    raise SystemExit(main())
