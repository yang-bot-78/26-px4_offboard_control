#!/usr/bin/env python3
"""Independent read-only analysis for the 2026-07-24 high-rate EV bags."""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path
import statistics
from typing import Any

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


INTEREST = {
    "/Odometry",
    "/Odometry/propagated",
    "/high_rate_ev/diagnostics",
    "/livox/imu",
    "/phase4/raw/imu",
    "/phase4_validation/marker",
}


def stamp_ns(message: Any) -> int:
    return int(message.header.stamp.sec) * 1_000_000_000 + int(
        message.header.stamp.nanosec
    )


def odometry_record(message: Any, receive_ns: int) -> dict[str, Any]:
    pose = message.pose.pose
    twist = message.twist.twist
    return {
        "receive_ns": receive_ns,
        "stamp_ns": stamp_ns(message),
        "frame_id": message.header.frame_id,
        "child_frame_id": message.child_frame_id,
        "position": [pose.position.x, pose.position.y, pose.position.z],
        "quaternion": [
            pose.orientation.x,
            pose.orientation.y,
            pose.orientation.z,
            pose.orientation.w,
        ],
        "linear": [twist.linear.x, twist.linear.y, twist.linear.z],
        "angular": [twist.angular.x, twist.angular.y, twist.angular.z],
        "pose_covariance": list(message.pose.covariance),
        "twist_covariance": list(message.twist.covariance),
    }


def diagnostic_record(message: Any, receive_ns: int) -> dict[str, Any]:
    if not message.status:
        return {"receive_ns": receive_ns, "state": "EMPTY", "values": {}}
    status = message.status[0]
    return {
        "receive_ns": receive_ns,
        "header_ns": stamp_ns(message),
        "state": status.message,
        "level": int(status.level[0]) if isinstance(status.level, bytes) else int(status.level),
        "values": {item.key: item.value for item in status.values},
    }


def read_bag(path: Path) -> dict[str, Any]:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(path), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    type_map = {item.name: item.type for item in reader.get_all_topics_and_types()}
    message_types = {
        topic: get_message(type_map[topic]) for topic in INTEREST if topic in type_map
    }
    result: dict[str, Any] = {
        "counts": {},
        "odometry": {},
        "diagnostics": [],
        "imu_stamps": {},
        "markers": {},
    }
    while reader.has_next():
        topic, serialized, receive_ns = reader.read_next()
        result["counts"][topic] = result["counts"].get(topic, 0) + 1
        if topic not in message_types:
            continue
        message = deserialize_message(serialized, message_types[topic])
        if topic in ("/Odometry", "/Odometry/propagated"):
            result["odometry"].setdefault(topic, []).append(
                odometry_record(message, receive_ns)
            )
        elif topic == "/high_rate_ev/diagnostics":
            result["diagnostics"].append(diagnostic_record(message, receive_ns))
        elif topic in ("/livox/imu", "/phase4/raw/imu"):
            result["imu_stamps"].setdefault(topic, []).append(stamp_ns(message))
        elif topic == "/phase4_validation/marker":
            result["markers"][message.data] = receive_ns
    return result


def percentile(values: list[float], fraction: float) -> float:
    if not values:
        return math.nan
    return float(np.percentile(np.asarray(values, dtype=float), fraction * 100.0))


def timestamp_membership(
    query_stamps: list[int], reference_stamps: list[int]
) -> dict[str, Any]:
    if not query_stamps or not reference_stamps:
        return {"queries": len(query_stamps), "references": len(reference_stamps)}
    reference = np.asarray(sorted(set(reference_stamps)), dtype=np.int64)
    query = np.asarray(query_stamps, dtype=np.int64)
    indices = np.searchsorted(reference, query)
    absolute_differences = []
    for value, index in zip(query, indices):
        candidates = []
        if index < len(reference):
            candidates.append(abs(int(reference[index]) - int(value)))
        if index > 0:
            candidates.append(abs(int(reference[index - 1]) - int(value)))
        absolute_differences.append(min(candidates))
    return {
        "queries": len(query_stamps),
        "references": len(reference_stamps),
        "exact_matches": sum(value == 0 for value in absolute_differences),
        "nearest_absolute_difference_ns": {
            "p50": percentile(absolute_differences, 0.50),
            "p95": percentile(absolute_differences, 0.95),
            "max": max(absolute_differences),
        },
        "within_1us": sum(value <= 1000 for value in absolute_differences),
    }


def timing(records: list[dict[str, Any]]) -> dict[str, Any]:
    if len(records) < 2:
        return {"count": len(records)}
    receive = np.asarray([item["receive_ns"] for item in records], dtype=np.int64)
    stamps = np.asarray([item["stamp_ns"] for item in records], dtype=np.int64)
    receive_dt = np.diff(receive) * 1e-9
    source_dt = np.diff(stamps) * 1e-9
    ages = (receive - stamps) * 1e-9
    return {
        "count": len(records),
        "receive_rate_hz": float((len(receive) - 1) / ((receive[-1] - receive[0]) * 1e-9)),
        "source_rate_hz": float((len(stamps) - 1) / ((stamps[-1] - stamps[0]) * 1e-9)),
        "duplicate_or_backward": int(np.count_nonzero(source_dt <= 0.0)),
        "receive_interval_ms": {
            "p50": percentile((receive_dt * 1000.0).tolist(), 0.50),
            "p95": percentile((receive_dt * 1000.0).tolist(), 0.95),
            "max": float(np.max(receive_dt) * 1000.0),
            "std": float(np.std(receive_dt) * 1000.0),
        },
        "source_age_ms": {
            "p50": percentile((ages * 1000.0).tolist(), 0.50),
            "p95": percentile((ages * 1000.0).tolist(), 0.95),
            "max": float(np.max(ages) * 1000.0),
            "min": float(np.min(ages) * 1000.0),
        },
    }


def covariance(records: list[dict[str, Any]]) -> dict[str, Any]:
    pose_nonfinite = 0
    twist_nonfinite = 0
    pose_symmetry = 0.0
    twist_symmetry = 0.0
    pose_min_eigenvalue = math.inf
    twist_min_eigenvalue = math.inf
    quaternion_error = 0.0
    angular_not_all_nan = 0
    angular_variance_bad = 0
    cross_block_bad = 0
    distinct_pose: set[tuple[float, ...]] = set()
    distinct_linear: set[tuple[float, ...]] = set()
    for item in records:
        quaternion_error = max(
            quaternion_error,
            abs(float(np.linalg.norm(np.asarray(item["quaternion"]))) - 1.0),
        )
        if not all(math.isnan(value) for value in item["angular"]):
            angular_not_all_nan += 1
        pose = np.asarray(item["pose_covariance"], dtype=float).reshape(6, 6)
        twist = np.asarray(item["twist_covariance"], dtype=float).reshape(6, 6)
        if not np.isfinite(pose).all():
            pose_nonfinite += 1
        else:
            pose_symmetry = max(pose_symmetry, float(np.max(np.abs(pose - pose.T))))
            pose_min_eigenvalue = min(
                pose_min_eigenvalue, float(np.min(np.linalg.eigvalsh((pose + pose.T) / 2.0)))
            )
            distinct_pose.add(tuple(np.round(pose.flatten(), 12)))
        if not np.isfinite(twist).all():
            twist_nonfinite += 1
        else:
            twist_symmetry = max(
                twist_symmetry, float(np.max(np.abs(twist - twist.T)))
            )
            twist_min_eigenvalue = min(
                twist_min_eigenvalue,
                float(np.min(np.linalg.eigvalsh((twist + twist.T) / 2.0))),
            )
            distinct_linear.add(tuple(np.round(twist[:3, :3].flatten(), 12)))
        if any(abs(twist[index, index] - 1e6) > 1e-6 for index in range(3, 6)):
            angular_variance_bad += 1
        cross = np.concatenate((twist[:3, 3:].flatten(), twist[3:, :3].flatten()))
        if np.max(np.abs(cross)) > 1e-12:
            cross_block_bad += 1
    return {
        "samples": len(records),
        "quaternion_max_norm_error": quaternion_error,
        "pose_nonfinite": pose_nonfinite,
        "twist_covariance_nonfinite": twist_nonfinite,
        "pose_max_symmetry_error": pose_symmetry,
        "twist_max_symmetry_error": twist_symmetry,
        "pose_min_eigenvalue": pose_min_eigenvalue,
        "twist_min_eigenvalue": twist_min_eigenvalue,
        "angular_velocity_not_all_nan": angular_not_all_nan,
        "angular_variance_not_1e6": angular_variance_bad,
        "linear_angular_cross_block_nonzero": cross_block_bad,
        "distinct_pose_covariances": len(distinct_pose),
        "distinct_linear_covariances": len(distinct_linear),
    }


def quaternion_matrix(quaternion: list[float]) -> np.ndarray:
    x, y, z, w = quaternion
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.asarray(
        [
            [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
            [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
            [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)],
        ]
    )


def between(
    records: list[dict[str, Any]], start_ns: int, end_ns: int
) -> list[dict[str, Any]]:
    return [item for item in records if start_ns <= item["receive_ns"] <= end_ns]


def motion_summary(
    records: list[dict[str, Any]], markers: dict[str, int]
) -> dict[str, Any]:
    result: dict[str, Any] = {}
    for label in (
        "FORWARD",
        "BACKWARD",
        "LEFT",
        "RIGHT",
        "UP",
        "DOWN",
        "YAW90",
        "YAW90_FORWARD",
    ):
        start = markers.get(f"{label}_START")
        stop = markers.get(f"{label}_STOPPED")
        if start is None or stop is None:
            result[label] = {"missing": True}
            continue
        samples = between(records, start, stop)
        if len(samples) < 2:
            result[label] = {"samples": len(samples)}
            continue
        world_delta = np.asarray(samples[-1]["position"]) - np.asarray(
            samples[0]["position"]
        )
        body_delta = quaternion_matrix(samples[0]["quaternion"]).T @ world_delta
        mean_twist = np.mean(
            np.asarray([item["linear"] for item in samples], dtype=float), axis=0
        )
        q0 = np.asarray(samples[0]["quaternion"], dtype=float)
        q1 = np.asarray(samples[-1]["quaternion"], dtype=float)
        dot = min(1.0, max(-1.0, abs(float(np.dot(q0, q1)))))
        angle_deg = math.degrees(2.0 * math.acos(dot))
        result[label] = {
            "samples": len(samples),
            "world_position_delta_m": world_delta.tolist(),
            "start_body_position_delta_m": body_delta.tolist(),
            "mean_body_twist_mps": mean_twist.tolist(),
            "attitude_change_deg": angle_deg,
        }
    return result


def velocity_consistency(records: list[dict[str, Any]]) -> dict[str, Any]:
    derived: list[np.ndarray] = []
    reported: list[np.ndarray] = []
    times: list[float] = []
    for left, right in zip(records, records[1:]):
        dt = (right["stamp_ns"] - left["stamp_ns"]) * 1e-9
        if not 0.005 < dt < 0.2:
            continue
        world_velocity = (
            np.asarray(right["position"]) - np.asarray(left["position"])
        ) / dt
        body_velocity = quaternion_matrix(left["quaternion"]).T @ world_velocity
        derived.append(body_velocity)
        reported.append(
            (np.asarray(left["linear"]) + np.asarray(right["linear"])) / 2.0
        )
        times.append(dt)
    if len(derived) < 10:
        return {"samples": len(derived)}
    derived_array = np.asarray(derived)
    reported_array = np.asarray(reported)
    output: dict[str, Any] = {"samples": len(derived)}
    correlations = []
    for axis in range(3):
        if np.std(derived_array[:, axis]) == 0 or np.std(reported_array[:, axis]) == 0:
            correlations.append(math.nan)
        else:
            correlations.append(
                float(np.corrcoef(derived_array[:, axis], reported_array[:, axis])[0, 1])
            )
    output["zero_lag_correlation_xyz"] = correlations
    best = []
    median_dt_ms = statistics.median(times) * 1000.0
    for axis in range(3):
        best_corr = -math.inf
        best_shift = 0
        for shift in range(-4, 5):
            if shift < 0:
                left_values = derived_array[-shift:, axis]
                right_values = reported_array[:shift, axis]
            elif shift > 0:
                left_values = derived_array[:-shift, axis]
                right_values = reported_array[shift:, axis]
            else:
                left_values = derived_array[:, axis]
                right_values = reported_array[:, axis]
            if (
                len(left_values) < 10
                or np.std(left_values) == 0
                or np.std(right_values) == 0
            ):
                continue
            correlation = float(np.corrcoef(left_values, right_values)[0, 1])
            if correlation > best_corr:
                best_corr = correlation
                best_shift = shift
        best.append(
            {
                "correlation": best_corr,
                "lag_ms": best_shift * median_dt_ms,
                "shift_samples": best_shift,
            }
        )
    output["best_correlation_and_lag_xyz"] = best
    return output


def windowed_velocity_consistency(
    records: list[dict[str, Any]], window_s: float
) -> dict[str, Any]:
    timestamps = np.asarray([item["stamp_ns"] for item in records], dtype=np.int64)
    times = (timestamps - timestamps[0]) * 1e-9
    positions = np.asarray([item["position"] for item in records], dtype=float)
    reported_all = np.asarray([item["linear"] for item in records], dtype=float)
    half_window = window_s / 2.0
    sample_times = []
    derived = []
    reported = []
    for index, center in enumerate(times):
        left_time = center - half_window
        right_time = center + half_window
        if left_time < times[0] or right_time > times[-1]:
            continue
        left_index = int(np.searchsorted(times, left_time))
        right_index = int(np.searchsorted(times, right_time))
        low = max(0, left_index - 1)
        high = min(len(times) - 1, right_index)
        if high <= low or np.max(np.diff(times[low : high + 1])) > 0.12:
            continue
        left_position = np.asarray(
            [np.interp(left_time, times, positions[:, axis]) for axis in range(3)]
        )
        right_position = np.asarray(
            [np.interp(right_time, times, positions[:, axis]) for axis in range(3)]
        )
        world_velocity = (right_position - left_position) / window_s
        body_velocity = (
            quaternion_matrix(records[index]["quaternion"]).T @ world_velocity
        )
        sample_times.append(center)
        derived.append(body_velocity)
        reported.append(reported_all[index])
    sample_times_array = np.asarray(sample_times)
    derived_array = np.asarray(derived)
    reported_array = np.asarray(reported)
    output: dict[str, Any] = {
        "window_ms": window_s * 1000.0,
        "samples": len(derived),
    }
    zero_lag = []
    best = []
    for axis in range(3):
        zero_lag.append(
            float(np.corrcoef(derived_array[:, axis], reported_array[:, axis])[0, 1])
        )
        best_correlation = -math.inf
        best_lag = math.nan
        for lag_s in np.arange(-0.100, 0.1001, 0.005):
            shifted_times = sample_times_array + lag_s
            valid = (shifted_times >= times[0]) & (shifted_times <= times[-1])
            shifted_reported = np.interp(
                shifted_times[valid], times, reported_all[:, axis]
            )
            correlation = float(
                np.corrcoef(derived_array[valid, axis], shifted_reported)[0, 1]
            )
            if correlation > best_correlation:
                best_correlation = correlation
                best_lag = float(lag_s)
        best.append(
            {"correlation": best_correlation, "lag_ms": best_lag * 1000.0}
        )
    output["zero_lag_correlation_xyz"] = zero_lag
    output["best_correlation_and_lag_xyz"] = best
    return output


def diagnostic_summary(
    diagnostics: list[dict[str, Any]],
    propagated: list[dict[str, Any]],
    imu_stamps: list[int],
) -> dict[str, Any]:
    transitions = []
    previous = None
    for item in diagnostics:
        current = (item["state"], item["values"].get("fault_reason", ""))
        if current != previous:
            transitions.append(
                {
                    "receive_ns": item["receive_ns"],
                    "state": current[0],
                    "fault_reason": current[1],
                }
            )
            previous = current
    integer_keys = (
        "last_candidate_sequence",
        "last_candidate_timestamp_ns",
        "last_published_sequence",
        "last_published_timestamp_ns",
    )
    parsed = []
    for item in diagnostics:
        try:
            parsed.append(
                {
                    key: int(item["values"].get(key, "0")) for key in integer_keys
                }
            )
        except ValueError:
            pass

    def unique_pairs(sequence_key: str, timestamp_key: str) -> list[tuple[int, int]]:
        pairs = []
        for item in parsed:
            pair = (item[sequence_key], item[timestamp_key])
            if pair != (0, 0) and (not pairs or pair != pairs[-1]):
                pairs.append(pair)
        return pairs

    candidates = unique_pairs("last_candidate_sequence", "last_candidate_timestamp_ns")
    published = unique_pairs("last_published_sequence", "last_published_timestamp_ns")
    imu_set = set(imu_stamps)
    propagated_set = {item["stamp_ns"] for item in propagated}
    float_keys = (
        "correction_age_s",
        "imu_age_s",
        "lidar_age_s",
        "imu_buffer_span_s",
        "worker_last_processing_s",
        "worker_max_processing_s",
    )
    maxima: dict[str, float] = {}
    for key in float_keys:
        values = []
        for item in diagnostics:
            try:
                value = float(item["values"].get(key, "nan"))
            except ValueError:
                continue
            if math.isfinite(value):
                values.append(value)
        maxima[key] = max(values) if values else math.nan
    counter_keys = (
        "worker_queue_depth",
        "worker_queue_max_depth",
        "publish_count",
        "reject_count",
        "drop_count",
        "supersede_count",
        "timeout_count",
    )
    max_counters: dict[str, int] = {}
    for key in counter_keys:
        values = []
        for item in diagnostics:
            try:
                values.append(int(item["values"].get(key, "0")))
            except ValueError:
                pass
        max_counters[key] = max(values) if values else 0
    return {
        "samples": len(diagnostics),
        "transitions": transitions,
        "candidate_updates": len(candidates),
        "candidate_non_strict_updates": sum(
            1
            for left, right in zip(candidates, candidates[1:])
            if right[0] <= left[0] or right[1] <= left[1]
        ),
        "candidate_timestamps_missing_from_imu": sum(
            1 for _, timestamp in candidates if timestamp not in imu_set
        ),
        "candidate_imu_membership": timestamp_membership(
            [timestamp for _, timestamp in candidates], imu_stamps
        ),
        "published_updates": len(published),
        "published_non_strict_updates": sum(
            1
            for left, right in zip(published, published[1:])
            if right[0] <= left[0] or right[1] <= left[1]
        ),
        "published_timestamps_missing_from_bag": sum(
            1 for _, timestamp in published if timestamp not in propagated_set
        ),
        "maxima": maxima,
        "max_counters": max_counters,
    }


def static_summary(
    records: list[dict[str, Any]], markers: dict[str, int], prefix: str
) -> dict[str, Any]:
    start = markers.get(f"{prefix}_START")
    end = markers.get(f"{prefix}_END")
    if start is None or end is None:
        return {"missing_markers": True}
    samples = between(records, start, end)
    norms = [float(np.linalg.norm(np.asarray(item["linear"]))) for item in samples]
    return {
        "samples": len(samples),
        "speed_norm_mps": {
            "p50": percentile(norms, 0.50),
            "p95": percentile(norms, 0.95),
            "max": max(norms) if norms else math.nan,
        },
        "mean_body_velocity_mps": (
            np.mean(np.asarray([item["linear"] for item in samples]), axis=0).tolist()
            if samples
            else []
        ),
    }


def resource_csv(path: Path) -> dict[str, Any]:
    rows = []
    with path.open(encoding="utf-8") as stream:
        next(stream)
        for line in stream:
            fields = line.rstrip("\n").split(",")
            if len(fields) != 6:
                continue
            rows.append(
                {
                    "threads": int(fields[3]),
                    "rss_kib": int(fields[4]),
                    "vsz_kib": int(fields[5]),
                }
            )
    first_60 = rows[:60]
    return {
        "samples": len(rows),
        "threads_min": min(item["threads"] for item in rows),
        "threads_max": max(item["threads"] for item in rows),
        "rss_kib_min": min(item["rss_kib"] for item in rows),
        "rss_kib_mean": statistics.fmean(item["rss_kib"] for item in rows),
        "rss_kib_max": max(item["rss_kib"] for item in rows),
        "rss_growth_kib": rows[-1]["rss_kib"] - rows[0]["rss_kib"],
        "first_60s_rss_kib_mean": statistics.fmean(
            item["rss_kib"] for item in first_60
        ),
        "first_60s_rss_kib_max": max(item["rss_kib"] for item in first_60),
        "first_60s_rss_growth_kib": (
            first_60[-1]["rss_kib"] - first_60[0]["rss_kib"]
        ),
    }


def pidstat(path: Path) -> dict[str, Any]:
    cpu = []
    rss = []
    for line in path.read_text(encoding="utf-8").splitlines():
        fields = line.split()
        if len(fields) < 16 or fields[3] != "-" or fields[-1] != "fastlio_mapping":
            continue
        try:
            cpu.append(float(fields[8]))
            rss.append(int(fields[13]))
        except ValueError:
            continue
    first_60_cpu = cpu[:60]
    return {
        "samples": len(cpu),
        "cpu_percent_mean": statistics.fmean(cpu) if cpu else math.nan,
        "cpu_percent_p95": percentile(cpu, 0.95),
        "cpu_percent_max": max(cpu) if cpu else math.nan,
        "rss_kib_mean": statistics.fmean(rss) if rss else math.nan,
        "rss_kib_max": max(rss) if rss else math.nan,
        "first_60s_cpu_percent_mean": (
            statistics.fmean(first_60_cpu) if first_60_cpu else math.nan
        ),
        "first_60s_cpu_percent_p95": percentile(first_60_cpu, 0.95),
    }


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("session", type=Path)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    bags = {
        "A": read_bag(args.session / "bags/A_default_off"),
        "B": read_bag(args.session / "bags/B_enabled_motion"),
        "C": read_bag(args.session / "bags/C_fault_recovery"),
    }
    summary: dict[str, Any] = {"bags": {}}
    for label, bag in bags.items():
        bag_summary: dict[str, Any] = {"counts": bag["counts"]}
        for topic, records in bag["odometry"].items():
            bag_summary.setdefault("timing", {})[topic] = timing(records)
            bag_summary.setdefault("frames", {})[topic] = {
                "frame_id": sorted({item["frame_id"] for item in records}),
                "child_frame_id": sorted(
                    {item["child_frame_id"] for item in records}
                ),
            }
        if "/Odometry/propagated" in bag["odometry"]:
            propagated = bag["odometry"]["/Odometry/propagated"]
            bag_summary["covariance"] = covariance(propagated)
            canonical_imu = bag["imu_stamps"].get("/livox/imu", [])
            bag_summary["propagated_stamps_missing_from_imu"] = sum(
                1 for item in propagated if item["stamp_ns"] not in set(canonical_imu)
            )
            bag_summary["propagated_imu_membership"] = timestamp_membership(
                [item["stamp_ns"] for item in propagated], canonical_imu
            )
            bag_summary["diagnostics"] = diagnostic_summary(
                bag["diagnostics"], propagated, canonical_imu
            )
        summary["bags"][label] = bag_summary

    propagated_b = bags["B"]["odometry"]["/Odometry/propagated"]
    summary["B_static"] = static_summary(
        propagated_b, bags["B"]["markers"], "STATIC_60S"
    )
    summary["B_final_static"] = static_summary(
        propagated_b, bags["B"]["markers"], "FINAL_STATIC"
    )
    summary["B_motion"] = motion_summary(propagated_b, bags["B"]["markers"])
    summary["B_velocity_consistency"] = velocity_consistency(propagated_b)
    summary["B_velocity_consistency_windowed"] = [
        windowed_velocity_consistency(propagated_b, window_s)
        for window_s in (0.10, 0.20, 0.30)
    ]
    summary["legacy_comparable_windows"] = {
        "A_default_off_60s": timing(
            between(
                bags["A"]["odometry"]["/Odometry"],
                bags["A"]["markers"]["A_DEFAULT_OFF_START"],
                bags["A"]["markers"]["A_DEFAULT_OFF_END"],
            )
        ),
        "B_enabled_static_60s": timing(
            between(
                bags["B"]["odometry"]["/Odometry"],
                bags["B"]["markers"]["STATIC_60S_START"],
                bags["B"]["markers"]["STATIC_60S_END"],
            )
        ),
    }

    c_marker = bags["C"]["markers"].get("IMU_GATE_STOP")
    c_diagnostics = bags["C"]["diagnostics"]
    c_propagated = bags["C"]["odometry"]["/Odometry/propagated"]
    first_fault = next(
        (
            item
            for item in c_diagnostics
            if c_marker is not None
            and item["receive_ns"] >= c_marker
            and item["state"] == "FAULT"
        ),
        None,
    )
    summary["C_fault_window"] = {
        "marker_receive_ns": c_marker,
        "first_fault_receive_ns": first_fault["receive_ns"] if first_fault else None,
        "first_fault_reason": (
            first_fault["values"].get("fault_reason") if first_fault else None
        ),
        "propagated_after_gate_stop": sum(
            1
            for item in c_propagated
            if c_marker is not None and item["receive_ns"] > c_marker
        ),
        "propagated_after_first_fault": sum(
            1
            for item in c_propagated
            if first_fault is not None and item["receive_ns"] > first_fault["receive_ns"]
        ),
        "last_propagated_minus_gate_stop_ms": (
            (c_propagated[-1]["receive_ns"] - c_marker) * 1e-6
            if c_marker is not None and c_propagated
            else None
        ),
    }
    summary["resources"] = {}
    for label, stem in (
        ("A", "A_default_off"),
        ("B", "B_enabled"),
        ("C", "C_fault"),
    ):
        summary["resources"][label] = {
            "rss_threads": resource_csv(
                args.session / f"metrics/{stem}_rss_threads.csv"
            ),
            "pidstat": pidstat(args.session / f"metrics/{stem}_pidstat_t.txt"),
        }
    args.output.write_text(
        json.dumps(summary, indent=2, sort_keys=True, allow_nan=True) + "\n",
        encoding="utf-8",
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
