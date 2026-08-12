#!/usr/bin/env python3
"""按中文阶段标记分析 FAST-LIO 速度协方差测试 B（水平版本）。"""

from __future__ import annotations

import argparse
import json
import math
import re
from pathlib import Path

import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message


MOVEMENT_RE = re.compile(
    r"^测试B\|(前后轴|左右轴)\|第(\d+)次\|"
    r"(向前移动|向后返回|向左移动|向右返回)(开始|结束)$"
)
STATIC_RE = re.compile(
    r"^测试B\|(前后轴|左右轴)\|第(\d+)次\|"
    r"(起点静止|终点静止|返回后静止)(开始|结束)$"
)


def stamp_seconds(stamp) -> float:
    return float(stamp.sec) + float(stamp.nanosec) * 1e-9


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else math.nan


def stats(error: np.ndarray) -> dict:
    if not error.size:
        return {key: math.nan for key in ("bias", "rmse", "p95_abs", "max_abs")}
    return {
        "bias": float(np.mean(error)),
        "rmse": float(np.sqrt(np.mean(np.square(error)))),
        "p95_abs": percentile(np.abs(error), 95),
        "max_abs": float(np.max(np.abs(error))),
    }


def corr(a: np.ndarray, b: np.ndarray) -> float:
    if len(a) < 3 or np.std(a) < 1e-9 or np.std(b) < 1e-9:
        return math.nan
    return float(np.corrcoef(a, b)[0, 1])


def interpolate_rows(times: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query, times, values[:, i]) for i in range(values.shape[1])])


def segment_mask(times: np.ndarray, segments: list[dict], margin: float = 0.0) -> np.ndarray:
    mask = np.zeros(len(times), dtype=bool)
    for segment in segments:
        start = segment["start"] + margin
        end = segment["end"] - margin
        if end > start:
            mask |= (times >= start) & (times <= end)
    return mask


def read_bag(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(
        rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"),
        rosbag2_py.ConverterOptions("", ""),
    )
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {
        "/velocity_calibration/marker",
        "/mavros/vision_speed/speed_twist_cov",
        "/mavros/local_position/velocity_local",
        "/Odometry",
        "/ev_health/status",
        "/ev_health/diagnostics",
        "/mavros/state",
    }
    missing = sorted(required - types.keys())
    if missing:
        raise RuntimeError("rosbag 缺少话题: " + ", ".join(missing))
    classes = {name: get_message(kind) for name, kind in types.items() if name in required}

    result = {
        "markers": [], "vision": [], "px4": [], "odom": [],
        "health": [], "diag": [], "states": [],
    }
    while reader.has_next():
        topic, payload, bag_ns = reader.read_next()
        if topic not in required:
            continue
        msg = deserialize_message(payload, classes[topic])
        bag_time = bag_ns * 1e-9
        if topic == "/velocity_calibration/marker":
            result["markers"].append((bag_time, msg.data))
        elif topic == "/mavros/vision_speed/speed_twist_cov":
            v = msg.twist.twist.linear
            result["vision"].append((
                bag_time, stamp_seconds(msg.header.stamp), v.x, v.y, v.z,
                msg.twist.covariance[0], msg.twist.covariance[7], msg.twist.covariance[14],
            ))
        elif topic == "/mavros/local_position/velocity_local":
            v = msg.twist.linear
            result["px4"].append((bag_time, stamp_seconds(msg.header.stamp), v.x, v.y, v.z))
        elif topic == "/Odometry":
            p = msg.pose.pose.position
            result["odom"].append((bag_time, stamp_seconds(msg.header.stamp), p.x, p.y, p.z))
        elif topic == "/ev_health/status":
            result["health"].append((bag_time, msg.data))
        elif topic == "/mavros/state":
            result["states"].append((bag_time, bool(msg.connected), bool(msg.armed), msg.mode))
        elif topic == "/ev_health/diagnostics" and msg.status:
            values = {item.key: item.value for item in msg.status[0].values}
            result["diag"].append((bag_time, values))
    return result


def make_segments(markers: list[tuple[float, str]]) -> tuple[list[dict], list[dict]]:
    pending: dict[tuple[str, int, str, str], float] = {}
    movement: list[dict] = []
    stationary: list[dict] = []
    for timestamp, text in markers:
        match = MOVEMENT_RE.match(text) or STATIC_RE.match(text)
        if not match:
            continue
        axis, repetition, phase, edge = match.groups()
        kind = "movement" if MOVEMENT_RE.match(text) else "stationary"
        key = axis, int(repetition), phase, kind
        if edge == "开始":
            pending[key] = timestamp
        elif key in pending:
            item = {
                "axis": axis,
                "repetition": int(repetition),
                "phase": phase,
                "start": pending.pop(key),
                "end": timestamp,
            }
            (movement if kind == "movement" else stationary).append(item)
    if pending:
        raise RuntimeError(f"存在未闭合的阶段标记: {sorted(pending)}")
    return movement, stationary


def valid_interp_mask(source_times: np.ndarray, query: np.ndarray, max_gap: float = 0.05) -> np.ndarray:
    right = np.searchsorted(source_times, query, side="left")
    valid = (right > 0) & (right < len(source_times))
    clipped = np.clip(right, 1, len(source_times) - 1)
    return valid & ((query - source_times[clipped - 1]) <= max_gap) & (
        (source_times[clipped] - query) <= max_gap
    )


def find_best_lag(
    vision_t: np.ndarray,
    vision_v: np.ndarray,
    px4_t: np.ndarray,
    px4_v: np.ndarray,
) -> tuple[float, float]:
    best_lag = 0.0
    best_rmse = math.inf
    for lag in np.arange(-0.5, 0.5001, 0.005):
        query = vision_t + lag
        valid = valid_interp_mask(px4_t, query)
        if np.count_nonzero(valid) < 20:
            continue
        reference = interpolate_rows(px4_t, px4_v, query[valid])
        residual = vision_v[valid, :2] - reference[:, :2]
        rmse = float(np.sqrt(np.mean(np.square(residual))))
        if rmse < best_rmse:
            best_lag, best_rmse = float(lag), rmse
    return best_lag, best_rmse


def settling_time(times: np.ndarray, velocity: np.ndarray, start: float) -> float:
    indices = np.flatnonzero((times >= start) & (times <= start + 3.0))
    speed = np.linalg.norm(velocity[:, :2], axis=1)
    for index in indices:
        end = np.searchsorted(times, times[index] + 0.5)
        if end > index and times[min(end, len(times) - 1)] - times[index] >= 0.45:
            if np.all(speed[index:end] <= 0.05):
                return max(0.0, float(times[index] - start))
    return math.nan


def diagnostic_numbers(rows: list[tuple[float, dict]], key: str, start: float, end: float) -> np.ndarray:
    values = []
    for timestamp, item in rows:
        if start <= timestamp <= end and key in item:
            try:
                value = float(item[key])
            except ValueError:
                continue
            if math.isfinite(value):
                values.append(value)
    return np.asarray(values, dtype=float)


def analyse(data: dict) -> dict:
    movement, stationary = make_segments(data["markers"])
    if len(movement) != 20 or len(stationary) != 30:
        raise RuntimeError(f"阶段数量异常: movement={len(movement)}, stationary={len(stationary)}")

    vision = np.asarray(data["vision"], dtype=float)
    px4 = np.asarray(data["px4"], dtype=float)
    odom = np.asarray(data["odom"], dtype=float)
    vt_bag, vt, vv, vcov = vision[:, 0], vision[:, 1], vision[:, 2:5], vision[:, 5:8]
    pt, pv = px4[:, 1], px4[:, 2:5]

    move_mask = segment_mask(vt_bag, movement)
    static_mask = segment_mask(vt_bag, stationary, margin=0.5)
    move_t, move_v = vt[move_mask], vv[move_mask]
    best_lag, _ = find_best_lag(move_t, move_v, pt, pv)

    same_valid = valid_interp_mask(pt, move_t)
    lag_valid = valid_interp_mask(pt, move_t + best_lag)
    same_ref = interpolate_rows(pt, pv, move_t[same_valid])
    lag_ref = interpolate_rows(pt, pv, move_t[lag_valid] + best_lag)
    same_error = move_v[same_valid] - same_ref
    lag_error = move_v[lag_valid] - lag_ref

    per_axis = {}
    static_baseline = {"x": 0.00803, "y": 0.00916}
    for index, name in enumerate(("x", "y")):
        same = stats(same_error[:, index])
        delayed = stats(lag_error[:, index])
        scale = max(delayed["rmse"], delayed["p95_abs"] / 1.96, static_baseline[name])
        per_axis[name] = {
            "same_timestamp": same,
            "lag_compensated": delayed,
            "correlation_same": corr(move_v[same_valid, index], same_ref[:, index]),
            "correlation_lag": corr(move_v[lag_valid, index], lag_ref[:, index]),
            "mean_declared_variance": float(np.mean(vcov[move_mask, index])),
            "recommended_sigma_1_5": 1.5 * scale,
            "recommended_sigma_2_0": 2.0 * scale,
            "recommended_variance_1_5": (1.5 * scale) ** 2,
            "recommended_variance_2_0": (2.0 * scale) ** 2,
        }

    same_horizontal = np.linalg.norm(same_error[:, :2], axis=1)
    lag_horizontal = np.linalg.norm(lag_error[:, :2], axis=1)
    position_t, position = odom[:, 0], odom[:, 2:5]
    segment_rows = []
    fast_settle = []
    px4_settle = []
    for segment in movement:
        start, end = segment["start"], segment["end"]
        p = interpolate_rows(position_t, position, np.asarray([start, end]))
        displacement = p[1] - p[0]
        mask = (vt_bag >= start) & (vt_bag <= end)
        seg_t, seg_v = vt[mask], vv[mask]
        valid = valid_interp_mask(pt, seg_t)
        reference = interpolate_rows(pt, pv, seg_t[valid])
        error = seg_v[valid] - reference
        horizontal_error = np.linalg.norm(error[:, :2], axis=1)
        fs = settling_time(vt_bag, vv, end)
        ps = settling_time(px4[:, 0], pv, end)
        fast_settle.append(fs)
        px4_settle.append(ps)
        segment_rows.append({
            "axis": segment["axis"], "repetition": segment["repetition"],
            "phase": segment["phase"], "duration": end - start,
            "displacement_x": float(displacement[0]),
            "displacement_y": float(displacement[1]),
            "distance_horizontal": float(np.linalg.norm(displacement[:2])),
            "mean_speed_horizontal": float(np.mean(np.linalg.norm(seg_v[:, :2], axis=1))),
            "error_p95_horizontal": percentile(horizontal_error, 95),
            "error_max_horizontal": float(np.max(horizontal_error)),
            "fastlio_settle_s": fs, "px4_settle_s": ps,
        })

    static_velocity = vv[static_mask]
    static_result = {}
    for index, name in enumerate(("x", "y")):
        static_result[name] = {
            "bias": float(np.mean(static_velocity[:, index])),
            "std": float(np.std(static_velocity[:, index])),
            "p95_abs": percentile(np.abs(static_velocity[:, index]), 95),
            "max_abs": float(np.max(np.abs(static_velocity[:, index]))),
        }

    test_start = min(item["start"] for item in movement + stationary)
    test_end = max(item["end"] for item in movement + stationary)
    health = [state for timestamp, state in data["health"] if test_start <= timestamp <= test_end]
    state_rows = [item for item in data["states"] if test_start <= item[0] <= test_end]
    diag_keys = (
        "input_age_s", "velocity_alignment_s", "horizontal_velocity_difference_mps",
        "internal_vs_position_velocity_difference_mps",
    )
    diagnostics = {}
    for key in diag_keys:
        values = diagnostic_numbers(data["diag"], key, test_start, test_end)
        diagnostics[key] = {
            "count": int(values.size), "mean": float(np.mean(values)),
            "p95": percentile(values, 95), "max": float(np.max(values)),
        } if values.size else {"count": 0}

    source_dt = np.diff(vt)
    receive_dt = np.diff(vt_bag)
    return {
        "movement_segment_count": len(movement),
        "stationary_segment_count": len(stationary),
        "movement_samples": int(np.count_nonzero(move_mask)),
        "stationary_samples": int(np.count_nonzero(static_mask)),
        "best_px4_lag_s": best_lag,
        "per_axis": per_axis,
        "horizontal_error": {
            "same_rmse": float(np.sqrt(np.mean(np.square(same_horizontal)))),
            "same_p95": percentile(same_horizontal, 95),
            "same_max": float(np.max(same_horizontal)),
            "lag_rmse": float(np.sqrt(np.mean(np.square(lag_horizontal)))),
            "lag_p95": percentile(lag_horizontal, 95),
            "lag_max": float(np.max(lag_horizontal)),
        },
        "static": static_result,
        "segments": segment_rows,
        "settling": {
            "fastlio_p95": percentile(np.asarray(fast_settle), 95),
            "fastlio_max": float(np.nanmax(fast_settle)),
            "px4_p95": percentile(np.asarray(px4_settle), 95),
            "px4_max": float(np.nanmax(px4_settle)),
        },
        "health_counts": {state: health.count(state) for state in sorted(set(health))},
        "all_connected": all(row[1] for row in state_rows),
        "any_armed": any(row[2] for row in state_rows),
        "modes": sorted(set(row[3] for row in state_rows)),
        "diagnostics": diagnostics,
        "rate": {
            "source_hz": 1.0 / float(np.mean(source_dt)),
            "source_max_gap_s": float(np.max(source_dt)),
            "receive_hz": 1.0 / float(np.mean(receive_dt)),
            "receive_max_gap_s": float(np.max(receive_dt)),
        },
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag 目录")
    parser.add_argument("--json", type=Path, help="可选 JSON 输出路径")
    args = parser.parse_args()
    result = analyse(read_bag(args.bag))
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=True)
    print(text)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
