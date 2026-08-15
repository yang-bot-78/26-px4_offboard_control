#!/usr/bin/env python3
"""分析拆桨三维速度校准 rosbag。

输入由 ``tools/flight/速度协方差测试B.py`` 产生。报告会明确区分：

* FAST-LIO /Odometry.twist：child/body FLU；先以同一条 odom 的姿态转到世界 ENU；
* MAVROS vision_speed：世界 ENU；上移时 z 应为正；
* PX4 VehicleLocalPosition：原生 NED；上移时 vz 应为负。

协方差建议取静态标准差和动态 PX4 残差中较大的量，再保守放大；不会把协方差
人为压小到掩盖 Z 轴误差。
"""

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


MARKER_RE = re.compile(r"^(速度校准\|(?:Z轴|X轴|Y轴)\|第\d+次\|.+)\|(开始|结束)$")
# 早期标记格式把“静止”和“开始/结束”直接拼接（例如“开场静止开始”）；
# 当前与旧格式均接受，避免完整录包因一个分隔符差异而无法标定。
STATIC_RE = re.compile(r"^(速度校准\|.+?静止)\|?(开始|结束)$")
PX4_TOPICS = (
    "/fmu/out/vehicle_local_position_v1",
    "/fmu/out/vehicle_local_position",
    "/mavros/local_position/velocity_local",
    "/ev_health/velocity_ned",
)
AXES = ("x", "y", "z")


def percentile(values: np.ndarray, q: float) -> float:
    return float(np.percentile(values, q)) if values.size else math.nan


def error_stats(values: np.ndarray) -> dict:
    if not values.size:
        return {key: math.nan for key in ("bias", "rmse", "p95_abs", "max_abs")}
    return {
        "bias": float(np.mean(values)),
        "rmse": float(np.sqrt(np.mean(np.square(values)))),
        "p95_abs": percentile(np.abs(values), 95),
        "max_abs": float(np.max(np.abs(values))),
    }


def velocity_stats(values: np.ndarray) -> dict:
    if not values.size:
        return {key: math.nan for key in ("mean", "std", "p95_abs", "max_abs")}
    return {
        "mean": float(np.mean(values)),
        "std": float(np.std(values)),
        "p95_abs": percentile(np.abs(values), 95),
        "max_abs": float(np.max(np.abs(values))),
    }


def rotation_matrix_xyzw(q: np.ndarray) -> np.ndarray:
    """返回 body/child 到 world 的旋转；坏四元数明确报错，不静默使用单位阵。"""
    x, y, z, w = (float(value) for value in q)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if not math.isfinite(norm) or norm < 1e-8:
        raise ValueError("invalid odometry quaternion")
    x, y, z, w = x / norm, y / norm, z / norm, w / norm
    return np.array(((1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)),
                     (2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)),
                     (2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y))), dtype=float)


def enu_from_ned(values: np.ndarray) -> np.ndarray:
    return values[:, (1, 0, 2)] * np.array((1.0, 1.0, -1.0))


def interpolate_rows(times: np.ndarray, values: np.ndarray, query: np.ndarray) -> np.ndarray:
    return np.column_stack([np.interp(query, times, values[:, axis]) for axis in range(3)])


def finite_time_rows(rows: list[tuple] | np.ndarray) -> np.ndarray:
    """丢弃启动阶段常见的 NaN 样本，并按 rosbag 时间排序。"""
    values = np.asarray(rows, dtype=float)
    values = values[np.all(np.isfinite(values), axis=1)]
    return values[np.argsort(values[:, 0])]


def valid_interp_mask(times: np.ndarray, query: np.ndarray, max_gap: float = 0.06) -> np.ndarray:
    right = np.searchsorted(times, query, side="left")
    valid = (right > 0) & (right < len(times))
    clipped = np.clip(right, 1, max(1, len(times) - 1))
    return valid & ((query - times[clipped - 1]) <= max_gap) & ((times[clipped] - query) <= max_gap)


def nearest_rows(times: np.ndarray, values: np.ndarray, query: np.ndarray, max_gap: float = 0.04) -> tuple[np.ndarray, np.ndarray]:
    right = np.searchsorted(times, query, side="left")
    left = np.clip(right - 1, 0, len(times) - 1)
    right = np.clip(right, 0, len(times) - 1)
    pick = np.where(np.abs(query - times[left]) <= np.abs(times[right] - query), left, right)
    valid = np.abs(query - times[pick]) <= max_gap
    return valid, values[pick]


def find_best_lag(source_t: np.ndarray, source_v: np.ndarray, px4_t: np.ndarray, px4_v: np.ndarray) -> float:
    best_lag, best_rmse = 0.0, math.inf
    for lag in np.arange(-0.5, 0.5001, 0.005):
        query = source_t + lag
        valid = valid_interp_mask(px4_t, query)
        if np.count_nonzero(valid) < 20:
            continue
        residual = source_v[valid] - interpolate_rows(px4_t, px4_v, query[valid])
        score = float(np.sqrt(np.mean(np.square(residual))))
        if score < best_rmse:
            best_lag, best_rmse = float(lag), score
    return best_lag


def pair_markers(markers: list[tuple[float, str]]) -> tuple[list[dict], list[dict]]:
    """从开始/结束标记构造移动段和静止段，任何未闭合标记都是采集失败。"""
    pending: dict[str, float] = {}
    movement, stationary = [], []
    for timestamp, text in markers:
        match = MARKER_RE.match(text) or STATIC_RE.match(text)
        if not match:
            continue
        prefix, edge = match.groups()
        if edge == "开始":
            pending[prefix] = timestamp
            continue
        start = pending.pop(prefix, None)
        if start is None:
            raise RuntimeError(f"找不到开始标记: {text}")
        item = {"label": prefix, "start": start, "end": timestamp}
        (stationary if "静止" in prefix else movement).append(item)
    if pending:
        raise RuntimeError("存在未闭合的阶段标记: " + ", ".join(sorted(pending)))
    return movement, stationary


def selected_px4_topic(types: dict[str, str]) -> str:
    for topic in PX4_TOPICS:
        if topic in types:
            return topic
    raise RuntimeError("rosbag 缺少 PX4 local velocity；需要 " + "、".join(PX4_TOPICS))


def read_bag(bag: Path) -> dict:
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=str(bag), storage_id="sqlite3"), rosbag2_py.ConverterOptions("", ""))
    types = {item.name: item.type for item in reader.get_all_topics_and_types()}
    required = {"/velocity_calibration/marker", "/Odometry", "/mavros/vision_speed/speed_twist_cov"}
    missing = sorted(required - types.keys())
    if missing:
        raise RuntimeError("rosbag 缺少话题: " + ", ".join(missing))
    px4_topic = selected_px4_topic(types)
    wanted = required | {px4_topic}
    classes = {topic: get_message(types[topic]) for topic in wanted}
    rows = {"markers": [], "odom": [], "vision": [], "px4": [], "px4_topic": px4_topic}
    while reader.has_next():
        topic, payload, bag_ns = reader.read_next()
        if topic not in wanted:
            continue
        msg = deserialize_message(payload, classes[topic])
        bag_time = bag_ns * 1e-9
        if topic == "/velocity_calibration/marker":
            rows["markers"].append((bag_time, msg.data))
        elif topic == "/Odometry":
            linear, orientation = msg.twist.twist.linear, msg.pose.pose.orientation
            rows["odom"].append((bag_time, linear.x, linear.y, linear.z, orientation.x, orientation.y, orientation.z, orientation.w))
        elif topic == "/mavros/vision_speed/speed_twist_cov":
            linear = msg.twist.twist.linear
            rows["vision"].append((bag_time, linear.x, linear.y, linear.z, msg.twist.covariance[0], msg.twist.covariance[7], msg.twist.covariance[14]))
        elif topic.startswith("/fmu/out/vehicle_local_position"):
            rows["px4"].append((bag_time, msg.vx, msg.vy, msg.vz))
        elif topic == "/mavros/local_position/velocity_local":
            # MAVROS reports ROS local ENU. Convert explicitly before comparison.
            linear = msg.twist.linear
            rows["px4"].append((bag_time, linear.y, linear.x, -linear.z))
        else:  # /ev_health/velocity_ned: fallback only; its header says px4_local_ned.
            linear = msg.twist.twist.linear
            rows["px4"].append((bag_time, linear.x, linear.y, linear.z))
    for key in ("odom", "vision", "px4"):
        if not rows[key]:
            raise RuntimeError(f"话题没有有效样本: {key}")
    return rows


def static_mask(times: np.ndarray, segments: list[dict], trim_s: float = 0.5) -> np.ndarray:
    mask = np.zeros(len(times), dtype=bool)
    for segment in segments:
        start, end = segment["start"] + trim_s, segment["end"] - trim_s
        if end > start:
            mask |= (times >= start) & (times <= end)
    return mask


def expected_axis(label: str) -> tuple[int, int] | None:
    for index, name in enumerate(("X", "Y", "Z")):
        if f"+{name}" in label:
            return index, 1
        if f"-{name}" in label:
            return index, -1
    return None


def segment_sign_report(segment: dict, vision_t: np.ndarray, vision_v: np.ndarray, raw_t: np.ndarray, raw_v: np.ndarray, px4_t: np.ndarray, px4_v: np.ndarray) -> dict:
    expected = expected_axis(segment["label"])
    if expected is None:
        return {"label": segment["label"], "verdict": "NOT_APPLICABLE"}
    axis, sign = expected
    mask = (vision_t >= segment["start"]) & (vision_t <= segment["end"])
    query = vision_t[mask]
    raw_ok, raw = nearest_rows(raw_t, raw_v, query)
    px4_ok, px4 = nearest_rows(px4_t, px4_v, query)
    means = {"vision_enu": float(np.mean(vision_v[mask, axis]))}
    if np.any(raw_ok):
        means["frlio_world_enu"] = float(np.mean(raw[raw_ok, axis]))
    px4_ned_mean = None
    if np.any(px4_ok):
        px4_enu_mean = np.mean(px4[px4_ok], axis=0)
        means["px4_enu"] = float(px4_enu_mean[axis])
        px4_ned_mean = (float(px4_enu_mean[1]), float(px4_enu_mean[0]), float(-px4_enu_mean[2]))
    observed = [value for value in means.values() if math.isfinite(value)]
    minimum_motion = 0.03
    passed = len(observed) == 3 and all(sign * value >= minimum_motion for value in observed)
    ned_axis = (1, 0, 2)[axis]
    ned_sign = sign if axis < 2 else -sign
    ned_ok = px4_ned_mean is not None and ned_sign * px4_ned_mean[ned_axis] >= minimum_motion
    return {
        "label": segment["label"], "expected_enu_axis": AXES[axis], "expected_enu_sign": sign,
        "expected_px4_ned_axis": ("y/E", "x/N", "z/D")[axis], "expected_px4_ned_sign": ned_sign,
        "mean_mps": means, "px4_ned_mean_xyz_mps": px4_ned_mean,
        "px4_ned_sign_verdict": "PASS" if ned_ok else "FAIL",
        "verdict": "PASS" if passed and ned_ok else "FAIL",
    }


def analyse(data: dict, world_yaw_alignment_rad: float = 0.0, axes: tuple[str, ...] = AXES) -> dict:
    movement, stationary = pair_markers(data["markers"])
    if not movement or not stationary:
        raise RuntimeError("未找到完整的移动或静止阶段标记；请使用新版校准脚本录制")
    vision = finite_time_rows(data["vision"])
    odom = finite_time_rows(data["odom"])
    px4_ned = finite_time_rows(data["px4"])
    if min(len(vision), len(odom), len(px4_ned)) < 20:
        raise RuntimeError("去除非有限值后有效速度样本不足")
    vt, vv, vcov = vision[:, 0], vision[:, 1:4], vision[:, 4:7]
    ot, child_v, quaternions = odom[:, 0], odom[:, 1:4], odom[:, 4:8]
    raw_world = np.asarray([rotation_matrix_xyzw(q) @ v for q, v in zip(quaternions, child_v)])
    if not math.isfinite(world_yaw_alignment_rad):
        raise ValueError("world_yaw_alignment_rad 必须为有限数")
    # 必须复现 bridge 在发布 vision_speed 前施加的世界 yaw 对齐，不能只旋转 child->world。
    c, s = math.cos(world_yaw_alignment_rad), math.sin(world_yaw_alignment_rad)
    raw_world[:, :2] = raw_world[:, :2] @ np.array(((c, s), (-s, c)))
    pt, pv = px4_ned[:, 0], enu_from_ned(px4_ned[:, 1:4])
    dynamic = np.zeros(len(vt), dtype=bool)
    for segment in movement:
        dynamic |= (vt >= segment["start"]) & (vt <= segment["end"])
    if np.count_nonzero(dynamic) < 20:
        raise RuntimeError("移动段中的 vision_speed 样本不足")

    lag = find_best_lag(vt[dynamic], vv[dynamic], pt, pv)
    px4_valid = valid_interp_mask(pt, vt + lag)
    px4_at_vision = interpolate_rows(pt, pv, (vt + lag)[px4_valid])
    px4_residual = vv[px4_valid] - px4_at_vision
    px4_dynamic_residual = px4_residual[dynamic[px4_valid]]
    if len(px4_dynamic_residual) < 20:
        raise RuntimeError("可与 PX4 配对的移动段样本不足")
    raw_valid, raw_at_vision = nearest_rows(ot, raw_world, vt)
    bridge_residual = vv[raw_valid] - raw_at_vision[raw_valid]
    bridge_dynamic_residual = bridge_residual[dynamic[raw_valid]]
    smask = static_mask(vt, stationary)
    if not np.any(smask):
        raise RuntimeError("静止段去除 0.5 秒边缘后没有样本")

    static = {
        "vision_enu": {axis: velocity_stats(vv[smask, index]) for index, axis in enumerate(AXES)},
        "frlio_world_enu": {
            axis: velocity_stats(raw_at_vision[smask[raw_valid], index])
            for index, axis in enumerate(AXES)
        },
        "px4_enu": {
            axis: velocity_stats(px4_at_vision[smask[px4_valid], index])
            for index, axis in enumerate(AXES)
        },
    }
    static_acceptance = {}
    for axis in axes:
        sources = {
            source: {
                "mean_lt_0_03": abs(values[axis]["mean"]) < 0.03,
                "p95_lt_0_10": values[axis]["p95_abs"] < 0.10,
                "stats": values[axis],
            }
            for source, values in static.items()
        }
        for source in sources.values():
            source["verdict"] = "PASS" if source["mean_lt_0_03"] and source["p95_lt_0_10"] else "FAIL"
        static_acceptance[axis] = {
            "sources": sources,
            "verdict": "PASS" if all(value["verdict"] == "PASS" for value in sources.values()) else "FAIL",
        }
    per_axis = {}
    for index, axis in enumerate(AXES):
        if axis not in axes:
            continue
        dynamic_residual = px4_dynamic_residual[:, index]
        scale = max(static["vision_enu"][axis]["std"], error_stats(dynamic_residual)["rmse"], percentile(np.abs(dynamic_residual), 95) / 1.96)
        per_axis[axis] = {
            "static_vision_enu": static["vision_enu"][axis],
            "vision_minus_px4_enu": error_stats(dynamic_residual),
            "vision_minus_rotated_frlio_enu": error_stats(bridge_dynamic_residual[:, index]),
            "mean_declared_variance": float(np.mean(vcov[dynamic, index])),
            "noise_scale_mps": float(scale),
            "recommended_sigma_mps": float(1.5 * scale),
            "recommended_variance_m2ps2": float((1.5 * scale) ** 2),
        }
    sign_reports = [segment_sign_report(segment, vt, vv, ot, raw_world, pt, pv) for segment in movement]
    return {
        "coordinate_convention": {
            "frlio_odometry_twist": "child/body FLU, rotated to world ENU before comparison",
            "vision_speed": "world ENU; upward vz > 0",
            "px4_local_velocity": "PX4 NED; upward vz < 0; converted to ENU only for residuals",
            "px4_source_topic": data["px4_topic"],
            "world_yaw_alignment_rad": world_yaw_alignment_rad,
        },
        "active_axes": list(axes),
        "segments": {"movement": len(movement), "stationary": len(stationary), "sign_checks": sign_reports},
        "best_px4_lag_s": lag,
        "static_acceptance": static_acceptance,
        "per_axis": per_axis,
        "sign_check_passed": all(row["verdict"] == "PASS" for row in sign_reports),
        "rate": {"vision_hz": 1.0 / float(np.mean(np.diff(vt))), "px4_hz": 1.0 / float(np.mean(np.diff(pt)))},
        "recommendation_note": "建议值=max(静态标准差、动态 PX4 残差 RMSE、动态残差 p95/1.96)×1.5 后平方；先修正 Z 轴符号/幅值/时延，勿以很小协方差强行开启 EKF2_EV_CTRL=15。",
    }


def markdown_report(result: dict) -> str:
    lines = ["# 三维速度校准报告", "", f"- PX4 速度源：`{result['coordinate_convention']['px4_source_topic']}`", f"- 最佳 PX4 时延：`{result['best_px4_lag_s']:.3f} s`", f"- 符号检查：**{'PASS' if result['sign_check_passed'] else 'FAIL'}**", "", "| 轴 | 静止均值 | 静止 p95 | 动态 PX4 残差 p95 | 建议方差 |", "| --- | ---: | ---: | ---: | ---: |"]
    for axis in result["active_axes"]:
        values = result["per_axis"][axis]
        lines.append(f"| {axis} | {values['static_vision_enu']['mean']:.4f} | {values['static_vision_enu']['p95_abs']:.4f} | {values['vision_minus_px4_enu']['p95_abs']:.4f} | {values['recommended_variance_m2ps2']:.6g} |")
    lines += ["", "静止验收要求：每轴 |均值| < 0.03 m/s，|速度| p95 < 0.10 m/s。Z 轴任一符号或幅值失败时，不应开启完整三维 EV 速度融合。", "", result["recommendation_note"]]
    return "\n".join(lines) + "\n"


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bag", required=True, type=Path, help="rosbag 目录")
    parser.add_argument("--json", type=Path, help="可选 JSON 输出路径")
    parser.add_argument("--markdown", type=Path, help="可选 Markdown 输出路径")
    parser.add_argument(
        "--world-yaw-alignment-rad", type=float, default=0.0,
        help="与 fastlio_mavros_vision_bridge 的同名参数一致；默认 0",
    )
    parser.add_argument(
        "--axes", default="xyz", help="输出的校准轴，例如 z；默认 xyz",
    )
    args = parser.parse_args()
    # argparse 已完成参数收集后再校验，确保空值或重复轴不会默默产生误导报告。
    selected_axes = tuple(args.axes.lower())
    if not selected_axes or any(axis not in AXES for axis in selected_axes) or len(set(selected_axes)) != len(selected_axes):
        parser.error("--axes 必须是 x、y、z 的非重复组合，例如 z 或 xyz")
    result = analyse(read_bag(args.bag), args.world_yaw_alignment_rad, selected_axes)
    text = json.dumps(result, ensure_ascii=False, indent=2, allow_nan=True)
    print(text)
    if args.json:
        args.json.write_text(text + "\n", encoding="utf-8")
    if args.markdown:
        args.markdown.write_text(markdown_report(result), encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
