#!/usr/bin/env python3
"""Runtime helpers for the propeller-off high-rate odometry validation."""

import argparse
from collections import deque
import csv
import json
import multiprocessing as mp
import os
import signal
import sys
import time
from pathlib import Path


def _import_ros():
    try:
        import rclpy
        from livox_ros_driver2.msg import CustomMsg
        from mavros_msgs.msg import State
        from mavros_msgs.msg import EstimatorStatus
        from nav_msgs.msg import Odometry
        from rclpy.executors import ExternalShutdownException
        from rclpy.node import Node
        from rclpy.qos import (
            DurabilityPolicy,
            HistoryPolicy,
            QoSProfile,
            ReliabilityPolicy,
        )
        from sensor_msgs.msg import Imu
        from std_msgs.msg import Bool, Float64, String
        from std_srvs.srv import SetBool
    except ImportError as exc:
        raise RuntimeError(
            "ROS 2/Livox Python 类型不可用；请先 source ROS、livox_ws 和 FR-LIO 工作区"
        ) from exc
    return {
        "rclpy": rclpy,
        "CustomMsg": CustomMsg,
        "State": State,
        "EstimatorStatus": EstimatorStatus,
        "Odometry": Odometry,
        "ExternalShutdownException": ExternalShutdownException,
        "Node": Node,
        "QoSProfile": QoSProfile,
        "ReliabilityPolicy": ReliabilityPolicy,
        "DurabilityPolicy": DurabilityPolicy,
        "HistoryPolicy": HistoryPolicy,
        "Imu": Imu,
        "Bool": Bool,
        "Float64": Float64,
        "String": String,
        "SetBool": SetBool,
    }


def _qos(ros, *, reliable=False, transient=False, depth=20):
    return ros["QoSProfile"](
        reliability=(
            ros["ReliabilityPolicy"].RELIABLE
            if reliable
            else ros["ReliabilityPolicy"].BEST_EFFORT
        ),
        durability=(
            ros["DurabilityPolicy"].TRANSIENT_LOCAL
            if transient
            else ros["DurabilityPolicy"].VOLATILE
        ),
        history=ros["HistoryPolicy"].KEEP_LAST,
        depth=depth,
    )


def run_gate(args):
    ros = _import_ros()
    rclpy = ros["rclpy"]

    class LidarGate(ros["Node"]):
        def __init__(self):
            super().__init__("frlio_validation_lidar_gate")
            self.enabled = True
            self.forwarded = 0
            data_qos = _qos(ros, reliable=True, depth=20)
            status_qos = _qos(ros, reliable=True, transient=True, depth=1)
            self.publisher = self.create_publisher(
                ros["CustomMsg"], args.output_topic, data_qos
            )
            self.status_publisher = self.create_publisher(
                ros["Bool"], args.status_topic, status_qos
            )
            self.subscription = self.create_subscription(
                ros["CustomMsg"], args.input_topic, self._on_lidar, data_qos
            )
            self.service = self.create_service(
                ros["SetBool"], args.service, self._set_enabled
            )
            self._publish_status()
            self.get_logger().info(
                f"LiDAR gate enabled: {args.input_topic} -> {args.output_topic}; "
                f"service={args.service}"
            )

        def _on_lidar(self, message):
            if self.enabled:
                self.publisher.publish(message)
                self.forwarded += 1

        def _publish_status(self):
            message = ros["Bool"]()
            message.data = self.enabled
            self.status_publisher.publish(message)

        def _set_enabled(self, request, response):
            self.enabled = bool(request.data)
            self._publish_status()
            state = "enabled" if self.enabled else "paused"
            response.success = True
            response.message = f"LiDAR forwarding {state}; forwarded={self.forwarded}"
            self.get_logger().warning(response.message)
            return response

    rclpy.init()
    node = LidarGate()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ros["ExternalShutdownException"]):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def run_mavros_state(args):
    """Capture only newly published MAVROS states and keep the newest stamp."""
    ros = _import_ros()
    rclpy = ros["rclpy"]
    rclpy.init()
    node = ros["Node"]("frlio_validation_mavros_state_reader")
    latest = {"message": None, "stamp_ns": -1}

    def on_state(message):
        stamp_ns = int(message.header.stamp.sec) * 1_000_000_000
        stamp_ns += int(message.header.stamp.nanosec)
        if stamp_ns >= latest["stamp_ns"]:
            latest["message"] = message
            latest["stamp_ns"] = stamp_ns

    # VOLATILE deliberately excludes transient historical samples. BEST_EFFORT
    # remains compatible with MAVROS's RELIABLE publisher.
    subscription = node.create_subscription(
        ros["State"], "/mavros/state", on_state,
        _qos(ros, reliable=False, transient=False, depth=20),
    )
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.1)
            message = latest["message"]
            if args.wait_connected and message is not None and message.connected:
                break
    finally:
        node.destroy_subscription(subscription)
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    message = latest["message"]
    result = {
        "received": message is not None,
        "stamp_ns": latest["stamp_ns"] if message is not None else None,
        "connected": bool(message.connected) if message is not None else None,
        "armed": bool(message.armed) if message is not None else None,
        "guided": bool(message.guided) if message is not None else None,
        "manual_input": bool(message.manual_input) if message is not None else None,
        "mode": str(message.mode) if message is not None else None,
        "system_status": int(message.system_status) if message is not None else None,
    }
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False), flush=True)
    if message is None:
        return 1
    if args.wait_connected and not message.connected:
        return 1
    return 0


def run_flight_monitor(args):
    """Gate POSITION entry, then keep alarming without commanding the aircraft."""
    ros = _import_ros()
    rclpy = ros["rclpy"]
    rclpy.init()
    node = ros["Node"]("frlio_position_flight_monitor")
    now = time.monotonic
    started_at = now()
    state = {
        "frlio": None,
        "ev": None,
        "ev_fault": None,
        "mavros": None,
        "mavros_at": None,
        "estimator": None,
        "estimator_at": None,
        "healthy_odom_at": None,
    }
    odom_times = deque()
    status_qos = _qos(ros, reliable=True, transient=True, depth=10)
    reliable_qos = _qos(ros, reliable=True, transient=False, depth=20)
    sensor_qos = _qos(ros, reliable=False, transient=False, depth=100)

    node.create_subscription(
        ros["String"], "/frlio/high_rate_odom/status",
        lambda msg: state.__setitem__("frlio", msg.data), status_qos,
    )
    node.create_subscription(
        ros["String"], "/ev_health/status",
        lambda msg: state.__setitem__("ev", msg.data), reliable_qos,
    )
    node.create_subscription(
        ros["Bool"], "/ev_health/fault",
        lambda msg: state.__setitem__("ev_fault", bool(msg.data)), reliable_qos,
    )
    def on_mavros(message):
        state["mavros"] = message
        state["mavros_at"] = now()

    def on_estimator(message):
        state["estimator"] = message
        state["estimator_at"] = now()

    node.create_subscription(
        ros["State"], "/mavros/state", on_mavros, sensor_qos
    )
    node.create_subscription(
        ros["EstimatorStatus"], "/mavros/estimator_status", on_estimator, sensor_qos
    )

    def on_healthy_odom(_message):
        stamp = now()
        state["healthy_odom_at"] = stamp
        odom_times.append(stamp)
        while odom_times and stamp - odom_times[0] > 1.0:
            odom_times.popleft()

    node.create_subscription(
        ros["Odometry"], "/Odometry/healthy", on_healthy_odom, sensor_qos
    )

    ready_path = Path(args.ready_file)
    alarm_path = Path(args.alarm_file)
    stop_path = Path(args.stop_file)
    event_path = Path(args.event_log)
    for path in (ready_path, alarm_path, stop_path, event_path):
        path.parent.mkdir(parents=True, exist_ok=True)
    for path in (ready_path, alarm_path, stop_path):
        try:
            path.unlink()
        except FileNotFoundError:
            pass

    last_reasons = None
    last_reason_signature = None
    last_alarm_print = 0.0
    stable_since = None
    ready = False

    def reasons(require_disarmed):
        current = now()
        mavros = state["mavros"]
        estimator = state["estimator"]
        result = []
        if state["frlio"] != "HEALTHY":
            result.append(f"frlio={state['frlio']}")
        if state["ev"] != "HEALTHY":
            result.append(f"ev={state['ev']}")
        if state["ev_fault"] is not False:
            result.append(f"ev_fault={state['ev_fault']}")
        if (mavros is None or state["mavros_at"] is None or
                current - state["mavros_at"] > args.status_timeout or
                not mavros.connected):
            result.append("mavros_disconnected")
        elif str(mavros.mode).upper() == "OFFBOARD":
            result.append("unexpected_OFFBOARD")
        elif require_disarmed and mavros.armed:
            result.append("armed_before_ready")
        estimator_stale = (
            estimator is None or state["estimator_at"] is None or
            current - state["estimator_at"] > args.status_timeout
        )
        if estimator_stale or not estimator.pos_horiz_rel_status_flag:
            result.append("px4_horizontal_position_invalid")
        if estimator_stale or not estimator.pos_vert_abs_status_flag:
            result.append("px4_vertical_position_invalid")
        odom_at = state["healthy_odom_at"]
        if odom_at is None or current - odom_at > args.odom_timeout:
            result.append("healthy_odometry_timeout")
        if require_disarmed and len(odom_times) < args.min_odom_rate:
            result.append(f"healthy_odometry_rate={len(odom_times)}Hz")
        for topic in (
            "/mavros/setpoint_raw/local",
            "/mavros/setpoint_position/local",
            "/fmu/in/trajectory_setpoint",
            "/fmu/in/vehicle_command",
        ):
            if node.count_publishers(topic) > 0:
                result.append(f"unexpected_control_publisher:{topic}")
        return result

    try:
        while rclpy.ok() and not stop_path.exists():
            rclpy.spin_once(node, timeout_sec=0.02)
            current = now()
            current_reasons = reasons(require_disarmed=not ready)

            if not ready:
                if current - started_at > args.ready_timeout:
                    print(
                        f"[飞行监视] 稳定门超时：{', '.join(current_reasons)}",
                        file=sys.stderr, flush=True,
                    )
                    return 2
                if current_reasons:
                    stable_since = None
                    reason_signature = tuple(
                        "healthy_odometry_rate" if reason.startswith("healthy_odometry_rate=")
                        else reason
                        for reason in current_reasons
                    )
                    if reason_signature != last_reason_signature:
                        print(
                            f"[飞行监视] 等待稳定：{', '.join(current_reasons)}",
                            flush=True,
                        )
                    last_reason_signature = reason_signature
                else:
                    if stable_since is None:
                        stable_since = current
                    stable_elapsed = current - stable_since
                    if int(stable_elapsed) != int(max(0.0, stable_elapsed - 0.03)):
                        print(
                            f"[飞行监视] 连续稳定 {stable_elapsed:.0f}/{args.ready_sec:.0f}s",
                            flush=True,
                        )
                        last_reason_signature = None
                    if stable_elapsed >= args.ready_sec:
                        ready = True
                        ready_path.write_text(
                            json.dumps(
                                {
                                    "ready_monotonic_s": current,
                                    "stable_s": stable_elapsed,
                                    "healthy_odom_rate_hz": len(odom_times),
                                },
                                ensure_ascii=False,
                                indent=2,
                            ) + "\n",
                            encoding="utf-8",
                        )
                        print(
                            f"[飞行监视] READY：健康链连续稳定 {stable_elapsed:.1f}s，"
                            f"/Odometry/healthy 约 {len(odom_times)}Hz",
                            flush=True,
                        )
            elif current_reasons:
                alarm = {
                    "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    "elapsed_s": current - started_at,
                    "reasons": current_reasons,
                }
                if current_reasons != last_reasons:
                    with event_path.open("a", encoding="utf-8") as stream:
                        stream.write(json.dumps(alarm, ensure_ascii=False) + "\n")
                    alarm_path.write_text(
                        json.dumps(alarm, ensure_ascii=False, indent=2) + "\n",
                        encoding="utf-8",
                    )
                if current - last_alarm_print >= 2.0:
                    print("\a", end="", flush=True)
                    print(
                        "[立即处置] 定位链异常："
                        f"{', '.join(current_reasons)}。立即切回 STABILIZED，人工降落并上锁！",
                        file=sys.stderr,
                        flush=True,
                    )
                    last_alarm_print = current
            elif last_reasons:
                recovery = {
                    "wall_time": time.strftime("%Y-%m-%dT%H:%M:%S%z"),
                    "elapsed_s": current - started_at,
                    "recovered": True,
                }
                with event_path.open("a", encoding="utf-8") as stream:
                    stream.write(json.dumps(recovery, ensure_ascii=False) + "\n")
                print("[飞行监视] 定位状态已恢复；飞行处置仍由飞手决定。", flush=True)
            last_reasons = current_reasons
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


class _ValidationMonitor:
    def __init__(self, ros, name, include_sensor_counts=True):
        self.ros = ros
        self.node = ros["Node"](name)
        self.start = time.monotonic()
        self.status = ""
        self.status_events = []
        self.latest_anchor = None
        self.anchor_samples = []
        self.counts = {"odom": 0, "imu": 0, "raw_lidar": 0, "gated_lidar": 0}
        self.last_receive = {key: None for key in self.counts}
        self.max_odom_gap_s = 0.0
        sensor_qos = _qos(ros, reliable=False, depth=50)
        status_qos = _qos(ros, reliable=True, transient=True, depth=1)
        self.node.create_subscription(
            ros["String"], "/frlio/high_rate_odom/status", self._on_status, status_qos
        )
        self.node.create_subscription(
            ros["Float64"], "/frlio/high_rate_odom/anchor_age", self._on_anchor, sensor_qos
        )
        self.node.create_subscription(
            ros["Odometry"], "/Odometry", lambda _: self._count("odom"), sensor_qos
        )
        if include_sensor_counts:
            self.node.create_subscription(
                ros["Imu"], "/livox/imu", lambda _: self._count("imu"), sensor_qos
            )
            self.node.create_subscription(
                ros["CustomMsg"], "/livox/lidar",
                lambda _: self._count("raw_lidar"), sensor_qos
            )
            self.node.create_subscription(
                ros["CustomMsg"], "/validation/livox/lidar",
                lambda _: self._count("gated_lidar"), sensor_qos
            )

    def elapsed(self):
        return time.monotonic() - self.start

    def _on_status(self, message):
        if message.data != self.status:
            self.status = message.data
            event = {
                "elapsed_s": self.elapsed(),
                "status": self.status,
                "anchor_age_s": self.latest_anchor,
            }
            self.status_events.append(event)
            print(
                f"[状态] t={event['elapsed_s']:.3f}s {self.status}, "
                f"anchor_age={self.latest_anchor}",
                flush=True,
            )

    def _on_anchor(self, message):
        self.latest_anchor = float(message.data)
        self.anchor_samples.append((self.elapsed(), self.latest_anchor))

    def _count(self, key):
        now = time.monotonic()
        previous = self.last_receive[key]
        if key == "odom" and previous is not None:
            self.max_odom_gap_s = max(self.max_odom_gap_s, now - previous)
        self.last_receive[key] = now
        self.counts[key] += 1

    def spin_until(self, predicate, timeout_s):
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.ros["rclpy"].spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return bool(predicate())

    def close(self):
        self.node.destroy_node()


def _call_gate(monitor, enabled, timeout_s=3.0):
    ros = monitor.ros
    client = monitor.node.create_client(ros["SetBool"], "/frlio_validation/lidar_gate")
    if not client.wait_for_service(timeout_sec=timeout_s):
        raise RuntimeError("LiDAR gate service is unavailable")
    request = ros["SetBool"].Request()
    request.data = enabled
    future = client.call_async(request)
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline and not future.done():
        ros["rclpy"].spin_once(monitor.node, timeout_sec=0.02)
    if not future.done() or not future.result() or not future.result().success:
        raise RuntimeError(f"LiDAR gate request failed: enabled={enabled}")
    return future.result().message


def run_fault_test(args):
    ros = _import_ros()
    rclpy = ros["rclpy"]
    rclpy.init()
    monitor = _ValidationMonitor(ros, "frlio_validation_fault_monitor")
    result = {"passed": False, "checks": {}, "status_events": []}
    gate_paused = False
    try:
        ready = monitor.spin_until(
            lambda: monitor.status == "HEALTHY"
            and monitor.counts["odom"] >= 20
            and monitor.counts["imu"] >= 20
            and monitor.counts["raw_lidar"] >= 2
            and monitor.counts["gated_lidar"] >= 2,
            8.0,
        )
        if not ready:
            raise RuntimeError("故障测试前数据链未达到 HEALTHY 或消息计数不足")

        baseline = dict(monitor.counts)
        print("[故障测试] 暂停门控 LiDAR；IMU 和原始 LiDAR 保持运行。", flush=True)
        print(f"[故障测试] {_call_gate(monitor, False)}", flush=True)
        gate_paused = True
        disabled_at = time.monotonic()

        saw_fault = monitor.spin_until(
            lambda: any(e["status"] == "FAULT_STALE_LIDAR" for e in monitor.status_events),
            2.0,
        )
        fault_at = next(
            (monitor.start + e["elapsed_s"] for e in monitor.status_events
             if e["status"] == "FAULT_STALE_LIDAR"),
            None,
        )
        target_resume_at = disabled_at + args.pause_seconds
        if saw_fault and fault_at is not None:
            target_resume_at = max(target_resume_at, fault_at + args.odom_quiet_seconds)
        while time.monotonic() < target_resume_at:
            rclpy.spin_once(monitor.node, timeout_sec=0.02)

        now = time.monotonic()
        last_odom = monitor.last_receive["odom"]
        last_gated = monitor.last_receive["gated_lidar"]
        suspect_events = [
            e for e in monitor.status_events if e["status"] == "SUSPECT_STALE_LIDAR"
        ]
        fault_events = [
            e for e in monitor.status_events if e["status"] == "FAULT_STALE_LIDAR"
        ]
        warn_crossings = [value for _, value in monitor.anchor_samples if value >= 0.15]
        fault_crossings = [value for _, value in monitor.anchor_samples if value >= 0.40]
        deltas = {key: monitor.counts[key] - baseline[key] for key in baseline}

        result["checks"] = {
            "saw_suspect": bool(suspect_events),
            "saw_fault": bool(fault_events),
            "anchor_crossed_0_15_s": bool(warn_crossings),
            "anchor_crossed_0_40_s": bool(fault_crossings),
            "odom_quiet_after_fault": bool(
                saw_fault and last_odom is not None and now - last_odom >= args.odom_quiet_seconds
            ),
            "imu_continued": deltas["imu"] >= 20,
            "raw_lidar_continued": deltas["raw_lidar"] >= 2,
            "gated_lidar_quiet": bool(
                last_gated is not None and now - last_gated >= args.odom_quiet_seconds
            ),
        }
        result["counts_during_pause"] = deltas
        result["odom_quiet_s"] = None if last_odom is None else now - last_odom
        result["gated_lidar_quiet_s"] = None if last_gated is None else now - last_gated
        result["first_anchor_at_or_above_0_15_s"] = warn_crossings[0] if warn_crossings else None
        result["first_anchor_at_or_above_0_40_s"] = fault_crossings[0] if fault_crossings else None

        print(f"[故障测试] {_call_gate(monitor, True)}", flush=True)
        gate_paused = False
        recovered = monitor.spin_until(lambda: monitor.status == "HEALTHY", args.recovery_timeout)
        result["checks"]["recovered_healthy"] = recovered
        result["status_events"] = monitor.status_events
        result["passed"] = all(result["checks"].values())
    except Exception as exc:  # Keep the gate fail-open for subsequent validation.
        result["error"] = str(exc)
        print(f"[故障测试] 错误：{exc}", file=sys.stderr, flush=True)
    finally:
        if gate_paused:
            try:
                print(f"[故障测试] 清理恢复：{_call_gate(monitor, True)}", flush=True)
            except Exception as exc:
                result["restore_error"] = str(exc)
        result["status_events"] = monitor.status_events
        output = Path(args.output)
        output.parent.mkdir(parents=True, exist_ok=True)
        output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
        monitor.close()
        rclpy.shutdown()

    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result["passed"] else 1


def _percentile(values, percentile):
    if not values:
        return None
    ordered = sorted(values)
    index = int(round((len(ordered) - 1) * percentile))
    return ordered[index]


def run_observe(args):
    ros = _import_ros()
    rclpy = ros["rclpy"]
    rclpy.init()
    monitor = _ValidationMonitor(
        ros,
        "frlio_validation_static_monitor",
        include_sensor_counts=not args.lightweight,
    )
    csv_path = Path(args.csv)
    csv_path.parent.mkdir(parents=True, exist_ok=True)
    next_print = time.monotonic() + args.print_every
    last_written = 0
    with csv_path.open("w", newline="", encoding="utf-8") as stream:
        writer = csv.writer(stream)
        writer.writerow(["elapsed_s", "anchor_age_s", "status"])
        deadline = time.monotonic() + args.duration
        try:
            while time.monotonic() < deadline:
                rclpy.spin_once(monitor.node, timeout_sec=0.02)
                while last_written < len(monitor.anchor_samples):
                    elapsed, value = monitor.anchor_samples[last_written]
                    writer.writerow([f"{elapsed:.9f}", f"{value:.9f}", monitor.status])
                    last_written += 1
                if time.monotonic() >= next_print:
                    values = [value for _, value in monitor.anchor_samples]
                    maximum = max(values) if values else float("nan")
                    over = sum(value > 0.15 for value in values)
                    print(
                        f"[静置监测] {monitor.elapsed():.0f}/{args.duration:.0f}s "
                        f"status={monitor.status} max_anchor={maximum:.4f}s "
                        f">0.15s={over} odom={monitor.counts['odom']}",
                        flush=True,
                    )
                    stream.flush()
                    next_print += args.print_every
        except KeyboardInterrupt:
            pass

    values = [value for _, value in monitor.anchor_samples]
    summary = {
        "requested_duration_s": args.duration,
        "observed_duration_s": monitor.elapsed(),
        "sample_count": len(values),
        "anchor_age_mean_s": sum(values) / len(values) if values else None,
        "anchor_age_p99_s": _percentile(values, 0.99),
        "anchor_age_max_s": max(values) if values else None,
        "anchor_age_over_0_15_count": sum(value > 0.15 for value in values),
        "anchor_age_over_0_30_count": sum(value > 0.30 for value in values),
        "anchor_age_over_0_40_count": sum(value > 0.40 for value in values),
        "max_odom_receive_gap_s": monitor.max_odom_gap_s,
        "counts": monitor.counts,
        "final_status": monitor.status,
        "status_events": monitor.status_events,
    }
    Path(args.summary).write_text(
        json.dumps(summary, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    monitor.close()
    rclpy.shutdown()
    print(json.dumps(summary, ensure_ascii=False, indent=2), flush=True)
    return 0


_load_stop = None


def _load_worker(duty_percent):
    global _load_stop
    signal.signal(signal.SIGINT, signal.SIG_IGN)
    cycle_s = 0.1
    busy_s = cycle_s * duty_percent / 100.0
    while not _load_stop.is_set():
        start = time.perf_counter()
        value = 1
        while time.perf_counter() - start < busy_s:
            value = (value * 1103515245 + 12345) & 0x7FFFFFFF
        remaining = cycle_s - (time.perf_counter() - start)
        if remaining > 0:
            time.sleep(remaining)


def run_load(args):
    global _load_stop
    _load_stop = mp.Event()

    def stop_handler(_signum, _frame):
        _load_stop.set()

    signal.signal(signal.SIGINT, stop_handler)
    signal.signal(signal.SIGTERM, stop_handler)
    workers = [mp.Process(target=_load_worker, args=(args.duty,)) for _ in range(args.workers)]
    for worker in workers:
        worker.start()
    print(
        f"[CPU 负载] workers={args.workers}, 每 worker 占空比={args.duty}%, "
        f"pid={os.getpid()}",
        flush=True,
    )
    try:
        while not _load_stop.is_set():
            time.sleep(0.2)
    except KeyboardInterrupt:
        _load_stop.set()
    finally:
        _load_stop.set()
        for worker in workers:
            worker.join(timeout=2.0)
        for worker in workers:
            if worker.is_alive():
                worker.terminate()
                worker.join(timeout=1.0)
    return 0


def build_parser():
    parser = argparse.ArgumentParser(description=__doc__)
    subparsers = parser.add_subparsers(dest="command", required=True)

    gate = subparsers.add_parser("gate", help="Run the Livox CustomMsg gate")
    gate.add_argument("--input-topic", default="/livox/lidar")
    gate.add_argument("--output-topic", default="/validation/livox/lidar")
    gate.add_argument("--service", default="/frlio_validation/lidar_gate")
    gate.add_argument("--status-topic", default="/frlio_validation/lidar_gate_enabled")
    gate.set_defaults(function=run_gate)

    mavros_state = subparsers.add_parser(
        "mavros-state", help="Capture the newest non-historical MAVROS state"
    )
    mavros_state.add_argument("--timeout", type=float, default=2.0)
    mavros_state.add_argument("--wait-connected", action="store_true")
    mavros_state.add_argument("--output", required=True)
    mavros_state.set_defaults(function=run_mavros_state)

    flight = subparsers.add_parser(
        "flight-monitor", help="Gate POSITION entry and alarm on localization faults"
    )
    flight.add_argument("--ready-sec", type=float, default=10.0)
    flight.add_argument("--ready-timeout", type=float, default=120.0)
    flight.add_argument("--odom-timeout", type=float, default=0.15)
    flight.add_argument("--status-timeout", type=float, default=3.0)
    flight.add_argument("--min-odom-rate", type=int, default=150)
    flight.add_argument("--ready-file", required=True)
    flight.add_argument("--alarm-file", required=True)
    flight.add_argument("--stop-file", required=True)
    flight.add_argument("--event-log", required=True)
    flight.set_defaults(function=run_flight_monitor)

    fault = subparsers.add_parser("fault-test", help="Run and verify a LiDAR interruption")
    fault.add_argument("--pause-seconds", type=float, default=1.0)
    fault.add_argument("--odom-quiet-seconds", type=float, default=0.2)
    fault.add_argument("--recovery-timeout", type=float, default=8.0)
    fault.add_argument("--output", required=True)
    fault.set_defaults(function=run_fault_test)

    observe = subparsers.add_parser("observe", help="Observe anchor age for a fixed duration")
    observe.add_argument("--duration", type=float, required=True)
    observe.add_argument("--print-every", type=float, default=30.0)
    observe.add_argument(
        "--lightweight",
        action="store_true",
        help="Do not deserialize IMU/Livox messages in the Python monitor",
    )
    observe.add_argument("--csv", required=True)
    observe.add_argument("--summary", required=True)
    observe.set_defaults(function=run_observe)

    load = subparsers.add_parser("load", help="Generate bounded process CPU load")
    load.add_argument("--workers", type=int, default=4)
    load.add_argument("--duty", type=float, default=75.0)
    load.set_defaults(function=run_load)
    return parser


def main():
    args = build_parser().parse_args()
    if getattr(args, "workers", 1) < 1:
        raise SystemExit("workers must be >= 1")
    if not 0.0 < getattr(args, "duty", 50.0) <= 100.0:
        raise SystemExit("duty must be in (0, 100]")
    return args.function(args)


if __name__ == "__main__":
    raise SystemExit(main())
