#!/usr/bin/env python3
"""End-to-end B/C fault-isolation checks for the prop-off hardware chain."""

import argparse
import json
import math
import time
from pathlib import Path


def ros_imports():
    import rclpy
    from diagnostic_msgs.msg import DiagnosticArray
    from nav_msgs.msg import Odometry
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import Bool, Float64, String
    from std_srvs.srv import SetBool

    return locals()


def qos(ros, reliable=False, transient=False, depth=20):
    return ros["QoSProfile"](
        reliability=(ros["ReliabilityPolicy"].RELIABLE if reliable
                     else ros["ReliabilityPolicy"].BEST_EFFORT),
        durability=(ros["DurabilityPolicy"].TRANSIENT_LOCAL if transient
                    else ros["DurabilityPolicy"].VOLATILE),
        history=ros["HistoryPolicy"].KEEP_LAST,
        depth=depth,
    )


class Monitor:
    def __init__(self, ros):
        self.ros = ros
        self.node = ros["rclpy"].create_node("frlio_fault_isolation_monitor")
        self.started = time.monotonic()
        self.status = None
        self.status_events = []
        self.health_events = []
        self.ev_usable = None
        self.planner_usable = None
        self.predictor_age = None
        self.odometry_count = 0
        self.last_odom = None
        self.max_odom_gap = 0.0
        self.last_odom_stamp = None
        self.max_odom_stamp_gap = 0.0
        self._subscriptions = [
            self.node.create_subscription(
                ros["String"], "/frlio/high_rate_odom/status", self._status,
                qos(ros, reliable=True, transient=True, depth=1)),
            self.node.create_subscription(
                ros["Bool"], "/frlio/high_rate_odom/ev_usable", self._ev,
                qos(ros, reliable=True, depth=5)),
            self.node.create_subscription(
                ros["Bool"], "/frlio/high_rate_odom/planner_usable", self._planner,
                qos(ros, reliable=True, depth=5)),
            self.node.create_subscription(
                ros["Float64"], "/frlio/high_rate_odom/predictor_age", self._predictor,
                qos(ros, reliable=False, depth=20)),
            self.node.create_subscription(
                ros["Odometry"], "/Odometry", self._odom,
                qos(ros, reliable=False, depth=50)),
            self.node.create_subscription(
                ros["DiagnosticArray"], "/frlio/high_rate_odom/localization_health",
                self._health, qos(ros, reliable=False, depth=20)),
        ]

    def elapsed(self):
        return time.monotonic() - self.started

    def spin(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.ros["rclpy"].spin_once(self.node, timeout_sec=0.02)

    def wait(self, predicate, timeout):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            self.ros["rclpy"].spin_once(self.node, timeout_sec=0.02)
            if predicate():
                return True
        return bool(predicate())

    def service(self, name, enabled):
        client = self.node.create_client(self.ros["SetBool"], name)
        if not client.wait_for_service(timeout_sec=5.0):
            raise RuntimeError(f"service unavailable: {name}")
        request = self.ros["SetBool"].Request()
        request.data = enabled
        future = client.call_async(request)
        deadline = time.monotonic() + 5.0
        while time.monotonic() < deadline and not future.done():
            self.ros["rclpy"].spin_once(self.node, timeout_sec=0.02)
        if not future.done() or future.result() is None or not future.result().success:
            raise RuntimeError(f"service call failed: {name}={enabled}")
        return future.result().message

    def _status(self, message):
        value = message.data
        if value != self.status:
            self.status = value
            self.status_events.append({
                "elapsed_s": self.elapsed(),
                "status": value,
                "ev_usable": self.ev_usable,
                "planner_usable": self.planner_usable,
                "predictor_age_s": self.predictor_age,
            })
            print(
                f"[状态] {value} ev={self.ev_usable} planner={self.planner_usable} "
                f"predictor_age={self.predictor_age}", flush=True)

    def _ev(self, message):
        self.ev_usable = bool(message.data)

    def _planner(self, message):
        self.planner_usable = bool(message.data)

    def _predictor(self, message):
        self.predictor_age = float(message.data)

    def _odom(self, message):
        now = time.monotonic()
        if self.last_odom is not None:
            self.max_odom_gap = max(self.max_odom_gap, now - self.last_odom)
        self.last_odom = now
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        if self.last_odom_stamp is not None and stamp > self.last_odom_stamp:
            self.max_odom_stamp_gap = max(
                self.max_odom_stamp_gap, stamp - self.last_odom_stamp)
        self.last_odom_stamp = stamp
        self.odometry_count += 1

    def _health(self, message):
        for status in message.status:
            if status.name != "frlio_localization_health":
                continue
            values = {item.key: item.value for item in status.values}
            event = {"elapsed_s": self.elapsed(), **values}
            self.health_events.append(event)

    def close(self):
        self.node.destroy_node()


def common_ready(monitor, timeout=15.0):
    return monitor.wait(
        lambda: monitor.status == "HEALTHY"
        and monitor.ev_usable is True
        and monitor.predictor_age is not None
        and math.isfinite(monitor.predictor_age)
        and monitor.predictor_age < 0.15
        and monitor.odometry_count >= 30,
        timeout,
    )


def health_summary(monitor):
    return {
        "status_events": monitor.status_events,
        "health_events": monitor.health_events,
        "final_status": monitor.status,
        "final_ev_usable": monitor.ev_usable,
        "final_planner_usable": monitor.planner_usable,
        "final_predictor_age_s": monitor.predictor_age,
        "odometry_count": monitor.odometry_count,
        "max_odom_gap_s": monitor.max_odom_gap,
        "max_odom_stamp_gap_s": monitor.max_odom_stamp_gap,
    }


def run_backend_delay(args, monitor):
    result = {"passed": False, "mode": "backend_delay", "checks": {}}
    if not common_ready(monitor):
        raise RuntimeError("precondition failed: chain did not reach HEALTHY with EV usable")
    baseline_odom = monitor.odometry_count
    monitor.max_odom_gap = 0.0
    monitor.last_odom = time.monotonic()
    monitor.max_odom_stamp_gap = 0.0
    monitor.last_odom_stamp = None
    baseline_events = len(monitor.status_events)
    baseline_health = len(monitor.health_events)
    start = time.monotonic()
    delay_message = monitor.service(args.delay_service, True)
    monitor.spin(args.fault_seconds)
    stop = time.monotonic()
    disable_message = monitor.service(args.delay_service, False)
    recovered = monitor.wait(
        lambda: monitor.status == "HEALTHY"
        and monitor.ev_usable is True
        and monitor.planner_usable is True
        and monitor.predictor_age is not None
        and monitor.predictor_age < 0.15,
        12.0,
    )
    fault_events = monitor.status_events[baseline_events:]
    health_events = monitor.health_events[baseline_health:]
    window_health = [
        event for event in health_events
        if start <= monitor.started + event["elapsed_s"] <= stop
    ]
    statuses = {event["status"] for event in fault_events}
    ev_values = [event.get("ev_usable") for event in window_health]
    planner_values = [event.get("planner_usable") for event in window_health]
    predictor_values = []
    for event in window_health:
        try:
            predictor_values.append(float(event["predictor_age_s"]))
        except (KeyError, TypeError, ValueError):
            pass
    result.update({
        "delay_ms": args.delay_ms,
        "fault_seconds": args.fault_seconds,
        "delay_service_response": delay_message,
        "restore_service_response": disable_message,
        "checks": {
            "saw_degraded_or_planner_status": bool(
                statuses & {"SUSPECT_STALE_LIDAR", "FAULT_STALE_LIDAR"}),
            "never_state_unusable": "FAULT_STATE_UNUSABLE" not in statuses,
            "ev_usable_stayed_true": bool(ev_values) and all(value == "true" for value in ev_values),
            "predictor_age_stayed_below_150ms": bool(predictor_values)
            and max(predictor_values) < 0.15,
            "odom_continued": monitor.odometry_count - baseline_odom >= 20,
            "odom_stamp_gap_under_150ms": monitor.max_odom_stamp_gap < 0.15,
            "recovered_healthy": recovered,
            "planner_axis_is_independent": all(
                event.get("ev_usable") != "false" or event.get("planner_usable") == "false"
                for event in window_health),
        },
    })
    result.update(health_summary(monitor))
    result["passed"] = all(result["checks"].values())
    return result


def run_predictor_stop(args, monitor):
    result = {"passed": False, "mode": "predictor_stop", "checks": {}}
    if not common_ready(monitor):
        raise RuntimeError("precondition failed: chain did not reach initialized HEALTHY state")
    baseline_generation = 0
    for event in monitor.health_events:
        try:
            baseline_generation = max(baseline_generation, int(event.get("predictor_generation", 0)))
        except (TypeError, ValueError):
            pass
    baseline_events = len(monitor.status_events)
    baseline_health = len(monitor.health_events)
    baseline_odom = monitor.odometry_count
    stopped_at = time.monotonic()
    pause_message = monitor.service(args.imu_service, False)
    saw_l3 = monitor.wait(
        lambda: any(
            event.get("level") == "STATE_UNUSABLE"
            and event.get("ev_usable") == "false"
            and float(event.get("predictor_age_s", "0")) >= 0.15
            for event in monitor.health_events[baseline_health:]
        ),
        4.0,
    )
    monitor.spin(max(0.0, args.stop_seconds - (time.monotonic() - stopped_at)))
    odom_during_stop = monitor.odometry_count - baseline_odom
    restore_message = monitor.service(args.imu_service, True)
    recovered = monitor.wait(
        lambda: monitor.status == "HEALTHY"
        and monitor.ev_usable is True
        and monitor.planner_usable is True
        and monitor.predictor_age is not None
        and monitor.predictor_age < 0.10,
        12.0,
    )
    if recovered:
        monitor.spin(0.35)
    recovery_health = monitor.health_events[baseline_health:]
    generations = []
    reasons = []
    for event in recovery_health:
        try:
            generations.append(int(event.get("predictor_generation", 0)))
        except (TypeError, ValueError):
            pass
        reasons.append(event.get("reason"))
    healthy_after_l3 = [
        event for event in recovery_health
        if event.get("level") == "GOOD"
        and event.get("ev_usable") == "true"
        and event.get("predictor_age_s") is not None
        and float(event["predictor_age_s"]) < 0.10
    ]
    result.update({
        "stop_seconds": args.stop_seconds,
        "pause_service_response": pause_message,
        "restore_service_response": restore_message,
        "baseline_predictor_generation": baseline_generation,
        "odometry_samples_during_stop": odom_during_stop,
        "checks": {
            "initialized_before_stop": baseline_generation > 0,
            "saw_state_unusable": saw_l3,
            "ev_usable_false_during_stop": any(
                event.get("ev_usable") is False
                for event in monitor.status_events[baseline_events:]),
            "predictor_stale_reason_observed": "predictor_stale" in reasons,
            "predictor_generation_advanced": bool(generations)
            and max(generations) > baseline_generation,
            "recovered_healthy": recovered,
            "recovery_has_three_healthy_events": len(healthy_after_l3) >= 3,
            "odom_not_claimed_healthy_during_l3": odom_during_stop < 10,
        },
    })
    result.update(health_summary(monitor))
    result["passed"] = all(result["checks"].values())
    return result


def main():
    parser = argparse.ArgumentParser()
    sub = parser.add_subparsers(dest="mode", required=True)
    backend = sub.add_parser("backend-delay")
    backend.add_argument("--delay-service", required=True)
    backend.add_argument("--delay-ms", type=int, required=True)
    backend.add_argument("--fault-seconds", type=float, default=4.0)
    backend.add_argument("--output", required=True)
    backend.set_defaults(runner=run_backend_delay)
    predictor = sub.add_parser("predictor-stop")
    predictor.add_argument("--imu-service", required=True)
    predictor.add_argument("--stop-seconds", type=float, default=1.0)
    predictor.add_argument("--output", required=True)
    predictor.set_defaults(runner=run_predictor_stop)
    args = parser.parse_args()

    ros = ros_imports()
    ros["rclpy"].init()
    monitor = Monitor(ros)
    try:
        result = args.runner(args, monitor)
    except Exception as exc:
        result = {"passed": False, "mode": args.mode, "error": str(exc),
                  **health_summary(monitor)}
    finally:
        monitor.close()
        if ros["rclpy"].ok():
            ros["rclpy"].shutdown()
    output = Path(args.output)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n")
    print(json.dumps(result, ensure_ascii=False, indent=2), flush=True)
    return 0 if result.get("passed") else 1


if __name__ == "__main__":
    raise SystemExit(main())
