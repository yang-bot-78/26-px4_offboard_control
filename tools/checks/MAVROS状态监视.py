#!/usr/bin/env python3
"""落盘 MAVROS 状态，并给出一份新鲜的、失效即拒绝的安全快照。"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from pathlib import Path
from typing import Any


REQUIRED_FIELDS = {
    "sequence",
    "received_monotonic_ns",
    "received_wall_ns",
    "connected",
    "armed",
    "mode",
    "system_status",
}


def atomic_write_json(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(f".{path.name}.tmp.{os.getpid()}")
    with temporary.open("w", encoding="utf-8") as stream:
        json.dump(value, stream, sort_keys=True, separators=(",", ":"))
        stream.write("\n")
        stream.flush()
        os.fsync(stream.fileno())
    os.replace(temporary, path)


def load_snapshot(path: Path) -> tuple[dict[str, Any] | None, str | None]:
    try:
        value = json.loads(path.read_text(encoding="utf-8"))
    except FileNotFoundError:
        return None, "SNAPSHOT_MISSING"
    except (OSError, UnicodeError, json.JSONDecodeError):
        return None, "SNAPSHOT_INVALID"
    if not isinstance(value, dict) or REQUIRED_FIELDS - value.keys():
        return None, "SNAPSHOT_INVALID"
    if type(value["connected"]) is not bool or type(value["armed"]) is not bool:
        return None, "SNAPSHOT_INVALID"
    if not isinstance(value["mode"], str):
        return None, "SNAPSHOT_INVALID"
    try:
        int(value["received_monotonic_ns"])
    except (TypeError, ValueError):
        return None, "SNAPSHOT_INVALID"
    return value, None


def evaluate_snapshot(
    snapshot: dict[str, Any] | None,
    error: str | None,
    *,
    now_monotonic_ns: int,
    max_age_s: float,
) -> dict[str, Any]:
    result: dict[str, Any] = {
        "checked_monotonic_ns": now_monotonic_ns,
        "max_age_ns": int(max_age_s * 1_000_000_000),
        "safe": False,
        "reason": error or "SNAPSHOT_INVALID",
        "age_ns": None,
        "snapshot": snapshot,
    }
    if error or snapshot is None:
        return result

    received_ns = int(snapshot["received_monotonic_ns"])
    age_ns = now_monotonic_ns - received_ns
    result["age_ns"] = age_ns
    if age_ns < 0:
        result["reason"] = "SNAPSHOT_FROM_FUTURE"
    elif snapshot["armed"]:
        result["reason"] = "ARMED"
    elif snapshot["mode"].strip().upper() == "OFFBOARD":
        result["reason"] = "OFFBOARD"
    elif not snapshot["connected"]:
        result["reason"] = "DISCONNECTED"
    elif age_ns > result["max_age_ns"]:
        result["reason"] = "STALE"
    else:
        result["safe"] = True
        result["reason"] = "SAFE"
    return result


def append_jsonl(path: Path, value: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with path.open("a", encoding="utf-8") as stream:
        stream.write(json.dumps(value, sort_keys=True, separators=(",", ":")) + "\n")
        stream.flush()


def check_command(args: argparse.Namespace) -> int:
    snapshot, error = load_snapshot(args.snapshot)
    result = evaluate_snapshot(
        snapshot,
        error,
        now_monotonic_ns=time.monotonic_ns(),
        max_age_s=args.max_age_s,
    )
    if args.samples:
        append_jsonl(args.samples, result)
    print(json.dumps(result, sort_keys=True, separators=(",", ":")))
    return 0 if result["safe"] else 2


def monitor_command(args: argparse.Namespace) -> int:
    import rclpy
    from mavros_msgs.msg import State
    from rclpy.executors import ExternalShutdownException
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

    class StateMonitor(Node):
        def __init__(self) -> None:
            super().__init__("mavros_state_persistent_monitor")
            self.sequence = 0
            self.history = args.history.open("a", encoding="utf-8")
            qos = QoSProfile(
                depth=10,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.VOLATILE,
            )
            self.subscription = self.create_subscription(
                State, args.topic, self.on_state, qos
            )

        def on_state(self, message: State) -> None:
            self.sequence += 1
            record = {
                "sequence": self.sequence,
                "received_monotonic_ns": time.monotonic_ns(),
                "received_wall_ns": time.time_ns(),
                "header_stamp_ns": (
                    int(message.header.stamp.sec) * 1_000_000_000
                    + int(message.header.stamp.nanosec)
                ),
                "connected": bool(message.connected),
                "armed": bool(message.armed),
                "guided": bool(message.guided),
                "manual_input": bool(message.manual_input),
                "mode": str(message.mode),
                "system_status": int(message.system_status),
            }
            self.history.write(
                json.dumps(record, sort_keys=True, separators=(",", ":")) + "\n"
            )
            self.history.flush()
            atomic_write_json(args.snapshot, record)

        def destroy_node(self) -> bool:
            self.history.close()
            return super().destroy_node()

    args.snapshot.parent.mkdir(parents=True, exist_ok=True)
    args.history.parent.mkdir(parents=True, exist_ok=True)
    rclpy.init()
    node = StateMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
    return 0


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser()
    subparsers = parser.add_subparsers(dest="command", required=True)
    monitor = subparsers.add_parser("monitor")
    monitor.add_argument("--topic", default="/mavros/state")
    monitor.add_argument("--snapshot", type=Path, required=True)
    monitor.add_argument("--history", type=Path, required=True)
    monitor.set_defaults(function=monitor_command)
    check = subparsers.add_parser("check")
    check.add_argument("--snapshot", type=Path, required=True)
    check.add_argument("--max-age-s", type=float, required=True)
    check.add_argument("--samples", type=Path)
    check.set_defaults(function=check_command)
    return parser


def main() -> int:
    args = build_parser().parse_args()
    if getattr(args, "max_age_s", 1.0) <= 0:
        print("--max-age-s must be positive", file=sys.stderr)
        return 2
    return args.function(args)


if __name__ == "__main__":
    raise SystemExit(main())
