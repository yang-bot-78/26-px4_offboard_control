#!/usr/bin/env python3

import importlib.util
import json
import tempfile
import unittest
from pathlib import Path


MONITOR_PATH = (
    Path(__file__).resolve().parents[3] / "tools" / "checks" / "MAVROS状态监视.py"
)
SPEC = importlib.util.spec_from_file_location("mavros_state_monitor", MONITOR_PATH)
MONITOR = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MONITOR)


def snapshot(**changes):
    value = {
        "sequence": 1,
        "received_monotonic_ns": 9_000_000_000,
        "received_wall_ns": 20,
        "connected": True,
        "armed": False,
        "mode": "STABILIZED",
        "system_status": 3,
    }
    value.update(changes)
    return value


class MavrosStateMonitorTest(unittest.TestCase):
    def evaluate(self, value, error=None):
        return MONITOR.evaluate_snapshot(
            value, error, now_monotonic_ns=10_000_000_000, max_age_s=3.0
        )

    def test_fresh_connected_disarmed_non_offboard_is_safe(self):
        result = self.evaluate(snapshot())
        self.assertTrue(result["safe"])
        self.assertEqual(result["reason"], "SAFE")
        self.assertEqual(result["age_ns"], 1_000_000_000)

    def test_missing_invalid_and_stale_fail_closed(self):
        self.assertEqual(self.evaluate(None, "SNAPSHOT_MISSING")["reason"], "SNAPSHOT_MISSING")
        self.assertEqual(self.evaluate(None, "SNAPSHOT_INVALID")["reason"], "SNAPSHOT_INVALID")
        self.assertEqual(
            self.evaluate(snapshot(received_monotonic_ns=6_000_000_000))["reason"],
            "STALE",
        )

    def test_unsafe_vehicle_states_fail_closed(self):
        self.assertEqual(self.evaluate(snapshot(connected=False))["reason"], "DISCONNECTED")
        self.assertEqual(self.evaluate(snapshot(armed=True))["reason"], "ARMED")
        self.assertEqual(self.evaluate(snapshot(mode="OFFBOARD"))["reason"], "OFFBOARD")

    def test_future_snapshot_fails_closed(self):
        result = self.evaluate(snapshot(received_monotonic_ns=10_000_000_001))
        self.assertFalse(result["safe"])
        self.assertEqual(result["reason"], "SNAPSHOT_FROM_FUTURE")

    def test_snapshot_loader_rejects_wrong_boolean_type(self):
        with tempfile.TemporaryDirectory() as directory:
            path = Path(directory) / "state.json"
            path.write_text(json.dumps(snapshot(armed="false")), encoding="utf-8")
            value, error = MONITOR.load_snapshot(path)
        self.assertIsNone(value)
        self.assertEqual(error, "SNAPSHOT_INVALID")

    def test_monitor_handles_external_shutdown_as_normal_teardown(self):
        source = MONITOR_PATH.read_text(encoding="utf-8")
        self.assertIn("from rclpy.executors import ExternalShutdownException", source)
        self.assertIn("except (KeyboardInterrupt, ExternalShutdownException):", source)


if __name__ == "__main__":
    unittest.main()
