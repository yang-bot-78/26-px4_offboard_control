#!/usr/bin/env python3

import importlib.util
import math
import pathlib
import types
import unittest

from builtin_interfaces.msg import Time
from geometry_msgs.msg import PoseStamped
from mavros_msgs.msg import State
from nav_msgs.msg import Odometry


SCRIPT_PATH = (
    pathlib.Path(__file__).resolve().parents[1]
    / "scripts"
    / "minipc_mavros_offboard.py"
)
SPEC = importlib.util.spec_from_file_location("minipc_mavros_offboard", SCRIPT_PATH)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)


class _Logger:
    def info(self, _message):
        pass

    def warning(self, _message):
        pass

    def error(self, _message):
        pass

    def fatal(self, _message):
        pass


class _Clock:
    def __init__(self, now_ns):
        self._now_ns = now_ns

    def now(self):
        return types.SimpleNamespace(
            nanoseconds=self._now_ns,
            to_msg=lambda: Time(
                sec=self._now_ns // 1_000_000_000,
                nanosec=self._now_ns % 1_000_000_000,
            ),
        )


class _Future:
    def __init__(self, result=None, done=True):
        self._result = result
        self._done = done
        self.cancelled = False

    def done(self):
        return self._done

    def result(self):
        return self._result

    def cancel(self):
        self.cancelled = True


class EvHealthTrackerTest(unittest.TestCase):
    def test_requires_continuous_healthy_window(self):
        tracker = MODULE.EvHealthTracker(required_s=7.5, freshness_s=1.0)
        for step in range(16):
            tracker.update("HEALTHY", int(step * 0.5e9))

        self.assertTrue(tracker.ready(int(7.5e9)))
        self.assertFalse(tracker.ready(int(8.6e9)))
        self.assertIn("timeout", tracker.unsafe_reason(int(8.6e9)))

    def test_suspect_resets_window_and_recovery_has_hysteresis(self):
        tracker = MODULE.EvHealthTracker(required_s=5.0, freshness_s=1.0)
        for step in range(9):
            tracker.update("HEALTHY", int(step * 0.5e9))
        tracker.update("SUSPECT", int(4.5e9))

        self.assertFalse(tracker.ready(int(4.5e9)))
        self.assertIn("SUSPECT", tracker.unsafe_reason(int(4.5e9)))

        for step in range(10, 20):
            tracker.update("HEALTHY", int(step * 0.5e9))
        self.assertFalse(tracker.ready(int(9.5e9)))
        tracker.update("HEALTHY", int(10.0e9))
        self.assertTrue(tracker.ready(int(10.0e9)))

    def test_invalid_status_is_fault(self):
        tracker = MODULE.EvHealthTracker(required_s=5.0, freshness_s=1.0)
        tracker.update("not-a-state", 0)

        self.assertEqual(MODULE.EvHealthState.FAULT, tracker.state)
        self.assertIn("invalid", tracker.unsafe_reason(0))


class OffboardSafetyHelpersTest(unittest.TestCase):
    def test_auto_land_requires_fresh_matching_mavros_state(self):
        state = State()
        state.mode = "AUTO.LAND"
        fake = types.SimpleNamespace(
            state=state,
            _last_state_ns=1,
            _recent_state_seen=lambda _stamp: True,
        )
        self.assertTrue(MODULE.MiniPcMavrosOffboard._auto_land_active(fake))

        fake._recent_state_seen = lambda _stamp: False
        self.assertFalse(MODULE.MiniPcMavrosOffboard._auto_land_active(fake))
        fake._recent_state_seen = lambda _stamp: True
        state.mode = "OFFBOARD"
        self.assertFalse(MODULE.MiniPcMavrosOffboard._auto_land_active(fake))

    def test_rejected_auto_land_keeps_heartbeat_and_retries_without_exit(self):
        future = _Future(result=types.SimpleNamespace(mode_sent=False))
        heartbeats = []
        fake = types.SimpleNamespace(
            phase=MODULE.Phase.REQUEST_LAND,
            _mode_future=future,
            _land_mode_request_ns=900_000_000,
            _last_land_mode_attempt_ns=900_000_000,
            land_mode_retry_s=1.0,
            land_mode_confirm_timeout_s=2.0,
            _handle_ev_health_flight_safety=lambda: False,
            _enforce_safety_land_latch=lambda: None,
            _handle_takeoff_transient_safety=lambda: False,
            _log_phase=lambda _text: None,
            _publish_land_handoff_setpoint=lambda: heartbeats.append(True),
            _safety_land_latched=True,
            _safety_descent_enabled=True,
            _auto_land_active=lambda: False,
            get_clock=lambda: _Clock(1_000_000_000),
            get_logger=lambda: _Logger(),
        )

        MODULE.MiniPcMavrosOffboard._timer_cb(fake)
        self.assertEqual([True], heartbeats)
        self.assertIsNone(fake._mode_future)
        self.assertEqual(MODULE.Phase.REQUEST_LAND, fake.phase)

    def test_auto_land_wait_keeps_heartbeat_until_state_confirmation(self):
        heartbeats = []
        confirmations = []
        fake = types.SimpleNamespace(
            phase=MODULE.Phase.WAIT_FOR_AUTO_LAND,
            _land_mode_accept_ns=900_000_000,
            land_mode_confirm_timeout_s=2.0,
            _handle_ev_health_flight_safety=lambda: False,
            _enforce_safety_land_latch=lambda: None,
            _handle_takeoff_transient_safety=lambda: False,
            _log_phase=lambda _text: None,
            _publish_land_handoff_setpoint=lambda: heartbeats.append(True),
            _safety_land_latched=True,
            _safety_descent_enabled=True,
            _auto_land_active=lambda: True,
            _mark_auto_land_active=lambda: confirmations.append(True),
            get_clock=lambda: _Clock(1_000_000_000),
            get_logger=lambda: _Logger(),
        )

        MODULE.MiniPcMavrosOffboard._timer_cb(fake)
        self.assertEqual([True], heartbeats)
        self.assertEqual([True], confirmations)

    def test_horizontal_target_locks_once_at_first_offboard_activation(self):
        fake = types.SimpleNamespace()
        fake._horizontal_target_locked = False
        fake._target_pose = PoseStamped()
        fake._reference_pose = PoseStamped()
        fake.local_odom = Odometry()
        fake.local_odom.pose.pose.position.x = 1.25
        fake.local_odom.pose.pose.position.y = -0.75
        fake._got_odom = True
        fake._local_odom_fresh = lambda: True
        fake._current_offboard_state_fresh = lambda: True
        fake.target_x_m = 0.0
        fake.target_y_m = 0.0
        fake.target_z_m = -0.5
        fake._final_target_z = 0.5
        fake._target_position_ned = None
        fake.get_logger = lambda: _Logger()
        fake._update_takeoff_reference_from_current = lambda current: (
            MODULE.MiniPcMavrosOffboard._update_takeoff_reference_from_current(
                fake, current
            )
        )

        locked = MODULE.MiniPcMavrosOffboard._lock_horizontal_target_at_offboard(fake)
        self.assertTrue(locked)
        self.assertEqual(1.25, fake._target_pose.pose.position.x)
        self.assertEqual(-0.75, fake._target_pose.pose.position.y)

        fake.local_odom.pose.pose.position.x = 9.0
        fake.local_odom.pose.pose.position.y = 8.0
        MODULE.MiniPcMavrosOffboard._lock_horizontal_target_at_offboard(fake)
        self.assertEqual(1.25, fake._target_pose.pose.position.x)
        self.assertEqual(-0.75, fake._target_pose.pose.position.y)

    def test_unlocked_prestream_tracks_fresh_xyz_without_mode_switch_jump(self):
        fake = types.SimpleNamespace(
            _horizontal_target_locked=False,
            _target_pose=PoseStamped(),
            _reference_pose=PoseStamped(),
            local_odom=Odometry(),
            state=State(),
            target_x_m=0.0,
            target_y_m=0.0,
            target_z_m=-0.5,
            arm_only=False,
            takeoff_height_m=0.8,
            _final_target_z=0.0,
            _target_position_ned=None,
            _local_odom_fresh=lambda: True,
            _current_offboard_state_fresh=lambda: True,
            _got_odom=True,
            get_logger=lambda: _Logger(),
        )
        fake._update_takeoff_reference_from_current = lambda current: (
            MODULE.MiniPcMavrosOffboard._update_takeoff_reference_from_current(
                fake, current
            )
        )
        fake.local_odom.pose.pose.position.x = 1.0
        fake.local_odom.pose.pose.position.y = -2.0
        fake.local_odom.pose.pose.position.z = 0.3

        MODULE.MiniPcMavrosOffboard._refresh_unlocked_prestream_target(fake)
        before_lock = (
            fake._target_pose.pose.position.x,
            fake._target_pose.pose.position.y,
            fake._target_pose.pose.position.z,
            fake._final_target_z,
        )
        self.assertEqual((1.0, -2.0, 0.3, 0.8), before_lock)

        self.assertTrue(
            MODULE.MiniPcMavrosOffboard._lock_horizontal_target_at_offboard(fake)
        )
        after_lock = (
            fake._target_pose.pose.position.x,
            fake._target_pose.pose.position.y,
            fake._target_pose.pose.position.z,
            fake._final_target_z,
        )
        self.assertEqual(before_lock, after_lock)

    def test_suspect_during_flight_requests_health_landing(self):
        tracker = MODULE.EvHealthTracker(required_s=5.0, freshness_s=1.0)
        tracker.update("SUSPECT", 1_000_000_000)
        calls = []
        fake = types.SimpleNamespace(
            require_ev_health=True,
            phase=MODULE.Phase.HOLD,
            _ev_health=tracker,
            _flight_started=True,
            get_clock=lambda: _Clock(1_000_000_000),
            _armed_active=lambda: True,
            _begin_ev_health_land=lambda reason: calls.append(reason),
        )

        handled = MODULE.MiniPcMavrosOffboard._handle_ev_health_flight_safety(fake)
        self.assertTrue(handled)
        self.assertEqual(1, len(calls))
        self.assertIn("SUSPECT", calls[0])

    def test_fault_during_pending_arm_latches_no_climb_without_land_race(self):
        tracker = MODULE.EvHealthTracker(required_s=5.0, freshness_s=1.0)
        tracker.update("FAULT", 1_000_000_000)
        calls = []
        fake = types.SimpleNamespace(
            require_ev_health=True,
            phase=MODULE.Phase.REQUEST_ARM,
            _arm_future=object(),
            _ev_health=tracker,
            _flight_started=False,
            get_clock=lambda: _Clock(1_000_000_000),
            _armed_active=lambda: False,
            _begin_ev_health_land=lambda reason: calls.append(reason),
            _capture_abort_hold_setpoint=lambda: True,
            _publish_abort_hold_setpoint=lambda: None,
            get_logger=lambda: _Logger(),
        )

        handled = MODULE.MiniPcMavrosOffboard._handle_ev_health_flight_safety(fake)
        self.assertTrue(handled)
        self.assertEqual([], calls)
        self.assertEqual(MODULE.Phase.EV_ABORT_HOLD, fake.phase)

    def test_pending_arm_abort_requests_land_only_after_armed_state(self):
        tracker = MODULE.EvHealthTracker(required_s=5.0, freshness_s=1.0)
        tracker.update("FAULT", 1_000_000_000)
        calls = []
        fake = types.SimpleNamespace(
            require_ev_health=True,
            phase=MODULE.Phase.EV_ABORT_HOLD,
            _ev_health=tracker,
            get_clock=lambda: _Clock(1_000_000_000),
            _armed_active=lambda: True,
            _begin_ev_health_land=lambda reason: calls.append(reason),
        )

        handled = MODULE.MiniPcMavrosOffboard._handle_ev_health_flight_safety(fake)
        self.assertTrue(handled)
        self.assertEqual(1, len(calls))
        self.assertIn("FAULT", calls[0])

    @staticmethod
    def _make_drift_fake(error_m=0.0):
        fake = types.SimpleNamespace(
            now_ns=0,
            error_m=error_m,
            drift_guard_enabled=True,
            drift_warning_m=0.2,
            drift_land_m=0.5,
            drift_emergency_m=0.6,
            drift_land_trigger_duration_s=0.3,
            drift_emergency_trigger_duration_s=0.1,
            _drift_warning_logged=False,
            _drift_guard_triggered=False,
            _drift_emergency_logged=False,
            _safety_condition_since_ns={},
            _safety_land_latched=False,
            _safety_land_reason="",
            _safety_descent_enabled=False,
            _safety_descent_start_ns=None,
            _safety_descent_start_z=None,
            _abort_hold_xyz=(0.0, 0.0, 0.5),
            _flight_started=True,
            phase=MODULE.Phase.HOLD,
            _mode_future=object(),
            _land_mode_request_ns=1,
            _land_mode_accept_ns=1,
            land_requests=[],
            get_clock=lambda: _Clock(fake.now_ns),
            get_logger=lambda: _Logger(),
            _horizontal_error_from_target=lambda: fake.error_m,
            _armed_active=lambda: True,
            _capture_abort_hold_setpoint=lambda: True,
            _publish_land_handoff_setpoint=lambda: fake.land_requests.append(
                fake._safety_land_reason
            ),
        )
        fake._begin_safety_land = lambda reason, descent_enabled: (
            MODULE.MiniPcMavrosOffboard._begin_safety_land(
                fake, reason, descent_enabled=descent_enabled
            )
        )
        fake._start_drift_safety_land = lambda error, emergency: (
            MODULE.MiniPcMavrosOffboard._start_drift_safety_land(
                fake, error, emergency=emergency
            )
        )
        fake._enforce_safety_land_latch = lambda: (
            MODULE.MiniPcMavrosOffboard._enforce_safety_land_latch(fake)
        )
        fake._condition_sustained = MODULE.MiniPcMavrosOffboard._condition_sustained
        return fake

    def test_below_drift_land_threshold_does_not_land(self):
        fake = self._make_drift_fake(0.49)
        for now_ns in (0, 200_000_000, 600_000_000):
            fake.now_ns = now_ns
            self.assertTrue(
                MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake)
            )

        self.assertFalse(fake._safety_land_latched)
        self.assertEqual(MODULE.Phase.HOLD, fake.phase)

    def test_sustained_drift_land_threshold_requests_auto_land_once(self):
        fake = self._make_drift_fake(0.51)

        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        fake.now_ns = 200_000_000
        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        fake.now_ns = 310_000_000
        self.assertFalse(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))

        self.assertTrue(fake._safety_land_latched)
        self.assertEqual(MODULE.Phase.REQUEST_LAND, fake.phase)
        self.assertEqual(1, len(fake.land_requests))

        fake.now_ns = 700_000_000
        self.assertFalse(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        self.assertEqual(1, len(fake.land_requests))

    def test_single_frame_drift_excursion_does_not_land(self):
        fake = self._make_drift_fake(0.51)
        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        fake.now_ns = 50_000_000
        fake.error_m = 0.1
        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        fake.now_ns = 400_000_000
        fake.error_m = 0.51
        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        self.assertFalse(fake._safety_land_latched)

    def test_safety_landing_latch_prevents_return_to_hold(self):
        fake = self._make_drift_fake(0.51)
        fake._safety_land_latched = True
        fake.phase = MODULE.Phase.HOLD

        MODULE.MiniPcMavrosOffboard._enforce_safety_land_latch(fake)

        self.assertEqual(MODULE.Phase.REQUEST_LAND, fake.phase)

    def test_emergency_drift_requests_land_without_airborne_disarm(self):
        fake = self._make_drift_fake(0.61)
        disarm_calls = []
        fake._request_disarm = lambda: disarm_calls.append(True)

        self.assertTrue(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))
        fake.now_ns = 110_000_000
        self.assertFalse(MODULE.MiniPcMavrosOffboard._drift_guard_allows_hold(fake))

        self.assertEqual(MODULE.Phase.REQUEST_LAND, fake.phase)
        self.assertTrue(fake._drift_emergency_logged)
        self.assertEqual([], disarm_calls)

    def test_safety_handoff_setpoint_descends_while_mode_retry_waits(self):
        published = []
        fake = types.SimpleNamespace(
            now_ns=1_000_000_000,
            _target_pose=PoseStamped(),
            _reference_pose=PoseStamped(),
            _abort_hold_xyz=(1.0, -2.0, 0.5),
            _safety_descent_start_ns=0,
            _safety_descent_start_z=0.5,
            offboard_land_speed_mps=0.1,
            get_clock=lambda: _Clock(fake.now_ns),
            _capture_abort_hold_setpoint=lambda: True,
            _current_yaw=lambda: 0.0,
            _publish_setpoint=lambda msg: published.append(msg),
        )
        fake._reference_pose.pose.position.z = 0.0

        MODULE.MiniPcMavrosOffboard._publish_safety_descent_setpoint(fake)

        self.assertEqual(1, len(published))
        self.assertAlmostEqual(1.0, published[0].position.x)
        self.assertAlmostEqual(-2.0, published[0].position.y)
        self.assertAlmostEqual(0.4, published[0].position.z)

    @staticmethod
    def _make_takeoff_transient_fake():
        fake = OffboardSafetyHelpersTest._make_drift_fake(0.0)
        fake.takeoff_transient_guard_enabled = True
        fake.takeoff_transient_window_s = 3.0
        fake.takeoff_transient_trigger_duration_s = 0.2
        fake.takeoff_transient_max_displacement_m = 0.25
        fake.takeoff_transient_max_speed_mps = 0.45
        fake.takeoff_transient_max_roll_deg = 8.0
        fake.takeoff_transient_max_pitch_deg = 8.0
        fake._hold_start_ns = 0
        fake.require_ev_health = False
        fake._got_odom = True
        fake.local_odom = Odometry()
        fake.local_odom.pose.pose.orientation.w = 1.0
        fake._target_pose = PoseStamped()
        fake._takeoff_transient_metrics = lambda: (
            MODULE.MiniPcMavrosOffboard._takeoff_transient_metrics(fake)
        )
        fake._begin_ev_health_land = lambda _reason: None
        return fake

    def test_takeoff_transient_sustained_displacement_latches_land(self):
        fake = self._make_takeoff_transient_fake()
        fake.error_m = 0.26

        self.assertFalse(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        fake.now_ns = 210_000_000
        self.assertTrue(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )

        self.assertTrue(fake._safety_land_latched)
        self.assertEqual(MODULE.Phase.REQUEST_LAND, fake.phase)

    def test_takeoff_transient_single_speed_sample_does_not_land(self):
        fake = self._make_takeoff_transient_fake()
        fake.local_odom.twist.twist.linear.x = 0.5

        self.assertFalse(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        fake.now_ns = 50_000_000
        fake.local_odom.twist.twist.linear.x = 0.0
        self.assertFalse(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        fake.now_ns = 400_000_000
        fake.local_odom.twist.twist.linear.x = 0.5
        self.assertFalse(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        self.assertFalse(fake._safety_land_latched)

    def test_takeoff_transient_sustained_pitch_latches_land(self):
        fake = self._make_takeoff_transient_fake()
        pitch_rad = math.radians(9.0)
        fake.local_odom.pose.pose.orientation.y = math.sin(pitch_rad / 2.0)
        fake.local_odom.pose.pose.orientation.w = math.cos(pitch_rad / 2.0)

        self.assertFalse(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        fake.now_ns = 210_000_000
        self.assertTrue(
            MODULE.MiniPcMavrosOffboard._handle_takeoff_transient_safety(fake)
        )
        self.assertIn("pitch_deg", fake._safety_land_reason)

    def test_normal_planned_landing_path_is_unchanged(self):
        fake = types.SimpleNamespace(
            _safety_land_latched=False,
            offboard_land=True,
            auto_land=False,
            phase=MODULE.Phase.HOLD,
        )

        MODULE.MiniPcMavrosOffboard._go_to_post_hover_landing(fake)
        self.assertEqual(MODULE.Phase.OFFBOARD_LAND, fake.phase)


if __name__ == "__main__":
    unittest.main()
