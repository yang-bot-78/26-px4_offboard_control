#!/usr/bin/env python3
"""Collect four RViz points, then execute the fixed A/B/C/D race sequence."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from enum import Enum
from pathlib import Path
from typing import Any, Optional

import yaml


MIN_POINT_SEPARATION_M = 0.05
POINT_NAMES = ('b1', 'b2', 'c', 'd')


class MissionConfigError(ValueError):
    """Raised when mission settings or an RViz point are unsafe to use."""


class MissionState(str, Enum):
    IDLE = 'IDLE'
    COLLECTING_POINTS = 'COLLECTING_POINTS'
    WAIT_TAKEOFF = 'WAIT_TAKEOFF'
    GOTO_B1 = 'GOTO_B1'
    GOTO_B2 = 'GOTO_B2'
    HOLD_B2 = 'HOLD_B2'
    GOTO_C = 'GOTO_C'
    GOTO_D = 'GOTO_D'
    HOLD_D = 'HOLD_D'
    MISSION_DONE = 'MISSION_DONE'


@dataclass(frozen=True)
class MissionSettings:
    arrive_radius: float
    b2_hold_sec: float
    d_hold_sec: float
    # Optional preset B1/B2/C/D waypoints in the map frame.
    #
    # Originally the four points could only arrive by clicking in RViz, which
    # means no autonomous run is possible without an operator at a screen. A
    # preset list lets the race sequence start on its own; clicking in RViz
    # still works and is the way to override a preset on the day.
    preset_points: tuple = ()


@dataclass(frozen=True)
class Point:
    name: str
    x: float
    y: float


@dataclass(frozen=True)
class SequencerAction:
    """A state change that the ROS wrapper must publish to the outside world."""

    goal: Optional[Point] = None
    zone: Optional[str] = None
    b_recognition: Optional[bool] = None
    d_recognition: Optional[bool] = None
    event: str = ''
    mission_done: bool = False


def horizontal_distance(x0: float, y0: float, x1: float, y1: float) -> float:
    return math.hypot(x1 - x0, y1 - y0)


def point_is_reached(x_enu: float, y_enu: float, point: Point, radius: float) -> bool:
    """Return true at the radius boundary as well as inside it."""
    return horizontal_distance(x_enu, y_enu, point.x, point.y) <= radius


def control_state_from_message(message: str) -> str:
    """Extract state=VALUE from the established /race/control/status payload."""
    for field in message.split():
        if field.startswith('state='):
            return field[len('state='):]
    return ''


def _finite_number(
        value: Any, description: str, *, minimum: Optional[float] = None,
        strict_minimum: bool = False) -> float:
    """Validate a finite scalar, optionally enforcing an inclusive lower bound."""
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigError(f'{description} must be a finite number')
    number = float(value)
    if not math.isfinite(number):
        raise MissionConfigError(f'{description} must be a finite number')
    if minimum is not None:
        invalid = number <= minimum if strict_minimum else number < minimum
        if invalid:
            comparator = '>' if strict_minimum else '>='
            raise MissionConfigError(f'{description} must be {comparator} {minimum:g}')
    return number


def load_settings(path: str | Path) -> MissionSettings:
    """Load hold and radius settings; coordinates always come from RViz."""
    settings_path = Path(path)
    try:
        with settings_path.open('r', encoding='utf-8') as stream:
            document = yaml.safe_load(stream)
    except (OSError, yaml.YAMLError) as error:
        raise MissionConfigError(
            f'failed to load mission file {settings_path}: {error}') from error
    if not isinstance(document, dict):
        raise MissionConfigError('mission YAML root must be a mapping')
    return MissionSettings(
        arrive_radius=_finite_number(
            document.get('arrive_radius', 0.40), 'arrive_radius', minimum=0.0,
            strict_minimum=True),
        b2_hold_sec=_finite_number(
            document.get('b2_hold_sec', 3.0), 'b2_hold_sec', minimum=0.0),
        d_hold_sec=_finite_number(
            document.get('d_hold_sec', 3.0), 'd_hold_sec', minimum=0.0),
        preset_points=_load_preset_points(document.get('preset_points')),
    )


def _load_preset_points(raw) -> tuple:
    """Parse optional preset waypoints; an empty result means "collect from RViz"."""
    if raw is None:
        return ()
    if not isinstance(raw, list):
        raise MissionConfigError('preset_points must be a list of [x, y] pairs')
    if len(raw) != len(POINT_NAMES):
        raise MissionConfigError(
            f'preset_points needs exactly {len(POINT_NAMES)} entries '
            f'({", ".join(POINT_NAMES)}), got {len(raw)}')
    points = []
    for index, entry in enumerate(raw):
        name = POINT_NAMES[index]
        if not isinstance(entry, (list, tuple)) or len(entry) != 2:
            raise MissionConfigError(f'preset_points[{name}] must be [x, y]')
        x = _finite_number(entry[0], f'preset_points[{name}].x')
        y = _finite_number(entry[1], f'preset_points[{name}].y')
        # Same separation rule the RViz path enforces: two points closer than
        # this are almost certainly a mis-entry, and would make "arrived"
        # ambiguous between consecutive legs.
        if points:
            previous = points[-1]
            separation = horizontal_distance(previous.x, previous.y, x, y)
            if separation <= MIN_POINT_SEPARATION_M:
                raise MissionConfigError(
                    f'preset_points[{name}] is {separation:.3f} m from '
                    f'{previous.name}; points must be > '
                    f'{MIN_POINT_SEPARATION_M:.2f} m apart')
        points.append(Point(name, x, y))
    return tuple(points)


class MissionSequencer:
    """Pure four-point mission state machine, independent from ROS messages."""

    def __init__(self, settings: MissionSettings):
        self._settings = settings
        self.state = MissionState.IDLE
        self.zone = 'A'
        self.b_recognition = False
        self.d_recognition = False
        # Presets seed the point list so the run needs no operator at RViz.
        self._points: list[Point] = list(settings.preset_points)
        self._takeoff_ready = False
        self._hold_started_at: Optional[float] = None

    @property
    def using_preset_points(self) -> bool:
        return bool(self._settings.preset_points)

    def clear_points(self) -> None:
        """Drop the collected points so RViz can replace a preset on the day."""
        self._points = []
        self._hold_started_at = None
        if self.state in (MissionState.IDLE, MissionState.COLLECTING_POINTS,
                          MissionState.WAIT_TAKEOFF):
            self.state = MissionState.COLLECTING_POINTS

    @property
    def collected_count(self) -> int:
        return len(self._points)

    @property
    def points(self) -> tuple[Point, ...]:
        return tuple(self._points)

    @property
    def current_point(self) -> Optional[Point]:
        index_by_state = {
            MissionState.GOTO_B1: 0,
            MissionState.GOTO_B2: 1,
            MissionState.HOLD_B2: 1,
            MissionState.GOTO_C: 2,
            MissionState.GOTO_D: 3,
            MissionState.HOLD_D: 3,
        }
        index = index_by_state.get(self.state)
        return self._points[index] if index is not None else None

    def start(self) -> None:
        if self.state != MissionState.IDLE:
            return
        # With presets the four points already exist, so skip straight to
        # waiting for takeoff instead of waiting for RViz clicks that will
        # never come.
        if self.collected_count == len(POINT_NAMES):
            self.state = MissionState.WAIT_TAKEOFF
        else:
            self.state = MissionState.COLLECTING_POINTS

    def add_rviz_point(self, x: float, y: float) -> Point:
        """Record the next B1/B2/C/D point before the flight begins."""
        # A click while presets are still untouched and the flight has not begun
        # means the operator is replacing the preset course. Clear it and start
        # collecting, rather than rejecting the click as "already have four".
        if (self.state == MissionState.WAIT_TAKEOFF and
                self.collected_count == len(POINT_NAMES)):
            self.clear_points()
        if self.state != MissionState.COLLECTING_POINTS:
            raise MissionConfigError(
                'mission already has four points; reset before collecting again')
        x = _finite_number(x, 'RViz point x')
        y = _finite_number(y, 'RViz point y')
        if self._points:
            previous = self._points[-1]
            separation = horizontal_distance(previous.x, previous.y, x, y)
            if separation <= MIN_POINT_SEPARATION_M:
                raise MissionConfigError(
                    f'RViz point is {separation:.3f} m from {previous.name}; '
                    f'points must be > {MIN_POINT_SEPARATION_M:.2f} m apart')
        point = Point(POINT_NAMES[len(self._points)], x, y)
        self._points.append(point)
        if len(self._points) == len(POINT_NAMES) and not self._takeoff_ready:
            self.state = MissionState.WAIT_TAKEOFF
        return point

    def on_control_status(self, control_state: str, now_sec: float) -> Optional[SequencerAction]:
        del now_sec
        if control_state and control_state != 'WAITING_FOR_ODOM':
            self._takeoff_ready = True
        return self._start_when_ready()

    def on_position(self, x_enu: float, y_enu: float, now_sec: float) -> Optional[SequencerAction]:
        del now_sec
        point = self.current_point
        if point is None or not point_is_reached(
                x_enu, y_enu, point, self._settings.arrive_radius):
            return None
        if self.state == MissionState.GOTO_B1:
            self.state = MissionState.GOTO_B2
            self.zone = 'B'
            self.b_recognition = True
            return SequencerAction(
                goal=self._points[1], zone='B', b_recognition=True,
                event='arrived_b1 recognition_b_started')
        if self.state == MissionState.GOTO_C:
            self.state = MissionState.GOTO_D
            return SequencerAction(goal=self._points[3], event='arrived_c')
        return None

    def on_control_hold(self, now_sec: float) -> Optional[SequencerAction]:
        if self.state == MissionState.GOTO_B2:
            self.state = MissionState.HOLD_B2
            self._hold_started_at = now_sec
            return SequencerAction(event='arrived_b2 hold_started')
        if self.state == MissionState.GOTO_D:
            self.state = MissionState.HOLD_D
            self._hold_started_at = now_sec
            self.zone = 'D'
            self.d_recognition = True
            return SequencerAction(
                zone='D', d_recognition=True,
                event='arrived_d recognition_d_started hold_started')
        return None

    def on_timer(self, now_sec: float) -> Optional[SequencerAction]:
        if self._hold_started_at is None:
            return None
        if self.state == MissionState.HOLD_B2:
            if now_sec - self._hold_started_at < self._settings.b2_hold_sec:
                return None
            self._hold_started_at = None
            self.state = MissionState.GOTO_C
            self.zone = 'C'
            self.b_recognition = False
            return SequencerAction(
                goal=self._points[2], zone='C', b_recognition=False,
                event='b2_hold_complete recognition_b_stopped')
        if self.state == MissionState.HOLD_D:
            if now_sec - self._hold_started_at < self._settings.d_hold_sec:
                return None
            self._hold_started_at = None
            self.state = MissionState.MISSION_DONE
            # Per the requested behavior, D recognition remains on after mission completion.
            return SequencerAction(
                event='d_hold_complete recognition_d_kept_running', mission_done=True)
        return None

    def _start_when_ready(self) -> Optional[SequencerAction]:
        if self.collected_count != len(POINT_NAMES) or not self._takeoff_ready:
            return None
        if self.state not in (MissionState.COLLECTING_POINTS, MissionState.WAIT_TAKEOFF):
            return None
        self.state = MissionState.GOTO_B1
        return SequencerAction(goal=self._points[0], event='mission_started goto_b1')


def _ros_main() -> int:
    import rclpy
    from geometry_msgs.msg import PoseStamped
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    class MissionSequencerNode(Node):
        def __init__(self) -> None:
            super().__init__('mission_sequencer_node')
            self.declare_parameter('mission_file', '')
            mission_file = self.get_parameter('mission_file').get_parameter_value().string_value
            if not mission_file:
                raise MissionConfigError('mission_file parameter must not be empty')
            self._sequencer = MissionSequencer(load_settings(mission_file))
            self._last_goal_xy: Optional[tuple[float, float]] = None
            self._goals_sent = 0
            self._done_logged = False

            transient_reliable = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sensor_qos = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT)
            self._goal_publisher = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self._zone_publisher = self.create_publisher(
                String, '/race/mission/zone', transient_reliable)
            self._status_publisher = self.create_publisher(
                String, '/race/mission/status', transient_reliable)
            self._event_publisher = self.create_publisher(String, '/race/mission/event', 10)
            # Recognition command for the detector (bs/code/bs_recognition_node.py).
            # Latched so a detector started after the aircraft entered the zone
            # still learns it should be running.
            self._recognition_publisher = self.create_publisher(
                String, '/race/mission/recognition', transient_reliable)
            self.create_subscription(
                PoseStamped, '/race/mission/clicked_goal', self._clicked_goal_callback, 10)
            self.create_subscription(
                String, '/race/control/status', self._status_callback, transient_reliable)
            self.create_subscription(
                PoseStamped, '/race/pose',
                self._position_callback, sensor_qos)
            self.create_timer(0.05, self._timer_callback)
            self._sequencer.start()
            self._publish_zone('A', previous='NONE')
            self._publish_status()
            self.get_logger().info(
                '[MISSION_READY] RViz points required in order: B1, B2, C, D; '
                'no goal will publish before all four')

        def _now_sec(self) -> float:
            return self.get_clock().now().nanoseconds * 1.0e-9

        def _clicked_goal_callback(self, message: PoseStamped) -> None:
            try:
                point = self._sequencer.add_rviz_point(
                    message.pose.position.x, message.pose.position.y)
            except MissionConfigError as error:
                self.get_logger().error(f'[MISSION_POINT] rejected: {error}')
                return
            self.get_logger().info(
                f'[MISSION_POINT] {point.name.upper()} accepted x={point.x:.3f} y={point.y:.3f} '
                f'({self._sequencer.collected_count}/4)')
            self._event_publisher.publish(String(
                data=f'event=point_accepted name={point.name} x={point.x:.3f} y={point.y:.3f}'))
            self._publish_status()
            self._handle_action(self._sequencer._start_when_ready())

        def _status_callback(self, message: String) -> None:
            state = control_state_from_message(message.data)
            now_sec = self._now_sec()
            # Every well-formed controller state, including an initial hold state,
            # confirms that odometry is available and the takeoff handover has progressed.
            action = self._sequencer.on_control_status(state, now_sec)
            if action is None and state == 'GOAL_REACHED_HOLD':
                action = self._sequencer.on_control_hold(now_sec)
            self._handle_action(action)

        def _position_callback(self, message: PoseStamped) -> None:
            enu_x = message.pose.position.x
            enu_y = message.pose.position.y
            if not (math.isfinite(enu_x) and math.isfinite(enu_y)):
                return
            # /race/pose is already in the unified, world-yaw-aligned map ENU.
            self._handle_action(self._sequencer.on_position(
                enu_x, enu_y, self._now_sec()))

        def _timer_callback(self) -> None:
            self._handle_action(self._sequencer.on_timer(self._now_sec()))

        def _handle_action(self, action: Optional[SequencerAction]) -> None:
            if action is None:
                return
            if action.zone is not None:
                self._publish_zone(action.zone)
            if action.b_recognition is not None:
                self._log_recognition('B', action.b_recognition)
            if action.d_recognition is not None:
                self._log_recognition('D', action.d_recognition)
            if action.event:
                self._event_publisher.publish(String(data=f'event={action.event}'))
            if action.goal is not None:
                self._publish_goal(action.goal)
            if action.mission_done and not self._done_logged:
                self.get_logger().info(
                    '[MISSION_DONE] D hold complete; no further goals. '
                    'D recognition remains enabled.')
                self._done_logged = True
            self._publish_status()

        def _publish_zone(self, zone: str, previous: Optional[str] = None) -> None:
            old_zone = previous if previous is not None else self._sequencer.zone
            # The action has already updated sequencer.zone; retain the prior
            # value for readable logs.
            if previous is None:
                old_zone = getattr(self, '_published_zone', 'NONE')
            self._zone_publisher.publish(String(data=zone))
            self.get_logger().info(f'[MISSION_ZONE] {old_zone}->{zone}')
            self._published_zone = zone

        def _log_recognition(self, zone: str, enabled: bool) -> None:
            # This used to be a pure log line, so the recognition subsystem was
            # never actually told when to run. Publish a latched command the
            # detector subscribes to; the log stays for flight review.
            state = 'started' if enabled else 'stopped'
            self._recognition_publisher.publish(String(
                data=f'zone={zone} recognition={"on" if enabled else "off"}'))
            self.get_logger().info(f'[MISSION_RECOGNITION] {zone} {state}')

        def _publish_status(self) -> None:
            b_state = 'on' if self._sequencer.b_recognition else 'off'
            d_state = 'on' if self._sequencer.d_recognition else 'off'
            self._status_publisher.publish(String(
                data=f'zone={self._sequencer.zone} b_recognition={b_state} '
                     f'd_recognition={d_state} state={self._sequencer.state.value} '
                     f'points={self._sequencer.collected_count}/4 '
                     f'goals_sent={self._goals_sent}/4'))

        def _publish_goal(self, point: Point) -> None:
            if (self._last_goal_xy is not None
                    and horizontal_distance(*self._last_goal_xy, point.x, point.y)
                    <= MIN_POINT_SEPARATION_M):
                self.get_logger().error(
                    f'[MISSION_GOAL] refusing {point.name}: too close to the last published goal')
                return
            goal = PoseStamped()
            goal.header.stamp = self.get_clock().now().to_msg()
            goal.header.frame_id = 'map'
            goal.pose.position.x = point.x
            goal.pose.position.y = point.y
            # The downstream controller ignores goal Z and uses cruise_altitude_m.
            goal.pose.position.z = 0.0
            goal.pose.orientation.w = 1.0
            self._goal_publisher.publish(goal)
            self._goals_sent += 1
            self._last_goal_xy = (point.x, point.y)
            self.get_logger().info(
                f'[MISSION_GOTO] {point.name.upper()} x={point.x:.3f} y={point.y:.3f}')

    rclpy.init()
    node: Optional[Node] = None
    try:
        node = MissionSequencerNode()
        rclpy.spin(node)
    except MissionConfigError as error:
        print(f'mission_sequencer_node configuration error: {error}', file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    sys.exit(_ros_main())
