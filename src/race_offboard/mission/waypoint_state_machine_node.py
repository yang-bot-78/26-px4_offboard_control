#!/usr/bin/env python3
"""
Follow every saved map waypoint and publish a state transition at arrival.

The node deliberately publishes planner goals (``/goal_pose``), never PX4
setpoints.  The existing navigation stack remains the only authority that
drives the vehicle.  A route entry may contain ``state``, ``behavior`` and
``hold_sec`` fields so future aircraft actions can subscribe to the published
mission events without changing this node.
"""

from __future__ import annotations

import argparse
import json
import math
import sys
from dataclasses import dataclass, field
from enum import Enum
from pathlib import Path
from typing import Any, Optional

import yaml

# Keep direct source-tree execution and importlib-based unit tests equivalent
# to the installed ROS executable.
_MISSION_DIR = str(Path(__file__).resolve().parent)
if _MISSION_DIR not in sys.path:
    sys.path.insert(0, _MISSION_DIR)
from waypoint_behaviors import (  # noqa: E402
    BehaviorConfigError, build_behavior, command_dict)


MIN_POINT_SEPARATION_M = 0.05


class MissionConfigError(ValueError):
    """Raised when the saved route is unsafe or malformed."""


@dataclass(frozen=True)
class RoutePoint:
    name: str
    x: float
    y: float
    z: float
    state: str
    behavior: str
    hold_sec: float
    behavior_params: dict[str, Any] = field(default_factory=dict)


class State(str, Enum):
    WAIT_OFFBOARD = 'WAIT_OFFBOARD'
    GOTO = 'GOTO'
    HOLD = 'HOLD'
    PAUSED = 'PAUSED'
    DONE = 'DONE'


@dataclass(frozen=True)
class Action:
    goal: Optional[RoutePoint] = None
    event: str = ''
    arrived: Optional[RoutePoint] = None
    done: bool = False


def _number(value: Any, label: str, minimum: Optional[float] = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise MissionConfigError(f'{label} must be a finite number')
    result = float(value)
    if not math.isfinite(result) or (minimum is not None and result < minimum):
        suffix = f' >= {minimum:g}' if minimum is not None else ''
        raise MissionConfigError(f'{label} must be a finite number{suffix}')
    return result


def load_route(path: str | Path, default_hold_sec: float = 0.0) -> tuple[RoutePoint, ...]:
    """Load route_points, accepting legacy four-point preset_points files."""
    route_path = Path(path).expanduser().resolve()
    try:
        with route_path.open(encoding='utf-8') as stream:
            document = yaml.safe_load(stream) or {}
    except (OSError, yaml.YAMLError) as error:
        raise MissionConfigError(f'failed to load mission file {route_path}: {error}') from error
    if not isinstance(document, dict):
        raise MissionConfigError('mission YAML root must be a mapping')
    raw = document.get('route_points')
    if raw is None:
        raw = document.get('preset_points')
        if isinstance(raw, list):
            source_names = document.get('source_waypoint_names')
            if not isinstance(source_names, list) or len(source_names) != len(raw):
                source_names = [f'P{index + 1:03d}' for index in range(len(raw))]
            converted = []
            for index, point in enumerate(raw):
                if not isinstance(point, (list, tuple)) or len(point) < 2:
                    raise MissionConfigError(f'preset_points[{index}] must be [x, y]')
                converted.append(
                    {'name': str(source_names[index]), 'x': point[0], 'y': point[1]})
            raw = converted
    if not isinstance(raw, list) or not raw:
        raise MissionConfigError('mission file must contain a non-empty route_points list')
    default_hold_sec = _number(default_hold_sec, 'default_hold_sec', 0.0)
    points: list[RoutePoint] = []
    for index, entry in enumerate(raw):
        if not isinstance(entry, dict):
            raise MissionConfigError(f'route_points[{index}] must be a mapping')
        name = str(entry.get('name', f'P{index + 1:03d}')).strip()
        if not name or any(char in name for char in '/\\'):
            raise MissionConfigError(f'route_points[{index}].name is invalid')
        x = _number(entry.get('x'), f'{name}.x')
        y = _number(entry.get('y'), f'{name}.y')
        z = _number(entry.get('z', document.get('default_flight_height', 0.75)), f'{name}.z')
        configured_hold = entry.get('hold_sec', default_hold_sec)
        # Preserve the established B2/D dwell behavior when a legacy export
        # did not carry per-waypoint behavior fields.
        if 'hold_sec' not in entry:
            if name.upper() == 'B2':
                configured_hold = document.get('b2_hold_sec', configured_hold)
            elif name.upper() == 'D':
                configured_hold = document.get('d_hold_sec', configured_hold)
        hold_sec = _number(configured_hold, f'{name}.hold_sec', 0.0)
        state = str(entry.get('state', f'GOTO_{name.upper()}')).strip() or f'GOTO_{name.upper()}'
        behavior = str(entry.get('behavior', '')).strip()
        behavior_params = entry.get('behavior_params', {})
        if not isinstance(behavior_params, dict):
            raise MissionConfigError(f'{name}.behavior_params must be a mapping')
        if any(point.name == name for point in points):
            raise MissionConfigError(f'duplicate route waypoint name: {name}')
        if points and math.hypot(x - points[-1].x, y - points[-1].y) <= MIN_POINT_SEPARATION_M:
            raise MissionConfigError(
                f'{name} is too close to {points[-1].name}; points must be > '
                f'{MIN_POINT_SEPARATION_M:.2f}m apart')
        points.append(RoutePoint(
            name, x, y, z, state, behavior, hold_sec, dict(behavior_params)))
    return tuple(points)


class WaypointStateMachine:
    """Pure route state machine, easy to exercise without ROS or PX4."""

    def __init__(
            self, points: tuple[RoutePoint, ...], arrive_radius: float,
            wait_for_takeoff: bool = False, takeoff_height: float = 0.55):
        if not points:
            raise MissionConfigError('at least one route waypoint is required')
        self.points = points
        self.arrive_radius = _number(arrive_radius, 'arrive_radius')
        if self.arrive_radius <= 0:
            raise MissionConfigError('arrive_radius must be > 0')
        self.state = State.WAIT_OFFBOARD
        self.index = -1
        self.offboard = False
        self.wait_for_takeoff = wait_for_takeoff
        self.takeoff_height = _number(takeoff_height, 'takeoff_height', 0.0)
        self.hold_started_at: Optional[float] = None

    @property
    def current(self) -> Optional[RoutePoint]:
        return self.points[self.index] if 0 <= self.index < len(self.points) else None

    def on_offboard(self, active: bool) -> Optional[Action]:
        if active == self.offboard:
            return None
        self.offboard = active
        if not active:
            if self.state in (State.GOTO, State.HOLD):
                self.state = State.PAUSED
                self.hold_started_at = None
                return Action(event='mission_paused_pilot_left_offboard')
            return None
        if self.state == State.WAIT_OFFBOARD:
            if self.wait_for_takeoff:
                return Action(event='offboard_waiting_for_takeoff')
            self.index = 0
            self.state = State.GOTO
            return Action(goal=self.current, event=f'mission_started goto={self.current.name}')
        if self.state == State.PAUSED:
            return Action(event='mission_paused_restart_required')
        return None

    def on_control_status(self, message: str) -> Optional[Action]:
        """Start a delayed route after the safety controller reports takeoff complete."""
        if not self.wait_for_takeoff or not self.offboard or self.state != State.WAIT_OFFBOARD:
            return None
        if 'offboard=1' not in message or 'armed=1' not in message:
            return None
        height = None
        for token in message.split():
            if token.startswith('current_height_m='):
                try:
                    height = float(token.split('=', 1)[1])
                except ValueError:
                    return None
                break
        if height is None or not math.isfinite(height) or height < self.takeoff_height:
            return None
        self.wait_for_takeoff = False
        self.index = 0
        self.state = State.GOTO
        return Action(
            goal=self.current,
            event=f'takeoff_complete mission_started goto={self.current.name}')

    def on_position(self, x: float, y: float, now_sec: float) -> Optional[Action]:
        point = self.current
        if not self.offboard or self.state != State.GOTO or point is None:
            return None
        if math.hypot(x - point.x, y - point.y) > self.arrive_radius:
            return None
        self.hold_started_at = None
        if point.hold_sec > 0:
            self.state = State.HOLD
            self.hold_started_at = now_sec
            return Action(
                arrived=point,
                event=f'arrived={point.name} state={point.state} hold_started')
        return self._advance(point)

    def on_timer(self, now_sec: float) -> Optional[Action]:
        if self.state != State.HOLD or self.hold_started_at is None or self.current is None:
            return None
        if now_sec - self.hold_started_at < self.current.hold_sec:
            return None
        point = self.current
        return self._advance(point)

    def start_hold(self, now_sec: float) -> Optional[Action]:
        """Allow the controller's GOAL_REACHED_HOLD status to start a dwell."""
        if self.state == State.GOTO and self.current is not None and self.current.hold_sec > 0:
            self.state = State.HOLD
            self.hold_started_at = now_sec
            return Action(arrived=self.current, event=f'arrived={self.current.name} hold_started')
        return None

    def _advance(self, arrived: RoutePoint) -> Action:
        self.index += 1
        if self.index >= len(self.points):
            self.state = State.DONE
            return Action(
                arrived=arrived, event=f'arrived={arrived.name} route_complete', done=True)
        self.state = State.GOTO
        return Action(
            goal=self.current, arrived=arrived,
            event=f'arrived={arrived.name} state={arrived.state} goto={self.current.name}')


def _ros_main(argv: Optional[list[str]] = None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mission-file', required=True)
    parser.add_argument('--arrive-radius', type=float, default=None)
    parser.add_argument('--default-hold-sec', type=float, default=0.0)
    parser.add_argument(
        '--wait-for-takeoff', action='store_true',
        help='delay the first route goal until the safety controller reports takeoff complete')
    parser.add_argument('--pose-topic', default='/race/pose')
    args, ros_args = parser.parse_known_args(argv)

    import rclpy
    from geometry_msgs.msg import PoseStamped
    from mavros_msgs.msg import State as MavrosState
    from rclpy.node import Node
    from rclpy.qos import DurabilityPolicy, HistoryPolicy, QoSProfile, ReliabilityPolicy
    from std_msgs.msg import String

    try:
        points = load_route(args.mission_file, args.default_hold_sec)
        with Path(args.mission_file).open(encoding='utf-8') as stream:
            settings = yaml.safe_load(stream) or {}
        radius = (args.arrive_radius if args.arrive_radius is not None else
                  settings.get('arrive_radius', 0.4))
        machine = WaypointStateMachine(
            points, radius, wait_for_takeoff=args.wait_for_takeoff)
    except (OSError, MissionConfigError, yaml.YAMLError) as error:
        print(f'waypoint_state_machine_node configuration error: {error}', file=sys.stderr)
        return 2

    class NodeImpl(Node):
        def __init__(self) -> None:
            super().__init__('waypoint_state_machine_node')
            self.machine = machine
            transient = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=1,
                reliability=ReliabilityPolicy.RELIABLE,
                durability=DurabilityPolicy.TRANSIENT_LOCAL)
            sensor = QoSProfile(
                history=HistoryPolicy.KEEP_LAST, depth=10,
                reliability=ReliabilityPolicy.BEST_EFFORT)
            self.goal_pub = self.create_publisher(PoseStamped, '/goal_pose', 10)
            self.status_pub = self.create_publisher(String, '/race/waypoint_fsm/status', transient)
            self.event_pub = self.create_publisher(String, '/race/waypoint_fsm/event', 10)
            self.behavior_pub = self.create_publisher(String, '/race/mission/behavior', transient)
            self.behavior_command_pub = self.create_publisher(
                String, '/race/mission/behavior_command', transient)
            self._last_yaw_rad = 0.0
            self.create_subscription(MavrosState, '/mavros/state', self.mode_callback, sensor)
            self.create_subscription(PoseStamped, args.pose_topic, self.pose_callback, sensor)
            self.create_subscription(
                String, '/race/control/status', self.control_callback, transient)
            self.create_timer(0.05, self.timer_callback)
            self.publish_status()
            self.get_logger().info(
                f'[WAYPOINT_FSM_READY] {len(points)} points; waiting for OFFBOARD')

        def now(self) -> float:
            return self.get_clock().now().nanoseconds / 1e9

        def mode_callback(self, message: MavrosState) -> None:
            self.process_action(self.machine.on_offboard(message.mode == 'OFFBOARD'))

        def pose_callback(self, message: PoseStamped) -> None:
            point = message.pose.position
            orientation = message.pose.orientation
            self._last_yaw_rad = math.atan2(
                2.0 * (orientation.w * orientation.z + orientation.x * orientation.y),
                1.0 - 2.0 * (orientation.y * orientation.y + orientation.z * orientation.z))
            if math.isfinite(point.x) and math.isfinite(point.y):
                self.process_action(self.machine.on_position(point.x, point.y, self.now()))

        def control_callback(self, message: String) -> None:
            self.process_action(self.machine.on_control_status(message.data))
            if 'state=GOAL_REACHED_HOLD' in message.data:
                self.process_action(self.machine.start_hold(self.now()))

        def timer_callback(self) -> None:
            self.process_action(self.machine.on_timer(self.now()))

        def process_action(self, action: Optional[Action]) -> None:
            if action is None:
                return
            if action.arrived is not None and action.arrived.behavior:
                try:
                    behavior = build_behavior(
                        action.arrived.behavior, action.arrived.behavior_params,
                        self._last_yaw_rad)
                    if behavior is not None:
                        command = command_dict(
                            behavior, action.arrived.name, self._last_yaw_rad)
                        self.behavior_command_pub.publish(String(
                            data=json.dumps(command, ensure_ascii=False, sort_keys=True)))
                        self.behavior_pub.publish(String(data=(
                            f'point={action.arrived.name} state={action.arrived.state} '
                            f'behavior={behavior.name}')))
                except BehaviorConfigError as error:
                    self.event_pub.publish(String(
                        data=f'behavior_rejected point={action.arrived.name} reason={error}'))
                    self.get_logger().error(
                        f'[WAYPOINT_BEHAVIOR_REJECTED] {action.arrived.name}: {error}')
            if action.event:
                self.event_pub.publish(String(data=action.event))
                self.get_logger().info(f'[WAYPOINT_FSM] {action.event}')
            if action.goal is not None:
                goal = PoseStamped()
                goal.header.stamp = self.get_clock().now().to_msg()
                goal.header.frame_id = 'map'
                goal.pose.position.x = action.goal.x
                goal.pose.position.y = action.goal.y
                goal.pose.position.z = action.goal.z
                goal.pose.orientation.w = 1.0
                self.goal_pub.publish(goal)
                self.get_logger().info(
                    f'[WAYPOINT_FSM_GOTO] {action.goal.name} x={action.goal.x:.3f} '
                    f'y={action.goal.y:.3f} z={action.goal.z:.3f}')
            self.publish_status()

        def publish_status(self) -> None:
            point = self.machine.current
            current = point.name if point else '-'
            point_state = point.state if point else '-'
            self.status_pub.publish(String(data=(
                f'state={self.machine.state.value} index={self.machine.index + 1}/'
                f'{len(self.machine.points)} current={current} point_state={point_state} '
                f'offboard={str(self.machine.offboard).lower()}')))

    rclpy.init(args=ros_args)
    node: Optional[NodeImpl] = None
    try:
        node = NodeImpl()
        rclpy.spin(node)
    except KeyboardInterrupt:
        return 0
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()
    return 0


if __name__ == '__main__':
    raise SystemExit(_ros_main())
