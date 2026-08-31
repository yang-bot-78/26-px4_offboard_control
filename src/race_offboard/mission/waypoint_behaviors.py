#!/usr/bin/env python3
"""
Declarative behavior archive for saved waypoint missions.

This module intentionally produces commands and phase plans instead of
publishing PX4 setpoints.  Hardware-facing executors can be replaced later
without changing the waypoint route or state machine.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from typing import Any, Mapping


class BehaviorConfigError(ValueError):
    """Raised when a waypoint behavior or one of its parameters is invalid."""


BEHAVIOR_ALIASES = {
    'start_recognition': 'start_recognition',
    'recognition_start': 'start_recognition',
    '开启识别程序': 'start_recognition',
    '开启识别': 'start_recognition',
    # Keep saved missions written with the former camera names usable, but
    # make their effect explicit: they now control the recognition process.
    'open_camera': 'start_recognition',
    'camera_open': 'start_recognition',
    '打开相机': 'start_recognition',
    'stop_recognition': 'stop_recognition',
    'recognition_stop': 'stop_recognition',
    '关闭识别程序': 'stop_recognition',
    '关闭识别': 'stop_recognition',
    'close_camera': 'stop_recognition',
    'camera_close': 'stop_recognition',
    '关闭相机': 'stop_recognition',
    'switch_to_astar': 'switch_to_astar',
    'astar_takeover': 'switch_to_astar',
    '关闭ego改为a*接管路径': 'switch_to_astar',
    '关闭ego改为astar接管路径': 'switch_to_astar',
    'yaw_sweep': 'yaw_sweep',
    'left_right_yaw': 'yaw_sweep',
    'turn_yaw': 'yaw_sweep',
    '左转右转': 'yaw_sweep',
    'set_global_speed': 'set_global_speed',
    'global_speed': 'set_global_speed',
    '全局速度': 'set_global_speed',
}


@dataclass(frozen=True)
class YawPhase:
    """One requested heading and dwell in the archived yaw behavior."""

    cycle: int
    direction: str
    target_yaw_rad: float
    hold_sec: float


@dataclass(frozen=True)
class BehaviorSpec:
    """Validated behavior command independent of ROS or vehicle hardware."""

    name: str
    params: dict[str, Any]
    phases: tuple[YawPhase, ...] = ()


def _finite(value: Any, label: str, minimum: float | None = None) -> float:
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise BehaviorConfigError(f'{label} must be a finite number')
    number = float(value)
    if not math.isfinite(number) or (minimum is not None and number < minimum):
        suffix = f' >= {minimum:g}' if minimum is not None else ''
        raise BehaviorConfigError(f'{label} must be a finite number{suffix}')
    return number


def _integer(value: Any, label: str, minimum: int = 1) -> int:
    if isinstance(value, bool) or not isinstance(value, int) or value < minimum:
        raise BehaviorConfigError(f'{label} must be an integer >= {minimum}')
    return value


def wrap_angle(angle_rad: float) -> float:
    """Wrap an angle to [-pi, pi), preserving a deterministic final heading."""
    return (angle_rad + math.pi) % (2.0 * math.pi) - math.pi


def canonical_behavior(value: Any) -> str:
    if value is None:
        return ''
    name = str(value).strip()
    if not name:
        return ''
    canonical = BEHAVIOR_ALIASES.get(name.lower(), BEHAVIOR_ALIASES.get(name))
    if canonical is None:
        raise BehaviorConfigError(
            f'unknown behavior {name!r}; supported: '
            f'{", ".join(sorted(set(BEHAVIOR_ALIASES.values())))}')
    return canonical


def build_behavior(
        value: Any, params: Mapping[str, Any] | None = None,
        arrival_yaw_rad: float = 0.0) -> BehaviorSpec | None:
    """Validate a route behavior and build its replaceable execution plan."""
    name = canonical_behavior(value)
    if not name:
        return None
    parameters = dict(params or {})
    if name in ('start_recognition', 'stop_recognition'):
        if parameters:
            raise BehaviorConfigError(f'{name} does not accept parameters')
        return BehaviorSpec(name, {})
    if name == 'switch_to_astar':
        if parameters:
            raise BehaviorConfigError('switch_to_astar does not accept parameters')
        # In this workspace the A* implementation is the Super planner;
        # ``super`` is the launch/backend value that disables EGO handoff.
        return BehaviorSpec(name, {
            'planner_backend': 'super',
            'ego_enabled': False,
            'requires_navigation_restart': True,
        })
    if name == 'set_global_speed':
        speed = _finite(
            parameters.get('speed_mps', parameters.get('y_mps', parameters.get('y'))),
            'speed_mps', 0.0)
        if speed <= 0.0:
            raise BehaviorConfigError('speed_mps must be > 0')
        return BehaviorSpec(name, {'speed_mps': speed})
    if name == 'yaw_sweep':
        degrees = _finite(
            parameters.get('n_deg', parameters.get('n')), 'n_deg', 0.0)
        hold_sec = _finite(
            parameters.get('m_sec', parameters.get('m')), 'm_sec', 0.0)
        cycles = _integer(parameters.get('cycles', parameters.get('x')), 'cycles')
        return_to_arrival = parameters.get('return_to_arrival', False)
        if not isinstance(return_to_arrival, bool):
            raise BehaviorConfigError('return_to_arrival must be boolean')
        return_hold_sec = _finite(
            parameters.get('return_m_sec', 0.0), 'return_m_sec', 0.0)
        direction = str(parameters.get('direction', 'left')).strip().lower()
        direction = {'左': 'left', '向左': 'left', '右': 'right', '向右': 'right'}.get(
            direction, direction)
        if direction not in ('left', 'right'):
            raise BehaviorConfigError("direction must be 'left' or 'right'")
        if degrees <= 0.0 or degrees > 360.0:
            raise BehaviorConfigError('n_deg must be > 0 and <= 360')
        arrival_yaw_rad = _finite(arrival_yaw_rad, 'arrival_yaw_rad')
        delta = math.radians(degrees)
        sign = 1.0 if direction == 'left' else -1.0
        phases: list[YawPhase] = []
        for cycle in range(1, cycles + 1):
            phases.append(YawPhase(
                cycle, direction,
                wrap_angle(arrival_yaw_rad + sign * delta * cycle), hold_sec))
        if return_to_arrival:
            phases.append(YawPhase(
                cycles + 1,
                'right' if direction == 'left' else 'left',
                wrap_angle(arrival_yaw_rad),
                return_hold_sec))
        return BehaviorSpec(
            name,
            {'n_deg': degrees, 'm_sec': hold_sec, 'cycles': cycles,
             'direction': direction,
             'return_to_arrival': return_to_arrival,
             'return_m_sec': return_hold_sec,
             'arrival_yaw_rad': wrap_angle(arrival_yaw_rad)},
            tuple(phases))
    raise BehaviorConfigError(f'unimplemented behavior {name!r}')


def command_dict(
        spec: BehaviorSpec, waypoint_name: str, arrival_yaw_rad: float) -> dict[str, Any]:
    """Build a JSON/YAML-friendly command envelope for a behavior executor."""
    return {
        'waypoint': waypoint_name,
        'behavior': spec.name,
        'parameters': spec.params,
        'arrival_yaw_rad': wrap_angle(arrival_yaw_rad),
        'phases': [
            {
                'cycle': phase.cycle,
                'direction': phase.direction,
                'target_yaw_rad': phase.target_yaw_rad,
                'hold_sec': phase.hold_sec,
            }
            for phase in spec.phases
        ],
    }
