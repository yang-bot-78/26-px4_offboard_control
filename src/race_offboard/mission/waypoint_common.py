#!/usr/bin/env python3
"""Shared storage, map identity, and geometry helpers for waypoint tools."""

from __future__ import annotations

import hashlib
import math
import os
import tempfile
from pathlib import Path
from typing import Any, Optional

import yaml


def default_waypoints_file() -> Path:
    configured = os.environ.get("WAYPOINTS_FILE")
    if configured:
        return Path(configured).expanduser().resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        return (Path(get_package_share_directory("race_offboard")) / "config" / "waypoints.yaml").resolve()
    except Exception:
        return Path(__file__).resolve().parent.parent / "config" / "waypoints.yaml"


def load_waypoints(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {"frame_id": "map", "waypoints": {}}
    with path.open("r", encoding="utf-8") as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f"waypoint file is not a YAML mapping: {path}")
    data.setdefault("frame_id", "map")
    data.setdefault("waypoints", {})
    if not isinstance(data["waypoints"], dict):
        raise ValueError("waypoints must be a YAML mapping")
    return data


def atomic_write_yaml(path: Path, data: dict[str, Any]) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    fd, temporary = tempfile.mkstemp(prefix=f".{path.name}.", suffix=".tmp", dir=str(path.parent))
    try:
        with os.fdopen(fd, "w", encoding="utf-8") as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=False, default_flow_style=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            os.unlink(temporary)
        except FileNotFoundError:
            pass


def sha256_file(path: Path) -> Optional[str]:
    if not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def resolve_map_file(explicit: Optional[str] = None) -> Optional[Path]:
    value = explicit or os.environ.get("MAP_FILE")
    if value:
        path = Path(value).expanduser().resolve()
        return path if path.is_file() else None
    return None


def quaternion_to_yaw(q: Any) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap_yaw(value: float) -> float:
    return math.atan2(math.sin(value), math.cos(value))
