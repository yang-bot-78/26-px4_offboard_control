#!/usr/bin/env python3
"""Shared storage helpers for saved map-frame waypoints."""

from __future__ import annotations

import hashlib
import os
import tempfile
from pathlib import Path
from typing import Any, Optional

import yaml


def default_waypoints_file() -> Path:
    configured = os.environ.get('WAYPOINTS_FILE')
    if configured:
        return Path(configured).expanduser().resolve()
    try:
        from ament_index_python.packages import get_package_share_directory
        return (Path(get_package_share_directory('race_offboard')) / 'config' /
                'waypoints' / 'main' / 'waypoints.yaml')
    except Exception:
        return (Path(__file__).resolve().parent.parent / 'config' /
                'waypoints' / 'main' / 'waypoints.yaml')


def load_waypoints(path: Path) -> dict[str, Any]:
    if not path.exists():
        return {'frame_id': 'map', 'waypoints': {}}
    with path.open(encoding='utf-8') as stream:
        data = yaml.safe_load(stream) or {}
    if not isinstance(data, dict):
        raise ValueError(f'waypoint file is not a YAML mapping: {path}')
    data.setdefault('frame_id', 'map')
    data.setdefault('waypoints', {})
    if not isinstance(data['waypoints'], dict):
        raise ValueError('waypoints must be a YAML mapping')
    return data


def atomic_write_yaml(path: Path, data: dict[str, Any]) -> None:
    """Commit a complete YAML document or leave the prior file untouched."""
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary_name = tempfile.mkstemp(
        prefix=f'.{path.name}.', suffix='.tmp', dir=str(path.parent))
    temporary = Path(temporary_name)
    try:
        with os.fdopen(descriptor, 'w', encoding='utf-8') as stream:
            yaml.safe_dump(data, stream, sort_keys=False, allow_unicode=False)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def sha256_file(path: Optional[Path]) -> Optional[str]:
    if path is None or not path.is_file():
        return None
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()
