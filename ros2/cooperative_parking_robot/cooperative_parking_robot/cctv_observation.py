#!/usr/bin/env python3
"""Versioned source-aware CCTV marker observation envelope."""

from __future__ import annotations

from dataclasses import dataclass
import json
import math
from typing import Mapping, Optional, Tuple


OBSERVATION_VERSION = 1


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _finite(name: str, value) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError(f'{name} must be finite')
    return result


def _pose(payload: Mapping, name: str) -> Tuple[float, float, float]:
    try:
        return (
            _finite(f'{name}.x', payload['x']),
            _finite(f'{name}.y', payload['y']),
            normalize_angle(_finite(f'{name}.yaw', payload['yaw'])),
        )
    except (KeyError, TypeError) as exc:
        raise ValueError(f'{name} requires x/y/yaw') from exc


@dataclass(frozen=True)
class CctvObservation:
    role: str
    camera_id: str
    stamp_ns: int
    sequence: int
    switch_sequence: int
    source_changed: bool
    handover_validated: bool
    pose: Tuple[float, float, float]
    raw_pose: Tuple[float, float, float]
    source_bias: Tuple[float, float, float]
    selection_cost: float

    def to_json(self) -> str:
        return json.dumps({
            'version': OBSERVATION_VERSION,
            'frame_id': 'map',
            'role': self.role,
            'camera_id': self.camera_id,
            'stamp_ns': int(self.stamp_ns),
            'sequence': int(self.sequence),
            'switch_sequence': int(self.switch_sequence),
            'source_changed': bool(self.source_changed),
            'handover_validated': bool(self.handover_validated),
            'pose': {
                'x': self.pose[0], 'y': self.pose[1], 'yaw': self.pose[2]},
            'raw_pose': {
                'x': self.raw_pose[0], 'y': self.raw_pose[1],
                'yaw': self.raw_pose[2]},
            'source_bias': {
                'x': self.source_bias[0], 'y': self.source_bias[1],
                'yaw': self.source_bias[2]},
            'selection_cost': float(self.selection_cost),
        }, ensure_ascii=False, separators=(',', ':'))

    @classmethod
    def from_json(cls, text: str) -> 'CctvObservation':
        try:
            payload = json.loads(text)
        except (TypeError, ValueError, json.JSONDecodeError) as exc:
            raise ValueError(f'invalid CCTV observation JSON: {exc}') from exc
        if not isinstance(payload, dict):
            raise ValueError('CCTV observation must be a JSON object')
        if int(payload.get('version', 0)) != OBSERVATION_VERSION:
            raise ValueError('unsupported CCTV observation version')
        if str(payload.get('frame_id', '')) != 'map':
            raise ValueError('CCTV observation frame_id must be map')
        role = str(payload.get('role', '')).strip()
        if role not in ('front', 'rear'):
            raise ValueError('CCTV observation role must be front or rear')
        camera_id = str(payload.get('camera_id', '')).strip()
        if not camera_id:
            raise ValueError('CCTV observation camera_id is required')
        stamp_ns = int(payload.get('stamp_ns', 0))
        if stamp_ns <= 0:
            raise ValueError('CCTV observation stamp_ns must be positive')
        pose = _pose(payload.get('pose', {}), 'pose')
        raw_pose = _pose(payload.get('raw_pose', payload.get('pose', {})),
                         'raw_pose')
        source_bias = _pose(
            payload.get('source_bias', {'x': 0.0, 'y': 0.0, 'yaw': 0.0}),
            'source_bias')
        return cls(
            role=role,
            camera_id=camera_id,
            stamp_ns=stamp_ns,
            sequence=int(payload.get('sequence', 0)),
            switch_sequence=int(payload.get('switch_sequence', 0)),
            source_changed=bool(payload.get('source_changed', False)),
            handover_validated=bool(
                payload.get('handover_validated', False)),
            pose=pose,
            raw_pose=raw_pose,
            source_bias=source_bias,
            selection_cost=_finite(
                'selection_cost', payload.get('selection_cost', 0.0)),
        )
