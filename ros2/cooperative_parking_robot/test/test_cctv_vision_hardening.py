#!/usr/bin/env python3
"""Regression tests for CCTV source handover and delayed correction replay."""

import math
from pathlib import Path

import pytest

from cooperative_parking_robot.cctv_observation import CctvObservation
from cooperative_parking_robot.cctv_source_policy import SourceSwitchGuard
from cooperative_parking_robot.pose_replay import EkfReplayBuffer
from cooperative_parking_robot.site_geometry import (
    CAMERA_GEOMETRY,
    ROBOT_MARKER_HEIGHT_M,
    VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M,
)


ROOT = Path(__file__).resolve().parents[1]


class FakeEkf:
    def __init__(self):
        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0
        self.P = [[1.0, 0.0, 0.0],
                  [0.0, 1.0, 0.0],
                  [0.0, 0.0, 1.0]]

    def predict(self, dx, dy, dtheta, dt):
        del dt
        self.x += dx
        self.y += dy
        self.yaw += dtheta

    def pose(self):
        return self.x, self.y, self.yaw


def test_observation_round_trip_preserves_source_and_pose():
    original = CctvObservation(
        role='front', camera_id='cam2', stamp_ns=123,
        sequence=5, switch_sequence=2, source_changed=True,
        handover_validated=True,
        pose=(1.0, 2.0, math.radians(179.0)),
        raw_pose=(1.1, 2.1, math.radians(-181.0)),
        source_bias=(-0.1, -0.1, 0.0), selection_cost=0.3)
    restored = CctvObservation.from_json(original.to_json())
    assert restored.role == 'front'
    assert restored.camera_id == 'cam2'
    assert restored.stamp_ns == 123
    assert restored.source_changed
    assert restored.handover_validated
    assert restored.pose[0:2] == pytest.approx((1.0, 2.0))
    assert math.degrees(restored.pose[2]) == pytest.approx(179.0)


def test_source_switch_requires_consistency_without_validated_handover():
    guard = SourceSwitchGuard(confirmations=3)
    assert guard.evaluate('cam0', (0, 0, 0), (0, 0, 0),
                          handover_validated=False).accepted
    first = guard.evaluate('cam2', (0.02, 0, 0), (0, 0, 0),
                           handover_validated=False)
    second = guard.evaluate('cam2', (0.021, 0, 0), (0, 0, 0),
                            handover_validated=False)
    third = guard.evaluate('cam2', (0.019, 0, 0), (0, 0, 0),
                           handover_validated=False)
    assert not first.accepted
    assert not second.accepted
    assert third.accepted and third.source_changed


def test_validated_handover_switches_immediately():
    guard = SourceSwitchGuard(confirmations=3)
    guard.evaluate('cam0', (0, 0, 0), (0, 0, 0),
                   handover_validated=False)
    decision = guard.evaluate('cam2', (0.01, 0, 0), (0, 0, 0),
                              handover_validated=True)
    assert decision.accepted and decision.source_changed


def test_implausible_source_jump_is_rejected():
    guard = SourceSwitchGuard(confirmations=2, max_position_jump_m=0.1)
    guard.evaluate('cam0', (0, 0, 0), (0, 0, 0),
                   handover_validated=False)
    decision = guard.evaluate('cam2', (0.5, 0, 0), (0, 0, 0),
                              handover_validated=False)
    assert not decision.accepted
    assert decision.reason == 'source_jump_limit'


def test_delayed_correction_rewinds_then_replays_later_wheel_steps():
    ekf = FakeEkf()
    history = EkfReplayBuffer(ekf, history_s=5.0)
    history.record_predict(1_000_000_000, 1.0, 0.0, 0.0, 1.0)
    history.record_predict(2_000_000_000, 1.0, 0.0, 0.0, 1.0)
    history.record_predict(3_000_000_000, 1.0, 0.0, 0.0, 1.0)
    assert ekf.x == pytest.approx(3.0)

    def correct():
        ekf.x = 1.5
        return True

    result = history.correct_at(2_100_000_000, correct)
    assert result.status == 'REWIND_REPLAY'
    assert result.replayed_steps == 1
    assert ekf.x == pytest.approx(2.5)
    assert result.quantization_s == pytest.approx(0.1)


def test_measured_geometry_and_unknown_sloped_vehicle_height():
    assert CAMERA_GEOMETRY['cam0'].optical_axis_ground_m == pytest.approx(
        (2.463, 1.982))
    assert CAMERA_GEOMETRY['cam2'].optical_axis_ground_m == pytest.approx(
        (1.831, 0.507))
    assert CAMERA_GEOMETRY['cam0'].optical_center_height_m == pytest.approx(2.61)
    assert CAMERA_GEOMETRY['cam2'].optical_center_height_m == pytest.approx(2.61)
    assert ROBOT_MARKER_HEIGHT_M == pytest.approx(0.12)
    assert VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M is None


def test_production_entrypoints_are_source_aware():
    setup = (ROOT / 'setup.py').read_text()
    expected = (
        'yolo_bev_map_production_node:main',
        'cctv_robot_marker_production_node:main',
        'pose_fusion_production_node:main',
        'rigid_body_sync_vision_node:main',
    )
    for value in expected:
        assert value in setup
