import math
from pathlib import Path

import pytest

from cooperative_parking_robot.ultrasonic_edge_node import (
    paired_lateral_offset,
)
from cooperative_parking_robot.vision_utils import principal_axis_yaw


ROOT = Path(__file__).resolve().parents[1]


def rotated_rectangle_points(yaw):
    points = []
    for x in (-2.0, -1.0, 0.0, 1.0, 2.0):
        points.extend(((x, -0.5), (x, 0.5)))
    c = math.cos(yaw)
    s = math.sin(yaw)
    return [
        (c * x - s * y, s * x + c * y)
        for x, y in points
    ]


def test_paired_ultrasonic_lateral_offset_and_validity_gate():
    assert paired_lateral_offset(0.06, 0.06, 0.10) == pytest.approx(0.0)
    assert paired_lateral_offset(0.04, 0.08, 0.10) == pytest.approx(0.02)
    assert paired_lateral_offset(
        0.04, 0.08, 0.10, -1.0) == pytest.approx(-0.02)
    assert paired_lateral_offset(0.04, 0.12, 0.10) is None
    assert paired_lateral_offset(float('inf'), 0.08, 0.10) is None


def test_segmentation_pca_recovers_vehicle_axis_and_applies_limit():
    expected = math.radians(12.0)
    points = rotated_rectangle_points(expected)
    assert principal_axis_yaw(
        points, 1.25, math.radians(20.0)) == pytest.approx(
            expected, abs=1e-6)
    assert principal_axis_yaw(
        points, 1.25, math.radians(10.0)) is None


def test_segmentation_pca_rejects_isotropic_contour():
    points = [
        (math.cos(index * math.pi / 4.0),
         math.sin(index * math.pi / 4.0))
        for index in range(8)
    ]
    assert principal_axis_yaw(points, 1.25, math.pi / 2.0) is None


def test_complete_merge_preserves_simultaneous_and_synchronized_modes():
    source = (ROOT / 'cooperative_parking_robot' /
              'individual_move_node.py').read_text(encoding='utf-8')
    for token in (
            'simultaneous_entry', 'WAIT_PEER_STAGED', 'PRE_ALIGN',
            'PREALIGNED', 'peer retreat', 'RETREAT', 'wheel_scan_reset',
            'same_direction_exit',
            'WAIT_PEER_RETURN', 'synchronized_exit_speed'):
        assert token in source


def test_complete_merge_preserves_aruco_and_ekf_runtime_fixes():
    aruco = (ROOT / 'cooperative_parking_robot' /
             'aruco_utils.py').read_text(encoding='utf-8')
    fusion = (ROOT / 'cooperative_parking_robot' /
              'pose_fusion_node.py').read_text(encoding='utf-8')
    tracker = (ROOT / 'cooperative_parking_robot' /
               'aruco_tracker_node.py').read_text(encoding='utf-8')
    assert 'DetectorParameters_create' in aruco
    assert 'ReliabilityPolicy.RELIABLE' in fusion
    assert 'yaw_sign' in tracker
    assert 'gray_gain' in tracker


def test_all_real_launches_expose_v9_controls():
    for relative in (
            'launch/front_robot.launch.py',
            'launch/rear_robot.launch.py',
            'launch/full_system.launch.py'):
        source = (ROOT / relative).read_text(encoding='utf-8')
        for token in (
                'prealign_hold_n', 'use_ultrasonic_lateral',
                'lateral_deviation_limit_m', 'max_scan_retry',
                'lateral_pair_timeout_s', 'lateral_sign'):
            assert token in source


def test_cctv_launches_expose_vehicle_yaw_controls():
    for relative in (
            'launch/cctv_server.launch.py',
            'launch/full_system.launch.py'):
        source = (ROOT / relative).read_text(encoding='utf-8')
        for token in (
                'yaw_pca_min_ratio', 'yaw_ema_alpha', 'yaw_limit_deg'):
            assert token in source
