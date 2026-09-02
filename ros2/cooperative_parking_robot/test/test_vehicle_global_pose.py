import math
from pathlib import Path

import pytest

from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics
from cooperative_parking_robot.vehicle_global_pose import VehicleGlobalPoseTracker


ROOT = Path(__file__).resolve().parents[1]


def pose(x, y, yaw):
    return {'x': x, 'y': y, 'theta': yaw}


def test_transport_yaw_does_not_follow_pair_line_lateral_skew():
    k = RigidBodyKinematics(0.785)
    front = pose(1.0, 0.20, 0.0)
    rear = pose(0.0, 0.00, 0.0)
    assert k.virtual_pose(front, rear)[2] == pytest.approx(
        math.atan2(0.20, 1.0))
    assert k.transport_pose(front, rear)[2] == pytest.approx(0.0)


def test_transport_heading_mean_wraps_179_degrees():
    yaw = RigidBodyKinematics.circular_mean_yaw(
        math.radians(179.0), math.radians(-179.0))
    assert abs(abs(math.degrees(yaw)) - 180.0) < 1.0e-6


def test_global_feedback_changes_map_bias_not_mounting_offset():
    tracker = VehicleGlobalPoseTracker(
        position_gate_m=0.50, position_alpha=0.25)
    front = pose(1.0, 0.4, 0.0)
    rear = pose(0.2, 0.4, 0.0)
    offset = [0.10, -0.02]
    original = tuple(offset)
    predicted = tracker.vehicle_pose(front, rear, *offset)
    assert tracker.update(
        measured_x_m=predicted[0] + 0.20,
        measured_y_m=predicted[1] - 0.10,
        front=front, rear=rear,
        offset_body_x=offset[0], offset_body_y=offset[1],
        source_stamp_ns=1, now_s=1.0)
    corrected = tracker.vehicle_pose(front, rear, *offset)
    assert corrected[:2] == pytest.approx(
        (predicted[0] + 0.20, predicted[1] - 0.10))
    assert tuple(offset) == original


def test_duplicate_and_outlier_measurements_do_not_move_pose():
    tracker = VehicleGlobalPoseTracker(position_gate_m=0.10)
    front = pose(1.0, 0.0, 0.0)
    rear = pose(0.0, 0.0, 0.0)
    before = tracker.transport_pose(front, rear)
    assert not tracker.update(
        measured_x_m=9.0, measured_y_m=9.0,
        front=front, rear=rear,
        offset_body_x=0.0, offset_body_y=0.0,
        source_stamp_ns=10, now_s=1.0)
    assert not tracker.update(
        measured_x_m=before[0], measured_y_m=before[1],
        front=front, rear=rear,
        offset_body_x=0.0, offset_body_y=0.0,
        source_stamp_ns=10, now_s=1.1)
    assert tracker.transport_pose(front, rear) == pytest.approx(before)


def test_production_entrypoints_and_authority_are_wired():
    setup = (ROOT / 'setup.py').read_text(encoding='utf-8')
    node = (ROOT / 'cooperative_parking_robot' /
            'rigid_body_sync_vehicle_global_node.py').read_text(
                encoding='utf-8')
    assert 'cctv_merge_global_vehicle_node:main' in setup
    assert 'rigid_body_sync_vehicle_global_node:main' in setup
    assert 'def cctv_feedback_cb' in node
    assert 'def _relative_predictor' in node
    assert 'def _largest_current_lateral_error' in node
    assert 'FUSED_ID0_ONLY' in node
