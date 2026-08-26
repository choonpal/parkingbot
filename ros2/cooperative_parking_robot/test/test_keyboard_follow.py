import math
from pathlib import Path

import pytest

from cooperative_parking_robot.keyboard_follow_core import (
    KeyboardFollowLimits,
    capture_aruco_reference,
    evaluate_follow,
    follow_pair_commands,
)


def test_aruco_reference_is_captured_exactly_without_geometry_offset():
    observed = (0.257886, 0.00749, math.radians(-0.58))
    assert capture_aruco_reference(observed) == observed


def _continue_kwargs():
    return {
        'gap_error_m': 0.0,
        'lateral_error_m': 0.0,
        'yaw_error_rad': 0.0,
        'front_distance_m': 0.0,
        'rear_distance_m': 0.0,
        'hardware_ok': True,
        'manual_ok': True,
        'marker_ok': True,
        'odom_ok': True,
        'graph_ok': True,
        'estop': False,
    }


def test_translation_commands_both_robots_in_same_direction():
    front, rear = follow_pair_commands(
        KeyboardFollowLimits(), (0.0628, 0.0, 0.0), 0.70,
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0)
    assert front == pytest.approx((0.0628, 0.0, 0.0))
    assert rear == pytest.approx(front)


def test_rotation_uses_opposite_lateral_rigid_body_velocities():
    front, rear = follow_pair_commands(
        KeyboardFollowLimits(), (0.0, 0.0, 0.10), 0.70,
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0)
    assert front == pytest.approx((0.0, 0.035, 0.10))
    assert rear == pytest.approx((0.0, -0.035, 0.10))


def test_positive_relative_errors_receive_opposite_corrections():
    front, rear = follow_pair_commands(
        KeyboardFollowLimits(), (0.05, 0.0, 0.0), 0.70,
        gap_error_m=0.01,
        lateral_error_m=0.02,
        yaw_error_rad=math.radians(2.0))
    assert front[0] < rear[0]
    assert front[1] < rear[1]
    assert front[2] < rear[2]


def test_common_scale_preserves_pair_shape_at_limit():
    front, rear = follow_pair_commands(
        KeyboardFollowLimits(), (0.0, 0.0, 0.30), 0.82,
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0)
    assert max(math.hypot(cmd[0], cmd[1]) for cmd in (front, rear)) <= 0.08
    assert max(abs(cmd[2]) for cmd in (front, rear)) <= 0.20
    assert front[1] == pytest.approx(-rear[1])
    assert front[2] == pytest.approx(rear[2])


@pytest.mark.parametrize(
    ('field', 'value', 'outcome'),
    [
        ('hardware_ok', False, 'FAULT'),
        ('manual_ok', False, 'FAULT'),
        ('marker_ok', False, 'FAULT'),
        ('odom_ok', False, 'FAULT'),
        ('graph_ok', False, 'FAULT'),
        ('estop', True, 'ESTOP'),
        ('gap_error_m', 0.031, 'FAULT'),
        ('lateral_error_m', -0.031, 'FAULT'),
        ('yaw_error_rad', math.radians(5.1), 'FAULT'),
        ('front_distance_m', 0.30, 'LIMIT'),
        ('rear_distance_m', 0.31, 'LIMIT'),
    ])
def test_follow_safety_gates(field, value, outcome):
    kwargs = _continue_kwargs()
    kwargs[field] = value
    assert evaluate_follow(
        KeyboardFollowLimits(), **kwargs).outcome == outcome


def test_follow_safety_continues_when_all_gates_are_good():
    decision = evaluate_follow(
        KeyboardFollowLimits(), **_continue_kwargs())
    assert decision.outcome == 'CONTINUE'


def test_rear_launch_exposes_separately_conditioned_controller_modes():
    launch_text = (
        Path(__file__).parents[1]
        / 'launch/cooperative_drive_test_rear.launch.py'
    ).read_text(encoding='utf-8')
    assert "'enable_drive_test_dashboard', default_value='true'" in launch_text
    assert "'enable_keyboard_follow', default_value='false'" in launch_text
    assert "executable='keyboard_follow'" in launch_text
    assert "LaunchConfiguration('enable_keyboard_follow')" in launch_text
    assert "LaunchConfiguration('enable_drive_test_dashboard')" in launch_text
