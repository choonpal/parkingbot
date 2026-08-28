import math
from pathlib import Path

import pytest

from cooperative_parking_robot.keyboard_follow_core import (
    KeyboardFollowLimits,
    capture_aruco_reference,
    evaluate_follow,
    follow_pair_commands,
    median_relative_pose,
)
from cooperative_parking_robot.rigid_pair_teleop_core import (
    relative_pose_is_stable,
    relative_pose_step_is_plausible,
)


def test_aruco_reference_is_captured_exactly_without_geometry_offset():
    observed = (0.257886, 0.00749, math.radians(-0.58))
    assert capture_aruco_reference(observed) == observed


def test_three_pose_median_rejects_one_aruco_yaw_spike():
    poses = [
        (0.210, 0.001, math.radians(0.5)),
        (0.211, 0.000, math.radians(0.7)),
        (0.202, -0.001, math.radians(-5.8)),
    ]
    filtered = median_relative_pose(poses)
    assert filtered[0] == pytest.approx(0.210)
    assert filtered[1] == pytest.approx(0.0)
    assert math.degrees(filtered[2]) == pytest.approx(0.5)


def test_three_pose_median_preserves_sustained_yaw_change_and_wraparound():
    changed = median_relative_pose([
        (0.21, 0.0, math.radians(0.0)),
        (0.21, 0.0, math.radians(6.0)),
        (0.21, 0.0, math.radians(6.2)),
    ])
    assert math.degrees(changed[2]) == pytest.approx(6.0)

    wrapped = median_relative_pose([
        (0.21, 0.0, math.radians(179.0)),
        (0.21, 0.0, math.radians(-179.0)),
        (0.21, 0.0, math.radians(178.0)),
    ])
    assert abs(math.degrees(wrapped[2])) == pytest.approx(179.0)


def test_impossible_pnp_pose_jump_is_rejected_before_the_median_filter():
    baseline = (0.376, 0.015, math.radians(1.0))
    assert relative_pose_step_is_plausible(
        baseline, (0.379, 0.014, math.radians(2.0)))
    assert not relative_pose_step_is_plausible(
        baseline, (0.376, 0.015, math.radians(7.3)))
    assert not relative_pose_step_is_plausible(
        baseline, (0.401, 0.015, math.radians(1.0)))


def test_pose_step_plausibility_handles_yaw_wraparound():
    assert relative_pose_step_is_plausible(
        (0.30, 0.0, math.radians(179.0)),
        (0.30, 0.0, math.radians(-179.0)))


def test_measured_720p_yaw_noise_is_stable_but_large_jump_is_rejected():
    samples = [
        (0.2135, 0.0217, math.radians(-2.36)),
        (0.2134, 0.0218, math.radians(-1.58)),
        (0.2133, 0.0217, math.radians(-0.28)),
    ]
    assert relative_pose_is_stable(
        samples, yaw_span_rad=math.radians(3.0))
    assert relative_pose_step_is_plausible(
        samples[-1], (0.2134, 0.0217, math.radians(4.5)),
        yaw_step_rad=math.radians(5.0))
    assert not relative_pose_step_is_plausible(
        samples[-1], (0.2134, 0.0217, math.radians(8.0)),
        yaw_step_rad=math.radians(5.0))

    source = (
        Path(__file__).parents[1]
        / 'cooperative_parking_robot/keyboard_follow_node.py'
    ).read_text(encoding='utf-8')
    assert "'relative_stable_yaw_span_deg', 3.0" in source
    assert "'relative_stable_window_s', 0.75" in source
    assert 'self.relative_stable_window < self.marker_timeout' in source
    assert "'relative_outlier_yaw_step_deg', 5.0" in source


def test_keyboard_web_uses_fixed_repeat_and_keyup_stop():
    source = (
        Path(__file__).parents[1]
        / 'cooperative_parking_robot/keyboard_follow_node.py'
    ).read_text(encoding='utf-8')
    repeat_source = (
        "setInterval(() => {\n"
        "  if (heldKey !== null) key(heldKey);\n"
        "}, 100)"
    )
    assert repeat_source in source
    assert "document.addEventListener('keyup'" in source
    assert "window.addEventListener('blur', stopHeld)" in source
    assert 'pendingKey = k' in source
    assert 'const next = pendingKey' in source
    assert 'setInterval(tick, 500)' in source
    assert "make_server(host, port, app, threaded=False)" in source
    assert "logging.getLogger('werkzeug').setLevel(logging.ERROR)" in source
    assert 'jsonify(self._status_payload())' not in source
    assert 'self._status_json, mimetype=' in source


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
    assert "'enable_rigid_pair_teleop', default_value='false'" in launch_text
    assert "executable='rigid_pair_teleop'" in launch_text
    assert "LaunchConfiguration('enable_rigid_pair_teleop')" in launch_text
    assert "LaunchConfiguration('enable_drive_test_dashboard')" in launch_text
