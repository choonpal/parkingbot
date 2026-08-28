import math
from pathlib import Path

import pytest

from cooperative_parking_robot.cooperative_drive_test_core import (
    DriveTestLimits,
    evaluate_running,
    odom_progress,
    pair_commands,
)


ROOT = Path(__file__).resolve().parents[1]


def _healthy(**overrides):
    values = {
        'elapsed_s': 1.0,
        'front_progress_m': 0.04,
        'rear_progress_m': 0.039,
        'gap_error_m': 0.001,
        'lateral_drift_m': 0.002,
        'yaw_error_rad': math.radians(0.5),
        'hardware_ok': True,
        'manual_ok': True,
        'marker_ok': True,
        'odom_ok': True,
        'estop': False,
    }
    values.update(overrides)
    return evaluate_running(DriveTestLimits(), **values)


def test_limits_keep_first_test_short_and_bounded():
    DriveTestLimits().validate()
    with pytest.raises(ValueError):
        DriveTestLimits(distance_m=0.21, max_distance_m=0.21).validate()
    with pytest.raises(ValueError):
        DriveTestLimits(speed_mps=0.09).validate()
    with pytest.raises(ValueError):
        DriveTestLimits(max_duration_s=11.0).validate()


def test_gap_and_yaw_corrections_are_shared_between_robots():
    limits = DriveTestLimits()
    front, rear = pair_commands(
        limits, gap_error_m=0.02, yaw_error_rad=math.radians(3.0))
    # Front is too far away: slow Front, speed Rear, and use opposite yaw.
    assert front[0] < limits.speed_mps < rear[0]
    assert front[2] < 0.0 < rear[2]
    assert max(abs(front[0]), abs(rear[0])) <= limits.max_command_speed_mps


def test_odom_progress_is_signed_in_each_robot_start_heading():
    assert odom_progress((1.0, 2.0, 0.0), (1.08, 2.0, 0.0)) == \
        pytest.approx(0.08)
    assert odom_progress((1.0, 2.0, math.pi / 2),
                         (1.0, 2.08, math.pi / 2)) == pytest.approx(0.08)
    assert odom_progress((1.0, 2.0, 0.0), (0.98, 2.0, 0.0)) < 0.0


def test_running_decision_continues_then_stops_at_either_robot_target():
    assert _healthy().outcome == 'CONTINUE'
    done = _healthy(front_progress_m=0.101, rear_progress_m=0.095)
    assert done.outcome == 'COMPLETED'


@pytest.mark.parametrize(('overrides', 'reason'), [
    ({'hardware_ok': False}, '하드웨어'),
    ({'manual_ok': False}, '제어권'),
    ({'marker_ok': False}, 'ArUco'),
    ({'odom_ok': False}, 'odometry'),
    ({'gap_error_m': 0.031}, '간격'),
    ({'lateral_drift_m': -0.031}, '좌우'),
    ({'yaw_error_rad': math.radians(5.1)}, '각도'),
    ({'front_progress_m': 0.08, 'rear_progress_m': 0.04}, '이동거리'),
    ({'front_progress_m': -0.011}, '반대 방향'),
    ({'elapsed_s': 4.01}, '시험시간'),
])
def test_running_decision_fails_closed(overrides, reason):
    decision = _healthy(**overrides)
    assert decision.outcome == 'FAULT'
    assert reason in decision.reason


def test_real_test_launches_do_not_start_production_motion_stack():
    front = (ROOT / 'launch/cooperative_drive_test_front.launch.py').read_text()
    rear = (ROOT / 'launch/cooperative_drive_test_rear.launch.py').read_text()
    source = front + rear
    assert "executable='stm32_bridge'" in front
    assert "executable='cooperative_drive_test'" in rear
    assert "executable='camera_preview'" in rear
    assert "Path.home() / 'ov2710_calib_23mm_white.npz'" in rear
    assert "'width', default_value='1280'" in rear
    assert "'height', default_value='720'" in rear
    assert "DeclareLaunchArgument('fps', default_value='8.0')" in rear
    assert "'preview_width', default_value='640'" in rear
    assert "'preview_height', default_value='360'" in rear
    assert "'preview_fps', default_value='4.0'" in rear
    assert "'preview_topic': LaunchConfiguration('preview_image_topic')" in rear
    assert "'image_topics_csv': LaunchConfiguration('preview_image_topic')" in rear
    assert "'marker_size_m', default_value='0.10'" in rear
    assert "'aruco_every_n': 2" in rear
    assert "'preview_enable_aruco', default_value='false'" in rear
    assert "'enable_aruco': _bool('preview_enable_aruco')" in rear
    assert "'aruco_min_marker_distance_rate', default_value='0.02'" in rear
    assert "'min_marker_distance_rate': _float(" in rear
    for forbidden in (
            "executable='rigid_body_sync'",
            "executable='state_machine'",
            "executable='individual_move'",
            "executable='fleet_manager'"):
        assert forbidden not in source


def test_bridge_reports_manual_override_ack_for_both_test_peers():
    bridge = (
        ROOT / 'cooperative_parking_robot/stm32_bridge_node.py').read_text()
    node = (
        ROOT / 'cooperative_parking_robot/cooperative_drive_test_node.py'
    ).read_text()
    assert "f'/{self.role}/manual_active'" in bridge
    assert "f'/{role}/manual_active'" in node
    assert "methods=['POST']" in node
    assert "queue.SimpleQueue()" in node
    assert 'SignalHandlerOptions.NO' in node
    assert node.index('node.destroy_node()') < node.rindex('rclpy.shutdown()')
