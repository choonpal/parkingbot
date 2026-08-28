"""ROS-independent regression tests for virtual rigid-pair teleoperation."""

import math
from pathlib import Path
from types import MethodType, SimpleNamespace

import pytest
from builtin_interfaces.msg import Time

from cooperative_parking_robot.keyboard_follow_node import RigidPairTeleopNode
from cooperative_parking_robot.rigid_pair_teleop_core import (
    RigidPairTeleopLimits,
    capture_pair_reference,
    evaluate_rigid_pair,
    split_pair_centre_twist,
    OdomPathAccumulator,
    relative_pose_is_stable,
    request_origin_is_same_host,
    evaluate_placement_guide,
)
from cooperative_parking_robot.vehicle_entry import DEFAULT_WHEELBASE_M


def _safe_inputs():
    return dict(
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0,
        front_distance_m=0.0, rear_distance_m=0.0, hardware_ok=True,
        manual_ok=True, marker_ok=True, odom_ok=True, graph_ok=True,
        estop=False)


def test_canonical_core_captures_measured_spacing_without_nominal_geometry():
    observed = (0.423, -0.014, math.radians(3.0))
    assert capture_pair_reference(observed) == observed


@pytest.mark.parametrize('intent', [
    (0.0628, 0.0, 0.0), (0.0, -0.0628, 0.0), (0.0, 0.0, 0.12),
])
def test_split_reconstructs_the_requested_virtual_pair_centre_twist(intent):
    front, rear = split_pair_centre_twist(
        RigidPairTeleopLimits(), intent, 0.70,
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0)
    assert ((front[0] + rear[0]) / 2.0,
            (front[1] + rear[1]) / 2.0,
            (front[2] + rear[2]) / 2.0) == pytest.approx(intent)


def test_rotation_uses_shared_pair_centre_geometry_not_raw_id0_range():
    front, rear = split_pair_centre_twist(
        RigidPairTeleopLimits(), (0.0, 0.0, 0.10), DEFAULT_WHEELBASE_M,
        gap_error_m=0.0, lateral_error_m=0.0, yaw_error_rad=0.0)
    assert front == pytest.approx(
        (0.0, 0.10 * DEFAULT_WHEELBASE_M / 2.0, 0.10))
    assert rear == pytest.approx(
        (0.0, -0.10 * DEFAULT_WHEELBASE_M / 2.0, 0.10))


def test_relative_pose_feedback_is_symmetric_between_both_robots():
    front, rear = split_pair_centre_twist(
        RigidPairTeleopLimits(), (0.04, 0.0, 0.0), 0.70,
        gap_error_m=0.02, lateral_error_m=-0.02,
        yaw_error_rad=math.radians(2.0))
    assert (front[0] + rear[0]) / 2.0 == pytest.approx(0.04)
    assert (front[1] + rear[1]) / 2.0 == pytest.approx(0.0)
    assert (front[2] + rear[2]) / 2.0 == pytest.approx(0.0)
    assert front[0] < rear[0]
    assert front[1] > rear[1]
    assert front[2] < rear[2]


def test_session_distance_accumulates_path_instead_of_start_end_displacement():
    path = OdomPathAccumulator(max_step_m=0.10)
    for pose in ((0.0, 0.0), (0.04, 0.0), (0.04, 0.04), (0.0, 0.04), (0.0, 0.0)):
        assert path.add(pose)
    assert path.distance_m == pytest.approx(0.16)


def test_odom_path_rejects_a_teleport_sample_without_counting_it():
    path = OdomPathAccumulator(max_step_m=0.10)
    assert path.add((0.0, 0.0))
    assert not path.add((0.50, 0.0))
    assert path.distance_m == 0.0


def test_arm_requires_three_tightly_clustered_id0_samples():
    stable = [(0.215, 0.001, 0.01), (0.218, 0.002, 0.015),
              (0.216, 0.000, 0.012)]
    assert relative_pose_is_stable(stable)
    assert not relative_pose_is_stable(stable[:2])
    assert not relative_pose_is_stable([
        (0.215, 0.0, 0.0), (0.235, 0.0, 0.0), (0.216, 0.0, 0.0)])


def test_read_only_placement_guide_progresses_without_authorizing_motion():
    common = dict(pair_separation_m=0.785, aruco_distance_offset_m=0.570)
    assert evaluate_placement_guide(
        relative_pose=None, marker_fresh=False, stable=False,
        **common).state == '마커 찾기'
    assert evaluate_placement_guide(
        relative_pose=(0.215, 0.0, 0.0), marker_fresh=True, stable=False,
        **common).state == '안정화 중'
    assert evaluate_placement_guide(
        relative_pose=(0.180, 0.0, 0.0), marker_fresh=True, stable=True,
        **common).state == '앞뒤 조정'
    assert evaluate_placement_guide(
        relative_pose=(0.215, 0.020, 0.0), marker_fresh=True, stable=True,
        **common).state == '좌우 조정'
    assert evaluate_placement_guide(
        relative_pose=(0.215, 0.0, math.radians(3.0)),
        marker_fresh=True, stable=True, **common).state == 'yaw 조정'
    candidate = evaluate_placement_guide(
        relative_pose=(0.215, 0.010, math.radians(1.0)),
        marker_fresh=True, stable=True, **common)
    assert candidate.state == '정렬 후보'
    assert candidate.centre_distance_m == pytest.approx(0.785)
    assert candidate.centre_error_m == pytest.approx(0.0)


def test_placement_guide_never_claims_inference_without_calibration_offset():
    guide = evaluate_placement_guide(
        relative_pose=(0.215, 0.0, 0.0), marker_fresh=True, stable=True,
        pair_separation_m=0.785, aruco_distance_offset_m=0.0)
    assert guide.state == '보정값 없음'
    assert not guide.calibration_available
    assert not guide.estimate_available
    assert guide.centre_distance_m is None


def test_placement_calibration_and_estimate_availability_are_independent():
    stale = evaluate_placement_guide(
        relative_pose=None, marker_fresh=False, stable=False,
        pair_separation_m=0.785, aruco_distance_offset_m=0.570)
    assert stale.state == '마커 찾기'
    assert stale.calibration_available
    assert not stale.estimate_available
    unstable = evaluate_placement_guide(
        relative_pose=(0.215, 0.0, 0.0), marker_fresh=True, stable=False,
        pair_separation_m=0.785, aruco_distance_offset_m=0.570)
    assert unstable.state == '안정화 중'
    assert unstable.calibration_available
    assert not unstable.estimate_available


def test_placement_guide_is_reference_independent_and_clears_numbers_when_stale():
    kwargs = dict(relative_pose=(0.215, 0.004, math.radians(1.0)),
                  marker_fresh=True, stable=True, pair_separation_m=0.785,
                  aruco_distance_offset_m=0.570)
    assert evaluate_placement_guide(**kwargs) == evaluate_placement_guide(**kwargs)
    stale = evaluate_placement_guide(
        relative_pose=kwargs['relative_pose'], marker_fresh=False, stable=True,
        pair_separation_m=0.785, aruco_distance_offset_m=0.570)
    assert stale.state == '마커 찾기'
    assert all(value is None for value in (
        stale.raw_forward_m, stale.centre_distance_m,
        stale.raw_lateral_error_m, stale.raw_yaw_error_rad))


def test_pair_publish_emits_both_commands_with_one_shared_timestamp():
    class RecordingPublisher:
        def __init__(self):
            self.messages = []

        def publish(self, message):
            self.messages.append(message)

    stamp = Time(sec=123, nanosec=456)
    clock = SimpleNamespace(
        now=lambda: SimpleNamespace(to_msg=lambda: stamp))
    node = SimpleNamespace(
        get_clock=lambda: clock,
        pub_command={
            'front': RecordingPublisher(),
            'rear': RecordingPublisher(),
        },
        last_commands={
            'front': (0.0, 0.0, 0.0),
            'rear': (0.0, 0.0, 0.0),
        },
    )
    node._publish_command = MethodType(
        RigidPairTeleopNode._publish_command, node)

    front = (0.02, 0.03, 0.04)
    rear = (-0.02, -0.03, -0.04)
    RigidPairTeleopNode._publish_pair_commands(node, front, rear)

    front_msg = node.pub_command['front'].messages[0]
    rear_msg = node.pub_command['rear'].messages[0]
    assert front_msg.header.stamp == rear_msg.header.stamp == stamp
    assert (front_msg.twist.linear.x, front_msg.twist.linear.y,
            front_msg.twist.angular.z) == pytest.approx(front)
    assert (rear_msg.twist.linear.x, rear_msg.twist.linear.y,
            rear_msg.twist.angular.z) == pytest.approx(rear)
    assert front_msg.header.frame_id == 'front_base'
    assert rear_msg.header.frame_id == 'rear_base'


@pytest.mark.parametrize('field, value', [
    ('hardware_ok', False), ('manual_ok', False), ('marker_ok', False),
    ('odom_ok', False), ('graph_ok', False), ('estop', True),
])
def test_all_required_real_feedback_gates_fail_closed(field, value):
    inputs = _safe_inputs()
    inputs[field] = value
    assert evaluate_rigid_pair(RigidPairTeleopLimits(), **inputs).outcome != 'CONTINUE'


def test_canonical_entrypoint_owns_a_real_main_function():
    source = (Path(__file__).parents[1] / 'cooperative_parking_robot' /
              'rigid_pair_teleop_node.py').read_text(encoding='utf-8')
    assert 'def main(args=None):' in source
    assert 'return _main(args=args)' in source


def test_web_post_origin_guard_allows_cli_but_rejects_other_sites():
    assert request_origin_is_same_host(None, 'robot-1.local:5007')
    assert request_origin_is_same_host(
        'http://robot-1.local:5007', 'robot-1.local:5007')
    assert not request_origin_is_same_host(
        'https://evil.example', 'robot-1.local:5007')


def test_launch_does_not_start_auto_drive_and_rigid_teleop_together_by_default():
    source = (Path(__file__).parents[1] / 'launch' /
              'cooperative_drive_test_rear.launch.py').read_text(encoding='utf-8')
    assert "'enable_drive_test_dashboard', default_value='true'" in source
    assert "'enable_rigid_pair_teleop', default_value='false'" in source
    assert "'require_fused_odom', default_value='false'" in source
    assert "'require_cctv_marker', default_value='false'" in source
    assert "default_value=str(DEFAULT_WHEELBASE_M)" in source
    assert "'enable_keyboard_follow', default_value='false'" in source
    assert "'id0_calibration'" in source
    assert source.count("parameters=[LaunchConfiguration('id0_calibration'), {") == 2
    assert "FindPackageShare('cooperative_parking_robot')" in source
    assert source.count("'pair_separation_m': _float('rigid_pair_separation_m')") == 2


def test_status_ui_exposes_read_only_camera_placement_guide_only():
    package = Path(__file__).parents[1] / 'cooperative_parking_robot'
    node_source = (package / 'keyboard_follow_node.py').read_text(
        encoding='utf-8')
    core_source = (package / 'rigid_pair_teleop_core.py').read_text(
        encoding='utf-8')
    for text in ('placement', '카메라 후보'):
        assert text in node_source
    for text in ('마커 찾기', '안정화 중', '앞뒤 조정', '좌우 조정',
                 'yaw 조정', '정렬 후보'):
        assert text in core_source
    assert 'api/align' not in node_source
    assert 'ALIGNING' not in node_source
    assert '표시 전용이며 Arm gate가 아니고 물리 정렬을 보증하지 않습니다.' in node_source
    assert '현재 자세 기준 준비 (후보 무관)' in node_source
    assert 'const signed =' in node_source
    assert 'statusInFlight' in node_source
    assert 'renderDisconnected' in node_source
    assert '!response.ok' in node_source
    assert 'invalid status payload' in node_source
    assert 'invalid config' in node_source
    assert 'AbortController' in node_source
    assert 'STATUS_FETCH_TIMEOUT_MS = 800' in node_source
    assert 'STATUS_WATCHDOG_MS = 1000' in node_source
    assert 'lastSuccessfulStatusMs' in node_source
    assert 'setInterval(statusWatchdog, 100)' in node_source
    assert 'calibration_available' in node_source
    assert 'estimate_available' in node_source
    assert 'Front ID0가 Rear 기준 왼쪽, 목표 0' in node_source
    assert 'Front가 Rear보다 CCW, 목표 0' in node_source
    assert 'inference_available' not in node_source


def test_preview_pose_badge_requires_true_visible_and_valid_pose_contract():
    source = (Path(__file__).parents[1] / 'cooperative_parking_robot' /
              'camera_preview_node.py').read_text(encoding='utf-8')
    assert 'rp.fresh && rp.visible === true' in source
    assert 'ID0 raw 카메라→마커' in source
    assert 'relative_pose_frame' in source
    assert 'relative_pose_gate.accept' in source
    assert 'stamp_to_ns(msg.header.stamp)' in source


def test_node_keeps_raw_id0_reference_out_of_the_rotation_lever_arm():
    source = (Path(__file__).parents[1] / 'cooperative_parking_robot' /
              'keyboard_follow_node.py').read_text(encoding='utf-8')
    call = source.rsplit('split_pair_centre_twist(', 1)[1].split(')', 1)[0]
    assert 'self.pair_separation_m' in call
    assert 'self.reference[0]' not in call
