"""메카넘 경로추종/강체분배 회귀 테스트."""

import math
from pathlib import Path

import pytest

from cooperative_parking_robot.pure_pursuit import PurePursuit
from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics
from cooperative_parking_robot.mission_protocol import (
    make_arrival_status,
    parse_arrival_status,
)


ROOT = Path(__file__).resolve().parents[1]


def test_target_behind_commands_reverse_without_yaw_rotation():
    tracker = PurePursuit(lookahead=0.15, max_speed=0.08)
    tracker.set_path([(0.0, 0.0), (-1.0, 0.0)])
    vx, vy, omega = tracker.compute(0.0, 0.0, 0.0)
    assert vx < 0.0
    assert abs(vy) < 1e-9
    assert omega == 0.0


def test_target_to_side_commands_mecanum_lateral_motion_without_rotation():
    tracker = PurePursuit(lookahead=0.15, max_speed=0.08)
    tracker.set_path([(0.0, 0.0), (0.0, 1.0)])
    vx, vy, omega = tracker.compute(0.0, 0.0, 0.0)
    assert abs(vx) < 1e-9
    assert vy > 0.0
    assert omega == 0.0


def test_passed_corner_is_not_selected_again():
    tracker = PurePursuit(lookahead=0.15, max_speed=0.08)
    tracker.set_path([(0.0, 0.0), (1.0, 0.0), (1.0, 1.0)])
    vx, vy, omega = tracker.compute(1.0, 0.20, 0.0)
    assert vx >= -1e-9
    assert vy > 0.0
    assert omega == 0.0


def test_rotate_to_path_remains_explicit_opt_in():
    tracker = PurePursuit(
        lookahead=0.15,
        max_speed=0.08,
        max_omega=0.30,
        rotate_to_path=True,
    )
    tracker.set_path([(0.0, 0.0), (-1.0, 0.0)])
    vx, _, omega = tracker.compute(0.0, 0.0, 0.0)
    assert vx < 0.0
    assert math.isclose(
        abs(omega), 0.30, rel_tol=0.0, abs_tol=1e-9)


def test_virtual_body_rotation_splits_opposite_lateral_velocities():
    kinematics = RigidBodyKinematics(wheelbase=0.25)
    front, rear = kinematics.split(0.0, 0.0, 0.4)
    assert front[1] > 0.0
    assert rear[1] < 0.0
    assert math.isclose(
        front[1], -rear[1], rel_tol=0.0, abs_tol=1e-12)


@pytest.mark.parametrize('error', [0.04, -0.04])
def test_lateral_correction_reduces_relative_lateral_error(error):
    """Relative y_dot = front_vy - rear_vy near aligned yaw."""
    kinematics = RigidBodyKinematics(0.785)
    front, rear = kinematics.apply_relative_correction(
        (0.0, 0.0, 0.0), (0.0, 0.0, 0.0),
        corr_x=0.0, corr_y=error, corr_yaw=0.0)
    relative_lateral_rate = front[1] - rear[1]
    assert error * relative_lateral_rate < 0.0


def test_zero_lateral_correction_inside_deadband_changes_nothing():
    kinematics = RigidBodyKinematics(0.785)
    error = 0.002
    deadbanded = 0.0 if abs(error) <= 0.003 else error
    front, rear = kinematics.apply_relative_correction(
        (0.02, 0.01, 0.0), (0.02, 0.01, 0.0),
        corr_x=0.0, corr_y=deadbanded, corr_yaw=0.0)
    assert front[1] == pytest.approx(0.01)
    assert rear[1] == pytest.approx(0.01)


def test_relative_pose_lateral_matches_rear_body_left_positive():
    longitudinal, lateral, yaw = RigidBodyKinematics.relative_pose_in_rear_frame(
        {'x': 1.0, 'y': 0.1, 'theta': 0.0},
        {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    assert longitudinal == pytest.approx(1.0)
    assert lateral == pytest.approx(0.1)
    assert yaw == pytest.approx(0.0)


def test_relative_x_is_not_euclidean_distance_when_lateral_is_nonzero():
    relative_x, relative_y, _ = RigidBodyKinematics.relative_pose_in_rear_frame(
        {'x': 0.785, 'y': 0.030, 'theta': 0.0},
        {'x': 0.0, 'y': 0.0, 'theta': 0.0})
    assert relative_x == pytest.approx(0.785)
    assert relative_y == pytest.approx(0.030)
    assert relative_x != pytest.approx(math.hypot(0.785, 0.030))


def test_relative_pose_rotates_world_displacement_into_rear_frame():
    relative_x, relative_y, relative_yaw = (
        RigidBodyKinematics.relative_pose_in_rear_frame(
            {'x': 0.97, 'y': 1.785, 'theta': math.pi / 2.0},
            {'x': 1.0, 'y': 1.0, 'theta': math.pi / 2.0}))
    assert relative_x == pytest.approx(0.785)
    assert relative_y == pytest.approx(0.030)
    assert relative_yaw == pytest.approx(0.0)


def test_pair_saturation_preserves_added_lateral_correction_ratio():
    kinematics = RigidBodyKinematics(0.785)
    front, rear = kinematics.apply_relative_correction(
        (0.08, 0.05, 0.2), (0.08, -0.05, 0.2),
        corr_x=0.03, corr_y=0.04, corr_yaw=0.05)
    limited_front, limited_rear = kinematics.limit_twist_pair(
        front, rear, linear_limit=0.05, angular_limit=0.15)
    scales = [limited / original for limited, original in zip(
        (*limited_front, *limited_rear), (*front, *rear)) if original != 0.0]
    assert max(scales) == pytest.approx(min(scales))


def test_offset_vehicle_centre_stays_fixed_during_rotation_command():
    """로봇 중점과 차량 중심이 달라도 회전 제어점은 표류하지 않는다."""
    kinematics = RigidBodyKinematics(0.70)
    dx, dy = 0.20, -0.05
    vx, vy, omega = kinematics.control_point_twist_to_centre(
        0.0, 0.0, 0.4, dx, dy)
    # body-frame 차량 중심 속도 = pair 속도 + omega x offset = 0.
    vehicle_vx = vx - omega * dy
    vehicle_vy = vy + omega * dx
    assert abs(vehicle_vx) < 1e-12
    assert abs(vehicle_vy) < 1e-12


def test_body_frame_offset_rotates_with_vehicle_yaw():
    x, y, _ = RigidBodyKinematics.control_point_pose(
        1.0, 2.0, math.pi / 2.0, 0.20, 0.0)
    assert abs(x - 1.0) < 1e-12
    assert abs(y - 2.20) < 1e-12


def test_rigid_pair_limiter_preserves_offset_rotation_compensation():
    """한쪽이 상한에 걸려도 차량 중심 고정 관계가 유지되어야 한다."""
    kinematics = RigidBodyKinematics(0.70)
    dx, dy = 0.20, -0.05
    centre = kinematics.control_point_twist_to_centre(
        0.0, 0.0, 0.10, dx, dy)
    front, rear = kinematics.split(*centre)
    limited_front, limited_rear = kinematics.limit_twist_pair(
        front, rear, linear_limit=0.024, angular_limit=0.10)

    # 모든 성분에 동일 scale이 적용됐으므로 두 로봇 omega가 같고,
    # 다시 합친 로봇 중점 속도도 차량 중심의 제자리 회전을 만족한다.
    assert math.isclose(
        limited_front[2], limited_rear[2], rel_tol=0.0, abs_tol=1e-12)
    pair_vx = 0.5 * (limited_front[0] + limited_rear[0])
    pair_vy = 0.5 * (limited_front[1] + limited_rear[1])
    omega = 0.5 * (limited_front[2] + limited_rear[2])
    assert abs(pair_vx - omega * dy) < 1e-12
    assert abs(pair_vy + omega * dx) < 1e-12
    assert max(
        math.hypot(limited_front[0], limited_front[1]),
        math.hypot(limited_rear[0], limited_rear[1])) <= 0.024 + 1e-12


def test_mission_path_and_slot_use_transient_local_qos():
    fleet = (
        ROOT / "cooperative_parking_robot/fleet_manager_node.py"
    ).read_text()
    sync = (
        ROOT / "cooperative_parking_robot/rigid_body_sync_node.py"
    ).read_text()

    for source in (fleet, sync):
        assert "DurabilityPolicy.TRANSIENT_LOCAL" in source
        assert "ReliabilityPolicy.RELIABLE" in source

    assert "Path, '/virtual_robot/waypoints', self.mission_qos" in fleet
    assert "PoseStamped, '/parking/slot_pose', self.mission_qos" in fleet
    assert (
        "Path, '/virtual_robot/waypoints', self.path_cb, self.mission_qos"
        in sync
    )
    assert (
        "PoseStamped, '/parking/slot_pose', self.slot_cb, self.mission_qos"
        in sync
    )


def test_rigid_sync_gates_latched_path_to_lifted_drive_state():
    source = (
        ROOT / "cooperative_parking_robot/rigid_body_sync_node.py"
    ).read_text()
    assert "Bool, '/robot/lifted', self.vehicle_lifted_cb" in source
    assert "String, '/front/robot_state', self.front_state_cb" in source
    assert "String, '/rear/robot_state', self.rear_state_cb" in source
    assert "self.front_robot_state != 'DRIVE'" in source
    assert "self.rear_robot_state != 'DRIVE'" in source


def test_release_waits_for_both_robots_before_return():
    source = (
        ROOT / "cooperative_parking_robot/robot_state_machine_node.py"
    ).read_text()
    assert "/release/{self.other_role}_done" in source
    assert "/release/{self.role}_done" in source
    assert 'self.publish_ready_stage("RETURN")' in source
    assert 'self.maybe_publish_commit("RETURN")' in source
    assert 'if "RETURN" in self.committed_stages:' in source
    release_guard = source.split(
        'elif self.state == "RELEASE":', 1)[1].split(
        'elif self.state == "RETURN":', 1)[0]
    assert "if not self.release_done:" in release_guard
    assert "self.send_action_with_retry(\"release\")" in release_guard


def test_cctv_vehicle_feedback_only_updates_during_active_transport():
    source = (
        ROOT / "cooperative_parking_robot/rigid_body_sync_node.py"
    ).read_text()
    callback = source.split("def cctv_feedback_cb", 1)[1].split(
        "def slot_cb", 1)[0]
    assert "not self.has_path" in callback
    assert "not self.vehicle_lifted" in callback
    assert "self.front_robot_state != 'DRIVE'" in callback
    assert "self.rear_robot_state != 'DRIVE'" in callback


def test_arrived_status_carries_map_vehicle_pose_and_plan_correlation():
    status = make_arrival_status(1.2, -0.4, 3.5, 123)

    assert status['error'] == 'ARRIVED'
    assert status['final_vehicle_pose']['frame_id'] == 'map'
    assert status['final_vehicle_pose']['yaw'] == pytest.approx(
        -2.7831853071795862)
    assert parse_arrival_status(status, 123) == pytest.approx(
        (1.2, -0.4, -2.7831853071795862))


def test_arrived_status_rejects_wrong_frame_or_plan_stamp():
    status = make_arrival_status(1.2, -0.4, 0.2, 123)

    assert parse_arrival_status(status, 122) is None
    status['final_vehicle_pose']['frame_id'] = 'odom'
    assert parse_arrival_status(status, 123) is None
