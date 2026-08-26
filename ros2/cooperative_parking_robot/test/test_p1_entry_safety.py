"""Behavior regressions for the P1 underbody-entry sensor contracts."""

import math
from pathlib import Path
import re
from types import SimpleNamespace

import pytest
import rclpy
from geometry_msgs.msg import PoseStamped
from rclpy.parameter import Parameter
from std_msgs.msg import String

from cooperative_parking_robot import individual_move_node as move_module
from cooperative_parking_robot import rigid_body_sync_node as sync_module
from cooperative_parking_robot.freshness import StampGate
from cooperative_parking_robot.individual_move_node import IndividualMoveNode
from cooperative_parking_robot.rigid_body_sync_node import RigidBodySyncNode
from cooperative_parking_robot.ultrasonic_edge_node import UltrasonicEdgeNode


ROOT = Path(__file__).resolve().parents[1]


def _individual_move_for_visual_fallback():
    node = object.__new__(IndividualMoveNode)
    node.phase = "SCAN_IN"
    node.cctv_marker_timeout = 0.50
    node.aruco_timeout = 0.30
    node.marker_slowdown = 0.75
    node.marker_stop = 1.50
    node.relative_marker_visible = False
    node.relative_receipt_time = None
    node.relative_x = None
    node.top_marker_visible = False
    node.top_visibility_received = False
    node.top_marker_receipt_time = None
    node.last_visual_observation_time = None
    node.relative_lost_since = None
    node.motion_speed_scale = 1.0
    node.stop_calls = 0
    node.fault_reasons = []
    node.stop = lambda: setattr(node, "stop_calls", node.stop_calls + 1)
    node.fault = node.fault_reasons.append
    return node


def test_last_top_marker_true_then_silence_slows_and_holds_without_fault(
        monkeypatch):
    node = _individual_move_for_visual_fallback()
    now = [100.0]
    monkeypatch.setattr(move_module.time, "monotonic", lambda: now[0])

    node.top_marker_cb(SimpleNamespace(data=True))
    assert node.update_visual_fallback()
    assert node.motion_speed_scale == 1.0

    now[0] = 100.80
    assert node.update_visual_fallback()
    assert node.motion_speed_scale == 0.35

    now[0] = 101.60
    assert not node.update_visual_fallback()
    assert node.motion_speed_scale == 0.0
    assert node.stop_calls == 1
    assert node.fault_reasons == []

    now[0] = 101.61
    node.top_marker_cb(SimpleNamespace(data=True))
    assert node.update_visual_fallback()
    assert node.motion_speed_scale == 1.0


def _pose(frame_id, stamp_ns, x=0.135, y=0.0, yaw=0.0, scale=1.0):
    msg = PoseStamped()
    msg.header.frame_id = frame_id
    msg.header.stamp.sec = stamp_ns // 1_000_000_000
    msg.header.stamp.nanosec = stamp_ns % 1_000_000_000
    msg.pose.position.x = x
    msg.pose.position.y = y
    msg.pose.orientation.z = scale * math.sin(yaw / 2.0)
    msg.pose.orientation.w = scale * math.cos(yaw / 2.0)
    return msg


def _individual_move_for_relative_pose(now_ns=10_000_000_000):
    node = object.__new__(IndividualMoveNode)
    node.aruco_distance_offset = 0.565
    node.relative_gate = StampGate(0.30, 0.10)
    node.relative_x = None
    node.relative_y = None
    node.relative_yaw = None
    node.relative_receipt_time = None
    node.last_visual_observation_time = None
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=now_ns))
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: warnings.append(message))
    node.warnings = warnings
    return node


def test_relative_pose_rejects_wrong_frame_without_poisoning_stamp_gate(
        monkeypatch):
    node = _individual_move_for_relative_pose()
    monkeypatch.setattr(move_module.time, "monotonic", lambda: 100.0)
    stamp = 9_900_000_000

    node.relative_pose_cb(_pose("map", stamp))
    assert node.relative_x is None

    node.relative_pose_cb(_pose("rear_base", stamp, yaw=0.2, scale=2.0))
    assert node.relative_x == pytest.approx(0.70)
    assert node.relative_yaw == pytest.approx(0.2)
    assert node.relative_receipt_time == 100.0


def test_relative_pose_rejects_invalid_quaternion_stale_and_duplicate_samples(
        monkeypatch):
    node = _individual_move_for_relative_pose()
    receipt_time = [100.0]
    monkeypatch.setattr(
        move_module.time, "monotonic", lambda: receipt_time[0])

    invalid = _pose("rear_base", 9_800_000_000)
    invalid.pose.orientation.z = 0.0
    invalid.pose.orientation.w = 0.0
    node.relative_pose_cb(invalid)
    assert node.relative_x is None

    stale = _pose("rear_base", 9_000_000_000)
    node.relative_pose_cb(stale)
    assert node.relative_x is None

    fresh = _pose("rear_base", 9_900_000_000, x=0.135)
    node.relative_pose_cb(fresh)
    assert node.relative_x == pytest.approx(0.70)

    receipt_time[0] = 101.0
    duplicate = _pose("rear_base", 9_900_000_000, x=0.50)
    node.relative_pose_cb(duplicate)
    assert node.relative_x == pytest.approx(0.70)
    assert node.relative_receipt_time == 100.0


def _rigid_sync_for_relative_pose(now_ns=10_000_000_000):
    node = object.__new__(RigidBodySyncNode)
    node.stamp_gates = {"aruco": StampGate(0.30, 0.10)}
    node.aruco_distance_offset = 0.565
    node.aruco_min_distance = 0.05
    node.aruco_max_distance = 1.50
    node.aruco_raw_dist = None
    node.aruco_dist = None
    node.aruco_yaw = None
    node.aruco_receipt_time = None
    node.marker_visible = False
    node.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=now_ns))
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: warnings.append(message))
    node.warnings = warnings
    return node


def test_rigid_sync_rejects_wrong_frame_and_invalid_quaternion_before_stamp(
        monkeypatch):
    node = _rigid_sync_for_relative_pose()
    monkeypatch.setattr(sync_module.time, "monotonic", lambda: 200.0)
    stamp = 9_900_000_000

    node.aruco_cb(_pose("map", stamp))
    assert node.aruco_dist is None

    invalid = _pose("rear_base", stamp)
    invalid.pose.orientation.z = 0.0
    invalid.pose.orientation.w = 0.0
    node.aruco_cb(invalid)
    assert node.aruco_dist is None

    node.aruco_cb(_pose("rear_base", stamp, yaw=-0.3, scale=3.0))
    assert node.aruco_dist == pytest.approx(0.70)
    assert node.aruco_yaw == pytest.approx(-0.3)
    assert node.aruco_receipt_time == 200.0


def _individual_move_for_final_relative_check():
    node = object.__new__(IndividualMoveNode)
    node.relative_marker_visible = True
    node.relative_receipt_time = 100.0
    node.relative_x = 0.70
    node.relative_y = 0.04
    node.relative_yaw = 0.0
    node.aruco_timeout = 0.30
    node.wheelbase = 0.70
    node.relative_distance_tolerance = 0.06
    node.relative_lateral_tolerance = 0.03
    node.relative_yaw_tolerance = math.radians(4.0)
    node.relative_mismatch_timeout = 0.50
    node.relative_mismatch_since = 99.0
    node.fault_reasons = []
    node.fault = node.fault_reasons.append
    warnings = []
    node.get_logger = lambda: SimpleNamespace(
        warn=lambda message, **_kwargs: warnings.append(message))
    node.warnings = warnings
    return node


def test_final_relative_lateral_mismatch_holds_without_motion_fault(
        monkeypatch):
    node = _individual_move_for_final_relative_check()
    monkeypatch.setattr(move_module.time, "monotonic", lambda: 100.0)

    assert not node.final_relative_check()
    assert node.fault_reasons == []

    node.relative_y = 0.02
    assert node.final_relative_check()


def test_missing_final_relative_pose_holds_without_motion_fault(monkeypatch):
    node = _individual_move_for_final_relative_check()
    node.relative_marker_visible = False
    monkeypatch.setattr(move_module.time, "monotonic", lambda: 100.0)

    assert not node.final_relative_check()
    assert node.fault_reasons == []


def test_ultrasonic_node_tracks_absolute_axle_windows_from_vehicle_spec():
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = UltrasonicEdgeNode(parameter_overrides=[
        Parameter("role", value="rear"),
        Parameter("window_size", value=1),
        Parameter("axle_position_tolerance_m", value=0.12),
    ])
    try:
        assert node.detector.expected_spacing == pytest.approx(0.785)
        assert node.detector.expected_first_position == pytest.approx(
            -0.785 / 2.0)
        assert (node.detector.expected_first_position +
                node.detector.expected_spacing) == pytest.approx(
                    0.785 / 2.0)
        assert node.detector.position_tolerance == pytest.approx(0.12)

        node.vehicle_spec_cb(String(data='{"wheelbase": 0.80}'))
        assert node.detector.expected_spacing == pytest.approx(0.80)
        assert node.detector.expected_first_position == pytest.approx(-0.40)
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()


def test_new_approach_does_not_reuse_previous_mission_visual_latches():
    node = object.__new__(IndividualMoveNode)
    node.robot_state = "IDLE"
    node.role = "front"
    node.simultaneous_entry = False
    node.relative_x = 0.70
    node.relative_y = 0.01
    node.relative_yaw = 0.0
    node.relative_receipt_time = 100.0
    node.relative_marker_visible = True
    node.top_marker_visible = True
    node.top_visibility_received = True
    node.top_marker_receipt_time = 100.0
    node.last_visual_observation_time = 100.0
    node.set_phase = lambda phase: setattr(node, "phase", phase)

    node.state_cb(SimpleNamespace(data="APPROACH"))

    assert node.relative_x is None
    assert node.relative_receipt_time is None
    assert not node.relative_marker_visible
    assert not node.top_marker_visible
    assert not node.top_visibility_received
    assert node.top_marker_receipt_time is None
    assert node.last_visual_observation_time is None


def test_p1_safety_parameters_reach_all_real_robot_launch_paths():
    front = (ROOT / "launch/front_robot.launch.py").read_text()
    rear = (ROOT / "launch/rear_robot.launch.py").read_text()
    full = (ROOT / "launch/full_system.launch.py").read_text()

    for launch in (front, rear):
        assert '"cctv_marker_timeout_s": _float(' in launch
        assert '"relative_lateral_tolerance_m": _float(' in launch
        assert '"axle_position_tolerance_m": _float(' in launch
        for argument in (
                "cctv_marker_timeout_s",
                "relative_lateral_tolerance_m",
                "axle_position_tolerance_m"):
            assert re.search(
                rf'DeclareLaunchArgument\(\s*"{argument}"', launch)

    assert "'cctv_marker_timeout_s': _float(" in full
    assert "'relative_lateral_tolerance_m': _float(" in full
    assert "'axle_position_tolerance_m': _float(" in full
    for argument in (
            "cctv_marker_timeout_s",
            "relative_lateral_tolerance_m",
            "axle_position_tolerance_m"):
        assert re.search(
            rf"DeclareLaunchArgument\(\s*'{argument}'", full)


def test_rigid_sync_marker_loss_holds_path_without_global_estop():
    owned_context = not rclpy.ok()
    if owned_context:
        rclpy.init()
    node = RigidBodySyncNode()
    try:
        node.front.update(x=0.35, y=0.0, theta=0.0)
        node.rear.update(x=-0.35, y=0.0, theta=0.0)
        node.sync_filters_initialized = True
        node.marker_lost_since = 100.0
        node.aruco_receipt_time = None
        node.front_top_marker_visible = False
        node.rear_top_marker_visible = False
        node.has_path = True

        assert not node.apply_sync_and_publish(
            0.02, 0.0, 0.0, 103.0,
            mode="PATH_TRACKING",
            linear_limit=node.max_speed,
            angular_limit=node.max_omega)
        assert not node.estop
        assert node.has_path
        assert node._err.startswith("MARKER_HOLD")
    finally:
        node.destroy_node()
        if owned_context:
            rclpy.shutdown()
