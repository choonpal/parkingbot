"""Regression tests for underbody entry geometry and control contracts."""

import math
from pathlib import Path

import pytest

from cooperative_parking_robot.vehicle_entry import (
    approach_motion_metrics,
    approach_longitudinal,
    exit_longitudinal_translation,
    inter_robot_gap,
    initial_align_phase,
    initial_approach_phase,
    marker_loss_speed_scale,
    axle_longitudinal,
    plan_around_vehicle,
    plan_peer_safe_approach,
    projected_robot_x_offset,
    rear_scan_speed_from_relative,
    relative_alignment_is_consistent,
    scan_direction,
    segment_intersects_open_rect,
    standoff_longitudinal,
    target_axle_index,
    validate_wheelbase_clearance,
    vehicle_to_world,
    world_to_vehicle,
)
from cooperative_parking_robot.wheel_edge_detector import (
    AxleSequenceDetector, DualWheelEdgeDetector,
)


ROOT = Path(__file__).resolve().parents[1]


def test_vehicle_frame_round_trip_with_nonzero_yaw():
    vehicle = (1.2, -0.4, math.radians(27.0))
    world = vehicle_to_world(0.31, -0.08, *vehicle)
    longitudinal, lateral = world_to_vehicle(*world, *vehicle)
    assert longitudinal == pytest.approx(0.31)
    assert lateral == pytest.approx(-0.08)


def test_roles_enter_from_same_rear_end_and_select_different_axles():
    assert standoff_longitudinal("front", 0.30) == -0.30
    assert standoff_longitudinal("rear", 0.30) == -0.30
    assert axle_longitudinal("front", 0.70) == 0.35
    assert axle_longitudinal("rear", 0.70) == -0.35
    assert scan_direction("front") == 1.0
    assert scan_direction("rear") == 1.0
    assert target_axle_index("front") == 2
    assert target_axle_index("rear") == 1
    assert approach_longitudinal("front", 0.85, 0.70) == -0.85
    assert approach_longitudinal("rear", 0.85, 0.70) == pytest.approx(-1.55)


def test_approach_motion_metrics_accepts_progress_toward_staging():
    travelled, progress, cross_track, alignment = approach_motion_metrics(
        (0.0, 0.0), (0.05, 0.0), (1.0, 0.0))
    assert travelled == pytest.approx(0.05)
    assert progress == pytest.approx(0.05)
    assert cross_track == pytest.approx(0.0)
    assert alignment == pytest.approx(1.0)


def test_approach_motion_metrics_exposes_wrong_or_sideways_motion():
    opposite = approach_motion_metrics(
        (0.0, 0.0), (-0.05, 0.0), (1.0, 0.0))
    sideways = approach_motion_metrics(
        (0.0, 0.0), (0.0, 0.05), (1.0, 0.0))
    assert opposite[3] == pytest.approx(-1.0)
    assert sideways[1] == pytest.approx(0.0)
    assert sideways[2] == pytest.approx(0.05)
    assert sideways[3] == pytest.approx(0.0)


def test_front_departs_axially_before_crossing_rear_lane():
    start = (-2.89, -0.50)
    rear = (-2.90, 0.0)
    goal = (-0.85, 0.0)
    route = plan_peer_safe_approach(
        start, goal, rear, robot_length=0.565,
        robot_width=0.420, clearance=0.06)
    assert len(route) == 2
    departure, staging = route
    assert departure == pytest.approx((-2.275, -0.50))
    assert staging == goal
    assert abs(departure[0] - rear[0]) == pytest.approx(0.625)


def test_rear_direct_route_is_safe_after_front_has_staged():
    route = plan_peer_safe_approach(
        (-2.90, 0.0), (-1.635, 0.0), (-0.85, 0.0),
        robot_length=0.565, robot_width=0.420, clearance=0.06)
    assert route == [(-1.635, 0.0)]


def test_simultaneous_entry_stages_and_scans_both_roles_together():
    for role in ("front", "rear"):
        assert initial_approach_phase(role, True) == "WAIT_TARGET"
        assert initial_align_phase(role, True) == "WAIT_PEER_STAGED"


def test_front_first_entry_remains_available_as_fallback():
    assert initial_approach_phase("front", False) == "WAIT_TARGET"
    assert initial_approach_phase("rear", False) == "WAIT_FRONT_STAGED"
    assert initial_align_phase("front", False) == "WAIT_REAR_OBSERVATION"
    assert initial_align_phase("rear", False) == "WAIT_FRONT_ALIGNED"


def test_same_direction_exit_keeps_front_leading_and_clears_rear():
    assert exit_longitudinal_translation(
        "front", 0.50, 0.70, True, 1) == pytest.approx(1.20)
    assert exit_longitudinal_translation(
        "rear", 0.50, 0.70, True, 1) == pytest.approx(1.20)


def test_split_exit_fallback_uses_nearest_vehicle_ends():
    assert exit_longitudinal_translation(
        "front", 0.50, 0.70, False, 1) == pytest.approx(0.50)
    assert exit_longitudinal_translation(
        "rear", 0.50, 0.70, False, 1) == pytest.approx(-0.50)


def test_070_wheelbase_clears_two_0565_robots():
    assert inter_robot_gap(0.70, 0.565) == pytest.approx(0.135)
    assert validate_wheelbase_clearance(0.70, 0.565, 0.10) == pytest.approx(
        0.135)
    with pytest.raises(ValueError, match="need at least"):
        validate_wheelbase_clearance(0.64, 0.565, 0.10)


def _feed_axle(detector, center, start_time):
    result = None
    for side, distance, position, stamp in (
        ("left", 0.05, center - 0.10, start_time),
        ("right", 0.05, center - 0.09, start_time + 0.01),
        ("left", 0.20, center + 0.10, start_time + 0.02),
        ("right", 0.20, center + 0.09, start_time + 0.03),
    ):
        event = detector.update(side, distance, position, stamp)
        if event is not None:
            result = event
    return result


def test_front_uses_second_axle_and_rear_uses_first_axle():
    front = AxleSequenceDetector(
        target_index=2, expected_spacing_m=0.70,
        spacing_tolerance_m=0.05, direction=1.0, window_size=1)
    first = _feed_axle(front, -0.35, 0.0)
    second = _feed_axle(front, 0.35, 1.0)
    assert (first.index, first.final) == (1, False)
    assert (second.index, second.final) == (2, True)

    rear = AxleSequenceDetector(
        target_index=1, expected_spacing_m=0.70,
        spacing_tolerance_m=0.05, direction=1.0, window_size=1)
    only = _feed_axle(rear, -0.35, 0.0)
    assert (only.index, only.final) == (1, True)


def test_axle_sequence_rejects_false_pair_outside_absolute_vehicle_window():
    front = AxleSequenceDetector(
        target_index=2, expected_spacing_m=0.70,
        spacing_tolerance_m=0.05, direction=1.0,
        expected_first_position_m=-0.35,
        position_tolerance_m=0.15, window_size=1)

    assert _feed_axle(front, -0.75, 0.0) is None
    rear_axle = _feed_axle(front, -0.35, 1.0)
    front_axle = _feed_axle(front, 0.35, 2.0)
    assert (rear_axle.index, rear_axle.final) == (1, False)
    assert (front_axle.index, front_axle.final) == (2, True)


def test_rear_rejects_false_first_pair_then_accepts_actual_rear_axle():
    rear = AxleSequenceDetector(
        target_index=1, expected_spacing_m=0.70,
        spacing_tolerance_m=0.05, direction=1.0,
        expected_first_position_m=-0.35,
        position_tolerance_m=0.15, window_size=1)

    assert _feed_axle(rear, 0.20, 0.0) is None
    actual = _feed_axle(rear, -0.35, 1.0)
    assert (actual.index, actual.final) == (1, True)


def test_aruco_coarse_guard_never_owns_final_axle_center():
    assert rear_scan_speed_from_relative(
        0.70, 0.70, 0.03, 0.006, 0.12, 0.10) > 0.0
    assert rear_scan_speed_from_relative(
        0.59, 0.70, 0.03, 0.006, 0.12, 0.10) is None
    assert relative_alignment_is_consistent(
        0.72, math.radians(2), 0.70, 0.06, math.radians(4))
    assert marker_loss_speed_scale(1.0, 0.75, 1.5) == 0.35


def test_final_relative_alignment_rejects_excess_lateral_offset():
    assert relative_alignment_is_consistent(
        0.70, math.radians(2), 0.70, 0.06, math.radians(4),
        relative_lateral=0.02, lateral_tolerance=0.03)
    assert not relative_alignment_is_consistent(
        0.70, math.radians(2), 0.70, 0.06, math.radians(4),
        relative_lateral=0.031, lateral_tolerance=0.03)


def test_approach_route_avoids_protected_vehicle_envelope():
    start = (-0.40, 0.0)
    goal = (0.32, -0.20)
    assert segment_intersects_open_rect(start, goal, 0.28, 0.16)
    route = plan_around_vehicle(start, goal, 0.28, 0.16)
    previous = start
    for waypoint in route:
        assert not segment_intersects_open_rect(
            previous, waypoint, 0.28, 0.16)
        previous = waypoint
    assert route[-1] == goal


def test_route_rejects_robot_already_inside_vehicle_envelope():
    with pytest.raises(ValueError, match="start lies inside"):
        plan_around_vehicle((0.0, 0.0), (0.32, -0.20), 0.28, 0.16)


def test_sensor_mount_offset_is_projected_on_vehicle_axis():
    assert projected_robot_x_offset(0.025, 0.0, 0.0) == pytest.approx(
        0.025)
    assert projected_robot_x_offset(
        0.025, math.radians(5.0), 0.0) == pytest.approx(
            0.025 * math.cos(math.radians(5.0)))


def test_one_sided_wheel_edge_never_completes_pair():
    detector = DualWheelEdgeDetector(window_size=1, pair_timeout_s=0.5)
    samples = [
        ("left", 0.06, -0.20, 0.0),
        ("right", 0.50, -0.20, 0.1),
        ("left", 0.50, -0.10, 0.2),
        ("right", 0.50, -0.10, 0.3),
        ("right", 0.50, 0.00, 0.8),
    ]
    assert all(
        detector.update(side, distance, position, stamp) is None
        for side, distance, position, stamp in samples
    )


def test_source_contract_has_explicit_entry_exit_and_bounded_scan():
    move = (
        ROOT / "cooperative_parking_robot/individual_move_node.py"
    ).read_text()
    edge = (
        ROOT / "cooperative_parking_robot/ultrasonic_edge_node.py"
    ).read_text()
    state = (
        ROOT / "cooperative_parking_robot/robot_state_machine_node.py"
    ).read_text()

    for phase in (
        "TO_REAR_STAGING",
        "WAIT_REAR_OBSERVATION",
        "WAIT_FRONT_ALIGNED",
        "WAIT_PEER_STAGED",
        "SCAN_IN",
        "CENTER_AXLE",
        "EXIT_UNDERBODY",
        "EXIT_TO_SIDE",
        "RETURN_HOME",
        "WAIT_PEER_RETURN",
        "WAIT_PEER_EXIT_CLEAR",
        "WAIT_PEER_SIDE_CLEAR",
    ):
        assert phase in move
    assert "WHEEL_PAIR_NOT_DETECTED" in move
    assert "RETURN_SLOT_POSE_MISSING" in move
    assert "direct home return" not in move
    assert "command_vehicle_axis" in move
    assert '"WAIT_FRONT_ALIGNED"' in move
    assert '"WAIT_REAR_OBSERVATION"' in move
    assert "self.relative_is_fresh()" in move
    assert "final_relative_check" in move
    assert "rear_scan_speed_from_relative" in move
    assert "APPROACH_START_NOT_BEHIND_VEHICLE" in move
    assert "APPROACH_NOT_LONGITUDINAL_SAFE" in move
    assert "APPROACH_WRONG_DIRECTION" in move
    assert "APPROACH_CROSS_TRACK" in move
    assert "PEER_ODOM_STALE_FOR_APPROACH" in move
    assert "PEER_COLLISION_ENVELOPE" in move
    assert 'self.set_phase("TO_SIDE_STAGING")' not in move
    assert "wheel_center_s" in move
    assert "get_scan_start" not in move
    assert "self.side_offset" not in move
    assert "scan_y = ty -" not in move
    assert "active_target_pose" in move
    assert 'f"/{self.role}/wheel_center_s"' in edge
    assert "world_to_vehicle" in edge
    assert "active_target_pose" in edge
    assert "AxleSequenceDetector" in edge
    assert "target_axle_index" in edge
    assert '"/parking/target_pose"' not in edge
    assert "ULTRASONIC_{side.upper()}_STREAM_TIMEOUT" in edge
    assert 'f"/{self.role}/motion_fault"' in state
