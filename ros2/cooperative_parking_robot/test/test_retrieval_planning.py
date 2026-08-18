import math

import pytest

from cooperative_parking_robot.parking_geometry import ParkingSlot, Pose2D
from cooperative_parking_robot.retrieval_planning import (
    clear_source_vehicle,
    corridor_is_free,
    make_extraction_geometry,
    make_waiting_staging,
    sequential_routes_clear,
    simultaneous_routes_clear,
)
from cooperative_parking_robot.vehicle_entry import (
    approach_longitudinal,
    vehicle_to_world,
)


def open_grid(width=40, height=40):
    return [0] * (width * height), width, height


def test_extraction_clear_extends_staging_by_lookahead_and_margin():
    slot = ParkingSlot('A1', 2.0, 2.0, 1.8, 0.7, math.pi / 2.0)
    final_pose = Pose2D(2.02, 2.03, math.pi / 2.0)

    geometry = make_extraction_geometry(
        slot, final_pose, loaded_length_m=1.2, staging_gap_m=0.10,
        lookahead_m=0.15, safety_margin_m=0.06)

    assert geometry.source_staging.y_m < slot.center_y_m
    assert math.dist(
        geometry.source_staging.position,
        geometry.clear_pose.position) >= 0.21 - 1e-9
    assert geometry.source_staging.x_m == final_pose.x_m
    assert geometry.clear_pose.yaw_rad == final_pose.yaw_rad


def test_only_selected_source_vehicle_is_removed_from_planning_grid():
    grid, width, height = open_grid(30, 20)
    resolution = 0.1
    source_index = 5 * width + 5
    other_index = 5 * width + 15
    grid[source_index] = 100
    grid[other_index] = 100

    masked = clear_source_vehicle(
        grid, width, height, resolution,
        Pose2D(0.55, 0.55, 0.0), length_m=0.3, width_m=0.3)

    assert masked[source_index] == 0
    assert masked[other_index] == 100
    assert grid[source_index] == 100


def test_source_mask_covers_perception_fallback_square_only():
    grid, width, height = open_grid(40, 30)
    resolution = 0.1
    source = Pose2D(1.55, 1.55, math.pi / 2.0)
    # COCO/dual fallback raster: 0.90m axis-aligned square.
    for gy in range(11, 20):
        for gx in range(11, 20):
            grid[gy * width + gx] = 100
    other_index = 15 * width + 25
    grid[other_index] = 100

    masked = clear_source_vehicle(
        grid, width, height, resolution, source,
        length_m=0.90, width_m=0.35,
        minimum_mask_size_m=0.90)

    assert all(
        masked[gy * width + gx] == 0
        for gy in range(11, 20) for gx in range(11, 20))
    assert masked[other_index] == 100
    assert grid[15 * width + 15] == 100


def test_corridor_checks_oriented_body_against_obstacle_and_boundary():
    grid, width, height = open_grid(30, 20)
    resolution = 0.1
    assert corridor_is_free(
        grid, width, height, resolution,
        (0.6, 0.6), (2.0, 0.6), 0.0, 0.3, 0.2)

    blocked = list(grid)
    blocked[6 * width + 13] = 100
    assert not corridor_is_free(
        blocked, width, height, resolution,
        (0.6, 0.6), (2.0, 0.6), 0.0, 0.3, 0.2)
    assert not corridor_is_free(
        grid, width, height, resolution,
        (0.05, 0.05), (0.5, 0.05), 0.0, 0.3, 0.2)


def test_approach_corridor_tracks_rotation_toward_staging_yaw():
    grid, width, height = open_grid(40, 30)
    resolution = 0.1
    # 이 셀은 수평 body에는 닿지 않지만 staging에서 세로로 선 body에는 닿는다.
    grid[12 * width + 19] = 100

    assert corridor_is_free(
        grid, width, height, resolution,
        (1.0, 1.0), (2.0, 1.0), 0.0, 0.565, 0.275)
    assert not corridor_is_free(
        grid, width, height, resolution,
        (1.0, 1.0), (2.0, 1.0), 0.0, 0.565, 0.275,
        goal_yaw_rad=math.pi / 2.0,
        speed_mps=0.035, yaw_gain=1.5, max_yaw_rate=0.15)


def test_simultaneous_approach_uses_time_not_segment_intersection_only():
    footprint = (0.20, 0.16)
    crossing_but_separated = simultaneous_routes_clear(
        ((-2.0, 0.0), (1.0, 0.0)),
        ((0.0, -0.5), (0.0, 0.5)),
        speed_mps=1.0, robot_length_m=footprint[0],
        robot_width_m=footprint[1], minimum_gap_m=0.05)
    simultaneous_collision = simultaneous_routes_clear(
        ((-1.0, 0.0), (1.0, 0.0)),
        ((0.0, -1.0), (0.0, 1.0)),
        speed_mps=1.0, robot_length_m=footprint[0],
        robot_width_m=footprint[1], minimum_gap_m=0.05)

    assert crossing_but_separated
    assert not simultaneous_collision


def test_demo_p1_p4_require_existing_front_first_approach():
    front_home = (1.15, 0.60)
    rear_home = (0.45, 0.60)
    slot_y = 3.0
    slot_yaw = math.pi / 2.0
    wheelbase = 0.70
    entry_standoff = 0.85
    kwargs = {
        'speed_mps': 0.035,
        'robot_length_m': 0.565,
        'robot_width_m': 0.275,
        'minimum_gap_m': 0.10,
    }

    for slot_x in (1.5, 2.5, 3.5, 4.5):
        front_goal = vehicle_to_world(
            approach_longitudinal('front', entry_standoff, wheelbase),
            0.0, slot_x, slot_y, slot_yaw)
        rear_goal = vehicle_to_world(
            approach_longitudinal('rear', entry_standoff, wheelbase),
            0.0, slot_x, slot_y, slot_yaw)
        front_route = (front_home, front_goal)
        rear_route = (rear_home, rear_goal)

        assert not simultaneous_routes_clear(
            front_route, rear_route,
            front_goal_yaw_rad=slot_yaw,
            rear_goal_yaw_rad=slot_yaw,
            yaw_gain=1.5, max_yaw_rate=0.15,
            **kwargs), slot_x
        assert sequential_routes_clear(
            front_route, rear_route,
            front_goal_yaw_rad=slot_yaw,
            rear_goal_yaw_rad=slot_yaw,
            yaw_gain=1.5, max_yaw_rate=0.15,
            **kwargs), slot_x


def test_waiting_staging_and_insertion_corridor_are_explicit():
    waiting = Pose2D(2.3, 0.6, 0.0)
    staging = make_waiting_staging(
        waiting, loaded_length_m=1.2, staging_gap_m=0.10)
    grid, width, height = open_grid(60, 40)

    assert staging.position == pytest.approx((1.0, 0.6))
    assert staging.yaw_rad == pytest.approx(0.0)
    assert corridor_is_free(
        grid, width, height, 0.1,
        staging.position, waiting.position, waiting.yaw_rad, 1.2, 0.5)
