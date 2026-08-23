import math
from pathlib import Path

import pytest

from cooperative_parking_robot.astar_planner import AStarPlanner
from cooperative_parking_robot.loaded_footprint import (
    compute_loaded_footprint,
)


ROOT = Path(__file__).resolve().parents[1]


def test_known_robot_dimensions_form_combined_loaded_rectangle():
    footprint = compute_loaded_footprint(
        wheelbase_m=0.70,
        robot_length_m=0.565,
        robot_width_m=0.275,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        safety_margin_m=0.06,
    )

    assert footprint.length_m == pytest.approx(1.385)
    assert footprint.width_m == pytest.approx(0.47)
    assert footprint.half_extent_cells(0.05) == (14, 5)


def test_vehicle_body_overrides_robot_pair_when_it_is_larger():
    footprint = compute_loaded_footprint(
        wheelbase_m=0.70,
        robot_length_m=0.565,
        robot_width_m=0.275,
        vehicle_length_m=1.40,
        vehicle_width_m=0.50,
        safety_margin_m=0.05,
    )

    assert footprint.length_m == pytest.approx(1.50)
    assert footprint.width_m == pytest.approx(0.60)


def test_vehicle_centre_offset_expands_symmetric_loaded_envelope():
    footprint = compute_loaded_footprint(
        wheelbase_m=0.70,
        robot_length_m=0.565,
        robot_width_m=0.275,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        safety_margin_m=0.06,
        vehicle_center_offset_x_m=0.20,
        vehicle_center_offset_y_m=0.05,
    )
    assert footprint.length_m == pytest.approx(1.785)
    assert footprint.width_m == pytest.approx(0.495)


def test_rectangular_inflation_and_map_boundary_are_applied():
    width, height = 11, 9
    grid = [0] * (width * height)
    grid[4 * width + 5] = 100
    planner = AStarPlanner(
        resolution=1.0,
        footprint_half_length_m=2.0,
        footprint_half_width_m=1.0,
    )

    inflated = planner._inflate(grid, width, height)

    for y in range(3, 6):
        for x in range(3, 8):
            assert inflated[y * width + x] == 100
    assert inflated[2 * width + 5] < 50
    assert inflated[4 * width + 2] < 50

    # A centre in the boundary band would put part of the footprint outside.
    assert inflated[4 * width + 0] == 100
    assert inflated[4 * width + 1] == 100
    assert inflated[0 * width + 8] == 100


def test_diagonal_corner_cutting_between_two_obstacles_is_rejected():
    width, height = 3, 3
    grid = [0] * (width * height)
    grid[0 * width + 1] = 100
    grid[1 * width + 0] = 100
    planner = AStarPlanner(
        resolution=1.0,
        footprint_half_length_m=0.0,
        footprint_half_width_m=0.0,
    )

    assert planner.plan(
        grid, width, height, (0.5, 0.5), (1.5, 1.5)) is None


def test_unknown_cells_are_blocked_by_default():
    grid = [0, -1, 0]
    safe = AStarPlanner(
        resolution=1.0,
        footprint_half_length_m=0.0,
        footprint_half_width_m=0.0,
    )
    permissive = AStarPlanner(
        resolution=1.0,
        footprint_half_length_m=0.0,
        footprint_half_width_m=0.0,
        unknown_is_occupied=False,
    )

    assert safe.plan(grid, 3, 1, (0.5, 0.5), (2.5, 0.5)) is None
    assert permissive.plan(
        grid, 3, 1, (0.5, 0.5), (2.5, 0.5)) is not None


def test_start_or_goal_whose_footprint_crosses_map_edge_is_rejected():
    planner = AStarPlanner(
        resolution=1.0,
        footprint_half_length_m=1.0,
        footprint_half_width_m=1.0,
    )
    grid = [0] * 25

    assert planner.plan(
        grid, 5, 5, (0.5, 2.5), (2.5, 2.5)) is None


def test_negative_map_origin_keeps_waiting_vehicle_pose_plannable():
    """Map coverage, not the vehicle pose, defines the physical boundary."""
    footprint = compute_loaded_footprint(
        wheelbase_m=0.70,
        robot_length_m=0.565,
        robot_width_m=0.275,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        safety_margin_m=0.06,
    )
    planner = AStarPlanner(
        resolution=0.05,
        footprint_half_length_m=footprint.half_length_m,
        footprint_half_width_m=footprint.half_width_m,
        origin_x_m=-0.40,
        origin_y_m=0.0,
    )
    width, height = 96, 77  # world x range: -0.40 .. 4.40 m
    grid = [0] * (width * height)

    path = planner.plan(
        grid, width, height, (0.60, 0.40), (1.20, 1.20))

    assert path is not None
    half_cell_diagonal = 0.05 / math.sqrt(2.0) + 1e-9
    assert math.dist(path[0], (0.60, 0.40)) <= half_cell_diagonal
    assert math.dist(path[-1], (1.20, 1.20)) <= half_cell_diagonal


def test_current_six_by_four_map_accepts_default_start_and_first_slot():
    footprint = compute_loaded_footprint(
        wheelbase_m=0.70,
        robot_length_m=0.565,
        robot_width_m=0.275,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        safety_margin_m=0.06,
    )
    planner = AStarPlanner(
        resolution=0.05,
        footprint_half_length_m=footprint.half_length_m,
        footprint_half_width_m=footprint.half_width_m,
    )
    width, height = 120, 80
    grid = [0] * (width * height)

    assert planner.plan(
        grid, width, height, (2.3, 0.6), (1.5, 3.5)) is not None


def test_fleet_manager_uses_vehicle_spec_and_live_virtual_start():
    source = (
        ROOT / "cooperative_parking_robot/fleet_manager_node.py"
    ).read_text(encoding="utf-8")

    assert "compute_loaded_footprint" in source
    assert "self.planner.set_footprint" in source
    assert "def current_virtual_start" in source
    assert "start = self.current_virtual_start()" in source
    assert "start = (self.wait_x, self.wait_y)" not in source
    assert "start = Pose2D(target_x, target_y, raw_start.yaw_rad)" in source
    # 빈 슬롯을 가까운 순으로만 고르지 않고, 결합 footprint
    # 적합성과 staging 회전/삽입 corridor까지 통과한 후보를 쓴다.
    assert "for slot, fit in compatible:" in source
    assert "make_approach_candidates" in source
    assert "_rotation_space_free" in source
    assert "_insertion_corridor_free" in source
