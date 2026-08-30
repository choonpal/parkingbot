"""Regression checks for the measured vehicle/robot geometry."""

import math

from cooperative_parking_robot.vehicle_entry import (
    DEFAULT_WHEELBASE_M,
    MIN_INTER_ROBOT_GAP_M,
    ROBOT_LENGTH_M,
    inter_robot_gap,
)
from cooperative_parking_robot.fleet_manager_node import (
    select_planning_dimensions,
)


def test_measured_wheelbase_leaves_22cm_robot_gap():
    assert math.isclose(DEFAULT_WHEELBASE_M, 0.785, abs_tol=1e-12)
    assert math.isclose(ROBOT_LENGTH_M, 0.565, abs_tol=1e-12)
    assert math.isclose(MIN_INTER_ROBOT_GAP_M, 0.220, abs_tol=1e-12)
    assert math.isclose(
        inter_robot_gap(DEFAULT_WHEELBASE_M, ROBOT_LENGTH_M),
        0.220,
        abs_tol=1e-12,
    )


def test_measured_dimensions_require_explicit_planning_policy_and_validity():
    configured = (0.90, 0.35)
    measured = (1.20, 0.50)
    assert select_planning_dimensions(
        *configured, *measured, True, False) == configured
    assert select_planning_dimensions(
        *configured, *measured, False, True) == configured
    assert select_planning_dimensions(
        *configured, *measured, True, True) == measured
