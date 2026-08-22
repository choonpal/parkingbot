"""Regression tests for directed vehicle yaw in the field merge adapter."""

import math

import pytest

from cooperative_parking_robot.field_heading_geometry import (
    circular_mean,
    normalize_angle,
    resolve_undirected_axis_yaw,
)


def test_waiting_axis_zero_resolves_to_180_degree_vehicle_heading():
    resolved = resolve_undirected_axis_yaw(0.0, math.pi)
    assert abs(normalize_angle(resolved - math.pi)) < 1e-12


def test_axis_near_minus_90_resolves_near_positive_90_reference():
    axis = math.radians(-88.0)
    reference = math.radians(92.0)
    resolved = resolve_undirected_axis_yaw(axis, reference)
    assert math.degrees(resolved) == pytest.approx(92.0)


def test_transport_axis_selects_heading_closest_to_robot_pair():
    axis = math.radians(5.0)
    robot_yaw = math.radians(-172.0)
    resolved = resolve_undirected_axis_yaw(axis, robot_yaw)
    assert math.degrees(resolved) == pytest.approx(-175.0)


def test_circular_mean_handles_wraparound():
    mean = circular_mean(math.radians(179.0), math.radians(-179.0))
    assert abs(abs(math.degrees(mean)) - 180.0) < 1e-9


def test_circular_mean_rejects_opposite_robot_headings():
    with pytest.raises(ValueError, match="no stable"):
        circular_mean(0.0, math.pi)
