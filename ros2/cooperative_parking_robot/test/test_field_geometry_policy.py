"""Regression tests for the field geometry policy."""

import pytest

from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    check_loaded_overhang_clearance,
    check_vehicle_only_slot_fit,
    plan_route_around_rectangles,
    projected_half_extents,
    route_is_clear,
)


def test_vehicle_only_slot_accepts_120_by_80_demo_slot():
    result = check_vehicle_only_slot_fit(
        slot_length_m=1.20,
        slot_width_m=0.80,
        vehicle_length_m=0.90,
        vehicle_width_m=0.35,
        longitudinal_margin_m=0.05,
        lateral_margin_m=0.05,
    )
    assert result.fits
    assert result.length_clearance_m == pytest.approx(0.20)
    assert result.width_clearance_m == pytest.approx(0.35)


def test_vehicle_only_slot_rejects_oversize_vehicle():
    result = check_vehicle_only_slot_fit(
        1.20, 0.80, 1.15, 0.75, 0.05, 0.05)
    assert not result.fits
    assert result.reason == "VEHICLE_SLOT_TOO_SHORT_AND_NARROW"


def test_23cm_back_clearance_accepts_default_loaded_geometry():
    # Existing defaults:
    # pair span = wheelbase 0.70 + robot length 0.565
    # loaded footprint adds 0.06 m base margin at both ends -> 1.385 m.
    result = check_loaded_overhang_clearance(
        slot_length_m=1.20,
        loaded_length_m=1.385,
        loaded_collision_margin_m=0.06,
        back_clearance_m=0.23,
        reserve_m=0.03,
    )
    assert result.fits
    assert result.overhang_each_end_m == pytest.approx(0.1525)
    assert result.required_back_clearance_m == pytest.approx(0.1825)
    assert result.clearance_m == pytest.approx(0.0475)


def test_23cm_back_clearance_rejects_longer_loaded_geometry():
    result = check_loaded_overhang_clearance(
        slot_length_m=1.20,
        loaded_length_m=1.55,
        loaded_collision_margin_m=0.06,
        back_clearance_m=0.23,
        reserve_m=0.03,
    )
    assert not result.fits
    assert result.reason == "INSUFFICIENT_SLOT_BACK_CLEARANCE"


def test_side_by_side_home_route_avoids_vehicle_and_stationary_peer():
    vehicle = AxisAlignedRect(
        center_s_m=0.0,
        center_d_m=0.0,
        half_s_m=0.793,
        half_d_m=0.373,
    )
    peer_half_s, peer_half_d = projected_half_extents(
        length_m=0.565,
        width_m=0.275,
        relative_yaw_rad=0.0,
        margin_m=0.0,
    )
    moving_half_s, moving_half_d = projected_half_extents(
        length_m=0.565,
        width_m=0.275,
        relative_yaw_rad=0.0,
        margin_m=0.0,
    )
    stationary_peer = AxisAlignedRect(
        center_s_m=-3.00,
        center_d_m=0.20,
        half_s_m=peer_half_s + moving_half_s + 0.10,
        half_d_m=peer_half_d + moving_half_d + 0.10,
    )
    # Field layout HOME poses become (-3.0,-0.2) and (-3.0,+0.2)
    # in the vehicle frame: 0.40 m centre spacing.
    start = (-3.00, -0.20)
    goal = (-0.85, 0.0)
    rectangles = (vehicle, stationary_peer)

    route = plan_route_around_rectangles(
        start, goal, rectangles, corner_margin_m=0.03)

    assert route[-1] == pytest.approx(goal)
    assert route_is_clear(start, route, rectangles)


def test_route_detours_when_direct_segment_crosses_vehicle():
    vehicle = AxisAlignedRect(0.0, 0.0, 0.8, 0.4)
    start = (-1.2, 0.0)
    goal = (1.2, 0.0)
    route = plan_route_around_rectangles(
        start, goal, (vehicle,), corner_margin_m=0.03)
    assert len(route) >= 3
    assert route_is_clear(start, route, (vehicle,))
