"""Regression tests for the field geometry policy."""

import math

import pytest

from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    check_loaded_overhang_clearance,
    check_vehicle_only_slot_fit,
    clamp_rotation_center,
    final_approach_polyline,
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


def _default_rotation_radius():
    return 0.5 * math.hypot(1.385 + 2.0 * 0.06,
                            0.470 + 2.0 * 0.06)


def test_p4_rotation_stage_is_moved_inside_right_and_bottom_boundaries():
    radius = _default_rotation_radius()
    # P4 nominal staging: centre (3.6,2.2), yaw +90deg,
    # slot half length .6 + loaded half .6925 + gap .1.
    result = clamp_rotation_center(
        x_m=3.60,
        y_m=2.20 - (0.60 + 1.385 / 2.0 + 0.10),
        map_width_m=4.40,
        map_height_m=3.83,
        rotation_radius_m=radius,
        boundary_margin_m=0.03,
        max_shift_m=0.60,
    )
    assert result.inset_m == pytest.approx(radius + 0.03)
    assert result.x_m == pytest.approx(4.40 - result.inset_m)
    assert result.y_m == pytest.approx(result.inset_m)
    assert result.shift_m == pytest.approx(0.0490892141)


def test_waiting_rotation_stage_shift_fits_field_limit():
    radius = _default_rotation_radius()
    # waiting pose (.6,.4), yaw 180deg; nominal staging is 1.485m
    # behind it, at x=2.085,y=.4.  Only the low-Y boundary forces a shift.
    result = clamp_rotation_center(
        x_m=0.60 + 1.385 + 0.10,
        y_m=0.40,
        map_width_m=4.40,
        map_height_m=3.83,
        rotation_radius_m=radius,
        boundary_margin_m=0.03,
        max_shift_m=0.60,
    )
    assert result.x_m == pytest.approx(2.085)
    assert result.y_m == pytest.approx(result.inset_m)
    assert result.shift_m == pytest.approx(0.4382581580)


def test_rotation_stage_rejects_too_small_map_or_excessive_shift():
    radius = _default_rotation_radius()
    with pytest.raises(ValueError, match="map is too small"):
        clamp_rotation_center(
            0.5, 0.5, 1.5, 1.5, radius, 0.03, 0.60)
    with pytest.raises(ValueError, match="exceeds"):
        clamp_rotation_center(
            2.085, 0.40, 4.40, 3.83, radius, 0.03, 0.20)


def test_final_approach_polyline_matches_lateral_then_longitudinal_control():
    start = (3.56174184198, 0.83825815802)
    goal = (3.60, 2.20)
    route = final_approach_polyline(start, goal, math.pi / 2.0)

    assert len(route) == 2
    aligned, final = route
    assert aligned == pytest.approx((3.60, start[1]))
    assert final == pytest.approx(goal)

    # After the first leg, the goal has zero lateral error in slot frame.
    ex = goal[0] - aligned[0]
    ey = goal[1] - aligned[1]
    lateral = -ex * math.sin(math.pi / 2.0) + ey * math.cos(math.pi / 2.0)
    assert lateral == pytest.approx(0.0, abs=1e-12)
