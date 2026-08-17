"""주차면 치수/방향과 단계형 메카넘 주차 기하 회귀 테스트."""

import math

import pytest

from cooperative_parking_robot.parking_geometry import (
    ParkingSlot,
    Pose2D,
    RegisteredSlot,
    build_slot,
    check_rotation_sweep_fit,
    check_slot_fit,
    choose_target_yaw,
    footprint_extents_in_slot_axes,
    make_approach_candidates,
    nearest_axis_yaw,
    parse_registered_slots,
    plan_mecanum_parking,
    polygon_overlap_ratio,
    slot_fits,
    slot_from_corners,
    slot_polygon,
)


def test_clicked_corners_create_metre_slot_directed_from_aisle():
    # 입력 순서가 섞여 있어도 통로(-x)에서 슬롯(+x) 안쪽으로 yaw가 정해져야 한다.
    slot = slot_from_corners(
        "P1",
        corners=[(5.0, 3.0), (1.0, 1.0), (5.0, 1.0), (1.0, 3.0)],
        aisle_point=(0.0, 2.0),
    )

    assert slot.center == pytest.approx((3.0, 2.0))
    assert slot.length_m == pytest.approx(4.0)
    assert slot.width_m == pytest.approx(2.0)
    assert slot.entry_yaw_rad == pytest.approx(0.0, abs=1e-12)


def test_slot_fit_uses_each_side_margin_and_reports_limiting_axis():
    slot = ParkingSlot("P1", 0.0, 0.0, 5.0, 2.5, 0.0)

    good = check_slot_fit(
        slot, 4.2, 1.8,
        longitudinal_margin_m=0.20,
        lateral_margin_m=0.15,
    )
    narrow = check_slot_fit(
        slot, 4.2, 2.3,
        longitudinal_margin_m=0.20,
        lateral_margin_m=0.15,
    )

    assert good.fits
    assert good.required_length_m == pytest.approx(4.6)
    assert good.required_width_m == pytest.approx(2.1)
    assert good.length_clearance_m == pytest.approx(0.4)
    assert good.width_clearance_m == pytest.approx(0.4)
    assert not narrow.fits
    assert narrow.reason == "SLOT_TOO_NARROW"


def test_build_and_parse_flat_yaml_slot_geometry():
    direct = build_slot(
        "P1", center=(3.0, 2.0), size=(5.0, 2.4), yaw_deg=90.0)
    parsed = parse_registered_slots(
        ["P1", "P2"],
        [3.0, 2.0, 6.0, 2.0],
        [5.0, 2.4, 5.5, 2.6],
        [90.0, -90.0],
    )

    assert isinstance(direct, RegisteredSlot)
    assert isinstance(direct, ParkingSlot)
    assert parsed[0] == direct
    assert parsed[1].slot_id == "P2"
    assert parsed[1].length_m == pytest.approx(5.5)
    assert parsed[1].entry_yaw_rad == pytest.approx(-math.pi / 2)


def test_polygon_overlap_is_intersection_over_slot_area():
    slot = ParkingSlot("P1", 0.0, 0.0, 4.0, 2.0, 0.0)
    # 차량 사각형이 8m^2 슬롯의 오른쪽 절반 4m^2를 차지한다.
    subject = [(0.0, -2.0), (3.0, -2.0), (3.0, 2.0), (0.0, 2.0)]

    ratio = polygon_overlap_ratio(subject, slot_polygon(slot))
    reversed_ratio = polygon_overlap_ratio(
        subject, list(reversed(slot_polygon(slot))))

    assert ratio == pytest.approx(0.5)
    assert reversed_ratio == pytest.approx(0.5)


def test_slot_fits_is_bool_wrapper_with_uniform_clearance():
    slot = ParkingSlot("P1", 0.0, 0.0, 5.0, 2.5, 0.0)

    assert slot_fits(4.0, 1.5, slot, clearance_m=0.25)
    assert not slot_fits(4.0, 2.1, slot, clearance_m=0.25)


def test_rotated_rectangle_projection_is_not_treated_as_a_point():
    length, width = footprint_extents_in_slot_axes(
        4.0, 2.0, math.radians(90.0))

    assert length == pytest.approx(2.0)
    assert width == pytest.approx(4.0)


def test_narrow_slot_accepts_final_pose_but_rejects_ninety_degree_sweep():
    slot = ParkingSlot("P1", 0.0, 0.0, 5.0, 2.5, 0.0)

    final_fit = check_slot_fit(slot, 4.0, 2.0)
    rotation_fit = check_rotation_sweep_fit(
        slot, 4.0, 2.0,
        start_yaw_rad=math.radians(90.0),
        target_yaw_rad=0.0,
    )

    assert final_fit.fits
    assert not rotation_fit.fits
    assert rotation_fit.required_width_m == pytest.approx(
        math.hypot(4.0, 2.0))


def test_minimum_rotation_selects_equivalent_reverse_orientation():
    slot = ParkingSlot("P1", 5.0, 2.0, 4.0, 2.0, 0.0)
    current_yaw = math.radians(170.0)

    selected = choose_target_yaw(slot, current_yaw, "minimum_rotation")

    assert abs(abs(selected) - math.pi) < 1e-12
    assert abs(math.degrees(selected - current_yaw)) == pytest.approx(10.0)
    assert nearest_axis_yaw(0.0, current_yaw) == pytest.approx(selected)


def test_approach_candidates_share_entry_and_are_sorted_by_yaw_change():
    slot = ParkingSlot("P1", 5.0, 2.0, 4.0, 2.0, 0.0)

    candidates = make_approach_candidates(
        slot, loaded_length_m=3.0, gap_m=0.25,
        current_yaw_rad=math.radians(170.0))

    assert [c.parking_direction for c in candidates] == ["reverse", "forward"]
    assert candidates[0].staging_pose.position == pytest.approx((1.25, 2.0))
    assert candidates[1].staging_pose.position == pytest.approx((1.25, 2.0))
    assert abs(math.degrees(candidates[0].yaw_change_rad)) == pytest.approx(10.0)
    assert abs(math.degrees(candidates[1].yaw_change_rad)) == pytest.approx(170.0)


def test_mecanum_plan_rotates_in_aisle_then_inserts_on_slot_axis():
    slot = ParkingSlot("P1", 5.0, 2.0, 4.0, 2.0, 0.0)
    current = Pose2D(0.0, 0.0, math.radians(90.0))

    plan = plan_mecanum_parking(
        slot,
        current,
        footprint_length_m=3.5,
        footprint_width_m=1.6,
        staging_gap_m=0.25,
        parking_direction="forward",
    )

    # 열린 경계 x=3.0에서 운반체 반길이 1.75 + gap 0.25만큼 더 뒤에 선다.
    assert plan.staging_before_rotation.position == pytest.approx((1.0, 2.0))
    assert plan.staging_before_rotation.yaw_rad == pytest.approx(math.pi / 2)
    assert plan.staging_aligned.position == pytest.approx((1.0, 2.0))
    assert plan.staging_aligned.yaw_rad == pytest.approx(0.0)
    assert plan.target_pose.position == pytest.approx((5.0, 2.0))
    assert plan.target_pose.yaw_rad == pytest.approx(0.0)
    assert plan.needs_rotation
    assert plan.insertion_segment == (
        plan.staging_aligned, plan.target_pose)


def test_mecanum_plan_rejects_slot_that_loaded_carrier_cannot_enter():
    slot = ParkingSlot("P1", 5.0, 2.0, 4.0, 1.5, 0.0)

    with pytest.raises(ValueError, match="SLOT_TOO_NARROW"):
        plan_mecanum_parking(
            slot,
            Pose2D(0.0, 0.0, 0.0),
            footprint_length_m=3.5,
            footprint_width_m=1.6,
        )


def test_forward_and_reverse_policies_keep_same_physical_entry_side():
    slot = ParkingSlot("P1", 5.0, 2.0, 4.0, 2.0, 0.0)
    current = Pose2D(0.0, 0.0, math.pi)

    forward = plan_mecanum_parking(
        slot, current, 3.0, 1.5, parking_direction="forward")
    reverse = plan_mecanum_parking(
        slot, current, 3.0, 1.5, parking_direction="reverse")

    # 차량 앞 방향만 180도 달라지고, 실제로 진입하는 열린 경계는 같아야 한다.
    assert forward.staging_aligned.position == reverse.staging_aligned.position
    assert abs(abs(forward.target_pose.yaw_rad - reverse.target_pose.yaw_rad)
               - math.pi) < 1e-12


def test_slot_corner_registration_rejects_ambiguous_aisle_side():
    with pytest.raises(ValueError, match="longitudinal entry side"):
        slot_from_corners(
            "P1",
            corners=[(1.0, 1.0), (5.0, 1.0), (5.0, 3.0), (1.0, 3.0)],
            aisle_point=(3.0, 0.0),
        )
