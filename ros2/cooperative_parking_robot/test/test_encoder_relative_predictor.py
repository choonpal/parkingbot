import math

import pytest

from cooperative_parking_robot.encoder_relative_predictor import (
    EncoderRelativePredictor,
)
from cooperative_parking_robot.mvp_recovery_policy import (
    final_slot_command,
    servo_attach_pulses_from_telemetry,
    stage_accepts_lift_status,
)


def test_encoder_predictor_propagates_front_motion_without_visual_wait():
    predictor = EncoderRelativePredictor(sync_slop_s=0.05)
    predictor.note_odom('front', (0.0, 0.0, 0.0), 1_000_000_000)
    predictor.note_odom('rear', (0.0, 0.0, 0.0), 1_010_000_000)
    assert predictor.note_visual((0.785, 0.0, 0.0))

    predictor.note_odom('front', (0.010, 0.0, 0.0), 1_020_000_000)
    predictor.note_odom('rear', (0.004, 0.0, 0.0), 1_021_000_000)
    x, y, yaw = predictor.predict()
    assert x == pytest.approx(0.791, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(0.0, abs=1e-6)


def test_encoder_predictor_uses_se2_for_rotation():
    predictor = EncoderRelativePredictor(sync_slop_s=0.05)
    predictor.note_odom('front', (0.0, 0.0, 0.0), 1_000_000_000)
    predictor.note_odom('rear', (0.0, 0.0, 0.0), 1_000_000_000)
    predictor.note_visual((1.0, 0.0, 0.0))
    predictor.note_odom('front', (0.0, 0.0, math.pi / 2), 2_000_000_000)
    predictor.note_odom('rear', (0.0, 0.0, 0.0), 2_000_000_000)
    x, y, yaw = predictor.predict()
    assert x == pytest.approx(1.0, abs=1e-6)
    assert y == pytest.approx(0.0, abs=1e-6)
    assert yaw == pytest.approx(math.pi / 2, abs=1e-6)


def test_encoder_predictor_holds_last_synchronized_pair():
    predictor = EncoderRelativePredictor(sync_slop_s=0.05)
    predictor.note_odom('front', (0.0, 0.0, 0.0), 1_000_000_000)
    predictor.note_odom('rear', (0.0, 0.0, 0.0), 1_000_000_000)
    predictor.note_visual((0.785, 0.0, 0.0))
    predictor.note_odom('front', (0.01, 0.0, 0.0), 1_010_000_000)
    predictor.note_odom('rear', (0.005, 0.0, 0.0), 1_010_000_000)
    synced = predictor.predict()
    predictor.note_odom('front', (0.03, 0.0, 0.0), 1_200_000_000)
    assert predictor.predict() == synced


def test_done_ack_is_phase_scoped():
    assert stage_accepts_lift_status('LIFT', 'GRIP_DONE')
    assert stage_accepts_lift_status('RELEASE', 'RELEASE_DONE')
    assert not stage_accepts_lift_status('IDLE', 'GRIP_DONE')
    assert not stage_accepts_lift_status('LIFT', 'RELEASE_DONE')


def test_live_servo_pulses_replace_open_attach_default():
    parsed = {'servo_us': [1710, 1320]}
    assert servo_attach_pulses_from_telemetry(
        parsed, (2600, 400)) == (1710, 1320)


def test_final_slot_rotation_and_insertion_are_separate():
    rotating = final_slot_command(
        base_command=(0.02, 0.01, 0.04),
        yaw_error=math.radians(10), yaw_tolerance=math.radians(3),
        yaw_kp=1.0, max_omega=0.30, max_speed=0.08,
        rotation_radius=0.3925, final_max_omega=0.15)
    assert rotating[0:2] == (0.0, 0.0)
    assert 0.0 < rotating[2] <= 0.15

    inserting = final_slot_command(
        base_command=(0.02, 0.01, 0.04),
        yaw_error=math.radians(1), yaw_tolerance=math.radians(3),
        yaw_kp=1.0, max_omega=0.30, max_speed=0.08,
        rotation_radius=0.3925, final_max_omega=0.15)
    assert inserting == pytest.approx((0.02, 0.01, 0.0))
