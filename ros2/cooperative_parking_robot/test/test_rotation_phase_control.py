import math
from pathlib import Path

import pytest
import yaml

from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics
from cooperative_parking_robot.rotation_phase_control import (
    PhaseAwareRigidBodyKinematics,
    RotationPhaseController,
    is_final_rotation_command,
    rotation_phase_error,
)


def test_final_rotation_gate_keeps_rotate_then_insert_separate():
    assert is_final_rotation_command(
        mode='FINAL_APPROACH', final_phase='ALIGN_SLOT_YAW',
        command=(0.0, 0.0, 0.08))
    assert not is_final_rotation_command(
        mode='FINAL_APPROACH', final_phase='INSERT_ALONG_SLOT_AXIS',
        command=(0.03, 0.0, 0.0))
    assert not is_final_rotation_command(
        mode='PATH_TRACKING', final_phase='ALIGN_SLOT_YAW',
        command=(0.0, 0.0, 0.08))


def test_phase_error_has_lateral_sign_and_bounded_geometry():
    positive = rotation_phase_error(0.785, 0.0, 0.04)
    negative = rotation_phase_error(0.785, 0.0, -0.04)
    shortened = rotation_phase_error(0.785, -10.0, 0.04)

    assert positive > 0.0
    assert negative == pytest.approx(-positive)
    assert math.isfinite(shortened)
    assert abs(shortened) < math.pi / 2.0


def test_rotation_suppresses_opposing_lateral_pid_and_adds_common_yaw():
    kinematics = PhaseAwareRigidBodyKinematics(0.785)
    front_base, rear_base = kinematics.split(0.0, 0.0, 0.12)
    kinematics.configure_rotation_phase(
        active=True, common_yaw_provider=lambda: 0.03)

    front, rear = kinematics.apply_relative_correction(
        front_base, rear_base,
        corr_x=0.02, corr_y=0.04, corr_yaw=0.06)

    # The intended +omega*L/2 / -omega*L/2 orbit is not opposed by corr_y.
    assert front[1] == pytest.approx(front_base[1])
    assert rear[1] == pytest.approx(rear_base[1])
    # Gap and relative-yaw loops retain their existing symmetric authority.
    assert front[0] == pytest.approx(-0.01)
    assert rear[0] == pytest.approx(+0.01)
    assert front[2] == pytest.approx(0.12 + 0.03 - 0.03)
    assert rear[2] == pytest.approx(0.12 + 0.03 + 0.03)
    assert (
        kinematics.last_suppressed_lateral_correction_mps ==
        pytest.approx(0.04))


def test_inactive_phase_kinematics_is_exactly_legacy_behavior():
    legacy = RigidBodyKinematics(0.785)
    phase_aware = PhaseAwareRigidBodyKinematics(0.785)
    front_base, rear_base = legacy.split(0.02, -0.01, 0.05)

    expected = legacy.apply_relative_correction(
        front_base, rear_base, 0.01, -0.02, 0.03)
    actual = phase_aware.apply_relative_correction(
        front_base, rear_base, 0.01, -0.02, 0.03)

    assert actual[0] == pytest.approx(expected[0])
    assert actual[1] == pytest.approx(expected[1])


def test_phase_controller_filters_new_id0_samples_and_slew_limits():
    controller = RotationPhaseController(
        kp=1.8,
        deadband_rad=math.radians(0.5),
        correction_limit_rps=0.06,
        correction_rate_limit_rps2=0.30,
        lateral_lpf_alpha=0.65)

    first = controller.update(
        active=True, measurement_fresh=True, sample_token=1,
        separation_m=0.785, gap_error_m=0.0,
        lateral_error_m=0.04, now_s=10.0)
    repeated = controller.update(
        active=True, measurement_fresh=True, sample_token=1,
        separation_m=0.785, gap_error_m=0.0,
        lateral_error_m=-0.04, now_s=10.02)
    second_sample = controller.update(
        active=True, measurement_fresh=True, sample_token=2,
        separation_m=0.785, gap_error_m=0.0,
        lateral_error_m=-0.04, now_s=10.12)

    assert 0.0 < first.correction_rps <= 0.006 + 1.0e-12
    # The same camera stamp must not be low-pass filtered more than once.
    assert repeated.filtered_lateral_error_m == pytest.approx(0.04)
    assert second_sample.filtered_lateral_error_m == pytest.approx(
        0.65 * -0.04 + 0.35 * 0.04)
    assert abs(second_sample.correction_rps) <= 0.06


def test_stale_id0_slews_phase_correction_back_toward_zero():
    controller = RotationPhaseController(
        kp=5.0, deadband_rad=0.001,
        correction_limit_rps=0.06,
        correction_rate_limit_rps2=0.30,
        lateral_lpf_alpha=1.0)
    live = controller.update(
        active=True, measurement_fresh=True, sample_token=1,
        separation_m=0.785, gap_error_m=0.0,
        lateral_error_m=0.10, now_s=1.0)
    stale = controller.update(
        active=True, measurement_fresh=False, sample_token=None,
        separation_m=0.785, gap_error_m=0.0,
        lateral_error_m=0.10, now_s=1.1)

    assert live.correction_rps > 0.0
    assert 0.0 <= stale.correction_rps < live.correction_rps


def test_main_configuration_retains_rotate_then_insert_and_phase_parameters():
    root = Path(__file__).resolve().parents[3]
    config = yaml.safe_load((
        root / 'ros2/cooperative_parking_robot/config/sync_params.yaml'
    ).read_text(encoding='utf-8'))
    params = config['rigid_body_sync_node']['ros__parameters']

    assert params['align_to_slot_yaw'] is True
    assert params['rotation_phase_kp'] > 0.0
    assert params['rotation_phase_correction_limit_rps'] > 0.0


def test_production_entrypoint_selects_phase_rotation_node():
    root = Path(__file__).resolve().parents[3]
    setup_text = (
        root / 'ros2/cooperative_parking_robot/setup.py'
    ).read_text(encoding='utf-8')

    assert (
        'cooperative_parking_robot.rigid_body_sync_phase_node:main'
        in setup_text)
