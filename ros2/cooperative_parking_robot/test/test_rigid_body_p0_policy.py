#!/usr/bin/env python3
"""P0 regression tests for production rigid-body lifecycle and safety."""

from pathlib import Path

import pytest

from cooperative_parking_robot.pid_controller import PID
from cooperative_parking_robot.rigid_body_p0_policy import (
    lateral_safety_state,
    wheel_pair_is_synchronized,
    wheel_pair_skew_s,
)


ROOT = Path(__file__).resolve().parents[1]


def test_wheel_pair_gate_rejects_asynchronous_latest_samples():
    assert wheel_pair_is_synchronized(
        1_000_000_000, 1_040_000_000, 0.05)
    assert not wheel_pair_is_synchronized(
        1_000_000_000, 1_060_000_000, 0.05)
    assert wheel_pair_skew_s(
        1_000_000_000, 1_060_000_000) == pytest.approx(0.06)


def test_lateral_safety_slow_then_timeout_stop():
    first = lateral_safety_state(
        error_m=0.025, now=10.0, error_since=None,
        error_limit_m=0.020, stop_limit_m=0.040,
        error_timeout_s=1.0)
    assert first.action == 'SLOW'
    assert first.speed_scale == pytest.approx(0.30)
    timed_out = lateral_safety_state(
        error_m=0.025, now=11.1, error_since=first.error_since,
        error_limit_m=0.020, stop_limit_m=0.040,
        error_timeout_s=1.0)
    assert timed_out.action == 'FATAL_TIMEOUT'
    assert timed_out.blocking


def test_lateral_stop_limit_is_immediate():
    decision = lateral_safety_state(
        error_m=-0.040, now=10.0, error_since=None,
        error_limit_m=0.020, stop_limit_m=0.040,
        error_timeout_s=1.0)
    assert decision.action == 'FATAL_LIMIT'
    assert decision.blocking


def test_lateral_recovery_clears_persistence_timer():
    decision = lateral_safety_state(
        error_m=0.010, now=20.0, error_since=10.0,
        error_limit_m=0.020, stop_limit_m=0.040,
        error_timeout_s=1.0)
    assert decision.action == 'OK'
    assert decision.error_since is None


def test_zero_deadband_error_resets_pid_integral():
    pid = PID(1.0, 1.0, 0.0, out_limit=1.0)
    assert pid.compute(0.1, 0.1) > 0.0
    assert pid.integral > 0.0
    assert pid.compute(0.0, 0.1) == 0.0
    assert pid.integral == 0.0
    assert pid.prev == 0.0


def test_production_path_callback_preserves_lift_reference():
    source = (ROOT / 'cooperative_parking_robot' /
              'rigid_body_sync_production_node.py').read_text()
    callback = source.split('def path_cb', 1)[1].split(
        'def vehicle_lifted_cb', 1)[0]
    assert 'LegacyRigidBodySyncNode.path_cb(self, msg)' in callback
    assert 'reference_capture.reset' not in callback


def test_production_entrypoint_and_conservative_config_are_wired():
    setup_text = (ROOT / 'setup.py').read_text()
    config_text = (ROOT / 'config/sync_params.yaml').read_text()
    assert ('cooperative_parking_robot.rigid_body_sync_production_node:main'
            in setup_text)
    expected = (
        'wheel_pair_sync_slop_s: 0.05',
        'sync_lateral_error_limit_m: 0.020',
        'sync_lateral_stop_limit_m: 0.040',
        'sync_reference_max_x_error_m: 0.025',
        'sync_reference_max_lateral_error_m: 0.020',
        'sync_reference_max_yaw_error_deg: 3.0',
        'sync_lateral_kp: 0.4',
        'sync_lateral_ki: 0.0',
        'sync_lateral_kd: 0.0',
        'sync_lateral_max_correction_mps: 0.015',
    )
    for value in expected:
        assert value in config_text
