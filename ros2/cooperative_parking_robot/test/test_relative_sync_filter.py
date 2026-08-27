#!/usr/bin/env python3
"""Regression tests for the rigid-body relative estimator helpers."""

import math
from pathlib import Path

import pytest

from cooperative_parking_robot.relative_sync_filter import (
    CctvPairStampGate,
    DeltaKalman1D,
    MissionReferenceCapture,
    OncePerStamp,
    RelativeObservationGate,
    ScalarObservationGate,
    anchored_pose,
    cctv_fallback_allowed,
    normalize_angle,
    reference_blocks_drive,
    stream_is_healthy,
    visual_safety_state,
)


def test_once_per_stamp_consumes_one_measurement_once():
    consumer = OncePerStamp()
    assert consumer.consume(100)
    assert not consumer.consume(100)
    assert not consumer.consume(99)
    assert consumer.consume(101)


def test_same_visual_stamp_updates_filter_exactly_once():
    consumer = OncePerStamp()
    filt = DeltaKalman1D(init=0.70, measurement_variance=0.01 ** 2)
    filt.reset(0.70, raw_value=0.70, covariance=0.01 ** 2)
    updates = 0
    for _ in range(5):
        if consumer.consume(123):
            filt.update(0.71)
            updates += 1
    assert updates == 1


def _scalar_gate(*, angle=False):
    return ScalarObservationGate(
        innovation_limit=(math.radians(5.0) if angle else 0.04),
        sigma_limit=4.0,
        reacquire_count=3,
        reacquire_limit=(math.radians(15.0) if angle else 0.12),
        consistency_limit=(math.radians(1.0) if angle else 0.01),
        angle=angle)


def test_distance_acceptance_is_independent_of_bad_yaw():
    distance = DeltaKalman1D(init=0.70, measurement_variance=0.01 ** 2)
    yaw = DeltaKalman1D(
        init=0.0, measurement_variance=math.radians(2.0) ** 2,
        angle=True)
    distance.reset(0.70, raw_value=0.70, covariance=0.01 ** 2)
    yaw.reset(0.0, raw_value=0.0, covariance=math.radians(2.0) ** 2)
    distance_decision = _scalar_gate().evaluate(0.705, distance)
    yaw_decision = _scalar_gate(angle=True).evaluate(math.radians(30.0), yaw)
    if distance_decision.accepted:
        distance.update(0.705)
    if yaw_decision.accepted:
        yaw.update(math.radians(30.0))
    assert distance_decision.action == 'ACCEPT'
    assert yaw_decision.action == 'REJECT'
    assert distance.x > 0.70
    assert yaw.x == pytest.approx(0.0)


def test_yaw_acceptance_is_independent_of_bad_distance():
    distance = DeltaKalman1D(init=0.70, measurement_variance=0.01 ** 2)
    yaw = DeltaKalman1D(
        init=0.0, measurement_variance=math.radians(2.0) ** 2,
        angle=True)
    distance.reset(0.70, raw_value=0.70, covariance=0.01 ** 2)
    yaw.reset(0.0, raw_value=0.0, covariance=math.radians(2.0) ** 2)
    distance_decision = _scalar_gate().evaluate(0.90, distance)
    yaw_decision = _scalar_gate(angle=True).evaluate(math.radians(1.0), yaw)
    if yaw_decision.accepted:
        yaw.update(math.radians(1.0))
    assert distance_decision.action == 'REJECT'
    assert yaw_decision.action == 'ACCEPT'
    assert yaw.x > 0.0


def test_lateral_visual_update_corrects_wheel_prediction_drift():
    lateral = DeltaKalman1D(
        init=0.0, measurement_variance=0.015 ** 2,
        process_variance_rate=0.003 ** 2)
    lateral.reset(0.0, raw_value=0.0, stamp_s=1.0)
    assert lateral.predict_from_raw(0.02, stamp_s=1.1)
    before = lateral.x
    decision = _scalar_gate().evaluate(0.005, lateral)
    assert decision.action == 'ACCEPT'
    lateral.update(0.005)
    assert abs(lateral.x - 0.005) < abs(before - 0.005)


def test_visible_marker_with_rejected_yaw_is_correction_degraded_not_lost():
    state, age, axes = visual_safety_state(
        now=12.0, marker_lost_since=None,
        correction_times={'distance': 11.9, 'lateral': 11.9, 'yaw': 10.5},
        slowdown_s=1.0, stop_s=2.0, correction_grace_s=0.5)
    assert state == 'CORRECTION_STALE'
    assert age == pytest.approx(1.0)
    assert axes == ('yaw',)


def test_actual_visual_loss_still_progresses_slowdown_then_hold():
    slow = visual_safety_state(
        now=11.5, marker_lost_since=10.0, correction_times={},
        slowdown_s=1.0, stop_s=2.0)
    hold = visual_safety_state(
        now=12.1, marker_lost_since=10.0, correction_times={},
        slowdown_s=1.0, stop_s=2.0)
    assert slow[0] == 'MARKER_SLOW'
    assert hold[0] == 'MARKER_HOLD'


def _reference_capture(sample_count=5, **overrides):
    values = dict(
        sample_count=sample_count, timeout_s=3.0,
        nominal_x=0.785, nominal_y=0.0, nominal_yaw=0.0,
        max_x_error=0.06, max_y_error=0.04,
        max_yaw_error=math.radians(5.0),
        max_std_x=0.01, max_std_y=0.01,
        max_std_yaw=math.radians(2.0))
    values.update(overrides)
    capture = MissionReferenceCapture(**values)
    capture.reset(start_time=10.0)
    return capture


def test_reference_capture_locks_medians_from_stable_id0_samples():
    capture = _reference_capture()
    samples = [
        (0.791, -0.005, math.radians(0.7)),
        (0.792, -0.006, math.radians(0.8)),
        (0.793, -0.007, math.radians(0.9)),
        (0.792, -0.006, math.radians(0.8)),
        (0.794, -0.004, math.radians(0.6)),
    ]
    for sample in samples:
        capture.add(*sample)
    assert capture.ready
    assert capture.reference.relative_x == pytest.approx(0.792)
    assert capture.reference.relative_y == pytest.approx(-0.006)
    assert capture.reference.relative_yaw == pytest.approx(math.radians(0.8))


def test_reference_is_frozen_until_mission_reset():
    capture = _reference_capture(sample_count=3)
    for x in (0.791, 0.792, 0.793):
        capture.add(x, -0.006, math.radians(0.8))
    locked = capture.reference
    for x in (0.800, 0.810):
        assert not capture.add(x, -0.006, math.radians(0.8))
    assert capture.reference == locked
    capture.reset()
    assert not capture.ready
    assert capture.reference is None
    assert capture.state == 'WAIT_LIFT'


@pytest.mark.parametrize('samples,reason', [
    ([(0.850, 0.0, 0.0)] * 5, 'nominal_sanity_envelope'),
    ([(0.785 + offset, 0.0, 0.0)
      for offset in (-0.02, 0.02, -0.02, 0.02, 0.0)],
     'sample_dispersion'),
])
def test_bad_reference_capture_is_rejected(samples, reason):
    capture = _reference_capture()
    for sample in samples:
        capture.add(*sample)
    assert not capture.ready
    assert capture.state == 'REFERENCE_RETRY_WAIT'
    assert capture.reason == reason


def test_reference_capture_timeout_does_not_nominally_fallback():
    capture = _reference_capture()
    capture.add(0.792, -0.006, 0.0)
    assert capture.timed_out(13.1)
    assert capture.state == 'REFERENCE_RETRY_WAIT'
    assert capture.reference is None


def test_reference_capture_retries_are_bounded_then_fatal():
    capture = _reference_capture(
        sample_count=3, max_retries=2, retry_delay_s=0.1)
    now = 10.0
    for attempt in range(3):
        for _ in range(3):
            capture.add(0.90, 0.0, 0.0, now=now)
        if attempt < 2:
            assert capture.state == 'REFERENCE_RETRY_WAIT'
            now += 0.11
            assert capture.advance(now) == 'REFERENCE_CAPTURE'
    assert capture.state == 'REFERENCE_FAILED'
    assert capture.retry_count == 3


def test_reference_capture_can_disable_uncalibrated_x():
    capture = _reference_capture(sample_count=3)
    for _ in range(3):
        capture.add(None, -0.006, math.radians(0.8))
    assert capture.ready
    assert capture.reference.relative_x is None
    assert capture.reference.relative_y == pytest.approx(-0.006)


def test_drive_is_blocked_after_lift_until_reference_is_ready():
    assert reference_blocks_drive(True, 'DRIVE', 'DRIVE', False)
    assert not reference_blocks_drive(True, 'DRIVE', 'DRIVE', True)


def test_id0_health_uses_age_not_new_frame_in_current_control_cycle():
    assert stream_is_healthy(0.20, 0.30)
    assert not stream_is_healthy(0.31, 0.30)
    assert not stream_is_healthy(None, 0.30)
    assert not cctv_fallback_allowed(0.20, 0.30)
    assert cctv_fallback_allowed(0.31, 0.30)


def test_cctv_fallback_pair_requires_sync_and_is_consumed_once():
    gate = CctvPairStampGate(sync_slop_s=0.12)
    assert not gate.accept(1_000_000_000, 1_200_000_001)
    assert gate.accept(2_000_000_000, 2_100_000_000)
    assert not gate.accept(2_000_000_000, 2_100_000_000)
    assert not gate.accept(3_000_000_000, 2_100_000_000)


def test_cached_predict_does_not_grow_covariance():
    filt = DeltaKalman1D(
        init=0.70,
        measurement_variance=0.02 ** 2,
        process_variance_rate=0.003 ** 2)
    filt.reset(0.70, raw_value=0.70, stamp_s=1.0)
    p0 = filt.P
    assert not filt.predict_from_raw(0.70, stamp_s=1.0)
    assert filt.P == pytest.approx(p0)
    assert filt.predict_from_raw(0.69, stamp_s=1.02)
    assert filt.x == pytest.approx(0.69)
    assert filt.P > p0


def test_angle_filter_uses_shortest_wrap_delta_and_innovation():
    filt = DeltaKalman1D(
        init=math.radians(179.0),
        measurement_variance=math.radians(3.0) ** 2,
        process_variance_rate=math.radians(0.5) ** 2,
        angle=True)
    filt.reset(
        math.radians(179.0), raw_value=math.radians(179.0), stamp_s=1.0)
    assert filt.predict_from_raw(math.radians(-179.0), stamp_s=1.02)
    assert normalize_angle(filt.x - math.radians(-179.0)) == pytest.approx(0.0)
    assert abs(filt.innovation(math.radians(179.5))) < math.radians(2.0)


def _gate(reacquire_count=3):
    return RelativeObservationGate(
        distance_limit=0.04,
        yaw_limit=math.radians(5.0),
        sigma_limit=4.0,
        reacquire_count=reacquire_count,
        reacquire_distance_limit=0.12,
        reacquire_yaw_limit=math.radians(15.0),
        consistency_distance=0.01,
        consistency_yaw=math.radians(1.0),
    )


def test_single_outlier_is_rejected():
    dist = DeltaKalman1D(init=0.70, measurement_variance=0.01 ** 2)
    yaw = DeltaKalman1D(
        init=0.0,
        measurement_variance=math.radians(2.0) ** 2,
        angle=True)
    dist.reset(0.70, raw_value=0.70, covariance=0.01 ** 2)
    yaw.reset(0.0, raw_value=0.0, covariance=math.radians(2.0) ** 2)
    decision = _gate().evaluate(
        distance_measurement=0.90,
        yaw_measurement=0.0,
        distance_filter=dist,
        yaw_filter=yaw)
    assert decision.action == 'REJECT'
    assert decision.reason == 'outside_reacquire_envelope'


def test_consistent_bounded_measurements_reacquire_once():
    dist = DeltaKalman1D(init=0.70, measurement_variance=0.01 ** 2)
    yaw = DeltaKalman1D(
        init=0.0,
        measurement_variance=math.radians(2.0) ** 2,
        angle=True)
    dist.reset(0.70, raw_value=0.70, covariance=0.005 ** 2)
    yaw.reset(0.0, raw_value=0.0, covariance=math.radians(1.0) ** 2)
    gate = _gate(reacquire_count=3)
    actions = [
        gate.evaluate(
            distance_measurement=value,
            yaw_measurement=math.radians(7.0),
            distance_filter=dist,
            yaw_filter=yaw).action
        for value in (0.765, 0.768, 0.766)
    ]
    assert actions == ['REJECT', 'REJECT', 'REACQUIRE']


def test_anchored_pose_preserves_world_anchor_and_relative_motion():
    result = anchored_pose(
        anchor_world=(2.0, 3.0, math.pi / 2.0),
        anchor_local=(10.0, -4.0, 0.0),
        current_local=(11.0, -4.0, math.pi / 2.0))
    assert result[0] == pytest.approx(2.0)
    assert result[1] == pytest.approx(4.0)
    assert result[2] == pytest.approx(math.pi)


def test_production_entrypoint_and_parameters_are_wired():
    root = Path(__file__).resolve().parents[1]
    setup_text = (root / 'setup.py').read_text()
    config_text = (root / 'config/sync_params.yaml').read_text()
    assert ('cooperative_parking_robot.rigid_body_sync_safe_node:main'
            in setup_text)
    for name in (
            'use_raw_wheel_relative_predictor',
            'sync_dist_measurement_sigma_m',
            'sync_lateral_measurement_sigma_m',
            'aruco_distance_innovation_gate_m',
            'aruco_lateral_innovation_gate_m',
            'aruco_reacquire_count',
            'sync_dist_deadband_m',
            'sync_lateral_deadband_m',
            'sync_target_lateral_m',
            'sync_target_yaw_deg',
            'sync_reference_capture_samples',
            'sync_reference_capture_timeout_s'):
        assert name in config_text


def test_production_predictor_keeps_raw_wheel_priority_and_fused_fallback():
    root = Path(__file__).resolve().parents[1]
    source = (root / 'cooperative_parking_robot' /
              'rigid_body_sync_safe_node.py').read_text()
    predictor = source.split('def _relative_predictor', 1)[1].split(
        'def _initialize_sync_filters', 1)[0]
    assert predictor.index('self._raw_wheel_relative(now)') < predictor.index(
        "'FUSED_ODOM_FALLBACK'")
    assert "'RAW_WHEEL_ODOM'" in source
    assert 'raw_lateral' in source
