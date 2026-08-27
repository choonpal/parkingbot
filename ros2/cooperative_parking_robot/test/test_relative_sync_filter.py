#!/usr/bin/env python3
"""Regression tests for the rigid-body relative estimator helpers."""

import math
from pathlib import Path

import pytest

from cooperative_parking_robot.relative_sync_filter import (
    DeltaKalman1D,
    OncePerStamp,
    RelativeObservationGate,
    anchored_pose,
    normalize_angle,
)


def test_once_per_stamp_consumes_one_measurement_once():
    consumer = OncePerStamp()
    assert consumer.consume(100)
    assert not consumer.consume(100)
    assert not consumer.consume(99)
    assert consumer.consume(101)


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
            'aruco_distance_innovation_gate_m',
            'aruco_reacquire_count',
            'sync_dist_deadband_m'):
        assert name in config_text
