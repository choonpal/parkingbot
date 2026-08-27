#!/usr/bin/env python3
"""Production safe-sync sensor priority regressions."""

import time

from cooperative_parking_robot.relative_sync_filter import (
    MissionReferenceCapture,
    OncePerStamp,
)
from cooperative_parking_robot.rigid_body_sync_safe_node import (
    RigidBodySyncNode,
)
from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics


def _sensor_priority_node(*, id0_age, stamp=100):
    node = object.__new__(RigidBodySyncNode)
    node.aruco_receipt_time = 10.0 - id0_age
    node.aruco_timeout = 0.30
    node.aruco_yaw = 0.0
    node.aruco_stamp_ns = stamp
    node._aruco_consumer = OncePerStamp()
    node._aruco_consumer.consume(stamp)
    node._relative_correction_source = 'UNSET'
    node._cctv_pair_gates = {}
    node.cctv_calls = 0

    def new_cctv_pair(_now):
        node.cctv_calls += 1
        return (0.785, 0.0, 0.0)

    node._new_cctv_pair = new_cctv_pair
    node._apply_visual_measurement = lambda **_kwargs: 'CCTV_ACCEPT'
    return node


def test_healthy_id0_blocks_cctv_when_current_cycle_has_no_new_id0():
    node = _sensor_priority_node(id0_age=0.20)
    correction = node._consume_visual_measurement(
        10.0, 0.785, 0.0, 0.0, 10.0)
    assert correction is None
    assert node.cctv_calls == 0
    assert node._relative_correction_source == 'ID0_WHEEL'


def test_actually_stale_id0_allows_one_qualified_cctv_fallback():
    node = _sensor_priority_node(id0_age=0.31)
    correction = node._consume_visual_measurement(
        10.0, 0.785, 0.0, 0.0, 10.0)
    assert correction == 'CCTV_ACCEPT'
    assert node.cctv_calls == 1
    assert node._relative_correction_source == 'CCTV_FALLBACK'


def test_fused_fallback_predictor_returns_relative_x_not_euclidean_norm():
    node = object.__new__(RigidBodySyncNode)
    node._raw_wheel_relative = lambda _now: None
    node.kinematics = RigidBodyKinematics(0.785)
    node.front = {'x': 0.785, 'y': 0.030, 'theta': 0.0, 't': 10.0}
    node.rear = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 't': 10.0}
    relative_x, yaw, _, source, relative_y = node._relative_predictor(10.0)
    assert relative_x == 0.785
    assert relative_y == 0.030
    assert yaw == 0.0
    assert source == 'FUSED_ODOM_FALLBACK'


def test_production_control_loop_outputs_stop_before_reference_ready():
    node = object.__new__(RigidBodySyncNode)
    node.vehicle_lifted = True
    node.front_robot_state = 'DRIVE'
    node.rear_robot_state = 'DRIVE'
    node.reference_capture = MissionReferenceCapture(
        sample_count=5, timeout_s=3.0,
        nominal_x=0.785, nominal_y=0.0, nominal_yaw=0.0,
        max_x_error=0.06, max_y_error=0.04, max_yaw_error=0.1,
        max_std_x=0.01, max_std_y=0.01, max_std_yaw=0.03)
    node.reference_capture.reset(start_time=time.monotonic())
    node.stop_calls = 0
    node.send_stop = lambda: setattr(node, 'stop_calls', node.stop_calls + 1)
    node._reference_telemetry = lambda _now: {'reference_state': 'CAPTURE'}
    node.control_loop()
    assert node.stop_calls == 1
    assert node._err == 'REFERENCE_CAPTURE'
