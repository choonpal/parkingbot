#!/usr/bin/env python3
"""Production entry point for robust Front/Rear relative-state fusion.

This module keeps the existing path/final-approach controller intact while
replacing the relative synchronization layer with these guarantees:

* each ID0 frame is corrected exactly once;
* wheel-only odometry, not CCTV-corrected ``/odom``, drives relative predict;
* direct overhead-marker poses are paired and consumed once as fallback;
* distance/yaw innovation gates reject one-frame solvePnP glitches;
* repeated bounded observations can deliberately re-acquire a drifted filter;
* angle prediction and correction are wrap-aware;
* process covariance grows on new odometry, not on every 50 Hz control tick.
"""

from __future__ import annotations

import math
import statistics
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry

from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.latest_qos import SENSOR_LATEST_QOS
from cooperative_parking_robot.relative_sync_filter import (
    CctvPairStampGate,
    DeltaKalman1D,
    MissionReferenceCapture,
    OncePerStamp,
    ScalarObservationGate,
    anchored_pose,
    cctv_fallback_allowed,
    normalize_angle,
    reference_blocks_drive,
    stream_is_healthy,
    visual_safety_state,
)
from cooperative_parking_robot.rigid_body_sync_node import (
    RigidBodySyncNode as LegacyRigidBodySyncNode,
)
from cooperative_parking_robot.pid_controller import PID


class RigidBodySyncNode(LegacyRigidBodySyncNode):
    """Legacy motion controller with a corrected relative estimator."""

    def __init__(self):
        super().__init__()

        # Estimator noise is intentionally exposed. Defaults are conservative
        # placeholders; replace them with static-camera and wheel-slip logs.
        self.declare_parameter('sync_dist_process_sigma_m_sqrt_s', 0.003)
        self.declare_parameter('sync_yaw_process_sigma_deg_sqrt_s', 0.50)
        self.declare_parameter('sync_lateral_process_sigma_m_sqrt_s', 0.003)
        self.declare_parameter('sync_dist_process_gain', 0.03)
        self.declare_parameter('sync_yaw_process_gain', 0.03)
        self.declare_parameter('sync_lateral_process_gain', 0.03)
        self.declare_parameter('sync_dist_measurement_sigma_m', 0.015)
        self.declare_parameter('sync_yaw_measurement_sigma_deg', 3.0)
        self.declare_parameter('sync_lateral_measurement_sigma_m', 0.015)
        self.declare_parameter('sync_initial_covariance_scale', 4.0)

        self.declare_parameter('aruco_distance_innovation_gate_m', 0.04)
        self.declare_parameter('aruco_yaw_innovation_gate_deg', 5.0)
        self.declare_parameter('aruco_innovation_sigma_gate', 4.0)
        self.declare_parameter('aruco_reacquire_count', 4)
        self.declare_parameter('aruco_reacquire_distance_m', 0.12)
        self.declare_parameter('aruco_reacquire_yaw_deg', 15.0)
        self.declare_parameter('aruco_consistency_distance_m', 0.015)
        self.declare_parameter('aruco_consistency_yaw_deg', 2.0)
        self.declare_parameter('aruco_lateral_envelope_m', 0.10)
        self.declare_parameter('aruco_lateral_innovation_gate_m', 0.04)
        self.declare_parameter('aruco_reacquire_lateral_m', 0.10)
        self.declare_parameter('aruco_consistency_lateral_m', 0.015)

        self.declare_parameter('sync_dist_deadband_m', 0.003)
        self.declare_parameter('sync_yaw_deadband_deg', 0.50)
        self.declare_parameter('sync_lateral_deadband_m', 0.003)
        self.declare_parameter('sync_target_lateral_m', 0.0)
        self.declare_parameter('sync_target_yaw_deg', 0.0)
        self.declare_parameter('sync_lateral_kp', 1.2)
        self.declare_parameter('sync_lateral_ki', 0.1)
        self.declare_parameter('sync_lateral_kd', 0.05)
        self.declare_parameter('sync_lateral_max_correction_mps', 0.08)
        self.declare_parameter('use_raw_wheel_relative_predictor', True)
        self.declare_parameter('wheel_relative_timeout_s', 0.50)
        self.declare_parameter('cctv_pair_timeout_s', 0.50)
        self.declare_parameter('cctv_pair_sync_slop_s', 0.12)
        self.declare_parameter('sync_reference_capture_samples', 20)
        self.declare_parameter('sync_reference_capture_timeout_s', 5.0)
        self.declare_parameter('sync_reference_settle_time_s', 0.5)
        self.declare_parameter('sync_reference_max_x_error_m', 0.06)
        self.declare_parameter('sync_reference_max_lateral_error_m', 0.04)
        self.declare_parameter('sync_reference_max_yaw_error_deg', 5.0)
        self.declare_parameter('sync_reference_max_sample_std_x_m', 0.01)
        self.declare_parameter('sync_reference_max_sample_std_y_m', 0.01)
        self.declare_parameter(
            'sync_reference_max_sample_std_yaw_deg', 2.0)
        self.declare_parameter('sync_reference_max_retries', 2)
        self.declare_parameter('sync_reference_retry_delay_s', 0.3)

        gp = self.get_parameter
        dist_process_sigma = float(
            gp('sync_dist_process_sigma_m_sqrt_s').value)
        yaw_process_sigma = math.radians(float(
            gp('sync_yaw_process_sigma_deg_sqrt_s').value))
        lateral_process_sigma = float(
            gp('sync_lateral_process_sigma_m_sqrt_s').value)
        dist_measurement_sigma = float(
            gp('sync_dist_measurement_sigma_m').value)
        yaw_measurement_sigma = math.radians(float(
            gp('sync_yaw_measurement_sigma_deg').value))
        lateral_measurement_sigma = float(
            gp('sync_lateral_measurement_sigma_m').value)
        self.initial_covariance_scale = float(
            gp('sync_initial_covariance_scale').value)
        self.dist_deadband = float(gp('sync_dist_deadband_m').value)
        self.yaw_deadband = math.radians(float(
            gp('sync_yaw_deadband_deg').value))
        self.lateral_deadband = float(
            gp('sync_lateral_deadband_m').value)
        self.target_lateral = float(gp('sync_target_lateral_m').value)
        self.target_relative_yaw = math.radians(float(
            gp('sync_target_yaw_deg').value))
        self.use_raw_wheel_predictor = bool(
            gp('use_raw_wheel_relative_predictor').value)
        self.wheel_relative_timeout = float(
            gp('wheel_relative_timeout_s').value)
        self.cctv_pair_timeout = float(
            gp('cctv_pair_timeout_s').value)
        self.cctv_pair_sync_slop = float(
            gp('cctv_pair_sync_slop_s').value)
        self.aruco_lateral_envelope = float(
            gp('aruco_lateral_envelope_m').value)

        positive = (
            dist_process_sigma, yaw_process_sigma,
            lateral_process_sigma, dist_measurement_sigma,
            yaw_measurement_sigma, lateral_measurement_sigma,
            self.initial_covariance_scale, self.dist_deadband,
            self.yaw_deadband, self.lateral_deadband,
            self.wheel_relative_timeout,
            self.cctv_pair_timeout, self.cctv_pair_sync_slop,
            self.aruco_lateral_envelope)
        if any(value <= 0.0 for value in positive):
            raise ValueError('relative estimator parameters must be positive')

        self.relative_x_kalman = DeltaKalman1D(
            init=self.wheelbase,
            measurement_variance=dist_measurement_sigma ** 2,
            process_variance_rate=dist_process_sigma ** 2,
            process_gain=float(gp('sync_dist_process_gain').value),
            angle=False)
        self.yaw_kalman = DeltaKalman1D(
            init=0.0,
            measurement_variance=yaw_measurement_sigma ** 2,
            process_variance_rate=yaw_process_sigma ** 2,
            process_gain=float(gp('sync_yaw_process_gain').value),
            angle=True)
        self.lateral_kalman = DeltaKalman1D(
            init=self.target_lateral,
            measurement_variance=lateral_measurement_sigma ** 2,
            process_variance_rate=lateral_process_sigma ** 2,
            process_gain=float(gp('sync_lateral_process_gain').value),
            angle=False)
        self.lateral_pid = PID(
            float(gp('sync_lateral_kp').value),
            float(gp('sync_lateral_ki').value),
            float(gp('sync_lateral_kd').value),
            out_limit=float(
                gp('sync_lateral_max_correction_mps').value))
        self.reference_capture = MissionReferenceCapture(
            sample_count=int(gp('sync_reference_capture_samples').value),
            timeout_s=float(gp('sync_reference_capture_timeout_s').value),
            nominal_x=self.wheelbase,
            nominal_y=self.target_lateral,
            nominal_yaw=self.target_relative_yaw,
            max_x_error=float(gp('sync_reference_max_x_error_m').value),
            max_y_error=float(
                gp('sync_reference_max_lateral_error_m').value),
            max_yaw_error=math.radians(float(
                gp('sync_reference_max_yaw_error_deg').value)),
            max_std_x=float(
                gp('sync_reference_max_sample_std_x_m').value),
            max_std_y=float(
                gp('sync_reference_max_sample_std_y_m').value),
            max_std_yaw=math.radians(float(
                gp('sync_reference_max_sample_std_yaw_deg').value)),
            max_retries=int(gp('sync_reference_max_retries').value),
            retry_delay_s=float(
                gp('sync_reference_retry_delay_s').value))
        self.reference_settle_time = float(
            gp('sync_reference_settle_time_s').value)
        if self.reference_settle_time < 0.0:
            raise ValueError('sync_reference_settle_time_s must be non-negative')
        self._relative_correction_source = 'WHEEL_ONLY'
        self._last_cctv_correction_time = None

        common_gate = dict(
            sigma_limit=float(gp('aruco_innovation_sigma_gate').value),
            reacquire_count=int(gp('aruco_reacquire_count').value),
        )

        def make_gates():
            return {
                'x': ScalarObservationGate(
                    innovation_limit=float(
                        gp('aruco_distance_innovation_gate_m').value),
                    reacquire_limit=float(
                        gp('aruco_reacquire_distance_m').value),
                    consistency_limit=float(
                        gp('aruco_consistency_distance_m').value),
                    **common_gate),
                'lateral': ScalarObservationGate(
                    innovation_limit=float(
                        gp('aruco_lateral_innovation_gate_m').value),
                    reacquire_limit=float(
                        gp('aruco_reacquire_lateral_m').value),
                    consistency_limit=float(
                        gp('aruco_consistency_lateral_m').value),
                    **common_gate),
                'yaw': ScalarObservationGate(
                    innovation_limit=math.radians(float(
                        gp('aruco_yaw_innovation_gate_deg').value)),
                    reacquire_limit=math.radians(float(
                        gp('aruco_reacquire_yaw_deg').value)),
                    consistency_limit=math.radians(float(
                        gp('aruco_consistency_yaw_deg').value)),
                    angle=True, **common_gate),
            }
        self._aruco_gates = make_gates()
        self._cctv_pair_gates = make_gates()
        self._aruco_consumer = OncePerStamp()

        self.aruco_stamp_ns = None
        self.aruco_lateral = None
        self._last_visual_seen_time = None
        self._last_correction_time = {
            'x': None, 'lateral': None, 'yaw': None}
        self._last_gate_decision = {
            'x': 'NONE', 'lateral': 'NONE', 'yaw': 'NONE'}
        self._last_visual_decision = 'NONE'
        self._last_visual_reason = 'no_measurement'

        self._wheel_local = {'front': None, 'rear': None}
        self._wheel_stamp_ns = {'front': 0, 'rear': 0}
        self._wheel_receipt_time = {'front': 0.0, 'rear': 0.0}
        self._wheel_anchor_local = {'front': None, 'rear': None}
        self._wheel_anchor_world = {'front': None, 'rear': None}
        self._wheel_predictor_initialized = False
        self._last_predictor_source = None

        self._cctv_pose = {'front': None, 'rear': None}
        self._cctv_stamp_ns = {'front': 0, 'rear': 0}
        self._cctv_receipt_time = {'front': 0.0, 'rear': 0.0}
        self._last_cctv_pair_used = {'front': 0, 'rear': 0}
        self._cctv_pair_stamp_gate = CctvPairStampGate(
            self.cctv_pair_sync_slop)
        self.lateral_pid.reset()

        self.create_subscription(
            Odometry, '/front/wheel_odom',
            lambda msg: self._wheel_odom_cb('front', msg),
            SENSOR_LATEST_QOS)
        self.create_subscription(
            Odometry, '/rear/wheel_odom',
            lambda msg: self._wheel_odom_cb('rear', msg),
            SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, '/front/cctv_pose',
            lambda msg: self._cctv_pose_cb('front', msg),
            SENSOR_LATEST_QOS)
        self.create_subscription(
            PoseStamped, '/rear/cctv_pose',
            lambda msg: self._cctv_pose_cb('rear', msg),
            SENSOR_LATEST_QOS)

        self.get_logger().info(
            'relative sync hardening active | one-frame-one-update | '
            f'raw_wheel_predict={self.use_raw_wheel_predictor} | '
            f'dist_R_sigma={dist_measurement_sigma * 1000.0:.1f}mm | '
            f'yaw_R_sigma={math.degrees(yaw_measurement_sigma):.1f}deg')

    def path_cb(self, msg):
        super().path_cb(msg)
        self.lateral_pid.reset()
        self.reference_capture.reset()

    def vehicle_lifted_cb(self, msg):
        was_lifted = self.vehicle_lifted
        super().vehicle_lifted_cb(msg)
        if self.vehicle_lifted and not was_lifted:
            self.reference_capture.reset(start_time=time.monotonic())
            self._err = 'REFERENCE_CAPTURE'
        elif not self.vehicle_lifted:
            self.reference_capture.reset()

    def vehicle_spec_cb(self, msg):
        super().vehicle_spec_cb(msg)
        if not self.vehicle_lifted:
            self.reference_capture.nominal = (
                self.wheelbase, self.target_lateral,
                self.target_relative_yaw)

    def send_stop(self):
        super().send_stop()
        self.lateral_pid.reset()

    def control_loop(self):
        if reference_blocks_drive(
                self.vehicle_lifted, self.front_robot_state,
                self.rear_robot_state, self.reference_capture.ready):
            now = time.monotonic()
            state = self.reference_capture.advance(now)
            if state == 'REFERENCE_FAILED':
                self.fatal_stop(
                    f'REFERENCE_CAPTURE_FAILED '
                    f'{self.reference_capture.reason} '
                    f'retries={self.reference_capture.retry_count}')
            elif state == 'REFERENCE_RETRY_WAIT':
                self.recoverable_hold(
                    f'REFERENCE_RETRY_WAIT '
                    f'{self.reference_capture.reason} '
                    f'{self.reference_capture.retry_count}/'
                    f'{self.reference_capture.max_retries}')
            else:
                self.send_stop()
                self._err = state
            self._info = self._reference_telemetry(now)
            return
        super().control_loop()

    def _reference_telemetry(self, now):
        reference = self.reference_capture.reference
        samples = self.reference_capture.samples
        std_x = std_y = std_yaw = None
        if len(samples) >= 2:
            xs, ys, yaws = zip(*samples)
            _, std_yaw = MissionReferenceCapture._yaw_median_and_std(yaws)
            available_xs = [value for value in xs if value is not None]
            std_x = (statistics.pstdev(available_xs)
                     if len(available_xs) >= 2 else None)
            std_y = statistics.pstdev(ys)
        return {
            'reference_state': self.reference_capture.state,
            'reference_sample_count': len(samples),
            'reference_retry_count': self.reference_capture.retry_count,
            'reference_reason': self.reference_capture.reason,
            'reference_capture_x_std': std_x,
            'reference_capture_x_available': (
                self.use_aruco_distance and std_x is not None),
            'reference_capture_y_std': std_y,
            'reference_capture_yaw_std': std_yaw,
            'relative_x_ref': (
                None if reference is None or not self.use_aruco_distance else
                reference.relative_x),
            'relative_y_ref': (None if reference is None else
                               reference.relative_y),
            'relative_yaw_ref': (None if reference is None else
                                 reference.relative_yaw),
            'id0_stream_age': (
                None if self.aruco_receipt_time is None else
                max(0.0, now - self.aruco_receipt_time)),
            'id0_stream_healthy': stream_is_healthy(
                None if self.aruco_receipt_time is None else
                now - self.aruco_receipt_time,
                self.aruco_timeout),
            'relative_correction_source': self._relative_correction_source,
        }

    @staticmethod
    def _yaw_from_quaternion(q):
        values = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1.0e-9:
            return None
        x, y, z, w = (value / norm for value in values)
        return math.atan2(
            2.0 * (w * z + x * y),
            1.0 - 2.0 * (y * y + z * z))

    def aruco_cb(self, msg):
        before = self.stamp_gates['aruco'].last_stamp_ns
        super().aruco_cb(msg)
        after = self.stamp_gates['aruco'].last_stamp_ns
        if after <= before:
            return
        lateral = float(msg.pose.position.y)
        if not math.isfinite(lateral):
            self.aruco_stamp_ns = None
            self.get_logger().warn(
                'ArUco lateral NaN/Inf rejected',
                throttle_duration_sec=2.0)
            return
        self.aruco_stamp_ns = after
        self.aruco_lateral = lateral
        self._last_visual_seen_time = self.aruco_receipt_time
        capture_settled = (
            self.reference_capture.started_at is not None and
            time.monotonic() - self.reference_capture.started_at >=
            self.reference_settle_time)
        if (self.vehicle_lifted and capture_settled and
                abs(lateral) <= self.aruco_lateral_envelope and
                self.reference_capture.add(
                    self.aruco_dist if self.use_aruco_distance else None,
                    lateral, self.aruco_yaw, now=time.monotonic())):
            self._lock_mission_reference()

    def _lock_mission_reference(self):
        reference = self.reference_capture.reference
        if reference is None:
            return
        now = time.monotonic()
        raw_x, raw_yaw, stamp_s, source, raw_y = (
            self._relative_predictor(now))
        locked_x = (raw_x if reference.relative_x is None else
                    reference.relative_x)
        if reference.relative_x is None:
            reference = type(reference)(
                locked_x, reference.relative_y, reference.relative_yaw,
                reference.std_x, reference.std_y, reference.std_yaw,
                reference.sample_count)
            self.reference_capture.reference = reference
        self.relative_x_kalman.reset(
            locked_x, raw_value=raw_x,
            covariance=self.initial_covariance_scale *
            self.relative_x_kalman.R, stamp_s=stamp_s)
        self.lateral_kalman.reset(
            reference.relative_y, raw_value=raw_y,
            covariance=self.initial_covariance_scale *
            self.lateral_kalman.R, stamp_s=stamp_s)
        self.yaw_kalman.reset(
            reference.relative_yaw, raw_value=raw_yaw,
            covariance=self.initial_covariance_scale * self.yaw_kalman.R,
            stamp_s=stamp_s)
        self._last_predictor_source = source
        self.get_logger().info(
            'mission relative reference locked: '
            f'x={reference.relative_x:.3f}m, '
            f'y={reference.relative_y:+.3f}m, '
            f'yaw={math.degrees(reference.relative_yaw):+.2f}deg')

    def _wheel_odom_cb(self, role, msg):
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0 or stamp_ns <= self._wheel_stamp_ns[role]:
            return
        if msg.header.frame_id not in ('', 'map'):
            self.get_logger().warn(
                f'{role} wheel_odom frame rejected: {msg.header.frame_id!r}',
                throttle_duration_sec=2.0)
            return
        yaw = self._yaw_from_quaternion(msg.pose.pose.orientation)
        values = (
            float(msg.pose.pose.position.x),
            float(msg.pose.pose.position.y), yaw)
        if yaw is None or not all(math.isfinite(value) for value in values):
            self.get_logger().warn(
                f'{role} wheel_odom invalid pose rejected',
                throttle_duration_sec=2.0)
            return
        self._wheel_local[role] = values
        self._wheel_stamp_ns[role] = stamp_ns
        self._wheel_receipt_time[role] = time.monotonic()

        # The raw streams may connect after a mission has already initialized.
        # Anchor them without changing the fused state or covariance, otherwise
        # the first raw sample would be interpreted as a physical jump.
        if (self.sync_filters_initialized and
                not self._wheel_predictor_initialized and
                all(self._wheel_local[name] is not None
                    for name in ('front', 'rear')) and
                self._anchor_raw_wheel_predictor()):
            now = time.monotonic()
            raw = self._raw_wheel_relative(now)
            if raw is not None:
                self.relative_x_kalman.reset(
                    self.relative_x_kalman.x, raw_value=raw[0],
                    covariance=self.relative_x_kalman.P, stamp_s=raw[2])
                self.yaw_kalman.reset(
                    self.yaw_kalman.x, raw_value=raw[1],
                    covariance=self.yaw_kalman.P, stamp_s=raw[2])
                self.lateral_kalman.reset(
                    self.lateral_kalman.x, raw_value=raw[4],
                    covariance=self.lateral_kalman.P, stamp_s=raw[2])
                self._last_predictor_source = raw[3]
                self.get_logger().info(
                    'raw wheel relative predictor connected and anchored')

    def _cctv_pose_cb(self, role, msg):
        if msg.header.frame_id != 'map':
            return
        stamp_ns = stamp_to_ns(msg.header.stamp)
        if stamp_ns <= 0 or stamp_ns <= self._cctv_stamp_ns[role]:
            return
        # Source stamps from the same Jetson are used for ordering and pairing.
        # Freshness uses local monotonic receipt time so Front/Jetson wall-clock
        # skew cannot turn a live overhead observation into a stale one.
        yaw = self._yaw_from_quaternion(msg.pose.orientation)
        values = (float(msg.pose.position.x), float(msg.pose.position.y), yaw)
        if yaw is None or not all(math.isfinite(value) for value in values):
            return
        self._cctv_pose[role] = values
        self._cctv_stamp_ns[role] = stamp_ns
        self._cctv_receipt_time[role] = time.monotonic()

    def _anchor_raw_wheel_predictor(self):
        if not self.use_raw_wheel_predictor:
            self._wheel_predictor_initialized = False
            return False
        if any(self._wheel_local[role] is None for role in ('front', 'rear')):
            self._wheel_predictor_initialized = False
            return False
        self._wheel_anchor_local = {
            role: tuple(self._wheel_local[role]) for role in ('front', 'rear')}
        self._wheel_anchor_world = {
            'front': (
                self.front['x'], self.front['y'], self.front['theta']),
            'rear': (
                self.rear['x'], self.rear['y'], self.rear['theta']),
        }
        self._wheel_predictor_initialized = True
        return True

    def _raw_wheel_relative(self, now):
        if not self._wheel_predictor_initialized:
            return None
        if any(
                now - self._wheel_receipt_time[role] >
                self.wheel_relative_timeout
                for role in ('front', 'rear')):
            return None
        world = {
            role: anchored_pose(
                self._wheel_anchor_world[role],
                self._wheel_anchor_local[role],
                self._wheel_local[role])
            for role in ('front', 'rear')
        }
        front = dict(x=world['front'][0], y=world['front'][1],
                     theta=world['front'][2])
        rear = dict(x=world['rear'][0], y=world['rear'][1],
                    theta=world['rear'][2])
        longitudinal, lateral, yaw = (
            self.kinematics.relative_pose_in_rear_frame(front, rear))
        stamp_s = max(self._wheel_receipt_time.values())
        return longitudinal, yaw, stamp_s, 'RAW_WHEEL_ODOM', lateral

    def _relative_predictor(self, now):
        raw = self._raw_wheel_relative(now)
        if raw is not None:
            return raw
        longitudinal, lateral, yaw = (
            self.kinematics.relative_pose_in_rear_frame(
                self.front, self.rear))
        stamp_s = max(self.front['t'], self.rear['t'])
        return longitudinal, yaw, stamp_s, 'FUSED_ODOM_FALLBACK', lateral

    def _initialize_sync_filters(self):
        if not (self.front_ready and self.rear_ready):
            return False
        fused_x, fused_lateral, fused_yaw = (
            self.kinematics.relative_pose_in_rear_frame(
                self.front, self.rear))
        if not all(math.isfinite(value) for value in (
                fused_x, fused_lateral, fused_yaw)):
            return False

        self._anchor_raw_wheel_predictor()
        raw_x, raw_yaw, stamp_s, source, raw_lateral = self._relative_predictor(
            time.monotonic())
        self.relative_x_kalman.reset(
            fused_x, raw_value=raw_x,
            covariance=(self.initial_covariance_scale *
                        self.relative_x_kalman.R),
            stamp_s=stamp_s)
        self.yaw_kalman.reset(
            fused_yaw, raw_value=raw_yaw,
            covariance=self.initial_covariance_scale * self.yaw_kalman.R,
            stamp_s=stamp_s)
        self.lateral_kalman.reset(
            fused_lateral, raw_value=raw_lateral,
            covariance=self.initial_covariance_scale * self.lateral_kalman.R,
            stamp_s=stamp_s)
        self._aruco_consumer.reset()
        for gate in (*self._aruco_gates.values(),
                     *self._cctv_pair_gates.values()):
            gate.reset()
        self._last_cctv_pair_used = {'front': 0, 'rear': 0}
        self._cctv_pair_stamp_gate.reset()
        self._last_cctv_correction_time = None
        initialized_at = time.monotonic()
        self._last_visual_seen_time = None
        self._last_correction_time = {
            axis: initialized_at for axis in ('x', 'lateral', 'yaw')}
        self.marker_lost_since = None
        self._last_predictor_source = source
        self.sync_filters_initialized = True
        self.get_logger().info(
            f'상대필터 초기화: relative_x={fused_x:.3f}m, '
            f'yaw={math.degrees(fused_yaw):+.2f}deg, predictor={source}')
        return True

    def _new_cctv_pair(self, now):
        if any(self._cctv_pose[role] is None for role in ('front', 'rear')):
            return None
        if any(
                now - self._cctv_receipt_time[role] > self.cctv_pair_timeout
                for role in ('front', 'rear')):
            return None
        front_stamp = self._cctv_stamp_ns['front']
        rear_stamp = self._cctv_stamp_ns['rear']
        if not self._cctv_pair_stamp_gate.accept(front_stamp, rear_stamp):
            return None
        self._last_cctv_pair_used = {
            'front': front_stamp, 'rear': rear_stamp}
        front = self._cctv_pose['front']
        rear = self._cctv_pose['rear']
        front_pose = dict(x=front[0], y=front[1], theta=front[2])
        rear_pose = dict(x=rear[0], y=rear[1], theta=rear[2])
        longitudinal, lateral, yaw = (
            self.kinematics.relative_pose_in_rear_frame(
                front_pose, rear_pose))
        self._last_visual_seen_time = now
        return longitudinal, lateral, yaw

    def _apply_visual_measurement(
            self, *, source, relative_x, lateral, yaw, gates,
            raw_x, raw_lateral, raw_yaw, predictor_stamp_s, now):
        measurements = {
            'x': relative_x, 'lateral': lateral, 'yaw': yaw}
        filters = {
            'x': self.relative_x_kalman,
            'lateral': self.lateral_kalman,
            'yaw': self.yaw_kalman}
        raw_values = {
            'x': raw_x, 'lateral': raw_lateral, 'yaw': raw_yaw}
        accepted = []
        reasons = []
        for axis in ('x', 'lateral', 'yaw'):
            measurement = measurements[axis]
            if measurement is None:
                self._last_gate_decision[axis] = 'NO_MEASUREMENT'
                continue
            decision = gates[axis].evaluate(measurement, filters[axis])
            self._last_gate_decision[axis] = decision.action
            reasons.append(f'{axis}={decision.action}:{decision.reason}')
            if decision.action == 'ACCEPT':
                filters[axis].update(measurement)
            elif decision.action == 'REACQUIRE':
                filters[axis].reset(
                    measurement, raw_value=raw_values[axis],
                    covariance=self.initial_covariance_scale * filters[axis].R,
                    stamp_s=predictor_stamp_s)
                self.get_logger().warn(
                    f'{source} {axis} filter re-acquired after consistent '
                    'bounded observations')
            else:
                residual = decision.residual
                suffix = ('deg' if axis == 'yaw' else 'm')
                shown = (math.degrees(residual)
                         if axis == 'yaw' else residual)
                self.get_logger().warn(
                    f'{source} {axis} innovation rejected: '
                    f'{shown:+.3f}{suffix}, {decision.reason}',
                    throttle_duration_sec=1.0)
                continue
            self._last_correction_time[axis] = now
            accepted.append(f'{axis}:{decision.action}')

        self._last_visual_decision = (
            ','.join(accepted) if accepted else 'REJECT')
        self._last_visual_reason = f'{source}:' + ','.join(reasons)
        return (f'{source}_' + '_'.join(accepted)) if accepted else None

    def _consume_visual_measurement(
            self, now, raw_x, raw_lateral, raw_yaw, predictor_stamp_s):
        aruco_age = (None if self.aruco_receipt_time is None else
                     now - self.aruco_receipt_time)
        aruco_fresh = (
            stream_is_healthy(aruco_age, self.aruco_timeout) and
            self.aruco_yaw is not None and self.aruco_stamp_ns is not None)
        if aruco_fresh and self._aruco_consumer.consume(self.aruco_stamp_ns):
            self._last_visual_seen_time = self.aruco_receipt_time
            if (self.aruco_lateral is None or
                    abs(self.aruco_lateral) > self.aruco_lateral_envelope):
                self._last_visual_decision = 'REJECT'
                self._last_visual_reason = 'ARUCO:lateral_envelope'
                self.get_logger().warn(
                    'ID0 lateral envelope exceeded: '
                    f'{self.aruco_lateral!r}m',
                    throttle_duration_sec=1.0)
            else:
                correction = self._apply_visual_measurement(
                    source=('ID0_X_Y_YAW' if self.use_aruco_distance
                            else 'ID0_Y_YAW'),
                    relative_x=(self.aruco_dist
                                if self.use_aruco_distance else None),
                    lateral=self.aruco_lateral,
                    yaw=self.aruco_yaw,
                    gates=self._aruco_gates,
                    raw_x=raw_x,
                    raw_lateral=raw_lateral,
                    raw_yaw=raw_yaw,
                    predictor_stamp_s=predictor_stamp_s,
                    now=now)
                if correction is not None:
                    self._relative_correction_source = 'ID0_WHEEL'
                    return correction

        # No new frame in this 20 ms cycle is not ID0 loss. While the ID0
        # receipt age is healthy, relative correction remains wheel+ID0 only.
        if not cctv_fallback_allowed(aruco_age, self.aruco_timeout):
            self._relative_correction_source = 'ID0_WHEEL'
            return None

        cctv_pair = self._new_cctv_pair(now)
        if cctv_pair is not None:
            correction = self._apply_visual_measurement(
                source='CCTV_TOP_PAIR',
                relative_x=cctv_pair[0],
                lateral=cctv_pair[1],
                yaw=cctv_pair[2],
                gates=self._cctv_pair_gates,
                raw_x=raw_x,
                raw_lateral=raw_lateral,
                raw_yaw=raw_yaw,
                predictor_stamp_s=predictor_stamp_s,
                now=now)
            if correction is not None:
                self._relative_correction_source = 'CCTV_FALLBACK'
                self._last_cctv_correction_time = now
                return correction
        cctv_fallback_fresh = (
            self._last_cctv_correction_time is not None and
            now - self._last_cctv_correction_time <= self.cctv_pair_timeout)
        self._relative_correction_source = (
            'CCTV_FALLBACK' if cctv_fallback_fresh else 'WHEEL_ONLY')
        return None

    @staticmethod
    def _deadband(error, width):
        magnitude = abs(error)
        if magnitude <= width:
            return 0.0
        return math.copysign(magnitude - width, error)

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        centre_vx, centre_vy, centre_omega = (
            self.kinematics.control_point_twist_to_centre(
                vx, vy, omega,
                self.vehicle_offset_body[0], self.vehicle_offset_body[1]))
        front_vel, rear_vel = self.kinematics.split(
            centre_vx, centre_vy, centre_omega)

        raw_x, raw_yaw, predictor_stamp_s, predictor_source, raw_lateral = (
            self._relative_predictor(now))
        if predictor_source != self._last_predictor_source:
            # Raw wheel and fused-odom fallback have different absolute raw
            # references. Rebase the delta input while preserving the current
            # corrected estimate and covariance.
            self.relative_x_kalman.reset(
                self.relative_x_kalman.x, raw_value=raw_x,
                covariance=self.relative_x_kalman.P,
                stamp_s=predictor_stamp_s)
            self.yaw_kalman.reset(
                self.yaw_kalman.x, raw_value=raw_yaw,
                covariance=self.yaw_kalman.P, stamp_s=predictor_stamp_s)
            self.lateral_kalman.reset(
                self.lateral_kalman.x, raw_value=raw_lateral,
                covariance=self.lateral_kalman.P,
                stamp_s=predictor_stamp_s)
            self._last_predictor_source = predictor_source
        else:
            self.relative_x_kalman.predict_from_raw(
                raw_x, predictor_stamp_s)
            self.yaw_kalman.predict_from_raw(raw_yaw, predictor_stamp_s)
            self.lateral_kalman.predict_from_raw(
                raw_lateral, predictor_stamp_s)

        correction = self._consume_visual_measurement(
            now, raw_x, raw_lateral, raw_yaw, predictor_stamp_s)
        if correction is None:
            correction = predictor_source

        fused_x = self.relative_x_kalman.x
        fused_lateral = self.lateral_kalman.x
        reference = self.reference_capture.reference
        if reference is None:
            self.recoverable_hold('REFERENCE_NOT_READY')
            return False
        relative_x_error = fused_x - reference.relative_x
        lateral_error = fused_lateral - reference.relative_y
        relative_yaw_error = normalize_angle(
            self.yaw_kalman.x - reference.relative_yaw)

        visual_grace = max(self.aruco_timeout, self.cctv_pair_timeout)
        if self._last_visual_seen_time is None:
            if self.marker_lost_since is None:
                self.marker_lost_since = now
        else:
            visual_expired_at = self._last_visual_seen_time + visual_grace
            if now > visual_expired_at:
                if self.marker_lost_since is None:
                    self.marker_lost_since = visual_expired_at
            else:
                self.marker_lost_since = None
                if self._err.startswith('MARKER_HOLD'):
                    self._err = 'OK'

        speed_scale = 1.0
        effective_yaw_limit = self.yaw_limit
        id0_age = (None if self.aruco_receipt_time is None else
                   max(0.0, now - self.aruco_receipt_time))
        if (self._relative_correction_source == 'CCTV_FALLBACK' and
                (id0_age is None or id0_age > self.marker_stop)):
            self.recoverable_hold(
                'ID0_LOSS_HOLD ' +
                ('never_seen' if id0_age is None else f'{id0_age:.1f}s'))
            return False
        if (self._relative_correction_source == 'CCTV_FALLBACK' and
                id0_age is not None and id0_age > self.marker_slowdown):
            speed_scale = 0.5
        # Visibility and correction validity are separate safety signals. A
        # visible marker with one rejected axis is degraded, not "lost", but
        # it must not permit indefinite fast wheel-only operation.
        safety, safety_age, stale_axes = visual_safety_state(
            now=now, marker_lost_since=self.marker_lost_since,
            correction_times=self._last_correction_time,
            slowdown_s=self.marker_slowdown, stop_s=self.marker_stop,
            correction_grace_s=visual_grace)
        if safety == 'MARKER_HOLD':
            self.recoverable_hold(f'MARKER_HOLD {safety_age:.1f}s')
            return False
        if safety == 'MARKER_SLOW':
            speed_scale = 0.5
            effective_yaw_limit = self.yaw_limit * 2.0
        if safety in ('CORRECTION_STALE', 'CORRECTION_HOLD'):
            label = '_'.join(axis.upper() for axis in stale_axes)
            self._err = f'{label}_{safety} {safety_age:.1f}s'
            if safety == 'CORRECTION_HOLD':
                self.recoverable_hold(
                    f'{label}_CORRECTION_HOLD {safety_age:.1f}s')
                return False
            if safety_age > self.marker_slowdown:
                speed_scale = min(speed_scale, 0.5)

        if abs(relative_yaw_error) > effective_yaw_limit:
            self.fatal_stop(
                f'YAW_ERROR {math.degrees(relative_yaw_error):.1f}deg')
            return False

        abs_dist_error = abs(relative_x_error)
        if abs_dist_error >= self.dist_stop_limit:
            self.fatal_stop(
                f'RELATIVE_X_ERROR_FATAL {relative_x_error * 1000:.0f}mm')
            return False
        if abs_dist_error > self.dist_limit:
            if self.dist_error_since is None:
                self.dist_error_since = now
            elif now - self.dist_error_since > self.dist_error_timeout:
                self.fatal_stop(
                    'RELATIVE_X_ERROR_TIMEOUT '
                    f'{relative_x_error * 1000:.0f}mm')
                return False
            speed_scale = min(speed_scale, 0.30)
            self._err = (
                f'RELATIVE_X_ERROR {relative_x_error * 1000:.0f}mm')
        else:
            self.dist_error_since = None

        pid_dist_error = self._deadband(
            relative_x_error, self.dist_deadband)
        pid_yaw_error = self._deadband(
            relative_yaw_error, self.yaw_deadband)
        pid_lateral_error = self._deadband(
            lateral_error, self.lateral_deadband)
        corr_x = self.dist_pid.compute(pid_dist_error, 0.02)
        corr_y = self.lateral_pid.compute(pid_lateral_error, 0.02)
        corr_w = self.yaw_pid.compute(pid_yaw_error, 0.02)

        front_cmd, rear_cmd = self.kinematics.apply_relative_correction(
            front_vel, rear_vel, corr_x, corr_y, corr_w)
        front_cmd = tuple(value * speed_scale for value in front_cmd)
        rear_cmd = tuple(value * speed_scale for value in rear_cmd)

        front_cmd, rear_cmd = self.kinematics.limit_twist_pair(
            front_cmd, rear_cmd,
            linear_limit * speed_scale,
            angular_limit * speed_scale)
        self.publish_twist(self.pub_fc, front_cmd, 'front_base')
        self.publish_twist(self.pub_rc, rear_cmd, 'rear_base')

        visual_age = (
            None if self._last_visual_seen_time is None else
            max(0.0, now - self._last_visual_seen_time))
        correction_ages = {
            axis: (None if stamp is None else max(0.0, now - stamp))
            for axis, stamp in self._last_correction_time.items()}
        self._info = {
            'mode': mode,
            'enc_dist_cm': round(raw_x * 100.0, 2),
            'relative_predictor': predictor_source,
            'aruco_distance_used': self.use_aruco_distance,
            'aruco_raw_cm': (None if self.aruco_raw_dist is None else
                             round(self.aruco_raw_dist * 100.0, 2)),
            'aruco_lateral_cm': (None if self.aruco_lateral is None else
                                 round(self.aruco_lateral * 100.0, 2)),
            'fused_dist_cm': round(fused_x * 100.0, 2),
            'relative_x_raw': round(raw_x, 4),
            'relative_y_raw': round(raw_lateral, 4),
            'relative_yaw_raw': round(raw_yaw, 4),
            'relative_x_fused': round(fused_x, 4),
            'relative_y_fused': round(fused_lateral, 4),
            'relative_yaw_fused': round(self.yaw_kalman.x, 4),
            'raw_relative_lateral': round(raw_lateral, 4),
            'fused_relative_lateral': round(fused_lateral, 4),
            'lateral_error': round(lateral_error, 4),
            'lateral_correction': round(corr_y, 4),
            'lateral_std': round(math.sqrt(self.lateral_kalman.P), 4),
            'dist_err_mm': round(relative_x_error * 1000.0, 1),
            'relative_x_error': round(relative_x_error, 4),
            'relative_y_error': round(lateral_error, 4),
            'relative_yaw_error': round(relative_yaw_error, 4),
            'relative_yaw_err_deg': round(
                math.degrees(relative_yaw_error), 2),
            'correction': correction,
            'visual_decision': self._last_visual_decision,
            'visual_reason': self._last_visual_reason,
            'visual_age_s': (None if visual_age is None else
                             round(visual_age, 3)),
            'visual_seen_age': (None if visual_age is None else
                                round(visual_age, 3)),
            'distance_gate_decision': self._last_gate_decision['x'],
            'relative_x_gate_decision': self._last_gate_decision['x'],
            'lateral_gate_decision': self._last_gate_decision['lateral'],
            'yaw_gate_decision': self._last_gate_decision['yaw'],
            'distance_correction_age': correction_ages['x'],
            'relative_x_correction_age': correction_ages['x'],
            'lateral_correction_age': correction_ages['lateral'],
            'yaw_correction_age': correction_ages['yaw'],
            'dist_std_mm': round(
                math.sqrt(self.relative_x_kalman.P) * 1000.0, 2),
            'yaw_std_deg': round(
                math.degrees(math.sqrt(self.yaw_kalman.P)), 2),
            'speed_scale': speed_scale,
        }
        self._info.update(self._reference_telemetry(now))
        if extra_info:
            self._info.update(extra_info)
        return True


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
