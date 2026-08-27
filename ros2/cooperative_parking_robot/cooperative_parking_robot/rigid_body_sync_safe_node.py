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
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.relative_sync_filter import (
    DeltaKalman1D,
    OncePerStamp,
    RelativeObservationGate,
    anchored_pose,
    normalize_angle,
)
from cooperative_parking_robot.rigid_body_sync_node import (
    RigidBodySyncNode as LegacyRigidBodySyncNode,
)


class RigidBodySyncNode(LegacyRigidBodySyncNode):
    """Legacy motion controller with a corrected relative estimator."""

    def __init__(self):
        super().__init__()

        # Estimator noise is intentionally exposed. Defaults are conservative
        # placeholders; replace them with static-camera and wheel-slip logs.
        self.declare_parameter('sync_dist_process_sigma_m_sqrt_s', 0.003)
        self.declare_parameter('sync_yaw_process_sigma_deg_sqrt_s', 0.50)
        self.declare_parameter('sync_dist_process_gain', 0.03)
        self.declare_parameter('sync_yaw_process_gain', 0.03)
        self.declare_parameter('sync_dist_measurement_sigma_m', 0.015)
        self.declare_parameter('sync_yaw_measurement_sigma_deg', 3.0)
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

        self.declare_parameter('sync_dist_deadband_m', 0.003)
        self.declare_parameter('sync_yaw_deadband_deg', 0.50)
        self.declare_parameter('use_raw_wheel_relative_predictor', True)
        self.declare_parameter('wheel_relative_timeout_s', 0.50)
        self.declare_parameter('cctv_pair_timeout_s', 0.50)
        self.declare_parameter('cctv_pair_sync_slop_s', 0.12)

        gp = self.get_parameter
        dist_process_sigma = float(
            gp('sync_dist_process_sigma_m_sqrt_s').value)
        yaw_process_sigma = math.radians(float(
            gp('sync_yaw_process_sigma_deg_sqrt_s').value))
        dist_measurement_sigma = float(
            gp('sync_dist_measurement_sigma_m').value)
        yaw_measurement_sigma = math.radians(float(
            gp('sync_yaw_measurement_sigma_deg').value))
        self.initial_covariance_scale = float(
            gp('sync_initial_covariance_scale').value)
        self.dist_deadband = float(gp('sync_dist_deadband_m').value)
        self.yaw_deadband = math.radians(float(
            gp('sync_yaw_deadband_deg').value))
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
            dist_measurement_sigma, yaw_measurement_sigma,
            self.initial_covariance_scale, self.dist_deadband,
            self.yaw_deadband, self.wheel_relative_timeout,
            self.cctv_pair_timeout, self.cctv_pair_sync_slop,
            self.aruco_lateral_envelope)
        if any(value <= 0.0 for value in positive):
            raise ValueError('relative estimator parameters must be positive')

        self.dist_kalman = DeltaKalman1D(
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

        gate_kwargs = dict(
            distance_limit=float(
                gp('aruco_distance_innovation_gate_m').value),
            yaw_limit=math.radians(float(
                gp('aruco_yaw_innovation_gate_deg').value)),
            sigma_limit=float(gp('aruco_innovation_sigma_gate').value),
            reacquire_count=int(gp('aruco_reacquire_count').value),
            reacquire_distance_limit=float(
                gp('aruco_reacquire_distance_m').value),
            reacquire_yaw_limit=math.radians(float(
                gp('aruco_reacquire_yaw_deg').value)),
            consistency_distance=float(
                gp('aruco_consistency_distance_m').value),
            consistency_yaw=math.radians(float(
                gp('aruco_consistency_yaw_deg').value)),
        )
        self._aruco_gate = RelativeObservationGate(**gate_kwargs)
        self._cctv_pair_gate = RelativeObservationGate(**gate_kwargs)
        self._aruco_consumer = OncePerStamp()

        self.aruco_stamp_ns = None
        self.aruco_lateral = None
        self._last_accepted_visual_time = None
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

        self.create_subscription(
            Odometry, '/front/wheel_odom',
            lambda msg: self._wheel_odom_cb('front', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            Odometry, '/rear/wheel_odom',
            lambda msg: self._wheel_odom_cb('rear', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/front/cctv_pose',
            lambda msg: self._cctv_pose_cb('front', msg),
            qos_profile_sensor_data)
        self.create_subscription(
            PoseStamped, '/rear/cctv_pose',
            lambda msg: self._cctv_pose_cb('rear', msg),
            qos_profile_sensor_data)

        self.get_logger().info(
            'relative sync hardening active | one-frame-one-update | '
            f'raw_wheel_predict={self.use_raw_wheel_predictor} | '
            f'dist_R_sigma={dist_measurement_sigma * 1000.0:.1f}mm | '
            f'yaw_R_sigma={math.degrees(yaw_measurement_sigma):.1f}deg')

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
                self.dist_kalman.reset(
                    self.dist_kalman.x, raw_value=raw[0],
                    covariance=self.dist_kalman.P, stamp_s=raw[2])
                self.yaw_kalman.reset(
                    self.yaw_kalman.x, raw_value=raw[1],
                    covariance=self.yaw_kalman.P, stamp_s=raw[2])
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
        dx = world['front'][0] - world['rear'][0]
        dy = world['front'][1] - world['rear'][1]
        distance = math.hypot(dx, dy)
        yaw = normalize_angle(world['front'][2] - world['rear'][2])
        stamp_s = max(self._wheel_receipt_time.values())
        return distance, yaw, stamp_s, 'RAW_WHEEL_ODOM'

    def _relative_predictor(self, now):
        raw = self._raw_wheel_relative(now)
        if raw is not None:
            return raw
        distance = self.kinematics.encoder_distance(self.front, self.rear)
        yaw = normalize_angle(self.front['theta'] - self.rear['theta'])
        stamp_s = max(self.front['t'], self.rear['t'])
        return distance, yaw, stamp_s, 'FUSED_ODOM_FALLBACK'

    def _initialize_sync_filters(self):
        if not (self.front_ready and self.rear_ready):
            return False
        fused_dist = self.kinematics.encoder_distance(self.front, self.rear)
        fused_yaw = normalize_angle(
            self.front['theta'] - self.rear['theta'])
        if not all(math.isfinite(value) for value in (fused_dist, fused_yaw)):
            return False

        self._anchor_raw_wheel_predictor()
        raw_dist, raw_yaw, stamp_s, source = self._relative_predictor(
            time.monotonic())
        self.dist_kalman.reset(
            fused_dist, raw_value=raw_dist,
            covariance=self.initial_covariance_scale * self.dist_kalman.R,
            stamp_s=stamp_s)
        self.yaw_kalman.reset(
            fused_yaw, raw_value=raw_yaw,
            covariance=self.initial_covariance_scale * self.yaw_kalman.R,
            stamp_s=stamp_s)
        self._aruco_consumer.reset()
        self._aruco_gate.reset()
        self._cctv_pair_gate.reset()
        self._last_cctv_pair_used = {'front': 0, 'rear': 0}
        self._last_accepted_visual_time = None
        self.marker_lost_since = None
        self._last_predictor_source = source
        self.sync_filters_initialized = True
        self.get_logger().info(
            f'상대필터 초기화: distance={fused_dist:.3f}m, '
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
        if (front_stamp <= self._last_cctv_pair_used['front'] or
                rear_stamp <= self._last_cctv_pair_used['rear']):
            return None
        if abs(front_stamp - rear_stamp) * 1.0e-9 > self.cctv_pair_sync_slop:
            return None
        self._last_cctv_pair_used = {
            'front': front_stamp, 'rear': rear_stamp}
        front = self._cctv_pose['front']
        rear = self._cctv_pose['rear']
        distance = math.hypot(front[0] - rear[0], front[1] - rear[1])
        yaw = normalize_angle(front[2] - rear[2])
        return distance, yaw

    def _apply_visual_measurement(
            self, *, source, distance, yaw, gate, raw_dist, raw_yaw,
            predictor_stamp_s, now):
        decision = gate.evaluate(
            distance_measurement=distance,
            yaw_measurement=yaw,
            distance_filter=(self.dist_kalman if distance is not None else None),
            yaw_filter=self.yaw_kalman)
        self._last_visual_decision = decision.action
        self._last_visual_reason = f'{source}:{decision.reason}'
        if decision.action == 'ACCEPT':
            if distance is not None:
                self.dist_kalman.update(distance)
            self.yaw_kalman.update(yaw)
        elif decision.action == 'REACQUIRE':
            if distance is not None:
                self.dist_kalman.reset(
                    distance, raw_value=raw_dist,
                    covariance=(self.initial_covariance_scale *
                                self.dist_kalman.R),
                    stamp_s=predictor_stamp_s)
            self.yaw_kalman.reset(
                yaw, raw_value=raw_yaw,
                covariance=self.initial_covariance_scale * self.yaw_kalman.R,
                stamp_s=predictor_stamp_s)
            self.get_logger().warn(
                f'{source} relative filter re-acquired after consistent '
                'bounded observations')
        else:
            dist_text = ('n/a' if decision.distance_residual is None else
                         f'{decision.distance_residual * 1000.0:+.0f}mm')
            yaw_text = ('n/a' if decision.yaw_residual is None else
                        f'{math.degrees(decision.yaw_residual):+.1f}deg')
            self.get_logger().warn(
                f'{source} innovation rejected: dist={dist_text}, '
                f'yaw={yaw_text}, {decision.reason}',
                throttle_duration_sec=1.0)
            return None

        self._last_accepted_visual_time = now
        self.marker_lost_since = None
        return f'{source}_{decision.action}'

    def _consume_visual_measurement(
            self, now, raw_dist, raw_yaw, predictor_stamp_s):
        aruco_age = (None if self.aruco_receipt_time is None else
                     now - self.aruco_receipt_time)
        aruco_fresh = (
            aruco_age is not None and 0.0 <= aruco_age < self.aruco_timeout and
            self.aruco_yaw is not None and self.aruco_stamp_ns is not None)
        if aruco_fresh and self._aruco_consumer.consume(self.aruco_stamp_ns):
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
                    source=('ARUCO_DIST_YAW' if self.use_aruco_distance
                            else 'ARUCO_YAW'),
                    distance=(self.aruco_dist
                              if self.use_aruco_distance else None),
                    yaw=self.aruco_yaw,
                    gate=self._aruco_gate,
                    raw_dist=raw_dist,
                    raw_yaw=raw_yaw,
                    predictor_stamp_s=predictor_stamp_s,
                    now=now)
                if correction is not None:
                    return correction

        cctv_pair = self._new_cctv_pair(now)
        if cctv_pair is not None:
            correction = self._apply_visual_measurement(
                source='CCTV_TOP_PAIR',
                distance=cctv_pair[0],
                yaw=cctv_pair[1],
                gate=self._cctv_pair_gate,
                raw_dist=raw_dist,
                raw_yaw=raw_yaw,
                predictor_stamp_s=predictor_stamp_s,
                now=now)
            if correction is not None:
                return correction
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

        raw_dist, raw_yaw, predictor_stamp_s, predictor_source = (
            self._relative_predictor(now))
        if predictor_source != self._last_predictor_source:
            # Raw wheel and fused-odom fallback have different absolute raw
            # references. Rebase the delta input while preserving the current
            # corrected estimate and covariance.
            self.dist_kalman.reset(
                self.dist_kalman.x, raw_value=raw_dist,
                covariance=self.dist_kalman.P, stamp_s=predictor_stamp_s)
            self.yaw_kalman.reset(
                self.yaw_kalman.x, raw_value=raw_yaw,
                covariance=self.yaw_kalman.P, stamp_s=predictor_stamp_s)
            self._last_predictor_source = predictor_source
        else:
            self.dist_kalman.predict_from_raw(raw_dist, predictor_stamp_s)
            self.yaw_kalman.predict_from_raw(raw_yaw, predictor_stamp_s)

        correction = self._consume_visual_measurement(
            now, raw_dist, raw_yaw, predictor_stamp_s)
        if correction is None:
            correction = predictor_source

        fused_dist = self.dist_kalman.x
        relative_yaw_error = normalize_angle(self.yaw_kalman.x)
        dist_error = fused_dist - self.wheelbase

        visual_grace = max(self.aruco_timeout, self.cctv_pair_timeout)
        if self._last_accepted_visual_time is None:
            if self.marker_lost_since is None:
                self.marker_lost_since = now
        else:
            visual_expired_at = self._last_accepted_visual_time + visual_grace
            if now > visual_expired_at:
                if self.marker_lost_since is None:
                    self.marker_lost_since = visual_expired_at
            else:
                self.marker_lost_since = None
                if self._err.startswith('MARKER_HOLD'):
                    self._err = 'OK'

        speed_scale = 1.0
        effective_yaw_limit = self.yaw_limit
        if self.marker_lost_since is not None:
            lost = now - self.marker_lost_since
            if lost > self.marker_stop:
                self.recoverable_hold(f'MARKER_HOLD {lost:.1f}s')
                return False
            if lost > self.marker_slowdown:
                speed_scale = 0.5
                effective_yaw_limit = self.yaw_limit * 2.0

        if abs(relative_yaw_error) > effective_yaw_limit:
            self.fatal_stop(
                f'YAW_ERROR {math.degrees(relative_yaw_error):.1f}deg')
            return False

        abs_dist_error = abs(dist_error)
        if abs_dist_error >= self.dist_stop_limit:
            self.fatal_stop(f'DIST_ERROR_FATAL {dist_error * 1000:.0f}mm')
            return False
        if abs_dist_error > self.dist_limit:
            if self.dist_error_since is None:
                self.dist_error_since = now
            elif now - self.dist_error_since > self.dist_error_timeout:
                self.fatal_stop(
                    f'DIST_ERROR_TIMEOUT {dist_error * 1000:.0f}mm')
                return False
            speed_scale = min(speed_scale, 0.30)
            self._err = f'DIST_ERROR {dist_error * 1000:.0f}mm'
        else:
            self.dist_error_since = None

        pid_dist_error = self._deadband(dist_error, self.dist_deadband)
        pid_yaw_error = self._deadband(
            relative_yaw_error, self.yaw_deadband)
        corr_x = self.dist_pid.compute(pid_dist_error, 0.02)
        corr_w = self.yaw_pid.compute(pid_yaw_error, 0.02)

        front_cmd = (
            (front_vel[0] - 0.5 * corr_x) * speed_scale,
            front_vel[1] * speed_scale,
            (front_vel[2] - 0.5 * corr_w) * speed_scale)
        rear_cmd = (
            (rear_vel[0] + 0.5 * corr_x) * speed_scale,
            rear_vel[1] * speed_scale,
            (rear_vel[2] + 0.5 * corr_w) * speed_scale)

        front_cmd, rear_cmd = self.kinematics.limit_twist_pair(
            front_cmd, rear_cmd,
            linear_limit * speed_scale,
            angular_limit * speed_scale)
        self.publish_twist(self.pub_fc, front_cmd, 'front_base')
        self.publish_twist(self.pub_rc, rear_cmd, 'rear_base')

        visual_age = (
            None if self._last_accepted_visual_time is None else
            max(0.0, now - self._last_accepted_visual_time))
        self._info = {
            'mode': mode,
            'enc_dist_cm': round(raw_dist * 100.0, 2),
            'relative_predictor': predictor_source,
            'aruco_distance_used': self.use_aruco_distance,
            'aruco_raw_cm': (None if self.aruco_raw_dist is None else
                             round(self.aruco_raw_dist * 100.0, 2)),
            'aruco_lateral_cm': (None if self.aruco_lateral is None else
                                 round(self.aruco_lateral * 100.0, 2)),
            'fused_dist_cm': round(fused_dist * 100.0, 2),
            'dist_err_mm': round(dist_error * 1000.0, 1),
            'relative_yaw_err_deg': round(
                math.degrees(relative_yaw_error), 2),
            'correction': correction,
            'visual_decision': self._last_visual_decision,
            'visual_reason': self._last_visual_reason,
            'visual_age_s': (None if visual_age is None else
                             round(visual_age, 3)),
            'dist_std_mm': round(math.sqrt(self.dist_kalman.P) * 1000.0, 2),
            'yaw_std_deg': round(
                math.degrees(math.sqrt(self.yaw_kalman.P)), 2),
            'speed_scale': speed_scale,
        }
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
