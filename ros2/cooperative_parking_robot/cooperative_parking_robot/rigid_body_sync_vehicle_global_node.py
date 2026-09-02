#!/usr/bin/env python3
"""Main rigid controller with YOLO global x/y and ID0 relative control.

The existing rotate-then-insert mission and ID0 rotation-phase controller are
unchanged.  This wrapper fills two missing production links:

* `/parking/vehicle_pose_feedback` corrects a map-frame transport bias instead
  of mutating the physical vehicle mounting offset;
* measured-bad wheel relative-y is held, while ID0 remains the relative
  lateral measurement authority.
"""

from __future__ import annotations

import math
import time

import rclpy

from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.rigid_body_sync_phase_node import (
    RigidBodySyncNode as PhaseRigidBodySyncNode,
)
from cooperative_parking_robot.rotation_phase_control import (
    PhaseAwareRigidBodyKinematics,
)
from cooperative_parking_robot.vehicle_global_pose import (
    VehicleGlobalPoseTracker,
)


class VehicleGlobalPhaseKinematics(PhaseAwareRigidBodyKinematics):
    """Use corrected transport pose for global path control only."""

    def __init__(self, wheelbase, tracker):
        super().__init__(wheelbase)
        self.tracker = tracker

    def virtual_pose(self, front, rear):
        # Relative-state code calls relative_pose_in_rear_frame() separately,
        # so this substitution cannot inject YOLO into Front/Rear formation.
        return self.tracker.transport_pose(front, rear)


class RigidBodySyncNode(PhaseRigidBodySyncNode):
    """Separate global vehicle localization from relative formation control."""

    def __init__(self):
        self.vehicle_global_tracker = None
        self.vehicle_global_pose_enabled = False
        self.vehicle_global_pose_timeout = 0.75
        self.vehicle_global_fallback_scale = 0.35
        self.vehicle_global_blind_hold = 2.0
        self.use_wheel_lateral_predictor = True
        self._held_raw_lateral = None
        self._both_visual_stale_since = None
        super().__init__()

        self.declare_parameter('vehicle_global_pose_enabled', True)
        self.declare_parameter('vehicle_global_pose_timeout_s', 0.75)
        self.declare_parameter(
            'vehicle_global_fallback_speed_scale', 0.35)
        self.declare_parameter('vehicle_global_blind_hold_s', 2.0)
        self.declare_parameter('sync_use_wheel_lateral_predictor', False)
        gp = self.get_parameter
        self.vehicle_global_pose_enabled = bool(
            gp('vehicle_global_pose_enabled').value)
        self.vehicle_global_pose_timeout = float(
            gp('vehicle_global_pose_timeout_s').value)
        self.vehicle_global_fallback_scale = float(
            gp('vehicle_global_fallback_speed_scale').value)
        self.vehicle_global_blind_hold = float(
            gp('vehicle_global_blind_hold_s').value)
        self.use_wheel_lateral_predictor = bool(
            gp('sync_use_wheel_lateral_predictor').value)
        if (self.vehicle_global_pose_timeout <= 0.0 or
                self.vehicle_global_blind_hold <= 0.0 or
                not 0.0 < self.vehicle_global_fallback_scale <= 1.0):
            raise ValueError('invalid vehicle-global fallback parameters')

        self.vehicle_global_tracker = VehicleGlobalPoseTracker(
            position_gate_m=self.cctv_feedback_gate,
            position_alpha=self.cctv_offset_alpha)
        self.kinematics = VehicleGlobalPhaseKinematics(
            self.wheelbase, self.vehicle_global_tracker)
        self.get_logger().info(
            'global vehicle pose active | YOLO x/y map correction + '
            'Front/Rear odom-heading mean')
        self.get_logger().info(
            'relative lateral source | '
            + ('wheel delta + ID0' if self.use_wheel_lateral_predictor else
               'ID0 measurement; wheel relative-y held'))

    def _reset_vehicle_global(self):
        tracker = getattr(self, 'vehicle_global_tracker', None)
        if tracker is not None:
            tracker.reset()
        self._held_raw_lateral = None
        self._both_visual_stale_since = None

    def path_cb(self, msg):
        self._reset_vehicle_global()
        result = super().path_cb(msg)
        self._seed_global_pose_if_ready()
        return result

    def target_cb(self, msg):
        result = super().target_cb(msg)
        self._seed_global_pose_if_ready()
        return result

    def _seed_global_pose_if_ready(self):
        tracker = self.vehicle_global_tracker
        if (self.vehicle_global_pose_enabled and tracker is not None and
                self.has_path and self.target_offset_initialized and
                self.target_pose is not None and not tracker.initialized):
            # The fixed body offset was just derived from this validated target,
            # therefore the current prediction already equals the target pose.
            tracker.seed(time.monotonic())

    def vehicle_lifted_cb(self, msg):
        if not bool(msg.data):
            self._reset_vehicle_global()
        return super().vehicle_lifted_cb(msg)

    def cctv_feedback_cb(self, msg):
        """Update map bias while leaving `vehicle_offset_body` unchanged."""
        tracker = self.vehicle_global_tracker
        if not self.vehicle_global_pose_enabled or tracker is None:
            return
        if (not self.has_path or not self.vehicle_lifted or
                self.front_robot_state != 'DRIVE' or
                self.rear_robot_state != 'DRIVE' or
                not (self.front_ready and self.rear_ready)):
            return
        if msg.header.frame_id != 'map':
            return
        if not self._accept_stamped('cctv_feedback', msg):
            return
        measured = (float(msg.pose.position.x), float(msg.pose.position.y))
        if not all(math.isfinite(value) for value in measured):
            return
        now = time.monotonic()
        accepted = tracker.update(
            measured_x_m=measured[0], measured_y_m=measured[1],
            front=self.front, rear=self.rear,
            offset_body_x=self.vehicle_offset_body[0],
            offset_body_y=self.vehicle_offset_body[1],
            source_stamp_ns=stamp_to_ns(msg.header.stamp), now_s=now)
        if accepted:
            self.cctv_time = now
            self._both_visual_stale_since = None
        elif tracker.last_decision == 'POSITION_GATE':
            self.get_logger().warn(
                'YOLO vehicle global gate rejected: '
                f'{tracker.last_residual_m:.3f}m',
                throttle_duration_sec=1.0)
        if isinstance(self._info, dict):
            self._info.update(tracker.telemetry(now))

    def _relative_predictor(self, now):
        raw_x, raw_yaw, stamp_s, source, raw_lateral = (
            super()._relative_predictor(now))
        if self.use_wheel_lateral_predictor:
            return raw_x, raw_yaw, stamp_s, source, raw_lateral
        if self._held_raw_lateral is None:
            self._held_raw_lateral = raw_lateral
        return (
            raw_x, raw_yaw, stamp_s,
            f'{source}_LATERAL_HELD', self._held_raw_lateral)

    def _largest_current_lateral_error(self, now):
        """Never promote raw wheel lateral to the physical stop decision."""
        reference = self.reference_capture.reference
        if reference is None:
            return None
        return self.lateral_kalman.x - reference.relative_y

    def _id0_fresh(self, now):
        return bool(
            self.aruco_receipt_time is not None and
            0.0 <= now - self.aruco_receipt_time <= self.aruco_timeout)

    def _global_pose_fresh(self, now):
        return bool(
            self.vehicle_global_pose_enabled and
            self.vehicle_global_tracker is not None and
            self.vehicle_global_tracker.fresh(
                now, self.vehicle_global_pose_timeout))

    def _visual_speed_scale(self, id0_age):
        scale = super()._visual_speed_scale(id0_age)
        id0_stale = (
            id0_age is None or not math.isfinite(id0_age) or
            id0_age >= self.aruco_timeout)
        if id0_stale and self._global_pose_fresh(time.monotonic()):
            return min(scale, self.vehicle_global_fallback_scale)
        return scale

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        now = float(now)
        id0_fresh = self._id0_fresh(now)
        global_fresh = self._global_pose_fresh(now)
        if not id0_fresh and not global_fresh:
            if self._both_visual_stale_since is None:
                self._both_visual_stale_since = now
            stale_age = now - self._both_visual_stale_since
            if (self.vehicle_global_pose_enabled and
                    stale_age > self.vehicle_global_blind_hold):
                self.recoverable_hold(
                    f'RIGID_VISUAL_BLIND_HOLD {stale_age:.1f}s')
                return False
        else:
            self._both_visual_stale_since = None
            stale_age = 0.0

        tracker = self.vehicle_global_tracker
        merged_info = dict(extra_info or {})
        if tracker is not None:
            merged_info.update(tracker.telemetry(now))
        merged_info.update({
            'vehicle_global_pose_enabled': self.vehicle_global_pose_enabled,
            'vehicle_global_pose_fresh': global_fresh,
            'id0_fresh_for_formation': id0_fresh,
            'wheel_lateral_predictor_enabled':
                self.use_wheel_lateral_predictor,
            'rigid_visual_blind_age_s': round(stale_age, 3),
        })

        # Do not keep applying the last visual lateral error open-loop after
        # ID0 disappears.  Temporarily present zero relative-y error to this
        # cycle; preserve the accepted state for diagnostics and reacquisition.
        held_state = None
        reference = self.reference_capture.reference
        if not id0_fresh and reference is not None:
            held_state = self.lateral_kalman.x
            self.lateral_kalman.x = reference.relative_y
            self.lateral_pid.reset()
            merged_info['formation_fallback'] = (
                'GLOBAL_VEHICLE_XY_ZERO_STALE_LATERAL'
                if global_fresh else 'NO_FRESH_RELATIVE_LATERAL')
            merged_info['held_last_id0_lateral_m'] = round(held_state, 5)
        try:
            return super().apply_sync_and_publish(
                vx, vy, omega, now,
                mode=mode, linear_limit=linear_limit,
                angular_limit=angular_limit, extra_info=merged_info)
        finally:
            if held_state is not None:
                self.lateral_kalman.x = held_state
                self.lateral_pid.reset()

    def _reference_telemetry(self, now):
        info = super()._reference_telemetry(now)
        tracker = getattr(self, 'vehicle_global_tracker', None)
        if tracker is not None:
            info.update(tracker.telemetry(now))
        info.update({
            'vehicle_global_pose_enabled': self.vehicle_global_pose_enabled,
            'vehicle_global_pose_fresh': (
                False if tracker is None else
                tracker.fresh(now, self.vehicle_global_pose_timeout)),
            'wheel_lateral_predictor_enabled':
                self.use_wheel_lateral_predictor,
            'lateral_safety_basis': 'FUSED_ID0_ONLY',
        })
        return info


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
