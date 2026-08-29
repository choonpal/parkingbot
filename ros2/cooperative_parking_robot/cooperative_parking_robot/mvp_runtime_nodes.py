#!/usr/bin/env python3
"""Production MVP motion wrappers for command ownership and recovery hardening."""

from __future__ import annotations

import math
import time
from typing import Type

import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import Bool, String

from cooperative_parking_robot.mvp_integration_nodes import (
    HomeAwareIndividualMoveNode as BaseIndividualMoveNode,
)
from cooperative_parking_robot.mvp_recovery_policy import final_slot_command
from cooperative_parking_robot.rigid_body_sync_vision_node import (
    RigidBodySyncNode as BaseRigidBodySyncNode,
)
from cooperative_parking_robot.rigid_pair_teleop_core import (
    relative_pose_step_is_plausible,
)
from cooperative_parking_robot.vehicle_entry import marker_loss_speed_scale


SIDE_OFFSET_MARGIN_M = 0.015


def minimum_entry_side_offset(
        vehicle_half_width_m: float,
        robot_width_m: float,
        robot_clearance_m: float,
        margin_m: float = SIDE_OFFSET_MARGIN_M) -> float:
    """Return a millimetre-rounded side lane that clears the full envelope."""
    required = (
        float(vehicle_half_width_m) +
        0.5 * float(robot_width_m) +
        float(robot_clearance_m) +
        float(margin_m)
    )
    if not math.isfinite(required) or required <= 0.0:
        raise ValueError('entry side-offset geometry must be finite and positive')
    return math.ceil(required * 1000.0 - 1.0e-9) / 1000.0


def rigid_drive_owns_command(
        *, has_path: bool, vehicle_lifted: bool,
        front_state: str, rear_state: str,
        front_ready: bool, rear_ready: bool, estop: bool) -> bool:
    """True only while the rigid controller is the active cmd_vel owner."""
    return bool(
        not estop and has_path and vehicle_lifted and
        front_ready and rear_ready and
        str(front_state) == 'DRIVE' and str(rear_state) == 'DRIVE'
    )


class MvpIndividualMoveNode(BaseIndividualMoveNode):
    """Keep entry fail-closed while tolerating ordered-pose and occlusion issues."""

    def __init__(self, **kwargs):
        self._last_relative_pose_for_jump = None
        self._last_relative_pose_time_for_jump = None
        super().__init__(**kwargs)
        self.declare_parameter('underbody_visual_fallback_s', 5.0)
        self.declare_parameter('underbody_visual_fallback_speed_scale', 0.35)
        self.declare_parameter('relative_jump_forward_m', 0.04)
        self.declare_parameter('relative_jump_lateral_m', 0.04)
        self.declare_parameter('relative_jump_yaw_deg', 5.0)
        gp = self.get_parameter
        self.underbody_visual_fallback = float(
            gp('underbody_visual_fallback_s').value)
        self.underbody_visual_fallback_scale = float(
            gp('underbody_visual_fallback_speed_scale').value)
        self.relative_jump_forward = float(
            gp('relative_jump_forward_m').value)
        self.relative_jump_lateral = float(
            gp('relative_jump_lateral_m').value)
        self.relative_jump_yaw = math.radians(float(
            gp('relative_jump_yaw_deg').value))
        if (self.underbody_visual_fallback <= self.marker_stop or
                not 0.0 < self.underbody_visual_fallback_scale <= 0.5 or
                min(self.relative_jump_forward, self.relative_jump_lateral,
                    self.relative_jump_yaw) <= 0.0):
            raise ValueError('invalid MVP visual fallback/jump parameters')
        self.create_subscription(Bool, '/robot/rearm', self._rearm_cb, 10)

    def _validate_parameters(self):
        required = minimum_entry_side_offset(
            self.vehicle_half_width,
            self.robot_width,
            self.robot_clearance,
        )
        if self.entry_side_offset < required:
            configured = self.entry_side_offset
            result = self.set_parameters([
                Parameter(
                    'entry_side_offset_m',
                    Parameter.Type.DOUBLE,
                    required),
            ])[0]
            if not result.successful:
                raise RuntimeError(
                    'failed to apply valid entry_side_offset_m: '
                    f'{result.reason}')
            self.entry_side_offset = required
            self.get_logger().warn(
                'entry_side_offset_m '
                f'{configured:.3f}m is inside the configured envelope; '
                f'using {required:.3f}m')
        super()._validate_parameters()

    def _rearm_cb(self, msg):
        if not bool(msg.data):
            return
        self.fault_sent = False
        # Fleet keeps the latest motion-fault string. Explicitly publish the
        # cleared value so the next mission is not blocked by an old fault.
        self.pub_fault.publish(String(data=''))

    def relative_is_fresh(self):
        """Stamped pose freshness is authoritative; Bool ordering is not."""
        if self.relative_receipt_time is None or self.relative_x is None:
            return False
        age = time.monotonic() - self.relative_receipt_time
        return 0.0 <= age < self.aruco_timeout

    def relative_pose_cb(self, msg):
        previous = None
        if (self.relative_x is not None and self.relative_y is not None and
                self.relative_yaw is not None and
                self.relative_receipt_time is not None):
            previous = (
                (self.relative_x, self.relative_y, self.relative_yaw),
                self.relative_receipt_time,
                self.last_visual_observation_time,
            )
        super().relative_pose_cb(msg)
        if previous is None or self.relative_receipt_time == previous[1]:
            if self.relative_x is not None:
                self._last_relative_pose_for_jump = (
                    self.relative_x, self.relative_y, self.relative_yaw)
                self._last_relative_pose_time_for_jump = (
                    self.relative_receipt_time)
            return
        old_pose, old_time, old_visual_time = previous
        new_pose = (self.relative_x, self.relative_y, self.relative_yaw)
        if (time.monotonic() - old_time <= self.aruco_timeout and
                not relative_pose_step_is_plausible(
                    old_pose, new_pose,
                    forward_step_m=self.relative_jump_forward,
                    lateral_step_m=self.relative_jump_lateral,
                    yaw_step_rad=self.relative_jump_yaw)):
            self.relative_x, self.relative_y, self.relative_yaw = old_pose
            self.relative_receipt_time = old_time
            self.last_visual_observation_time = old_visual_time
            self.get_logger().warn(
                f'[{self.role}] ID0 pose jump rejected; keeping last pose',
                throttle_duration_sec=1.0)
            return
        self._last_relative_pose_for_jump = new_pose
        self._last_relative_pose_time_for_jump = self.relative_receipt_time

    def update_visual_fallback(self):
        """Permit bounded odom+ultrasonic entry while the underbody hides ID0."""
        if not self.underbody_visual_required():
            self.relative_lost_since = None
            self.motion_speed_scale = 1.0
            return True
        now = time.monotonic()
        if self.top_marker_is_fresh(now) or self.relative_is_fresh():
            self.relative_lost_since = None
            self.motion_speed_scale = 1.0
            return True
        if self.relative_lost_since is None:
            self.relative_lost_since = (
                self.last_visual_observation_time
                if self.last_visual_observation_time is not None else now)
        lost_age = now - self.relative_lost_since
        if lost_age <= self.marker_stop:
            self.motion_speed_scale = marker_loss_speed_scale(
                lost_age, self.marker_slowdown, self.marker_stop)
            return self.motion_speed_scale > 0.0
        odom_fresh = (
            self.odom_ready and
            0.0 <= now - self.last_odom_time <= self.odom_timeout)
        if (odom_fresh and self.ultrasonic_ready and
                lost_age <= self.underbody_visual_fallback):
            self.motion_speed_scale = self.underbody_visual_fallback_scale
            return True
        self.stop()
        self.motion_speed_scale = 0.0
        return False


class MvpRigidBodySyncNode(BaseRigidBodySyncNode):
    """Own DRIVE only and harden final insertion/visual fallback behavior."""

    def __init__(self, **kwargs):
        self._drive_command_owned = False
        self._verified_cctv_fallback_time = None
        self._yaw_visual_disagreement_since = None
        super().__init__(**kwargs)
        self.declare_parameter('final_max_omega', 0.15)
        self.declare_parameter('id0_verified_fallback_stop_s', 10.0)
        self.declare_parameter('id0_verified_fallback_speed_scale', 0.30)
        self.declare_parameter('yaw_visual_disagreement_grace_s', 0.50)
        gp = self.get_parameter
        self.final_max_omega = float(gp('final_max_omega').value)
        self.id0_verified_fallback_stop = float(
            gp('id0_verified_fallback_stop_s').value)
        self.id0_verified_fallback_scale = float(
            gp('id0_verified_fallback_speed_scale').value)
        self.yaw_visual_disagreement_grace = float(
            gp('yaw_visual_disagreement_grace_s').value)
        if (not 0.0 < self.final_max_omega <= self.max_omega or
                self.id0_verified_fallback_stop <= self.marker_stop or
                not 0.0 < self.id0_verified_fallback_scale <= 0.5 or
                self.yaw_visual_disagreement_grace <= 0.0):
            raise ValueError('invalid MVP rigid-body hardening parameters')
        self.get_logger().info(
            'cmd_vel ownership active | individual=APPROACH/ALIGN/RETURN, '
            'rigid=DRIVE')

    def _owns_drive_command_now(self) -> bool:
        return rigid_drive_owns_command(
            has_path=self.has_path,
            vehicle_lifted=self.vehicle_lifted,
            front_state=self.front_robot_state,
            rear_state=self.rear_robot_state,
            front_ready=self.front_ready,
            rear_ready=self.rear_ready,
            estop=self.estop,
        )

    def send_stop(self):
        """Send one final zero only when this node previously owned cmd_vel."""
        if not self._drive_command_owned:
            return
        super().send_stop()
        self._drive_command_owned = False

    def control_loop(self):
        if not self._owns_drive_command_now():
            self.send_stop()
            return
        self._drive_command_owned = True
        return super().control_loop()

    def compute_final_command(self, cx, cy, ct):
        done, base_command, info = super().compute_final_command(cx, cy, ct)
        if done or not self.align_to_slot_yaw:
            return done, base_command, info
        slot_yaw = self.slot_pose[2]
        yaw_error = self.angle_norm(slot_yaw - ct)
        rotation_radius = (
            self.kinematics.half_L + math.hypot(
                self.vehicle_offset_body[0], self.vehicle_offset_body[1]))
        command = final_slot_command(
            base_command=base_command,
            yaw_error=yaw_error,
            yaw_tolerance=self.final_yaw_tol,
            yaw_kp=self.yaw_hold_kp,
            max_omega=self.max_omega,
            max_speed=self.max_speed,
            rotation_radius=rotation_radius,
            final_max_omega=self.final_max_omega,
        )
        return False, command, info

    def _new_cctv_pair(self, now):
        pair = super()._new_cctv_pair(now)
        if pair is not None:
            self._verified_cctv_fallback_time = now
        return pair

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        fallback_recent = (
            self._verified_cctv_fallback_time is not None and
            now - self._verified_cctv_fallback_time <= self.cctv_pair_timeout)
        old_marker_stop = self.marker_stop
        id0_age = (
            None if self.aruco_receipt_time is None else
            max(0.0, now - self.aruco_receipt_time))
        if fallback_recent:
            self.marker_stop = max(
                self.marker_stop, self.id0_verified_fallback_stop)
            if id0_age is not None and id0_age > self.marker_slowdown:
                # Parent CCTV fallback applies 0.5. Pre-scale so the effective
                # verified-fallback motion is the configured conservative rate.
                pre_scale = min(
                    1.0, self.id0_verified_fallback_scale / 0.5)
                vx *= pre_scale
                vy *= pre_scale
                omega *= pre_scale
        try:
            result = super().apply_sync_and_publish(
                vx, vy, omega, now,
                mode=mode, linear_limit=linear_limit,
                angular_limit=angular_limit, extra_info=extra_info)
        finally:
            self.marker_stop = old_marker_stop
        if result:
            reference = self.reference_capture.reference
            if reference is not None:
                fused_error = self.angle_norm(
                    self.yaw_kalman.x - reference.relative_yaw)
                if abs(fused_error) <= self.yaw_limit:
                    self._yaw_visual_disagreement_since = None
        return result

    def fatal_stop(self, reason):
        if str(reason).startswith('YAW_ERROR'):
            now = time.monotonic()
            reference = self.reference_capture.reference
            raw = self._raw_wheel_relative(now)
            if reference is not None and raw is not None:
                wheel_yaw_error = self.angle_norm(
                    raw[1] - reference.relative_yaw)
                if abs(wheel_yaw_error) <= self.yaw_limit:
                    if self._yaw_visual_disagreement_since is None:
                        self._yaw_visual_disagreement_since = now
                    age = now - self._yaw_visual_disagreement_since
                    if age < self.yaw_visual_disagreement_grace:
                        self.recoverable_hold(
                            'YAW_VISUAL_DISAGREEMENT '
                            f'{math.degrees(wheel_yaw_error):+.1f}deg')
                        return
                else:
                    self._yaw_visual_disagreement_since = None
        return super().fatal_stop(reason)


def _spin(node_type: Type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def individual_move_main(args=None):
    _spin(MvpIndividualMoveNode, args)


def rigid_body_sync_main(args=None):
    _spin(MvpRigidBodySyncNode, args)
