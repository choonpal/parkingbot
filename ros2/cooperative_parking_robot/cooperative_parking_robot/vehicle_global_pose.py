#!/usr/bin/env python3
"""Low-rate global vehicle position correction for rigid transport.

Front/Rear odometry propagates the transport pose at control rate.  Ceiling
CCTV/YOLO contributes only a bounded map-frame x/y bias.  The physical
vehicle-to-pair body offset captured at Lift is never rewritten by later
vision frames.
"""

from __future__ import annotations

import math
from typing import Mapping

from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics


class VehicleGlobalPoseTracker:
    """Complement pair odometry with ordered, gated vehicle x/y observations."""

    def __init__(self, *, position_gate_m=0.25, position_alpha=0.30):
        self.position_gate_m = float(position_gate_m)
        self.position_alpha = float(position_alpha)
        if (not math.isfinite(self.position_gate_m) or
                self.position_gate_m <= 0.0):
            raise ValueError('position_gate_m must be finite and positive')
        if (not math.isfinite(self.position_alpha) or
                not 0.0 < self.position_alpha <= 1.0):
            raise ValueError('position_alpha must be in (0,1]')
        self.reset()

    def reset(self):
        self.map_dx_m = 0.0
        self.map_dy_m = 0.0
        self.initialized = False
        self.last_stamp_ns = 0
        self.last_update_s = None
        self.last_residual_m = None
        self.last_decision = 'RESET'
        self.accepted_count = 0
        self.rejected_count = 0

    @staticmethod
    def _pose(pose: Mapping[str, float], label: str):
        try:
            values = (
                float(pose['x']), float(pose['y']), float(pose['theta']))
        except (KeyError, TypeError, ValueError) as exc:
            raise ValueError(f'{label} pose must contain x/y/theta') from exc
        if not all(math.isfinite(value) for value in values):
            raise ValueError(f'{label} pose values must be finite')
        return values

    @classmethod
    def raw_transport_pose(cls, front, rear):
        fx, fy, fyaw = cls._pose(front, 'front')
        rx, ry, ryaw = cls._pose(rear, 'rear')
        return (
            0.5 * (fx + rx),
            0.5 * (fy + ry),
            RigidBodyKinematics.circular_mean_yaw(fyaw, ryaw),
        )

    def transport_pose(self, front, rear):
        cx, cy, yaw = self.raw_transport_pose(front, rear)
        return cx + self.map_dx_m, cy + self.map_dy_m, yaw

    def vehicle_pose(self, front, rear, offset_body_x, offset_body_y):
        cx, cy, yaw = self.transport_pose(front, rear)
        return RigidBodyKinematics.control_point_pose(
            cx, cy, yaw, float(offset_body_x), float(offset_body_y))

    def seed(self, now_s):
        """Start freshness from the validated target-aligned initial pose."""
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError('now_s must be finite and non-negative')
        self.initialized = True
        self.last_update_s = now
        self.last_decision = 'TARGET_POSE_SEED'

    def update(self, *, measured_x_m, measured_y_m, front, rear,
               offset_body_x, offset_body_y, source_stamp_ns, now_s):
        measured_x = float(measured_x_m)
        measured_y = float(measured_y_m)
        stamp = int(source_stamp_ns)
        now = float(now_s)
        if not all(math.isfinite(value) for value in
                   (measured_x, measured_y, now)) or now < 0.0:
            raise ValueError('vehicle feedback values must be finite')
        if stamp <= 0 or stamp <= self.last_stamp_ns:
            self.last_decision = 'STALE_OR_INVALID_STAMP'
            self.rejected_count += 1
            return False
        # Consume source ordering even when the physical residual is rejected.
        self.last_stamp_ns = stamp

        predicted_x, predicted_y, _yaw = self.vehicle_pose(
            front, rear, offset_body_x, offset_body_y)
        residual_x = measured_x - predicted_x
        residual_y = measured_y - predicted_y
        residual = math.hypot(residual_x, residual_y)
        self.last_residual_m = residual
        if residual > self.position_gate_m:
            self.last_decision = 'POSITION_GATE'
            self.rejected_count += 1
            return False

        gain = self.position_alpha if self.initialized else 1.0
        self.map_dx_m += gain * residual_x
        self.map_dy_m += gain * residual_y
        self.initialized = True
        self.last_update_s = now
        self.last_decision = 'POSITION_ACCEPT'
        self.accepted_count += 1
        return True

    def age_s(self, now_s):
        if self.last_update_s is None:
            return None
        age = float(now_s) - self.last_update_s
        if not math.isfinite(age):
            raise ValueError('now_s must be finite')
        return max(0.0, age)

    def fresh(self, now_s, timeout_s):
        timeout = float(timeout_s)
        if not math.isfinite(timeout) or timeout <= 0.0:
            raise ValueError('timeout_s must be finite and positive')
        age = self.age_s(now_s)
        return age is not None and age <= timeout

    def telemetry(self, now_s):
        return {
            'vehicle_global_pose_initialized': self.initialized,
            'vehicle_global_pose_age_s': self.age_s(now_s),
            'vehicle_global_map_dx_m': round(self.map_dx_m, 5),
            'vehicle_global_map_dy_m': round(self.map_dy_m, 5),
            'vehicle_global_position_residual_m': (
                None if self.last_residual_m is None else
                round(self.last_residual_m, 5)),
            'vehicle_global_feedback_decision': self.last_decision,
            'vehicle_global_feedback_accepts': self.accepted_count,
            'vehicle_global_feedback_rejects': self.rejected_count,
        }
