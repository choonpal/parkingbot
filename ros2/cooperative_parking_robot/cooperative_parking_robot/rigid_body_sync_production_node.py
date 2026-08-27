#!/usr/bin/env python3
"""Production P0 guard layer for mission-reference rigid-body control.

This entry point deliberately subclasses the mission-reference controller so a
small, reviewable layer can enforce the final real-robot P0 contracts:

* waypoint publication/replanning never clears a Lift-session reference;
* Front/Rear wheel poses are paired only within a bounded source-time skew;
* a previously synchronized wheel relative pose is held rather than combining
  asynchronous latest samples;
* lateral error has slowdown, persistence and immediate-stop limits;
* zero/deadbanded PID errors cannot retain integral output.
"""

from __future__ import annotations

import rclpy

from cooperative_parking_robot.pid_controller import PID
from cooperative_parking_robot.rigid_body_p0_policy import (
    lateral_safety_state,
    wheel_pair_is_synchronized,
    wheel_pair_skew_s,
)
from cooperative_parking_robot.rigid_body_sync_node import (
    RigidBodySyncNode as LegacyRigidBodySyncNode,
)
from cooperative_parking_robot.rigid_body_sync_safe_node import (
    RigidBodySyncNode as MissionReferenceRigidBodySyncNode,
)


class RigidBodySyncNode(MissionReferenceRigidBodySyncNode):
    """Mission-reference controller with the final P0 lifecycle/safety guards."""

    def __init__(self):
        super().__init__()

        self.declare_parameter('wheel_pair_sync_slop_s', 0.05)
        self.declare_parameter('sync_lateral_error_limit_m', 0.020)
        self.declare_parameter('sync_lateral_stop_limit_m', 0.040)
        self.declare_parameter('sync_lateral_error_timeout_s', 1.0)

        gp = self.get_parameter
        self.wheel_pair_sync_slop = float(
            gp('wheel_pair_sync_slop_s').value)
        self.lateral_error_limit = float(
            gp('sync_lateral_error_limit_m').value)
        self.lateral_stop_limit = float(
            gp('sync_lateral_stop_limit_m').value)
        self.lateral_error_timeout = float(
            gp('sync_lateral_error_timeout_s').value)
        if self.wheel_pair_sync_slop <= 0.0:
            raise ValueError('wheel_pair_sync_slop_s must be positive')
        if not 0.0 < self.lateral_error_limit < self.lateral_stop_limit:
            raise ValueError(
                'need 0 < sync_lateral_error_limit_m < '
                'sync_lateral_stop_limit_m')
        if self.lateral_error_timeout <= 0.0:
            raise ValueError('sync_lateral_error_timeout_s must be positive')

        # Rebuild after all launch/YAML overrides have been applied. Production
        # config starts P-only with a low correction cap; this also guarantees
        # a direct ``ros2 run`` uses the parameter values visible at runtime.
        self.lateral_pid = PID(
            float(gp('sync_lateral_kp').value),
            float(gp('sync_lateral_ki').value),
            float(gp('sync_lateral_kd').value),
            out_limit=float(
                gp('sync_lateral_max_correction_mps').value))

        self.lateral_error_since = None
        self._last_wheel_pair_skew_s = None
        self._last_synced_raw_relative = None
        self._last_lateral_safety = 'OK'

        self.get_logger().info(
            'production P0 guards active | '
            f'wheel_pair_slop={self.wheel_pair_sync_slop * 1000.0:.0f}ms | '
            f'lateral warn/stop={self.lateral_error_limit * 1000.0:.0f}/'
            f'{self.lateral_stop_limit * 1000.0:.0f}mm')

    def path_cb(self, msg):
        """Accept/replan a path without clearing the current Lift reference.

        Fleet publishes the path *after* Lift. Calling the parent safe-node
        callback would reset ``reference_capture`` to WAIT_LIFT and permanently
        block DRIVE because no second Lift rising edge occurs. The legacy path
        setup is invoked directly, while the locked/in-progress reference is
        intentionally preserved for the whole Lift session.
        """
        LegacyRigidBodySyncNode.path_cb(self, msg)
        self.lateral_pid.reset()
        self.lateral_error_since = None

    def vehicle_lifted_cb(self, msg):
        super().vehicle_lifted_cb(msg)
        if not self.vehicle_lifted:
            self.lateral_error_since = None
            self._last_synced_raw_relative = None

    def _raw_wheel_relative(self, now):
        """Use only a synchronized Front/Rear wheel pose pair.

        If one WiFi stream is temporarily ahead, hold the last synchronized
        relative sample. Its unchanged predictor stamp means DeltaKalman1D does
        not grow covariance or invent motion from cached data.
        """
        if not self._wheel_predictor_initialized:
            return None
        if any(
                now - self._wheel_receipt_time[role] >
                self.wheel_relative_timeout
                for role in ('front', 'rear')):
            return None

        front_stamp = self._wheel_stamp_ns['front']
        rear_stamp = self._wheel_stamp_ns['rear']
        self._last_wheel_pair_skew_s = wheel_pair_skew_s(
            front_stamp, rear_stamp)
        if not wheel_pair_is_synchronized(
                front_stamp, rear_stamp, self.wheel_pair_sync_slop):
            return self._last_synced_raw_relative

        raw = super()._raw_wheel_relative(now)
        if raw is not None:
            self._last_synced_raw_relative = raw
        return raw

    def _largest_current_lateral_error(self, now):
        reference = self.reference_capture.reference
        if reference is None:
            return None
        errors = [self.lateral_kalman.x - reference.relative_y]

        raw = self._raw_wheel_relative(now)
        if raw is not None:
            errors.append(raw[4] - reference.relative_y)

        # ID0 is first passed through its innovation gate in the parent
        # estimator. Never stop directly on one unvalidated solvePnP frame.
        return max(errors, key=abs)

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        lateral_error = self._largest_current_lateral_error(now)
        decision = None
        if lateral_error is not None:
            decision = lateral_safety_state(
                error_m=lateral_error,
                now=now,
                error_since=self.lateral_error_since,
                error_limit_m=self.lateral_error_limit,
                stop_limit_m=self.lateral_stop_limit,
                error_timeout_s=self.lateral_error_timeout)
            self.lateral_error_since = decision.error_since
            self._last_lateral_safety = decision.action

            # The parent deadband passes an exact zero to this PID. Resetting
            # here, plus PID's zero-error behavior, prevents latent I output.
            if abs(lateral_error) <= self.lateral_deadband:
                self.lateral_pid.reset()

            if decision.blocking:
                label = ('LATERAL_ERROR_FATAL'
                         if decision.action == 'FATAL_LIMIT'
                         else 'LATERAL_ERROR_TIMEOUT')
                self.fatal_stop(
                    f'{label} {lateral_error * 1000.0:+.0f}mm')
                return False
            if decision.action == 'SLOW':
                # Slow the requested rigid-body motion while retaining full
                # relative-correction authority inside the parent controller.
                vx *= decision.speed_scale
                vy *= decision.speed_scale
                omega *= decision.speed_scale

        result = super().apply_sync_and_publish(
            vx, vy, omega, now,
            mode=mode,
            linear_limit=linear_limit,
            angular_limit=angular_limit,
            extra_info=extra_info)
        if isinstance(self._info, dict):
            self._info.update({
                'wheel_pair_skew_s': self._last_wheel_pair_skew_s,
                'wheel_pair_synchronized': (
                    self._last_wheel_pair_skew_s is not None and
                    self._last_wheel_pair_skew_s <=
                    self.wheel_pair_sync_slop),
                'lateral_safety_state': self._last_lateral_safety,
                'lateral_error_since': self.lateral_error_since,
            })
        if (result and decision is not None and
                decision.action == 'SLOW'):
            self._err = (
                f'LATERAL_ERROR {lateral_error * 1000.0:+.0f}mm')
        return result

    def _reference_telemetry(self, now):
        info = super()._reference_telemetry(now)
        info.update({
            'wheel_pair_skew_s': self._last_wheel_pair_skew_s,
            'wheel_pair_sync_slop_s': self.wheel_pair_sync_slop,
            'lateral_safety_state': self._last_lateral_safety,
        })
        return info


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
