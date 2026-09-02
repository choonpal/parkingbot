#!/usr/bin/env python3
"""Production rigid-body controller with Q/E-style final-slot rotation.

The main parking sequence remains:

    path to external staging -> rotate to slot yaw -> straight insertion

Only ``ALIGN_SLOT_YAW`` is changed. During that phase, the newest ID0 lateral
measurement accepted by the existing innovation gate is used as pair-rotation
phase feedback instead of being fed to the opposing lateral PID that can fight
the intended orbital velocities.
"""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot.mvp_runtime_nodes import MvpRigidBodySyncNode
from cooperative_parking_robot.rotation_phase_control import (
    PhaseAwareRigidBodyKinematics,
    RotationPhaseController,
    RotationPhaseTelemetry,
    is_final_rotation_command,
)


class RigidBodySyncNode(MvpRigidBodySyncNode):
    """Keep main's rotate-then-insert mission with ID0 phase rotation control."""

    def __init__(self):
        # Base constructors may call virtual stop helpers. Keep reset state
        # valid before entering the inherited production stack.
        self.rotation_phase_controller = None
        self._rotation_phase_requested = False
        self._rotation_phase_now = 0.0
        self._rotation_phase_last_measurement_time = None
        self._rotation_phase_last_sample_token = None
        self._rotation_phase_last_lateral_error = 0.0
        self._rotation_phase_telemetry = RotationPhaseTelemetry(
            False, False, 0.0, 0.0, None)

        super().__init__()

        self.declare_parameter('rotation_phase_kp', 1.8)
        self.declare_parameter('rotation_phase_deadband_deg', 0.5)
        self.declare_parameter('rotation_phase_correction_limit_rps', 0.06)
        self.declare_parameter(
            'rotation_phase_correction_rate_limit_rps2', 0.30)
        self.declare_parameter('rotation_phase_lateral_lpf_alpha', 0.65)

        gp = self.get_parameter
        self.rotation_phase_controller = RotationPhaseController(
            kp=float(gp('rotation_phase_kp').value),
            deadband_rad=math.radians(float(
                gp('rotation_phase_deadband_deg').value)),
            correction_limit_rps=float(
                gp('rotation_phase_correction_limit_rps').value),
            correction_rate_limit_rps2=float(
                gp('rotation_phase_correction_rate_limit_rps2').value),
            lateral_lpf_alpha=float(
                gp('rotation_phase_lateral_lpf_alpha').value),
        )

        # All inherited motion remains equivalent while this object is
        # inactive. The flag is enabled only for ALIGN_SLOT_YAW.
        self.kinematics = PhaseAwareRigidBodyKinematics(self.wheelbase)

        self.get_logger().info(
            'final rotation control | pair-centre split + ID0 phase common yaw '
            '| rotate-then-insert retained')

    def _reset_rotation_phase(self) -> None:
        self._rotation_phase_requested = False
        controller = getattr(self, 'rotation_phase_controller', None)
        if controller is not None:
            controller.reset()
        kinematics = getattr(self, 'kinematics', None)
        if isinstance(kinematics, PhaseAwareRigidBodyKinematics):
            kinematics.clear_rotation_phase()
        self._rotation_phase_last_measurement_time = None
        self._rotation_phase_last_sample_token = None
        self._rotation_phase_last_lateral_error = 0.0
        self._rotation_phase_telemetry = RotationPhaseTelemetry(
            False, False, 0.0, 0.0, None)

    def path_cb(self, msg):
        self._reset_rotation_phase()
        return super().path_cb(msg)

    def vehicle_lifted_cb(self, msg):
        if not bool(msg.data):
            self._reset_rotation_phase()
        return super().vehicle_lifted_cb(msg)

    def send_stop(self):
        self._reset_rotation_phase()
        return super().send_stop()

    def compute_final_command(self, cx, cy, ct):
        done, command, info = super().compute_final_command(cx, cy, ct)
        phase = info.get('final_phase') if isinstance(info, dict) else None
        active = (
            not done and self.align_to_slot_yaw and
            is_final_rotation_command(
                mode='FINAL_APPROACH', final_phase=phase,
                command=command)
        )
        if not active and self._rotation_phase_requested:
            self._reset_rotation_phase()
        self._rotation_phase_requested = active
        if isinstance(info, dict):
            info['rotation_control'] = (
                'PAIR_CENTRE_ID0_PHASE' if active else
                'TRANSLATION_INSERTION')
        return done, command, info

    def _accepted_id0_phase_sample(self):
        """Cache only lateral samples accepted by the parent innovation gate."""
        reference = self.reference_capture.reference
        correction_time = self._last_correction_time.get('lateral')
        accepted = self._last_gate_decision.get('lateral') in {
            'ACCEPT', 'REACQUIRE'}
        new_accepted_sample = (
            reference is not None and accepted and
            correction_time is not None and
            correction_time != self._rotation_phase_last_measurement_time and
            self.aruco_lateral is not None and
            math.isfinite(float(self.aruco_lateral)))
        if new_accepted_sample:
            self._rotation_phase_last_measurement_time = correction_time
            self._rotation_phase_last_sample_token = self.aruco_stamp_ns
            self._rotation_phase_last_lateral_error = (
                float(self.aruco_lateral) - reference.relative_y)

        age = (
            None if self._rotation_phase_last_measurement_time is None else
            self._rotation_phase_now -
            self._rotation_phase_last_measurement_time)
        fresh = bool(
            age is not None and 0.0 <= age <= self.aruco_timeout)
        return (
            fresh,
            self._rotation_phase_last_sample_token,
            self._rotation_phase_last_lateral_error,
        )

    def _phase_common_yaw_correction(self):
        """Run after the parent has consumed and gated the current ID0 frame."""
        reference = self.reference_capture.reference
        controller = self.rotation_phase_controller
        if reference is None or controller is None:
            self._rotation_phase_telemetry = RotationPhaseTelemetry(
                False, False, 0.0, 0.0, None)
            return 0.0

        fresh, token, lateral_error = self._accepted_id0_phase_sample()
        gap_error = self.relative_x_kalman.x - reference.relative_x
        telemetry = controller.update(
            active=True,
            measurement_fresh=fresh,
            sample_token=token,
            separation_m=self.wheelbase,
            gap_error_m=gap_error,
            lateral_error_m=lateral_error,
            now_s=self._rotation_phase_now,
        )
        self._rotation_phase_telemetry = telemetry
        return telemetry.correction_rps

    def apply_sync_and_publish(self, vx, vy, omega, now, *, mode,
                               linear_limit, angular_limit, extra_info=None):
        final_phase = (
            extra_info.get('final_phase')
            if isinstance(extra_info, dict) else None)
        active = (
            self._rotation_phase_requested and
            is_final_rotation_command(
                mode=mode, final_phase=final_phase,
                command=(vx, vy, omega))
        )
        self._rotation_phase_now = float(now)
        controller = self.rotation_phase_controller
        self._rotation_phase_telemetry = RotationPhaseTelemetry(
            active, False, 0.0,
            controller.correction_rps
            if active and controller is not None else 0.0,
            controller.filtered_lateral_error_m
            if active and controller is not None else None)

        if active and controller is not None:
            self.kinematics.configure_rotation_phase(
                active=True,
                common_yaw_provider=self._phase_common_yaw_correction)
            # An ignored lateral PID output must not accumulate and then be
            # released into the straight insertion phase.
            self.lateral_pid.reset()
        else:
            if controller is not None:
                controller.reset()
            self.kinematics.clear_rotation_phase()

        merged_info = dict(extra_info or {})
        merged_info['rotation_control'] = (
            'PAIR_CENTRE_ID0_PHASE' if active else 'LEGACY')

        suppressed = 0.0
        common_yaw = 0.0
        try:
            result = super().apply_sync_and_publish(
                vx, vy, omega, now,
                mode=mode,
                linear_limit=linear_limit,
                angular_limit=angular_limit,
                extra_info=merged_info)
            suppressed = (
                self.kinematics.last_suppressed_lateral_correction_mps)
            common_yaw = self.kinematics.last_common_yaw_correction_rps
        finally:
            self.kinematics.clear_rotation_phase()
            if active:
                self.lateral_pid.reset()

        telemetry = self._rotation_phase_telemetry
        phase_info = {
            'rotation_control': (
                'PAIR_CENTRE_ID0_PHASE' if active else 'LEGACY'),
            'rotation_phase_feedback_fresh': telemetry.measurement_fresh,
            'rotation_phase_error_deg': round(
                math.degrees(telemetry.phase_error_rad), 3),
            'rotation_phase_correction_rps': round(common_yaw, 5),
            'rotation_phase_lateral_error_m': (
                None if telemetry.filtered_lateral_error_m is None else
                round(telemetry.filtered_lateral_error_m, 5)),
            'rotation_lateral_pid_suppressed': active,
            'rotation_suppressed_lateral_correction_mps': round(
                suppressed, 5),
        }
        if isinstance(self._info, dict):
            self._info.update(phase_info)
        return result


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
