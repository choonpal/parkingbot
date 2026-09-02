#!/usr/bin/env python3
"""Q/E-style phase feedback for the automatic final slot rotation.

The final parking sequence still rotates at the external staging point and then
inserts along the slot axis. Only the rotation synchronizer changes:

* the pair-centre angular command is split symmetrically;
* relative lateral feedback is not applied as opposing orbital velocity while
  the pair is rotating;
* ID0 lateral displacement is interpreted as rotation phase and adds one
  bounded common-yaw correction to both robots;
* Front/Rear relative-yaw correction remains equal and opposite.

This module has no ROS dependency so the geometry and controller can be unit
-tested on the normal Python CI runner.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Callable, Hashable

from cooperative_parking_robot.rigid_body_kinematics import RigidBodyKinematics


def clamp(value: float, limit: float) -> float:
    """Clamp ``value`` symmetrically to a finite non-negative limit."""
    value = float(value)
    limit = float(limit)
    if not math.isfinite(value) or not math.isfinite(limit) or limit < 0.0:
        raise ValueError('clamp values must be finite and limit non-negative')
    return max(-limit, min(limit, value))


def smooth_deadband_correction(
        error: float, *, kp: float, deadband: float, limit: float) -> float:
    """Return a continuous bounded correction outside a symmetric deadband."""
    error = float(error)
    kp = float(kp)
    deadband = float(deadband)
    limit = float(limit)
    values = (error, kp, deadband, limit)
    if (not all(math.isfinite(value) for value in values) or
            kp <= 0.0 or deadband <= 0.0 or limit <= 0.0):
        raise ValueError('phase correction inputs must be finite and positive')
    excess = abs(error) - deadband
    if excess <= 0.0:
        return 0.0
    return math.copysign(min(kp * excess, limit), error)


def rotation_phase_error(
        separation_m: float, gap_error_m: float,
        lateral_error_m: float) -> float:
    """Return the ID0-derived angular phase error around the pair centre.

    ``gap_error_m`` is relative to the captured Lift-session separation. The
    half-separation floor prevents one bad distance estimate from amplifying
    lateral noise into an excessive angular correction.
    """
    separation = float(separation_m)
    gap_error = float(gap_error_m)
    lateral_error = float(lateral_error_m)
    values = (separation, gap_error, lateral_error)
    if (not all(math.isfinite(value) for value in values) or
            separation <= 0.0):
        raise ValueError('phase geometry must be finite and separation positive')
    forward = max(0.5 * separation, separation + gap_error)
    return math.atan2(lateral_error, forward)


def is_final_rotation_command(
        *, mode: str, final_phase: str | None, command,
        tolerance: float = 1.0e-9) -> bool:
    """Return true only for the rotate-before-insert final phase."""
    values = tuple(float(value) for value in command)
    tolerance = abs(float(tolerance))
    if (len(values) != 3 or
            not all(math.isfinite(value) for value in values) or
            not math.isfinite(tolerance)):
        raise ValueError('final rotation command must contain three finite axes')
    return (
        str(mode) == 'FINAL_APPROACH' and
        str(final_phase) == 'ALIGN_SLOT_YAW' and
        math.hypot(values[0], values[1]) <= tolerance and
        abs(values[2]) > tolerance
    )


@dataclass(frozen=True)
class RotationPhaseTelemetry:
    active: bool
    measurement_fresh: bool
    phase_error_rad: float
    correction_rps: float
    filtered_lateral_error_m: float | None


class RotationPhaseController:
    """Stateful deadbanded, low-pass and slew-limited common-yaw controller."""

    def __init__(
            self, *, kp: float = 1.8,
            deadband_rad: float = math.radians(0.5),
            correction_limit_rps: float = 0.06,
            correction_rate_limit_rps2: float = 0.30,
            lateral_lpf_alpha: float = 0.65):
        self.kp = float(kp)
        self.deadband_rad = float(deadband_rad)
        self.correction_limit_rps = float(correction_limit_rps)
        self.correction_rate_limit_rps2 = float(
            correction_rate_limit_rps2)
        self.lateral_lpf_alpha = float(lateral_lpf_alpha)
        if (not all(math.isfinite(value) and value > 0.0 for value in (
                self.kp, self.deadband_rad, self.correction_limit_rps,
                self.correction_rate_limit_rps2)) or
                not math.isfinite(self.lateral_lpf_alpha) or
                not 0.0 < self.lateral_lpf_alpha <= 1.0):
            raise ValueError('invalid rotation phase controller parameters')
        self.reset()

    def reset(self) -> None:
        self.filtered_lateral_error_m = None
        self.correction_rps = 0.0
        self.last_update_s = None
        self.last_sample_token: Hashable | None = None

    def update(
            self, *, active: bool, measurement_fresh: bool,
            sample_token: Hashable | None, separation_m: float,
            gap_error_m: float, lateral_error_m: float,
            now_s: float) -> RotationPhaseTelemetry:
        now = float(now_s)
        if not math.isfinite(now) or now < 0.0:
            raise ValueError(
                'phase controller time must be finite and non-negative')
        if not active:
            self.reset()
            return RotationPhaseTelemetry(False, False, 0.0, 0.0, None)

        dt = 0.02 if self.last_update_s is None else min(
            0.10, max(0.001, now - self.last_update_s))
        self.last_update_s = now

        lateral_error = float(lateral_error_m)
        if not math.isfinite(lateral_error):
            measurement_fresh = False
        if measurement_fresh and (
                self.filtered_lateral_error_m is None or
                sample_token != self.last_sample_token):
            if self.filtered_lateral_error_m is None:
                self.filtered_lateral_error_m = lateral_error
            else:
                alpha = self.lateral_lpf_alpha
                self.filtered_lateral_error_m = (
                    alpha * lateral_error +
                    (1.0 - alpha) * self.filtered_lateral_error_m)
            self.last_sample_token = sample_token

        phase_error = 0.0
        target = 0.0
        if measurement_fresh and self.filtered_lateral_error_m is not None:
            phase_error = rotation_phase_error(
                separation_m, gap_error_m,
                self.filtered_lateral_error_m)
            target = smooth_deadband_correction(
                phase_error, kp=self.kp,
                deadband=self.deadband_rad,
                limit=self.correction_limit_rps)

        max_step = self.correction_rate_limit_rps2 * dt
        self.correction_rps += clamp(
            target - self.correction_rps, max_step)
        self.correction_rps = clamp(
            self.correction_rps, self.correction_limit_rps)
        if abs(self.correction_rps) < 1.0e-12:
            self.correction_rps = 0.0

        return RotationPhaseTelemetry(
            active=True,
            measurement_fresh=bool(measurement_fresh),
            phase_error_rad=phase_error,
            correction_rps=self.correction_rps,
            filtered_lateral_error_m=self.filtered_lateral_error_m,
        )


class PhaseAwareRigidBodyKinematics(RigidBodyKinematics):
    """Preserve legacy kinematics except during Q/E-style pair rotation."""

    def __init__(self, wheelbase=0.70):
        super().__init__(wheelbase)
        self.rotation_phase_active = False
        self._common_yaw_provider: Callable[[], float] | None = None
        self.last_common_yaw_correction_rps = 0.0
        self.last_suppressed_lateral_correction_mps = 0.0

    def configure_rotation_phase(
            self, *, active: bool,
            common_yaw_provider: Callable[[], float] | None = None) -> None:
        if active and common_yaw_provider is None:
            raise ValueError('active rotation phase requires a correction provider')
        if common_yaw_provider is not None and not callable(common_yaw_provider):
            raise ValueError('common yaw provider must be callable')
        self.rotation_phase_active = bool(active)
        self._common_yaw_provider = (
            common_yaw_provider if self.rotation_phase_active else None)
        self.last_common_yaw_correction_rps = 0.0
        self.last_suppressed_lateral_correction_mps = 0.0

    def clear_rotation_phase(self) -> None:
        self.configure_rotation_phase(active=False)

    def apply_relative_correction(
            self, front_velocity, rear_velocity,
            corr_x, corr_y, corr_yaw):
        applied_corr_y = 0.0 if self.rotation_phase_active else corr_y
        front, rear = RigidBodyKinematics.apply_relative_correction(
            front_velocity, rear_velocity,
            corr_x, applied_corr_y, corr_yaw)
        if not self.rotation_phase_active:
            self.last_common_yaw_correction_rps = 0.0
            self.last_suppressed_lateral_correction_mps = 0.0
            return front, rear

        phase = float(self._common_yaw_provider())
        if not math.isfinite(phase):
            raise ValueError('common yaw provider returned a non-finite value')
        self.last_common_yaw_correction_rps = phase
        self.last_suppressed_lateral_correction_mps = float(corr_y)
        return (
            (front[0], front[1], front[2] + phase),
            (rear[0], rear[1], rear[2] + phase),
        )
