"""Small ROS-independent policies for the real-robot MVP wrappers."""

from __future__ import annotations

import math


def stage_accepts_lift_status(state, status):
    """Accept a servo DONE only in the mission phase that issued the action."""
    state = str(state)
    status = str(status)
    return ((state == 'LIFT' and status == 'GRIP_DONE') or
            (state == 'RELEASE' and status == 'RELEASE_DONE'))


def servo_attach_pulses_from_telemetry(parsed, fallback):
    """Return live servo pulses when telemetry provides a valid pair."""
    fallback = tuple(int(value) for value in fallback)
    if len(fallback) != 2:
        raise ValueError('fallback must contain two servo pulses')
    values = parsed.get('servo_us') if isinstance(parsed, dict) else None
    if values is None or len(values) != 2:
        return fallback
    try:
        pulses = tuple(int(value) for value in values)
    except (TypeError, ValueError):
        return fallback
    if any(not 400 <= value <= 2600 for value in pulses):
        return fallback
    return pulses


def final_slot_command(*, base_command, yaw_error, yaw_tolerance,
                       yaw_kp, max_omega, max_speed, rotation_radius,
                       final_max_omega):
    """Separate final slot rotation from lateral/longitudinal insertion."""
    values = tuple(float(value) for value in base_command)
    if len(values) != 3 or not all(math.isfinite(value) for value in values):
        raise ValueError('base_command must contain three finite values')
    numeric = tuple(float(value) for value in (
        yaw_error, yaw_tolerance, yaw_kp, max_omega, max_speed,
        rotation_radius, final_max_omega))
    if not all(math.isfinite(value) for value in numeric):
        raise ValueError('final-slot values must be finite')
    yaw_error, yaw_tolerance, yaw_kp, max_omega, max_speed, \
        rotation_radius, final_max_omega = numeric
    if min(yaw_tolerance, max_omega, max_speed, rotation_radius,
           final_max_omega) <= 0.0:
        raise ValueError('final-slot limits must be positive')
    if abs(yaw_error) >= yaw_tolerance:
        omega_limit = min(
            max_omega,
            final_max_omega,
            0.8 * max_speed / max(rotation_radius, 1.0e-6),
        )
        omega = max(-omega_limit, min(omega_limit, yaw_kp * yaw_error))
        return (0.0, 0.0, omega)
    # Once yaw is accepted, slot centering/insertion is translation-only.
    return (values[0], values[1], 0.0)
