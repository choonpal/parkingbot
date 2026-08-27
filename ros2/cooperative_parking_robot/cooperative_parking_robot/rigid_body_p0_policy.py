#!/usr/bin/env python3
"""ROS-independent P0 policies for production rigid-body control."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional


NSEC_PER_SEC = 1_000_000_000


def wheel_pair_skew_s(front_stamp_ns: int, rear_stamp_ns: int) -> float:
    """Return absolute source timestamp skew for the two wheel streams."""
    front = int(front_stamp_ns)
    rear = int(rear_stamp_ns)
    if front <= 0 or rear <= 0:
        return float('inf')
    return abs(front - rear) / NSEC_PER_SEC


def wheel_pair_is_synchronized(
        front_stamp_ns: int, rear_stamp_ns: int,
        sync_slop_s: float) -> bool:
    """Return whether Front/Rear wheel poses may form one relative sample."""
    slop = float(sync_slop_s)
    if not math.isfinite(slop) or slop <= 0.0:
        raise ValueError('wheel sync slop must be finite and positive')
    return wheel_pair_skew_s(front_stamp_ns, rear_stamp_ns) <= slop


@dataclass(frozen=True)
class LateralSafetyDecision:
    """One lateral-error safety decision for the current control cycle."""

    action: str
    error_since: Optional[float]
    age_s: float
    speed_scale: float
    reason: str

    @property
    def blocking(self) -> bool:
        return self.action in ('FATAL_LIMIT', 'FATAL_TIMEOUT')


def lateral_safety_state(
        *, error_m: float, now: float, error_since: Optional[float],
        error_limit_m: float, stop_limit_m: float,
        error_timeout_s: float) -> LateralSafetyDecision:
    """Apply warning, immediate-stop and persistence limits to lateral error."""
    error = float(error_m)
    current = float(now)
    warning = float(error_limit_m)
    stop = float(stop_limit_m)
    timeout = float(error_timeout_s)
    if not all(math.isfinite(value) for value in (
            error, current, warning, stop, timeout)):
        return LateralSafetyDecision(
            'FATAL_LIMIT', error_since, 0.0, 0.0,
            'non_finite_lateral_state')
    if not 0.0 < warning < stop:
        raise ValueError('need 0 < lateral error limit < stop limit')
    if timeout <= 0.0:
        raise ValueError('lateral error timeout must be positive')

    magnitude = abs(error)
    if magnitude >= stop:
        return LateralSafetyDecision(
            'FATAL_LIMIT', error_since, 0.0, 0.0,
            'lateral_stop_limit')
    if magnitude <= warning:
        return LateralSafetyDecision('OK', None, 0.0, 1.0, 'within_limit')

    started = current if error_since is None else float(error_since)
    age = max(0.0, current - started)
    if age > timeout:
        return LateralSafetyDecision(
            'FATAL_TIMEOUT', started, age, 0.0,
            'lateral_error_timeout')
    return LateralSafetyDecision(
        'SLOW', started, age, 0.30, 'lateral_error_degraded')
