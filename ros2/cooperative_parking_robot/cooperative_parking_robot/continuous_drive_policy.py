"""ROS-independent helpers for completion-first rigid-body control.

The controller still stops for explicit manual E-stop, lost command authority,
missing odometry, and the physical lateral-load limit.  Distance/yaw drift and
visual degradation instead reduce feed-forward motion while leaving relative
PID correction active.
"""

from __future__ import annotations

import math
from typing import Optional


NON_LATCHING_SYNC_PREFIXES = (
    'MARKER_',
    'ID0_',
    'CORRECTION_',
    'YAW_ERROR',
    'YAW_VISUAL_DISAGREEMENT',
    'DIST_ERROR',
    'RELATIVE_X_ERROR',
    'REFERENCE_CAPTURE_FAILED',
    'REFERENCE_RETRY_WAIT',
    'REFERENCE_NOT_READY',
)


def _validate_scale(minimum_scale: float) -> float:
    value = float(minimum_scale)
    if not 0.0 < value <= 1.0:
        raise ValueError('minimum_scale must be in (0, 1]')
    return value


def proportional_error_scale(
        error: Optional[float], threshold: float,
        minimum_scale: float) -> float:
    """Return a smooth non-zero motion scale for one control error.

    Motion remains unchanged inside ``threshold``.  Outside it, the scale falls
    as ``threshold / abs(error)`` and is bounded by ``minimum_scale``.  This is
    deliberately not a stop decision; the inner relative PID keeps correcting.
    """
    floor = _validate_scale(minimum_scale)
    limit = float(threshold)
    if not math.isfinite(limit) or limit <= 0.0:
        raise ValueError('threshold must be finite and positive')
    if error is None:
        return 1.0
    value = float(error)
    if not math.isfinite(value):
        return floor
    magnitude = abs(value)
    if magnitude <= limit:
        return 1.0
    return max(floor, min(1.0, limit / magnitude))


def visual_loss_scale(
        age_s: Optional[float], slowdown_s: float, full_degrade_s: float,
        minimum_scale: float) -> float:
    """Gradually degrade but never stop when ID0 observations are unavailable."""
    floor = _validate_scale(minimum_scale)
    slow = float(slowdown_s)
    full = float(full_degrade_s)
    if not all(math.isfinite(value) for value in (slow, full)):
        raise ValueError('visual timing limits must be finite')
    if slow < 0.0 or full <= slow:
        raise ValueError('need 0 <= slowdown_s < full_degrade_s')
    if age_s is None:
        return floor
    age = float(age_s)
    if not math.isfinite(age) or age < 0.0:
        return floor
    if age <= slow:
        return 1.0
    if age >= full:
        return floor
    progress = (age - slow) / (full - slow)
    return 1.0 - progress * (1.0 - floor)


def is_non_latching_sync_reason(reason: str) -> bool:
    """True for sensor/control degradation that must not abort the mission."""
    value = str(reason or '').strip().upper()
    if value.startswith('LATERAL_ERROR'):
        return False
    return value.startswith(NON_LATCHING_SYNC_PREFIXES)
