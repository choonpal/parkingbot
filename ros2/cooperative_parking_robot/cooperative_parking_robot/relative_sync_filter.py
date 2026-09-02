#!/usr/bin/env python3
"""Relative Front/Rear state helpers used by the rigid-body sync controller.

The helpers are ROS-independent so the estimator math can be regression tested
without a running ROS graph.
"""

from __future__ import annotations

from dataclasses import dataclass
import math
import statistics
from typing import Optional, Tuple


def normalize_angle(angle: float) -> float:
    """Return ``angle`` wrapped to [-pi, pi]."""
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def relative_pose_step_is_plausible(
        previous, candidate, *, forward_step_m=0.020,
        lateral_step_m=0.020, yaw_step_rad=math.radians(3.0)):
    """Reject relative-pose jumps faster than the bounded pair can produce."""
    old = tuple(float(value) for value in previous)
    new = tuple(float(value) for value in candidate)
    limits = (float(forward_step_m), float(lateral_step_m),
              float(yaw_step_rad))
    if (len(old) != 3 or len(new) != 3 or
            not all(math.isfinite(value) for value in old + new + limits) or
            not all(value > 0.0 for value in limits)):
        raise ValueError('relative pose values/step limits must be finite')
    return (
        abs(new[0] - old[0]) <= limits[0] and
        abs(new[1] - old[1]) <= limits[1] and
        abs(normalize_angle(new[2] - old[2])) <= limits[2])


def stream_is_healthy(age_s: Optional[float], timeout_s: float) -> bool:
    """Return whether a locally timed sensor stream is currently healthy."""
    return (age_s is not None and math.isfinite(age_s) and
            0.0 <= age_s < float(timeout_s))


def cctv_fallback_allowed(id0_age_s: Optional[float],
                          id0_timeout_s: float) -> bool:
    """CCTV may correct internal relative pose only after actual ID0 staleness."""
    return not stream_is_healthy(id0_age_s, id0_timeout_s)


def reference_blocks_drive(lifted: bool, front_state: str, rear_state: str,
                           reference_ready: bool) -> bool:
    """Return whether production DRIVE must output stop pending reference."""
    return (bool(lifted) and front_state == 'DRIVE' and
            rear_state == 'DRIVE' and not bool(reference_ready))


class CctvPairStampGate:
    """Accept each tightly synchronized CCTV marker pair exactly once."""

    def __init__(self, sync_slop_s: float):
        self.sync_slop_s = float(sync_slop_s)
        if self.sync_slop_s <= 0.0:
            raise ValueError('CCTV pair sync slop must be positive')
        self.reset()

    def reset(self) -> None:
        self.last_used = {'front': 0, 'rear': 0}

    def accept(self, front_stamp_ns: int, rear_stamp_ns: int) -> bool:
        front = int(front_stamp_ns)
        rear = int(rear_stamp_ns)
        if front <= self.last_used['front'] or rear <= self.last_used['rear']:
            return False
        if abs(front - rear) * 1.0e-9 > self.sync_slop_s:
            return False
        self.last_used = {'front': front, 'rear': rear}
        return True


def visual_safety_state(*, now: float, marker_lost_since: Optional[float],
                        correction_times: dict, slowdown_s: float,
                        stop_s: float, correction_grace_s: float = 0.0):
    """Classify visual loss separately from per-axis correction staleness."""
    if marker_lost_since is not None:
        age = max(0.0, now - marker_lost_since)
        return ('MARKER_HOLD' if age > stop_s else
                'MARKER_SLOW' if age > slowdown_s else 'MARKER_GRACE',
                age, ())
    stale = tuple(
        axis for axis, stamp in correction_times.items()
        if stamp is None or now - stamp > correction_grace_s)
    if not stale:
        return 'OK', 0.0, ()
    ages = [stop_s + 1.0 if correction_times[axis] is None else
            max(0.0, now - correction_times[axis] - correction_grace_s)
            for axis in stale]
    age = max(ages)
    return ('CORRECTION_HOLD' if age > stop_s else 'CORRECTION_STALE',
            age, stale)


@dataclass(frozen=True)
class MissionReference:
    """One mission's locked relative x/y/yaw target and sample dispersion."""

    relative_x: Optional[float]
    relative_y: float
    relative_yaw: float
    std_x: float
    std_y: float
    std_yaw: float
    sample_count: int


class MissionReferenceCapture:
    """Collect stable ID0 samples and lock one bounded mission reference."""

    def __init__(self, *, sample_count: int, timeout_s: float,
                 nominal_x: float, nominal_y: float, nominal_yaw: float,
                 max_x_error: float, max_y_error: float,
                 max_yaw_error: float, max_std_x: float,
                 max_std_y: float, max_std_yaw: float,
                 max_retries: int = 2, retry_delay_s: float = 0.3):
        self.sample_limit = int(sample_count)
        self.timeout_s = float(timeout_s)
        self.nominal = (float(nominal_x), float(nominal_y),
                        normalize_angle(nominal_yaw))
        self.max_error = (float(max_x_error), float(max_y_error),
                          float(max_yaw_error))
        self.max_std = (float(max_std_x), float(max_std_y),
                        float(max_std_yaw))
        self.max_retries = int(max_retries)
        self.retry_delay_s = float(retry_delay_s)
        if self.sample_limit < 3:
            raise ValueError('reference sample_count must be at least 3')
        if self.timeout_s <= 0.0 or self.max_retries < 0 or self.retry_delay_s < 0.0 or any(
                value <= 0.0 for value in (*self.max_error, *self.max_std)):
            raise ValueError('reference capture limits must be positive')
        self.reset()

    def reset(self, start_time: Optional[float] = None) -> None:
        self.state = 'WAIT_LIFT' if start_time is None else 'REFERENCE_CAPTURE'
        self.started_at = start_time
        self.samples = []
        self.reference: Optional[MissionReference] = None
        self.reason = 'waiting_for_lift' if start_time is None else 'collecting'
        self.retry_count = 0
        self.retry_at = None

    @property
    def ready(self) -> bool:
        return self.state == 'REFERENCE_READY' and self.reference is not None

    def _schedule_retry(self, now: float, reason: str) -> None:
        self.samples = []
        self.retry_count += 1
        self.reason = reason
        if self.retry_count > self.max_retries:
            self.state = 'REFERENCE_FAILED'
            self.retry_at = None
            return
        self.state = 'REFERENCE_RETRY_WAIT'
        self.retry_at = float(now) + self.retry_delay_s

    def advance(self, now: float) -> str:
        """Advance bounded timeout/retry state and return the current state."""
        now = float(now)
        if (self.state == 'REFERENCE_CAPTURE' and self.started_at is not None
                and now - self.started_at > self.timeout_s):
            self._schedule_retry(now, 'insufficient_valid_id0_samples')
        elif (self.state == 'REFERENCE_RETRY_WAIT' and
              self.retry_at is not None and now >= self.retry_at):
            self.state = 'REFERENCE_CAPTURE'
            self.started_at = now
            self.retry_at = None
            self.reason = 'collecting'
        return self.state

    def timed_out(self, now: float) -> bool:
        """Compatibility wrapper; True when this call leaves capture state."""
        if self.state != 'REFERENCE_CAPTURE' or self.started_at is None:
            return False
        previous = self.state
        self.advance(now)
        return self.state != previous

    @staticmethod
    def _yaw_median_and_std(values):
        base = values[0]
        unwrapped = [base + normalize_angle(value - base) for value in values]
        median = normalize_angle(statistics.median(unwrapped))
        variance = sum(
            normalize_angle(value - median) ** 2 for value in values
        ) / len(values)
        return median, math.sqrt(variance)

    def add(self, relative_x: Optional[float], relative_y: float,
            relative_yaw: float, now: Optional[float] = None) -> bool:
        """Add one valid unique ID0 observation; return True when locked."""
        if self.state != 'REFERENCE_CAPTURE':
            return False
        x = None if relative_x is None else float(relative_x)
        values = (x, float(relative_y), normalize_angle(relative_yaw))
        if ((x is not None and not math.isfinite(x)) or
                not all(math.isfinite(value) for value in values[1:])):
            return False
        self.samples.append(values)
        if len(self.samples) < self.sample_limit:
            return False

        xs, ys, yaws = zip(*self.samples[-self.sample_limit:])
        x_enabled = all(value is not None for value in xs)
        median_x = statistics.median(xs) if x_enabled else None
        median_y = statistics.median(ys)
        median_yaw, std_yaw = self._yaw_median_and_std(yaws)
        std_x = statistics.pstdev(xs) if x_enabled else None
        std_y = statistics.pstdev(ys)
        errors = (
            0.0 if median_x is None else abs(median_x - self.nominal[0]),
            abs(median_y - self.nominal[1]),
            abs(normalize_angle(median_yaw - self.nominal[2])))
        stds = ((0.0 if std_x is None else std_x), std_y, std_yaw)
        if any(value > limit for value, limit in zip(errors, self.max_error)):
            self._schedule_retry(
                self.started_at if now is None else now,
                'nominal_sanity_envelope')
            return False
        if any(value > limit for value, limit in zip(stds, self.max_std)):
            self._schedule_retry(
                self.started_at if now is None else now, 'sample_dispersion')
            return False
        self.reference = MissionReference(
            median_x, median_y, median_yaw,
            std_x, std_y, std_yaw, self.sample_limit)
        self.state = 'REFERENCE_READY'
        self.reason = 'locked'
        return True


class DeltaKalman1D:
    """One-dimensional delta-propagated Kalman filter.

    ``raw_value`` is an external dead-reckoning absolute value. Only its delta
    is propagated, so a visual correction is not overwritten on the next
    predict. Process uncertainty grows with elapsed time and motion instead of
    with the controller-loop call count.
    """

    def __init__(
            self,
            init: float = 0.0,
            *,
            measurement_variance: float = 0.0004,
            process_variance_rate: float = 1.0e-5,
            process_gain: float = 0.0,
            angle: bool = False):
        self.x = float(init)
        self.R = float(measurement_variance)
        self.process_variance_rate = float(process_variance_rate)
        self.process_gain = float(process_gain)
        self.angle = bool(angle)
        if self.R <= 0.0:
            raise ValueError('measurement_variance must be positive')
        if self.process_variance_rate < 0.0 or self.process_gain < 0.0:
            raise ValueError('process noise values must be non-negative')
        self.P = max(4.0 * self.R, 1.0e-9)
        self._prev_raw: Optional[float] = None
        self._prev_stamp_s: Optional[float] = None

    def _difference(self, lhs: float, rhs: float) -> float:
        value = float(lhs) - float(rhs)
        return normalize_angle(value) if self.angle else value

    def _normalize_state(self) -> None:
        if self.angle:
            self.x = normalize_angle(self.x)

    def reset(
            self,
            value: float = 0.0,
            raw_value: Optional[float] = None,
            covariance: Optional[float] = None,
            stamp_s: Optional[float] = None) -> None:
        self.x = float(value)
        self._normalize_state()
        self.P = float(
            max(4.0 * self.R, 1.0e-9)
            if covariance is None else covariance)
        if self.P <= 0.0 or not math.isfinite(self.P):
            raise ValueError('covariance must be finite and positive')
        raw = self.x if raw_value is None else float(raw_value)
        self._prev_raw = normalize_angle(raw) if self.angle else raw
        self._prev_stamp_s = (
            None if stamp_s is None else float(stamp_s))

    def predict_from_raw(
            self,
            raw_value: float,
            stamp_s: Optional[float] = None) -> bool:
        """Propagate one new dead-reckoning observation.

        Returns ``True`` only when a new propagation was applied. Equal or
        backwards timestamps are ignored, preventing covariance growth from a
        cached raw value being reused by a faster control loop.
        """
        raw = float(raw_value)
        stamp = None if stamp_s is None else float(stamp_s)
        if not math.isfinite(raw) or (stamp is not None and not math.isfinite(stamp)):
            return False
        if self._prev_raw is None:
            self._prev_raw = normalize_angle(raw) if self.angle else raw
            self._prev_stamp_s = stamp
            return False
        if (stamp is not None and self._prev_stamp_s is not None and
                stamp <= self._prev_stamp_s):
            return False

        delta = self._difference(raw, self._prev_raw)
        self.x += delta
        self._normalize_state()

        if stamp is not None and self._prev_stamp_s is not None:
            dt = min(max(stamp - self._prev_stamp_s, 0.0), 1.0)
        else:
            dt = 0.02
        self.P += (
            self.process_variance_rate * max(dt, 1.0e-3) +
            (self.process_gain * abs(delta)) ** 2)
        self._prev_raw = normalize_angle(raw) if self.angle else raw
        self._prev_stamp_s = stamp
        return True

    # Compatibility with the legacy ScalarKalman call pattern.
    def predict(self, raw_value: float) -> bool:
        return self.predict_from_raw(raw_value)

    def innovation(self, measured: float) -> float:
        return self._difference(float(measured), self.x)

    def innovation_variance(self) -> float:
        return self.P + self.R

    def update(self, measured: float) -> float:
        residual = self.innovation(measured)
        gain = self.P / self.innovation_variance()
        self.x += gain * residual
        self._normalize_state()
        self.P = max((1.0 - gain) * self.P, 1.0e-12)
        return gain


class OncePerStamp:
    """Consume each positive timestamp at most once and in order."""

    def __init__(self) -> None:
        self.last_stamp_ns = 0

    def reset(self) -> None:
        self.last_stamp_ns = 0

    def consume(self, stamp_ns: Optional[int]) -> bool:
        if stamp_ns is None:
            return False
        stamp = int(stamp_ns)
        if stamp <= 0 or stamp <= self.last_stamp_ns:
            return False
        self.last_stamp_ns = stamp
        return True


@dataclass(frozen=True)
class ScalarGateDecision:
    """Decision for one independently gated relative-state axis."""

    action: str
    residual: Optional[float]
    reason: str

    @property
    def accepted(self) -> bool:
        return self.action in ('ACCEPT', 'REACQUIRE')


class ScalarObservationGate:
    """Scalar innovation gate with consistency-based re-acquisition.

    Keeping one instance per axis prevents an unreliable solvePnP yaw from
    discarding a sound distance or lateral observation.
    """

    def __init__(self, *, innovation_limit: float, sigma_limit: float,
                 reacquire_count: int, reacquire_limit: float,
                 consistency_limit: float, angle: bool = False):
        self.innovation_limit = float(innovation_limit)
        self.sigma_limit = float(sigma_limit)
        self.reacquire_count = int(reacquire_count)
        self.reacquire_limit = float(reacquire_limit)
        self.consistency_limit = float(consistency_limit)
        self.angle = bool(angle)
        if any(value <= 0.0 for value in (
                self.innovation_limit, self.sigma_limit,
                self.reacquire_limit, self.consistency_limit)):
            raise ValueError('gate limits must be positive')
        if self.reacquire_count < 2:
            raise ValueError('reacquire_count must be at least 2')
        self._candidate: Optional[float] = None
        self._candidate_count = 0

    def reset(self) -> None:
        self._candidate = None
        self._candidate_count = 0

    def _difference(self, lhs: float, rhs: float) -> float:
        value = float(lhs) - float(rhs)
        return normalize_angle(value) if self.angle else value

    def evaluate(self, measurement: float,
                 state_filter: DeltaKalman1D) -> ScalarGateDecision:
        residual = state_filter.innovation(measurement)
        variance = state_filter.innovation_variance()
        within_sigma = (
            variance > 0.0 and math.isfinite(variance) and
            residual * residual <= self.sigma_limit ** 2 * variance)
        if abs(residual) <= self.innovation_limit and within_sigma:
            self.reset()
            return ScalarGateDecision('ACCEPT', residual, 'innovation_ok')

        if abs(residual) > self.reacquire_limit:
            self.reset()
            return ScalarGateDecision(
                'REJECT', residual, 'outside_reacquire_envelope')

        consistent = (
            self._candidate is not None and
            abs(self._difference(measurement, self._candidate)) <=
            self.consistency_limit)
        if consistent:
            self._candidate_count += 1
        else:
            self._candidate = float(measurement)
            self._candidate_count = 1
        if self._candidate_count >= self.reacquire_count:
            self.reset()
            return ScalarGateDecision(
                'REACQUIRE', residual, 'consistent_bounded_observations')
        return ScalarGateDecision(
            'REJECT', residual,
            f'candidate_{self._candidate_count}/{self.reacquire_count}')


@dataclass(frozen=True)
class GateDecision:
    action: str
    distance_residual: Optional[float]
    yaw_residual: Optional[float]
    reason: str

    @property
    def accepted(self) -> bool:
        return self.action in ('ACCEPT', 'REACQUIRE')


class RelativeObservationGate:
    """Innovation gate with bounded, consistency-based re-acquisition."""

    def __init__(
            self,
            *,
            distance_limit: float,
            yaw_limit: float,
            sigma_limit: float,
            reacquire_count: int,
            reacquire_distance_limit: float,
            reacquire_yaw_limit: float,
            consistency_distance: float,
            consistency_yaw: float):
        self.distance_limit = float(distance_limit)
        self.yaw_limit = float(yaw_limit)
        self.sigma_limit = float(sigma_limit)
        self.reacquire_count = int(reacquire_count)
        self.reacquire_distance_limit = float(reacquire_distance_limit)
        self.reacquire_yaw_limit = float(reacquire_yaw_limit)
        self.consistency_distance = float(consistency_distance)
        self.consistency_yaw = float(consistency_yaw)
        values = (
            self.distance_limit, self.yaw_limit, self.sigma_limit,
            self.reacquire_distance_limit, self.reacquire_yaw_limit,
            self.consistency_distance, self.consistency_yaw)
        if any(value <= 0.0 for value in values):
            raise ValueError('gate limits must be positive')
        if self.reacquire_count < 2:
            raise ValueError('reacquire_count must be at least 2')
        self._candidate: Optional[Tuple[Optional[float], float]] = None
        self._candidate_count = 0

    def reset(self) -> None:
        self._candidate = None
        self._candidate_count = 0

    @staticmethod
    def _within_sigma(residual: float, variance: float, sigma: float) -> bool:
        if variance <= 0.0 or not math.isfinite(variance):
            return False
        return residual * residual <= sigma * sigma * variance

    def _consistent(
            self,
            distance: Optional[float],
            yaw: float) -> bool:
        if self._candidate is None:
            return False
        candidate_distance, candidate_yaw = self._candidate
        distance_ok = (
            distance is None or candidate_distance is None or
            abs(distance - candidate_distance) <= self.consistency_distance)
        yaw_ok = abs(normalize_angle(yaw - candidate_yaw)) <= self.consistency_yaw
        return distance_ok and yaw_ok

    def evaluate(
            self,
            *,
            distance_measurement: Optional[float],
            yaw_measurement: float,
            distance_filter: Optional[DeltaKalman1D],
            yaw_filter: DeltaKalman1D) -> GateDecision:
        distance_residual = (
            None if distance_measurement is None or distance_filter is None
            else distance_filter.innovation(distance_measurement))
        yaw_residual = yaw_filter.innovation(yaw_measurement)

        distance_accept = (
            distance_residual is None or
            (abs(distance_residual) <= self.distance_limit and
             self._within_sigma(
                 distance_residual,
                 distance_filter.innovation_variance(),
                 self.sigma_limit)))
        yaw_accept = (
            abs(yaw_residual) <= self.yaw_limit and
            self._within_sigma(
                yaw_residual, yaw_filter.innovation_variance(),
                self.sigma_limit))
        if distance_accept and yaw_accept:
            self.reset()
            return GateDecision(
                'ACCEPT', distance_residual, yaw_residual, 'innovation_ok')

        bounded = (
            (distance_residual is None or
             abs(distance_residual) <= self.reacquire_distance_limit) and
            abs(yaw_residual) <= self.reacquire_yaw_limit)
        if not bounded:
            self.reset()
            return GateDecision(
                'REJECT', distance_residual, yaw_residual,
                'outside_reacquire_envelope')

        if self._consistent(distance_measurement, yaw_measurement):
            self._candidate_count += 1
        else:
            self._candidate = (distance_measurement, yaw_measurement)
            self._candidate_count = 1

        if self._candidate_count >= self.reacquire_count:
            self.reset()
            return GateDecision(
                'REACQUIRE', distance_residual, yaw_residual,
                'consistent_bounded_observations')
        return GateDecision(
            'REJECT', distance_residual, yaw_residual,
            f'candidate_{self._candidate_count}/{self.reacquire_count}')


def anchored_pose(
        anchor_world: Tuple[float, float, float],
        anchor_local: Tuple[float, float, float],
        current_local: Tuple[float, float, float]) -> Tuple[float, float, float]:
    """Map a local wheel-odometry pose onto a world-frame anchor pose."""
    lx0, ly0, lyaw0 = anchor_local
    lx, ly, lyaw = current_local
    dx = lx - lx0
    dy = ly - ly0
    c0 = math.cos(lyaw0)
    s0 = math.sin(lyaw0)
    # Translation expressed in the local anchor body frame.
    rel_x = c0 * dx + s0 * dy
    rel_y = -s0 * dx + c0 * dy
    rel_yaw = normalize_angle(lyaw - lyaw0)

    wx0, wy0, wyaw0 = anchor_world
    cw = math.cos(wyaw0)
    sw = math.sin(wyaw0)
    return (
        wx0 + cw * rel_x - sw * rel_y,
        wy0 + sw * rel_x + cw * rel_y,
        normalize_angle(wyaw0 + rel_yaw),
    )
