#!/usr/bin/env python3
"""Timestamped rewind/replay buffer for delayed absolute pose corrections."""

from __future__ import annotations

from dataclasses import dataclass
from collections import deque
import copy
from typing import Callable, Deque, Optional, Tuple


@dataclass(frozen=True)
class ReplayResult:
    accepted: bool
    status: str
    replayed_steps: int
    rewind_s: float
    quantization_s: float


@dataclass(frozen=True)
class WheelStep:
    stamp_ns: int
    dx_body: float
    dy_body: float
    dtheta: float
    dt: float
    state_before: Tuple


class EkfReplayBuffer:
    """Keep EKF snapshots before wheel increments and replay after correction."""

    def __init__(self, ekf, history_s: float = 1.0):
        self.ekf = ekf
        self.history_ns = int(float(history_s) * 1_000_000_000)
        if self.history_ns <= 0:
            raise ValueError('history_s must be positive')
        self.steps: Deque[WheelStep] = deque()
        self.latest_stamp_ns: Optional[int] = None

    def _snapshot(self):
        attrs = ('x', 'y', 'yaw', 'P', '_reject_streak',
                 'last_correction_accepted', 'last_mahalanobis',
                 'forced_reacquire', '_ever_accepted')
        return tuple(copy.deepcopy(getattr(self.ekf, name, None))
                     for name in attrs)

    def _restore(self, snapshot):
        attrs = ('x', 'y', 'yaw', 'P', '_reject_streak',
                 'last_correction_accepted', 'last_mahalanobis',
                 'forced_reacquire', '_ever_accepted')
        for name, value in zip(attrs, snapshot):
            if hasattr(self.ekf, name):
                setattr(self.ekf, name, copy.deepcopy(value))

    def _prune(self):
        if self.latest_stamp_ns is None:
            return
        cutoff = self.latest_stamp_ns - self.history_ns
        while self.steps and self.steps[0].stamp_ns < cutoff:
            self.steps.popleft()

    def record_predict(self, stamp_ns: int, dx_body: float, dy_body: float,
                       dtheta: float, dt: float):
        stamp = int(stamp_ns)
        if self.latest_stamp_ns is not None and stamp <= self.latest_stamp_ns:
            return False
        self.steps.append(WheelStep(
            stamp, float(dx_body), float(dy_body), float(dtheta), float(dt),
            self._snapshot()))
        self.ekf.predict(dx_body, dy_body, dtheta, dt)
        self.latest_stamp_ns = stamp
        self._prune()
        return True

    def correct_at(self, stamp_ns: int, correct_fn: Callable[[], bool]):
        """Correct at the latest wheel snapshot not newer than the measurement."""
        measurement_stamp = int(stamp_ns)
        if self.latest_stamp_ns is None or not self.steps:
            accepted = bool(correct_fn())
            return ReplayResult(
                accepted, 'CURRENT_NO_HISTORY', 0, 0.0, 0.0)
        if measurement_stamp >= self.latest_stamp_ns:
            accepted = bool(correct_fn())
            return ReplayResult(
                accepted, 'CURRENT', 0, 0.0,
                max(0.0, (measurement_stamp - self.latest_stamp_ns) * 1.0e-9))

        anchor_index = None
        for index, step in enumerate(self.steps):
            if step.stamp_ns <= measurement_stamp:
                anchor_index = index
            else:
                break
        if anchor_index is None:
            return ReplayResult(
                False, 'TOO_OLD_FOR_HISTORY', 0,
                (self.latest_stamp_ns - measurement_stamp) * 1.0e-9, 0.0)

        anchor = self.steps[anchor_index]
        replay_steps = list(self.steps)[anchor_index + 1:]
        self._restore(anchor.state_before)
        # Anchor step ends at anchor.stamp_ns. Re-apply it before the correction
        # so measurement quantization is to the latest wheel sample <= stamp.
        self.ekf.predict(
            anchor.dx_body, anchor.dy_body, anchor.dtheta, anchor.dt)
        quantization_s = max(
            0.0, (measurement_stamp - anchor.stamp_ns) * 1.0e-9)
        accepted = bool(correct_fn())
        if not accepted:
            # Restore the exact current state by replaying from the anchor.
            self._restore(anchor.state_before)
            self.ekf.predict(
                anchor.dx_body, anchor.dy_body, anchor.dtheta, anchor.dt)
            for step in replay_steps:
                self.ekf.predict(
                    step.dx_body, step.dy_body, step.dtheta, step.dt)
            return ReplayResult(
                False, 'REJECTED_RESTORED', len(replay_steps),
                (self.latest_stamp_ns - measurement_stamp) * 1.0e-9,
                quantization_s)
        for step in replay_steps:
            self.ekf.predict(
                step.dx_body, step.dy_body, step.dtheta, step.dt)
        return ReplayResult(
            True, 'REWIND_REPLAY', len(replay_steps),
            (self.latest_stamp_ns - measurement_stamp) * 1.0e-9,
            quantization_s)
