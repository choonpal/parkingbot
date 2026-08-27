#!/usr/bin/env python3
"""Pure policy for accepting absolute-pose camera source handovers."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple


def normalize_angle(value: float) -> float:
    return math.atan2(math.sin(float(value)), math.cos(float(value)))


def _distance(lhs, rhs):
    return math.hypot(float(lhs[0]) - float(rhs[0]),
                      float(lhs[1]) - float(rhs[1]))


@dataclass(frozen=True)
class SourceDecision:
    accepted: bool
    source_changed: bool
    reason: str


class SourceSwitchGuard:
    """Reject unverified cross-camera jumps before they reach the EKF."""

    def __init__(self, *, confirmations: int = 3,
                 consistency_position_m: float = 0.03,
                 consistency_yaw_rad: float = math.radians(3.0),
                 max_position_jump_m: float = 0.12,
                 max_yaw_jump_rad: float = math.radians(12.0)):
        self.confirmations = int(confirmations)
        self.consistency_position = float(consistency_position_m)
        self.consistency_yaw = float(consistency_yaw_rad)
        self.max_position_jump = float(max_position_jump_m)
        self.max_yaw_jump = float(max_yaw_jump_rad)
        if self.confirmations < 1:
            raise ValueError('confirmations must be positive')
        if min(self.consistency_position, self.consistency_yaw,
               self.max_position_jump, self.max_yaw_jump) <= 0.0:
            raise ValueError('source-switch limits must be positive')
        self.current_source: Optional[str] = None
        self._candidate_source: Optional[str] = None
        self._candidate_pose: Optional[Tuple[float, float, float]] = None
        self._candidate_count = 0

    def reset(self):
        self.current_source = None
        self._candidate_source = None
        self._candidate_pose = None
        self._candidate_count = 0

    def _clear_candidate(self):
        self._candidate_source = None
        self._candidate_pose = None
        self._candidate_count = 0

    def evaluate(self, source: str, measured_pose, predicted_pose, *,
                 handover_validated: bool) -> SourceDecision:
        source = str(source).strip()
        if not source:
            return SourceDecision(False, False, 'missing_source')
        measurement = tuple(float(value) for value in measured_pose)
        predicted = tuple(float(value) for value in predicted_pose)
        if len(measurement) != 3 or len(predicted) != 3:
            return SourceDecision(False, False, 'invalid_pose')
        if not all(math.isfinite(value) for value in (*measurement, *predicted)):
            return SourceDecision(False, False, 'nonfinite_pose')
        position_jump = _distance(measurement, predicted)
        yaw_jump = abs(normalize_angle(measurement[2] - predicted[2]))
        if (position_jump > self.max_position_jump or
                yaw_jump > self.max_yaw_jump):
            self._clear_candidate()
            return SourceDecision(False, False, 'source_jump_limit')

        if self.current_source is None:
            self.current_source = source
            self._clear_candidate()
            return SourceDecision(True, False, 'initial_source')
        if source == self.current_source:
            self._clear_candidate()
            return SourceDecision(True, False, 'same_source')
        if handover_validated:
            previous = self.current_source
            self.current_source = source
            self._clear_candidate()
            return SourceDecision(True, source != previous,
                                  'validated_handover')

        consistent = (
            self._candidate_source == source and
            self._candidate_pose is not None and
            _distance(measurement, self._candidate_pose) <=
            self.consistency_position and
            abs(normalize_angle(
                measurement[2] - self._candidate_pose[2])) <=
            self.consistency_yaw)
        if consistent:
            self._candidate_count += 1
        else:
            self._candidate_source = source
            self._candidate_pose = measurement
            self._candidate_count = 1
        self._candidate_pose = measurement
        if self._candidate_count < self.confirmations:
            return SourceDecision(False, False,
                                  f'confirming_source_{self._candidate_count}')
        previous = self.current_source
        self.current_source = source
        self._clear_candidate()
        return SourceDecision(True, source != previous,
                              'consistent_handover')
