"""ROS-independent fast relative-pose predictor for cooperative control.

The two wheel-odometry streams are local dead-reckoning frames and therefore
cannot be subtracted directly.  A fresh visual Front-in-Rear pose anchors the
relationship once, then synchronized wheel SE(2) increments propagate it at
the encoder rate until the next visual correction arrives.
"""

from __future__ import annotations

import math


def angle_norm(angle):
    return math.atan2(math.sin(float(angle)), math.cos(float(angle)))


def _compose(a, b):
    ax, ay, ath = a
    bx, by, bth = b
    c = math.cos(ath)
    s = math.sin(ath)
    return (
        ax + c * bx - s * by,
        ay + s * bx + c * by,
        angle_norm(ath + bth),
    )


def _inverse(pose):
    x, y, yaw = pose
    c = math.cos(yaw)
    s = math.sin(yaw)
    return (
        -c * x - s * y,
        s * x - c * y,
        angle_norm(-yaw),
    )


def _relative_motion(anchor, current):
    return _compose(_inverse(anchor), current)


class EncoderRelativePredictor:
    """Propagate a stamped visual relative pose with synchronized wheel odom."""

    def __init__(self, sync_slop_s=0.05):
        self.sync_slop_s = float(sync_slop_s)
        if not math.isfinite(self.sync_slop_s) or self.sync_slop_s <= 0.0:
            raise ValueError('sync_slop_s must be finite and positive')
        self.reset()

    def reset(self):
        self._odom = {'front': None, 'rear': None}
        self._visual_pending = None
        self._anchor_relative = None
        self._anchor_odom = {'front': None, 'rear': None}
        self._last_prediction = None

    @staticmethod
    def _validate_pose(pose):
        values = tuple(float(value) for value in pose)
        if len(values) != 3 or not all(math.isfinite(value) for value in values):
            raise ValueError('pose must contain three finite values')
        return values

    def note_odom(self, role, pose, stamp_ns):
        if role not in ('front', 'rear'):
            raise ValueError("role must be 'front' or 'rear'")
        stamp_ns = int(stamp_ns)
        if stamp_ns <= 0:
            return False
        current = self._odom[role]
        if current is not None and stamp_ns <= current['stamp_ns']:
            return False
        self._odom[role] = {
            'pose': self._validate_pose(pose),
            'stamp_ns': stamp_ns,
        }
        self._try_anchor()
        return True

    def note_visual(self, relative_pose):
        self._visual_pending = self._validate_pose(relative_pose)
        return self._try_anchor()

    def _pair_is_synchronized(self):
        front = self._odom['front']
        rear = self._odom['rear']
        if front is None or rear is None:
            return False
        skew_s = abs(front['stamp_ns'] - rear['stamp_ns']) * 1.0e-9
        return skew_s <= self.sync_slop_s

    def _try_anchor(self):
        if self._visual_pending is None or not self._pair_is_synchronized():
            return False
        self._anchor_relative = self._visual_pending
        self._anchor_odom = {
            role: tuple(self._odom[role]['pose']) for role in ('front', 'rear')
        }
        self._last_prediction = self._anchor_relative
        self._visual_pending = None
        return True

    def predict(self):
        if self._anchor_relative is None:
            return None
        if not self._pair_is_synchronized():
            return self._last_prediction
        front_delta = _relative_motion(
            self._anchor_odom['front'], self._odom['front']['pose'])
        rear_delta = _relative_motion(
            self._anchor_odom['rear'], self._odom['rear']['pose'])
        # T_Rt_Ft = inv(ΔR) * T_R0_F0 * ΔF
        predicted = _compose(
            _inverse(rear_delta),
            _compose(self._anchor_relative, front_delta),
        )
        self._last_prediction = predicted
        return predicted
