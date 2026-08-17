"""ROS-independent source timestamp freshness and ordering checks."""

from __future__ import annotations


NSEC_PER_SEC = 1_000_000_000


def stamp_to_ns(stamp):
    """Convert a ROS ``builtin_interfaces/Time``-like object to nanoseconds."""
    return int(stamp.sec) * NSEC_PER_SEC + int(stamp.nanosec)


class StampGate:
    """Reject zero, stale, future, duplicate, and out-of-order samples."""

    def __init__(self, max_age_s, future_tolerance_s=0.10):
        self.max_age_ns = int(float(max_age_s) * NSEC_PER_SEC)
        self.future_tolerance_ns = int(
            float(future_tolerance_s) * NSEC_PER_SEC)
        if self.max_age_ns <= 0:
            raise ValueError("max_age_s must be positive")
        if self.future_tolerance_ns < 0:
            raise ValueError("future_tolerance_s must be non-negative")
        self.last_stamp_ns = 0

    def accept(self, stamp_ns, now_ns):
        stamp_ns = int(stamp_ns)
        now_ns = int(now_ns)
        if stamp_ns <= 0:
            return False, "ZERO_STAMP"
        if stamp_ns <= self.last_stamp_ns:
            return False, "DUPLICATE_OR_OUT_OF_ORDER"
        age_ns = now_ns - stamp_ns
        if age_ns < -self.future_tolerance_ns:
            return False, "FUTURE_STAMP"
        if age_ns > self.max_age_ns:
            return False, "STALE_STAMP"
        self.last_stamp_ns = stamp_ns
        return True, "OK"

    def reset(self):
        self.last_stamp_ns = 0
