"""ROS-independent source timestamp freshness and ordering checks."""

from __future__ import annotations

from collections import OrderedDict


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


class RequestReplayGuard:
    '''Bounded replay protection for legacy and client-scoped UI requests.'''

    def __init__(self, max_clients=32, max_request_ids=128):
        self.max_clients = int(max_clients)
        self.max_request_ids = int(max_request_ids)
        if self.max_clients <= 0 or self.max_request_ids <= 0:
            raise ValueError('replay guard bounds must be positive')
        self._legacy_sequence = -1
        self._client_sequences = OrderedDict()
        self._request_ids = OrderedDict()

    def accept(self, client_id, sequence, request_id):
        client_id = str(client_id or '').strip()
        request_id = str(request_id or '').strip()
        sequence = int(sequence)
        if sequence < 0:
            return False, 'INVALID_SEQUENCE'
        if request_id and request_id in self._request_ids:
            return False, 'DUPLICATE_REQUEST_ID'

        if client_id:
            previous = self._client_sequences.get(client_id, -1)
            if sequence <= previous:
                return False, 'DUPLICATE_SEQUENCE'
            self._client_sequences[client_id] = sequence
            self._client_sequences.move_to_end(client_id)
            while len(self._client_sequences) > self.max_clients:
                self._client_sequences.popitem(last=False)
        else:
            if sequence <= self._legacy_sequence:
                return False, 'DUPLICATE_SEQUENCE'
            self._legacy_sequence = sequence

        if request_id:
            self._request_ids[request_id] = None
            self._request_ids.move_to_end(request_id)
            while len(self._request_ids) > self.max_request_ids:
                self._request_ids.popitem(last=False)
        return True, 'OK'
