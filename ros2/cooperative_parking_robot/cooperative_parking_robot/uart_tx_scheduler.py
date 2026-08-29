"""Single-writer, bounded UART transmit scheduler.

The scheduler deliberately keeps heartbeat and velocity as replaceable slots.
They are state updates, not an event history: transmitting an old backlog is
both useless and unsafe.  Emergency frames use a small FIFO and always win.
"""

from collections import deque
from dataclasses import dataclass
import threading
import time


P0_EMERGENCY = 0
P1_REALTIME = 1
P2_HANDSHAKE = 2
P3_ACTION = 3
P4_MAINTENANCE = 4


@dataclass
class TxItem:
    payload: bytes
    kind: str
    priority: int
    deadline: float
    enqueued_at: float
    metadata: object = None


class UartTxScheduler:
    """Thread-safe priority scheduler with one serial writer.

    ``pop_next`` is public so ordering can be tested with a fake clock and no
    sleeps.  Production uses ``start`` and the worker calls the same method.
    """

    COALESCED_KINDS = frozenset({
        'heartbeat', 'velocity', 'hello', 'zero_probe', 'servo_attach'})
    COALESCED_KINDS = COALESCED_KINDS | frozenset({'ultrasonic_control'})

    def __init__(self, serial_getter, on_result=None, clock=time.monotonic):
        self._serial_getter = serial_getter
        self._on_result = on_result
        self._clock = clock
        self._condition = threading.Condition()
        self._emergency = deque()
        self._slots = {}
        self._queues = {priority: deque() for priority in range(1, 5)}
        self._stopping = False
        self._thread = None

    def start(self):
        with self._condition:
            if self._thread is not None:
                return
            self._stopping = False
            self._thread = threading.Thread(
                target=self._run, name='stm32-uart-tx', daemon=True)
            self._thread.start()

    def enqueue(self, payload, *, kind, priority, deadline=float('inf'),
                metadata=None):
        if not isinstance(payload, bytes):
            raise TypeError('UART scheduler payload must be bytes')
        if priority not in range(5):
            raise ValueError('UART priority must be in [0, 4]')
        item = TxItem(payload, kind, priority, deadline, self._clock(), metadata)
        with self._condition:
            if self._stopping:
                return False
            if priority == P0_EMERGENCY:
                self._emergency.append(item)
            elif kind in self.COALESCED_KINDS:
                self._slots[kind] = item
            else:
                self._queues[priority].append(item)
            self._condition.notify()
        return True

    def discard_non_emergency(self):
        with self._condition:
            self._slots.clear()
            for queue in self._queues.values():
                queue.clear()

    def pop_next(self):
        """Return the next item; P1 replaceable slots use earliest deadline."""
        with self._condition:
            if self._emergency:
                return self._emergency.popleft()
            realtime = [item for item in self._slots.values()
                        if item.priority == P1_REALTIME]
            if realtime:
                selected = min(realtime,
                               key=lambda item: (item.deadline,
                                                 item.enqueued_at))
                del self._slots[selected.kind]
                return selected
            for priority in range(P2_HANDSHAKE, P4_MAINTENANCE + 1):
                slots = [item for item in self._slots.values()
                         if item.priority == priority]
                if slots:
                    selected = min(slots, key=lambda item: item.enqueued_at)
                    del self._slots[selected.kind]
                    return selected
                if self._queues[priority]:
                    return self._queues[priority].popleft()
            return None

    def pending_count(self, kind):
        with self._condition:
            count = int(kind in self._slots)
            count += sum(item.kind == kind for item in self._emergency)
            count += sum(item.kind == kind for queue in self._queues.values()
                         for item in queue)
            return count

    def pending_snapshot(self):
        """Return bounded queue-depth diagnostics without exposing internals."""
        with self._condition:
            by_kind = {}
            items = list(self._emergency) + list(self._slots.values())
            for queue in self._queues.values():
                items.extend(queue)
            for item in items:
                by_kind[item.kind] = by_kind.get(item.kind, 0) + 1
            return {
                'total': len(items),
                'emergency': len(self._emergency),
                'slots': len(self._slots),
                'queued': sum(len(queue) for queue in self._queues.values()),
                'by_kind': by_kind,
            }

    def stop(self, timeout=1.0, *, drain=True):
        with self._condition:
            self._stopping = True
            if not drain:
                self._emergency.clear()
                self._slots.clear()
                for queue in self._queues.values():
                    queue.clear()
            self._condition.notify_all()
            thread = self._thread
        if thread is not None and thread is not threading.current_thread():
            thread.join(timeout)
        with self._condition:
            self._thread = None

    def _run(self):
        while True:
            item = self.pop_next()
            if item is None:
                with self._condition:
                    if self._stopping:
                        return
                    self._condition.wait(timeout=0.05)
                continue
            serial_handle = self._serial_getter()
            started = self._clock()
            error = None
            written = 0
            try:
                if serial_handle is None:
                    raise OSError('serial disconnected')
                written = serial_handle.write(item.payload)
                if written != len(item.payload):
                    raise OSError(
                        f'partial UART write {written}/{len(item.payload)}')
            except Exception as exc:  # reported on the common result path
                error = exc
            finished = self._clock()
            if self._on_result is not None:
                self._on_result(item, started, finished - started, error)
