"""Deterministic tests for bounded UART scheduling (no ROS or sleeps)."""

from pathlib import Path
import threading

from cooperative_parking_robot.uart_tx_scheduler import (
    P0_EMERGENCY,
    P1_REALTIME,
    P2_HANDSHAKE,
    P3_ACTION,
    P4_MAINTENANCE,
    UartTxScheduler,
)


class Clock:
    def __init__(self):
        self.now = 0.0

    def __call__(self):
        return self.now


def scheduler():
    clock = Clock()
    return UartTxScheduler(lambda: None, clock=clock), clock


def test_heartbeat_uses_latest_slot_and_keeps_deadline():
    tx, clock = scheduler()
    for sequence, at in enumerate((0.0, 0.1, 0.2), 1):
        clock.now = at
        tx.enqueue(f'hb{sequence}'.encode(), kind='heartbeat',
                   priority=P1_REALTIME, deadline=at)
        item = tx.pop_next()
        assert item.payload == f'hb{sequence}'.encode()
        assert item.deadline == at


def test_low_priority_retries_never_precede_heartbeat():
    tx, _ = scheduler()
    tx.enqueue(b'attach', kind='servo_attach', priority=P4_MAINTENANCE)
    tx.enqueue(b'hello', kind='hello', priority=P2_HANDSHAKE)
    tx.enqueue(b'hb', kind='heartbeat', priority=P1_REALTIME, deadline=1.0)
    assert tx.pop_next().payload == b'hb'


def test_velocity_and_heartbeat_are_earliest_deadline_first():
    tx, _ = scheduler()
    tx.enqueue(b'hb', kind='heartbeat', priority=P1_REALTIME, deadline=1.2)
    tx.enqueue(b'velocity', kind='velocity', priority=P1_REALTIME, deadline=1.1)
    assert tx.pop_next().payload == b'velocity'
    assert tx.pop_next().payload == b'hb'


def test_estop_preempts_every_priority():
    tx, _ = scheduler()
    tx.enqueue(b'action', kind='action', priority=P3_ACTION)
    tx.enqueue(b'hb', kind='heartbeat', priority=P1_REALTIME, deadline=0.0)
    tx.enqueue(b'estop', kind='estop', priority=P0_EMERGENCY)
    assert tx.pop_next().payload == b'estop'


def test_latest_velocity_wins_without_backlog():
    tx, _ = scheduler()
    for value in range(20):
        tx.enqueue(str(value).encode(), kind='velocity',
                   priority=P1_REALTIME, deadline=1.0)
    assert tx.pending_count('velocity') == 1
    assert tx.pop_next().payload == b'19'
    assert tx.pop_next() is None


def test_writer_wait_predicate_and_dequeue_share_the_condition_lock():
    source = (Path(__file__).resolve().parents[1] /
              'cooperative_parking_robot/uart_tx_scheduler.py').read_text()
    run = source[source.index('    def _run(self):'):]
    assert 'self._condition.wait_for(' in run
    assert 'self._stopping or self._has_pending_locked()' in run
    assert 'item = self._pop_next_locked()' in run
    assert 'wait(timeout=0.05)' not in run


def test_stop_wakes_idle_writer_and_joins_cleanly():
    tx = UartTxScheduler(lambda: None)
    tx.start()
    worker = tx._thread
    tx.stop(drain=False)
    assert not worker.is_alive()
    assert tx.is_running() is False


def test_reconnect_requires_old_writer_exit_before_new_handle_install():
    source = (Path(__file__).resolve().parents[1] /
              'cooperative_parking_robot/stm32_bridge_node.py').read_text()
    reconnect = source[source.index('    def serial_reconnect_tick(self):'):
                       source.index('    def publish_motor_diagnostics',
                                    source.index(
                                        '    def serial_reconnect_tick(self):'))]
    stop = reconnect.index('self.tx_scheduler.stop(drain=False)')
    verify = reconnect.index('if self.tx_scheduler.is_running():')
    install = reconnect.index('self._prepare_serial_session(handle)')
    start = reconnect.index('self.tx_scheduler.start()')
    assert stop < verify < install < start
    assert "handle.close()" in reconnect[verify:install]


def test_repeated_writer_restart_has_no_stale_threads():
    tx = UartTxScheduler(lambda: None)
    previous = []
    for _ in range(10):
        tx.start()
        worker = tx._thread
        assert tx.is_running()
        assert worker not in previous
        tx.stop(drain=False)
        assert not worker.is_alive()
        assert not tx.is_running()
        previous.append(worker)


def test_writer_cannot_restart_until_self_stopped_worker_has_exited():
    callback_entered = threading.Event()
    callback_release = threading.Event()
    serial = type('Serial', (), {'write': lambda _self, data: len(data)})()
    holder = {}

    def on_result(*_args):
        holder['scheduler'].stop(drain=False)
        callback_entered.set()
        assert callback_release.wait(1.0)

    tx = UartTxScheduler(lambda: serial, on_result=on_result)
    holder['scheduler'] = tx
    tx.start()
    original = tx._thread
    tx.enqueue(b'hb', kind='heartbeat', priority=P1_REALTIME)
    assert callback_entered.wait(1.0)
    tx.start()
    assert tx._thread is original
    callback_release.set()
    original.join(1.0)
    assert not original.is_alive()
    tx.start()
    assert tx._thread is not original
    tx.stop(drain=False)


def test_servo_attach_and_hello_are_deduplicated():
    tx, _ = scheduler()
    for _ in range(20):
        tx.enqueue(b'attach', kind='servo_attach', priority=P4_MAINTENANCE)
        tx.enqueue(b'hello', kind='hello', priority=P2_HANDSHAKE)
    assert tx.pending_count('servo_attach') == 1
    assert tx.pending_count('hello') == 1


def test_firmware_heartbeat_ack_has_dedicated_high_priority_mailbox():
    source = (Path(__file__).resolve().parents[3] /
              'stm32/parking_robot/Core/Src/parking_robot_firmware.c').read_text()
    assert '#define TX_HEARTBEAT_ACK' in source
    assert 'QueueHeartbeatAck(token);' in source
    definition = source.index('static void UART_SendPending(void)\n{')
    send_pending = source[definition:
                          source.index('static void UART_QueueRxCommand',
                                       definition)]
    assert send_pending.index('TX_ERR') < send_pending.index('TX_HEARTBEAT_ACK')
    assert send_pending.index('TX_HEARTBEAT_ACK') < send_pending.index('TX_ACK')
    assert send_pending.index('TX_ACK') < send_pending.index('TX_GRIP_DONE')


def test_firmware_keeps_fast_motor_watchdog_and_new_session_recovery():
    source = (Path(__file__).resolve().parents[3] /
              'stm32/parking_robot/Core/Src/parking_robot_firmware.c').read_text()
    assert '#define HEARTBEAT_TIMEOUT_MS 300U' in source
    assert '#define COMMAND_TIMEOUT_MS   250U' in source
    assert 'Robot_StopMotorsImmediate();' in source
    assert 'strcmp(g_robot.session_id, session_id) == 0' in source
    assert 'g_robot.heartbeat_timed_out = 0U;' in source


def test_command_keepalive_is_executor_independent_and_keeps_50hz_default():
    source = (Path(__file__).resolve().parents[1] /
              'cooperative_parking_robot/stm32_bridge_node.py').read_text()
    assert "declare_parameter('velocity_tx_rate_hz', 50.0)" in source
    assert 'target=self._command_producer_loop' in source
    assert "name=f'{self.role}-command-producer'" in source
    assert 'self.send_velocity_loop(scheduled_due=next_tick)' in source
    assert '1.0 / self.velocity_tx_rate_hz,\n            self.send_velocity_loop' not in source
    assert "kind='velocity', priority=P1_REALTIME" in source
