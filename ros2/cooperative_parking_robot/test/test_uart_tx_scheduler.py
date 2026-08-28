"""Deterministic tests for bounded UART scheduling (no ROS or sleeps)."""

from pathlib import Path

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
