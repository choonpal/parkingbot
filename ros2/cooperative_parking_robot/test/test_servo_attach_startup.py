"""Regression tests for the session-bound ROS ↔ STM32 startup handshake."""

from collections import deque
from pathlib import Path

import pytest
import rclpy
from rclpy.parameter import Parameter

from cooperative_parking_robot import stm32_bridge_node as bridge_module
from cooperative_parking_robot.hardware_profile import servo_attach_pulses_for
from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode
from cooperative_parking_robot.uart_protocol import UartProtocol


class FakeLogger:
    def __init__(self):
        self.messages = []

    def _record(self, level, message, **_kwargs):
        self.messages.append((level, message))

    def info(self, message, **kwargs):
        self._record('info', message, **kwargs)

    def warn(self, message, **kwargs):
        self._record('warn', message, **kwargs)

    def error(self, message, **kwargs):
        self._record('error', message, **kwargs)


class FakeSerial:
    is_open = True

    def __init__(self):
        self.writes = []
        self.closed = False

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True
        self.is_open = False


class FakePublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class FakeArbiter:
    manual_enabled = False

    def __init__(self):
        self.force_zero_calls = []

    def force_zero(self, now):
        self.force_zero_calls.append(now)

    def output(self, _now):
        return 0.0, 0.0, 0.0


def bridge_for_unit_test(profile='robot-2'):
    node = object.__new__(Stm32BridgeNode)
    node.role = 'front' if profile == 'robot-2' else 'rear'
    node.hardware_profile = profile
    node.protocol = UartProtocol()
    node.session_id = 'abcdef0123456789'
    node.ser = FakeSerial()
    node.servo_attach_pulses = servo_attach_pulses_for(profile)
    node.servo_attach_retry_interval = 0.75
    node.last_servo_attach_request_time = None
    node.servo_attach_requested = False
    node.servo_attached = False
    node.servo_attach_blocked = False
    node.estop_latched = False
    node.hello_started_at = 10.0
    node.last_hello_request_time = None
    node.hello_retry_interval = 0.25
    node.hello_handshake_timeout = 2.0
    node.hello_acknowledged = False
    node.heartbeat_sequence = 0
    node.outstanding_heartbeats = {}
    node.last_heartbeat_ack_time = 0.0
    node.heartbeat_ack_timeout = 0.30
    node.last_zero_request_time = None
    node.zero_command_sent = False
    node.zero_command_acknowledged = False
    node.previous_session_faults = []
    node.active_fault = None
    node.transport_fault = None
    node.invalid_frame_times = deque()
    node.uart_frame_fault_count = 3
    node.uart_frame_fault_window = 1.0
    node.last_ultrasonic_valid = {'left': 0.0, 'right': 0.0}
    node.ultrasonic_frame_timeout = 0.5
    node.require_ultrasonic_for_ready = True
    node.ultrasonic_stale_reported = False
    node.hardware_ready = False
    node.command_arbiter = FakeArbiter()
    node.command_sign = (1.0, 1.0, 1.0)
    node.pub_ready = FakePublisher()
    node.pub_manual_active = FakePublisher()
    node.statuses = []
    node.publish_status = node.statuses.append
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    return node


def complete_to_heartbeat_ack(node):
    node.send_hello()
    node._handle_serial_line(
        f'ACK,{node.protocol.hello_ack_value(node.session_id)}')
    heartbeat = node.ser.writes[-1].decode().strip().removeprefix('@HB,')
    node._handle_serial_line(f'ACK,{heartbeat}')
    return heartbeat


def complete_startup(node, now):
    complete_to_heartbeat_ack(node)
    assert node.ser.writes[-1] == node.protocol.encode_zero_velocity(
        node.session_id).encode()
    node._handle_serial_line(
        f'ACK,{node.protocol.zero_velocity_ack_value(node.session_id)}')
    assert node.ser.writes[-1].startswith(b'@S,attach,')
    node._handle_serial_line('ACK,SERVO_ATTACH')
    node.last_ultrasonic_valid = {'left': now, 'right': now}


def test_protocol_v2_encoders_match_firmware_wire_format():
    protocol = UartProtocol()
    session = 'abcdef0123456789'
    assert protocol.encode_hello(session) == f'@HELLO,2,{session}\n'
    assert protocol.hello_ack_value(session) == f'HELLO:2:{session}'
    assert protocol.encode_zero_velocity(session) == \
        f'@V,0.000,0.000,0.000,{session}\n'
    assert protocol.zero_velocity_ack_value(session) == f'V:{session}'
    assert protocol.encode_servo_attach(2600, 400) == \
        '@S,attach,2600,400\n'


@pytest.mark.parametrize('session', ('', 'short', 'has,comma', 'space bad id'))
def test_hello_rejects_invalid_session_ids(session):
    with pytest.raises(ValueError):
        UartProtocol().encode_hello(session)


def test_startup_state_is_not_ready_without_serial(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = Stm32BridgeNode(parameter_overrides=[
        Parameter('enable_serial', Parameter.Type.BOOL, False),
        Parameter('require_ultrasonic_for_ready', Parameter.Type.BOOL, False),
    ])
    try:
        assert node.hello_acknowledged is False
        assert node.zero_command_acknowledged is False
        assert node.servo_attached is False
        assert node.hardware_ready is False
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def test_strict_handshake_order_and_wrong_hello_token(monkeypatch):
    node = bridge_for_unit_test()
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node.send_hello()
    assert node.ser.writes == [b'@HELLO,2,abcdef0123456789\n']
    node.send_heartbeat()
    node.send_servo_attach()
    assert len(node.ser.writes) == 1

    node._handle_serial_line('ACK,HELLO:2:0000000000000000')
    assert node.hello_acknowledged is False
    assert len(node.ser.writes) == 1

    complete_to_heartbeat_ack(node)
    assert node.hello_acknowledged is True
    assert node.last_heartbeat_ack_time == now[0]
    assert node.zero_command_sent is True
    assert node.servo_attach_requested is False


def test_hardware_ready_requires_every_specific_gate(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node._handle_serial_line('ACK,ARBITRARY')
    assert node.last_heartbeat_ack_time == 0.0
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is False

    complete_startup(node, now[0])
    conditions = node.hardware_ready_conditions(now[0])
    assert all(conditions.values()), conditions
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is True


def test_previous_session_timeout_is_information_current_timeout_is_fatal(
        monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    node._handle_serial_line('ERR,COMMAND_TIMEOUT')
    node._handle_serial_line('ERR,HEARTBEAT_TIMEOUT')
    assert node.active_fault is None
    assert node.statuses == []

    complete_startup(node, now[0])
    assert 'INFO,PREVIOUS_SESSION_FAULT:COMMAND_TIMEOUT' in node.statuses
    assert 'INFO,PREVIOUS_SESSION_FAULT:HEARTBEAT_TIMEOUT' in node.statuses
    node.publish_hardware_state()
    assert node.hardware_ready is True

    node._handle_serial_line('ERR,HEARTBEAT_TIMEOUT')
    assert node.active_fault == 'ERR,HEARTBEAT_TIMEOUT'
    assert node.servo_attached is False
    node._handle_serial_line('ACK,SERVO_ATTACH')
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is False


def test_command_timeout_race_never_reasserts_ready(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    complete_to_heartbeat_ack(node)
    node._handle_serial_line('ERR,COMMAND_TIMEOUT')
    node._handle_serial_line(
        f'ACK,{node.protocol.zero_velocity_ack_value(node.session_id)}')
    node._handle_serial_line('ACK,SERVO_ATTACH')
    node.last_ultrasonic_valid = {'left': now[0], 'right': now[0]}
    assert not all(node.hardware_ready_conditions(now[0]).values())


def test_estop_latch_stops_handshake_and_is_not_cleared_by_hello(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)
    node._handle_serial_line('ERR,ESTOP_LATCHED')
    assert node.estop_latched is True
    assert node.active_fault == 'ERR,ESTOP_LATCHED'
    before = list(node.ser.writes)
    node.send_hello()
    node.send_heartbeat()
    node.send_servo_attach()
    assert node.ser.writes == before


def test_repeated_malformed_uart_frames_fail_closed(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)
    node.hardware_ready = True
    serial_handle = node.ser
    node._handle_serial_line('EACK,12376.633')
    node._handle_serial_line('U,L,TIMEU,R,163')
    assert node.transport_fault is None
    node._handle_serial_line('T,V,0,0,0,0,0,0,0,0,2600E,0,0,0,0')
    assert node.transport_fault == 'ERR,UART_FRAME_CORRUPTION'
    assert node.active_fault == 'ERR,UART_FRAME_CORRUPTION'
    assert node.hardware_ready is False
    assert node.ser is None
    assert serial_handle.closed is True


def test_partial_uart_write_is_transport_fault(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)
    node.ser.write = lambda payload: len(payload) - 1
    node.send_hello()
    assert node.transport_fault == 'ERR,UART_PARTIAL_WRITE'
    assert node.ser is None


def test_serial_port_is_opened_exclusively():
    text = Path(bridge_module.__file__).read_text(encoding='utf-8')
    assert 'exclusive=True' in text


def test_grip_is_blocked_until_full_attach_ack(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    node._send_grip('grip')
    assert node.ser.writes == []
    complete_startup(node, now[0])
    node._send_grip('grip')
    assert node.ser.writes[-1] == b'@S,grip\n'


def test_physical_profiles_select_opposite_attach_pulses():
    assert servo_attach_pulses_for('robot-1') == (400, 2600)
    assert servo_attach_pulses_for('robot-2') == (2600, 400)
