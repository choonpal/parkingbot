"""Regression tests for the ROS ↔ STM32 servo startup handshake."""

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
    def __init__(self):
        self.writes = []

    def write(self, data):
        self.writes.append(data)
        return len(data)


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


def bridge_for_unit_test(profile='robot-2'):
    node = object.__new__(Stm32BridgeNode)
    node.role = 'front' if profile == 'robot-2' else 'rear'
    node.hardware_profile = profile
    node.protocol = UartProtocol()
    node.ser = FakeSerial()
    node.servo_attach_pulses = servo_attach_pulses_for(profile)
    node.servo_attach_retry_interval = 0.75
    node.last_servo_attach_request_time = None
    node.servo_attached = False
    node.servo_attach_blocked = False
    node.estop_latched = False
    node.last_ack_time = 0.0
    node.last_ultrasonic_frame = {'left': 0.0, 'right': 0.0}
    node.ultrasonic_frame_timeout = 0.5
    node.require_ultrasonic_for_ready = False
    node.ultrasonic_stale_reported = False
    node.hardware_ready = False
    node.command_arbiter = FakeArbiter()
    node.pub_ready = FakePublisher()
    node.pub_manual_active = FakePublisher()
    node.statuses = []
    node.publish_status = node.statuses.append
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    return node


def test_encode_servo_attach_matches_firmware_wire_format():
    assert UartProtocol().encode_servo_attach(2600, 400) == \
        '@S,attach,2600,400\n'


@pytest.mark.parametrize(
    ('pulse1', 'pulse2'),
    ((399, 400), (400, 2601), (400.0, 2600), ('400', 2600),
     (True, 2600)),
)
def test_encode_servo_attach_rejects_invalid_pulses(pulse1, pulse2):
    with pytest.raises(ValueError):
        UartProtocol().encode_servo_attach(pulse1, pulse2)


def test_startup_state_is_not_servo_attached(monkeypatch, tmp_path):
    monkeypatch.setenv('ROS_LOG_DIR', str(tmp_path / 'ros-log'))
    initialized_here = not rclpy.ok()
    if initialized_here:
        rclpy.init()
    node = Stm32BridgeNode(parameter_overrides=[
        Parameter('enable_serial', Parameter.Type.BOOL, False),
        Parameter(
            'require_ultrasonic_for_ready', Parameter.Type.BOOL, False),
    ])
    try:
        assert node.servo_attached is False
        assert node.hardware_ready is False
    finally:
        node.destroy_node()
        if initialized_here:
            rclpy.shutdown()


def test_attach_request_is_immediate_profile_based_and_rate_limited(
        monkeypatch):
    node = bridge_for_unit_test('robot-2')
    now = [10.0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node.send_servo_attach()
    assert node.ser.writes == [b'@S,attach,2600,400\n']

    now[0] += 0.74
    node.send_servo_attach()
    assert len(node.ser.writes) == 1

    now[0] += 0.01
    node.send_servo_attach()
    assert node.ser.writes[-1] == b'@S,attach,2600,400\n'
    assert len(node.ser.writes) == 2


def test_hardware_ready_requires_specific_servo_attach_ack(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node._handle_serial_line('ACK,100.000')
    assert node.last_ack_time == 100.0
    assert node.servo_attached is False
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is False

    node._handle_serial_line('ACK,SERVO_ATTACH')
    assert node.servo_attached is True
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is True


def test_grip_and_release_are_blocked_until_attach_ack(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)

    node._send_grip('grip')
    node._send_grip('release')
    assert node.ser.writes == []
    assert node.statuses == [
        'WARN,SERVO_NOT_READY', 'WARN,SERVO_NOT_READY']

    node._handle_serial_line('ACK,SERVO_ATTACH')
    node._send_grip('grip')
    node._send_grip('release')
    assert node.ser.writes == [b'@S,grip\n', b'@S,release\n']


def test_estop_latch_stops_attach_retry_and_servo_commands(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)

    node._handle_serial_line('ERR,ESTOP_LATCHED')
    assert node.estop_latched is True
    assert node.servo_attached is False
    assert node.servo_attach_blocked is True
    assert node.command_arbiter.force_zero_calls == [100.0]

    node.send_servo_attach()
    node._send_grip('grip')
    node._handle_serial_line('ACK,SERVO_ATTACH')
    assert node.ser.writes == []
    assert node.servo_attached is False
    assert 'ERR,ESTOP_LATCHED' in node.statuses
    assert 'WARN,ESTOP_LATCHED_POWER_CYCLE_REQUIRED' in node.statuses


def test_bad_servo_attach_stops_retry_and_rejects_late_ack(monkeypatch):
    node = bridge_for_unit_test()
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: 100.0)

    node._handle_serial_line('ERR,BAD_SERVO_ATTACH')
    assert node.servo_attached is False
    assert node.servo_attach_blocked is True

    node.send_servo_attach()
    node._handle_serial_line('ACK,SERVO_ATTACH')
    assert node.ser.writes == []
    assert node.servo_attached is False
    assert 'ERR,BAD_SERVO_ATTACH' in node.statuses


def test_physical_profiles_select_opposite_attach_pulses():
    assert servo_attach_pulses_for('robot-1') == (400, 2600)
    assert servo_attach_pulses_for('robot-2') == (2600, 400)
    with pytest.raises(ValueError):
        servo_attach_pulses_for('robot-3')
