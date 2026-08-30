"""Regression tests for the session-bound ROS ↔ STM32 startup handshake."""

from collections import deque
import math
from pathlib import Path
import threading

import pytest
import rclpy
from rclpy.parameter import Parameter
from std_msgs.msg import String

from cooperative_parking_robot import stm32_bridge_node as bridge_module
from cooperative_parking_robot.hardware_profile import servo_attach_pulses_for
from cooperative_parking_robot.stm32_bridge_node import (
    Stm32BridgeNode,
    odom_delta_since,
    rate_limited_publish_due,
)
from cooperative_parking_robot.uart_protocol import UartProtocol
from cooperative_parking_robot.ultrasonic_phase_health import (
    UltrasonicPhaseHealth,
)


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

    def debug(self, message, **kwargs):
        self._record('debug', message, **kwargs)


def test_odom_publish_rate_gate_keeps_first_and_due_samples():
    assert rate_limited_publish_due(10.0, None, 0.05)
    assert not rate_limited_publish_due(10.049, 10.0, 0.05)
    assert rate_limited_publish_due(10.05, 10.0, 0.05)


def test_odom_publish_rate_gate_recovers_from_clock_reset():
    assert rate_limited_publish_due(9.0, 10.0, 0.05)


def test_rate_limited_odom_delta_includes_skipped_encoder_frames():
    current = {
        'x': 0.03, 'y': 0.0, 'theta': 0.0,
        'dx_body': 0.01, 'dy_body': 0.0, 'dtheta': 0.0,
        'discontinuity': False,
    }
    published = odom_delta_since((0.0, 0.0, 0.0), current)
    assert published['dx_body'] == pytest.approx(0.03)
    assert published['dy_body'] == pytest.approx(0.0)
    assert published['dtheta'] == pytest.approx(0.0)


def test_rate_limited_odom_delta_preserves_curved_displacement():
    half_sqrt = 2.0 ** -0.5
    current = {
        'x': half_sqrt, 'y': half_sqrt, 'theta': math.pi / 2.0,
        'dx_body': 0.01, 'dy_body': 0.0, 'dtheta': 0.01,
        'discontinuity': False,
    }
    published = odom_delta_since((0.0, 0.0, 0.0), current)
    assert published['dx_body'] == pytest.approx(1.0)
    assert published['dy_body'] == pytest.approx(0.0)
    assert published['dtheta'] == pytest.approx(math.pi / 2.0)


class FakeSerial:
    is_open = True

    def __init__(self):
        self.writes = []
        self.closed = False
        self.incoming = bytearray()
        self.reset_input_calls = 0

    def write(self, data):
        self.writes.append(data)
        return len(data)

    def close(self):
        self.closed = True
        self.is_open = False

    @property
    def in_waiting(self):
        return len(self.incoming)

    def read(self, count):
        payload = bytes(self.incoming[:count])
        del self.incoming[:count]
        return payload

    def reset_input_buffer(self):
        self.reset_input_calls += 1
        self.incoming.clear()


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


class ImmediateScheduler:
    def __init__(self, node):
        self.node = node

    def enqueue(self, payload, **kwargs):
        started = bridge_module.time.monotonic()
        written = self.node.ser.write(payload)
        error = None
        if written != len(payload):
            error = OSError(f'partial UART write {written}/{len(payload)}')
        item = type('Item', (), {'kind': kwargs['kind'],
                                 'metadata': kwargs.get('metadata')})()
        self.node._uart_write_result(item, started, 0.0, error)
        return True

    def discard_non_emergency(self):
        pass

    def stop(self, **_kwargs):
        pass


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
    node.heartbeat_period = 0.10
    node.heartbeat_tx_late_warn = 0.04
    node.heartbeat_rtt_warn = 0.10
    node.heartbeat_recovery_ack_count = 3
    node.next_heartbeat_due = 10.1
    node.last_heartbeat_tx_time = 0.0
    node.communication_recovering = False
    node.communication_recovered = False
    node.recovery_fault_latched = False
    node.recovery_ack_count = 0
    node.heartbeat_lock = threading.RLock()
    node.heartbeat_stats = {
        'tx_count': 0, 'ack_count': 0, 'lost_count': 0,
        'stale_ack_count': 0, 'duplicate_ack_count': 0,
        'timeout_count': 0, 'uart_write_failure_count': 0,
        'max_tx_gap_ms': 0.0, 'max_rtt_ms': 0.0,
        'rtt_average_ms': 0.0, 'scheduler_max_lateness_ms': 0.0,
        'uart_write_max_ms': 0.0,
    }
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
    node.ultrasonic_health = UltrasonicPhaseHealth(
        required_valid_samples=3, invalid_samples_to_drop=3,
        max_sample_age_s=0.35, activation_timeout_s=1.0)
    node.ultrasonic_enable_target = False
    node.ultrasonic_command_acknowledged = True
    node.last_ultrasonic_command_time = None
    node.ultrasonic_command_retry_interval = 0.25
    node.ultrasonic_activation_timeout_reported = False
    node.ultrasonic_stale_reported = False
    node.hardware_ready = False
    node.motion_armed = False
    node.motion_arm_source = None
    node.robot_state = 'IDLE'
    node.command_arbiter = FakeArbiter()
    node.command_sign = (1.0, 1.0, 1.0)
    node.pub_ready = FakePublisher()
    node.pub_motion_armed = FakePublisher()
    node.pub_manual_active = FakePublisher()
    node.pub_ultrasonic_ready = FakePublisher()
    node.statuses = []
    node.publish_status = node.statuses.append
    node.logger = FakeLogger()
    node.get_logger = lambda: node.logger
    node.tx_scheduler = ImmediateScheduler(node)
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


def test_hello_waits_until_serial_startup_settle_has_elapsed(monkeypatch):
    node = bridge_for_unit_test()
    now = [10.0]
    node.serial_ready_at = 10.5
    node.serial_input_drained = False
    node.rx_buffer = bytearray()
    node.max_rx_buffer = 4096
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node.send_hello()
    assert node.ser.writes == []
    now[0] = 10.5
    node.send_hello()
    assert node.ser.writes == []
    node.read_serial()
    assert node.serial_input_drained is True
    node.send_hello()
    assert node.ser.writes == [b'@HELLO,2,abcdef0123456789\n']


def test_serial_boot_noise_is_drained_before_frames_are_parsed(monkeypatch):
    node = bridge_for_unit_test()
    now = [10.0]
    node.serial_ready_at = 10.5
    node.serial_input_drained = False
    node.rx_buffer = bytearray()
    node.max_rx_buffer = 4096
    node.ser.incoming.extend(b'HELLO,2,partial\nERR,UNKNOWN_COMMAND\n')
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node.read_serial()
    assert node.ser.in_waiting == 0
    assert node.active_fault is None
    now[0] = 10.5
    node.ser.incoming.extend(b'E,0,0,0,0\n')
    node.read_serial()
    assert node.ser.in_waiting == 0
    assert node.serial_input_drained is True
    assert node.active_fault is None


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
    assert node.motion_armed is False
    assert node._arm_motion('test') is True
    assert node.motion_armed is True

    node._handle_serial_line('ERR,HEARTBEAT_TIMEOUT')
    assert node.active_fault == 'ERR,HEARTBEAT_TIMEOUT'
    assert node.servo_attached is False
    node._handle_serial_line('ACK,SERVO_ATTACH')
    node.publish_hardware_state()
    assert node.pub_ready.messages[-1].data is False


@pytest.mark.parametrize(
    'code', ('UNKNOWN_COMMAND', 'RX_QUEUE_OVERFLOW', 'UART_RX_ERROR'))
def test_prehello_uart_rejection_is_quarantined_until_valid_hello(
        monkeypatch, code):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])

    node._handle_serial_line(f'ERR,{code}')
    assert node.active_fault is None
    assert node.previous_session_faults == [code]
    complete_startup(node, now[0])
    assert f'INFO,PREVIOUS_SESSION_FAULT:{code}' in node.statuses
    assert all(node.hardware_ready_conditions(now[0]).values())


@pytest.mark.parametrize(
    'code', ('UNKNOWN_COMMAND', 'RX_QUEUE_OVERFLOW', 'UART_RX_ERROR'))
def test_uart_rejection_after_hello_remains_fail_closed(monkeypatch, code):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    complete_startup(node, now[0])

    node._handle_serial_line(f'ERR,{code}')
    assert node.active_fault == f'ERR,{code}'
    assert node.servo_attached is False
    assert node.command_arbiter.force_zero_calls == [now[0]]


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


def test_delayed_and_previous_session_heartbeat_ack_are_stale(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    node.send_hello()
    node._handle_serial_line(
        f'ACK,{node.protocol.hello_ack_value(node.session_id)}')
    token = node.ser.writes[-1].decode().strip().removeprefix('@HB,')
    now[0] += node.heartbeat_ack_timeout + 0.001
    node._handle_serial_line(f'ACK,{token}')
    assert node.last_heartbeat_ack_time == 0.0
    assert node.heartbeat_stats['stale_ack_count'] == 1

    node._handle_serial_line('ACK,0000000000000000:99')
    assert node.heartbeat_stats['stale_ack_count'] == 2


def test_timeout_recovers_communication_but_never_rearms_motion(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge_module.secrets, 'token_hex',
                        lambda _length: '1111111111111111')
    complete_startup(node, now[0])
    assert node._arm_motion('test') is True
    old_session = node.session_id

    node._handle_serial_line('ERR,HEARTBEAT_TIMEOUT')
    assert node.active_fault == 'ERR,HEARTBEAT_TIMEOUT'
    assert node.communication_recovering is True
    assert node.session_id != old_session
    assert node.ser.writes[-1].startswith(b'@HELLO,2,1111111111111111')

    node._handle_serial_line(
        f'ACK,{node.protocol.hello_ack_value(node.session_id)}')
    for _ in range(node.heartbeat_recovery_ack_count):
        token = node.ser.writes[-1].decode().strip().removeprefix('@HB,')
        node._handle_serial_line(f'ACK,{token}')
        if node.communication_recovering:
            now[0] += node.heartbeat_period
            node.send_heartbeat()

    assert node.communication_recovered is True
    assert node.communication_recovering is False
    assert node.motion_armed is False
    assert node.active_fault is None
    assert node.zero_command_acknowledged is False
    assert node.servo_attached is False
    # Recovery continues through a session-bound zero and servo attach. The
    # arbiter was force-cleared, so no pre-fault velocity can be replayed.
    node.send_zero_velocity_probe()
    node._handle_serial_line(
        f'ACK,{node.protocol.zero_velocity_ack_value(node.session_id)}')
    node._handle_serial_line('ACK,SERVO_ATTACH')
    assert node.zero_command_acknowledged is True
    assert node.servo_attached is True
    assert all(node.hardware_ready_conditions(now[0]).values())
    assert len(node.command_arbiter.force_zero_calls) >= 2
    node.send_velocity_loop()
    assert node.ser.writes[-1].startswith(b'@V,0.000,0.000,0.000')
    assert 'INFO,COMMUNICATION_RECOVERED:MOTION_REARM_REQUIRED' in node.statuses


def test_disarmed_startup_timeout_recovers_without_latching_fault(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge_module.secrets, 'token_hex',
                        lambda _length: '2222222222222222')
    complete_startup(node, now[0])

    node._handle_serial_line('ERR,HEARTBEAT_TIMEOUT')

    assert node.motion_armed is False
    assert node.active_fault is None
    assert node.communication_recovering is True
    assert 'ERR,HEARTBEAT_TIMEOUT' not in node.statuses
    assert ('WARN,DISARMED_COMMUNICATION_RECOVERY:HEARTBEAT_TIMEOUT'
            in node.statuses)


def test_disarmed_host_ack_timeout_is_also_non_latching(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    monkeypatch.setattr(bridge_module.secrets, 'token_hex',
                        lambda _length: '3333333333333333')
    complete_startup(node, now[0])
    node.send_heartbeat(force=True)
    now[0] += node.heartbeat_ack_timeout + 0.001

    node.send_heartbeat(force=True)

    assert node.active_fault is None
    assert node.motion_armed is False
    assert node.communication_recovering is True
    assert ('WARN,DISARMED_COMMUNICATION_RECOVERY:HEARTBEAT_ACK_TIMEOUT'
            in node.statuses)


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


def test_shutdown_zero_survives_repeated_sigint_during_logging():
    node = bridge_for_unit_test()

    class InterruptedLogger(FakeLogger):
        def info(self, message, **kwargs):
            raise KeyboardInterrupt

    node.logger = InterruptedLogger()
    node.get_logger = lambda: node.logger
    node.shutdown_stop()
    assert node.ser.writes == [b'@V,0.000,0.000,0.000\n']


def test_serial_port_is_opened_exclusively():
    text = Path(bridge_module.__file__).read_text(encoding='utf-8')
    assert 'exclusive=True' in text


def test_heartbeat_producer_is_independent_of_ros_callback_execution():
    bridge_source = Path(bridge_module.__file__).read_text(encoding='utf-8')
    mvp_source = Path(
        bridge_module.__file__).with_name('mvp_stm32_bridge_node.py').read_text(
            encoding='utf-8')

    assert 'MutuallyExclusiveCallbackGroup' in bridge_source
    assert 'self.serial_callback_group' in bridge_source
    assert 'target=self._heartbeat_producer_loop' in bridge_source
    assert "name=f'{self.role}-heartbeat-producer'" in bridge_source
    assert 'send_heartbeat(force=True, scheduled_due=next_tick)' in bridge_source
    assert 'target=self._serial_reader_loop' in bridge_source
    assert "name=f'{self.role}-uart-reader'" in bridge_source
    assert 'self.create_timer(\n            0.02, self.read_serial' not in bridge_source
    assert 'self.heartbeat_period, self.send_heartbeat,' not in bridge_source
    assert 'callback_group=self.serial_callback_group' in bridge_source
    assert 'SingleThreadedExecutor()' in bridge_source
    assert 'SingleThreadedExecutor()' in mvp_source
    assert 'MultiThreadedExecutor' not in bridge_source
    assert 'MultiThreadedExecutor' not in mvp_source
    assert 'executor.spin()' in mvp_source


def test_dedicated_producer_does_not_skip_boundary_ticks(monkeypatch):
    node = bridge_for_unit_test()
    now = [1.0]

    class FourTickEvent:
        def __init__(self):
            self.wait_count = 0

        def is_set(self):
            return self.wait_count >= 4

        def wait(self, delay):
            now[0] += max(0.0, delay)
            self.wait_count += 1
            return False

    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    node.hello_acknowledged = True
    node.communication_recovering = True
    node.next_heartbeat_due = now[0]
    node.heartbeat_thread_stop = FourTickEvent()

    node._heartbeat_producer_loop()

    heartbeats = [frame for frame in node.ser.writes
                  if frame.startswith(b'@HB,')]
    assert len(heartbeats) == 4


def test_grip_is_blocked_until_full_attach_ack(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    node._send_grip('grip')
    assert node.ser.writes == []
    complete_startup(node, now[0])
    node._send_grip('grip')
    assert node.ser.writes[-1].startswith(b'@S,attach,')
    assert node._arm_motion('test') is True
    node._send_grip('grip')
    assert node.ser.writes[-1] == b'@S,grip\n'


def test_live_robot_state_arms_and_idle_disarms(monkeypatch):
    node = bridge_for_unit_test()
    now = [100.0]
    node.hello_started_at = now[0]
    monkeypatch.setattr(bridge_module.time, 'monotonic', lambda: now[0])
    complete_startup(node, now[0])

    node.robot_state_cb(String(data='APPROACH'))
    assert node.motion_armed is True
    assert node.motion_arm_source == 'robot_state:APPROACH'

    node.robot_state_cb(String(data='IDLE'))
    assert node.motion_armed is False
    assert node.ser.writes[-1] == b'@V,0.000,0.000,0.000\n'


def test_physical_profiles_select_opposite_attach_pulses():
    assert servo_attach_pulses_for('robot-1') == (400, 2600)
    assert servo_attach_pulses_for('robot-2') == (2600, 400)
