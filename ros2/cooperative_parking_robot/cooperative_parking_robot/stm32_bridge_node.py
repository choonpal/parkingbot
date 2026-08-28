#!/usr/bin/env python3
"""ROS2 ↔ STM32 UART bridge.

==================================================
stm32_bridge_node.py (role: front/rear)
==================================================
ROS2 ↔ STM32 UART 변환 전담.
역기구학은 수행하지 않음 (STM32 motor_pid_task가 함).

WiFi 지연 대비 (Master→Rear cmd_vel이 WiFi 경유):
  cmd_vel 수신 시각을 기록하고, 200ms 이상 갱신 없으면
  속도를 단계적으로 감쇠(decay)하여 STM32에 전달.
  → 순간 끊김에도 급정지 대신 부드러운 감속.
  (STM32 자체 워치독 300ms는 최후 안전망으로 그대로 유지)

입력:
  /{role}/cmd_vel (TwistStamped) → UART "@V,vx,vy,w"
  /{role}/grip_command (String) → UART "@S,grip/release"
출력:
  /{role}/wheel_odom (Odometry) ← UART "E,fl,fr,rl,rr"
      순수 엔코더 dead-reckoning (진단용). pose_fusion_node가 이걸
      CCTV+ArUco 절대측정과 융합해 최종 /{role}/odom을 다시 발행한다
      (v1.6 — 이전엔 이 노드가 /{role}/odom을 직접 발행했음).
      twist.linear.x/y·angular.z = 이번 UART 수신 구간의 body-frame
      "변위"(속도 아님) — pose_fusion_node와의 내부 계약.
  /{role}/lift_status (String) ← UART "LIFT,..."
  /{role}/ultrasonic_left|right (Range) ← UART "U,L|R,<mm|TIMEOUT>"
"""

from collections import deque
import math
import secrets
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String, Bool

from cooperative_parking_robot.command_qos import CMD_VEL_QOS
from cooperative_parking_robot.latest_qos import (
    SAFETY_STATE_QOS,
    SENSOR_LATEST_QOS,
    STATE_LATEST_QOS,
)
from cooperative_parking_robot.uart_protocol import UART_BAUD_RATE, UartProtocol
from cooperative_parking_robot.encoder_odometry import EncoderOdometry
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
from cooperative_parking_robot.hardware_profile import (
    command_sign_for,
    resolve_hardware_profile,
    servo_attach_pulses_for,
)
from cooperative_parking_robot.manual_control import VelocityCommandArbiter

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False


class Stm32BridgeNode(Node):
    def __init__(self, **kwargs):
        super().__init__('stm32_bridge_node', **kwargs)

        self.declare_parameter('role', 'front')
        self.declare_parameter('hardware_profile', 'auto')
        self.declare_parameter('serial_port', '/dev/ttyACM0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('enable_serial', True)
        self.declare_parameter('require_serial', False)
        # BOM의 100 mm 메카넘 휠 기준 명목 반경. 실물 유효반경은 실측 후 갱신.
        self.declare_parameter('wheel_radius', 0.05)
        self.declare_parameter('encoder_ppr', 5182.0)
        self.declare_parameter('max_delta_ticks', 2591)
        self.declare_parameter('max_linear_mps', 0.25)
        self.declare_parameter('max_angular_rps', 1.0)
        # 로봇 섀시 축간/윤거의 절반 — 실측 후 확정 (이전엔 파라미터로 노출되지
        # 않아 EncoderOdometry 기본값 0.10/0.10에 항상 고정되는 버그였음)
        self.declare_parameter('lx', 0.10)
        self.declare_parameter('ly', 0.10)
        # STM32 HC-SR04 측정값을 ROS Range로 변환하는 계약.
        self.declare_parameter('ultrasonic_min_range_m', 0.02)
        self.declare_parameter('ultrasonic_max_range_m', 4.00)
        self.declare_parameter('ultrasonic_field_of_view_rad', 0.261799)
        self.declare_parameter('ultrasonic_frame_timeout_s', 0.50)
        self.declare_parameter('require_ultrasonic_for_ready', True)
        self.declare_parameter('command_source_timeout_s', 0.25)
        self.declare_parameter('command_future_tolerance_s', 0.10)
        self.declare_parameter('clock_reject_fault_count', 3)
        # ACK가 유실되어도 UART를 flood하지 않는 attach 재시도 주기.
        self.declare_parameter('servo_attach_retry_interval_s', 0.75)
        self.declare_parameter('hello_retry_interval_s', 0.25)
        self.declare_parameter('hello_handshake_timeout_s', 2.0)
        self.declare_parameter('heartbeat_ack_timeout_s', 0.30)
        self.declare_parameter('uart_frame_fault_count', 3)
        self.declare_parameter('uart_frame_fault_window_s', 1.0)

        self.role = self.get_parameter('role').value
        if self.role not in ('front', 'rear'):
            raise ValueError("role must be 'front' or 'rear'")
        # role은 토픽 namespace이고 profile은 물리 기체다. 역할을 바꿔
        # 단독 시험해도 실측 배선 보정이 함께 바뀌면 안 된다.
        self.hardware_profile = resolve_hardware_profile(
            self.role, self.get_parameter('hardware_profile').value)
        self.command_sign = command_sign_for(self.hardware_profile)
        self.servo_attach_pulses = servo_attach_pulses_for(
            self.hardware_profile)
        self.servo_attach_retry_interval = float(
            self.get_parameter('servo_attach_retry_interval_s').value)
        if self.servo_attach_retry_interval < 0.5:
            raise ValueError(
                'servo_attach_retry_interval_s must be at least 0.5')
        self.hello_retry_interval = float(
            self.get_parameter('hello_retry_interval_s').value)
        self.hello_handshake_timeout = float(
            self.get_parameter('hello_handshake_timeout_s').value)
        self.heartbeat_ack_timeout = float(
            self.get_parameter('heartbeat_ack_timeout_s').value)
        self.uart_frame_fault_count = int(
            self.get_parameter('uart_frame_fault_count').value)
        self.uart_frame_fault_window = float(
            self.get_parameter('uart_frame_fault_window_s').value)
        if (self.hello_retry_interval <= 0.0 or
                self.hello_handshake_timeout <= self.hello_retry_interval):
            raise ValueError('invalid HELLO retry/timeout parameters')
        if not 0.0 < self.heartbeat_ack_timeout <= 0.30:
            raise ValueError(
                'heartbeat_ack_timeout_s must be in (0, STM32 0.30s]')
        if (self.uart_frame_fault_count <= 0 or
                self.uart_frame_fault_window <= 0.0):
            raise ValueError('invalid UART frame fault threshold')
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_rps').value)
        self.protocol = UartProtocol()
        self.serial_baud = int(self.get_parameter('serial_baud').value)
        if self.serial_baud != UART_BAUD_RATE:
            raise ValueError(
                f'serial_baud must match firmware ({UART_BAUD_RATE})')
        self.ultrasonic_min_range = float(
            self.get_parameter('ultrasonic_min_range_m').value)
        self.ultrasonic_max_range = float(
            self.get_parameter('ultrasonic_max_range_m').value)
        self.ultrasonic_fov = float(
            self.get_parameter('ultrasonic_field_of_view_rad').value)
        self.ultrasonic_frame_timeout = float(
            self.get_parameter('ultrasonic_frame_timeout_s').value)
        self.require_ultrasonic_for_ready = bool(
            self.get_parameter('require_ultrasonic_for_ready').value)
        self.command_source_timeout = float(
            self.get_parameter('command_source_timeout_s').value)
        self.command_future_tolerance = float(
            self.get_parameter('command_future_tolerance_s').value)
        self.clock_reject_fault_count = int(
            self.get_parameter('clock_reject_fault_count').value)
        if self.clock_reject_fault_count <= 0:
            raise ValueError('clock_reject_fault_count must be positive')
        self.clock_reject_count = {'auto': 0, 'manual': 0}
        if not (0.0 < self.ultrasonic_min_range < self.ultrasonic_max_range):
            raise ValueError('ultrasonic min/max range is invalid')
        if self.ultrasonic_frame_timeout <= 0.0:
            raise ValueError('ultrasonic_frame_timeout_s must be positive')
        self.command_stamp_gate = StampGate(
            self.command_source_timeout, self.command_future_tolerance)
        self.manual_command_stamp_gate = StampGate(
            self.command_source_timeout, self.command_future_tolerance)
        self.odom_calc = EncoderOdometry(
            wheel_radius=self.get_parameter('wheel_radius').value,
            ppr=self.get_parameter('encoder_ppr').value,
            lx=self.get_parameter('lx').value,
            ly=self.get_parameter('ly').value,
            max_delta_ticks=self.get_parameter('max_delta_ticks').value,
            axis_sign=self.command_sign)

        # 수동 모드는 auto cmd보다 우선하며, 수동 송신이 끊겨도 auto로
        # 되돌아가지 않고 0속도를 유지한다.
        self.command_arbiter = VelocityCommandArbiter(
            manual_timeout_s=self.command_source_timeout)
        self.session_id = secrets.token_hex(8)
        self.hello_started_at = time.monotonic()
        self.last_hello_request_time = None
        self.hello_acknowledged = False
        self.heartbeat_sequence = 0
        self.outstanding_heartbeats = {}
        self.last_heartbeat_ack_time = 0.0
        self.last_zero_request_time = None
        self.zero_command_sent = False
        self.zero_command_acknowledged = False
        self.previous_session_faults = []
        self.active_fault = None
        self.transport_fault = None
        self.invalid_frame_times = deque()
        self.last_ultrasonic_valid = {'left': 0.0, 'right': 0.0}
        self.ultrasonic_stale_reported = False
        self.estop_latched = False
        # 실제 배선 감지가 아니라 STM32 RAM과 RPi pulse 기준이
        # ACK,SERVO_ATTACH로 동기화됐는지를 나타내는 protocol 상태.
        self.servo_attached = False
        self.servo_attach_blocked = False
        self.servo_attach_requested = False
        self.last_servo_attach_request_time = None
        self.hardware_ready = False
        # non-blocking serial.readline()은 newline 도착 전 부분 프레임을
        # 반환할 수 있으므로 직접 byte buffer를 유지한다.
        self.rx_buffer = bytearray()
        self.max_rx_buffer = 4096

        # ===== 시리얼 연결 =====
        self.ser = None
        self.enable_serial = bool(
            self.get_parameter('enable_serial').value)
        require_serial = bool(
            self.get_parameter('require_serial').value)
        if require_serial and not self.enable_serial:
            raise ValueError(
                'require_serial=true requires enable_serial=true')
        if not self.enable_serial:
            self.get_logger().warn(
                f'[{self.role}] serial 연결 비활성화 — smoke mode')
        elif SERIAL_OK:
            try:
                self.ser = serial.Serial(
                    self.get_parameter('serial_port').value,
                    self.serial_baud,
                    timeout=0.0,
                    write_timeout=0.05,
                    exclusive=True)
                self.ser.reset_input_buffer()
                self.get_logger().info(f'[{self.role}] STM32 연결')
            except Exception as e:
                self.get_logger().error(f'STM32 연결 실패: {e}')
                if require_serial:
                    raise RuntimeError(
                        f'[{self.role}] required STM32 serial unavailable') from e
        elif require_serial:
            raise RuntimeError('pyserial is required when require_serial=true')

        # ===== 구독 =====
        self.create_subscription(
            TwistStamped, f'/{self.role}/cmd_vel', self.cmd_vel_cb,
            CMD_VEL_QOS)
        self.create_subscription(String, f'/{self.role}/grip_command',
                                 self.grip_cb, 10)
        self.create_subscription(
            TwistStamped, f'/{self.role}/manual_cmd_vel',
            self.manual_cmd_vel_cb, CMD_VEL_QOS)
        self.create_subscription(
            String, f'/{self.role}/manual_grip_command',
            self.manual_grip_cb, 10)
        self.create_subscription(
            Bool, f'/{self.role}/manual_enable',
            self.manual_enable_cb, 10)
        self.create_subscription(Bool, '/emergency_stop',
                                 self.estop_cb, STATE_LATEST_QOS)

        # ===== 발행 =====
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.role}/wheel_odom', SENSOR_LATEST_QOS)
        self.pub_lift = self.create_publisher(
            String, f'/{self.role}/lift_status', 10)
        self.pub_hw = self.create_publisher(
            String, f'/{self.role}/hardware_status', SAFETY_STATE_QOS)
        self.pub_ready = self.create_publisher(
            Bool, f'/{self.role}/hardware_ready', SAFETY_STATE_QOS)
        # 수동 제어 요청이 실제로 이 bridge까지 도착했는지 시험 제어기가
        # 확인할 수 있는 명시적 ACK.  주기 발행하므로 늦게 연결된 peer도 받는다.
        self.pub_manual_active = self.create_publisher(
            Bool, f'/{self.role}/manual_active', 10)
        self.pub_ultrasonic = {
            side: self.create_publisher(
                Range, f'/{self.role}/ultrasonic_{side}',
                SENSOR_LATEST_QOS)
            for side in ('left', 'right')
        }
        self.pub_ultrasonic_status = self.create_publisher(
            String, f'/{self.role}/ultrasonic_status', 10)
        # 바퀴별 진단. E frame은 합쳐진 odom만 주므로 개별 모터가 목표를
        # 못 따라가는 상황은 여기서만 보인다.
        self.pub_motor_diag = self.create_publisher(
            String, f'/{self.role}/motor_diagnostics', 10)

        # ===== 루프 =====
        self.create_timer(0.02, self.read_serial)       # UART 수신
        self.create_timer(0.02, self.send_velocity_loop)  # 속도 송신 50Hz (감쇠 포함)
        self.create_timer(0.1, self.send_heartbeat)     # heartbeat 10Hz
        self.create_timer(0.1, self.send_servo_attach)  # bounded startup retry
        self.create_timer(0.05, self.startup_handshake_tick)
        self.create_timer(0.2, self.publish_hardware_state)

        # DTR reset 여부와 관계없이 HELLO가 Linux communication session의
        # 유일한 경계다. HB/V/servo는 matching HELLO ACK 전에는 보내지 않는다.
        self.send_hello()

        self.get_logger().info(
            f'stm32_bridge_node 시작 [{self.role}] '
            f'hardware={self.hardware_profile} sign={self.command_sign}')

    # ===== ROS2 → STM32 =====
    def cmd_vel_cb(self, msg):
        # 역기구학 안 함. 최신 명령 저장 (송신은 send_velocity_loop가 담당)
        stamp_ns = stamp_to_ns(msg.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        accepted, reason = self.command_stamp_gate.accept(stamp_ns, now_ns)
        if not accepted:
            self._report_stamp_rejection('auto', reason, stamp_ns, now_ns)
            return
        self.clock_reject_count['auto'] = 0
        if msg.header.frame_id not in ('', f'{self.role}_base'):
            self.get_logger().warn(
                f'cmd_vel frame rejected: {msg.header.frame_id}')
            return
        values = (float(msg.twist.linear.x), float(msg.twist.linear.y),
                  float(msg.twist.angular.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('NaN/Inf cmd_vel 거부')
            return
        vx = max(-self.max_linear, min(self.max_linear, values[0]))
        vy = max(-self.max_linear, min(self.max_linear, values[1]))
        w = max(-self.max_angular, min(self.max_angular, values[2]))
        self.command_arbiter.update_auto((vx, vy, w), time.monotonic())

    def manual_cmd_vel_cb(self, msg):
        if not self.command_arbiter.manual_enabled:
            return
        stamp_ns = stamp_to_ns(msg.header.stamp)
        now_ns = self.get_clock().now().nanoseconds
        accepted, reason = self.manual_command_stamp_gate.accept(
            stamp_ns, now_ns)
        if not accepted:
            self._report_stamp_rejection('manual', reason, stamp_ns, now_ns)
            return
        self.clock_reject_count['manual'] = 0
        if msg.header.frame_id not in ('', f'{self.role}_base'):
            self.get_logger().warn(
                f'manual cmd frame rejected: {msg.header.frame_id}')
            return
        values = (float(msg.twist.linear.x), float(msg.twist.linear.y),
                  float(msg.twist.angular.z))
        if not all(math.isfinite(value) for value in values):
            self.get_logger().error('NaN/Inf manual cmd_vel 거부')
            return
        command = (
            max(-self.max_linear, min(self.max_linear, values[0])),
            max(-self.max_linear, min(self.max_linear, values[1])),
            max(-self.max_angular, min(self.max_angular, values[2])))
        self.command_arbiter.update_manual(command, time.monotonic())

    def _report_stamp_rejection(self, source, reason, stamp_ns, now_ns):
        delta_s = (int(now_ns) - int(stamp_ns)) * 1.0e-9
        self.get_logger().warn(
            f'{source} cmd_vel rejected: {reason} '
            f'msg_stamp_ns={stamp_ns} local_now_ns={now_ns} '
            f'age_s={delta_s:+.6f}', throttle_duration_sec=1.0)
        if reason not in ('STALE_STAMP', 'FUTURE_STAMP'):
            return
        self.clock_reject_count[source] += 1
        if self.clock_reject_count[source] == self.clock_reject_fault_count:
            self.publish_status(
                f'ERR,CLOCK_SKEW:{source}:{reason}:'
                f'age_s={delta_s:+.6f}:count='
                f'{self.clock_reject_count[source]}')

    def manual_enable_cb(self, msg):
        was_enabled = self.command_arbiter.manual_enabled
        self.command_arbiter.set_manual_enabled(msg.data, time.monotonic())
        if msg.data and not was_enabled:
            self.publish_status('INFO,MANUAL_MODE_ON')
            self.get_logger().warn(f'[{self.role}] manual override ON')
        elif not msg.data and was_enabled:
            self.publish_status('INFO,MANUAL_MODE_OFF')
            self.get_logger().info(f'[{self.role}] manual override OFF')
        self.pub_manual_active.publish(Bool(
            data=self.command_arbiter.manual_enabled))

    def send_velocity_loop(self):
        """50Hz로 STM32에 속도 송신. cmd가 오래되면 감쇠."""
        if (self.estop_latched or self.active_fault or self.transport_fault or
                not self.zero_command_acknowledged or
                not self._heartbeat_fresh(time.monotonic())):
            # STM32는 ESTOP을 전원 재인가까지 latch한다. 이후 V 프레임을
            # 계속 보내면 ESTOP_LATCHED 오류만 반복되므로 송신을 멈춘다.
            return
        vx, vy, w = self.command_arbiter.output(time.monotonic())
        sx, sy, sw = self.command_sign
        cmd = self.protocol.encode_velocity(vx * sx, vy * sy, w * sw)
        self._write(cmd)

    def grip_cb(self, msg):
        if self.command_arbiter.manual_enabled:
            return
        self._send_grip(msg.data)

    def manual_grip_cb(self, msg):
        if not self.command_arbiter.manual_enabled:
            return
        self._send_grip(msg.data)

    def _send_grip(self, action):
        if self.estop_latched or self.active_fault or self.transport_fault:
            self.get_logger().warn('ESTOP latch 상태에서 그리퍼 명령 거부')
            return
        if not self.servo_attached:
            self.get_logger().warn(
                'servo attach ACK 전 그리퍼 명령 거부',
                throttle_duration_sec=1.0)
            self.publish_status('WARN,SERVO_NOT_READY')
            return
        try:
            cmd = self.protocol.encode_servo(action)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self._write(cmd)

    def send_servo_attach(self):
        """Request STM32/RPi servo pulse synchronization at a bounded rate."""
        if (not self.ser or self.servo_attached or self.estop_latched or
                self.servo_attach_blocked or self.active_fault or
                self.transport_fault or not self.hello_acknowledged or
                not self.zero_command_acknowledged or
                not self._heartbeat_fresh(time.monotonic())):
            return
        now = time.monotonic()
        if (self.last_servo_attach_request_time is not None and
                now - self.last_servo_attach_request_time <
                self.servo_attach_retry_interval):
            return
        self.last_servo_attach_request_time = now
        pulse1, pulse2 = self.servo_attach_pulses
        cmd = self.protocol.encode_servo_attach(pulse1, pulse2)
        if self._write(cmd):
            self.servo_attach_requested = True
            self.get_logger().info(
                f'[{self.role}] servo attach 요청 '
                f'hardware={self.hardware_profile} '
                f'pulses=({pulse1},{pulse2})')
            self.publish_status(
                f'INFO,SERVO_ATTACH_REQUEST:{pulse1}:{pulse2}')

    def shutdown_stop(self):
        """종료 직전 0 속도를 보낸다. 실패해도 종료를 막지 않는다."""
        if self.ser is None:
            return
        try:
            self._write(self.protocol.encode_velocity(0.0, 0.0, 0.0))
            self.get_logger().info(f'[{self.role}] 종료 — 0 속도 전송')
        except Exception as exc:
            self.get_logger().warn(f'[{self.role}] 종료 정지 전송 실패: {exc}')

    def estop_cb(self, msg):
        if msg.data and not self.estop_latched:
            self.estop_latched = True
            self.servo_attached = False
            self.servo_attach_blocked = True
            self.command_arbiter.force_zero(time.monotonic())
            self._write(self.protocol.encode_estop())
            self.publish_status('ESTOP')

    def send_hello(self):
        """Send the versioned session boundary; no other startup command leads."""
        if (not self.ser or self.hello_acknowledged or self.estop_latched or
                self.active_fault or self.transport_fault):
            return
        now = time.monotonic()
        if (self.last_hello_request_time is not None and
                now - self.last_hello_request_time < self.hello_retry_interval):
            return
        self.last_hello_request_time = now
        if self._write(self.protocol.encode_hello(self.session_id)):
            self.publish_status(
                f'INFO,HELLO_SENT:{self.protocol.hello_ack_value(self.session_id)}')

    def send_zero_velocity_probe(self):
        """Initialize the MCU command watchdog with a session-bound zero ACK."""
        now = time.monotonic()
        if (not self.ser or not self.hello_acknowledged or
                not self._heartbeat_fresh(now) or
                self.zero_command_acknowledged or self.estop_latched or
                self.active_fault or self.transport_fault):
            return
        if (self.last_zero_request_time is not None and
                now - self.last_zero_request_time < 0.20):
            return
        self.last_zero_request_time = now
        if self._write(self.protocol.encode_zero_velocity(self.session_id)):
            self.zero_command_sent = True

    def startup_handshake_tick(self):
        """Advance only through matching, session-bound startup evidence."""
        if (not self.ser or self.estop_latched or self.active_fault or
                self.transport_fault):
            return
        now = time.monotonic()
        if not self.hello_acknowledged:
            if now - self.hello_started_at > self.hello_handshake_timeout:
                self._latch_fault('ERR,PROTOCOL_HANDSHAKE_TIMEOUT')
                return
            self.send_hello()
            return
        if not self._heartbeat_fresh(now):
            self.send_heartbeat()
            return
        if not self.zero_command_acknowledged:
            self.send_zero_velocity_probe()
            return
        if not self.servo_attached:
            self.send_servo_attach()

    def send_heartbeat(self):
        if (not self.ser or not self.hello_acknowledged or
                self.estop_latched or self.active_fault or
                self.transport_fault):
            return
        self.heartbeat_sequence += 1
        token = f'{self.session_id}:{self.heartbeat_sequence}'
        sent_at = time.monotonic()
        if self._write(self.protocol.encode_heartbeat(token)):
            self.outstanding_heartbeats[token] = sent_at
            if len(self.outstanding_heartbeats) > 8:
                oldest = min(
                    self.outstanding_heartbeats,
                    key=self.outstanding_heartbeats.get)
                del self.outstanding_heartbeats[oldest]

    def _heartbeat_fresh(self, now=None):
        now = time.monotonic() if now is None else now
        return (self.hello_acknowledged and
                self.last_heartbeat_ack_time > 0.0 and
                now - self.last_heartbeat_ack_time <
                self.heartbeat_ack_timeout)

    def _latch_fault(self, status, *, transport=False):
        if not status.startswith(('ERR,', 'ESTOP')):
            raise ValueError('latched hardware status must be ERR or ESTOP')
        if transport:
            self.transport_fault = status
        if self.active_fault is None:
            self.active_fault = status
        self.hardware_ready = False
        self.servo_attached = False
        self.servo_attach_blocked = True
        self.command_arbiter.force_zero(time.monotonic())
        self.publish_status(status)

    def _latch_transport_fault(self, status, detail):
        self.get_logger().error(f'{status}: {detail}')
        self._latch_fault(status, transport=True)
        serial_handle = self.ser
        self.ser = None
        if serial_handle is not None and hasattr(serial_handle, 'close'):
            try:
                serial_handle.close()
            except Exception:
                pass

    def _write(self, cmd):
        if self.ser:
            try:
                payload = cmd.encode('ascii')
                written = self.ser.write(payload)
                if written != len(payload):
                    self._latch_transport_fault(
                        'ERR,UART_PARTIAL_WRITE',
                        f'wrote {written}/{len(payload)} bytes')
                    return False
                return True
            except Exception as e:
                self._latch_transport_fault('ERR,UART_WRITE', str(e))
        return False

    # ===== STM32 → ROS2 =====
    def read_serial(self):
        """UART byte stream을 newline 단위로 조립해 완전한 프레임만 파싱한다."""
        if not self.ser:
            return
        try:
            waiting = int(self.ser.in_waiting)
            if waiting > 0:
                self.rx_buffer.extend(self.ser.read(min(waiting, 1024)))
        except Exception as exc:
            self._latch_transport_fault('ERR,UART_READ', str(exc))
            return

        if len(self.rx_buffer) > self.max_rx_buffer:
            self.rx_buffer.clear()
            self._latch_transport_fault(
                'ERR,UART_RX_OVERFLOW',
                f'buffer exceeded {self.max_rx_buffer} bytes')
            return

        processed = 0
        while processed < 20:
            newline = self.rx_buffer.find(b'\n')
            if newline < 0:
                break
            raw = bytes(self.rx_buffer[:newline])
            del self.rx_buffer[:newline + 1]
            processed += 1
            if len(raw) > 256:
                self._record_invalid_frame('oversize frame')
                continue
            try:
                line = raw.rstrip(b'\r').decode('ascii', errors='strict').strip()
            except UnicodeDecodeError:
                self._record_invalid_frame('non-ASCII frame')
                continue
            if not line:
                continue
            self._handle_serial_line(line)
            if self.transport_fault:
                break

    def _record_invalid_frame(self, reason):
        now = time.monotonic()
        self.invalid_frame_times.append(now)
        while (self.invalid_frame_times and
               now - self.invalid_frame_times[0] >
               self.uart_frame_fault_window):
            self.invalid_frame_times.popleft()
        self.get_logger().warn(
            f'잘못된 STM32 frame: {reason} '
            f'({len(self.invalid_frame_times)}/'
            f'{self.uart_frame_fault_count})')
        if len(self.invalid_frame_times) >= self.uart_frame_fault_count:
            self._latch_transport_fault(
                'ERR,UART_FRAME_CORRUPTION', reason)

    def _handle_serial_line(self, line):
        parsed = self.protocol.parse(line)
        if parsed is None:
            self._record_invalid_frame(line[:80])
            return

        if parsed['type'] == 'encoder':
            odom = self.odom_calc.update(parsed['values'])
            if odom.get('discontinuity'):
                self.get_logger().warn(
                    f'[{self.role}] 엔코더 카운터 불연속 감지 '
                    f'— 이번 주기 모션 폐기, 재동기화함')
            self.publish_odom(odom)
        elif parsed['type'] == 'ultrasonic':
            self.publish_ultrasonic(parsed)
        elif parsed['type'] == 'lift':
            self.pub_lift.publish(String(data=parsed['status']))
        elif parsed['type'] == 'ack':
            self._handle_ack(parsed['value'])
        elif parsed['type'] == 'error':
            self._handle_error(parsed['code'])
        elif parsed['type'] == 'telemetry':
            # E/U frame가 ROS 데이터의 기준이며 T는 수동 시험기용 진단값이다.
            # 바퀴별로 목표를 못 따라가는 모터를 찾을 수 있도록 그대로 발행한다.
            self.publish_motor_diagnostics(parsed)

    def _handle_ack(self, value):
        now = time.monotonic()
        expected_hello = self.protocol.hello_ack_value(self.session_id)
        if value.startswith('HELLO:'):
            if (value != expected_hello or self.active_fault or
                    self.estop_latched):
                self.get_logger().warn(
                    f'current session과 불일치하는 HELLO ACK 무시: {value}')
                self.publish_status('WARN,IGNORED_HELLO_ACK')
                return
            if not self.hello_acknowledged:
                self.hello_acknowledged = True
                self.get_logger().info(
                    f'[{self.role}] protocol v2 HELLO ACK '
                    f'session={self.session_id}')
                self.publish_status(f'ACK,{value}')
                for code in self.previous_session_faults:
                    self.publish_status(
                        f'INFO,PREVIOUS_SESSION_FAULT:{code}')
                self.previous_session_faults.clear()
                self.send_heartbeat()
            return

        sent_at = self.outstanding_heartbeats.pop(value, None)
        if sent_at is not None:
            if (not self.hello_acknowledged or
                    now - sent_at >= self.heartbeat_ack_timeout or
                    self.active_fault or self.estop_latched):
                self.get_logger().warn(f'stale heartbeat ACK 무시: {value}')
                return
            self.last_heartbeat_ack_time = now
            self.publish_status(f'ACK,{value}')
            self.send_zero_velocity_probe()
            return

        if value == self.protocol.zero_velocity_ack_value(self.session_id):
            if (not self.zero_command_sent or not self._heartbeat_fresh(now) or
                    self.active_fault or self.estop_latched):
                self.get_logger().warn('startup zero command ACK 무시')
                return
            self.zero_command_acknowledged = True
            self.publish_status(f'ACK,{value}')
            self.send_servo_attach()
            return

        if value == 'SERVO_ATTACH':
            if (not self.servo_attach_requested or
                    not self.zero_command_acknowledged or
                    not self._heartbeat_fresh(now) or self.estop_latched or
                    self.servo_attach_blocked or self.active_fault):
                self.get_logger().warn(
                    'startup gate 이전 ACK,SERVO_ATTACH 무시')
                return
            if not self.servo_attached:
                self.servo_attached = True
                self.get_logger().info(
                    f'[{self.role}] ACK,SERVO_ATTACH; servo_attached=true')
            self.publish_status('ACK,SERVO_ATTACH')
            return

        # Servo/action ACKs remain observable but are never heartbeat evidence.
        self.publish_status(f'ACK,{value}')

    def _handle_error(self, code):
        communication_timeouts = {
            'HEARTBEAT_TIMEOUT', 'COMMAND_TIMEOUT'}
        if not self.hello_acknowledged and code in communication_timeouts:
            if code not in self.previous_session_faults:
                self.previous_session_faults.append(code)
            self.get_logger().warn(
                f'[{self.role}] 이전 communication session fault 격리: {code}')
            return

        status = f'ERR,{code}'
        self.get_logger().error(f'STM32 ERR: {code}')
        if code == 'ESTOP_LATCHED':
            self.estop_latched = True
            self.get_logger().error(
                f'[{self.role}] STM32 ESTOP latch; 수동 power-cycle/reset 후 '
                'bridge restart 필요')
        if code in {'LIFT_WHILE_MOVING', 'SERVO_NOT_ATTACHED'}:
            self.publish_status(status)
            return
        self._latch_fault(status)

    def publish_motor_diagnostics(self, parsed):
        wheels = ('FL', 'FR', 'RL', 'RR')
        rpm = ','.join(
            f'{name}:{value / 10.0:.1f}'
            for name, value in zip(wheels, parsed['rpm_x10']))
        pwm = ','.join(
            f'{name}:{value}'
            for name, value in zip(wheels, parsed['pwm']))
        self.pub_motor_diag.publish(String(
            data=f"cmd={parsed['command']} rpm={rpm} pwm={pwm}"))

    def publish_ultrasonic(self, parsed):
        side = parsed['side']
        now = time.monotonic()

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role}_ultrasonic_{side}'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.ultrasonic_fov
        msg.min_range = self.ultrasonic_min_range
        msg.max_range = self.ultrasonic_max_range

        if not parsed['valid']:
            self.last_ultrasonic_valid[side] = 0.0
            msg.range = float('inf')
            status = f'{side},TIMEOUT'
        else:
            distance = float(parsed['distance_m'])
            if distance < self.ultrasonic_min_range:
                msg.range = float('-inf')
                status = f'{side},TOO_CLOSE,{distance:.3f}'
            elif distance > self.ultrasonic_max_range:
                msg.range = float('inf')
                status = f'{side},OUT_OF_RANGE,{distance:.3f}'
            else:
                msg.range = distance
                status = f'{side},OK,{distance:.3f}'
                if self.hello_acknowledged:
                    self.last_ultrasonic_valid[side] = now
        self.pub_ultrasonic[side].publish(msg)
        self.pub_ultrasonic_status.publish(String(data=status))

    def publish_status(self, value):
        msg = String()
        msg.data = value
        self.pub_hw.publish(msg)

    def hardware_ready_conditions(self, now=None):
        """Return the single authoritative fail-closed readiness gate."""
        now = time.monotonic() if now is None else now
        serial_connected = (
            self.ser is not None and getattr(self.ser, 'is_open', True))
        ultrasonic_fresh = all(
            stamp > 0.0 and now - stamp < self.ultrasonic_frame_timeout
            for stamp in self.last_ultrasonic_valid.values())
        if not self.require_ultrasonic_for_ready:
            ultrasonic_fresh = True
        return {
            'serial_connected': serial_connected,
            'hello_ack_for_current_session': self.hello_acknowledged,
            'heartbeat_ack_fresh': self._heartbeat_fresh(now),
            'zero_command_initialized': self.zero_command_acknowledged,
            'servo_attach_ack': self.servo_attached,
            'ultrasonic_fresh': ultrasonic_fresh,
            'no_estop_latch': not self.estop_latched,
            'no_active_fault': self.active_fault is None,
            'no_uart_transport_fault': self.transport_fault is None,
        }

    def publish_hardware_state(self):
        now = time.monotonic()
        conditions = self.hardware_ready_conditions(now)
        ready = all(conditions.values())
        if ready and not self.hardware_ready:
            self.get_logger().info(f'[{self.role}] hardware_ready=true')
        elif not ready and self.hardware_ready:
            self.get_logger().warn(f'[{self.role}] hardware_ready=false')
        self.hardware_ready = ready
        self.pub_ready.publish(Bool(data=ready))
        self.pub_manual_active.publish(Bool(
            data=self.command_arbiter.manual_enabled))

        stale = (self.require_ultrasonic_for_ready and
                 not conditions['ultrasonic_fresh'])
        if stale and not self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = True
            missing = [
                side for side, stamp in self.last_ultrasonic_valid.items()
                if stamp <= 0.0 or
                now - stamp >= self.ultrasonic_frame_timeout
            ]
            self.publish_status(
                'WARN,ULTRASONIC_STALE:' + '|'.join(missing))
        elif not stale and self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = False
            self.publish_status('INFO,ULTRASONIC_STREAM_OK')
        if self.active_fault is not None:
            # Preserve the current safety latch for late-joining consumers;
            # ACK/INFO diagnostics must never become the retained final value.
            self.publish_status(self.active_fault)

    def publish_odom(self, odom):
        msg = Odometry()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.child_frame_id = f'{self.role}_base'
        # pose: 참고용 순수 dead-reckoning 누적치 (진단/RViz용, pose_fusion은 안 씀)
        msg.pose.pose.position.x = odom['x']
        msg.pose.pose.position.y = odom['y']
        msg.pose.pose.orientation.z = math.sin(odom['theta']/2)
        msg.pose.pose.orientation.w = math.cos(odom['theta']/2)
        # twist: pose_fusion_node의 Kalman predict 입력 — body-frame "변위"
        # (관례적 속도 아님). dx_body/dy_body/dtheta를 그대로 담는다.
        msg.twist.twist.linear.x = odom['dx_body']
        msg.twist.twist.linear.y = odom['dy_body']
        msg.twist.twist.angular.z = odom['dtheta']
        self.pub_odom.publish(msg)

    def destroy_node(self):
        if self.ser:
            serial_handle = self.ser
            if not self.estop_latched:
                self._write(self.protocol.encode_velocity(0.0, 0.0, 0.0))
            if self.ser is serial_handle:
                serial_handle.close()
                self.ser = None
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # 브리지가 사라지면 STM32 는 명령/heartbeat 를 못 받아 watchdog 으로
        # 멈춘다. 그 전에 0 속도를 명시적으로 보내 관성 주행을 없앤다.
        # ESTOP 은 보내지 않는다. latch 되면 전원 재인가 전까지 못 푼다.
        node.shutdown_stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
