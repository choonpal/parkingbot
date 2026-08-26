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

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import TwistStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Range
from std_msgs.msg import String, Bool
import math
import time

from cooperative_parking_robot.uart_protocol import UartProtocol
from cooperative_parking_robot.encoder_odometry import EncoderOdometry
from cooperative_parking_robot.freshness import StampGate, stamp_to_ns
from cooperative_parking_robot.manual_control import VelocityCommandArbiter

try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False


class Stm32BridgeNode(Node):
    def __init__(self):
        super().__init__('stm32_bridge_node')

        self.declare_parameter('role', 'front')
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

        self.role = self.get_parameter('role').value
        if self.role not in ('front', 'rear'):
            raise ValueError("role must be 'front' or 'rear'")
        # 실차 수동 조작기에서 확인된 섀시 장착 방향 보정. STM32의 양의 축은
        # 실차 firmware 기준이고, 여기서 ROS REP-103 명령축으로 변환한다.
        self.command_sign = ((-1.0, -1.0, -1.0) if self.role == 'front'
                             else (-1.0, 1.0, 1.0))
        self.max_linear = float(self.get_parameter('max_linear_mps').value)
        self.max_angular = float(self.get_parameter('max_angular_rps').value)
        self.protocol = UartProtocol()
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
            max_delta_ticks=self.get_parameter('max_delta_ticks').value)

        # 수동 모드는 auto cmd보다 우선하며, 수동 송신이 끊겨도 auto로
        # 되돌아가지 않고 0속도를 유지한다.
        self.command_arbiter = VelocityCommandArbiter(
            manual_timeout_s=self.command_source_timeout)
        self.last_ack_time = 0.0
        self.last_ultrasonic_frame = {'left': 0.0, 'right': 0.0}
        self.ultrasonic_stale_reported = False
        self.estop_latched = False
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
                    self.get_parameter('serial_baud').value,
                    timeout=0.0,
                    write_timeout=0.05)
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
            TwistStamped, f'/{self.role}/cmd_vel', self.cmd_vel_cb, 10)
        self.create_subscription(String, f'/{self.role}/grip_command',
                                 self.grip_cb, 10)
        self.create_subscription(
            TwistStamped, f'/{self.role}/manual_cmd_vel',
            self.manual_cmd_vel_cb, 10)
        self.create_subscription(
            String, f'/{self.role}/manual_grip_command',
            self.manual_grip_cb, 10)
        self.create_subscription(
            Bool, f'/{self.role}/manual_enable',
            self.manual_enable_cb, 10)
        self.create_subscription(Bool, '/emergency_stop',
                                 self.estop_cb, 10)

        # ===== 발행 =====
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.role}/wheel_odom', qos_profile_sensor_data)
        self.pub_lift = self.create_publisher(
            String, f'/{self.role}/lift_status', 10)
        self.pub_hw = self.create_publisher(
            String, f'/{self.role}/hardware_status', 10)
        self.pub_ready = self.create_publisher(
            Bool, f'/{self.role}/hardware_ready', 10)
        # 수동 제어 요청이 실제로 이 bridge까지 도착했는지 시험 제어기가
        # 확인할 수 있는 명시적 ACK.  주기 발행하므로 늦게 연결된 peer도 받는다.
        self.pub_manual_active = self.create_publisher(
            Bool, f'/{self.role}/manual_active', 10)
        self.pub_ultrasonic = {
            side: self.create_publisher(
                Range, f'/{self.role}/ultrasonic_{side}',
                qos_profile_sensor_data)
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
        self.create_timer(0.2, self.publish_hardware_state)

        self.get_logger().info(f'stm32_bridge_node 시작 [{self.role}]')

    # ===== ROS2 → STM32 =====
    def cmd_vel_cb(self, msg):
        # 역기구학 안 함. 최신 명령 저장 (송신은 send_velocity_loop가 담당)
        accepted, reason = self.command_stamp_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'cmd_vel rejected: {reason}',
                throttle_duration_sec=1.0)
            return
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
        accepted, reason = self.manual_command_stamp_gate.accept(
            stamp_to_ns(msg.header.stamp),
            self.get_clock().now().nanoseconds)
        if not accepted:
            self.get_logger().warn(
                f'manual cmd_vel rejected: {reason}',
                throttle_duration_sec=1.0)
            return
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
        if self.estop_latched:
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
        if self.estop_latched:
            self.get_logger().warn('ESTOP latch 상태에서 그리퍼 명령 거부')
            return
        try:
            cmd = self.protocol.encode_servo(action)
        except ValueError as e:
            self.get_logger().error(str(e))
            return
        self._write(cmd)

    def estop_cb(self, msg):
        if msg.data and not self.estop_latched:
            self.estop_latched = True
            self.command_arbiter.force_zero(time.monotonic())
            self._write(self.protocol.encode_estop())
            self.publish_status('ESTOP')

    def send_heartbeat(self):
        self._write(self.protocol.encode_heartbeat(time.monotonic()))

    def _write(self, cmd):
        if self.ser:
            try:
                self.ser.write(cmd.encode())
                return True
            except Exception as e:
                self.get_logger().error(f'UART 쓰기 실패: {e}')
                self.publish_status(f'ERR,UART_WRITE:{e}')
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
            self.get_logger().error(f'UART 읽기 실패: {exc}')
            self.publish_status(f'ERR,UART_READ:{exc}')
            return

        if len(self.rx_buffer) > self.max_rx_buffer:
            self.rx_buffer.clear()
            self.get_logger().error('UART RX buffer overflow — buffer reset')
            self.publish_status('ERR,UART_RX_OVERFLOW')
            return

        processed = 0
        while processed < 20:
            newline = self.rx_buffer.find(b'\n')
            if newline < 0:
                break
            raw = bytes(self.rx_buffer[:newline])
            del self.rx_buffer[:newline + 1]
            processed += 1
            try:
                line = raw.rstrip(b'\r').decode('ascii', errors='strict').strip()
            except UnicodeDecodeError:
                self.get_logger().warn('비 ASCII STM32 frame 폐기')
                continue
            if not line:
                continue
            self._handle_serial_line(line)

    def _handle_serial_line(self, line):
        parsed = self.protocol.parse(line)
        if parsed is None:
            self.get_logger().warn(f'잘못된 STM32 frame 폐기: {line[:48]}')
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
            self.last_ack_time = time.monotonic()
            self.publish_status(f"ACK,{parsed['value']}")
        elif parsed['type'] == 'error':
            self.get_logger().error(f"STM32 ERR: {parsed['code']}")
            self.publish_status(f"ERR,{parsed['code']}")
        elif parsed['type'] == 'telemetry':
            # E/U frame가 ROS 데이터의 기준이며 T는 수동 시험기용 진단값이다.
            # 바퀴별로 목표를 못 따라가는 모터를 찾을 수 있도록 그대로 발행한다.
            self.publish_motor_diagnostics(parsed)

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
        self.last_ultrasonic_frame[side] = now

        msg = Range()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role}_ultrasonic_{side}'
        msg.radiation_type = Range.ULTRASOUND
        msg.field_of_view = self.ultrasonic_fov
        msg.min_range = self.ultrasonic_min_range
        msg.max_range = self.ultrasonic_max_range

        if not parsed['valid']:
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
        self.pub_ultrasonic[side].publish(msg)
        self.pub_ultrasonic_status.publish(String(data=status))

    def publish_status(self, value):
        msg = String()
        msg.data = value
        self.pub_hw.publish(msg)

    def publish_hardware_state(self):
        now = time.monotonic()
        uart_ready = (self.ser is not None and not self.estop_latched and
                      now - self.last_ack_time < 0.5)
        ultrasonic_ready = all(
            now - stamp < self.ultrasonic_frame_timeout
            for stamp in self.last_ultrasonic_frame.values())
        if not self.require_ultrasonic_for_ready:
            ultrasonic_ready = True

        ready = uart_ready and ultrasonic_ready
        self.pub_ready.publish(Bool(data=ready))
        self.pub_manual_active.publish(Bool(
            data=self.command_arbiter.manual_enabled))

        stale = self.require_ultrasonic_for_ready and not ultrasonic_ready
        if stale and not self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = True
            missing = [
                side for side, stamp in self.last_ultrasonic_frame.items()
                if now - stamp >= self.ultrasonic_frame_timeout
            ]
            self.publish_status(
                'WARN,ULTRASONIC_STALE:' + '|'.join(missing))
        elif not stale and self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = False
            self.publish_status('INFO,ULTRASONIC_STREAM_OK')

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
            if not self.estop_latched:
                self._write(self.protocol.encode_velocity(0.0, 0.0, 0.0))
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = Stm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
