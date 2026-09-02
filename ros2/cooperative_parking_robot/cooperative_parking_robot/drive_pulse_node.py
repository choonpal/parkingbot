#!/usr/bin/env python3
"""'이 속도로 이만큼' 한 번 주행하고 스스로 멈추는 점검 노드.

속도와 시간을 인자로 받아 제한된 명령을 실행하고 끝나면 정지한다. 거리
반복성이 필요한 시험(엔코더 보정, 방향 확인, 직진 편차)에 쓴다.

안전 설계
--------
* ``confirm_clear`` 없이는 명령을 한 줄도 보내지 않는다.
* 속도와 시간에 상한이 있다. 오타로 10 m/s 를 넣어도 거부한다.
* 명령은 20 Hz 로 계속 나간다. STM32 watchdog 이 250 ms 라 발행이 끊기면
  로봇이 알아서 멈춘다 — 이 노드가 죽어도 안전한 쪽으로 실패한다.
* 끝나면 0 속도를 0.5 초 보낸 뒤 ``manual_enable`` 을 내린다. 관성으로 더
  가는 것을 막고, 수동 모드를 열어둔 채 떠나지 않는다.
* Ctrl+C 도 같은 정지 절차를 밟는다.

예
--
    ros2 run cooperative_parking_robot drive_pulse --ros-args \
      -p role:=front -p vx:=0.0628 -p seconds:=1.6 -p confirm_clear:=true

0.0628 m/s x 1.6 s = 약 10 cm.
"""

import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TwistStamped
from std_msgs.msg import Bool

from cooperative_parking_robot.command_qos import CMD_VEL_QOS


# 오타로 로봇을 날려먹지 않기 위한 상한. 현장에서 넘겨야 하면 파라미터로
# 올리되 기본값은 보수적으로 둔다.
MAX_LINEAR_MPS = 0.15
MAX_ANGULAR_RPS = 0.6
MAX_SECONDS = 10.0

# 확인된 정상 구동점(12 rpm). 이보다 낮추면 모터가 스톨 근처에서 덜컥거린다.
NOMINAL_LINEAR_MPS = 0.0628

COMMAND_HZ = 20.0
ENABLE_HZ = 10.0
STOP_HOLD_S = 0.5


def validate_pulse(vx, vy, wz, seconds,
                   max_linear=MAX_LINEAR_MPS,
                   max_angular=MAX_ANGULAR_RPS,
                   max_seconds=MAX_SECONDS):
    """속도·시간을 검사해 정규화한다. 문제가 있으면 ValueError.

    상한을 넘기면 잘라내지 않고 **거부**한다. 조용히 줄이면 사용자가
    자기가 넣은 값대로 움직였다고 믿게 된다.
    """
    values = {'vx': float(vx), 'vy': float(vy), 'wz': float(wz),
              'seconds': float(seconds)}
    for name, value in values.items():
        if not math.isfinite(value):
            raise ValueError(f'{name} 이 유한한 숫자가 아닙니다: {value!r}')

    if values['seconds'] <= 0.0:
        raise ValueError('seconds 는 0보다 커야 합니다')
    if values['seconds'] > max_seconds:
        raise ValueError(
            f'seconds 상한 {max_seconds:.1f} s 를 넘었습니다: '
            f"{values['seconds']:.2f}")

    speed = math.hypot(values['vx'], values['vy'])
    if speed > max_linear:
        raise ValueError(
            f'선속도 상한 {max_linear:.3f} m/s 를 넘었습니다: {speed:.3f}')
    if abs(values['wz']) > max_angular:
        raise ValueError(
            f'각속도 상한 {max_angular:.3f} rad/s 를 넘었습니다: '
            f"{abs(values['wz']):.3f}")
    if speed == 0.0 and values['wz'] == 0.0:
        raise ValueError('vx, vy, wz 가 모두 0 입니다 — 움직일 명령이 없습니다')

    return values


def pulse_velocity(elapsed, seconds, vx, vy, wz):
    """경과 시간에 따른 속도. 구간이 끝나면 None.

    가감속을 넣지 않는다. 엔코더 보정용이라 '명령 속도 x 시간' 이 그대로
    거리가 되어야 계산이 단순하다.
    """
    if elapsed < 0.0:
        raise ValueError('elapsed must be non-negative')
    if elapsed >= seconds:
        return None
    return (vx, vy, wz)


def expected_distance_m(vx, vy, seconds):
    """이 명령이 이론적으로 가야 할 거리. 실측과 비교하는 기준값."""
    return math.hypot(float(vx), float(vy)) * float(seconds)


class DrivePulseNode(Node):
    def __init__(self):
        super().__init__('drive_pulse_node')
        self.declare_parameter('role', 'front')
        self.declare_parameter('vx', NOMINAL_LINEAR_MPS)
        self.declare_parameter('vy', 0.0)
        self.declare_parameter('wz', 0.0)
        self.declare_parameter('seconds', 1.6)
        # 바퀴가 떠 있거나 주변이 비었음을 사람이 확인했다는 표시.
        self.declare_parameter('confirm_clear', False)
        self.declare_parameter('max_linear_mps', MAX_LINEAR_MPS)
        self.declare_parameter('max_angular_rps', MAX_ANGULAR_RPS)
        self.declare_parameter('max_seconds', MAX_SECONDS)

        self.role = str(self.get_parameter('role').value).strip().lower()
        if self.role not in ('front', 'rear'):
            raise ValueError("role 은 'front' 또는 'rear' 여야 합니다")

        self.plan = validate_pulse(
            self.get_parameter('vx').value,
            self.get_parameter('vy').value,
            self.get_parameter('wz').value,
            self.get_parameter('seconds').value,
            float(self.get_parameter('max_linear_mps').value),
            float(self.get_parameter('max_angular_rps').value),
            float(self.get_parameter('max_seconds').value))

        self.confirmed = bool(self.get_parameter('confirm_clear').value)

        self.pub_cmd = self.create_publisher(
            TwistStamped, f'/{self.role}/manual_cmd_vel', CMD_VEL_QOS)
        self.pub_enable = self.create_publisher(
            Bool, f'/{self.role}/manual_enable', 10)

        distance = expected_distance_m(
            self.plan['vx'], self.plan['vy'], self.plan['seconds'])
        summary = (
            f"[{self.role}] vx={self.plan['vx']:.4f} vy={self.plan['vy']:.4f} "
            f"wz={self.plan['wz']:.4f} x {self.plan['seconds']:.2f}s "
            f'-> 예상 {distance * 100:.1f} cm')

        if not self.confirmed:
            # 여기서 멈추는 것이 이 노드의 기본값이다.
            self.get_logger().error(
                '실행하지 않았습니다. 바퀴를 띄우거나 주변을 비운 뒤 '
                'confirm_clear:=true 를 주세요.\n  계획: ' + summary)
            self.finished = True
            self.started_at = None
            return

        self.get_logger().warn('주행 시작 — ' + summary)
        self.finished = False
        self.started_at = time.monotonic()
        self.create_timer(1.0 / ENABLE_HZ, self.publish_enable)
        self.create_timer(1.0 / COMMAND_HZ, self.publish_command)

    # ------------------------------------------------------------------
    def publish_enable(self):
        # 정지 절차 중에도 enable 을 유지해야 0 속도가 실제로 전달된다.
        self.pub_enable.publish(Bool(data=not self.finished))

    def _send(self, vx, vy, wz):
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'base_link'
        msg.twist.linear.x = float(vx)
        msg.twist.linear.y = float(vy)
        msg.twist.angular.z = float(wz)
        self.pub_cmd.publish(msg)

    def publish_command(self):
        if self.finished:
            return
        elapsed = time.monotonic() - self.started_at
        velocity = pulse_velocity(
            elapsed, self.plan['seconds'],
            self.plan['vx'], self.plan['vy'], self.plan['wz'])
        if velocity is None:
            self.get_logger().info(
                f"{self.plan['seconds']:.2f}s 경과 — 정지합니다")
            self.stop()
            return
        self._send(*velocity)

    def stop(self):
        """0 속도를 잠깐 유지한 뒤 수동 모드를 내린다."""
        if self.started_at is None:
            return
        deadline = time.monotonic() + STOP_HOLD_S
        while time.monotonic() < deadline:
            self._send(0.0, 0.0, 0.0)
            self.pub_enable.publish(Bool(data=True))
            time.sleep(1.0 / COMMAND_HZ)
        self._send(0.0, 0.0, 0.0)
        self.pub_enable.publish(Bool(data=False))
        self.finished = True


def main(args=None):
    rclpy.init(args=args)
    node = None
    try:
        node = DrivePulseNode()
        if node.finished:          # confirm_clear 없음 -> 아무것도 안 함
            return
        while rclpy.ok() and not node.finished:
            rclpy.spin_once(node, timeout_sec=0.05)
    except KeyboardInterrupt:
        if node is not None:
            node.get_logger().warn('Ctrl+C — 정지합니다')
            node.stop()
    except ValueError as exc:
        print(f'설정 오류: {exc}')
    finally:
        if node is not None:
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
