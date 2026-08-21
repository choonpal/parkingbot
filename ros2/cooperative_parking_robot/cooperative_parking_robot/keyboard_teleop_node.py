#!/usr/bin/env python3
"""운용 PC/Jetson 터미널에서 Front 또는 Rear 한 대를 수동 조작한다."""

import os
import select
import sys
import termios
import time
import tty

import rclpy
from geometry_msgs.msg import TwistStamped
from rclpy.node import Node
from rclpy.signals import SignalHandlerOptions
from std_msgs.msg import Bool, String

from cooperative_parking_robot.manual_control import (
    DEFAULT_ANGULAR_SPEED_RPS,
    DEFAULT_LINEAR_SPEED_MPS,
    KeyboardTeleopState,
)


class KeyboardTeleopNode(Node):
    def __init__(self):
        super().__init__('keyboard_teleop')
        self.declare_parameter('role', 'front')
        self.declare_parameter('linear_speed_mps', DEFAULT_LINEAR_SPEED_MPS)
        self.declare_parameter('angular_speed_rps', DEFAULT_ANGULAR_SPEED_RPS)
        self.declare_parameter('deadman_s', 0.30)

        self.role = str(self.get_parameter('role').value)
        if self.role not in ('front', 'rear'):
            raise ValueError("role must be 'front' or 'rear'")
        if not sys.stdin.isatty():
            raise RuntimeError('keyboard_teleop must run in an interactive terminal')

        self.state = KeyboardTeleopState(
            self.get_parameter('linear_speed_mps').value,
            self.get_parameter('angular_speed_rps').value,
            self.get_parameter('deadman_s').value)
        self.old_terminal = termios.tcgetattr(sys.stdin)
        tty.setcbreak(sys.stdin.fileno())
        self.closed = False

        self.pub_cmd = self.create_publisher(
            TwistStamped, f'/{self.role}/manual_cmd_vel', 10)
        self.pub_grip = self.create_publisher(
            String, f'/{self.role}/manual_grip_command', 10)
        self.pub_enable = self.create_publisher(
            Bool, f'/{self.role}/manual_enable', 10)

        self.create_timer(0.02, self.poll_keyboard)
        self.create_timer(0.05, self.publish_velocity)
        self.create_timer(0.10, self.publish_enable)
        self.publish_enable()
        self.get_logger().info(
            f'[{self.role}] MANUAL MODE — WASD 이동, Q/E 회전, '
            'Space 정지, T/G grip/release, Ctrl+C 종료')

    def publish_enable(self):
        self.pub_enable.publish(Bool(data=True))

    def poll_keyboard(self):
        while select.select([sys.stdin], [], [], 0.0)[0]:
            key = os.read(sys.stdin.fileno(), 1).decode(
                'ascii', errors='ignore')
            action = self.state.handle_key(key, time.monotonic())
            if action is not None:
                self.pub_grip.publish(String(data=action))

    def publish_velocity(self):
        vx, vy, w = self.state.velocity(time.monotonic())
        msg = TwistStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = f'{self.role}_base'
        msg.twist.linear.x = vx
        msg.twist.linear.y = vy
        msg.twist.angular.z = w
        self.pub_cmd.publish(msg)

    def destroy_node(self):
        if not self.closed:
            self.closed = True
            self.state.stop()
            self.publish_velocity()
            self.pub_enable.publish(Bool(data=False))
            termios.tcsetattr(
                sys.stdin, termios.TCSADRAIN, self.old_terminal)
        super().destroy_node()


def main(args=None):
    # rclpy의 기본 SIGINT 처리기가 context를 먼저 종료하면 destroy_node()가
    # 마지막 0속도/manual-off 메시지를 발행할 수 없다. Python의
    # KeyboardInterrupt로 먼저 빠져나와 안전 메시지를 보낸 뒤 shutdown한다.
    rclpy.init(args=args, signal_handler_options=SignalHandlerOptions.NO)
    node = KeyboardTeleopNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
