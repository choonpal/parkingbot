#!/usr/bin/env python3
"""
==================================================
robot_node.py — 로봇 노드 (front/rear 공용)
==================================================
각 로봇(라즈베리파이)에서 실행. STM32로 모터 제어.

역할:
  - 오케스트레이터 명령 수신 (approach/grip/transport/release/return)
  - 초음파로 바퀴 탐지
  - STM32에 UART로 모터/서보 명령 전송
  - 강체 기구학 기반 동기 이동 (TRANSPORT)
  - 자기 상태를 /front(rear)/status로 발행

실행:
  ros2 run parking_system robot --ros-args -p name:=front
  ros2 run parking_system robot --ros-args -p name:=rear

STM32 통신은 UART(시리얼)로 가정.
실제 STM32 펌웨어는 별도 (C 코드).
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Float32
from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
import json
import math
import time

# STM32 시리얼 통신 (실제 환경)
try:
    import serial
    SERIAL_OK = True
except ImportError:
    SERIAL_OK = False


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # ===== 파라미터 =====
        self.declare_parameter('name', 'front')
        self.declare_parameter('serial_port', '/dev/ttyUSB0')
        self.declare_parameter('serial_baud', 115200)
        self.declare_parameter('wheelbase', 0.25)
        self.declare_parameter('max_speed', 0.08)
        self.declare_parameter('grip_distance', 0.15)  # 초음파 임계 (m)

        self.name = self.get_parameter('name').value
        self.wheelbase = self.get_parameter('wheelbase').value
        self.half_L = self.wheelbase / 2
        self.max_speed = self.get_parameter('max_speed').value
        self.grip_dist = self.get_parameter('grip_distance').value

        # ===== 상태 =====
        self.status = 'idle'   # idle/aligned/gripped/arrived/released
        self.command = None
        self.target_xy = None
        self.reach_xy = None

        # 위치 (엔코더 odom)
        self.x = 0.0
        self.y = 0.0
        self.theta = 0.0

        # 초음파
        self.ultrasonic = 1.0
        self.prev_ultrasonic = 1.0
        self.wheel_detected = False

        # ===== STM32 시리얼 연결 =====
        self.ser = None
        if SERIAL_OK:
            try:
                port = self.get_parameter('serial_port').value
                baud = self.get_parameter('serial_baud').value
                self.ser = serial.Serial(port, baud, timeout=0.01)
                self.get_logger().info(f'STM32 연결: {port}')
            except Exception as e:
                self.get_logger().warn(f'STM32 연결 실패 (시뮬레이션 모드): {e}')

        # ===== 구독 =====
        self.create_subscription(
            String, '/system/command', self.command_cb, 10)
        self.create_subscription(
            Odometry, f'/{self.name}/odom', self.odom_cb, 10)
        # 동기 이동 시 컨트롤러가 주는 cmd_vel
        self.create_subscription(
            Twist, f'/{self.name}/cmd_vel', self.cmd_vel_cb, 10)
        # 초음파 (STM32에서 발행하거나 별도 노드)
        self.create_subscription(
            Float32, f'/{self.name}/ultrasonic',
            self.ultrasonic_cb, 10)

        # ===== 발행 =====
        self.pub_status = self.create_publisher(
            String, f'/{self.name}/status', 10)

        # ===== 루프 =====
        self.create_timer(0.05, self.control_loop)  # 20Hz
        self.create_timer(0.5, self.publish_status)

        self.get_logger().info(f'[{self.name}] 로봇 노드 시작')

    # ================================================
    # 콜백
    # ================================================
    def command_cb(self, msg):
        data = json.loads(msg.data)
        cmd = data['command']
        if cmd != self.command:
            self.get_logger().info(f'[{self.name}] 명령 수신: {cmd}')
        self.command = cmd

        if cmd == 'approach':
            self.target_xy = (data.get('target_x'), data.get('target_y'))
        elif cmd == 'transport':
            self.reach_xy = (data.get('reach_x'), data.get('reach_y'))

    def odom_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.x = p.x
        self.y = p.y
        self.theta = math.atan2(2*q.w*q.z, 1-2*q.z*q.z)

    def cmd_vel_cb(self, msg):
        """동기 이동 시 컨트롤러가 주는 속도 → STM32로 전달"""
        if self.command == 'transport':
            self.send_to_stm32_velocity(
                msg.linear.x, msg.linear.y, msg.angular.z)

    def ultrasonic_cb(self, msg):
        self.prev_ultrasonic = self.ultrasonic
        self.ultrasonic = msg.data

    # ================================================
    # STM32 통신 (UART)
    # ================================================
    def send_to_stm32_velocity(self, vx, vy, omega):
        """메카넘 속도 명령 → STM32"""
        cmd = f"V,{vx:.3f},{vy:.3f},{omega:.3f}\n"
        self._write_serial(cmd)

    def send_to_stm32_servo(self, action):
        """arm 서보 제어 → STM32 (grip/release)"""
        cmd = f"S,{action}\n"   # S,grip 또는 S,release
        self._write_serial(cmd)

    def _write_serial(self, cmd):
        if self.ser is not None:
            try:
                self.ser.write(cmd.encode())
            except Exception as e:
                self.get_logger().error(f'STM32 쓰기 실패: {e}')
        # 시뮬레이션 모드면 로그만

    # ================================================
    # 메인 제어 루프 (명령별 동작)
    # ================================================
    def control_loop(self):
        if self.command == 'approach':
            self.do_approach()
        elif self.command == 'grip':
            self.do_grip()
        elif self.command == 'transport':
            pass  # cmd_vel_cb에서 처리 (컨트롤러 주도)
            self.check_arrived()
        elif self.command == 'release':
            self.do_release()
        elif self.command == 'return':
            self.do_return()

    # ---- 접근: 차량 바퀴 옆으로 이동 ----
    def do_approach(self):
        if self.target_xy is None or self.target_xy[0] is None:
            return
        tx, ty = self.target_xy
        dx = tx - self.x
        dy = ty - self.y
        dist = math.sqrt(dx*dx + dy*dy)

        if dist < 0.05:  # 5cm 이내 도착
            self.send_to_stm32_velocity(0, 0, 0)
            self.status = 'aligned'
            return

        # 목표 방향으로 비례 제어
        ct, st = math.cos(self.theta), math.sin(self.theta)
        local_x = dx*ct + dy*st
        local_y = -dx*st + dy*ct
        vx = max(-self.max_speed, min(self.max_speed, 0.8*local_x))
        vy = max(-self.max_speed, min(self.max_speed, 0.8*local_y))
        self.send_to_stm32_velocity(vx, vy, 0)

    # ---- 들기: 초음파로 바퀴 찾고 arm 닫기 ----
    def do_grip(self):
        # 초음파로 바퀴 탐지: 거리가 줄었다 늘어나는 지점
        # (줄었다 = 바퀴 접근, 늘어남 = 바퀴 통과)
        if not self.wheel_detected:
            # 거리가 임계 이하로 줄면 바퀴 위치로 판단
            if (self.prev_ultrasonic < self.grip_dist and
                    self.ultrasonic > self.prev_ultrasonic):
                # 줄었다가 늘어나는 변곡점 = 바퀴 중심
                self.wheel_detected = True
                self.send_to_stm32_velocity(0, 0, 0)
                self.send_to_stm32_servo('grip')
                self.get_logger().info(f'[{self.name}] 바퀴 감지 → arm 닫기')
        else:
            # arm 닫는 시간 대기 후 완료
            self.status = 'gripped'

    # ---- 도착 판정 (TRANSPORT 중) ----
    def check_arrived(self):
        if self.reach_xy is None or self.reach_xy[0] is None:
            return
        rx, ry = self.reach_xy
        dist = math.sqrt((rx-self.x)**2 + (ry-self.y)**2)
        if dist < 0.04:  # 4cm 이내
            self.status = 'arrived'

    # ---- 내려놓기: arm 열기 ----
    def do_release(self):
        self.send_to_stm32_velocity(0, 0, 0)
        self.send_to_stm32_servo('release')
        self.status = 'released'

    # ---- 복귀: 대기공간으로 ----
    def do_return(self):
        # 대기 위치 (0, 0) 가정
        dist = math.sqrt(self.x**2 + self.y**2)
        if dist < 0.05:
            self.send_to_stm32_velocity(0, 0, 0)
            self.status = 'idle'
            self.wheel_detected = False
            self.command = None
            return
        ct, st = math.cos(self.theta), math.sin(self.theta)
        local_x = (-self.x)*ct + (-self.y)*st
        local_y = -(-self.x)*st + (-self.y)*ct
        vx = max(-self.max_speed, min(self.max_speed, 0.8*local_x))
        vy = max(-self.max_speed, min(self.max_speed, 0.8*local_y))
        self.send_to_stm32_velocity(vx, vy, 0)

    def publish_status(self):
        msg = String()
        msg.data = self.status
        self.pub_status.publish(msg)

    def destroy_node(self):
        if self.ser is not None:
            self.send_to_stm32_velocity(0, 0, 0)
            self.ser.close()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RobotNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
