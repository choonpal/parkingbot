#!/usr/bin/env python3
"""
시뮬레이션 메카넘 로봇 노드
============================
cmd_vel을 받아서 2D 물리 시뮬레이션으로 위치를 업데이트하고
odom을 발행합니다.

로봇 1대 = 이 노드 1개
앞 로봇: ros2 run parking_sim robot --ros-args -p name:=front
뒤 로봇: ros2 run parking_sim robot --ros-args -p name:=rear
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, TransformStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool
import math
import random
import time


class SimMecanumRobot(Node):
    def __init__(self):
        super().__init__('sim_robot')

        # 파라미터
        self.declare_parameter('name', 'front')
        self.declare_parameter('start_x', 0.7)       # m
        self.declare_parameter('start_y', 0.6)
        self.declare_parameter('start_theta', 1.5708)  # 90도
        self.declare_parameter('noise', 0.001)         # 위치 노이즈 (m)
        self.declare_parameter('max_speed', 0.10)      # m/s
        self.declare_parameter('wheel_slip', 0.02)     # 바퀴 미끄러짐 비율

        self.name = self.get_parameter('name').value
        self.noise = self.get_parameter('noise').value
        self.max_speed = self.get_parameter('max_speed').value
        self.slip = self.get_parameter('wheel_slip').value

        # 현재 위치/속도
        self.x = self.get_parameter('start_x').value
        self.y = self.get_parameter('start_y').value
        self.theta = self.get_parameter('start_theta').value
        self.vx = 0.0
        self.vy = 0.0
        self.omega = 0.0

        # 비상 정지
        self.e_stopped = False

        # ---- 구독 ----
        self.sub_cmd = self.create_subscription(
            Twist, f'/{self.name}/cmd_vel',
            self.cmd_callback, 10)
        self.sub_estop = self.create_subscription(
            Bool, '/emergency_stop',
            self.estop_callback, 10)

        # ---- 발행 ----
        self.pub_odom = self.create_publisher(
            Odometry, f'/{self.name}/odom', 10)

        # ---- 시뮬레이션 루프 (50Hz) ----
        self.dt = 0.02
        self.timer = self.create_timer(self.dt, self.update)

        self.get_logger().info(
            f'[{self.name}] 시작 위치: ({self.x:.2f}, {self.y:.2f}) '
            f'θ={math.degrees(self.theta):.0f}°')

    def cmd_callback(self, msg):
        """속도 명령 수신"""
        if self.e_stopped:
            return
        self.vx = max(-self.max_speed, min(self.max_speed, msg.linear.x))
        self.vy = max(-self.max_speed, min(self.max_speed, msg.linear.y))
        self.omega = max(-1.0, min(1.0, msg.angular.z))

    def estop_callback(self, msg):
        """비상 정지"""
        if msg.data:
            self.e_stopped = True
            self.vx = self.vy = self.omega = 0.0
            self.get_logger().warn(f'[{self.name}] 비상 정지!')
        else:
            self.e_stopped = False
            self.get_logger().info(f'[{self.name}] 비상 정지 해제')

    def update(self):
        """물리 시뮬레이션 1스텝"""
        if self.e_stopped:
            self.vx = self.vy = self.omega = 0.0

        # 로컬 → 글로벌 좌표 변환
        cos_t = math.cos(self.theta)
        sin_t = math.sin(self.theta)

        global_vx = self.vx * cos_t - self.vy * sin_t
        global_vy = self.vx * sin_t + self.vy * cos_t

        # 노이즈 + 미끄러짐 추가 (현실적 시뮬레이션)
        noise_x = random.gauss(0, self.noise)
        noise_y = random.gauss(0, self.noise)
        slip_x = random.gauss(0, self.slip * abs(self.vx))
        slip_y = random.gauss(0, self.slip * abs(self.vy))

        # 위치 업데이트
        self.x += (global_vx + slip_x) * self.dt + noise_x
        self.y += (global_vy + slip_y) * self.dt + noise_y
        self.theta += self.omega * self.dt
        self.theta = math.atan2(math.sin(self.theta), math.cos(self.theta))

        # 벽 충돌 (0~1.2m x 0~0.8m)
        self.x = max(0.05, min(1.15, self.x))
        self.y = max(0.05, min(0.75, self.y))

        # ---- Odometry 발행 ----
        odom = Odometry()
        odom.header.stamp = self.get_clock().now().to_msg()
        odom.header.frame_id = 'world'
        odom.child_frame_id = f'{self.name}_base'

        odom.pose.pose.position.x = self.x
        odom.pose.pose.position.y = self.y
        odom.pose.pose.position.z = 0.0

        # theta → quaternion (z축 회전만)
        odom.pose.pose.orientation.z = math.sin(self.theta / 2)
        odom.pose.pose.orientation.w = math.cos(self.theta / 2)

        odom.twist.twist.linear.x = self.vx
        odom.twist.twist.linear.y = self.vy
        odom.twist.twist.angular.z = self.omega

        self.pub_odom.publish(odom)


def main(args=None):
    rclpy.init(args=args)
    node = SimMecanumRobot()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
