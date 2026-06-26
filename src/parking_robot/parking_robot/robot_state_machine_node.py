#!/usr/bin/env python3
"""
==================================================
[2-4] robot_state_machine_node
==================================================
상태 관리자. 로봇의 현재 상태 결정 + 하위 노드 활성/비활성.

상태: IDLE → APPROACH → LIFTING → DRIVING → RELEASE → RETURN

입력:
  /robot/wheel_aligned (Bool) — 2-1 초음파 정렬 완료
  /fleet/state (String) — 1-2 관제탑 지시
  /sync/arrived (Bool) — 도착 신호
출력:
  /robot/state (String) — 현재 상태 (다른 노드가 참조)
  /robot/lifted (Bool) — 들기 완료 (관제탑에게)
  /robot/grip_cmd (String) — STM32에 grip/release 명령
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool, String
import json
import time


class RobotStateMachineNode(Node):
    def __init__(self):
        super().__init__('robot_state_machine_node')

        self.declare_parameter('role', 'front')  # front 또는 rear
        self.role = self.get_parameter('role').value

        # ===== 상태 =====
        self.STATES = ['IDLE', 'APPROACH', 'LIFTING',
                       'DRIVING', 'RELEASE', 'RETURN']
        self.state = 'IDLE'
        self.enter_time = time.time()

        # 신호 플래그
        self.wheel_aligned = False
        self.fleet_state = 'WAIT_TARGET'
        self.arrived = False

        # ===== 구독 =====
        self.create_subscription(Bool, '/robot/wheel_aligned',
                                 self.aligned_cb, 10)
        self.create_subscription(String, '/fleet/state',
                                 self.fleet_cb, 10)
        self.create_subscription(Bool, '/sync/arrived',
                                 self.arrived_cb, 10)

        # ===== 발행 =====
        self.pub_state = self.create_publisher(String, '/robot/state', 10)
        self.pub_lifted = self.create_publisher(Bool, '/robot/lifted', 10)
        self.pub_grip = self.create_publisher(String, '/robot/grip_cmd', 10)

        self.create_timer(0.1, self.state_machine)
        self.create_timer(0.5, self.publish_state)

        self.get_logger().info(
            f'robot_state_machine_node 시작 [{self.role}]')

    # ===== 콜백 =====
    def aligned_cb(self, msg):
        if msg.data:
            self.wheel_aligned = True

    def fleet_cb(self, msg):
        try:
            data = json.loads(msg.data)
            self.fleet_state = data.get('state', 'WAIT_TARGET')
        except:
            pass

    def arrived_cb(self, msg):
        if msg.data:
            self.arrived = True

    # ===== 상태 전환 =====
    def transition(self, new):
        self.get_logger().info(f'[{self.role}] {self.state} → {new}')
        self.state = new
        self.enter_time = time.time()

    def elapsed(self):
        return time.time() - self.enter_time

    # ================================================
    # 상태 머신
    # ================================================
    def state_machine(self):

        if self.state == 'IDLE':
            # 관제탑이 타겟 인식하면 접근 시작
            if self.fleet_state in ('WAIT_LIFT', 'SELECT_SLOT'):
                self.transition('APPROACH')

        elif self.state == 'APPROACH':
            # 초음파로 바퀴 정렬되면 들기
            if self.wheel_aligned:
                self.transition('LIFTING')
                self.send_grip('grip')

        elif self.state == 'LIFTING':
            # arm 닫는 시간 (2초) 후 들기 완료
            if self.elapsed() > 2.0:
                self.publish_lifted()
                self.transition('DRIVING')

        elif self.state == 'DRIVING':
            # Nav2 + 강체 동기화로 이동 (rigid_body_sync_node가 처리)
            # 도착 신호 받으면 내려놓기
            if self.arrived:
                self.transition('RELEASE')
                self.send_grip('release')

        elif self.state == 'RELEASE':
            if self.elapsed() > 2.0:
                self.transition('RETURN')

        elif self.state == 'RETURN':
            # 대기공간 복귀 (관제탑이 idle로 돌리면 완료)
            if self.fleet_state == 'WAIT_TARGET':
                self.reset()
                self.transition('IDLE')
            elif self.elapsed() > 30.0:
                self.reset()
                self.transition('IDLE')

    def send_grip(self, action):
        msg = String()
        msg.data = action   # 'grip' 또는 'release'
        self.pub_grip.publish(msg)
        self.get_logger().info(f'[{self.role}] arm {action}')

    def publish_lifted(self):
        msg = Bool()
        msg.data = True
        self.pub_lifted.publish(msg)

    def reset(self):
        self.wheel_aligned = False
        self.arrived = False

    def publish_state(self):
        msg = String()
        msg.data = self.state
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
