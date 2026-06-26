#!/usr/bin/env python3
"""
==================================================
orchestrator.py — 전체 지휘 상태 머신
==================================================
주차 시스템 전체 흐름을 관리하는 두뇌.

상태 흐름:
  IDLE → DETECT_TARGET → APPROACH → GRIP
       → REQUEST_SLOT → TRANSPORT → RELEASE → RETURN → IDLE

각 단계에서 CCTV/로봇 노드와 토픽·서비스로 통신하며
다음 단계로 진행할지 판단한다.

구독:
  /cctv/target_detected   (target 차량 인식됨)
  /cctv/reach_pose        (목표 주차 좌표)
  /front/status, /rear/status  (각 로봇 상태)

발행:
  /system/command         (로봇에 명령: approach/grip/transport/release)
  /system/state           (현재 시스템 상태 - 모니터링용)

서비스 클라이언트:
  /cctv/find_empty_slot   (빈자리 탐색 요청)
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String, Bool
from geometry_msgs.msg import PoseStamped
from std_srvs.srv import Trigger
import json
import time


class Orchestrator(Node):
    def __init__(self):
        super().__init__('orchestrator')

        # ===== 상태 정의 =====
        self.STATES = [
            'IDLE',           # 대기
            'DETECT_TARGET',  # 차량 인식 대기
            'APPROACH',       # 로봇이 차량 바퀴 옆으로 이동
            'GRIP',           # arm으로 들기
            'REQUEST_SLOT',   # 빈자리 요청
            'TRANSPORT',      # reach로 동기 이동
            'RELEASE',        # 내려놓기
            'RETURN',         # 대기공간 복귀
        ]
        self.state = 'IDLE'
        self.state_enter_time = time.time()

        # ===== 상태 추적 =====
        self.target_pose = None      # 인식된 차량 위치
        self.reach_pose = None       # 목표 주차 좌표
        self.front_status = 'idle'
        self.rear_status = 'idle'

        # ===== 구독 =====
        self.create_subscription(
            PoseStamped, '/cctv/target_detected',
            self.target_cb, 10)
        self.create_subscription(
            PoseStamped, '/cctv/reach_pose',
            self.reach_cb, 10)
        self.create_subscription(
            String, '/front/status',
            lambda m: self.status_cb('front', m), 10)
        self.create_subscription(
            String, '/rear/status',
            lambda m: self.status_cb('rear', m), 10)

        # ===== 발행 =====
        self.pub_command = self.create_publisher(
            String, '/system/command', 10)
        self.pub_state = self.create_publisher(
            String, '/system/state', 10)

        # ===== 서비스 클라이언트 (빈자리 요청) =====
        self.slot_client = self.create_client(
            Trigger, '/cctv/find_empty_slot')

        # ===== 메인 상태 머신 루프 =====
        self.create_timer(0.1, self.state_machine)
        self.create_timer(1.0, self.publish_state)

        self.slot_requested = False

        self.get_logger().info('오케스트레이터 시작 — IDLE 상태')

    # ================================================
    # 콜백
    # ================================================
    def target_cb(self, msg):
        self.target_pose = msg
        if self.state == 'DETECT_TARGET':
            self.get_logger().info(
                f'차량 인식! ({msg.pose.position.x:.2f}, '
                f'{msg.pose.position.y:.2f})')

    def reach_cb(self, msg):
        self.reach_pose = msg
        self.get_logger().info(
            f'빈자리 수신! reach: ({msg.pose.position.x:.2f}, '
            f'{msg.pose.position.y:.2f})')

    def status_cb(self, robot, msg):
        if robot == 'front':
            self.front_status = msg.data
        else:
            self.rear_status = msg.data

    # ================================================
    # 상태 전환 헬퍼
    # ================================================
    def transition(self, new_state):
        self.get_logger().info(f'[상태] {self.state} → {new_state}')
        self.state = new_state
        self.state_enter_time = time.time()

    def elapsed(self):
        return time.time() - self.state_enter_time

    def send_command(self, cmd, data=None):
        msg = String()
        payload = {'command': cmd}
        if data:
            payload.update(data)
        msg.data = json.dumps(payload)
        self.pub_command.publish(msg)

    def both_robots(self, status):
        """두 로봇이 모두 특정 상태인지"""
        return self.front_status == status and self.rear_status == status

    # ================================================
    # 메인 상태 머신
    # ================================================
    def state_machine(self):

        # ---- IDLE: 시작 대기 ----
        if self.state == 'IDLE':
            # 시작 후 1초 뒤 차량 탐지 모드로
            if self.elapsed() > 1.0:
                self.transition('DETECT_TARGET')

        # ---- DETECT_TARGET: 차량 인식 대기 ----
        elif self.state == 'DETECT_TARGET':
            if self.target_pose is not None:
                # 차량 위치를 로봇에게 전달하고 접근 명령
                self.send_command('approach', {
                    'target_x': self.target_pose.pose.position.x,
                    'target_y': self.target_pose.pose.position.y,
                })
                self.transition('APPROACH')

        # ---- APPROACH: 로봇이 바퀴 옆으로 이동 ----
        elif self.state == 'APPROACH':
            # 두 로봇이 모두 정렬 완료('aligned')되면 들기
            if self.both_robots('aligned'):
                self.send_command('grip')
                self.transition('GRIP')
            elif self.elapsed() > 30.0:
                self.get_logger().warn('접근 타임아웃 — IDLE 복귀')
                self.transition('IDLE')
                self.reset()

        # ---- GRIP: arm으로 들기 ----
        elif self.state == 'GRIP':
            # 두 로봇이 모두 들기 완료('gripped')
            if self.both_robots('gripped'):
                self.transition('REQUEST_SLOT')
            elif self.elapsed() > 15.0:
                self.get_logger().warn('들기 타임아웃')
                self.transition('IDLE')
                self.reset()

        # ---- REQUEST_SLOT: 빈자리 요청 ----
        elif self.state == 'REQUEST_SLOT':
            if not self.slot_requested:
                self.request_empty_slot()
                self.slot_requested = True
            # reach_pose가 수신되면 이동 시작
            if self.reach_pose is not None:
                self.send_command('transport', {
                    'reach_x': self.reach_pose.pose.position.x,
                    'reach_y': self.reach_pose.pose.position.y,
                })
                self.transition('TRANSPORT')

        # ---- TRANSPORT: reach로 동기 이동 ----
        elif self.state == 'TRANSPORT':
            # 두 로봇이 모두 도착('arrived')
            if self.both_robots('arrived'):
                self.send_command('release')
                self.transition('RELEASE')
            elif self.elapsed() > 60.0:
                self.get_logger().warn('이동 타임아웃')
                self.transition('RELEASE')

        # ---- RELEASE: 내려놓기 ----
        elif self.state == 'RELEASE':
            if self.both_robots('released'):
                self.send_command('return')
                self.transition('RETURN')
            elif self.elapsed() > 15.0:
                self.transition('RETURN')

        # ---- RETURN: 복귀 ----
        elif self.state == 'RETURN':
            if self.both_robots('idle'):
                self.get_logger().info('주차 완료! 사이클 종료')
                self.reset()
                self.transition('IDLE')
            elif self.elapsed() > 30.0:
                self.reset()
                self.transition('IDLE')

    # ================================================
    # 빈자리 서비스 요청
    # ================================================
    def request_empty_slot(self):
        if not self.slot_client.service_is_ready():
            self.get_logger().warn('빈자리 서비스 대기 중...',
                                   throttle_duration_sec=2.0)
            return
        req = Trigger.Request()
        future = self.slot_client.call_async(req)
        future.add_done_callback(self.slot_response_cb)
        self.get_logger().info('빈자리 탐색 요청 전송')

    def slot_response_cb(self, future):
        try:
            response = future.result()
            if response.success:
                self.get_logger().info(f'빈자리 응답: {response.message}')
                # reach_pose는 /cctv/reach_pose 토픽으로 별도 수신
        except Exception as e:
            self.get_logger().error(f'빈자리 요청 실패: {e}')

    # ================================================
    # 상태 초기화
    # ================================================
    def reset(self):
        self.target_pose = None
        self.reach_pose = None
        self.slot_requested = False

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'front': self.front_status,
            'rear': self.rear_status,
            'has_target': self.target_pose is not None,
            'has_reach': self.reach_pose is not None,
        })
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = Orchestrator()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
