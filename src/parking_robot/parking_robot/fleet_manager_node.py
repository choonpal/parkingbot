#!/usr/bin/env python3
"""
==================================================
[1-2] fleet_manager_node
==================================================
관제탑 (Jetson Orin Nano). 빈자리 선정 + Nav2 이동 명령.

빈자리들 중 대기장소와 가까운 곳을 선정하여,
가상 로봇을 그 좌표로 보내는 Nav2 액션 호출.

입력:
  /parking/target_pose (PoseStamped) — 타겟 차량
  /parking/empty_slots (PoseArray) — 빈자리들
  /robot/lifted (Bool) — 차량 들림 완료 신호
출력:
  NavigateToPose 액션 (가상 로봇용 goal)
  /fleet/state (String) — 관제 상태
"""

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from geometry_msgs.msg import PoseStamped, PoseArray
from std_msgs.msg import Bool, String
import math
import json

try:
    from nav2_msgs.action import NavigateToPose
    NAV2_OK = True
except ImportError:
    NAV2_OK = False


class FleetManagerNode(Node):
    def __init__(self):
        super().__init__('fleet_manager_node')

        self.declare_parameter('waiting_x', 0.5)
        self.declare_parameter('waiting_y', 0.5)
        self.wait_x = self.get_parameter('waiting_x').value
        self.wait_y = self.get_parameter('waiting_y').value

        # 상태
        self.target_pose = None
        self.empty_slots = []
        self.car_lifted = False
        self.goal_sent = False
        self.state = 'WAIT_TARGET'

        # 구독
        self.create_subscription(PoseStamped, '/parking/target_pose',
                                 self.target_cb, 10)
        self.create_subscription(PoseArray, '/parking/empty_slots',
                                 self.slots_cb, 10)
        self.create_subscription(Bool, '/robot/lifted',
                                 self.lifted_cb, 10)

        # 발행
        self.pub_state = self.create_publisher(String, '/fleet/state', 10)

        # Nav2 액션 클라이언트 (가상 로봇)
        if NAV2_OK:
            self.nav_client = ActionClient(
                self, NavigateToPose, '/navigate_to_pose')
        else:
            self.nav_client = None
            self.get_logger().warn('nav2_msgs 없음 — 액션 비활성')

        self.create_timer(0.5, self.manage_loop)
        self.create_timer(1.0, self.publish_state)
        self.get_logger().info('fleet_manager_node 시작 (관제탑)')

    def target_cb(self, msg):
        self.target_pose = msg

    def slots_cb(self, msg):
        self.empty_slots = [(p.position.x, p.position.y) for p in msg.poses]

    def lifted_cb(self, msg):
        if msg.data and not self.car_lifted:
            self.car_lifted = True
            self.get_logger().info('차량 들림 신호 수신!')

    # ================================================
    # 관제 로직
    # ================================================
    def manage_loop(self):
        if self.state == 'WAIT_TARGET':
            if self.target_pose is not None:
                self.state = 'WAIT_LIFT'
                self.get_logger().info('타겟 인식 — 들기 대기')

        elif self.state == 'WAIT_LIFT':
            # 차량이 들리면 빈자리 선정
            if self.car_lifted:
                self.state = 'SELECT_SLOT'

        elif self.state == 'SELECT_SLOT':
            slot = self.select_nearest_slot()
            if slot is not None:
                self.send_nav_goal(slot)
                self.state = 'NAVIGATING'
            else:
                self.get_logger().warn('빈자리 없음', throttle_duration_sec=3.0)

        elif self.state == 'NAVIGATING':
            pass  # 액션 결과 콜백에서 처리

    def select_nearest_slot(self):
        """대기장소와 가장 가까운 빈자리 선정"""
        if not self.empty_slots:
            return None
        nearest = min(self.empty_slots, key=lambda s:
                      math.hypot(s[0]-self.wait_x, s[1]-self.wait_y))
        self.get_logger().info(
            f'빈자리 선정: ({nearest[0]:.2f}, {nearest[1]:.2f})')
        return nearest

    def send_nav_goal(self, slot):
        """Nav2에 가상 로봇 이동 명령"""
        if self.nav_client is None:
            self.get_logger().warn('Nav2 클라이언트 없음 — goal 스킵')
            return
        if not self.nav_client.wait_for_server(timeout_sec=2.0):
            self.get_logger().warn('Nav2 서버 대기 중...')
            return

        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = slot[0]
        goal.pose.pose.position.y = slot[1]
        goal.pose.pose.orientation.w = 1.0

        self.get_logger().info(f'Nav2 goal 전송: {slot}')
        future = self.nav_client.send_goal_async(goal)
        future.add_done_callback(self.goal_response_cb)

    def goal_response_cb(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().error('Nav2 goal 거부됨')
            return
        self.get_logger().info('Nav2 goal 수락 — 이동 시작')
        handle.get_result_async().add_done_callback(self.result_cb)

    def result_cb(self, future):
        self.get_logger().info('Nav2 이동 완료 — 도착')
        self.state = 'ARRIVED'

    def publish_state(self):
        msg = String()
        msg.data = json.dumps({
            'state': self.state,
            'has_target': self.target_pose is not None,
            'empty_count': len(self.empty_slots),
            'lifted': self.car_lifted,
        })
        self.pub_state.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
