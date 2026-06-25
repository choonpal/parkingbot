#!/usr/bin/env python3
"""
시뮬레이션 CCTV 노드
=====================
빈 주차 슬롯 정보를 발행합니다.
실제에서는 YOLO가 하는 일을 시뮬레이션에서는 설정값으로 대체.
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseArray, Pose, PoseStamped
from std_msgs.msg import String
import json
import math


# 주차 슬롯 정의 (m 단위)
SLOTS = [
    {"id": 1, "cx": 0.15, "cy": 0.10},
    {"id": 2, "cx": 0.40, "cy": 0.10},
    {"id": 3, "cx": 0.65, "cy": 0.10},
    {"id": 4, "cx": 0.90, "cy": 0.10},
]

# 이미 차가 있는 슬롯
OCCUPIED = [1, 3]


class SimCCTV(Node):
    def __init__(self):
        super().__init__('sim_cctv')

        self.declare_parameter('occupied_slots', OCCUPIED)
        self.occupied = self.get_parameter('occupied_slots').value

        # 발행
        self.pub_empty = self.create_publisher(
            PoseArray, '/parking/empty_slots', 10)
        self.pub_target = self.create_publisher(
            PoseStamped, '/parking/target_pose', 10)
        self.pub_info = self.create_publisher(
            String, '/parking/info', 10)

        # 1Hz로 발행
        self.timer = self.create_timer(1.0, self.publish_slots)

        empty_ids = [s["id"] for s in SLOTS if s["id"] not in self.occupied]
        self.get_logger().info(
            f'CCTV 시작 | 점유: {self.occupied} | '
            f'빈 슬롯: {empty_ids}')

    def publish_slots(self):
        now = self.get_clock().now().to_msg()

        empty_slots = [s for s in SLOTS if s["id"] not in self.occupied]

        # ---- PoseArray: 모든 빈 슬롯 ----
        pa = PoseArray()
        pa.header.stamp = now
        pa.header.frame_id = 'world'

        for s in empty_slots:
            p = Pose()
            p.position.x = s["cx"]
            p.position.y = s["cy"]
            pa.poses.append(p)

        self.pub_empty.publish(pa)

        # ---- PoseStamped: 가장 가까운 빈 슬롯 ----
        if empty_slots:
            target_slot = empty_slots[0]  # 첫 번째 빈 슬롯
            ts = PoseStamped()
            ts.header.stamp = now
            ts.header.frame_id = 'world'
            ts.pose.position.x = target_slot["cx"]
            ts.pose.position.y = target_slot["cy"]
            # 주차 방향: 위쪽 (90도)
            ts.pose.orientation.z = math.sin(math.pi / 4)
            ts.pose.orientation.w = math.cos(math.pi / 4)
            self.pub_target.publish(ts)

        # ---- JSON 정보 ----
        info = {
            "total": len(SLOTS),
            "empty": [s["id"] for s in empty_slots],
            "occupied": self.occupied,
            "target_slot": empty_slots[0]["id"] if empty_slots else None,
        }
        msg = String()
        msg.data = json.dumps(info)
        self.pub_info.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimCCTV()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
