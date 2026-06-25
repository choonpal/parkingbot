#!/usr/bin/env python3
"""
2D 시각화 노드
==============
OpenCV로 주차장, 로봇 2대, 주차 슬롯을 실시간 표시합니다.
"""

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseArray, PoseStamped
from std_msgs.msg import String
import cv2
import numpy as np
import math
import json


# 스케일: 1cm = 5px
SCALE = 5
WORLD_W = 120  # cm
WORLD_H = 80
IMG_W = WORLD_W * SCALE
IMG_H = WORLD_H * SCALE

# 주차 슬롯 (cm)
SLOTS = [
    {"id": 1, "x": 5, "y": 3, "w": 20, "h": 14},
    {"id": 2, "x": 30, "y": 3, "w": 20, "h": 14},
    {"id": 3, "x": 55, "y": 3, "w": 20, "h": 14},
    {"id": 4, "x": 80, "y": 3, "w": 20, "h": 14},
]

# 색상 (BGR)
BG_COLOR = (30, 30, 40)
SLOT_EMPTY = (0, 180, 0)
SLOT_OCCUPIED = (0, 0, 200)
FRONT_COLOR = (240, 160, 50)
REAR_COLOR = (50, 200, 50)
LINK_COLOR = (100, 100, 200)
TARGET_COLOR = (0, 200, 255)
CENTER_COLOR = (0, 100, 255)


def cm_to_px(cx, cy):
    return int(cx * SCALE), int(cy * SCALE)


class Visualizer(Node):
    def __init__(self):
        super().__init__('visualizer')

        # 로봇 위치
        self.front = {'x': 0, 'y': 0, 'theta': 0}
        self.rear = {'x': 0, 'y': 0, 'theta': 0}
        self.target = None
        self.nav_status = {}
        self.occupied_slots = [1, 3]

        # 구독
        self.create_subscription(
            Odometry, '/front/odom', self.front_cb, 10)
        self.create_subscription(
            Odometry, '/rear/odom', self.rear_cb, 10)
        self.create_subscription(
            PoseStamped, '/parking/target_pose', self.target_cb, 10)
        self.create_subscription(
            String, '/nav/status', self.status_cb, 10)
        self.create_subscription(
            String, '/parking/info', self.parking_cb, 10)

        # 30fps 렌더링
        self.timer = self.create_timer(1/30, self.render)
        self.get_logger().info('시각화 시작 — q 키로 종료')

    def front_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.front = {
            'x': p.x * 100, 'y': p.y * 100,  # m→cm
            'theta': math.atan2(2*q.w*q.z, 1-2*q.z*q.z)}

    def rear_cb(self, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        self.rear = {
            'x': p.x * 100, 'y': p.y * 100,
            'theta': math.atan2(2*q.w*q.z, 1-2*q.z*q.z)}

    def target_cb(self, msg):
        self.target = (msg.pose.position.x * 100,
                       msg.pose.position.y * 100)

    def status_cb(self, msg):
        try:
            self.nav_status = json.loads(msg.data)
        except:
            pass

    def parking_cb(self, msg):
        try:
            info = json.loads(msg.data)
            self.occupied_slots = info.get('occupied', [])
        except:
            pass

    def draw_robot(self, img, robot, color, label):
        """로봇을 사각형 + 방향 화살표로 그리기"""
        cx, cy = cm_to_px(robot['x'], robot['y'])
        t = robot['theta']
        w, h = 8 * SCALE // 2, 8 * SCALE // 2  # 로봇 크기

        # 회전된 사각형
        corners = np.array([
            [-w, -h], [w, -h], [w, h], [-w, h]
        ], dtype=np.float32)

        rot = np.array([[math.cos(t), -math.sin(t)],
                        [math.sin(t), math.cos(t)]])
        rotated = (rot @ corners.T).T + np.array([cx, cy])
        pts = rotated.astype(np.int32)

        cv2.fillPoly(img, [pts], color)
        cv2.polylines(img, [pts], True, (255, 255, 255), 1)

        # 방향 화살표
        arrow_len = 20
        ax = int(cx + arrow_len * math.cos(t))
        ay = int(cy + arrow_len * math.sin(t))
        cv2.arrowedLine(img, (cx, cy), (ax, ay), (255, 255, 255), 2)

        # 라벨
        cv2.putText(img, label, (cx - 15, cy - 25),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.4, color, 1)

    def render(self):
        img = np.full((IMG_H, IMG_W, 3), BG_COLOR, dtype=np.uint8)

        # 주차장 경계
        cv2.rectangle(img, (0, 0), (IMG_W-1, IMG_H-1), (80, 80, 80), 1)

        # 주차 슬롯
        for s in SLOTS:
            x1, y1 = cm_to_px(s['x'], s['y'])
            x2, y2 = cm_to_px(s['x']+s['w'], s['y']+s['h'])

            if s['id'] in self.occupied_slots:
                cv2.rectangle(img, (x1, y1), (x2, y2), SLOT_OCCUPIED, -1)
                cv2.rectangle(img, (x1, y1), (x2, y2), (100, 100, 100), 1)
                # 차량 모형
                mx, my = (x1+x2)//2, (y1+y2)//2
                cv2.rectangle(img, (mx-12, my-8), (mx+12, my+8),
                              (50, 50, 150), -1)
                cv2.putText(img, 'CAR', (mx-14, my+4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.3, (200, 200, 200), 1)
            else:
                cv2.rectangle(img, (x1, y1), (x2, y2), SLOT_EMPTY, 1)

            # 슬롯 번호
            cv2.putText(img, f'#{s["id"]}', (x1+3, y2-3),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, (200, 200, 200), 1)

        # 목표 마커
        if self.target:
            tx, ty = cm_to_px(*self.target)
            cv2.drawMarker(img, (tx, ty), TARGET_COLOR,
                           cv2.MARKER_CROSS, 20, 2)
            cv2.putText(img, 'TARGET', (tx+12, ty-5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.35, TARGET_COLOR, 1)

        # 두 로봇 사이 연결선 (강체 링크)
        fp = cm_to_px(self.front['x'], self.front['y'])
        rp = cm_to_px(self.rear['x'], self.rear['y'])
        cv2.line(img, fp, rp, LINK_COLOR, 1, cv2.LINE_AA)

        # 가상 중심점
        center_x = (self.front['x'] + self.rear['x']) / 2
        center_y = (self.front['y'] + self.rear['y']) / 2
        cp = cm_to_px(center_x, center_y)
        cv2.circle(img, cp, 6, CENTER_COLOR, 2)
        cv2.circle(img, cp, 2, CENTER_COLOR, -1)

        # 로봇 그리기
        self.draw_robot(img, self.front, FRONT_COLOR, 'FRONT')
        self.draw_robot(img, self.rear, REAR_COLOR, 'REAR')

        # 상태 텍스트
        y = IMG_H - 15
        state = self.nav_status.get('state', 'IDLE')
        dist = self.nav_status.get('dist_to_target_cm', 0)
        level = self.nav_status.get('sync_level', 'OK')
        speed = self.nav_status.get('speed_scale', 1.0)

        status_color = {'OK': (0,255,0), 'WARNING': (0,200,255),
                        'DANGER': (0,100,255), 'E_STOP': (0,0,255)
                        }.get(level, (200,200,200))

        cv2.putText(img, f'State: {state} | Target: {dist:.1f}cm | '
                    f'Sync: {level} | Speed: {speed*100:.0f}%',
                    (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.4, status_color, 1)

        cv2.putText(img, f'Center: ({center_x:.1f}, {center_y:.1f})cm',
                    (10, y - 20), cv2.FONT_HERSHEY_SIMPLEX, 0.35,
                    CENTER_COLOR, 1)

        cv2.imshow('Parking Robot Simulation', img)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            self.get_logger().info('시각화 종료')
            rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = Visualizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    cv2.destroyAllWindows()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
