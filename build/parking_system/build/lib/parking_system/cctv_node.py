#!/usr/bin/env python3
"""
==================================================
cctv_node.py — 천장 CCTV 노드
==================================================
천장 중앙 카메라로 차량 인식 + 빈자리 탐색.

역할:
  1. YOLOv8-seg로 대기공간의 target 차량 인식
     → /cctv/target_detected 발행
  2. 빈자리 탐색 서비스 제공 (/cctv/find_empty_slot)
     → 호출되면 가까운 빈자리를 /cctv/reach_pose로 발행
  3. BEV occupancy grid 생성 → /cctv_bev_map 발행

실행: ros2 run parking_system cctv
"""

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid
from std_srvs.srv import Trigger
import math
import time

try:
    import cv2
    import numpy as np
    from ultralytics import YOLO
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class CCTVNode(Node):
    def __init__(self):
        super().__init__('cctv_node')

        # ===== 파라미터 =====
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('model_path', 'yolov8n-seg.engine')
        self.declare_parameter('homography_file', 'homography_matrix.npy')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width_m', 6.0)
        self.declare_parameter('map_height_m', 4.0)
        # 대기공간 영역 (이 안에 차 들어오면 target)
        self.declare_parameter('waiting_zone', [0.0, 0.0, 1.0, 1.0])

        self.resolution = self.get_parameter('map_resolution').value
        self.map_w_m = self.get_parameter('map_width_m').value
        self.map_h_m = self.get_parameter('map_height_m').value
        self.grid_w = int(self.map_w_m / self.resolution)
        self.grid_h = int(self.map_h_m / self.resolution)
        wz = self.get_parameter('waiting_zone').value
        self.waiting_zone = wz  # [x1, y1, x2, y2]

        # ===== 주차 슬롯 정의 (m) =====
        # 실제로는 YOLO-seg가 인식하지만, 좌표는 미리 알고 있음
        self.slots = [
            {'id': 1, 'x': 1.5, 'y': 3.5, 'occupied': True},
            {'id': 2, 'x': 2.5, 'y': 3.5, 'occupied': False},
            {'id': 3, 'x': 3.5, 'y': 3.5, 'occupied': True},
            {'id': 4, 'x': 4.5, 'y': 3.5, 'occupied': False},
        ]

        # ===== YOLO / 호모그래피 로드 =====
        self.model = None
        self.H = None
        if DEPS_OK:
            self._load_models()

        # ===== 카메라 =====
        self.cap = None
        if DEPS_OK:
            cam_id = self.get_parameter('camera_id').value
            self.cap = cv2.VideoCapture(cam_id)
            if not self.cap.isOpened():
                self.get_logger().warn('카메라 없음 — 시뮬레이션 모드')
                self.cap = None

        # ===== 상태 =====
        self.car_lifted = False  # 차량이 들렸는지

        # ===== 발행 =====
        self.pub_target = self.create_publisher(
            PoseStamped, '/cctv/target_detected', 10)
        self.pub_reach = self.create_publisher(
            PoseStamped, '/cctv/reach_pose', 10)
        self.pub_map = self.create_publisher(
            OccupancyGrid, '/cctv_bev_map', 10)

        # ===== 서비스 (빈자리 탐색) =====
        self.srv_slot = self.create_service(
            Trigger, '/cctv/find_empty_slot',
            self.find_empty_slot_cb)

        # ===== 루프 =====
        self.create_timer(0.1, self.detect_loop)   # 10Hz 탐지
        self.create_timer(1.0, self.publish_map)    # 1Hz 맵

        self.get_logger().info('CCTV 노드 시작')

    def _load_models(self):
        import os
        import numpy as np
        model_path = self.get_parameter('model_path').value
        if not os.path.exists(model_path):
            model_path = 'yolov8n-seg.pt'
        try:
            self.model = YOLO(model_path)
            self.get_logger().info(f'YOLO 로드: {model_path}')
        except Exception as e:
            self.get_logger().warn(f'YOLO 로드 실패: {e}')

        h_file = self.get_parameter('homography_file').value
        if os.path.exists(h_file):
            self.H = np.load(h_file)
            self.get_logger().info('호모그래피 로드 완료')

    # ================================================
    # 차량 탐지 루프
    # ================================================
    def detect_loop(self):
        if self.cap is None or self.model is None:
            # 시뮬레이션: 5초 후 가짜 차량 발행
            if not hasattr(self, '_sim_start'):
                self._sim_start = time.time()
            if time.time() - self._sim_start > 5.0 and not self.car_lifted:
                self.publish_fake_target()
            return

        ret, frame = self.cap.read()
        if not ret:
            return

        results = self.model(frame, conf=0.4, verbose=False)
        for result in results:
            if result.boxes is None:
                continue
            for box in result.boxes:
                cls = int(box.cls[0])
                if cls not in [2, 7]:  # car, truck
                    continue
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                cx, cy = (x1+x2)/2, (y1+y2)/2
                wx, wy = self.pixel_to_world(cx, cy)
                if wx is None:
                    continue
                # 대기공간 안에 있으면 target
                if self.in_waiting_zone(wx, wy):
                    self.publish_target(wx, wy)
                    break

    def pixel_to_world(self, px, py):
        if self.H is None:
            return None, None
        import numpy as np
        pt = np.array([px, py, 1.0])
        r = self.H @ pt
        if abs(r[2]) < 1e-10:
            return None, None
        return r[0]/r[2]/100.0, r[1]/r[2]/100.0

    def in_waiting_zone(self, x, y):
        x1, y1, x2, y2 = self.waiting_zone
        return x1 <= x <= x2 and y1 <= y <= y2

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub_target.publish(msg)

    def publish_fake_target(self):
        """시뮬레이션용 가짜 차량"""
        self.publish_target(0.5, 0.5)
        self.get_logger().info('[시뮬] 가짜 차량 발행 (0.5, 0.5)',
                               throttle_duration_sec=3.0)

    # ================================================
    # 빈자리 탐색 서비스
    # ================================================
    def find_empty_slot_cb(self, request, response):
        """가장 가까운 빈자리를 찾아서 reach 발행"""
        self.car_lifted = True  # 차량 들림 표시

        empty = [s for s in self.slots if not s['occupied']]
        if not empty:
            response.success = False
            response.message = '빈자리 없음'
            return response

        # 대기공간(0.5, 0.5)에서 가장 가까운 빈자리
        wait_x, wait_y = 0.5, 0.5
        nearest = min(empty, key=lambda s:
                      math.sqrt((s['x']-wait_x)**2 + (s['y']-wait_y)**2))

        # reach 발행
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = nearest['x']
        msg.pose.position.y = nearest['y']
        msg.pose.orientation.w = 1.0
        self.pub_reach.publish(msg)

        response.success = True
        response.message = f"슬롯 {nearest['id']} → ({nearest['x']}, {nearest['y']})"
        self.get_logger().info(f'빈자리 응답: {response.message}')
        return response

    # ================================================
    # OccupancyGrid 맵
    # ================================================
    def publish_map(self):
        import numpy as np
        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)

        # 점유된 슬롯을 장애물로
        for s in self.slots:
            if s['occupied']:
                gx = int(s['x'] / self.resolution)
                gy = int(s['y'] / self.resolution)
                size = int(0.4 / self.resolution)
                y1 = max(0, gy-size//2)
                y2 = min(self.grid_h, gy+size//2)
                x1 = max(0, gx-size//2)
                x2 = min(self.grid_w, gx+size//2)
                grid[y1:y2, x1:x2] = 100

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.flatten().tolist()
        self.pub_map.publish(msg)

    def destroy_node(self):
        if self.cap is not None:
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = CCTVNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
