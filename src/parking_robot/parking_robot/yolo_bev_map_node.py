#!/usr/bin/env python3
"""
==================================================
[1-1] yolo_bev_map_node
==================================================
천장 카메라 영상 → 맵 생성 + 빈자리/타겟 포착

입력:
  /cctv/image_raw (sensor_msgs/Image) — 천장 카메라
출력:
  /parking/map (nav_msgs/OccupancyGrid) — 주차장 2D 지도
  /parking/target_pose (geometry_msgs/PoseStamped) — 타겟 차량 좌표
  /parking/empty_slots (geometry_msgs/PoseArray) — 빈자리 좌표들

YOLOv8-seg + 호모그래피(BEV) 사용. 90도 수직 천장 카메라.
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from nav_msgs.msg import OccupancyGrid
from geometry_msgs.msg import PoseStamped, PoseArray, Pose
import math
import os

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    from ultralytics import YOLO
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class YoloBevMapNode(Node):
    def __init__(self):
        super().__init__('yolo_bev_map_node')

        # ===== 파라미터 =====
        self.declare_parameter('model_path', 'yolov8n-seg.engine')
        self.declare_parameter('homography_file', 'homography_matrix.npy')
        self.declare_parameter('map_resolution', 0.05)
        self.declare_parameter('map_width_m', 6.0)
        self.declare_parameter('map_height_m', 4.0)
        self.declare_parameter('confidence', 0.4)
        self.declare_parameter('car_size_m', 0.20)
        # 대기공간 영역 [x1, y1, x2, y2] m
        self.declare_parameter('waiting_zone', [0.0, 0.0, 1.2, 1.0])
        # 주차 슬롯 좌표 [x1,y1, x2,y2, ...]
        self.declare_parameter('slot_coords',
                               [1.5, 3.5, 2.5, 3.5, 3.5, 3.5, 4.5, 3.5])

        self.resolution = self.get_parameter('map_resolution').value
        self.map_w_m = self.get_parameter('map_width_m').value
        self.map_h_m = self.get_parameter('map_height_m').value
        self.conf = self.get_parameter('confidence').value
        self.car_size = self.get_parameter('car_size_m').value
        self.waiting_zone = self.get_parameter('waiting_zone').value
        self.grid_w = int(self.map_w_m / self.resolution)
        self.grid_h = int(self.map_h_m / self.resolution)

        # 슬롯 좌표 파싱
        raw = self.get_parameter('slot_coords').value
        self.slots = [{'id': i//2 + 1, 'x': raw[i], 'y': raw[i+1]}
                      for i in range(0, len(raw)-1, 2)]

        # ===== 모델 로드 =====
        self.bridge = CvBridge() if DEPS_OK else None
        self.model = None
        self.H = None
        if DEPS_OK:
            self._load_models()

        # ===== 구독 =====
        self.create_subscription(Image, '/cctv/image_raw',
                                 self.image_cb, 10)

        # ===== 발행 =====
        self.pub_map = self.create_publisher(
            OccupancyGrid, '/parking/map', 10)
        self.pub_target = self.create_publisher(
            PoseStamped, '/parking/target_pose', 10)
        self.pub_empty = self.create_publisher(
            PoseArray, '/parking/empty_slots', 10)

        # 맵은 주기적으로도 발행
        self.create_timer(1.0, self.publish_map_periodic)
        self.latest_obstacles = []

        self.get_logger().info('yolo_bev_map_node 시작')

    def _load_models(self):
        mp = self.get_parameter('model_path').value
        if not os.path.exists(mp):
            mp = 'yolov8n-seg.pt'
        try:
            self.model = YOLO(mp)
            self.get_logger().info(f'YOLO 로드: {mp}')
        except Exception as e:
            self.get_logger().warn(f'YOLO 로드 실패: {e}')
        hf = self.get_parameter('homography_file').value
        if os.path.exists(hf):
            self.H = np.load(hf)
            self.get_logger().info('호모그래피 로드')
        else:
            self.get_logger().warn(f'{hf} 없음 — 캘리브레이션 필요')

    def pixel_to_world(self, px, py):
        if self.H is None:
            return None, None
        pt = np.array([px, py, 1.0])
        r = self.H @ pt
        if abs(r[2]) < 1e-10:
            return None, None
        return r[0]/r[2]/100.0, r[1]/r[2]/100.0

    def in_waiting_zone(self, x, y):
        x1, y1, x2, y2 = self.waiting_zone
        return x1 <= x <= x2 and y1 <= y <= y2

    # ================================================
    # 이미지 콜백 — 메인 처리
    # ================================================
    def image_cb(self, msg):
        if self.model is None:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        results = self.model(frame, conf=self.conf, verbose=False)

        obstacles = []        # 맵용 (모든 차량 위치)
        cars_in_slots = []    # 슬롯 점유 판정용
        target = None         # 대기공간 차량

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
                obstacles.append((wx, wy))
                if self.in_waiting_zone(wx, wy):
                    target = (wx, wy)
                else:
                    cars_in_slots.append((wx, wy))

        # 타겟 발행
        if target:
            self.publish_target(*target)

        # 빈자리 판별 + 발행
        self.publish_empty_slots(cars_in_slots)

        # 맵 갱신
        self.latest_obstacles = obstacles

    def publish_target(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'map'
        msg.pose.position.x = x
        msg.pose.position.y = y
        msg.pose.orientation.w = 1.0
        self.pub_target.publish(msg)

    def publish_empty_slots(self, cars):
        pa = PoseArray()
        pa.header.stamp = self.get_clock().now().to_msg()
        pa.header.frame_id = 'map'
        for slot in self.slots:
            # 슬롯 근처에 차가 있으면 점유
            occupied = any(
                math.hypot(slot['x']-cx, slot['y']-cy) < 0.3
                for cx, cy in cars)
            if not occupied:
                p = Pose()
                p.position.x = slot['x']
                p.position.y = slot['y']
                p.orientation.w = 1.0
                pa.poses.append(p)
        self.pub_empty.publish(pa)

    def publish_map_periodic(self):
        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8) \
            if DEPS_OK else None
        if grid is None:
            return
        # 차량 위치를 장애물로 (접지점 + 크기)
        car_px = int(self.car_size / self.resolution)
        for wx, wy in self.latest_obstacles:
            gx = int(wx / self.resolution)
            gy = int(wy / self.resolution)
            half = car_px // 2
            y1 = max(0, gy-half); y2 = min(self.grid_h, gy+half)
            x1 = max(0, gx-half); x2 = min(self.grid_w, gx+half)
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


def main(args=None):
    rclpy.init(args=args)
    node = YoloBevMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
