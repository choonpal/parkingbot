#!/usr/bin/env python3
"""
==================================================
cctv_fusion_node.py
==================================================
CCTV 기반 인프라 인지 시스템 (90도 수직 천장 카메라)

기능:
  A. YOLO로 차량/로봇/장애물 탐지 → BEV 좌표(cm) 변환
  B. 엔코더(odom)와 CCTV 데이터 매칭 → 로봇 절대 위치 발행
  C. OccupancyGrid 맵 생성 → Nav2에서 사용

구독:
  /robot/odom (nav_msgs/Odometry)

발행:
  /cctv/global_pose (geometry_msgs/PoseWithCovarianceStamped)
  /cctv_bev_map (nav_msgs/OccupancyGrid)

환경: Jetson Orin Nano, Ubuntu 24.04, ROS2 Jazzy/Humble
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy

from nav_msgs.msg import Odometry, OccupancyGrid
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import Header

import cv2
import numpy as np
import math
import time
import os

# YOLO
try:
    from ultralytics import YOLO
    YOLO_OK = True
except ImportError:
    YOLO_OK = False


class CCTVFusionNode(Node):
    """CCTV 인프라 인지 + odom 융합 노드"""

    def __init__(self):
        super().__init__('cctv_fusion_node')

        # ============================================
        # 파라미터 선언
        # ============================================
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('model_path', 'yolov8n-seg.engine')  # TensorRT
        self.declare_parameter('homography_file', 'homography_matrix.npy')
        self.declare_parameter('confidence', 0.4)

        # 맵 파라미터
        self.declare_parameter('map_resolution', 0.05)   # 5cm/픽셀
        self.declare_parameter('map_width_m', 6.0)        # 주차장 가로 (m)
        self.declare_parameter('map_height_m', 4.0)       # 주차장 세로 (m)
        self.declare_parameter('map_frame', 'map')

        # 매칭 파라미터
        self.declare_parameter('match_threshold_m', 0.5)  # 0.5m 이내면 매칭
        self.declare_parameter('odom_timeout_s', 1.0)     # odom 타임아웃

        # 차량 크기 (장애물 박스 크기, m)
        self.declare_parameter('car_size_m', 0.20)

        # 고정 장애물 (기둥 등) - [x_m, y_m, radius_m]
        self.declare_parameter('fixed_obstacles', [])

        # 파라미터 읽기
        self.camera_id = self.get_parameter('camera_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.conf = self.get_parameter('confidence').value
        self.resolution = self.get_parameter('map_resolution').value
        self.map_w_m = self.get_parameter('map_width_m').value
        self.map_h_m = self.get_parameter('map_height_m').value
        self.map_frame = self.get_parameter('map_frame').value
        self.match_thresh = self.get_parameter('match_threshold_m').value
        self.odom_timeout = self.get_parameter('odom_timeout_s').value
        self.car_size = self.get_parameter('car_size_m').value

        # 맵 격자 크기 계산
        self.grid_w = int(self.map_w_m / self.resolution)
        self.grid_h = int(self.map_h_m / self.resolution)

        # ============================================
        # 호모그래피 행렬 로드
        # ============================================
        h_file = self.get_parameter('homography_file').value
        if not os.path.exists(h_file):
            self.get_logger().error(
                f'호모그래피 파일 없음: {h_file} — 캘리브레이션 먼저 실행!')
            raise FileNotFoundError(h_file)
        self.H = np.load(h_file)
        self.get_logger().info(f'호모그래피 로드: {h_file}')

        # ============================================
        # YOLO 모델 로드 (TensorRT engine 우선)
        # ============================================
        if not YOLO_OK:
            self.get_logger().error('ultralytics 미설치!')
            raise ImportError('pip install ultralytics')

        model_path = self.get_parameter('model_path').value
        if not os.path.exists(model_path):
            # engine 파일 없으면 .pt로 폴백
            fallback = 'yolov8n-seg.pt'
            self.get_logger().warn(
                f'{model_path} 없음 → {fallback} 사용 (TensorRT 변환 권장)')
            model_path = fallback
        self.model = YOLO(model_path)
        self.get_logger().info(f'YOLO 로드: {model_path}')

        # COCO 클래스: 2=car, 7=truck, 0=person
        self.vehicle_classes = [2, 7]
        self.obstacle_classes = [0]  # 사람 등

        # ============================================
        # 카메라 초기화
        # ============================================
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            raise RuntimeError('camera open failed')

        # ============================================
        # 상태 변수
        # ============================================
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = 0.0
        self.last_odom_time = 0.0

        # 고정 장애물 파싱
        self.fixed_obstacles = self._parse_fixed_obstacles()

        # ============================================
        # QoS 설정
        # ============================================
        # 맵은 latched (늦게 연결된 노드도 받도록)
        map_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL)

        # ============================================
        # 구독 / 발행
        # ============================================
        self.sub_odom = self.create_subscription(
            Odometry, '/robot/odom', self.odom_callback, 10)

        self.pub_pose = self.create_publisher(
            PoseWithCovarianceStamped, '/cctv/global_pose', 10)
        self.pub_map = self.create_publisher(
            OccupancyGrid, '/cctv_bev_map', map_qos)

        # ============================================
        # 메인 루프 (30fps 목표)
        # ============================================
        self.timer = self.create_timer(1.0 / 30.0, self.main_loop)

        # 통계
        self.frame_count = 0
        self.fps_time = time.time()

        self.get_logger().info(
            f'CCTV 융합 노드 시작 | 맵: {self.grid_w}x{self.grid_h} '
            f'({self.map_w_m}x{self.map_h_m}m @ {self.resolution}m)')

    # ================================================
    # 고정 장애물 파싱
    # ================================================
    def _parse_fixed_obstacles(self):
        """파라미터에서 고정 장애물 좌표 파싱"""
        raw = self.get_parameter('fixed_obstacles').value
        obstacles = []
        # [x1, y1, r1, x2, y2, r2, ...] 형태로 받음
        for i in range(0, len(raw) - 2, 3):
            obstacles.append({
                'x': float(raw[i]),
                'y': float(raw[i+1]),
                'r': float(raw[i+2])
            })
        if obstacles:
            self.get_logger().info(f'고정 장애물 {len(obstacles)}개 등록')
        return obstacles

    # ================================================
    # 좌표 변환: 픽셀 → 실제 (m)
    # ================================================
    def pixel_to_world(self, px, py):
        """
        픽셀 좌표 → 실제 BEV 좌표 (m)
        호모그래피 행렬은 cm 단위로 캘리브레이션했다고 가정 → m로 변환
        """
        pt = np.array([px, py, 1.0], dtype=np.float64)
        result = self.H @ pt
        w = result[2]
        if abs(w) < 1e-10:
            return None, None
        x_cm = result[0] / w
        y_cm = result[1] / w
        return x_cm / 100.0, y_cm / 100.0  # cm → m

    # ================================================
    # 실제 좌표 → 격자 인덱스
    # ================================================
    def world_to_grid(self, x_m, y_m):
        """실제 좌표(m) → OccupancyGrid 인덱스"""
        gx = int(x_m / self.resolution)
        gy = int(y_m / self.resolution)
        if 0 <= gx < self.grid_w and 0 <= gy < self.grid_h:
            return gx, gy
        return None, None

    # ================================================
    # odom 콜백
    # ================================================
    def odom_callback(self, msg):
        """엔코더 기반 추정 위치 수신"""
        self.odom_x = msg.pose.pose.position.x
        self.odom_y = msg.pose.pose.position.y
        q = msg.pose.pose.orientation
        self.odom_yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        self.last_odom_time = time.time()

    # ================================================
    # A. YOLO 탐지 → BEV 변환
    # ================================================
    def detect_objects(self, frame):
        """
        YOLO-seg로 탐지 → BEV 좌표 리스트 + 마스크 반환

        Returns:
            vehicles: [{'x_m', 'y_m', 'mask', 'bbox'}, ...]
            obstacles: 동일 구조 (사람 등)
        """
        results = self.model(frame, conf=self.conf, verbose=False)

        vehicles = []
        obstacles = []

        for result in results:
            if result.boxes is None:
                continue

            masks = result.masks
            for i, box in enumerate(result.boxes):
                cls = int(box.cls[0])
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()

                # 90도 수직 카메라 → bbox 중심 사용 (왜곡 적음)
                cx = (x1 + x2) / 2
                cy = (y1 + y2) / 2

                x_m, y_m = self.pixel_to_world(cx, cy)
                if x_m is None:
                    continue

                obj = {
                    'x_m': x_m,
                    'y_m': y_m,
                    'bbox': (int(x1), int(y1), int(x2), int(y2)),
                    'conf': float(box.conf[0]),
                    'class': cls,
                }

                # 세그멘테이션 마스크 저장 (있으면)
                if masks is not None and i < len(masks):
                    obj['mask'] = masks[i].data[0].cpu().numpy()

                if cls in self.vehicle_classes:
                    vehicles.append(obj)
                elif cls in self.obstacle_classes:
                    obstacles.append(obj)

        return vehicles, obstacles

    # ================================================
    # B. odom-CCTV 매칭 (Data Association)
    # ================================================
    def match_robot(self, vehicles):
        """
        odom 좌표와 가장 가까운 CCTV 객체를 로봇으로 식별

        Returns:
            matched_obj: 매칭된 객체 (없으면 None)
        """
        # odom 신호 체크
        if self.odom_x is None:
            return None
        if time.time() - self.last_odom_time > self.odom_timeout:
            self.get_logger().warn('odom 타임아웃 — 매칭 스킵', throttle_duration_sec=2.0)
            return None
        if not vehicles:
            return None

        # 유클리디안 거리로 가장 가까운 객체 탐색
        min_dist = float('inf')
        matched = None
        for v in vehicles:
            dist = math.sqrt(
                (v['x_m'] - self.odom_x) ** 2 +
                (v['y_m'] - self.odom_y) ** 2)
            if dist < min_dist:
                min_dist = dist
                matched = v

        # 임계값 체크
        if min_dist <= self.match_thresh:
            return matched
        else:
            self.get_logger().warn(
                f'매칭 실패: 최소거리 {min_dist:.2f}m > '
                f'임계값 {self.match_thresh}m',
                throttle_duration_sec=2.0)
            return None

    def publish_global_pose(self, matched_obj):
        """매칭된 CCTV 위치를 global_pose로 발행"""
        msg = PoseWithCovarianceStamped()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        # CCTV 기반 위치 (X, Y)
        msg.pose.pose.position.x = matched_obj['x_m']
        msg.pose.pose.position.y = matched_obj['y_m']
        msg.pose.pose.position.z = 0.0

        # Yaw는 odom 신뢰 (CCTV는 방향 추정 어려움)
        msg.pose.pose.orientation.z = math.sin(self.odom_yaw / 2)
        msg.pose.pose.orientation.w = math.cos(self.odom_yaw / 2)

        # 공분산 (CCTV 위치 신뢰도 높음, yaw는 odom 신뢰)
        cov = [0.0] * 36
        cov[0] = 0.01   # x 분산 (10cm)
        cov[7] = 0.01   # y 분산
        cov[35] = 0.05  # yaw 분산
        msg.pose.covariance = cov

        self.pub_pose.publish(msg)

    # ================================================
    # C. OccupancyGrid 맵 생성
    # ================================================
    def build_occupancy_grid(self, vehicles, obstacles, robot_obj):
        """
        90도 수직 카메라 기준 OccupancyGrid 생성

        값: 0=빈공간, 100=장애물, -1=미확인
        """
        # 빈 맵 (전부 0 = 빈 공간으로 시작, 카메라가 전체를 보므로)
        grid = np.zeros((self.grid_h, self.grid_w), dtype=np.int8)

        car_px = int(self.car_size / self.resolution)  # 차량 크기(격자)

        # --- 고정 장애물 (기둥 등) ---
        for obs in self.fixed_obstacles:
            self._fill_circle(grid, obs['x'], obs['y'], obs['r'], 100)

        # --- 탐지된 차량 (로봇 제외) ---
        for v in vehicles:
            # 매칭된 로봇은 장애물로 안 넣음 (자기 자신)
            if robot_obj is not None and v is robot_obj:
                continue
            self._fill_box(grid, v['x_m'], v['y_m'], car_px, 100)

        # --- 동적 장애물 (사람 등) ---
        for o in obstacles:
            self._fill_box(grid, o['x_m'], o['y_m'], car_px, 100)

        return grid

    def _fill_box(self, grid, x_m, y_m, size_px, value):
        """격자에 사각형 채우기"""
        gx, gy = self.world_to_grid(x_m, y_m)
        if gx is None:
            return
        half = size_px // 2
        y_min = max(0, gy - half)
        y_max = min(self.grid_h, gy + half + 1)
        x_min = max(0, gx - half)
        x_max = min(self.grid_w, gx + half + 1)
        grid[y_min:y_max, x_min:x_max] = value

    def _fill_circle(self, grid, x_m, y_m, r_m, value):
        """격자에 원 채우기 (기둥용)"""
        gx, gy = self.world_to_grid(x_m, y_m)
        if gx is None:
            return
        r_px = int(r_m / self.resolution)
        for dy in range(-r_px, r_px + 1):
            for dx in range(-r_px, r_px + 1):
                if dx*dx + dy*dy <= r_px*r_px:
                    ny, nx = gy + dy, gx + dx
                    if 0 <= ny < self.grid_h and 0 <= nx < self.grid_w:
                        grid[ny, nx] = value

    def publish_map(self, grid):
        """OccupancyGrid 발행"""
        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame

        msg.info.resolution = self.resolution
        msg.info.width = self.grid_w
        msg.info.height = self.grid_h
        # 맵 원점 (좌하단)
        msg.info.origin.position.x = 0.0
        msg.info.origin.position.y = 0.0
        msg.info.origin.position.z = 0.0
        msg.info.origin.orientation.w = 1.0

        # 1차원 배열로 변환 (row-major)
        msg.data = grid.flatten().tolist()

        self.pub_map.publish(msg)

    # ================================================
    # 메인 루프
    # ================================================
    def main_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            self.get_logger().warn('프레임 읽기 실패', throttle_duration_sec=2.0)
            return

        # A. YOLO 탐지 + BEV 변환
        vehicles, obstacles = self.detect_objects(frame)

        # B. odom-CCTV 매칭
        robot_obj = self.match_robot(vehicles)
        if robot_obj is not None:
            self.publish_global_pose(robot_obj)

        # C. OccupancyGrid 생성 + 발행
        grid = self.build_occupancy_grid(vehicles, obstacles, robot_obj)
        self.publish_map(grid)

        # FPS 통계
        self.frame_count += 1
        if time.time() - self.fps_time >= 5.0:
            fps = self.frame_count / (time.time() - self.fps_time)
            matched_str = "MATCHED" if robot_obj else "NO MATCH"
            self.get_logger().info(
                f'FPS: {fps:.1f} | 차량: {len(vehicles)} | '
                f'장애물: {len(obstacles)} | 로봇: {matched_str}')
            self.frame_count = 0
            self.fps_time = time.time()

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = CCTVFusionNode()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f'[오류] {e}')
    finally:
        if 'node' in dict(locals()):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
