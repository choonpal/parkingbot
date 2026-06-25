#!/usr/bin/env python3
"""
==================================================
aruco_relative_pose.py
==================================================
마스터 로봇 카메라로 슬레이브 로봇의 ArUco 마커를 인식하여
두 로봇 간 상대 위치(거리, 각도)를 측정.

역할: 엔코더 강체 기구학의 드리프트를 보정하는 "측정 소스"
  - 엔코더: 빠르지만 누적 오차 (50Hz, 주력)
  - ArUco: 느리지만 정확 (5~10Hz, 보정)

발행:
  /sync/aruco_relative (geometry_msgs/PoseStamped)
    → 마스터 기준 슬레이브의 상대 위치

환경: 마스터 로봇에 카메라 + 슬레이브 앞쪽에 ArUco 마커
"""

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool

import cv2
import numpy as np
import math
import time
import os


class ArucoRelativePose(Node):
    def __init__(self):
        super().__init__('aruco_relative_pose')

        # ============================================
        # 파라미터
        # ============================================
        self.declare_parameter('camera_id', 0)
        self.declare_parameter('frame_width', 1280)
        self.declare_parameter('frame_height', 720)
        self.declare_parameter('marker_id', 1)          # 슬레이브 마커 ID
        self.declare_parameter('marker_size_m', 0.05)   # 마커 한 변 (5cm)
        self.declare_parameter('camera_calib', 'camera_calibration.npz')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('publish_rate', 10.0)    # Hz

        self.camera_id = self.get_parameter('camera_id').value
        self.frame_w = self.get_parameter('frame_width').value
        self.frame_h = self.get_parameter('frame_height').value
        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size_m').value

        # ============================================
        # 카메라 내부 파라미터 로드 (왜곡 보정용)
        # ============================================
        calib_file = self.get_parameter('camera_calib').value
        if os.path.exists(calib_file):
            data = np.load(calib_file)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.get_logger().info(f'카메라 캘리브레이션 로드: {calib_file}')
        else:
            # 캘리브레이션 없으면 대략적 추정값 사용
            self.get_logger().warn(
                f'{calib_file} 없음 — 추정 파라미터 사용 (정확도 낮음)')
            fx = fy = self.frame_w  # 대략적 초점거리
            cx, cy = self.frame_w / 2, self.frame_h / 2
            self.camera_matrix = np.array([
                [fx, 0, cx], [0, fy, cy], [0, 0, 1]], dtype=np.float64)
            self.dist_coeffs = np.zeros(5)

        # ============================================
        # ArUco 검출기 설정
        # ============================================
        dict_name = self.get_parameter('aruco_dict').value
        aruco_dict_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_dict_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params)
        self.get_logger().info(f'ArUco 딕셔너리: {dict_name}, '
                               f'타겟 마커 ID: {self.marker_id}')

        # ============================================
        # 카메라 초기화
        # ============================================
        self.cap = cv2.VideoCapture(self.camera_id)
        self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, self.frame_w)
        self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, self.frame_h)
        if not self.cap.isOpened():
            self.get_logger().error('카메라 열기 실패!')
            raise RuntimeError('camera failed')

        # ============================================
        # 발행
        # ============================================
        self.pub_relative = self.create_publisher(
            PoseStamped, '/sync/aruco_relative', 10)
        self.pub_visible = self.create_publisher(
            Bool, '/sync/aruco_visible', 10)

        # ============================================
        # 인식 루프
        # ============================================
        rate = self.get_parameter('publish_rate').value
        self.timer = self.create_timer(1.0 / rate, self.detect_loop)

        self.last_visible = False
        self.get_logger().info('ArUco 상대 위치 노드 시작')

    def detect_loop(self):
        ret, frame = self.cap.read()
        if not ret:
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        visible = False

        if ids is not None and self.marker_id in ids.flatten():
            # 타겟 마커 인덱스 찾기
            idx = list(ids.flatten()).index(self.marker_id)
            marker_corners = corners[idx]

            # 마커의 3D 자세 추정 (solvePnP)
            # 마커 4개 모서리의 3D 좌표 (마커 중심 기준)
            half = self.marker_size / 2
            obj_points = np.array([
                [-half,  half, 0],
                [ half,  half, 0],
                [ half, -half, 0],
                [-half, -half, 0]
            ], dtype=np.float32)

            img_points = marker_corners[0].astype(np.float32)

            success, rvec, tvec = cv2.solvePnP(
                obj_points, img_points,
                self.camera_matrix, self.dist_coeffs)

            if success:
                visible = True

                # tvec = 카메라 기준 마커 위치 (x, y, z) 미터
                # z = 정면 거리, x = 좌우, y = 상하
                x = float(tvec[0][0])   # 좌우 오프셋
                z = float(tvec[2][0])   # 전방 거리

                # 마커 회전 (yaw 추출)
                rot_mat, _ = cv2.Rodrigues(rvec)
                yaw = math.atan2(rot_mat[1][0], rot_mat[0][0])

                # 상대 위치 발행
                # 로봇 좌표계: x=전방, y=좌측
                msg = PoseStamped()
                msg.header.stamp = self.get_clock().now().to_msg()
                msg.header.frame_id = 'master_base'
                msg.pose.position.x = z      # 전방 거리
                msg.pose.position.y = -x     # 좌우 (부호 변환)
                msg.pose.position.z = 0.0
                msg.pose.orientation.z = math.sin(yaw / 2)
                msg.pose.orientation.w = math.cos(yaw / 2)
                self.pub_relative.publish(msg)

                self.get_logger().info(
                    f'슬레이브 인식: 거리 {z:.3f}m, '
                    f'좌우 {x:+.3f}m, yaw {math.degrees(yaw):+.0f}°',
                    throttle_duration_sec=1.0)

        # 마커 가시성 발행
        vis_msg = Bool()
        vis_msg.data = visible
        self.pub_visible.publish(vis_msg)

        # 가시성 변화 로깅
        if visible != self.last_visible:
            if visible:
                self.get_logger().info('마커 재인식 — ArUco 보정 재개')
            else:
                self.get_logger().warn('마커 시야 벗어남 — 엔코더만으로 주행')
            self.last_visible = visible

    def destroy_node(self):
        if hasattr(self, 'cap'):
            self.cap.release()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    try:
        node = ArucoRelativePose()
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        if 'node' in dict(locals()):
            node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
