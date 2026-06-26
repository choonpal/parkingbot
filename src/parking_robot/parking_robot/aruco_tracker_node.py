#!/usr/bin/env python3
"""
==================================================
[2-2] aruco_tracker_node
==================================================
시각 기반 보정기. 카메라로 마커를 보고 오차 계산.

마스터(rear) 카메라가 front 로봇의 ArUco 마커를 인식하여
두 로봇 간 상대 거리/각도를 측정.

입력:
  /robot/front_camera/image (sensor_msgs/Image) — 전면 카메라
출력:
  /sync/relative_pose (geometry_msgs/PoseStamped) — front-rear 상대
  /sync/marker_visible (std_msgs/Bool) — 마커 가시성
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import os

try:
    import cv2
    import numpy as np
    from cv_bridge import CvBridge
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class ArucoTrackerNode(Node):
    def __init__(self):
        super().__init__('aruco_tracker_node')

        # ===== 파라미터 =====
        self.declare_parameter('marker_id', 0)          # front 로봇 마커 ID
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('camera_calib', 'camera_calibration.npz')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size_m').value

        # ===== 카메라 캘리브레이션 =====
        self.bridge = CvBridge() if DEPS_OK else None
        if DEPS_OK:
            self._load_calib()
            self._setup_aruco()

        # ===== 구독/발행 =====
        self.create_subscription(Image, '/robot/front_camera/image',
                                 self.image_cb, 10)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/sync/relative_pose', 10)
        self.pub_visible = self.create_publisher(
            Bool, '/sync/marker_visible', 10)

        self.last_visible = False
        self.get_logger().info('aruco_tracker_node 시작')

    def _load_calib(self):
        cf = self.get_parameter('camera_calib').value
        if os.path.exists(cf):
            data = np.load(cf)
            self.camera_matrix = data['camera_matrix']
            self.dist_coeffs = data['dist_coeffs']
            self.get_logger().info('카메라 캘리브레이션 로드')
        else:
            self.get_logger().warn('캘리브레이션 없음 — 추정값 사용')
            self.camera_matrix = np.array(
                [[800, 0, 640], [0, 800, 360], [0, 0, 1]], dtype=float)
            self.dist_coeffs = np.zeros(5)

    def _setup_aruco(self):
        dict_name = self.get_parameter('aruco_dict').value
        aruco_id = getattr(cv2.aruco, dict_name)
        self.aruco_dict = cv2.aruco.getPredefinedDictionary(aruco_id)
        self.aruco_params = cv2.aruco.DetectorParameters()
        self.detector = cv2.aruco.ArucoDetector(
            self.aruco_dict, self.aruco_params)

    def image_cb(self, msg):
        if not DEPS_OK:
            return
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        corners, ids, _ = self.detector.detectMarkers(gray)

        visible = False
        if ids is not None and self.marker_id in ids.flatten():
            idx = list(ids.flatten()).index(self.marker_id)
            half = self.marker_size / 2
            obj_pts = np.array([
                [-half, half, 0], [half, half, 0],
                [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
            img_pts = corners[idx][0].astype(np.float32)

            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.camera_matrix, self.dist_coeffs)
            if ok:
                visible = True
                z = float(tvec[2][0])   # 전방 거리
                x = float(tvec[0][0])   # 좌우
                rot, _ = cv2.Rodrigues(rvec)
                yaw = math.atan2(rot[1][0], rot[0][0])

                msg_out = PoseStamped()
                msg_out.header.stamp = self.get_clock().now().to_msg()
                msg_out.header.frame_id = 'rear_base'
                msg_out.pose.position.x = z       # 전방 거리
                msg_out.pose.position.y = -x      # 좌우
                msg_out.pose.orientation.z = math.sin(yaw/2)
                msg_out.pose.orientation.w = math.cos(yaw/2)
                self.pub_pose.publish(msg_out)

        vis = Bool()
        vis.data = visible
        self.pub_visible.publish(vis)

        if visible != self.last_visible:
            self.get_logger().info(
                '마커 인식' if visible else '마커 놓침 — 엔코더 의존')
            self.last_visible = visible


def main(args=None):
    rclpy.init(args=args)
    node = ArucoTrackerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
