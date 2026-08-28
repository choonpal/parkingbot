#!/usr/bin/env python3
"""
==================================================
[2-2] aruco_tracker_node
==================================================
시각 기반 보정기. 카메라로 마커를 보고 오차 계산.

마스터(rear) 카메라가 front 로봇의 ArUco 마커를 인식하여
두 로봇 간 상대 거리/각도를 측정.

입력:
  /rear/marker_camera/image (sensor_msgs/Image) — 전면 카메라
출력:
  /sync/relative_pose (geometry_msgs/PoseStamped) — front-rear 상대
  /sync/marker_visible (std_msgs/Bool) — 마커 가시성
"""

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import Bool
import math
import os

from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
    scale_camera_matrix,
)
from cooperative_parking_robot.aruco_utils import (
    ArucoDetectorCompat,
    normalize_angle,
    relative_yaw_from_rotation,
)

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
        self.declare_parameter('image_topic', '/rear/marker_camera/image')
        self.declare_parameter('marker_id', 0)          # front 로봇 마커 ID
        self.declare_parameter('marker_size_m', 0.05)
        self.declare_parameter('camera_calib', 'rear_camera_calibration.npz')
        self.declare_parameter('aruco_dict', 'DICT_4X4_50')
        self.declare_parameter('yaw_offset_deg', 0.0)
        self.declare_parameter('yaw_sign', 1.0)
        self.declare_parameter('allow_uncalibrated', False)
        self.declare_parameter('gray_gain', 1.0)

        self.marker_id = self.get_parameter('marker_id').value
        self.marker_size = self.get_parameter('marker_size_m').value
        self.yaw_offset = math.radians(
            self.get_parameter('yaw_offset_deg').value)
        self.yaw_sign = float(self.get_parameter('yaw_sign').value)
        if self.yaw_sign not in (-1.0, 1.0):
            raise ValueError('yaw_sign must be +1.0 or -1.0')
        self.gray_gain = float(
            self.get_parameter('gray_gain').value)
        if self.gray_gain <= 0.0:
            raise ValueError('gray_gain must be positive')

        # ===== 카메라 캘리브레이션 =====
        if not DEPS_OK:
            raise RuntimeError(
                'ArUco dependencies missing (cv2, numpy, cv_bridge)')
        self.bridge = CvBridge()
        self._load_calib()
        self._setup_aruco()

        # ===== 구독/발행 =====
        self.image_topic = str(self.get_parameter('image_topic').value)
        if not self.image_topic:
            raise ValueError('image_topic must not be empty')
        self.create_subscription(Image, self.image_topic,
                                 self.image_cb, qos_profile_sensor_data)
        self.pub_pose = self.create_publisher(
            PoseStamped, '/sync/relative_pose', qos_profile_sensor_data)
        self.pub_visible = self.create_publisher(
            Bool, '/sync/marker_visible', qos_profile_sensor_data)

        self.last_visible = False
        self.get_logger().info(
            f'aruco_tracker_node 시작 | image={self.image_topic}')

    def _load_calib(self):
        cf = str(self.get_parameter('camera_calib').value)
        if os.path.exists(os.path.expanduser(cf)):
            try:
                (self.camera_matrix,
                 self.dist_coeffs,
                 source_keys) = load_camera_calibration(cf)
            except Exception as exc:
                raise RuntimeError(
                    f'Rear 카메라 캘리브레이션 로드 실패: {cf}: {exc}') from exc
            self.get_logger().info(
                'Rear 카메라 캘리브레이션 로드 | '
                f'keys={source_keys[0]}/{source_keys[1]}')
            # calibrate_camera 도구는 촬영 해상도를 함께 저장한다. 운용
            # 해상도가 이와 다르면 초점거리가 그만큼 틀리므로, 첫 프레임에서
            # 비교해 스케일을 보정한다. 이 검사가 없으면 거리만 조용히
            # 어긋나 ALIGN 단계에서 원인 불명의 오차로 나타난다.
            self._calib_size = None
            try:
                import numpy as _np
                data = _np.load(os.path.expanduser(cf))
                if 'image_width' in data and 'image_height' in data:
                    self._calib_size = (int(data['image_width']),
                                        int(data['image_height']))
            except Exception:
                self._calib_size = None
            if self._calib_size is None:
                self.get_logger().warn(
                    '캘리브레이션에 해상도 정보 없음 — 운용 해상도가 촬영 '
                    '해상도와 같은지 직접 확인하세요')
        else:
            if not self.get_parameter('allow_uncalibrated').value:
                raise FileNotFoundError(
                    f'{cf} 없음 — 잘못된 상대 pose 발행을 막기 위해 시작 중단')
            self.get_logger().warn(
                '캘리브레이션 없음 — allow_uncalibrated=true로 추정값 사용')
            self.camera_matrix = np.array(
                [[800, 0, 640], [0, 800, 360], [0, 0, 1]], dtype=float)
            self.dist_coeffs = np.zeros(5)
            # 추정 K는 1280x720 기준값이다.
            self._calib_size = (1280, 720)

    def _match_calibration_resolution(self, width, height):
        """운용 해상도와 캘리브레이션 해상도가 다르면 K를 한 번만 보정한다."""
        if getattr(self, '_resolution_checked', False):
            return
        self._resolution_checked = True
        calib = getattr(self, '_calib_size', None)
        if calib is None or (int(width), int(height)) == calib:
            return
        try:
            self.camera_matrix = scale_camera_matrix(
                self.camera_matrix, calib[0], calib[1], int(width),
                int(height))
            self.get_logger().warn(
                f'해상도 불일치 보정: 캘리브레이션 {calib[0]}x{calib[1]} -> '
                f'운용 {width}x{height} (초점거리 스케일 적용)')
        except ValueError as exc:
            # 화면비가 다르면 크롭이므로 초점거리 스케일로 못 고친다.
            raise RuntimeError(
                f'캘리브레이션 {calib[0]}x{calib[1]}과 운용 {width}x{height}의 '
                f'화면비가 다릅니다. 운용 해상도로 다시 캘리브레이션하세요: {exc}'
            ) from exc

    def _setup_aruco(self):
        dict_name = self.get_parameter('aruco_dict').value
        self.detector = ArucoDetectorCompat(cv2, dict_name)

    def image_cb(self, msg):
        frame = self.bridge.imgmsg_to_cv2(msg, 'bgr8')
        self._match_calibration_resolution(msg.width, msg.height)
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if abs(self.gray_gain - 1.0) > 1e-6:
            gray = cv2.convertScaleAbs(
                gray, alpha=self.gray_gain, beta=0.0)
        corners, ids, _ = self.detector.detect_markers(gray)

        visible = False
        if ids is not None and self.marker_id in ids.flatten():
            idx = list(ids.flatten()).index(self.marker_id)
            half = self.marker_size / 2
            obj_pts = np.array([
                [-half, half, 0], [half, half, 0],
                [half, -half, 0], [-half, -half, 0]], dtype=np.float32)
            img_pts = corners[idx][0].astype(np.float32)

            # 일부 OpenCV 4.x 빌드에서 IPPE_SQUARE가 이 마커
            # object-frame 방향에 대해 재투영 오차가 큰 반대면 해를 반환했다.
            # 합성 투영 회귀검증에서 정확한 해를 복원한 ITERATIVE를 고정한다.
            ok, rvec, tvec = cv2.solvePnP(
                obj_pts, img_pts, self.camera_matrix, self.dist_coeffs,
                flags=cv2.SOLVEPNP_ITERATIVE)
            if ok:
                visible = True
                z = float(tvec[2][0])   # 전방 거리
                x = float(tvec[0][0])   # 좌우
                rot, _ = cv2.Rodrigues(rvec)
                # 마커 +Z 법선은 카메라 쪽, 로봇 진행축은 카메라 반대쪽을
                # 향하므로 -R[:,2]의 X-Z 투영을 heading으로 사용한다.
                yaw = normalize_angle(
                    self.yaw_sign * relative_yaw_from_rotation(rot)
                    + self.yaw_offset)

                msg_out = PoseStamped()
                # 처리/전송 지연을 숨기지 않도록 촬영시각을 보존한다.
                msg_out.header.stamp = msg.header.stamp
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
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
