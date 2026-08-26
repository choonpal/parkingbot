#!/usr/bin/env python3
"""체커보드로 카메라 내부 파라미터를 구해 ``.npz``로 저장하는 도구.

``aruco_tracker_node``와 ``cctv_rectify_node``는 ``camera_matrix``/``dist_coeffs``
키를 가진 npz를 요구하지만, v1.10까지 그 파일을 만드는 수단이 패키지에 없었다.
Rear 카메라용 ``rear_camera_calibration.npz``가 없으면 ArUco 노드가
FileNotFoundError로 기동을 거부하므로, ID0 관측 경로 전체가 막힌다.

이 노드는 ROS 이미지 토픽을 구독하면서 체커보드를 자동 수집한 뒤,
목표 장수를 채우면 캘리브레이션을 수행하고 파일을 쓴 다음 종료한다.

사용 예 (Rear 라즈베리파이에서 카메라 노드가 떠 있는 상태로)::

    ros2 run cooperative_parking_robot calibrate_camera \\
      --ros-args \\
      -p image_topic:=/rear/marker_camera/image \\
      -p output_path:=~/ros2_ws/src/cooperative_parking_robot/config/rear_camera_calibration.npz \\
      -p board_cols:=9 -p board_rows:=6 -p square_size_m:=0.025

체커보드는 "내부 코너 개수"로 세는 점에 주의한다. A4에 흔한 10x7 칸 보드는
``board_cols=9``, ``board_rows=6``이다.
"""

from __future__ import annotations

import os
import time

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import cv2
except ImportError as exc:  # pragma: no cover - 런타임 환경 의존
    raise ImportError('calibrate_camera requires OpenCV (cv2)') from exc


def _image_to_bgr(msg):
    """cv_bridge 없이 sensor_msgs/Image를 BGR ndarray로 변환한다."""
    encoding = msg.encoding.lower()
    buffer = np.frombuffer(msg.data, dtype=np.uint8)
    if encoding in ('bgr8', 'rgb8'):
        frame = buffer.reshape(msg.height, msg.width, 3)
        if encoding == 'rgb8':
            frame = frame[:, :, ::-1]
        return np.ascontiguousarray(frame)
    if encoding == 'mono8':
        gray = buffer.reshape(msg.height, msg.width)
        return cv2.cvtColor(gray, cv2.COLOR_GRAY2BGR)
    raise ValueError(f'unsupported image encoding: {msg.encoding}')


class CalibrateCameraNode(Node):

    def __init__(self):
        super().__init__('calibrate_camera_node')
        self.declare_parameter('image_topic', '/rear/marker_camera/image')
        self.declare_parameter('output_path', 'camera_calibration.npz')
        self.declare_parameter('board_cols', 9)
        self.declare_parameter('board_rows', 6)
        self.declare_parameter('square_size_m', 0.025)
        self.declare_parameter('target_samples', 20)
        # 같은 자세의 프레임을 연속으로 담으면 해가 나빠진다. 사람이 보드를
        # 옮길 시간을 강제로 확보한다.
        self.declare_parameter('min_sample_interval_s', 1.0)
        self.declare_parameter('max_rms_error_px', 1.0)

        self.image_topic = str(self.get_parameter('image_topic').value)
        self.output_path = os.path.expanduser(
            str(self.get_parameter('output_path').value))
        self.cols = int(self.get_parameter('board_cols').value)
        self.rows = int(self.get_parameter('board_rows').value)
        self.square = float(self.get_parameter('square_size_m').value)
        self.target_samples = int(self.get_parameter('target_samples').value)
        self.min_interval = float(
            self.get_parameter('min_sample_interval_s').value)
        self.max_rms = float(self.get_parameter('max_rms_error_px').value)

        if self.cols < 2 or self.rows < 2:
            raise ValueError('board_cols/board_rows are inner corner counts '
                             'and must be >= 2')
        if self.cols == self.rows:
            # 정사각 보드는 회전 모호성이 생겨 코너 순서가 뒤집힐 수 있다.
            raise ValueError('board_cols must differ from board_rows')
        if self.square <= 0.0:
            raise ValueError('square_size_m must be positive')
        if self.target_samples < 5:
            raise ValueError('target_samples must be at least 5')

        # 보드 평면(z=0)에서의 3D 코너 좌표. 모든 샘플에서 동일하다.
        objp = np.zeros((self.rows * self.cols, 3), np.float32)
        objp[:, :2] = np.mgrid[0:self.cols, 0:self.rows].T.reshape(-1, 2)
        self.object_template = objp * self.square

        self.object_points = []
        self.image_points = []
        self.image_size = None
        self._last_capture = 0.0
        self._done = False

        self.create_subscription(
            Image, self.image_topic, self.image_cb, qos_profile_sensor_data)
        self.get_logger().info(
            f'{self.image_topic} 구독 시작 — 체커보드 '
            f'{self.cols}x{self.rows} (내부 코너), '
            f'{self.target_samples}장 수집 목표')
        self.get_logger().info(
            '보드를 화면 중앙/모서리, 정면/기울임으로 골고루 보여주세요')

    def image_cb(self, msg):
        if self._done:
            return
        now = time.monotonic()
        if now - self._last_capture < self.min_interval:
            return
        try:
            frame = _image_to_bgr(msg)
        except ValueError as exc:
            self.get_logger().error(str(exc))
            return

        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        if self.image_size is None:
            self.image_size = (gray.shape[1], gray.shape[0])
        elif self.image_size != (gray.shape[1], gray.shape[0]):
            # 해상도가 바뀌면 이전 샘플과 섞을 수 없다.
            self.get_logger().error('이미지 해상도가 도중에 바뀌었습니다')
            return

        flags = (cv2.CALIB_CB_ADAPTIVE_THRESH |
                 cv2.CALIB_CB_NORMALIZE_IMAGE |
                 cv2.CALIB_CB_FAST_CHECK)
        found, corners = cv2.findChessboardCorners(
            gray, (self.cols, self.rows), flags)
        if not found:
            return

        corners = cv2.cornerSubPix(
            gray, corners, (11, 11), (-1, -1),
            (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001))
        self.object_points.append(self.object_template.copy())
        self.image_points.append(corners)
        self._last_capture = now
        self.get_logger().info(
            f'수집 {len(self.image_points)}/{self.target_samples}')

        if len(self.image_points) >= self.target_samples:
            self._done = True
            self.solve_and_save()

    def solve_and_save(self):
        self.get_logger().info('캘리브레이션 계산 중...')
        rms, camera_matrix, dist_coeffs, _, _ = cv2.calibrateCamera(
            self.object_points, self.image_points, self.image_size,
            None, None)

        fx = camera_matrix[0, 0]
        fy = camera_matrix[1, 1]
        cx = camera_matrix[0, 2]
        cy = camera_matrix[1, 2]
        self.get_logger().info(
            f'RMS 재투영 오차 = {rms:.4f} px  '
            f'fx={fx:.1f} fy={fy:.1f} cx={cx:.1f} cy={cy:.1f}')

        if rms > self.max_rms:
            # 나쁜 해를 파일로 남기면 이후 거리 추정이 조용히 틀어진다.
            self.get_logger().error(
                f'RMS 오차가 한계({self.max_rms} px)를 초과했습니다. '
                '저장하지 않습니다. 보드를 더 다양한 각도/거리로 다시 촬영하세요')
            rclpy.shutdown()
            return

        directory = os.path.dirname(self.output_path)
        if directory:
            os.makedirs(directory, exist_ok=True)
        np.savez(
            self.output_path,
            camera_matrix=camera_matrix,
            dist_coeffs=dist_coeffs,
            image_width=self.image_size[0],
            image_height=self.image_size[1],
            rms_error_px=rms,
        )
        self.get_logger().info(f'저장 완료: {self.output_path}')
        self.get_logger().info(
            '이 파일을 config/에 두고 colcon build 후 launch의 '
            'camera_calib 인자로 지정하세요')
        rclpy.shutdown()


def main(args=None):
    rclpy.init(args=args)
    node = CalibrateCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        node.get_logger().info('중단됨 — 저장하지 않았습니다')
    finally:
        if rclpy.ok():
            node.destroy_node()
            rclpy.shutdown()


if __name__ == '__main__':
    main()
