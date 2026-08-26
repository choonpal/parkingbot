#!/usr/bin/env python3
"""Rectify the ceiling-camera image before YOLO and homography processing.

The output image keeps the same dimensions as the input. Both the YOLO/BEV
node and the CCTV robot-marker node must subscribe to this same rectified topic,
and the homography must be calibrated on this output coordinate system.
"""

from __future__ import annotations

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
    scale_camera_matrix,
)

try:
    import cv2
    from cv_bridge import CvBridge
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


class CctvRectifyNode(Node):
    def __init__(self):
        super().__init__('cctv_rectify_node')

        self.declare_parameter('input_topic', '/cctv/image_raw')
        self.declare_parameter('output_topic', '/cctv/image_rect')
        self.declare_parameter(
            'camera_calib', 'cctv_camera_calibration.npz')
        # The received NPZ does not store image size. Keep these at 0 only when
        # the live stream uses the exact calibration resolution.
        self.declare_parameter('calibration_width_px', 0)
        self.declare_parameter('calibration_height_px', 0)
        self.declare_parameter('expected_width_px', 0)
        self.declare_parameter('expected_height_px', 0)
        self.declare_parameter('require_exact_resolution', False)

        if not DEPS_OK:
            raise RuntimeError(
                'CCTV rectification dependencies missing (cv2, numpy, cv_bridge)')

        self.input_topic = str(self.get_parameter('input_topic').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.calibration_width = int(
            self.get_parameter('calibration_width_px').value)
        self.calibration_height = int(
            self.get_parameter('calibration_height_px').value)
        self.expected_width = int(
            self.get_parameter('expected_width_px').value)
        self.expected_height = int(
            self.get_parameter('expected_height_px').value)
        self.require_exact_resolution = bool(
            self.get_parameter('require_exact_resolution').value)
        if bool(self.calibration_width) != bool(self.calibration_height):
            raise ValueError(
                'calibration_width_px and calibration_height_px must both be '
                'zero or both be positive')
        if bool(self.expected_width) != bool(self.expected_height):
            raise ValueError(
                'expected_width_px and expected_height_px must both be zero '
                'or both be positive')
        if (self.require_exact_resolution and
                (self.expected_width <= 0 or self.expected_height <= 0)):
            raise ValueError(
                'require_exact_resolution needs positive expected dimensions')

        calib_path = str(self.get_parameter('camera_calib').value)
        try:
            (self.base_camera_matrix,
             self.dist_coeffs,
             source_keys) = load_camera_calibration(calib_path)
        except Exception as exc:
            raise RuntimeError(
                f'CCTV calibration load failed: {calib_path}: {exc}') from exc

        self.bridge = CvBridge()
        self.map1 = None
        self.map2 = None
        self.map_size = None
        self.effective_camera_matrix = None
        self._assumption_warned = False

        self.publisher = self.create_publisher(
            Image, self.output_topic, qos_profile_sensor_data)
        self.create_subscription(
            Image, self.input_topic, self.image_cb, qos_profile_sensor_data)

        self.get_logger().info(
            'CCTV calibration loaded | '
            f'keys={source_keys[0]}/{source_keys[1]} | '
            f'fx={self.base_camera_matrix[0, 0]:.2f} '
            f'fy={self.base_camera_matrix[1, 1]:.2f} | '
            f'dist_n={self.dist_coeffs.size} | '
            f'{self.input_topic} -> {self.output_topic}')

    def _ensure_maps(self, width: int, height: int) -> None:
        if self.map_size == (width, height):
            return

        if self.calibration_width > 0:
            camera_matrix = scale_camera_matrix(
                self.base_camera_matrix,
                self.calibration_width,
                self.calibration_height,
                width,
                height,
            )
        else:
            camera_matrix = self.base_camera_matrix.copy()
            cx = float(camera_matrix[0, 2])
            cy = float(camera_matrix[1, 2])
            if not (0.0 <= cx < width and 0.0 <= cy < height):
                raise RuntimeError(
                    'live CCTV resolution is incompatible with calibration '
                    f'principal point: frame={width}x{height}, cx={cx:.2f}, '
                    f'cy={cy:.2f}; provide calibration_width_px/height_px')
            if not self._assumption_warned:
                self.get_logger().warn(
                    'calibration NPZ has no image-size metadata; assuming the '
                    f'live {width}x{height} stream is the calibration resolution')
                self._assumption_warned = True

        # Preserve original K as the rectified projection matrix. This keeps
        # output size and a deterministic pixel coordinate system; homography
        # must be measured from this exact /cctv/image_rect output.
        self.map1, self.map2 = cv2.initUndistortRectifyMap(
            camera_matrix,
            self.dist_coeffs,
            None,
            camera_matrix,
            (width, height),
            cv2.CV_32FC1,
        )
        self.effective_camera_matrix = camera_matrix
        self.map_size = (width, height)
        self.get_logger().info(
            f'CCTV undistort map ready: {width}x{height} | '
            'homography must be calibrated on /cctv/image_rect')

    def image_cb(self, msg: Image) -> None:
        frame = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        height, width = frame.shape[:2]
        if (self.require_exact_resolution and
                (width != self.expected_width or
                 height != self.expected_height)):
            self.get_logger().error(
                'CCTV frame resolution mismatch; dropping frame to protect '
                'the calibrated Homography coordinate system: '
                f'expected={self.expected_width}x{self.expected_height}, '
                f'actual={width}x{height}',
                throttle_duration_sec=5.0)
            return
        self._ensure_maps(width, height)
        rectified = cv2.remap(
            frame,
            self.map1,
            self.map2,
            interpolation=cv2.INTER_LINEAR,
            borderMode=cv2.BORDER_CONSTANT,
        )
        output = self.bridge.cv2_to_imgmsg(rectified, encoding='bgr8')
        output.header = msg.header
        self.publisher.publish(output)


def main(args=None):
    rclpy.init(args=args)
    node = CctvRectifyNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
