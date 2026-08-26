#!/usr/bin/env python3
"""Publish a USB/V4L2 or GStreamer camera as a ROS 2 Image topic."""

from __future__ import annotations

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image

try:
    import cv2
    from cv_bridge import CvBridge
    DEPS_OK = True
except ImportError:
    DEPS_OK = False


def resolve_camera_source(camera_device: str, camera_id: int):
    """Prefer a persistent V4L device path, with an integer ID fallback."""
    device = os.path.expanduser(str(camera_device).strip())
    return device if device else int(camera_id)


class OpenCvCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('output_topic', '/cctv/image_raw')
        self.declare_parameter('frame_id', 'cctv_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('require_camera', True)
        self.declare_parameter('reopen_interval_s', 2.0)

        if not DEPS_OK:
            raise RuntimeError(
                'OpenCV camera dependencies missing (cv2, cv_bridge)')

        self.camera_id = int(self.get_parameter('camera_id').value)
        self.camera_device = str(
            self.get_parameter('camera_device').value).strip()
        self.camera_source = resolve_camera_source(
            self.camera_device, self.camera_id)
        self.pipeline = str(self.get_parameter('gstreamer_pipeline').value)
        self.output_topic = str(self.get_parameter('output_topic').value)
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.require_camera = bool(self.get_parameter('require_camera').value)
        self.reopen_interval = float(
            self.get_parameter('reopen_interval_s').value)

        if self.width <= 0 or self.height <= 0:
            raise ValueError('camera width/height must be positive')
        if self.fps <= 0.0:
            raise ValueError('camera fps must be positive')
        if self.reopen_interval <= 0.0:
            raise ValueError('reopen_interval_s must be positive')
        if not self.output_topic:
            raise ValueError('output_topic must not be empty')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, self.output_topic, qos_profile_sensor_data)
        self.capture = None
        self.last_open_attempt = 0.0
        self.failure_count = 0
        self.actual_reported = False

        self._open_camera(initial=True)
        self.timer = self.create_timer(1.0 / self.fps, self.capture_once)

    def _open_camera(self, initial: bool = False) -> bool:
        now = time.monotonic()
        if not initial and now - self.last_open_attempt < self.reopen_interval:
            return False
        self.last_open_attempt = now
        self._release_camera()

        if self.pipeline:
            self.capture = cv2.VideoCapture(
                self.pipeline, cv2.CAP_GSTREAMER)
            source = 'GStreamer pipeline'
        elif isinstance(self.camera_source, str):
            self.capture = cv2.VideoCapture(
                self.camera_source, cv2.CAP_V4L2)
            source = f'camera_device={self.camera_source}'
        else:
            self.capture = cv2.VideoCapture(self.camera_source)
            source = f'camera_id={self.camera_source}'

        if self.capture is not None:
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            self.capture.set(cv2.CAP_PROP_FPS, self.fps)
            self.capture.set(cv2.CAP_PROP_BUFFERSIZE, self.buffer_size)

        if self.capture is None or not self.capture.isOpened():
            message = f'CCTV camera open failed: {source}'
            if initial and self.require_camera:
                self._release_camera()
                raise RuntimeError(message)
            self.get_logger().error(message, throttle_duration_sec=5.0)
            self._release_camera()
            return False

        self.failure_count = 0
        self.actual_reported = False
        self.get_logger().info(
            f'CCTV camera opened: {source} -> {self.output_topic}')
        return True

    def capture_once(self) -> None:
        if self.capture is None or not self.capture.isOpened():
            self._open_camera()
            return

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self.failure_count += 1
            self.get_logger().warn(
                f'CCTV frame read failed ({self.failure_count}); reopening',
                throttle_duration_sec=2.0)
            self._release_camera()
            return

        if not self.actual_reported:
            height, width = frame.shape[:2]
            actual_fps = float(self.capture.get(cv2.CAP_PROP_FPS))
            if width != self.width or height != self.height:
                self.get_logger().warn(
                    'camera ignored requested resolution: '
                    f'requested={self.width}x{self.height}, '
                    f'actual={width}x{height}')
            self.get_logger().info(
                f'CCTV first frame: {width}x{height}, reported_fps={actual_fps:.2f}')
            self.actual_reported = True

        message = self.bridge.cv2_to_imgmsg(frame, encoding='bgr8')
        message.header.stamp = self.get_clock().now().to_msg()
        message.header.frame_id = self.frame_id
        self.publisher.publish(message)

    def _release_camera(self) -> None:
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:
                pass
        self.capture = None

    def destroy_node(self):
        self._release_camera()
        return super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = OpenCvCameraNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
