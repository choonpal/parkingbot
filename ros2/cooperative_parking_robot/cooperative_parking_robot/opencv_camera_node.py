#!/usr/bin/env python3
"""Publish a USB/V4L2 or GStreamer camera as a ROS 2 Image topic."""

from __future__ import annotations

import os
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Image
from std_msgs.msg import Bool

from cooperative_parking_robot.latest_qos import STATE_LATEST_QOS

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


def preview_frame_due(now: float, last: float | None, period: float) -> bool:
    """Return whether a rate-limited diagnostic preview should publish."""
    return last is None or now - last >= period


class OpenCvCameraNode(Node):
    def __init__(self):
        super().__init__('opencv_camera_node')

        self.declare_parameter('camera_id', 0)
        self.declare_parameter('camera_device', '')
        self.declare_parameter('gstreamer_pipeline', '')
        self.declare_parameter('output_topic', '/cctv/image_raw')
        self.declare_parameter('preview_topic', '')
        self.declare_parameter('preview_width', 640)
        self.declare_parameter('preview_height', 360)
        self.declare_parameter('preview_fps', 4.0)
        self.declare_parameter('frame_id', 'cctv_camera')
        self.declare_parameter('width', 1280)
        self.declare_parameter('height', 720)
        self.declare_parameter('fps', 30.0)
        # Capture cadence/format can differ from the ROS publish cadence.
        # Zero preserves the legacy behavior of using ``fps`` for capture.
        self.declare_parameter('capture_fps', 0.0)
        self.declare_parameter('v4l2_fourcc', '')
        self.declare_parameter('buffer_size', 1)
        self.declare_parameter('require_camera', True)
        self.declare_parameter('reopen_interval_s', 2.0)
        # Empty runtime_enable_topic preserves the legacy always-on contract.
        # When configured, the device remains open and is sampled slowly while
        # disabled so UVC startup/buffering cost does not hit a motion phase.
        self.declare_parameter('runtime_enable_topic', '')
        self.declare_parameter('runtime_ready_topic', '')
        self.declare_parameter('start_enabled', True)
        self.declare_parameter('standby_fps', 1.0)
        self.declare_parameter('activation_drop_frames', 2)

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
        self.preview_topic = str(
            self.get_parameter('preview_topic').value).strip()
        self.frame_id = str(self.get_parameter('frame_id').value)
        self.width = int(self.get_parameter('width').value)
        self.height = int(self.get_parameter('height').value)
        self.fps = float(self.get_parameter('fps').value)
        self.capture_fps = float(
            self.get_parameter('capture_fps').value)
        self.v4l2_fourcc = str(
            self.get_parameter('v4l2_fourcc').value).strip().upper()
        self.preview_width = int(
            self.get_parameter('preview_width').value)
        self.preview_height = int(
            self.get_parameter('preview_height').value)
        self.preview_fps = float(
            self.get_parameter('preview_fps').value)
        self.buffer_size = int(self.get_parameter('buffer_size').value)
        self.require_camera = bool(self.get_parameter('require_camera').value)
        self.reopen_interval = float(
            self.get_parameter('reopen_interval_s').value)
        self.runtime_enable_topic = str(
            self.get_parameter('runtime_enable_topic').value).strip()
        self.runtime_ready_topic = str(
            self.get_parameter('runtime_ready_topic').value).strip()
        self.runtime_gated = bool(self.runtime_enable_topic)
        self.runtime_enabled = (
            bool(self.get_parameter('start_enabled').value)
            if self.runtime_gated else True)
        self.standby_fps = float(
            self.get_parameter('standby_fps').value)
        self.activation_drop_frames = int(
            self.get_parameter('activation_drop_frames').value)

        if self.width <= 0 or self.height <= 0:
            raise ValueError('camera width/height must be positive')
        if self.fps <= 0.0:
            raise ValueError('camera fps must be positive')
        if self.capture_fps < 0.0:
            raise ValueError('capture_fps must be non-negative')
        if self.v4l2_fourcc and len(self.v4l2_fourcc) != 4:
            raise ValueError('v4l2_fourcc must be empty or exactly 4 chars')
        if self.preview_topic and (
                self.preview_width <= 0 or self.preview_height <= 0 or
                self.preview_width > self.width or
                self.preview_height > self.height or
                self.preview_fps <= 0.0 or self.preview_fps > self.fps):
            raise ValueError(
                'preview size must fit the main image and preview_fps must '
                'be in (0, fps]')
        if self.reopen_interval <= 0.0:
            raise ValueError('reopen_interval_s must be positive')
        if not 0.0 <= self.standby_fps <= self.fps:
            raise ValueError('standby_fps must be in [0, fps]')
        if self.activation_drop_frames < 0:
            raise ValueError('activation_drop_frames must be non-negative')
        if not self.output_topic:
            raise ValueError('output_topic must not be empty')

        self.bridge = CvBridge()
        self.publisher = self.create_publisher(
            Image, self.output_topic, qos_profile_sensor_data)
        self.preview_publisher = None
        self.preview_period = None
        self.last_preview_publish = None
        if self.preview_topic:
            self.preview_publisher = self.create_publisher(
                Image, self.preview_topic, qos_profile_sensor_data)
            self.preview_period = 1.0 / self.preview_fps
        self.capture = None
        self.last_open_attempt = 0.0
        self.failure_count = 0
        self.actual_reported = False
        self.last_standby_read = None
        self.activation_drop_remaining = (
            self.activation_drop_frames if self.runtime_enabled else 0)
        self.runtime_ready = False
        self.ready_publisher = None
        if self.runtime_ready_topic:
            self.ready_publisher = self.create_publisher(
                Bool, self.runtime_ready_topic, STATE_LATEST_QOS)
        if self.runtime_gated:
            self.create_subscription(
                Bool, self.runtime_enable_topic, self.runtime_enable_cb,
                STATE_LATEST_QOS)

        self._open_camera(initial=True)
        self.timer = self.create_timer(1.0 / self.fps, self.capture_once)
        self._set_runtime_ready(False, force=True)

    def _set_runtime_ready(self, ready: bool, *, force=False) -> None:
        ready = bool(ready)
        changed = ready != self.runtime_ready
        self.runtime_ready = ready
        if self.ready_publisher is not None and (changed or force):
            self.ready_publisher.publish(Bool(data=ready))

    def runtime_enable_cb(self, msg) -> None:
        enabled = bool(msg.data)
        if enabled == self.runtime_enabled:
            return
        self.runtime_enabled = enabled
        self._set_runtime_ready(False)
        self.last_standby_read = None
        self.activation_drop_remaining = (
            self.activation_drop_frames if enabled else 0)
        self.get_logger().info(
            f'camera runtime {"enabled" if enabled else "standby"}: '
            f'{self.output_topic}')

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
            if self.v4l2_fourcc and not self.pipeline:
                self.capture.set(
                    cv2.CAP_PROP_FOURCC,
                    cv2.VideoWriter_fourcc(*self.v4l2_fourcc))
            self.capture.set(cv2.CAP_PROP_FRAME_WIDTH, self.width)
            self.capture.set(cv2.CAP_PROP_FRAME_HEIGHT, self.height)
            requested_capture_fps = self.capture_fps or self.fps
            self.capture.set(cv2.CAP_PROP_FPS, requested_capture_fps)
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
            self._set_runtime_ready(False)
            self._open_camera()
            return

        now = time.monotonic()
        if not self.runtime_enabled:
            if (self.standby_fps <= 0.0 or
                    not preview_frame_due(
                        now, self.last_standby_read,
                        1.0 / self.standby_fps)):
                return
            self.last_standby_read = now

        ok, frame = self.capture.read()
        if not ok or frame is None:
            self._set_runtime_ready(False)
            self.failure_count += 1
            self.get_logger().warn(
                f'CCTV frame read failed ({self.failure_count}); reopening',
                throttle_duration_sec=2.0)
            self._release_camera()
            return

        # Disabled reads only drain/prewarm the UVC buffer. No image leaves
        # this node and downstream ArUco therefore consumes no CPU.
        if not self.runtime_enabled:
            return
        if self.activation_drop_remaining > 0:
            self.activation_drop_remaining -= 1
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
        self._set_runtime_ready(True)

        now = time.monotonic()
        if (self.preview_publisher is not None and
                preview_frame_due(
                    now, self.last_preview_publish, self.preview_period)):
            if (frame.shape[1] == self.preview_width and
                    frame.shape[0] == self.preview_height):
                preview = frame
            else:
                preview = cv2.resize(
                    frame, (self.preview_width, self.preview_height),
                    interpolation=cv2.INTER_AREA)
            preview_message = self.bridge.cv2_to_imgmsg(
                preview, encoding='bgr8')
            preview_message.header.stamp = message.header.stamp
            preview_message.header.frame_id = self.frame_id
            self.preview_publisher.publish(preview_message)
            self.last_preview_publish = now

    def _release_camera(self) -> None:
        if self.capture is not None:
            try:
                self.capture.release()
            except Exception:
                pass
        self.capture = None

    def destroy_node(self):
        self._set_runtime_ready(False)
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
