#!/usr/bin/env python3
"""Dual-camera BEV detection with exactly one YOLO/TensorRT model.

Each camera keeps the existing YoloBevMapNode state (homography, optical
geometry, coverage and sequence), while both instances reuse one raw
Ultralytics model inside this process.  Image callbacks only retain the latest
message; a round-robin timer performs at most one inference per tick so a busy
camera cannot starve its peer.
"""

from __future__ import annotations

import os
import threading

import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter

from cooperative_parking_robot.vision_utils import (
    load_yolo_model,
    normalize_model_mode,
)
from cooperative_parking_robot.yolo_bev_map_production_node import (
    YoloBevMapNode,
)


class SharedYoloModelProvider:
    """Load one model and reject attempts to mix model assets or modes."""

    retain_on_suspend = True

    def __init__(self):
        self._lock = threading.Lock()
        self._key = None
        self._model = None
        self._task = None
        self.load_count = 0

    def acquire(self, yolo_factory, model_path, model_mode):
        key = (
            os.path.realpath(os.path.expanduser(str(model_path))),
            normalize_model_mode(model_mode),
        )
        with self._lock:
            if self._model is None:
                self._model, self._task = load_yolo_model(
                    yolo_factory, key[0], key[1])
                self._key = key
                self.load_count += 1
                return self._model, self._task, True
            if key != self._key:
                raise ValueError(
                    'shared YOLO cameras must use the same model and mode: '
                    f'{self._key!r} != {key!r}')
            return self._model, self._task, False


class RoundRobinCameraSelector:
    """Choose ready camera indices fairly, skipping cameras with no new frame."""

    def __init__(self, camera_count):
        if int(camera_count) <= 0:
            raise ValueError('camera_count must be positive')
        self.camera_count = int(camera_count)
        self.cursor = 0

    def choose(self, ready):
        if len(ready) != self.camera_count:
            raise ValueError('ready length must match camera_count')
        for offset in range(self.camera_count):
            index = (self.cursor + offset) % self.camera_count
            if bool(ready[index]):
                self.cursor = (index + 1) % self.camera_count
                return index
        return None


class SharedCameraYoloNode(YoloBevMapNode):
    """Camera-specific BEV state whose inference is driven by a coordinator."""

    _PEER_PARAMETER_DEFAULTS = {
        'shared_inference_rate_hz': 10.0,
        'shared_cam2_image_topic': '/cctv2/image_rect',
        'shared_cam2_detection_topic': '/cctv2/detections',
        'shared_cam2_homography_file': '',
        'shared_cam2_ground_x_m': 0.0,
        'shared_cam2_ground_y_m': 0.0,
        'shared_cam2_height_m': 0.0,
    }

    def __init__(self, *, declare_peer_parameters=False, **kwargs):
        self._latest_image = None
        self._latest_generation = 0
        self._processed_generation = 0
        self._scheduled_cameras = None
        self._camera_selector = None
        self._inference_timer = None
        super().__init__(**kwargs)
        if declare_peer_parameters:
            for name, default in self._PEER_PARAMETER_DEFAULTS.items():
                self.declare_parameter(name, default)

    def image_cb(self, msg):
        # SENSOR_LATEST_QOS already bounds DDS queueing.  This additional slot
        # makes the scheduling contract explicit while inference is busy.
        self._latest_image = msg
        self._latest_generation += 1

    def has_pending_image(self):
        return (
            not self.detector_suspended and
            self.model is not None and
            self._latest_image is not None and
            self._latest_generation != self._processed_generation)

    def process_latest_image(self):
        if not self.has_pending_image():
            return False
        message = self._latest_image
        self._processed_generation = self._latest_generation
        super().image_cb(message)
        return True

    def start_shared_scheduler(self, cameras, rate_hz):
        rate_hz = float(rate_hz)
        if rate_hz <= 0.0:
            raise ValueError('shared_inference_rate_hz must be positive')
        self._scheduled_cameras = tuple(cameras)
        self._camera_selector = RoundRobinCameraSelector(len(cameras))
        self._inference_timer = self.create_timer(
            1.0 / rate_hz, self._run_shared_inference)
        self.get_logger().info(
            'shared YOLO scheduler active | '
            f'cameras={[camera.camera_id for camera in cameras]} | '
            f'total_rate={rate_hz:.1f}Hz | model_instances=1')

    def _run_shared_inference(self):
        cameras = self._scheduled_cameras
        if not cameras:
            return
        index = self._camera_selector.choose([
            camera.has_pending_image() for camera in cameras])
        if index is not None:
            cameras[index].process_latest_image()


def _camera2_overrides(source):
    replacements = {
        'camera_id': 'cam2',
        'image_topic': source.get_parameter(
            'shared_cam2_image_topic').value,
        'detection_topic': source.get_parameter(
            'shared_cam2_detection_topic').value,
        'homography_file': source.get_parameter(
            'shared_cam2_homography_file').value,
        'camera_ground_x_m': source.get_parameter(
            'shared_cam2_ground_x_m').value,
        'camera_ground_y_m': source.get_parameter(
            'shared_cam2_ground_y_m').value,
        'camera_height_m': source.get_parameter(
            'shared_cam2_height_m').value,
        # The timer is the only rate limiter in the shared runtime.
        'process_every_n': 1,
    }
    parameters = []
    # Humble's Python Node has get_parameters_by_prefix(), but unlike newer
    # rclpy releases it does not expose list_parameters() on the Node object.
    for name in source.get_parameters_by_prefix('').keys():
        if name.startswith('shared_'):
            continue
        value = replacements.get(name, source.get_parameter(name).value)
        parameters.append(Parameter(name, value=value))
    return parameters


def main(args=None):
    rclpy.init(args=args)
    provider = SharedYoloModelProvider()
    cam0 = SharedCameraYoloNode(
        node_name='shared_yolo_bev_map_node',
        shared_model_provider=provider,
        declare_peer_parameters=True)
    cam2 = SharedCameraYoloNode(
        node_name='yolo_bev_map_node_cam2',
        parameter_overrides=_camera2_overrides(cam0),
        shared_model_provider=provider,
        use_global_arguments=False)
    rate_hz = cam0.get_parameter('shared_inference_rate_hz').value
    cam0.start_shared_scheduler((cam0, cam2), rate_hz)

    executor = SingleThreadedExecutor()
    executor.add_node(cam0)
    executor.add_node(cam2)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.remove_node(cam2)
        executor.remove_node(cam0)
        cam2.destroy_node()
        cam0.destroy_node()
        executor.shutdown()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
