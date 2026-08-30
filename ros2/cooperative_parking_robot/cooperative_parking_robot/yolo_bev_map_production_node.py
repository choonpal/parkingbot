#!/usr/bin/env python3
"""Production YOLO/BEV wrapper with measured geometry and mask-centred pose."""

from __future__ import annotations

import rclpy

from cooperative_parking_robot.mvp_integration_nodes import (
    OriginAwareYoloBevMapNode as BaselineYoloBevMapNode,
)
from cooperative_parking_robot.gpu_inference_guard import (
    GPU_INFERENCE_GUARD,
    release_unused_cuda_cache,
)
from cooperative_parking_robot.site_geometry import CAMERA_GEOMETRY
from cooperative_parking_robot.vehicle_mask_center import (
    recenter_vehicle_result_boxes,
)


class _MaskCenteredYoloModel:
    """Delegate to Ultralytics and expose preview-compatible vehicle centres.

    The inherited node calculates world position from the centre of ``xyxy``.
    For Seg detections we therefore re-centre only the vehicle ``xyxy`` on the
    same ``cv2.minAreaRect(mask)`` centre used by ``camera_preview``. Mask,
    class and confidence data are not modified. If a valid mask is missing,
    the original bounding-box centre remains the fallback.
    """

    def __init__(self, delegate, vehicle_class_id=None):
        self._delegate = delegate
        self._vehicle_class_id = (
            None if vehicle_class_id is None else int(vehicle_class_id))

    def __getattr__(self, name):
        return getattr(self._delegate, name)

    def __call__(self, frame, *args, **kwargs):
        # Two production camera nodes are separate processes. Serializing the
        # CUDA section prevents simultaneous CUBLAS handle/workspace peaks.
        # Move finite result tensors to CPU before releasing the lock so the
        # other camera does not overlap inference with retained GPU results.
        with GPU_INFERENCE_GUARD.hold():
            try:
                raw_results = self._delegate(frame, *args, **kwargs)
                results = [
                    result.cpu() if hasattr(result, 'cpu') else result
                    for result in raw_results
                ]
                # Drop the GPU Results container before empty_cache() and
                # before another camera is allowed to enter the CUDA section.
                del raw_results
                if self._vehicle_class_id is not None:
                    height, width = frame.shape[:2]
                    recenter_vehicle_result_boxes(
                        results,
                        frame_width=width,
                        frame_height=height,
                        vehicle_class_id=self._vehicle_class_id,
                    )
            finally:
                release_unused_cuda_cache()
        return results


class YoloBevMapNode(BaselineYoloBevMapNode):
    """Apply measured camera geometry and Seg-mask centre position policy."""

    def __init__(self, **kwargs):
        super().__init__(**kwargs)
        geometry = CAMERA_GEOMETRY.get(self.camera_id)
        configured = (
            self.camera_height > 0.0 and
            any(abs(value) > 1.0e-9 for value in self.camera_ground)
        )
        if configured:
            self.get_logger().info(
                f'[{self.camera_id}] configured optical geometry active | '
                f'ground=({self.camera_ground[0]:.3f},'
                f'{self.camera_ground[1]:.3f})m | '
                f'height={self.camera_height:.3f}m')
        elif geometry is not None:
            self.camera_ground = geometry.optical_axis_ground_m
            self.camera_height = geometry.optical_center_height_m
            self.get_logger().warn(
                f'[{self.camera_id}] site optical geometry was not configured; '
                'using repository fallback | '
                f'ground=({self.camera_ground[0]:.3f},'
                f'{self.camera_ground[1]:.3f})m | '
                f'height={self.camera_height:.3f}m')

        # _load_models() also owns runtime reloads, so it installs the wrapper.
        if (self.model is not None and
                self.model_mode in ('vehicle_seg', 'parking_seg')):
            self.get_logger().info(
                f'[{self.camera_id}] vehicle centre policy: '
                'segmentation minAreaRect centre (bbox fallback)')

        self.get_logger().info(
            f'[{self.camera_id}] effective geometry | '
            f'ground=({self.camera_ground[0]:.3f},'
            f'{self.camera_ground[1]:.3f})m | '
            f'camera_height={self.camera_height:.3f}m | '
            f'vehicle_detection_height_m={self.vehicle_detection_height:.3f}')

    def _load_models(self):
        """Bound concurrent CUDA context/CUBLAS initialization across cameras."""
        with GPU_INFERENCE_GUARD.hold():
            try:
                super()._load_models()
            finally:
                release_unused_cuda_cache()
        # Restore the production mask-centre policy after both cold start and
        # a mission-snapshot unload/reload.
        if self.model is not None:
            vehicle_class_id = (
                self.cls_vehicle
                if self.model_mode in ('vehicle_seg', 'parking_seg')
                else None)
            self.model = _MaskCenteredYoloModel(
                self.model, vehicle_class_id)


def main(args=None):
    rclpy.init(args=args)
    node = YoloBevMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
