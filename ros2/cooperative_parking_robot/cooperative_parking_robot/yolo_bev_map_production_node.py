#!/usr/bin/env python3
"""Production YOLO/BEV wrapper with measured geometry and mask-centred pose."""

from __future__ import annotations

import rclpy

from cooperative_parking_robot.mvp_integration_nodes import (
    OriginAwareYoloBevMapNode as BaselineYoloBevMapNode,
)
from cooperative_parking_robot.site_geometry import (
    CAMERA_GEOMETRY,
    VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M,
)
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

    def __init__(self, delegate, vehicle_class_id: int):
        self._delegate = delegate
        self._vehicle_class_id = int(vehicle_class_id)

    def __getattr__(self, name):
        return getattr(self._delegate, name)

    def __call__(self, frame, *args, **kwargs):
        results = self._delegate(frame, *args, **kwargs)
        height, width = frame.shape[:2]
        recenter_vehicle_result_boxes(
            results,
            frame_width=width,
            frame_height=height,
            vehicle_class_id=self._vehicle_class_id,
        )
        return results


class YoloBevMapNode(BaselineYoloBevMapNode):
    """Apply measured camera geometry and Seg-mask centre position policy."""

    def __init__(self):
        super().__init__()
        geometry = CAMERA_GEOMETRY.get(self.camera_id)
        if geometry is not None:
            self.camera_ground = geometry.optical_axis_ground_m
            self.camera_height = geometry.optical_center_height_m
            self.get_logger().info(
                f'[{self.camera_id}] measured optical geometry active | '
                f'ground=({self.camera_ground[0]:.3f},'
                f'{self.camera_ground[1]:.3f})m | '
                f'height={self.camera_height:.3f}m')

        # Keep runtime and camera_preview on exactly the same centre definition:
        # Seg mask minAreaRect centre -> Homography -> optional parallax.
        if (self.model is not None and
                self.model_mode in ('vehicle_seg', 'parking_seg')):
            self.model = _MaskCenteredYoloModel(self.model, self.cls_vehicle)
            self.get_logger().info(
                f'[{self.camera_id}] vehicle centre policy: '
                'segmentation minAreaRect centre (bbox fallback)')

        if VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M is None:
            self.vehicle_detection_height = 0.0
            self.get_logger().warn(
                'vehicle top is sloped; 0.74m is maximum only. '
                'Vehicle parallax remains disabled until effective '
                'segmentation height is measured.')
        else:
            self.vehicle_detection_height = float(
                VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M)


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
