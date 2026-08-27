#!/usr/bin/env python3
"""Production YOLO/BEV wrapper with confirmed ceiling-camera geometry."""

from __future__ import annotations

import rclpy

from cooperative_parking_robot.mvp_integration_nodes import (
    OriginAwareYoloBevMapNode as BaselineYoloBevMapNode,
)
from cooperative_parking_robot.site_geometry import (
    CAMERA_GEOMETRY,
    VEHICLE_DETECTION_EFFECTIVE_HEIGHT_M,
)


class YoloBevMapNode(BaselineYoloBevMapNode):
    """Apply measured camera geometry; do not guess the sloped car height."""

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
