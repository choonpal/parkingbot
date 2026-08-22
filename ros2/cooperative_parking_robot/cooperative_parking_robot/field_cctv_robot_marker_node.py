#!/usr/bin/env python3
"""Apply the physical-map origin offset to overhead robot-marker poses."""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot import cctv_robot_marker_node as base_marker


class FieldCctvRobotMarkerNode(base_marker.CctvRobotMarkerNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter("field_map_offset_x_m", 0.0)
        self.declare_parameter("field_map_offset_y_m", 0.80)
        offset_x = float(self.get_parameter("field_map_offset_x_m").value)
        offset_y = float(self.get_parameter("field_map_offset_y_m").value)
        if not all(math.isfinite(value) for value in (offset_x, offset_y)):
            raise ValueError("field map offsets must be finite")

        translation = base_marker.np.array([
            [1.0, 0.0, offset_x],
            [0.0, 1.0, offset_y],
            [0.0, 0.0, 1.0],
        ], dtype=float)
        for camera in self.cameras:
            camera["homography"] = translation @ camera["homography"]

        self.get_logger().info(
            "field overhead-marker Homography offset applied to "
            f"{len(self.cameras)} camera(s): "
            f"dx={offset_x:+.3f}m, dy={offset_y:+.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = FieldCctvRobotMarkerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
