#!/usr/bin/env python3
"""Apply the physical-map origin offset to each CCTV Homography.

The calibration GUI's common Tile(0,0) line is 0.80 m above the measured lower
map boundary.  Stored H files intentionally remain unchanged; this adapter
left-multiplies the pixel->tile-map matrix by a metric translation before any
vehicle, mask or coverage projection is produced.
"""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot import yolo_bev_map_node as base_yolo


class FieldYoloBevMapNode(base_yolo.YoloBevMapNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter("field_map_offset_x_m", 0.0)
        self.declare_parameter("field_map_offset_y_m", 0.80)
        offset_x = float(self.get_parameter("field_map_offset_x_m").value)
        offset_y = float(self.get_parameter("field_map_offset_y_m").value)
        if not all(math.isfinite(value) for value in (offset_x, offset_y)):
            raise ValueError("field map offsets must be finite")

        if self.H is not None:
            translation = base_yolo.np.array([
                [1.0, 0.0, offset_x],
                [0.0, 1.0, offset_y],
                [0.0, 0.0, 1.0],
            ], dtype=float)
            self.H = translation @ self.H
            # Coverage and optical-axis fallbacks must be rebuilt from the
            # translated H on the first image.
            self._coverage_size = None
            self._coverage_polygon = None
            self._axis_reference = None

        self.get_logger().info(
            f"[{self.camera_id}] field Homography offset applied: "
            f"dx={offset_x:+.3f}m, dy={offset_y:+.3f}m")


def main(args=None):
    rclpy.init(args=args)
    node = FieldYoloBevMapNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
