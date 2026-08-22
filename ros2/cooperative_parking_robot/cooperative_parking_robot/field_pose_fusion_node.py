#!/usr/bin/env python3
"""Pose-fusion adapter for the field HOME orientation.

Both robots are parked side-by-side at HOME and face map -X (180 degrees).
The first valid CCTV marker observation remains authoritative and replaces this
initial value, but setting the nominal yaw correctly prevents a wrong 0-degree
encoder-only pose during startup or a short marker-acquisition delay.
"""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot import pose_fusion_node as base_pose


class FieldPoseFusionNode(base_pose.PoseFusionNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter("field_initial_yaw_deg", 180.0)
        initial_yaw_deg = float(
            self.get_parameter("field_initial_yaw_deg").value)
        if not math.isfinite(initial_yaw_deg):
            raise ValueError("field_initial_yaw_deg must be finite")
        self.ekf.yaw = math.atan2(
            math.sin(math.radians(initial_yaw_deg)),
            math.cos(math.radians(initial_yaw_deg)),
        )
        self._last_source = "FIELD_HOME_INIT"
        self.get_logger().info(
            f"field HOME nominal yaw={initial_yaw_deg:.1f}deg; "
            "first fresh CCTV fix remains authoritative")


def main(args=None):
    rclpy.init(args=args)
    node = FieldPoseFusionNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
