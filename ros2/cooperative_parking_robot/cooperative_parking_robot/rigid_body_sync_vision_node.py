#!/usr/bin/env python3
"""Rear-ID0-only production wrapper for rigid-body synchronization.

CCTV remains available to the global pose/path pipeline.  It is deliberately
not subscribed to or consumed here as a Front/Rear relative x/y/yaw source.
"""

from __future__ import annotations

import rclpy

from cooperative_parking_robot.rigid_body_sync_production_node import (
    RigidBodySyncNode as P0RigidBodySyncNode,
)


class RigidBodySyncNode(P0RigidBodySyncNode):
    """Keep relative fusion limited to wheel odometry and Rear-camera ID0."""

    def __init__(self):
        super().__init__()
        self.get_logger().info(
            'relative visual source policy active | Rear ID0 only | '
            'CCTV top-marker fallback disabled')

    def _new_cctv_pair(self, now):
        """Never inject overhead marker poses into the relative Kalman state."""
        self._last_visual_reason = 'CCTV_RELATIVE_DISABLED'
        return None


def main(args=None):
    rclpy.init(args=args)
    node = RigidBodySyncNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    if rclpy.ok():
        rclpy.shutdown()


if __name__ == '__main__':
    main()
