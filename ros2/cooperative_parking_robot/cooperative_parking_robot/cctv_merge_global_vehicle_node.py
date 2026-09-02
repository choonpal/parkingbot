#!/usr/bin/env python3
"""Origin-aware CCTV merge with continuous carried-vehicle feedback.

Baseline merge publishes `/parking/vehicle_pose_feedback` only after the target
leaves the waiting polygon.  A lifted vehicle can already hide Rear ID0 while
its centre is still inside that polygon, so this wrapper publishes one ordered
feedback attempt per new merged camera frame throughout Lift.
"""

from __future__ import annotations

import time

import rclpy

from cooperative_parking_robot.mvp_integration_nodes import (
    OriginAwareCctvMergeNode,
)


class CctvMergeNode(OriginAwareCctvMergeNode):
    def __init__(self):
        self._last_vehicle_feedback_stamp_ns = 0
        super().__init__()
        self.get_logger().info(
            'lifted vehicle feedback active inside and outside waiting ROI')

    def _publish_vehicle_feedback(self, cars, stamp_ns):
        stamp = int(stamp_ns)
        if stamp <= 0 or stamp <= self._last_vehicle_feedback_stamp_ns:
            return
        super()._publish_vehicle_feedback(cars, stamp)
        # Consume each source camera timestamp only once; a new image receives
        # a new source stamp even though the merge timer is faster.
        self._last_vehicle_feedback_stamp_ns = stamp

    def _publish_map(self, merged, latched, coverage_polygons):
        if self.vehicle_lifted and merged:
            now = time.monotonic()
            stamps = [
                int(envelope['stamp_ns'])
                for camera_id, envelope in self.latest.items()
                if (envelope is not None and
                    now - self.latest_wall.get(camera_id, 0.0) <=
                    self.camera_timeout_s and
                    int(envelope.get('stamp_ns', 0)) > 0)
            ]
            if stamps:
                # The inherited association gate selects the detection nearest
                # the current Front/Rear midpoint.  Include the waiting-ROI
                # detection because it may already be physically lifted.
                self._publish_vehicle_feedback(merged, max(stamps))
        return super()._publish_map(merged, latched, coverage_polygons)


def main(args=None):
    rclpy.init(args=args)
    node = CctvMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
