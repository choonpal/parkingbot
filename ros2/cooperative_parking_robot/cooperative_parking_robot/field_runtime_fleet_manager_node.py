#!/usr/bin/env python3
"""Runtime refinements for the measured field Fleet Manager.

This final adapter keeps the policy in ``field_fleet_manager_node`` and adds
pure, testable boundary clamping plus an insertion-corridor check that follows
the actual mecanum controller: lateral centreline correction first, then axial
insertion.
"""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot.field_fleet_manager_node import (
    FieldFleetManagerNode,
)
from cooperative_parking_robot.field_geometry_policy import (
    clamp_rotation_center,
    final_approach_polyline,
)
from cooperative_parking_robot.field_map_geometry import (
    check_oriented_box_inside_map,
)
from cooperative_parking_robot.parking_geometry import Pose2D


class FieldRuntimeFleetManagerNode(FieldFleetManagerNode):
    def _rotation_safe_pose(self, pose, label):
        if self.grid is None or self.grid_w <= 0 or self.grid_h <= 0:
            raise ValueError("OccupancyGrid is required for rotation staging")
        radius = self._rotation_radius()
        result = clamp_rotation_center(
            pose.x_m,
            pose.y_m,
            self.grid_w * self.resolution,
            self.grid_h * self.resolution,
            radius,
            self.rotation_boundary_margin,
            self.max_rotation_stage_shift,
        )
        if result.shift_m > 1e-6:
            self.get_logger().warn(
                f"{label} rotation stage moved "
                f"({pose.x_m:.3f},{pose.y_m:.3f}) -> "
                f"({result.x_m:.3f},{result.y_m:.3f}), "
                f"inset={result.inset_m:.3f}m, "
                f"shift={result.shift_m:.3f}m")
        return Pose2D(result.x_m, result.y_m, pose.yaw_rad)

    def _insertion_corridor_free(self, start, goal):
        """Check the same L-shaped final approach used by RigidBodySync."""

        points = [
            (float(start.x_m), float(start.y_m)),
            *final_approach_polyline(
                (start.x_m, start.y_m),
                (goal.x_m, goal.y_m),
                goal.yaw_rad,
            ),
        ]
        for segment_start, segment_goal in zip(points, points[1:]):
            dx = segment_goal[0] - segment_start[0]
            dy = segment_goal[1] - segment_start[1]
            distance = math.hypot(dx, dy)
            sample_count = max(1, int(math.ceil(
                distance / max(self.resolution * 0.5, 1e-3))))
            for index in range(sample_count + 1):
                ratio = index / sample_count
                x_m = segment_start[0] + ratio * dx
                y_m = segment_start[1] + ratio * dy
                if not self._oriented_footprint_free(
                        x_m, y_m, goal.yaw_rad):
                    return False
        return True

    def _sequential_approach_preflight(self, target, vehicle_spec):
        """Reject approval when either robot odometry is missing or stale."""

        if self.current_virtual_start() is None:
            self.get_logger().error(
                "side-by-side approach preflight: fresh Front/Rear odom missing")
            return False
        return super()._sequential_approach_preflight(target, vehicle_spec)

    def _rollback_park_approval(self, payload, reason):
        self.ui_park_approved = False
        self.requested_destination_slot_id = ""
        self.active_vehicle_number = ""
        self.active_parking_credential = None
        self._set_request_status(payload, "REJECTED", reason)

    def _target_loaded_pose_inside_map(self, target):
        if (target is None or self.grid_w <= 0 or self.grid_h <= 0 or
                self.grid is None):
            return False
        fit = check_oriented_box_inside_map(
            target.x_m,
            target.y_m,
            target.yaw_rad,
            self.loaded_footprint.length_m,
            self.loaded_footprint.width_m,
            self.grid_w * self.resolution,
            self.grid_h * self.resolution,
            # Match AStar's ceil-to-cell boundary strip conservatively.
            boundary_margin_m=self.resolution,
        )
        if not fit.fits:
            self.get_logger().error(
                "waiting target cannot hold loaded footprint: "
                f"{fit.reason} | clearances L/R/B/T="
                f"{fit.left_clearance_m:+.3f}/"
                f"{fit.right_clearance_m:+.3f}/"
                f"{fit.bottom_clearance_m:+.3f}/"
                f"{fit.top_clearance_m:+.3f}m")
        return fit.fits

    def _handle_park_request(self, payload):
        """Approve only when start geometry and a destination both fit."""

        super()._handle_park_request(payload)
        if (self.request_status is None or
                self.request_status.get("status") != "ACCEPTED"):
            return

        target = self._pose_from_stamped(self.target_pose)
        if not self._target_loaded_pose_inside_map(target):
            self._rollback_park_approval(
                payload, "APPROACH_CORRIDOR_BLOCKED")
            return

        candidates = list(self._eligible_park_slots())
        if candidates and any(
                self._field_slot_fit(slot).fits for slot in candidates):
            return

        self._rollback_park_approval(
            payload, "DESTINATION_SLOT_UNAVAILABLE")
        self.get_logger().error(
            "park request rolled back: no vehicle-fit/back-clearance slot")


def main(args=None):
    rclpy.init(args=args)
    node = FieldRuntimeFleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
