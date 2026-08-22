#!/usr/bin/env python3
"""Individual motion adapter for side-by-side HOME poses.

The base state machine remains Front-first.  This adapter changes only the
route creation from each HOME pose to the existing longitudinal staging pose:
it plans around both the vehicle protected envelope and the stationary peer
robot.  Thus the robots may start parallel and side-by-side behind the vehicle
without requiring a fake in-line HOME arrangement.
"""

from __future__ import annotations

import math
import time

import rclpy
from nav_msgs.msg import Odometry
from rclpy.qos import qos_profile_sensor_data

from cooperative_parking_robot import individual_move_node as base_move
from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    plan_route_around_rectangles,
    projected_half_extents,
)
from cooperative_parking_robot.vehicle_entry import (
    approach_longitudinal,
    vehicle_to_world,
    world_to_vehicle,
)


class FieldIndividualMoveNode(base_move.IndividualMoveNode):
    def __init__(self):
        super().__init__()

        self.declare_parameter("approach_corner_margin_m", 0.03)
        self.declare_parameter("require_peer_odom_for_approach", True)

        self.approach_corner_margin = float(
            self.get_parameter("approach_corner_margin_m").value)
        self.require_peer_odom = bool(
            self.get_parameter("require_peer_odom_for_approach").value)
        if not math.isfinite(self.approach_corner_margin) or \
                self.approach_corner_margin <= 0.0:
            raise ValueError("approach_corner_margin_m must be positive")

        self.peer_odom = None
        self.peer_odom_receipt = None
        self.create_subscription(
            Odometry,
            f"/{self.other_role}/odom",
            self.peer_odom_cb,
            qos_profile_sensor_data,
        )
        self.get_logger().info(
            f"[{self.role}] side-by-side HOME routing enabled | "
            f"peer_odom_required={self.require_peer_odom} | "
            f"corner_margin={self.approach_corner_margin:.3f}m")

    def peer_odom_cb(self, msg):
        if msg.header.frame_id not in ("", "map"):
            return
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        yaw = math.atan2(
            2.0 * (q.w * q.z + q.x * q.y),
            1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        values = (float(p.x), float(p.y), float(yaw))
        if not all(math.isfinite(value) for value in values):
            return
        self.peer_odom = values
        self.peer_odom_receipt = time.monotonic()

    def peer_odom_is_fresh(self):
        return (
            self.peer_odom is not None and
            self.peer_odom_receipt is not None and
            0.0 <= time.monotonic() - self.peer_odom_receipt <
            self.odom_timeout
        )

    def latch_target_and_plan(self):
        if self.latest_target is None:
            if self.phase_timed_out():
                return False
            self.stop()
            return False
        if time.monotonic() - self.latest_target_time > self.target_timeout:
            self.fault("TARGET_POSE_STALE")
            return False
        if self.require_peer_odom and not self.peer_odom_is_fresh():
            self.stop()
            if self.phase_timed_out():
                return False
            self.get_logger().warn(
                f"[{self.role}] waiting for fresh {self.other_role} odom "
                "before side-by-side approach",
                throttle_duration_sec=2.0)
            return False

        self.active_target = self.latest_target
        tx, ty, yaw = self.active_target
        start = world_to_vehicle(self.x, self.y, tx, ty, yaw)
        goal = (
            approach_longitudinal(
                self.role, self.entry_standoff, self.wheelbase),
            0.0,
        )

        # The moving robot rotates toward the vehicle yaw while translating.
        # Use the larger current/aligned projection for a conservative centre
        # route around the vehicle and the stationary peer.
        current_half_s, current_half_d = projected_half_extents(
            self.robot_length, self.robot_width, self.theta - yaw, 0.0)
        aligned_half_s, aligned_half_d = projected_half_extents(
            self.robot_length, self.robot_width, 0.0, 0.0)
        moving_half_s = max(current_half_s, aligned_half_s)
        moving_half_d = max(current_half_d, aligned_half_d)

        protected_s = (
            self.vehicle_half_length + moving_half_s +
            self.robot_clearance)
        protected_d = (
            self.vehicle_half_width + moving_half_d +
            self.robot_clearance)
        rectangles = [
            AxisAlignedRect(0.0, 0.0, protected_s, protected_d)
        ]

        if self.peer_odom_is_fresh():
            peer_x, peer_y, peer_yaw = self.peer_odom
            peer_s, peer_d = world_to_vehicle(
                peer_x, peer_y, tx, ty, yaw)
            peer_half_s, peer_half_d = projected_half_extents(
                self.robot_length, self.robot_width, peer_yaw - yaw, 0.0)
            # Route nodes represent the moving robot centre. Inflate the fixed
            # peer by both bodies plus the requested body-to-body gap.
            rectangles.append(AxisAlignedRect(
                peer_s,
                peer_d,
                peer_half_s + moving_half_s +
                self.minimum_inter_robot_gap,
                peer_half_d + moving_half_d +
                self.minimum_inter_robot_gap,
            ))

        try:
            route_sd = plan_route_around_rectangles(
                start,
                goal,
                rectangles,
                corner_margin_m=self.approach_corner_margin,
            )
        except ValueError as exc:
            self.fault(f"APPROACH_ROUTE_INVALID:{exc}")
            return False

        self.route = [
            vehicle_to_world(s, d, tx, ty, yaw)
            for s, d in route_sd
        ]
        self.set_phase("TO_REAR_STAGING")
        self.get_logger().info(
            f"[{self.role}] side-by-side HOME route: "
            f"start=({start[0]:+.3f},{start[1]:+.3f}) -> "
            f"goal=({goal[0]:+.3f},{goal[1]:+.3f}), "
            f"waypoints={len(self.route)}")
        return True


def main(args=None):
    rclpy.init(args=args)
    node = FieldIndividualMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
