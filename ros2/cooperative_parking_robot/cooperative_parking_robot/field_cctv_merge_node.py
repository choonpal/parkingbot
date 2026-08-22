#!/usr/bin/env python3
"""Field adapter for directed vehicle yaw in the dual-CCTV merge node.

Vehicle segmentation/PCA returns an undirected longitudinal axis.  The field
WAITING pose is directed at 180 degrees, and during transport the robot-pair
odometry supplies the closest directed heading.  This adapter prevents the
same physical vehicle from alternating between yaw and yaw+pi.
"""

from __future__ import annotations

import math

import rclpy
from geometry_msgs.msg import PoseStamped

from cooperative_parking_robot import cctv_merge_node as base_merge
from cooperative_parking_robot.field_heading_geometry import (
    circular_mean,
    normalize_angle,
    resolve_undirected_axis_yaw,
)


class FieldCctvMergeNode(base_merge.CctvMergeNode):
    def __init__(self):
        super().__init__()
        self.declare_parameter("waiting_yaw_deg", 180.0)
        waiting_yaw_deg = float(
            self.get_parameter("waiting_yaw_deg").value)
        if not math.isfinite(waiting_yaw_deg):
            raise ValueError("waiting_yaw_deg must be finite")
        self.waiting_yaw = normalize_angle(
            math.radians(waiting_yaw_deg))
        self.last_resolved_vehicle_yaw = self.waiting_yaw
        self.get_logger().info(
            "field directed-yaw merge | "
            f"WAITING reference={waiting_yaw_deg:.1f}deg")

    def odom_cb(self, role, msg):
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        values = (
            float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in values):
            return
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            return
        qx, qy, qz, qw = (value / norm for value in values)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        x = float(p.x)
        y = float(p.y)
        if not all(math.isfinite(value) for value in (x, y, yaw)):
            return
        # Base map/self-mask logic only indexes [0]/[1], so retaining yaw as
        # the third tuple item is backward compatible.
        self.robot_pose[role] = (x, y, normalize_angle(yaw))

    def _robot_reference_yaw(self):
        front = self.robot_pose.get("front")
        rear = self.robot_pose.get("rear")
        front_yaw = (
            front[2] if front is not None and len(front) >= 3 else None)
        rear_yaw = (
            rear[2] if rear is not None and len(rear) >= 3 else None)
        if front_yaw is not None and rear_yaw is not None:
            try:
                return circular_mean(front_yaw, rear_yaw)
            except ValueError:
                return self.last_resolved_vehicle_yaw
        if front_yaw is not None:
            return front_yaw
        if rear_yaw is not None:
            return rear_yaw
        return self.last_resolved_vehicle_yaw

    def _publish_target(self, center):
        measured_axis = (
            self.dimension_tracker.yaw
            if self.dimension_tracker.yaw_valid else self.waiting_yaw)
        directed_yaw = resolve_undirected_axis_yaw(
            measured_axis, self.waiting_yaw)
        self.last_resolved_vehicle_yaw = directed_yaw

        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = float(center[0])
        msg.pose.position.y = float(center[1])
        msg.pose.orientation.z = math.sin(directed_yaw / 2.0)
        msg.pose.orientation.w = math.cos(directed_yaw / 2.0)
        self.pub_target.publish(msg)

    def _publish_vehicle_feedback(self, cars, stamp_ns):
        front = self.robot_pose.get("front")
        rear = self.robot_pose.get("rear")
        if not cars or front is None or rear is None:
            return
        predicted = (
            (front[0] + rear[0]) / 2.0,
            (front[1] + rear[1]) / 2.0,
        )
        candidate = min(cars, key=lambda car: math.hypot(
            car.center[0] - predicted[0],
            car.center[1] - predicted[1]))
        distance = math.hypot(
            candidate.center[0] - predicted[0],
            candidate.center[1] - predicted[1])
        if distance > self.feedback_gate:
            self.get_logger().warn(
                f"운반 차량 feedback association gate 초과: {distance:.3f}m",
                throttle_duration_sec=2.0)
            return

        reference_yaw = self._robot_reference_yaw()
        if candidate.yaw is None:
            directed_yaw = reference_yaw
        else:
            directed_yaw = resolve_undirected_axis_yaw(
                candidate.yaw, reference_yaw)
        self.last_resolved_vehicle_yaw = directed_yaw

        msg = PoseStamped()
        if stamp_ns > 0:
            msg.header.stamp.sec = int(stamp_ns // 1_000_000_000)
            msg.header.stamp.nanosec = int(stamp_ns % 1_000_000_000)
        else:
            msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "map"
        msg.pose.position.x = candidate.center[0]
        msg.pose.position.y = candidate.center[1]
        msg.pose.orientation.z = math.sin(directed_yaw / 2.0)
        msg.pose.orientation.w = math.cos(directed_yaw / 2.0)
        self.pub_vehicle_fb.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = FieldCctvMergeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
