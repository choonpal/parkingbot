#!/usr/bin/env python3
"""Runtime return policy for the side-by-side field HOME layout.

The 1.20 m vehicle-only slot has only 0.23 m behind its closed end.  The legacy
split exit would move the Front robot farther into that end and is therefore
not valid.  This adapter requires both robots to clear toward the aisle in the
same direction, then returns Front first and Rear second to the adjacent HOME
poses.  The final HOME waypoint also restores the measured 180-degree yaw.
"""

from __future__ import annotations

import math
import time

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    plan_route_around_rectangles,
    projected_half_extents,
)
from cooperative_parking_robot.field_individual_move_node import (
    FieldIndividualMoveNode,
)
from cooperative_parking_robot.vehicle_entry import (
    vehicle_to_world,
    world_to_vehicle,
)


class FieldRuntimeIndividualMoveNode(FieldIndividualMoveNode):
    def __init__(self):
        super().__init__()

        self.declare_parameter("field_home_yaw_deg", 180.0)
        self.declare_parameter("sequential_home_return", True)
        self.field_home_yaw = math.radians(float(
            self.get_parameter("field_home_yaw_deg").value))
        self.field_home_yaw = math.atan2(
            math.sin(self.field_home_yaw),
            math.cos(self.field_home_yaw),
        )
        self.sequential_home_return = bool(
            self.get_parameter("sequential_home_return").value)

        if not self.same_direction_exit:
            raise ValueError(
                "field vehicle-only slots require same_direction_exit=true")
        if self.same_direction_exit_sign != -1:
            raise ValueError(
                "field slots require same_direction_exit_sign=-1 "
                "to clear toward the aisle")
        if not self.sequential_home_return:
            raise ValueError(
                "side-by-side HOME requires sequential_home_return=true")

        self.get_logger().info(
            f"[{self.role}] field return policy | shared aisle exit | "
            f"home_yaw={math.degrees(self.field_home_yaw):.1f}deg | "
            "Front-first HOME return")

    def _plan_field_return_home(self):
        if self.slot_target is None:
            self.fault("RETURN_SLOT_POSE_MISSING")
            return
        if self.require_peer_odom and not self.peer_odom_is_fresh():
            self.stop()
            self.set_phase("WAIT_RETURN_PEER_ODOM")
            return

        tx, ty, slot_yaw = self.slot_target
        start = world_to_vehicle(
            self.x, self.y, tx, ty, slot_yaw)
        goal = world_to_vehicle(
            self.wait_pos[0], self.wait_pos[1], tx, ty, slot_yaw)

        current_half_s, current_half_d = projected_half_extents(
            self.robot_length,
            self.robot_width,
            self.theta - slot_yaw,
            0.0,
        )
        home_half_s, home_half_d = projected_half_extents(
            self.robot_length,
            self.robot_width,
            self.field_home_yaw - slot_yaw,
            0.0,
        )
        moving_half_s = max(current_half_s, home_half_s)
        moving_half_d = max(current_half_d, home_half_d)

        rectangles = [AxisAlignedRect(
            0.0,
            0.0,
            self.vehicle_half_length + moving_half_s + self.robot_clearance,
            self.vehicle_half_width + moving_half_d + self.robot_clearance,
        )]

        if self.peer_odom_is_fresh():
            peer_x, peer_y, peer_yaw = self.peer_odom
            peer_s, peer_d = world_to_vehicle(
                peer_x, peer_y, tx, ty, slot_yaw)
            peer_half_s, peer_half_d = projected_half_extents(
                self.robot_length,
                self.robot_width,
                peer_yaw - slot_yaw,
                0.0,
            )
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
            self.fault(f"FIELD_RETURN_ROUTE_INVALID:{exc}")
            return

        self.route = [
            vehicle_to_world(s, d, tx, ty, slot_yaw)
            for s, d in route_sd
        ]
        self.set_phase("RETURN_HOME")
        self.get_logger().info(
            f"[{self.role}] field HOME route: "
            f"waypoints={len(self.route)}, "
            f"goal=({self.wait_pos[0]:.3f},{self.wait_pos[1]:.3f},"
            f"{math.degrees(self.field_home_yaw):.1f}deg)")

    def plan_return_home(self):
        if (self.sequential_home_return and not self.is_front and
                not self.peer_reached_phase("RETURNED")):
            self.stop()
            self.set_phase("WAIT_FRONT_HOME")
            return
        self._plan_field_return_home()

    def _complete_field_return(self):
        self.stop()
        self.set_phase("RETURNED")
        if not self.return_sent:
            self.pub_return_done.publish(Bool(data=True))
            self.return_sent = True

    def _run_field_return_home(self):
        if self.phase_timed_out():
            return
        if not self.route:
            self._complete_field_return()
            return

        gx, gy = self.route[0]
        final_waypoint = len(self.route) == 1
        arrived = self.move_pose_toward(
            gx,
            gy,
            self.field_home_yaw if final_waypoint else None,
            self.max_speed,
            self.position_tolerance,
        )
        if arrived:
            self.route.pop(0)
            self.phase_enter_time = time.monotonic()
        if not self.route:
            self._complete_field_return()

    def run_return(self):
        if self.phase == "WAIT_FRONT_HOME":
            self.stop()
            if self.phase_timed_out():
                return
            if self.peer_reached_phase("RETURNED"):
                self._plan_field_return_home()
            return

        if self.phase == "WAIT_RETURN_PEER_ODOM":
            self.stop()
            if self.phase_timed_out():
                return
            peer_ready = self.peer_odom_is_fresh()
            turn_ready = (
                self.is_front or
                not self.sequential_home_return or
                self.peer_reached_phase("RETURNED")
            )
            if peer_ready and turn_ready:
                self._plan_field_return_home()
            return

        if self.phase == "RETURN_HOME":
            self._run_field_return_home()
            return

        super().run_return()


def main(args=None):
    rclpy.init(args=args)
    node = FieldRuntimeIndividualMoveNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
