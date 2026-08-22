#!/usr/bin/env python3
"""Runtime return policy for the side-by-side field HOME layout.

The 1.20 m vehicle-only slot has only 0.23 m behind its closed end.  The legacy
split exit would move the Front robot farther into that end and is therefore
not valid.  Both robots first clear toward the aisle while keeping wheelbase
separation.  Rear then returns HOME first, Front returns second, and each robot
rotates to the measured 180-degree HOME yaw at an interior staging point before
entering the tight side-by-side HOME pair.
"""

from __future__ import annotations

import math
import time

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    clamp_rotation_center,
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
        self.declare_parameter("field_map_width_m", 4.40)
        self.declare_parameter("field_map_height_m", 3.83)
        self.declare_parameter("home_rotation_boundary_margin_m", 0.02)
        self.declare_parameter("max_home_rotation_stage_shift_m", 0.15)

        self.field_home_yaw = math.radians(float(
            self.get_parameter("field_home_yaw_deg").value))
        self.field_home_yaw = math.atan2(
            math.sin(self.field_home_yaw),
            math.cos(self.field_home_yaw),
        )
        self.sequential_home_return = bool(
            self.get_parameter("sequential_home_return").value)
        self.field_map_width = float(
            self.get_parameter("field_map_width_m").value)
        self.field_map_height = float(
            self.get_parameter("field_map_height_m").value)
        self.home_rotation_boundary_margin = float(
            self.get_parameter("home_rotation_boundary_margin_m").value)
        self.max_home_rotation_stage_shift = float(
            self.get_parameter("max_home_rotation_stage_shift_m").value)

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
        if min(self.field_map_width, self.field_map_height) <= 0.0:
            raise ValueError("field map dimensions must be positive")
        if (self.home_rotation_boundary_margin < 0.0 or
                self.max_home_rotation_stage_shift <= 0.0):
            raise ValueError("invalid HOME rotation-stage limits")

        self.robot_rotation_radius = 0.5 * math.hypot(
            self.robot_length,
            self.robot_width,
        )
        front_clear_s = self.wheelbase / 2.0 + self.exit_distance
        required_clear_s = (
            self.vehicle_half_length + self.robot_rotation_radius +
            self.robot_clearance + self.home_rotation_boundary_margin)
        if front_clear_s <= required_clear_s:
            raise ValueError(
                "exit_distance_m is too short for Front to rotate after "
                f"aisle exit: clear={front_clear_s:.3f}m, "
                f"required>{required_clear_s:.3f}m")

        self.home_rotation_target = None
        self.get_logger().info(
            f"[{self.role}] field return policy | shared aisle exit | "
            f"exit_distance={self.exit_distance:.3f}m | "
            f"home_yaw={math.degrees(self.field_home_yaw):.1f}deg | "
            "Rear-first HOME return")

    def _moving_home_extents(self, slot_yaw):
        return projected_half_extents(
            self.robot_length,
            self.robot_width,
            self.field_home_yaw - slot_yaw,
            0.0,
        )

    def _plan_aligned_return_home(self):
        """Plan with the robot already fixed at the HOME yaw."""

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
        moving_half_s, moving_half_d = self._moving_home_extents(slot_yaw)

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

    def _begin_home_rotation(self):
        if self.require_peer_odom and not self.peer_odom_is_fresh():
            self.stop()
            self.set_phase("WAIT_RETURN_PEER_ODOM")
            return

        result = clamp_rotation_center(
            self.x,
            self.y,
            self.field_map_width,
            self.field_map_height,
            self.robot_rotation_radius,
            self.home_rotation_boundary_margin,
            self.max_home_rotation_stage_shift,
        )
        self.home_rotation_target = (result.x_m, result.y_m)
        if result.shift_m > self.position_tolerance:
            self.get_logger().warn(
                f"[{self.role}] HOME rotation stage shifted "
                f"{result.shift_m:.3f}m to "
                f"({result.x_m:.3f},{result.y_m:.3f})")
            self.set_phase("MOVE_HOME_ROTATION_STAGE")
        else:
            self.set_phase("ALIGN_HOME_YAW")

    def plan_return_home(self):
        # Rear leaves the shared side lane first.  Once Rear is parked, Front
        # can enter the upper HOME while preserving the 0.10m body gap.
        if (self.sequential_home_return and self.is_front and
                not self.peer_reached_phase("RETURNED")):
            self.stop()
            self.set_phase("WAIT_REAR_HOME")
            return
        self._begin_home_rotation()

    def _complete_field_return(self):
        self.stop()
        self.set_phase("RETURNED")
        if not self.return_sent:
            self.pub_return_done.publish(Bool(data=True))
            self.return_sent = True

    def _run_move_home_rotation_stage(self):
        if self.phase_timed_out():
            return
        if self.home_rotation_target is None:
            self.fault("HOME_ROTATION_STAGE_MISSING")
            return
        if self.move_pose_toward(
                self.home_rotation_target[0],
                self.home_rotation_target[1],
                None,
                self.centerline_speed,
                self.position_tolerance):
            self.stop()
            self.set_phase("ALIGN_HOME_YAW")

    def _run_align_home_yaw(self):
        if self.phase_timed_out():
            return
        if self.home_rotation_target is None:
            self.home_rotation_target = (self.x, self.y)
        if self.move_pose_toward(
                self.home_rotation_target[0],
                self.home_rotation_target[1],
                self.field_home_yaw,
                self.centerline_speed,
                self.position_tolerance):
            self.stop()
            self._plan_aligned_return_home()

    def _run_field_return_home(self):
        if self.phase_timed_out():
            return
        if not self.route:
            self._complete_field_return()
            return

        gx, gy = self.route[0]
        arrived = self.move_pose_toward(
            gx,
            gy,
            self.field_home_yaw,
            self.max_speed,
            self.position_tolerance,
        )
        if arrived:
            self.route.pop(0)
            self.phase_enter_time = time.monotonic()
        if not self.route:
            self._complete_field_return()

    def run_return(self):
        if self.phase == "WAIT_REAR_HOME":
            self.stop()
            if self.phase_timed_out():
                return
            if self.peer_reached_phase("RETURNED"):
                self._begin_home_rotation()
            return

        if self.phase == "WAIT_RETURN_PEER_ODOM":
            self.stop()
            if self.phase_timed_out():
                return
            peer_ready = self.peer_odom_is_fresh()
            turn_ready = (
                not self.is_front or
                not self.sequential_home_return or
                self.peer_reached_phase("RETURNED")
            )
            if peer_ready and turn_ready:
                self._begin_home_rotation()
            return

        if self.phase == "MOVE_HOME_ROTATION_STAGE":
            self._run_move_home_rotation_stage()
            return

        if self.phase == "ALIGN_HOME_YAW":
            self._run_align_home_yaw()
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
