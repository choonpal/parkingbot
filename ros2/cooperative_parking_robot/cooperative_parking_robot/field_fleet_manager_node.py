#!/usr/bin/env python3
"""Field-adapted Fleet Manager.

Differences from the base node:
- A physical parking slot is checked against the vehicle body only.
- A measured 0.23 m free strip behind each slot is checked against the loaded
  robot-pair overhang.
- A*, rotation, insertion and final-pose collision checks continue to use the
  complete loaded footprint.
- Front-first approach preflight accepts side-by-side HOME poses and checks
  multi-segment routes around the vehicle and stationary peer.
"""

from __future__ import annotations

import math
import threading

import rclpy

from cooperative_parking_robot import fleet_manager_node as base_fleet
from cooperative_parking_robot.field_geometry_policy import (
    AxisAlignedRect,
    check_loaded_overhang_clearance,
    check_vehicle_only_slot_fit,
    plan_route_around_rectangles,
    projected_half_extents,
)
from cooperative_parking_robot.parking_geometry import FitResult, Pose2D
from cooperative_parking_robot.retrieval_planning import corridor_is_free
from cooperative_parking_robot.vehicle_entry import (
    approach_longitudinal,
    vehicle_to_world,
    world_to_vehicle,
)


class FieldFleetManagerNode(base_fleet.FleetManagerNode):
    _fit_patch_lock = threading.Lock()

    def __init__(self):
        super().__init__()

        self.declare_parameter(
            "vehicle_slot_longitudinal_margin_m", 0.05)
        self.declare_parameter(
            "vehicle_slot_lateral_margin_m", 0.05)
        self.declare_parameter("slot_back_clearance_m", 0.23)
        self.declare_parameter("slot_back_clearance_reserve_m", 0.03)
        self.declare_parameter("approach_robot_clearance_m", 0.06)
        self.declare_parameter("approach_corner_margin_m", 0.03)

        gp = self.get_parameter
        self.vehicle_slot_long_margin = float(
            gp("vehicle_slot_longitudinal_margin_m").value)
        self.vehicle_slot_lat_margin = float(
            gp("vehicle_slot_lateral_margin_m").value)
        self.slot_back_clearance = float(
            gp("slot_back_clearance_m").value)
        self.slot_back_clearance_reserve = float(
            gp("slot_back_clearance_reserve_m").value)
        self.approach_robot_clearance = float(
            gp("approach_robot_clearance_m").value)
        self.approach_corner_margin = float(
            gp("approach_corner_margin_m").value)

        values = (
            self.vehicle_slot_long_margin,
            self.vehicle_slot_lat_margin,
            self.slot_back_clearance,
            self.slot_back_clearance_reserve,
            self.approach_robot_clearance,
        )
        if not all(math.isfinite(value) and value >= 0.0
                   for value in values):
            raise ValueError(
                "field geometry parameters must be finite and non-negative")
        if (not math.isfinite(self.approach_corner_margin) or
                self.approach_corner_margin <= 0.0):
            raise ValueError("approach_corner_margin_m must be positive")

        self.get_logger().info(
            "field slot policy | vehicle-only fit | "
            f"vehicle_margin={self.vehicle_slot_long_margin:.3f}/"
            f"{self.vehicle_slot_lat_margin:.3f}m | "
            f"back_clearance={self.slot_back_clearance:.3f}m | "
            f"reserve={self.slot_back_clearance_reserve:.3f}m")

    def _field_slot_fit(self, slot, *_ignored_args, **_ignored_kwargs):
        """Return a FitResult while preserving the base planner interface."""

        vehicle_fit = check_vehicle_only_slot_fit(
            slot.length_m,
            slot.width_m,
            self.vehicle_length,
            self.vehicle_width,
            self.vehicle_slot_long_margin,
            self.vehicle_slot_lat_margin,
        )
        if not vehicle_fit.fits:
            return FitResult(
                fits=False,
                reason=vehicle_fit.reason,
                required_length_m=vehicle_fit.required_length_m,
                required_width_m=vehicle_fit.required_width_m,
                length_clearance_m=vehicle_fit.length_clearance_m,
                width_clearance_m=vehicle_fit.width_clearance_m,
            )

        overhang = check_loaded_overhang_clearance(
            slot.length_m,
            self.loaded_footprint.length_m,
            self.slot_fit_long_margin,
            self.slot_back_clearance,
            self.slot_back_clearance_reserve,
        )
        if not overhang.fits:
            return FitResult(
                fits=False,
                reason=overhang.reason,
                required_length_m=overhang.effective_loaded_length_m,
                required_width_m=vehicle_fit.required_width_m,
                length_clearance_m=overhang.clearance_m,
                width_clearance_m=vehicle_fit.width_clearance_m,
            )

        self.get_logger().info(
            f"slot {slot.slot_id} field fit OK | "
            f"vehicle clearance L/W="
            f"{vehicle_fit.length_clearance_m:.3f}/"
            f"{vehicle_fit.width_clearance_m:.3f}m | "
            f"loaded back overhang={overhang.overhang_each_end_m:.3f}m | "
            f"back reserve clearance={overhang.clearance_m:.3f}m",
            throttle_duration_sec=2.0,
        )
        return FitResult(
            fits=True,
            reason="OK",
            required_length_m=vehicle_fit.required_length_m,
            required_width_m=vehicle_fit.required_width_m,
            length_clearance_m=vehicle_fit.length_clearance_m,
            width_clearance_m=vehicle_fit.width_clearance_m,
        )

    def plan_and_publish(self):
        """Run the base planner with only its parking-slot fit hook replaced."""

        if self.mission_type == "retrieve":
            return super().plan_and_publish()

        # The base node imported check_slot_fit as a module global.  Keep this
        # adapter isolated and restore the original function even on errors.
        with self._fit_patch_lock:
            original = base_fleet.check_slot_fit
            base_fleet.check_slot_fit = self._field_slot_fit
            try:
                return super().plan_and_publish()
            finally:
                base_fleet.check_slot_fit = original

    @staticmethod
    def _pose_from_stamped(message):
        if message is None or message.header.frame_id not in ("", "map"):
            return None
        p = message.pose.position
        q = message.pose.orientation
        values = (float(q.x), float(q.y), float(q.z), float(q.w))
        if not all(math.isfinite(value) for value in values):
            return None
        norm = math.sqrt(sum(value * value for value in values))
        if norm < 1e-9:
            return None
        qx, qy, qz, qw = (value / norm for value in values)
        yaw = math.atan2(
            2.0 * (qw * qz + qx * qy),
            1.0 - 2.0 * (qy * qy + qz * qz),
        )
        pose = Pose2D(float(p.x), float(p.y), yaw)
        return pose

    def _current_vehicle_spec(self):
        if self.active_vehicle_spec is not None:
            return dict(self.active_vehicle_spec)
        return {
            "wheelbase": self.current_wheelbase,
            "vehicle_length_m": self.vehicle_length,
            "vehicle_width_m": self.vehicle_width,
        }

    def _approach_route_world(
            self, role, start_odom, target, peer_center_world, peer_yaw,
            vehicle_spec):
        """Plan one Front-first approach leg around vehicle and fixed peer."""

        vehicle_length = float(vehicle_spec["vehicle_length_m"])
        vehicle_width = float(vehicle_spec["vehicle_width_m"])
        wheelbase = float(vehicle_spec["wheelbase"])
        if min(vehicle_length, vehicle_width, wheelbase) <= 0.0:
            raise ValueError("vehicle dimensions and wheelbase must be positive")

        start_sd = world_to_vehicle(
            start_odom["x"], start_odom["y"],
            target.x_m, target.y_m, target.yaw_rad)
        goal_sd = (
            approach_longitudinal(role, self.entry_standoff, wheelbase),
            0.0,
        )

        # The robot turns toward target yaw while translating.  Inflate by the
        # larger projection at the initial and aligned orientations.
        moving_start_s, moving_start_d = projected_half_extents(
            self.robot_length, self.robot_width,
            float(start_odom["yaw"]) - target.yaw_rad, 0.0)
        moving_aligned_s, moving_aligned_d = projected_half_extents(
            self.robot_length, self.robot_width, 0.0, 0.0)
        moving_half_s = max(moving_start_s, moving_aligned_s)
        moving_half_d = max(moving_start_d, moving_aligned_d)

        protected_s = (
            vehicle_length / 2.0 + moving_half_s +
            self.approach_robot_clearance)
        protected_d = (
            vehicle_width / 2.0 + moving_half_d +
            self.approach_robot_clearance)
        rectangles = [
            AxisAlignedRect(0.0, 0.0, protected_s, protected_d)
        ]

        peer_s, peer_d = world_to_vehicle(
            peer_center_world[0], peer_center_world[1],
            target.x_m, target.y_m, target.yaw_rad)
        peer_half_s, peer_half_d = projected_half_extents(
            self.robot_length, self.robot_width,
            peer_yaw - target.yaw_rad, 0.0)
        rectangles.append(AxisAlignedRect(
            peer_s,
            peer_d,
            peer_half_s + moving_half_s + self.minimum_inter_robot_gap,
            peer_half_d + moving_half_d + self.minimum_inter_robot_gap,
        ))

        route_sd = plan_route_around_rectangles(
            start_sd,
            goal_sd,
            rectangles,
            corner_margin_m=self.approach_corner_margin,
        )
        route_world = [
            (float(start_odom["x"]), float(start_odom["y"])),
            *[
                vehicle_to_world(
                    s, d, target.x_m, target.y_m, target.yaw_rad)
                for s, d in route_sd
            ],
        ]

        # Geometry avoids the target and peer; this second pass checks the
        # actual OccupancyGrid, map boundary and any other parked vehicles.
        segment_yaw = float(start_odom["yaw"])
        for segment_start, segment_goal in zip(
                route_world, route_world[1:]):
            if not corridor_is_free(
                    self.grid, self.grid_w, self.grid_h, self.resolution,
                    segment_start, segment_goal, segment_yaw,
                    self.robot_length, self.robot_width,
                    margin_m=0.0,
                    unknown_is_occupied=self.unknown_is_occupied,
                    goal_yaw_rad=target.yaw_rad,
                    speed_mps=self.approach_speed,
                    yaw_gain=self.approach_yaw_gain,
                    max_yaw_rate=self.approach_max_yaw_rate):
                return None
            segment_yaw = target.yaw_rad
        return route_world

    def _sequential_approach_preflight(self, target, vehicle_spec):
        """Check Front move, then Rear move, from side-by-side HOME poses."""

        if (self.grid is None or self.front_odom is None or
                self.rear_odom is None or target is None or
                vehicle_spec is None):
            return False
        try:
            front_route = self._approach_route_world(
                "front",
                self.front_odom,
                target,
                (self.rear_odom["x"], self.rear_odom["y"]),
                self.rear_odom["yaw"],
                vehicle_spec,
            )
            if front_route is None:
                return False

            front_goal = front_route[-1]
            rear_route = self._approach_route_world(
                "rear",
                self.rear_odom,
                target,
                front_goal,
                target.yaw_rad,
                vehicle_spec,
            )
            if rear_route is None:
                return False
        except (KeyError, TypeError, ValueError) as exc:
            self.get_logger().error(
                f"side-by-side approach preflight failed: {exc}")
            return False

        self.get_logger().info(
            "side-by-side approach preflight OK | "
            f"front_segments={len(front_route)-1}, "
            f"rear_segments={len(rear_route)-1}")
        return True

    def _handle_park_request(self, payload):
        """Apply base UI validation, then verify the changed HOME geometry."""

        super()._handle_park_request(payload)
        if (self.request_status is None or
                self.request_status.get("status") != "ACCEPTED"):
            return

        target = self._pose_from_stamped(self.target_pose)
        vehicle_spec = self._current_vehicle_spec()
        if self.simultaneous_entry:
            # The field branch is designed for Front-first.  Do not silently
            # approve an unvalidated simultaneous approach.
            preflight_ok = False
        else:
            preflight_ok = self._sequential_approach_preflight(
                target, vehicle_spec)
        if preflight_ok:
            return

        self.ui_park_approved = False
        self.requested_destination_slot_id = ""
        self.active_vehicle_number = ""
        self.active_parking_credential = None
        self._set_request_status(
            payload, "REJECTED", "APPROACH_CORRIDOR_BLOCKED")
        self.get_logger().error(
            "park request rolled back: side-by-side approach preflight failed")

    def _retrieve_approach_preflight(self, record):
        """Front-first preflight for side-by-side HOME poses."""

        if self.simultaneous_entry:
            return super()._retrieve_approach_preflight(record)
        if record.final_vehicle_pose is None or record.vehicle_spec is None:
            return False
        return self._sequential_approach_preflight(
            record.final_vehicle_pose,
            dict(record.vehicle_spec),
        )


def main(args=None):
    rclpy.init(args=args)
    node = FieldFleetManagerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
