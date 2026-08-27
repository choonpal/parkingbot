#!/usr/bin/env python3
"""Minimal runtime ownership fixes for the distributed MVP.

The production motion nodes intentionally share most of their implementation
with the hardened stack. This wrapper resolves two integration problems found
on the real Front robot:

* ``individual_move`` owns ``/{role}/cmd_vel`` during APPROACH, ALIGN and
  RETURN. ``rigid_body_sync`` owns it only while both robots are in DRIVE.
  Outside DRIVE, the rigid controller must not continuously publish zeroes on
  the same topic because the bridge cannot order commands from two publishers.
* The legacy 0.40 m side-lane value is smaller than the configured vehicle +
  robot + clearance envelope. Clamp it to the smallest valid MVP value so the
  normal one-line Front/Rear launches do not fail during node construction.
"""

from __future__ import annotations

import math
from typing import Type

import rclpy
from rclpy.parameter import Parameter

from cooperative_parking_robot.mvp_integration_nodes import (
    HomeAwareIndividualMoveNode as BaseIndividualMoveNode,
)
from cooperative_parking_robot.rigid_body_sync_vision_node import (
    RigidBodySyncNode as BaseRigidBodySyncNode,
)


SIDE_OFFSET_MARGIN_M = 0.015


def minimum_entry_side_offset(
        vehicle_half_width_m: float,
        robot_width_m: float,
        robot_clearance_m: float,
        margin_m: float = SIDE_OFFSET_MARGIN_M) -> float:
    """Return a millimetre-rounded side lane that clears the full envelope."""
    required = (
        float(vehicle_half_width_m) +
        0.5 * float(robot_width_m) +
        float(robot_clearance_m) +
        float(margin_m)
    )
    if not math.isfinite(required) or required <= 0.0:
        raise ValueError('entry side-offset geometry must be finite and positive')
    return math.ceil(required * 1000.0 - 1.0e-9) / 1000.0


def rigid_drive_owns_command(
        *, has_path: bool, vehicle_lifted: bool,
        front_state: str, rear_state: str,
        front_ready: bool, rear_ready: bool, estop: bool) -> bool:
    """True only while the rigid controller is the active cmd_vel owner."""
    return bool(
        not estop and has_path and vehicle_lifted and
        front_ready and rear_ready and
        str(front_state) == 'DRIVE' and str(rear_state) == 'DRIVE'
    )


class MvpIndividualMoveNode(BaseIndividualMoveNode):
    """Use the smallest valid side lane instead of aborting the launch."""

    def _validate_parameters(self):
        required = minimum_entry_side_offset(
            self.vehicle_half_width,
            self.robot_width,
            self.robot_clearance,
        )
        if self.entry_side_offset < required:
            configured = self.entry_side_offset
            result = self.set_parameters([
                Parameter(
                    'entry_side_offset_m',
                    Parameter.Type.DOUBLE,
                    required),
            ])[0]
            if not result.successful:
                raise RuntimeError(
                    'failed to apply valid entry_side_offset_m: '
                    f'{result.reason}')
            self.entry_side_offset = required
            self.get_logger().warn(
                'entry_side_offset_m '
                f'{configured:.3f}m is inside the configured envelope; '
                f'using {required:.3f}m')
        super()._validate_parameters()


class MvpRigidBodySyncNode(BaseRigidBodySyncNode):
    """Publish on cmd_vel only while the rigid controller owns DRIVE."""

    def __init__(self, **kwargs):
        self._drive_command_owned = False
        super().__init__(**kwargs)
        self.get_logger().info(
            'cmd_vel ownership active | individual=APPROACH/ALIGN/RETURN, '
            'rigid=DRIVE')

    def _owns_drive_command_now(self) -> bool:
        return rigid_drive_owns_command(
            has_path=self.has_path,
            vehicle_lifted=self.vehicle_lifted,
            front_state=self.front_robot_state,
            rear_state=self.rear_robot_state,
            front_ready=self.front_ready,
            rear_ready=self.rear_ready,
            estop=self.estop,
        )

    def send_stop(self):
        """Send one final zero only when this node previously owned cmd_vel."""
        if not self._drive_command_owned:
            return
        super().send_stop()
        self._drive_command_owned = False

    def control_loop(self):
        if not self._owns_drive_command_now():
            self.send_stop()
            return

        # From this point, any hold/fault inside the inherited DRIVE controller
        # is allowed to publish a zero command through ``send_stop``.
        self._drive_command_owned = True
        return super().control_loop()


def _spin(node_type: Type, args=None):
    rclpy.init(args=args)
    node = node_type()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


def individual_move_main(args=None):
    _spin(MvpIndividualMoveNode, args)


def rigid_body_sync_main(args=None):
    _spin(MvpRigidBodySyncNode, args)
