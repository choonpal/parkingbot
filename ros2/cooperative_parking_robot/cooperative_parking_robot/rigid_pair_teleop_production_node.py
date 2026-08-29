#!/usr/bin/env python3
"""Rigid-pair keyboard control with encoder-rate relative prediction."""

from __future__ import annotations

import math

import rclpy

from cooperative_parking_robot.encoder_relative_predictor import (
    EncoderRelativePredictor,
)
from cooperative_parking_robot.freshness import stamp_to_ns
from cooperative_parking_robot.rigid_pair_teleop_node import (
    RigidPairTeleopNode as BaseRigidPairTeleopNode,
)


class RigidPairTeleopNode(BaseRigidPairTeleopNode):
    """Use wheel SE(2) increments between visual relative-pose updates."""

    def __init__(self):
        self._encoder_relative = None
        super().__init__()
        self.declare_parameter('wheel_pair_sync_slop_s', 0.05)
        sync_slop = float(
            self.get_parameter('wheel_pair_sync_slop_s').value)
        self._encoder_relative = EncoderRelativePredictor(sync_slop)
        self.get_logger().info(
            'keyboard rigid-pair encoder predictor active | '
            f'wheel_pair_slop={sync_slop * 1000.0:.0f}ms')

    def _odom_cb(self, role, msg):
        previous_time = self.odom[role]['time']
        super()._odom_cb(role, msg)
        if (self._encoder_relative is None or
                self.odom[role]['time'] <= previous_time or
                self.odom[role]['pose'] is None):
            return
        self._encoder_relative.note_odom(
            role, self.odom[role]['pose'], stamp_to_ns(msg.header.stamp))

    def _relative_cb(self, msg):
        previous_time = self.relative_time
        super()._relative_cb(msg)
        if (self._encoder_relative is not None and
                self.relative_time > previous_time and
                self.relative is not None):
            # Use the existing outlier-gated/median visual pose as the absolute
            # anchor, then let wheel odom propagate it until the next frame.
            self._encoder_relative.note_visual(self.relative)

    def _control_loop(self):
        if self._encoder_relative is None:
            return super()._control_loop()
        predicted = self._encoder_relative.predict()
        if predicted is None or self.state != 'ARMED':
            return super()._control_loop()
        visual_relative = self.relative
        self.relative = predicted
        try:
            return super()._control_loop()
        finally:
            # Keep visual history/outlier rejection independent from the fast
            # predictor. Only the control-cycle error uses the predicted pose.
            self.relative = visual_relative

    def _status_payload(self):
        payload = super()._status_payload()
        predicted = (
            None if self._encoder_relative is None else
            self._encoder_relative.predict())
        if predicted is not None:
            payload['pose'].update({
                'encoder_predicted_forward_cm': predicted[0] * 100.0,
                'encoder_predicted_lateral_cm': predicted[1] * 100.0,
                'encoder_predicted_yaw_deg': math.degrees(predicted[2]),
            })
        return payload


def main(args=None):
    rclpy.init(args=args)
    node = RigidPairTeleopNode()
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
