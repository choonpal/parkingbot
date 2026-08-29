#!/usr/bin/env python3
"""Fleet wrapper that replans after an explicitly rearmed recoverable fault."""

from __future__ import annotations

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot.fault_policy import classify_fault
from cooperative_parking_robot.fleet_manager_node import (
    FleetManagerNode as BaseFleetManagerNode,
)


class FleetManagerNode(BaseFleetManagerNode):
    def __init__(self):
        self._fault_origin_state = 'WAIT_TARGET'
        super().__init__()
        self.create_subscription(Bool, '/robot/rearm', self.rearm_cb, 10)

    def sync_status_cb(self, msg):
        previous = self.state
        super().sync_status_cb(msg)
        if previous != 'FAULT' and self.state == 'FAULT':
            self._fault_origin_state = previous

    def rearm_cb(self, msg):
        if not bool(msg.data):
            return
        # Robot-side motion faults are cleared by each individual-move wrapper.
        # Fleet FAULT itself is entered by rigid-body sync faults.
        if self.state != 'FAULT':
            return
        policy = classify_fault(f'SYNC,{self.sync_fault}')
        if policy.estop_required:
            self.get_logger().error(
                f'fleet software rearm rejected for hard sync fault: '
                f'{self.sync_fault}')
            return
        self.sync_fault = ''
        if self.mission_id:
            # fatal_stop discards the old controller path. Replan from current
            # fresh odometry instead of replaying the stale pre-fault path.
            self.path_published = False
            self.state = 'PLAN_PATH'
        else:
            self.state = 'WAIT_TARGET'
        self.publish_state()
        self.get_logger().warn(
            f'fleet software rearm -> {self.state}')


def main(args=None):
    rclpy.init(args=args)
    node = FleetManagerNode()
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
