#!/usr/bin/env python3
"""Mission FSM wrapper with phase-scoped servo ACKs and software rearm."""

from __future__ import annotations

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot.fault_policy import classify_fault
from cooperative_parking_robot.mvp_recovery_policy import (
    stage_accepts_lift_status,
)
from cooperative_parking_robot.robot_state_machine_node import (
    RobotStateMachineNode as BaseRobotStateMachineNode,
)


_REARMABLE_STATES = {
    'APPROACH', 'ALIGN', 'LIFT', 'DRIVE',
    'WAIT_RELEASE', 'RELEASE', 'RETURN',
}


class RobotStateMachineNode(BaseRobotStateMachineNode):
    """Resume recoverable mission faults without touching the hard E-stop."""

    def __init__(self, **kwargs):
        self._fault_origin_state = 'IDLE'
        super().__init__(**kwargs)
        self.create_subscription(Bool, '/robot/rearm', self.rearm_cb, 10)

    def lift_cb(self, msg):
        status = str(msg.data)
        if not stage_accepts_lift_status(self.state, status):
            if status in ('GRIP_DONE', 'RELEASE_DONE'):
                self.get_logger().warn(
                    f'[{self.role}] stale {status} ignored in {self.state}',
                    throttle_duration_sec=1.0)
            return
        if status == 'GRIP_DONE':
            self.lift_done = True
        else:
            self.release_done = True

    def transition(self, new):
        if new == 'FAULT' and self.state != 'FAULT':
            self._fault_origin_state = self.state
        if new == 'LIFT':
            self.lift_done = False
        elif new == 'RELEASE':
            self.release_done = False
        return super().transition(new)

    def rearm_cb(self, msg):
        if not bool(msg.data) or self.state != 'FAULT':
            return
        policy = classify_fault(self.hardware_fault)
        if policy.estop_required:
            self.get_logger().error(
                f'[{self.role}] software rearm rejected for hard fault: '
                f'{self.hardware_fault}')
            return
        if self.require_hardware_ready and not self.hardware_ready:
            self.get_logger().warn(
                f'[{self.role}] software rearm waiting for hardware_ready=true')
            return
        target = (
            self._fault_origin_state
            if self._fault_origin_state in _REARMABLE_STATES
            else 'IDLE')
        self.hardware_fault = None
        if target == 'LIFT':
            self.lift_done = False
        elif target == 'RELEASE':
            self.release_done = False
        self.get_logger().warn(
            f'[{self.role}] software rearm: FAULT -> {target}')
        self.transition(target)


def main(args=None):
    rclpy.init(args=args)
    node = RobotStateMachineNode()
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
