#!/usr/bin/env python3
"""Production entry point for the phase-scoped STM32 bridge."""

import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor

from cooperative_parking_robot.hardware_profile import servo_attach_pulses_for
from cooperative_parking_robot.mvp_recovery_policy import (
    servo_attach_pulses_from_telemetry,
)
from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode


_GRIP_TARGETS = {
    'robot-1': (1600, 1400),
    'robot-2': (1550, 1450),
}


class MvpStm32BridgeNode(Stm32BridgeNode):
    """Preserve a non-opening servo baseline across communication recovery."""

    def __init__(self, **kwargs):
        self._servo_action_active = None
        super().__init__(**kwargs)

    def _send_grip(self, action):
        # Before the UART action is sent, move the recovery baseline to that
        # action's target. If communication disappears during the servo ramp,
        # a new HELLO session can never re-attach at the startup OPEN position.
        if action == 'grip':
            self.servo_attach_pulses = _GRIP_TARGETS[self.hardware_profile]
            self._servo_action_active = 'grip'
        elif action == 'release':
            self.servo_attach_pulses = servo_attach_pulses_for(
                self.hardware_profile)
            self._servo_action_active = 'release'
        return super()._send_grip(action)

    def _handle_serial_line(self, line):
        result = super()._handle_serial_line(line)
        if line == 'LIFT,GRIP_DONE' and self._servo_action_active == 'grip':
            self._servo_action_active = None
        elif (line == 'LIFT,RELEASE_DONE' and
              self._servo_action_active == 'release'):
            self._servo_action_active = None
        return result

    def publish_motor_diagnostics(self, parsed):
        # Outside an active servo ramp, T telemetry is the best current pulse
        # baseline. During a ramp keep the action target above so stale/early
        # telemetry cannot revert the recovery baseline to OPEN.
        if self._servo_action_active is None:
            self.servo_attach_pulses = servo_attach_pulses_from_telemetry(
                parsed, self.servo_attach_pulses)
        return super().publish_motor_diagnostics(parsed)


def main(args=None):
    rclpy.init(args=args)
    node = MvpStm32BridgeNode()
    executor = MultiThreadedExecutor(num_threads=2)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_stop()
        executor.remove_node(node)
        node.destroy_node()
        executor.shutdown(timeout_sec=1.0)
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
