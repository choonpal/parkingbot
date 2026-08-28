#!/usr/bin/env python3
"""Production entry point for the phase-scoped STM32 bridge."""

import rclpy

from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode


class MvpStm32BridgeNode(Stm32BridgeNode):
    """Ultrasonic lifecycle and readiness are implemented by the base node."""


def main(args=None):
    rclpy.init(args=args)
    node = MvpStm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
