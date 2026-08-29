#!/usr/bin/env python3
"""Production entry point for the phase-scoped STM32 bridge."""

import signal

import rclpy
from rclpy.executors import MultiThreadedExecutor

from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode


class MvpStm32BridgeNode(Stm32BridgeNode):
    """Ultrasonic lifecycle and readiness are implemented by the base node."""


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
