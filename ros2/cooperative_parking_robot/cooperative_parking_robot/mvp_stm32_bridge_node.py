#!/usr/bin/env python3
"""MVP STM32 bridge with stream-aware ultrasonic readiness.

The base bridge intentionally publishes ``Range(inf)`` for ``U,*,TIMEOUT`` so
motion/ALIGN logic cannot treat a missing echo as a measured target. Hardware
readiness answers a different question: is the sensor/UART stream alive? A
TIMEOUT frame is positive evidence that the MCU and measurement cycle are still
running, so it must not drop ``hardware_ready`` by itself.
"""

import signal
import time

import rclpy
from std_msgs.msg import Bool

from cooperative_parking_robot.stm32_bridge_node import Stm32BridgeNode
from cooperative_parking_robot.ultrasonic_health import (
    mark_ultrasonic_frame,
    stale_ultrasonic_sides,
    ultrasonic_streams_fresh,
)


class MvpStm32BridgeNode(Stm32BridgeNode):
    """Use frame freshness for readiness and echo validity for measurements."""

    def __init__(self, **kwargs):
        self.last_ultrasonic_frame = {'left': 0.0, 'right': 0.0}
        super().__init__(**kwargs)
        self.get_logger().info(
            f'[{self.role}] ultrasonic readiness uses stream freshness; '
            'U,*,TIMEOUT remains a valid alive frame')

    def publish_ultrasonic(self, parsed):
        if self.hello_acknowledged:
            mark_ultrasonic_frame(
                self.last_ultrasonic_frame,
                parsed['side'],
                time.monotonic(),
            )
        return super().publish_ultrasonic(parsed)

    def hardware_ready_conditions(self, now=None):
        now = time.monotonic() if now is None else now
        conditions = super().hardware_ready_conditions(now)
        conditions['ultrasonic_fresh'] = ultrasonic_streams_fresh(
            self.last_ultrasonic_frame,
            now,
            self.ultrasonic_frame_timeout,
            required=self.require_ultrasonic_for_ready,
        )
        return conditions

    def publish_hardware_state(self):
        now = time.monotonic()
        conditions = self.hardware_ready_conditions(now)
        ready = all(conditions.values())
        if ready and not self.hardware_ready:
            self.get_logger().info(f'[{self.role}] hardware_ready=true')
        elif not ready and self.hardware_ready:
            self.get_logger().warn(f'[{self.role}] hardware_ready=false')
        self.hardware_ready = ready
        self.pub_ready.publish(Bool(data=ready))
        self.pub_manual_active.publish(Bool(
            data=self.command_arbiter.manual_enabled))

        stale = (self.require_ultrasonic_for_ready and
                 not conditions['ultrasonic_fresh'])
        if stale and not self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = True
            missing = stale_ultrasonic_sides(
                self.last_ultrasonic_frame,
                now,
                self.ultrasonic_frame_timeout,
            )
            self.publish_status(
                'WARN,ULTRASONIC_STALE:' + '|'.join(missing))
        elif not stale and self.ultrasonic_stale_reported:
            self.ultrasonic_stale_reported = False
            self.publish_status('INFO,ULTRASONIC_STREAM_OK')

        if self.active_fault is not None:
            self.publish_status(self.active_fault)


def main(args=None):
    rclpy.init(args=args)
    node = MvpStm32BridgeNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        signal.signal(signal.SIGINT, signal.SIG_IGN)
        node.shutdown_stop()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
