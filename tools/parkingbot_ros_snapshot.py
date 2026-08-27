#!/usr/bin/env python3
"""Collect one bounded ParkingBot ROS snapshot with a single DDS participant."""

import argparse
import json
import time

import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import OccupancyGrid, Odometry
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Bool, String

from parkingbot_ops import PRESENCE_TOPICS, TOPICS


BOOL_KEYS = {
    "target_ready", "front_hw", "rear_hw", "front_marker",
    "rear_marker", "id0_marker",
}
PRESENCE_TYPES = {
    "front_odom": Odometry,
    "rear_odom": Odometry,
    "relative_pose": PoseStamped,
    "map_stream": OccupancyGrid,
}


def qos(*, transient=False):
    return QoSProfile(
        history=HistoryPolicy.KEEP_LAST,
        depth=1,
        reliability=ReliabilityPolicy.BEST_EFFORT,
        durability=(DurabilityPolicy.TRANSIENT_LOCAL if transient else
                    DurabilityPolicy.VOLATILE),
    )


class SnapshotNode(Node):
    def __init__(self):
        super().__init__("parkingbot_diagnostic_snapshot")
        self.values = {key: None for key in TOPICS}
        self.values.update({key: False for key in PRESENCE_TOPICS})
        self.received = set()

        for key, topic in TOPICS.items():
            message_type = Bool if key in BOOL_KEYS else String
            # hardware_status is a retained safety state; all other sampled
            # topics are volatile latest-value streams.
            profile = qos(transient=key in (
                "front_hw_status", "rear_hw_status"))
            self.create_subscription(
                message_type, topic,
                lambda msg, k=key: self._value(k, msg), profile)
        for key, topic in PRESENCE_TOPICS.items():
            self.create_subscription(
                PRESENCE_TYPES[key], topic,
                lambda _msg, k=key: self._present(k), qos())

    def _value(self, key, message):
        self.values[key] = message.data
        self.received.add(key)

    def _present(self, key):
        self.values[key] = True
        self.received.add(key)


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--timeout", type=float, default=1.2)
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")

    rclpy.init(args=[])
    node = SnapshotNode()
    deadline = time.monotonic() + args.timeout
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.05, deadline - time.monotonic()))
            if len(node.received) == len(node.values):
                break
        print(json.dumps({"topics": node.values}, ensure_ascii=False))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
