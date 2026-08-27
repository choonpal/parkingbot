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

from parkingbot_ops import (
    observer_complete, PRESENCE_TOPICS, TOPICS,
)


BOOL_KEYS = {
    "target_ready", "front_hw", "rear_hw", "front_marker",
    "rear_marker", "id0_marker", "front_aligned_hold",
    "rear_aligned_hold",
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
    parser.add_argument("--mode", choices=("full", "startup"), default="full")
    parser.add_argument(
        "--stream-interval", type=float, default=0.0,
        help="emit JSON-lines progress at this interval; zero emits only final")
    args = parser.parse_args()
    if args.timeout <= 0.0:
        parser.error("--timeout must be positive")
    if args.stream_interval < 0.0:
        parser.error("--stream-interval must be non-negative")

    rclpy.init(args=[])
    node = SnapshotNode()
    deadline = time.monotonic() + args.timeout
    next_emit = time.monotonic() + args.stream_interval
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(
                node, timeout_sec=min(0.05, deadline - time.monotonic()))
            complete = observer_complete(node.received, args.mode)
            if args.stream_interval and time.monotonic() >= next_emit:
                print(json.dumps({
                    "topics": node.values, "complete": complete,
                    "mode": args.mode,
                }, ensure_ascii=False), flush=True)
                next_emit = time.monotonic() + args.stream_interval
            # Streaming startup mode keeps this participant alive so changing
            # readiness values (not merely topic discovery) remain observable.
            # One-shot callers may return as soon as their required keys arrive.
            if complete and not args.stream_interval:
                break
        print(json.dumps({
            "topics": node.values,
            "complete": observer_complete(node.received, args.mode),
            "mode": args.mode,
        }, ensure_ascii=False), flush=True)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
