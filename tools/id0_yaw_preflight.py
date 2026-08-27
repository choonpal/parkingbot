#!/usr/bin/env python3
"""Fail-closed stationary stability check for the Rear camera's Front ID0 yaw."""

from __future__ import annotations

import argparse
import json
import math
import statistics
import time
from dataclasses import asdict, dataclass


def normalize_angle(angle: float) -> float:
    return math.atan2(math.sin(angle), math.cos(angle))


def circular_center(samples: list[float]) -> float:
    if not samples:
        raise ValueError("samples must not be empty")
    sine = sum(math.sin(value) for value in samples)
    cosine = sum(math.cos(value) for value in samples)
    if abs(sine) < 1.0e-12 and abs(cosine) < 1.0e-12:
        return samples[0]
    return math.atan2(sine, cosine)


@dataclass(frozen=True)
class YawStabilityResult:
    passed: bool
    reason: str
    sample_count: int
    visibility_count: int
    visible_count: int
    visible_ratio: float
    center_deg: float | None
    std_deg: float | None
    max_deviation_deg: float | None
    max_step_deg: float | None


def evaluate_yaw_stability(
        samples: list[float], *, visibility_count: int, visible_count: int,
        min_samples: int, min_visible_ratio: float, max_std_deg: float,
        max_deviation_deg: float, max_step_deg: float) -> YawStabilityResult:
    ratio = (
        float(visible_count) / float(visibility_count)
        if visibility_count > 0 else 0.0)
    if visibility_count <= 0:
        return YawStabilityResult(
            False, "NO_MARKER_VISIBILITY_MESSAGES", len(samples),
            visibility_count, visible_count, ratio, None, None, None, None)
    if ratio < min_visible_ratio:
        return YawStabilityResult(
            False, "MARKER_VISIBILITY_RATIO_LOW", len(samples),
            visibility_count, visible_count, ratio, None, None, None, None)
    if len(samples) < min_samples:
        return YawStabilityResult(
            False, "INSUFFICIENT_POSE_SAMPLES", len(samples),
            visibility_count, visible_count, ratio, None, None, None, None)

    center = circular_center(samples)
    deviations_deg = [
        math.degrees(normalize_angle(value - center)) for value in samples]
    std_deg = statistics.pstdev(deviations_deg)
    peak_deg = max(abs(value) for value in deviations_deg)
    steps_deg = [
        abs(math.degrees(normalize_angle(current - previous)))
        for previous, current in zip(samples, samples[1:])]
    step_deg = max(steps_deg, default=0.0)

    reason = "OK"
    passed = True
    if std_deg > max_std_deg:
        passed = False
        reason = "YAW_STD_EXCEEDED"
    elif peak_deg > max_deviation_deg:
        passed = False
        reason = "YAW_DEVIATION_EXCEEDED"
    elif step_deg > max_step_deg:
        passed = False
        reason = "YAW_STEP_EXCEEDED"

    return YawStabilityResult(
        passed, reason, len(samples), visibility_count, visible_count, ratio,
        math.degrees(center), std_deg, peak_deg, step_deg)


def _yaw_from_quaternion(orientation) -> float | None:
    values = (
        float(orientation.x), float(orientation.y),
        float(orientation.z), float(orientation.w))
    if not all(math.isfinite(value) for value in values):
        return None
    norm = math.sqrt(sum(value * value for value in values))
    if norm < 1.0e-9:
        return None
    x, y, z, w = (value / norm for value in values)
    return math.atan2(
        2.0 * (w * z + x * y),
        1.0 - 2.0 * (y * y + z * z))


def _stamp_ns(stamp) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def collect_and_evaluate(args) -> YawStabilityResult:
    try:
        import rclpy
        from geometry_msgs.msg import PoseStamped
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from std_msgs.msg import Bool
    except ImportError as exc:
        return YawStabilityResult(
            False, f"ROS_IMPORT_FAILED:{exc}", 0, 0, 0, 0.0,
            None, None, None, None)

    class Collector(Node):
        def __init__(self):
            super().__init__("parkingbot_id0_yaw_preflight")
            self.samples: list[float] = []
            self.visibility_count = 0
            self.visible_count = 0
            self.last_stamp_ns = 0
            self.create_subscription(
                Bool, args.visible_topic, self.visible_cb,
                qos_profile_sensor_data)
            self.create_subscription(
                PoseStamped, args.pose_topic, self.pose_cb,
                qos_profile_sensor_data)

        def visible_cb(self, message):
            self.visibility_count += 1
            if bool(message.data):
                self.visible_count += 1

        def pose_cb(self, message):
            stamp_ns = _stamp_ns(message.header.stamp)
            if stamp_ns <= 0 or stamp_ns <= self.last_stamp_ns:
                return
            yaw = _yaw_from_quaternion(message.pose.orientation)
            if yaw is None:
                return
            self.last_stamp_ns = stamp_ns
            self.samples.append(yaw)

    rclpy.init(args=[])
    node = Collector()
    deadline = time.monotonic() + args.duration
    try:
        while True:
            remaining = deadline - time.monotonic()
            if remaining <= 0.0:
                break
            rclpy.spin_once(node, timeout_sec=min(0.05, remaining))
    finally:
        samples = list(node.samples)
        visibility_count = node.visibility_count
        visible_count = node.visible_count
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()

    return evaluate_yaw_stability(
        samples,
        visibility_count=visibility_count,
        visible_count=visible_count,
        min_samples=args.min_samples,
        min_visible_ratio=args.min_visible_ratio,
        max_std_deg=args.max_std_deg,
        max_deviation_deg=args.max_deviation_deg,
        max_step_deg=args.max_step_deg)


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Keep both robots stationary and place them so the Rear camera "
            "continuously sees Front marker ID0 before running this check."))
    parser.add_argument("--duration", type=float, default=5.0)
    parser.add_argument("--min-samples", type=int, default=30)
    parser.add_argument("--min-visible-ratio", type=float, default=0.80)
    parser.add_argument("--max-std-deg", type=float, default=2.0)
    parser.add_argument("--max-deviation-deg", type=float, default=5.0)
    parser.add_argument("--max-step-deg", type=float, default=5.0)
    parser.add_argument("--pose-topic", default="/sync/relative_pose")
    parser.add_argument("--visible-topic", default="/sync/marker_visible")
    args = parser.parse_args()

    if args.duration <= 0.0:
        parser.error("--duration must be positive")
    if args.min_samples <= 0:
        parser.error("--min-samples must be positive")
    if not 0.0 < args.min_visible_ratio <= 1.0:
        parser.error("--min-visible-ratio must be in (0, 1]")
    if min(args.max_std_deg, args.max_deviation_deg, args.max_step_deg) <= 0.0:
        parser.error("yaw limits must be positive")

    result = collect_and_evaluate(args)
    print(json.dumps(asdict(result), ensure_ascii=False, sort_keys=True))
    return 0 if result.passed else 1


if __name__ == "__main__":
    raise SystemExit(main())
