#!/usr/bin/env python3
"""QoS contracts for superseded latest-value and retained safety state."""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


# High-rate sensor/control state is superseded by the next sample.  A depth
# larger than one can replay stale poses after a slow executor or Wi-Fi pause.
SENSOR_LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)

# Low-rate state still needs reliable delivery, but old states must not queue.
STATE_LATEST_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.VOLATILE,
)

# A late-joining safety consumer must immediately see the current latch/fault.
SAFETY_STATE_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.RELIABLE,
    durability=DurabilityPolicy.TRANSIENT_LOCAL,
)
