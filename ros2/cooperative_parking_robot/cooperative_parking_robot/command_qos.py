"""QoS contract for automatic velocity commands."""

from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)


# Velocity commands are ephemeral state, not an event stream.  Keeping only
# the newest best-effort sample prevents delayed DDS/Wi-Fi delivery from
# replaying a backlog of superseded commands.
CMD_VEL_QOS = QoSProfile(
    history=HistoryPolicy.KEEP_LAST,
    depth=1,
    reliability=ReliabilityPolicy.BEST_EFFORT,
    durability=DurabilityPolicy.VOLATILE,
)
