"""Shared classification for production rigid-body synchronization faults."""

# These faults mean the controller no longer has a usable control basis or the
# lifted payload can be mechanically overloaded.  Ordinary distance/yaw error,
# ID0 loss and correction staleness remain telemetry/degraded-motion states.
SYNC_FATAL_PREFIXES = (
    'ODOM_TIMEOUT',
    'LATERAL_ERROR_FATAL', 'LATERAL_ERROR_TIMEOUT',
    'SYNC_FILTER_INIT_FAILED', 'REFERENCE_CAPTURE_FAILED',
)


def is_fatal_sync_error(error: str) -> bool:
    """Return whether an error requires both Robot and Fleet to stop."""
    return str(error).strip().startswith(SYNC_FATAL_PREFIXES)
