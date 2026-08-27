"""Shared classification for production rigid-body synchronization faults."""

SYNC_FATAL_PREFIXES = (
    'ODOM_TIMEOUT', 'MARKER_LOST', 'YAW_ERROR',
    'DIST_ERROR_FATAL', 'DIST_ERROR_TIMEOUT',
    'RELATIVE_X_ERROR_FATAL', 'RELATIVE_X_ERROR_TIMEOUT',
    'LATERAL_ERROR_FATAL', 'LATERAL_ERROR_TIMEOUT',
    'SYNC_FILTER_INIT_FAILED', 'REFERENCE_CAPTURE_FAILED',
)


def is_fatal_sync_error(error: str) -> bool:
    """Return whether an error requires both Robot and Fleet to stop."""
    return str(error).strip().startswith(SYNC_FATAL_PREFIXES)
