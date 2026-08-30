"""Central safety semantics shared by ROS fault producers/consumers."""

from dataclasses import dataclass
from enum import Enum


class FaultClass(Enum):
    EMERGENCY = "EMERGENCY"
    RECOVERABLE = "RECOVERABLE"
    AVAILABILITY = "AVAILABILITY"


@dataclass(frozen=True)
class FaultPolicy:
    classification: FaultClass
    motion_stop_required: bool
    hardware_ready_false: bool
    mission_abort_required: bool
    reconnect_allowed: bool
    estop_required: bool
    manual_reset_required: bool


EMERGENCY = FaultPolicy(
    FaultClass.EMERGENCY, True, True, True, False, True, True)
RECOVERABLE = FaultPolicy(
    FaultClass.RECOVERABLE, True, True, True, True, False, False)
AVAILABILITY = FaultPolicy(
    FaultClass.AVAILABILITY, False, True, False, True, False, False)


# Only conditions with an explicit physical hard-latch justification belong
# here. Unknown faults remain fail-closed (EMERGENCY), so adding a new code
# cannot silently weaken safety.
_EMERGENCY_PREFIXES = (
    "ESTOP", "ERR,ESTOP_LATCHED", "ERR,WHEEL_DIR_MISMATCH",
    "SYNC,YAW_ERROR", "SYNC,DIST_ERROR_FATAL",
    "SYNC,RELATIVE_X_ERROR_FATAL", "SYNC,LATERAL_ERROR_FATAL",
)

_RECOVERABLE_PREFIXES = (
    "HARDWARE_ACK_TIMEOUT", "FLEET_STATE_TIMEOUT",
    "MISSION_CHANGED_WHILE_ACTIVE", "APPROACH_TIMEOUT", "ALIGN_TIMEOUT",
    "LIFT_TIMEOUT", "DRIVE_TIMEOUT", "RELEASE_BARRIER_TIMEOUT",
    "RELEASE_TIMEOUT", "RETURN_TIMEOUT", "MOTION,",
    "SYNC,ODOM_TIMEOUT", "SYNC,MARKER_LOST", "SYNC,DIST_ERROR_TIMEOUT",
    "SYNC,RELATIVE_X_ERROR_TIMEOUT", "SYNC,LATERAL_ERROR_TIMEOUT",
    "SYNC,SYNC_FILTER_INIT_FAILED", "SYNC,REFERENCE_CAPTURE_FAILED",
    "SYNC,SLOT_POSE_MISSING",
    "ERR,HEARTBEAT_TIMEOUT", "ERR,HEARTBEAT_ACK_TIMEOUT",
    "ERR,PROTOCOL_HANDSHAKE_TIMEOUT",
    "ERR,COMMAND_TIMEOUT", "ERR,HELLO_REQUIRED",
    "ERR,HEARTBEAT_REQUIRED", "ERR,COMMAND_REQUIRED",
    "ERR,STARTUP_SEQUENCE", "ERR,CLOCK_SKEW",
    "ERR,UART_", "ERR,RX_QUEUE_OVERFLOW", "ERR,TX_FRAME_INVALID",
    "ERR,BAD_HELLO", "ERR,BAD_HEARTBEAT_TOKEN", "ERR,BAD_V_FRAME",
    "ERR,BAD_ZERO_PROBE", "ERR,BAD_VELOCITY", "ERR,BAD_MOTOR_TEST",
    "ERR,BAD_SERVO_COMMAND", "ERR,UNKNOWN_COMMAND",
    "ERR,SERVO_NOT_ATTACHED", "ERR,BAD_SERVO_ATTACH",
    "ERR,LIFT_WHILE_MOVING", "ERR,SERVO_TIMEOUT",
    "ULTRASONIC_", "WARN,ULTRASONIC_",
)

_AVAILABILITY_PREFIXES = (
    "CAMERA_", "CCTV_", "YOLO_", "UI_", "SERIAL_UNAVAILABLE",
    "SERIAL_BUSY", "STARTUP_DEPENDENCY_", "OPTIONAL_SENSOR_",
)


def classify_fault(reason: str) -> FaultPolicy:
    """Classify a normalized status/reason without scattered substring tests."""
    value = str(reason or "").strip().upper()
    if value.startswith(_EMERGENCY_PREFIXES):
        return EMERGENCY
    if value.startswith(_RECOVERABLE_PREFIXES):
        return RECOVERABLE
    if value.startswith(_AVAILABILITY_PREFIXES):
        return AVAILABILITY
    return EMERGENCY
