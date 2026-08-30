from cooperative_parking_robot.fault_policy import FaultClass, classify_fault


def test_required_fault_matrix_classification():
    recoverable = (
        'ERR,HEARTBEAT_TIMEOUT', 'ERR,COMMAND_TIMEOUT',
        'ERR,CLOCK_SKEW:auto:STALE_STAMP', 'ERR,UART_FRAME_CORRUPTION',
        'SERIAL_UNAVAILABLE', 'SERIAL_BUSY', 'ERR,HELLO_REQUIRED',
        'ERR,SERVO_NOT_ATTACHED', 'ERR,BAD_SERVO_ATTACH',
        'MOTION,ULTRASONIC_STALE', 'MOTION,ULTRASONIC_INVALID',
    )
    for reason in recoverable:
        policy = classify_fault(reason)
        assert policy.classification in {
            FaultClass.RECOVERABLE, FaultClass.AVAILABILITY}
        assert policy.estop_required is False
        assert policy.manual_reset_required is False


def test_only_explicit_physical_conditions_request_hard_estop():
    for reason in ('ESTOP', 'ERR,ESTOP_LATCHED',
                   'ERR,WHEEL_DIR_MISMATCH',
                   'SYNC,LATERAL_ERROR_FATAL +41mm'):
        policy = classify_fault(reason)
        assert policy.classification is FaultClass.EMERGENCY
        assert policy.motion_stop_required
        assert policy.estop_required
        assert policy.manual_reset_required


def test_distance_yaw_and_visual_degradation_do_not_stop_or_abort():
    reasons = (
        'SYNC,YAW_ERROR +12.0deg',
        'SYNC,DIST_ERROR_FATAL +90mm',
        'SYNC,RELATIVE_X_ERROR_TIMEOUT +45mm',
        'SYNC,MARKER_HOLD 3.0s',
        'SYNC,ID0_LOSS_HOLD 3.0s',
        'SYNC,CORRECTION_STALE 2.0s',
        'SYNC,SYNC_DEGRADED dist=+0.090m',
    )
    for reason in reasons:
        policy = classify_fault(reason)
        assert policy.classification is FaultClass.AVAILABILITY
        assert policy.motion_stop_required is False
        assert policy.mission_abort_required is False
        assert policy.estop_required is False
        assert policy.manual_reset_required is False


def test_recoverable_communication_faults_still_stop_safely():
    policy = classify_fault('ERR,HEARTBEAT_TIMEOUT')
    assert policy.motion_stop_required
    assert policy.hardware_ready_false
    assert policy.mission_abort_required
    assert policy.reconnect_allowed
