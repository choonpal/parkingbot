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
                   'SYNC,DIST_ERROR_FATAL 250mm'):
        policy = classify_fault(reason)
        assert policy.classification is FaultClass.EMERGENCY
        assert policy.motion_stop_required
        assert policy.estop_required
        assert policy.manual_reset_required


def test_recoverable_faults_are_safe_stop_not_ignored():
    policy = classify_fault('ERR,HEARTBEAT_TIMEOUT')
    assert policy.motion_stop_required
    assert policy.hardware_ready_false
    assert policy.mission_abort_required
    assert policy.reconnect_allowed
