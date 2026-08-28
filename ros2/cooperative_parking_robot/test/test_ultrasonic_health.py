import pytest

from cooperative_parking_robot.ultrasonic_health import (
    mark_ultrasonic_frame,
    stale_ultrasonic_sides,
    ultrasonic_streams_fresh,
)


def test_timeout_frames_still_count_as_alive_streams():
    stamps = {'left': 0.0, 'right': 0.0}
    mark_ultrasonic_frame(stamps, 'left', 100.00)
    mark_ultrasonic_frame(stamps, 'right', 100.02)
    assert ultrasonic_streams_fresh(stamps, 100.20, 0.50)
    assert stale_ultrasonic_sides(stamps, 100.20, 0.50) == []


def test_missing_stream_not_missing_echo_drops_readiness():
    stamps = {'left': 100.00, 'right': 100.40}
    assert stale_ultrasonic_sides(stamps, 100.55, 0.50) == ['left']
    assert not ultrasonic_streams_fresh(stamps, 100.55, 0.50)


def test_requirement_can_be_disabled_without_fake_timestamps():
    stamps = {'left': 0.0, 'right': 0.0}
    assert ultrasonic_streams_fresh(
        stamps, 100.0, 0.50, required=False)


def test_unknown_side_is_rejected():
    with pytest.raises(ValueError):
        mark_ultrasonic_frame({'left': 0.0, 'right': 0.0}, 'center', 1.0)
