from cooperative_parking_robot.opencv_camera_node import preview_frame_due


def test_preview_rate_limiter_publishes_first_frame_and_respects_period():
    assert preview_frame_due(10.0, None, 0.25)
    assert not preview_frame_due(10.24, 10.0, 0.25)
    assert preview_frame_due(10.25, 10.0, 0.25)


def test_preview_rate_limiter_handles_small_clock_roundoff():
    assert not preview_frame_due(1.0 + 0.25 - 1e-6, 1.0, 0.25)
    assert preview_frame_due(1.0 + 0.25 + 1e-6, 1.0, 0.25)
