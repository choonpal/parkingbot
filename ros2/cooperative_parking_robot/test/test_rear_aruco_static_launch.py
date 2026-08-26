from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_static_aruco_launch_is_perception_only_and_site_configured():
    source = (ROOT / "launch" / "rear_aruco_static_check.launch.py").read_text(
        encoding="utf-8")

    assert 'executable="opencv_camera"' in source
    assert 'executable="aruco_tracker"' in source
    assert 'executable="camera_preview"' in source
    for forbidden in (
            'executable="stm32_bridge"',
            'executable="state_machine"',
            'executable="individual_move"',
            'executable="rigid_body_sync"'):
        assert forbidden not in source

    assert 'default_value="640"' in source
    assert 'default_value="480"' in source
    assert 'default_value="0.05"' in source
    assert 'Path.home() / "ov2710_calib_23mm_white.npz"' in source
    assert '"allow_uncalibrated": False' in source
    assert '"enable_bev": False' in source
    assert '"enable_yolo": False' in source
    assert 'default_value="5005"' in source
