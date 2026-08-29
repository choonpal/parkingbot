"""Demand-gating regression tests for Rear relative vision."""

from pathlib import Path
import time

from std_msgs.msg import Bool

from cooperative_parking_robot.aruco_tracker_node import ArucoTrackerNode
from cooperative_parking_robot.individual_move_node import IndividualMoveNode
from cooperative_parking_robot.opencv_camera_node import OpenCvCameraNode


ROOT = Path(__file__).resolve().parents[1]


class Publisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class Logger:
    def info(self, *_args, **_kwargs):
        pass


def test_only_rear_owns_relative_vision_phase_request():
    rear = object.__new__(IndividualMoveNode)
    rear.is_front = False
    rear.phase = 'IDLE'
    rear.pub_relative_vision_enable = Publisher()
    rear.relative_vision_ready = True
    rear.relative_marker_visible = True
    rear.relative_receipt_time = time.monotonic()
    rear.relative_x = 0.8
    rear.relative_y = 0.0
    rear.relative_yaw = 0.0

    rear.publish_relative_vision_request()
    assert rear.pub_relative_vision_enable.messages[-1].data is False
    assert rear.relative_receipt_time is None

    rear.phase = 'READY_TO_SCAN'
    rear.publish_relative_vision_request()
    assert rear.pub_relative_vision_enable.messages[-1].data is True

    rear.phase = 'EXIT_TO_SIDE'
    rear.publish_relative_vision_request()
    assert rear.pub_relative_vision_enable.messages[-1].data is False

    front = object.__new__(IndividualMoveNode)
    front.is_front = True
    front.phase = 'READY_TO_SCAN'
    front.pub_relative_vision_enable = None
    front.publish_relative_vision_request()  # no publisher, no conflict


def test_rear_approach_gate_requires_ready_and_fresh_visible_id0():
    node = object.__new__(IndividualMoveNode)
    node.is_front = False
    node.approach_sent = False
    node.relative_vision_ready = False
    node.relative_marker_visible = False
    node.relative_receipt_time = None
    node.relative_x = None
    node.aruco_timeout = 0.30
    node.pub_approach_done = Publisher()
    stops = []
    node.stop = lambda: stops.append(True)

    assert node.publish_approach_ready_if_observed() is False
    assert stops and not node.pub_approach_done.messages

    node.relative_vision_ready = True
    node.relative_marker_visible = True
    node.relative_receipt_time = time.monotonic()
    node.relative_x = 0.8
    assert node.publish_approach_ready_if_observed() is True
    assert node.pub_approach_done.messages[-1].data is True


def test_disabled_aruco_returns_before_image_conversion():
    class ExplodingBridge:
        def imgmsg_to_cv2(self, *_args, **_kwargs):
            raise AssertionError('disabled ArUco must not convert frames')

    node = object.__new__(ArucoTrackerNode)
    node.runtime_enabled = False
    node.bridge = ExplodingBridge()
    node.image_cb(object())


def test_disabled_aruco_repeats_false_state_for_late_startup_observers():
    node = object.__new__(ArucoTrackerNode)
    node.runtime_enabled = False
    node.pub_visible = Publisher()

    node.runtime_enable_cb(Bool(data=False))

    assert len(node.pub_visible.messages) == 1
    assert node.pub_visible.messages[0].data is False


def test_camera_activation_resets_readiness_and_drops_buffered_frames():
    node = object.__new__(OpenCvCameraNode)
    node.runtime_enabled = False
    node.runtime_ready = True
    node.ready_publisher = Publisher()
    node.last_standby_read = 1.0
    node.activation_drop_frames = 3
    node.activation_drop_remaining = 0
    node.output_topic = '/rear/marker_camera/image'
    node.get_logger = lambda: Logger()

    node.runtime_enable_cb(Bool(data=True))
    assert node.runtime_enabled is True
    assert node.runtime_ready is False
    assert node.activation_drop_remaining == 3
    assert node.ready_publisher.messages[-1].data is False


def test_rear_launch_prewarms_vision_then_starts_bridge_in_standby():
    source = (ROOT / 'launch/rear_robot.launch.py').read_text()
    camera = source.index('executable="opencv_camera"')
    aruco = source.index('executable="aruco_tracker"')
    bridge = source.index('executable="stm32_bridge"')
    assert camera < aruco < bridge
    assert 'TimerAction(period=3.0' in source
    assert 'TimerAction(period=8.0' in source
    assert '"runtime_enable_topic": "/rear/relative_vision_enable"' in source
    assert source.count('"start_enabled": False') >= 2
    assert '"standby_fps": _float("rear_camera_standby_fps")' in source
    assert '"capture_fps": _float("rear_camera_capture_fps")' in source
    assert '"rear_camera_fourcc", default_value="MJPG"' in source


def test_full_system_gates_only_rear_marker_camera_not_overview_cctv():
    source = (ROOT / 'launch/full_system.launch.py').read_text()
    overview_start = source.index("name='opencv_camera_node'")
    rear_start = source.index("name='rear_marker_camera_node'")
    overview_block = source[overview_start:rear_start]
    rear_block = source[rear_start:]

    assert "'runtime_enable_topic'" not in overview_block
    assert "'runtime_enable_topic': '/rear/relative_vision_enable'" in rear_block
    assert "'capture_fps': _float('rear_camera_capture_fps')" in rear_block


def test_real_robot_launches_reserve_one_core_for_each_uart_bridge():
    front = (ROOT / 'launch/front_robot.launch.py').read_text()
    rear = (ROOT / 'launch/rear_robot.launch.py').read_text()

    for source in (front, rear):
        assert '"bridge_cpu_set", default_value="3"' in source
        assert '"worker_cpu_set", default_value="0-2"' in source
        assert 'prefix=bridge_prefix' in source
    assert front.count('prefix=worker_prefix') == 5
    assert rear.count('prefix=worker_prefix') == 6
