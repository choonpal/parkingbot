"""ROS 2 Humble port metadata and static launch regression tests."""

from pathlib import Path
import re
import xml.etree.ElementTree as ET

from cooperative_parking_robot.hardware_preflight import (
    check_ros_environment,
    validate_second_cctv_assets,
)


ROOT = Path(__file__).resolve().parents[1]


def test_package_and_setup_versions_match():
    package_root = ET.parse(ROOT / 'package.xml').getroot()
    package_version = package_root.findtext('version')
    setup_text = (ROOT / 'setup.py').read_text()
    setup_version = re.search(r"version='([^']+)'", setup_text).group(1)
    assert package_version == setup_version
    assert re.fullmatch(r'\d+\.\d+\.\d+', package_version or '')
    test_dependencies = {
        element.text for element in package_root.findall('test_depend')}
    assert 'ament_pytest' in test_dependencies
    assert 'tests_require' not in setup_text


def test_package_declares_humble_launch_runtime():
    root = ET.parse(ROOT / 'package.xml').getroot()
    dependencies = {node.text for node in root.findall('exec_depend')}
    assert {'rclpy', 'ament_index_python', 'launch', 'launch_ros',
            'ros2launch', 'python3-flask', 'python3-werkzeug',
            'python3-yaml'} <= dependencies


def test_humble_environment_check_accepts_distributed_configuration():
    errors = []
    warnings = []
    check_ros_environment(errors, warnings, {
        'ROS_DISTRO': 'humble',
        'ROS_DOMAIN_ID': '42',
        'ROS_LOCALHOST_ONLY': '0',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
    })
    assert errors == []
    assert warnings == []


def test_humble_environment_check_rejects_wrong_or_localhost_only():
    errors = []
    warnings = []
    check_ros_environment(errors, warnings, {
        'ROS_DISTRO': 'jazzy',
        'ROS_DOMAIN_ID': '42',
        'ROS_LOCALHOST_ONLY': '1',
        'RMW_IMPLEMENTATION': 'rmw_fastrtps_cpp',
    })
    assert any('ROS_DISTRO' in error for error in errors)
    assert any('ROS_LOCALHOST_ONLY' in error for error in errors)


def test_dual_cctv_preflight_rejects_a_missing_second_asset_pair():
    errors = []
    validate_second_cctv_assets(errors, '', '', required=True)
    assert errors and '--cctv2-camera-calib' in errors[0]

    errors = []
    validate_second_cctv_assets(
        errors, '/tmp/cctv2_camera_calibration.npz', '', required=False)
    assert errors and '함께 지정' in errors[0]


def test_camera_topics_are_runtime_parameters():
    expected = {
        'cooperative_parking_robot/yolo_bev_map_node.py':
            "declare_parameter('image_topic', '/cctv/image_rect')",
        'cooperative_parking_robot/cctv_robot_marker_node.py':
            "declare_parameter('image_topic', '/cctv/image_rect')",
        'cooperative_parking_robot/aruco_tracker_node.py':
            "declare_parameter('image_topic', '/rear/marker_camera/image')",
    }
    for relative, text in expected.items():
        source = (ROOT / relative).read_text()
        assert text in source
        assert 'self.image_topic' in source


def test_real_launches_expose_hardware_gates():
    quote_map = str.maketrans({chr(34): chr(39)})
    front = (
        ROOT / 'launch/front_robot.launch.py').read_text().translate(quote_map)
    rear = (
        ROOT / 'launch/rear_robot.launch.py').read_text().translate(quote_map)
    for source in (front, rear):
        assert "'enable_serial'" in source
        assert "'require_serial'" in source
        assert "'require_hardware_ready'" in source
        assert "'require_ultrasonic_for_ready'" in source
        assert 'ParameterValue' in source
    assert "'rear_camera_topic'" in rear


def test_full_system_defaults_to_safe_smoke_mode():
    source = (ROOT / 'launch/full_system.launch.py').read_text()
    for name in (
            'enable_opencv_camera', 'enable_cctv_rectify', 'enable_vision',
            'enable_cctv_robot_markers', 'enable_debug_overlay',
            'enable_rear_aruco', 'enable_serial', 'require_serial',
            'require_hardware_ready', 'require_ultrasonic_for_ready'):
        pattern = rf"'{name}', default_value='false'"
        assert re.search(pattern, source), name
    assert re.search(
        r"'enable_operator_ui', default_value='true'", source)


def test_humble_scripts_are_executable():
    for relative in (
            'scripts/humble_build_check.sh',
            'scripts/humble_topic_check.sh',
            'scripts/run_feature_tests.sh'):
        path = ROOT / relative
        assert path.is_file()
        assert path.stat().st_mode & 0o111
    build_check = (ROOT / 'scripts/humble_build_check.sh').read_text()
    assert 'PYTHONNOUSERSITE=1' in build_check
    assert 'colcon collected zero tests' in build_check


def test_console_entry_points_and_launch_executables_resolve():
    import ast

    setup_text = (ROOT / 'setup.py').read_text()
    setup_tree = ast.parse(setup_text)
    setup_call = next(
        node for node in ast.walk(setup_tree)
        if isinstance(node, ast.Call)
        and isinstance(node.func, ast.Name)
        and node.func.id == 'setup')
    entry_points = next(
        keyword.value for keyword in setup_call.keywords
        if keyword.arg == 'entry_points')
    console_scripts = next(
        value for key, value in zip(entry_points.keys, entry_points.values)
        if isinstance(key, ast.Constant) and key.value == 'console_scripts')
    entries = {}
    for element in console_scripts.elts:
        spec = element.value
        executable, target = (part.strip() for part in spec.split('=', 1))
        module_name, function_name = target.rsplit(':', 1)
        entries[executable] = (module_name, function_name)
    assert entries

    for executable, (module_name, function_name) in entries.items():
        module_path = ROOT / (module_name.replace('.', '/') + '.py')
        assert module_path.is_file(), executable
        tree = ast.parse(module_path.read_text())
        # 진입 함수 이름은 main 이 아닐 수 있다 (예: cctv_merge_main)
        assert any(isinstance(node, (ast.FunctionDef, ast.AsyncFunctionDef))
                   and node.name == function_name
                   for node in tree.body), executable

    launch_executables = set()
    for launch_path in (ROOT / 'launch').glob('*.py'):
        launch_executables.update(re.findall(
            r"executable='([a-zA-Z0-9_]+)'", launch_path.read_text()))
    assert launch_executables <= set(entries)


def test_ceiling_calibration_is_separate_from_rear_calibration():
    setup_text = (ROOT / 'setup.py').read_text()
    cctv_launch = (ROOT / 'launch/cctv_server.launch.py').read_text()
    rear_launch = (
        ROOT / 'launch/rear_robot.launch.py').read_text().translate(
            str.maketrans({chr(34): chr(39)}))
    full_launch = (ROOT / 'launch/full_system.launch.py').read_text()

    assert (ROOT / 'config/cctv_camera_calibration.npz').is_file()
    assert "glob('config/*.npz')" in setup_text
    gitignore = (ROOT / '.gitignore').read_text()
    assert '!config/cctv_camera_calibration.npz' in gitignore
    assert "executable='cctv_rectify'" in cctv_launch
    assert "'cctv_raw_topic'" in cctv_launch
    assert "'cctv_rect_topic'" in cctv_launch
    assert "'config', 'cctv_camera_calibration.npz'" in cctv_launch
    assert "Path.home() / 'ov2710_calib_23mm_white.npz'" in rear_launch
    assert "'rear_camera_width', default_value='1280'" in rear_launch
    assert "'rear_camera_height', default_value='720'" in rear_launch
    assert "'rear_camera_fps', default_value='4.0'" in rear_launch
    assert "'odom_publish_hz', default_value='20.0'" in rear_launch
    assert "'marker_size_m', default_value='0.10'" in rear_launch
    assert "rear_camera_calib" in full_launch
    assert "cctv_camera_calib" in full_launch
    assert 'cctv_camera_calibration.npz' not in rear_launch


def test_rectified_homography_is_not_silently_mixed_with_raw_image():
    cctv_launch = (ROOT / 'launch/cctv_server.launch.py').read_text()
    yolo = (ROOT / 'cooperative_parking_robot/yolo_bev_map_node.py').read_text()
    marker = (ROOT / 'cooperative_parking_robot/cctv_robot_marker_node.py').read_text()
    assert "EnvironmentVariable('HOME')" in cctv_launch
    assert "'homography_rectified.npy'" in cctv_launch
    assert "'.ros', 'adaptive_valet_bot'" in cctv_launch
    assert "'/cctv/image_rect'" in yolo
    assert "'/cctv/image_rect'" in marker
    assert "LaunchConfiguration('cctv_rect_topic')" in cctv_launch
