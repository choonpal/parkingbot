#!/usr/bin/env python3
"""ROS-independent orchestration and status helpers for production tooling."""

from __future__ import annotations

import datetime as dt
import json
import os
from pathlib import Path
import re
import shlex
import subprocess
import time

SESSION = "parkingbot-production"
ROLES = ("jetson", "rear", "front")
REQUIRED = (
    "JETSON_HOST", "FRONT_HOST", "REAR_HOST", "JETSON_WORKSPACE",
    "FRONT_WORKSPACE", "REAR_WORKSPACE", "CONTROL_WORKSPACE", "ROS_DOMAIN_ID",
    "CAM0_DEVICE", "CAM2_DEVICE", "MODEL_PATH", "CAM0_GROUND_X_M",
    "CAM0_GROUND_Y_M", "CAM0_HEIGHT_M", "CAM2_GROUND_X_M",
    "CAM2_GROUND_Y_M", "CAM2_HEIGHT_M", "FRONT_MARKER_HEIGHT_M",
    "REAR_MARKER_HEIGHT_M", "WHEELBASE", "FRONT_SERIAL",
    "FRONT_WHEEL_RADIUS", "FRONT_ENCODER_PPR", "FRONT_LX", "FRONT_LY",
    "FRONT_LEFT_SENSOR_X", "FRONT_RIGHT_SENSOR_X", "REAR_SERIAL",
    "REAR_WHEEL_RADIUS", "REAR_ENCODER_PPR", "REAR_LX", "REAR_LY",
    "REAR_LEFT_SENSOR_X", "REAR_RIGHT_SENSOR_X", "REAR_CALIB",
)
ABSOLUTE_PATH_KEYS = (
    "JETSON_WORKSPACE", "FRONT_WORKSPACE", "REAR_WORKSPACE",
    "CONTROL_WORKSPACE", "ROS_SETUP", "CAM0_DEVICE", "CAM2_DEVICE",
    "MODEL_PATH", "FRONT_SERIAL", "REAR_SERIAL", "REAR_CALIB",
    "REAR_CAMERA_DEVICE",
)

TOPICS = {
    "fleet": "/fleet/state",
    "target_ready": "/parking/target_ready",
    "target_status": "/parking/target_status",
    "vehicle_spec": "/parking/vehicle_spec",
    "merge": "/cctv/merge_status",
    "front_state": "/front/robot_state",
    "rear_state": "/rear/robot_state",
    "front_aligned_hold": "/front/aligned_hold",
    "rear_aligned_hold": "/rear/aligned_hold",
    "front_hw": "/front/hardware_ready",
    "rear_hw": "/rear/hardware_ready",
    "front_hw_status": "/front/hardware_status",
    "rear_hw_status": "/rear/hardware_status",
    "front_fault": "/front/motion_fault",
    "rear_fault": "/rear/motion_fault",
    "front_loc": "/front/localization_status",
    "rear_loc": "/rear/localization_status",
    "front_marker": "/front/cctv_marker_visible",
    "rear_marker": "/rear/cctv_marker_visible",
    "id0_marker": "/sync/marker_visible",
    "sync": "/sync/error_state",
}
PRESENCE_TOPICS = {
    "front_odom": "/front/odom",
    "rear_odom": "/rear/odom",
    "relative_pose": "/sync/relative_pose",
    "map_stream": "/parking/map",
}
STARTUP_REQUIRED_TOPIC_KEYS = (
    "fleet", "front_state", "rear_state", "merge", "id0_marker",
    "front_hw", "rear_hw",
)
EXPECTED_UART_PROTOCOL_VERSION = 2
EXPECTED_UART_BAUD_RATE = 115200


def protocol_consistency_errors(repository_root) -> list[str]:
    """Verify that firmware, encoder API and bridge require one protocol."""
    root = Path(repository_root)
    sources = {
        "firmware": root / (
            "stm32/parking_robot/Core/Src/parking_robot_firmware.c"),
        "uart_protocol": root / (
            "ros2/cooperative_parking_robot/cooperative_parking_robot/"
            "uart_protocol.py"),
        "bridge": root / (
            "ros2/cooperative_parking_robot/cooperative_parking_robot/"
            "stm32_bridge_node.py"),
    }
    text = {}
    errors = []
    for name, path in sources.items():
        try:
            text[name] = path.read_text(encoding="utf-8")
        except OSError:
            errors.append(f"{name}: source missing: {path}")
    if errors:
        return errors
    version = EXPECTED_UART_PROTOCOL_VERSION
    required = {
        "firmware": (
            rf"#define\s+UART_PROTOCOL_VERSION\s+{version}U",
            rf"#define\s+UART_BAUD_RATE\s+{EXPECTED_UART_BAUD_RATE}U",
            rf'"HELLO:%u:%s"',
            "protocol_session_active",
        ),
        "uart_protocol": (
            rf"PROTOCOL_VERSION\s*=\s*{version}",
            rf"UART_BAUD_RATE\s*=\s*{EXPECTED_UART_BAUD_RATE}",
            "def encode_hello",
            "def encode_zero_velocity",
        ),
        "bridge": (
            "self.protocol.encode_hello(self.session_id)",
            "hello_acknowledged",
            "zero_command_acknowledged",
        ),
    }
    for name, patterns in required.items():
        for pattern in patterns:
            found = (re.search(pattern, text[name]) is not None
                     if "\\s" in pattern else pattern in text[name])
            if not found:
                errors.append(f"{name}: missing protocol capability {pattern}")
    return errors


def protocol_source_check_command(repository_root) -> str:
    """Render the remote doctor equivalent of protocol_consistency_errors."""
    # 원격에 배포되는 것은 ROS 패키지뿐이고 펌웨어 소스(stm32/)는 저장소
    # 루트에만 있다. 여기서 펌웨어까지 찾으면 어떤 장비에서도 통과할 수
    # 없다. 펌웨어 대조는 제어 장비의 protocol_consistency_errors() 가
    # 이미 수행하므로, 원격에서는 배포된 ROS 소스만 확인한다.
    # 호출부는 저장소 루트(.../src/cooperative_parking_robot)를 넘긴다.
    # 현재 통합 저장소는 ROS 패키지가 ros2/ 아래에 있지만, 기존 장비의
    # package-only 배포도 계속 진단할 수 있어야 한다.
    repository = str(repository_root)
    monorepo_package = shlex.quote(
        repository + "/ros2/cooperative_parking_robot/"
        "cooperative_parking_robot")
    legacy_package = shlex.quote(
        repository + "/cooperative_parking_robot")
    return (
        f"if test -f {monorepo_package}/uart_protocol.py; then "
        f"package={monorepo_package}; else package={legacy_package}; fi && "
        f"grep -Eq 'PROTOCOL_VERSION[[:space:]]*=[[:space:]]*"
        f"{EXPECTED_UART_PROTOCOL_VERSION}' \"$package/uart_protocol.py\" && "
        f"grep -Eq 'UART_BAUD_RATE[[:space:]]*=[[:space:]]*"
        f"{EXPECTED_UART_BAUD_RATE}' \"$package/uart_protocol.py\" && "
        f"grep -q 'encode_hello(self.session_id)' "
        f"\"$package/stm32_bridge_node.py\"")


def launch_file_check_command(repository_root, launch_file) -> str:
    """Check a launch file in monorepo and legacy package-only deployments."""
    repository = str(repository_root)
    monorepo_launch = shlex.quote(
        repository + "/ros2/cooperative_parking_robot/launch/" + launch_file)
    legacy_launch = shlex.quote(repository + "/launch/" + launch_file)
    return f"test -f {monorepo_launch} || test -f {legacy_launch}"


def load_env(path: Path) -> dict[str, str]:
    values = {}
    if not path.exists():
        raise ValueError(
            f"site config missing: {path}\n"
            "copy tools/production_hosts.env.example and fill measured values")
    for number, raw in enumerate(path.read_text().splitlines(), 1):
        line = raw.strip()
        if not line or line.startswith("#"):
            continue
        if "=" not in line:
            raise ValueError(f"{path}:{number}: expected KEY=VALUE")
        key, value = line.split("=", 1)
        key = key.strip()
        if not re.fullmatch(r"[A-Z][A-Z0-9_]*", key):
            raise ValueError(f"{path}:{number}: invalid key {key!r}")
        parsed = shlex.split(value, comments=True)
        values[key] = parsed[0] if parsed else ""
    values.setdefault("ROS_LOCALHOST_ONLY", "0")
    values.setdefault("RMW_IMPLEMENTATION", "rmw_fastrtps_cpp")
    values.setdefault("ROS_SETUP", "/opt/ros/humble/setup.bash")
    values.setdefault("OBSERVER_PYTHON", "/usr/bin/python3")
    values.setdefault("REAR_CAMERA_TOPIC", "/rear/marker_camera/image")
    values.setdefault("REAR_ENABLE_INTERNAL_CAMERA", "false")
    values.setdefault("REAR_CAMERA_DEVICE", "")
    return values


def missing_config(config: dict[str, str]) -> list[str]:
    return [key for key in REQUIRED if not config.get(key, "").strip()]


def invalid_paths(config: dict[str, str]) -> list[str]:
    return [key for key in ABSOLUTE_PATH_KEYS
            if config.get(key) and not config[key].startswith("/")]


def conditional_config_errors(config: dict[str, str]) -> list[str]:
    internal = config.get("REAR_ENABLE_INTERNAL_CAMERA", "false").lower()
    if internal not in ("true", "false"):
        return ["REAR_ENABLE_INTERNAL_CAMERA must be true or false"]
    if (internal == "true" and
            not config.get("REAR_CAMERA_DEVICE", "").strip() and
            not config.get("REAR_CAMERA_ID", "").strip()):
        return [
            "REAR_CAMERA_DEVICE or REAR_CAMERA_ID is required for the "
            "internal camera"]
    if internal == "false" and not config.get(
            "REAR_EXTERNAL_CAMERA_COMMAND", "").strip():
        return ["REAR_EXTERNAL_CAMERA_COMMAND is required for the external camera"]
    serial_write_timeout = config.get("SERIAL_WRITE_TIMEOUT_S", "0.05")
    try:
        serial_write_timeout_value = float(serial_write_timeout)
    except ValueError:
        return ["SERIAL_WRITE_TIMEOUT_S must be a number in [0.01, 0.10]"]
    if not 0.01 <= serial_write_timeout_value <= 0.10:
        return ["SERIAL_WRITE_TIMEOUT_S must be in [0.01, 0.10]"]
    return []


class Runner:
    def run(self, argv, *, timeout=10, check=False):
        try:
            return subprocess.run(
                argv, text=True, capture_output=True,
                timeout=timeout, check=check)
        except subprocess.TimeoutExpired as exc:
            # 원격이 느리다고 진단 도구가 죽으면 무엇이 느린지도 못 본다.
            # 실패한 결과로 바꿔 돌려주고 판단은 호출부에 맡긴다.
            return subprocess.CompletedProcess(
                argv, returncode=124,
                stdout=(exc.stdout or "") if isinstance(exc.stdout, str) else "",
                stderr=f"timeout after {timeout}s")


def remote_argv(host: str, command: str) -> list[str]:
    if host in ("local", "localhost", "127.0.0.1"):
        return ["bash", "-lc", command]
    return [
        "ssh", "-o", "BatchMode=yes", "-o", "ConnectTimeout=5",
        "-o", "ServerAliveInterval=2", "-o", "ServerAliveCountMax=2",
        host, "bash", "-lc", shlex.quote(command),
    ]


def remote_run(runner: Runner, host: str, command: str, timeout=10):
    return runner.run(remote_argv(host, command), timeout=timeout)


def stack_process_status(runner, host, timeout=7):
    """Return authoritative tmux pane state without treating SSH errors as exit."""
    command = (
        f"if tmux has-session -t {shlex.quote(SESSION)} 2>/dev/null; then "
        f"tmux display-message -p -t {shlex.quote(SESSION + ':stack.0')} "
        "'#{pane_pid}|#{pane_dead}|#{pane_dead_status}|"
        "#{pane_current_command}'; "
        "else printf 'MISSING\\n'; exit 3; fi")
    result = remote_run(runner, host, command, timeout=timeout)
    status = {
        "state": "UNKNOWN",
        "pid": None,
        "process_alive": None,
        "returncode": None,
        "probe_returncode": result.returncode,
    }
    output = result.stdout.strip().splitlines()
    if result.returncode == 3 and output and output[-1] == "MISSING":
        status["state"] = "SESSION_MISSING"
        status["process_alive"] = False
        return status
    if result.returncode != 0 or not output:
        return status
    fields = output[-1].split("|", 3)
    if len(fields) != 4 or not fields[0].isdigit():
        return status
    pid_text, dead_text, dead_status, current_command = fields
    dead = dead_text in ("1", "on", "true")
    status.update({
        "state": "EXITED" if dead else "RUNNING",
        "pid": int(pid_text),
        "process_alive": not dead,
        "returncode": (int(dead_status) if dead and
                       dead_status.lstrip("-").isdigit() else None),
        "current_command": current_command,
    })
    return status


def role_host(config, role):
    return config[f"{role.upper()}_HOST"]


def role_workspace(config, role):
    return config[f"{role.upper()}_WORKSPACE"]


def ros_environment_prefix(config, workspace):
    return (
        f"source {shlex.quote(config['ROS_SETUP'])} && "
        f"source {shlex.quote(workspace + '/install/setup.bash')} && "
        f"export ROS_DOMAIN_ID={shlex.quote(config['ROS_DOMAIN_ID'])} && "
        f"export ROS_LOCALHOST_ONLY={shlex.quote(config['ROS_LOCALHOST_ONLY'])} && "
        f"export RMW_IMPLEMENTATION={shlex.quote(config['RMW_IMPLEMENTATION'])}")


def ros_prefix(config, role):
    return ros_environment_prefix(config, role_workspace(config, role))


def local_ros_prefix(config):
    """Controller ROS environment, independent of the calling shell."""
    return ros_environment_prefix(config, config["CONTROL_WORKSPACE"])


def observer_environment_prefix(config):
    """ROS environment for the standard-message-only diagnostic observer."""
    return (
        f"source {shlex.quote(config['ROS_SETUP'])} && "
        f"export ROS_DOMAIN_ID={shlex.quote(config['ROS_DOMAIN_ID'])} && "
        f"export ROS_LOCALHOST_ONLY={shlex.quote(config['ROS_LOCALHOST_ONLY'])} && "
        f"export RMW_IMPLEMENTATION={shlex.quote(config['RMW_IMPLEMENTATION'])}")


def local_ros_argv(config, argv):
    """Run a controller-side ROS command after sourcing underlay + overlay."""
    command = local_ros_prefix(config) + " && exec " + shlex.join(
        [str(item) for item in argv])
    return ["bash", "-lc", command]


def observer_argv(config, argv):
    """Run an observer command with the ROS underlay, without a project overlay."""
    command = observer_environment_prefix(config) + " && exec " + shlex.join(
        [str(item) for item in argv])
    return ["bash", "-lc", command]


def observer_helper_path():
    return Path(__file__).with_name("parkingbot_ros_snapshot.py")


def observer_prerequisite_errors(config, runner=None, helper=None):
    """Validate the local observer before any remote production process starts."""
    runner = runner or Runner()
    helper = Path(helper or observer_helper_path())
    python = Path(config.get("OBSERVER_PYTHON", "/usr/bin/python3"))
    errors = []
    if not Path(config["ROS_SETUP"]).is_file():
        errors.append(f"ROS setup missing: {config['ROS_SETUP']}")
    if not helper.is_file() or not os.access(helper, os.R_OK):
        errors.append(f"snapshot helper missing: {helper}")
    ops_path = helper.with_name("parkingbot_ops.py")
    if not ops_path.is_file() or not os.access(ops_path, os.R_OK):
        errors.append(f"snapshot support module missing: {ops_path}")
    if not python.is_absolute():
        errors.append("OBSERVER_PYTHON must be an absolute path")
    elif not python.is_file() or not os.access(python, os.X_OK):
        errors.append(f"observer Python is not executable: {python}")
    if errors:
        return errors
    probe = (
        f"import sys; sys.path.insert(0, {str(helper.parent)!r}); "
        "import parkingbot_ops; import rclpy; "
        "import geometry_msgs.msg; import nav_msgs.msg; "
        "import std_msgs.msg; "
        "rclpy.init(args=[]); "
        "n=rclpy.create_node('parkingbot_observer_preflight'); "
        "n.destroy_node(); rclpy.shutdown()")
    result = runner.run(
        observer_argv(config, [str(python), "-c", probe]), timeout=12)
    if result.returncode != 0:
        detail = (result.stderr or result.stdout or "no diagnostic output").strip()
        errors.append(
            f"observer Python/ROS/RMW preflight failed rc={result.returncode}: "
            f"{detail[-1200:]}")
    return errors


def q(value):
    if str(value).startswith("$HOME/"):
        return '"' + str(value) + '"'
    return shlex.quote(str(value))


def launch_command(config, role):
    if role == "jetson":
        runtime = "$HOME/.ros/adaptive_valet_bot"
        camera_width = config.get("CAMERA_WIDTH_PX", "640")
        camera_height = config.get("CAMERA_HEIGHT_PX", "360")
        camera_fps = config.get("CAMERA_FPS", "30")

        def mjpeg_pipeline(device):
            return (
                f"v4l2src device={device} io-mode=2 ! "
                f"image/jpeg,width={camera_width},height={camera_height},"
                f"framerate={camera_fps}/1 ! jpegdec ! videoconvert ! "
                "video/x-raw,format=BGR ! "
                "appsink drop=true max-buffers=1 sync=false")

        args = {
            "enable_opencv_camera": "true", "camera0_device": config["CAM0_DEVICE"],
            "camera2_device": config["CAM2_DEVICE"],
            "camera_width_px": camera_width,
            "camera_height_px": camera_height,
            "camera_fps": camera_fps,
            "camera0_gstreamer_pipeline": mjpeg_pipeline(
                config["CAM0_DEVICE"]),
            "camera2_gstreamer_pipeline": mjpeg_pipeline(
                config["CAM2_DEVICE"]),
            "cctv0_camera_calib": f"{runtime}/cctv0_camera_calibration.npz",
            "cctv2_camera_calib": f"{runtime}/cctv2_camera_calibration.npz",
            "calibration_width_px": camera_width,
            "calibration_height_px": camera_height,
            "homography_cam0_file": f"{runtime}/homography_cam0_rectified.npy",
            "homography_cam2_file": f"{runtime}/homography_cam2_rectified.npy",
            "layout_config": f"{runtime}/parking_layout.yaml",
            "parking_registry_db_path": config.get(
                "PARKING_REGISTRY_DB_PATH",
                f"{runtime}/parking_registry.db"),
            "model_path": config["MODEL_PATH"],
            "cam0_ground_x_m": config["CAM0_GROUND_X_M"],
            "cam0_ground_y_m": config["CAM0_GROUND_Y_M"],
            "cam0_height_m": config["CAM0_HEIGHT_M"],
            "cam2_ground_x_m": config["CAM2_GROUND_X_M"],
            "cam2_ground_y_m": config["CAM2_GROUND_Y_M"],
            "cam2_height_m": config["CAM2_HEIGHT_M"],
            "camera_ground_points": "[" + ", ".join((
                config["CAM0_GROUND_X_M"], config["CAM0_GROUND_Y_M"],
                config["CAM2_GROUND_X_M"], config["CAM2_GROUND_Y_M"])) + "]",
            "front_marker_height_m": config["FRONT_MARKER_HEIGHT_M"],
            "rear_marker_height_m": config["REAR_MARKER_HEIGHT_M"],
            "enable_operator_ui": "true", "enable_debug_overlay": "false",
            "simultaneous_entry": "false", "require_all_cameras": "true",
            "require_exact_camera_resolution": "true",
        }
        launch = "cctv_server_dual.launch.py"
    else:
        p = role.upper()
        args = {
            "serial_port": config[f"{p}_SERIAL"], "enable_serial": "true",
            "require_serial": "true", "require_hardware_ready": "true",
            "require_ultrasonic_for_ready": "true", "wheelbase": config["WHEELBASE"],
            "serial_write_timeout_s": config.get(
                "SERIAL_WRITE_TIMEOUT_S", "0.05"),
            "wheel_radius": config[f"{p}_WHEEL_RADIUS"],
            "encoder_ppr": config[f"{p}_ENCODER_PPR"],
            "lx": config[f"{p}_LX"], "ly": config[f"{p}_LY"],
            "left_sensor_to_gripper_x_m": config[f"{p}_LEFT_SENSOR_X"],
            "right_sensor_to_gripper_x_m": config[f"{p}_RIGHT_SENSOR_X"],
            "simultaneous_entry": "false",
            "stop_after_align": config.get("STOP_AFTER_ALIGN", "false"),
            # 벤치 시험용 완화값. 실주행 전 0.25 로 되돌린다.
            "command_source_timeout_s": config.get(
                "COMMAND_SOURCE_TIMEOUT_S", "0.25"),
        }
        if role == "front":
            args["use_aruco_distance"] = "true"
            launch = "front_robot.launch.py"
        else:
            args.update({
                "enable_rear_camera": config["REAR_ENABLE_INTERNAL_CAMERA"],
                "rear_camera_topic": config["REAR_CAMERA_TOPIC"],
                "camera_calib": config["REAR_CALIB"],
            })
            if config["REAR_ENABLE_INTERNAL_CAMERA"].lower() == "true":
                camera_device = config.get(
                    "REAR_CAMERA_DEVICE", "").strip()
                if camera_device:
                    args["rear_camera_device"] = camera_device
                else:
                    args["rear_camera_id"] = config["REAR_CAMERA_ID"]
            launch = "rear_robot.launch.py"
    rendered = " ".join(f"{key}:={q(value)}" for key, value in args.items())
    return f"ros2 launch cooperative_parking_robot {launch} {rendered}"


def run_root(base=None):
    root = Path(base or os.environ.get(
        "PARKINGBOT_LOG_ROOT", "~/.ros/parkingbot_logs")).expanduser()
    root.mkdir(parents=True, exist_ok=True)
    return root


def create_run_dir(base=None, timestamp=None):
    stamp = timestamp or dt.datetime.now().strftime("%Y%m%d_%H%M%S")
    root = run_root(base)
    target = root / stamp
    for name in (*ROLES, "state", "incidents"):
        (target / name).mkdir(parents=True, exist_ok=False)
    latest = root / "latest"
    if latest.is_symlink() or latest.exists():
        latest.unlink()
    latest.symlink_to(target.name)
    return target


def parse_scalar(raw):
    # `ros2 topic echo` 는 메시지마다 구분선 '---' 를 덧붙인다. 그대로 두면
    # JSON 파싱이 실패해 값이 문자열로 남고, 멀쩡한 토픽이 UNAVAILABLE 로
    # 보인다. 구분선 줄만 걷어낸 뒤 파싱한다.
    text = "\n".join(
        line for line in str(raw).splitlines()
        if line.strip() != "---").strip()
    if text in ("true", "false"):
        return text == "true"
    if (len(text) >= 2 and text[0] == text[-1] and text[0] in "'\""):
        text = text[1:-1]
    try:
        return json.loads(text)
    except (json.JSONDecodeError, TypeError):
        return text


def topic_once(config, runner, topic, timeout=1.2):
    try:
        result = runner.run(
            local_ros_argv(config, [
                "ros2", "topic", "echo", "--once", "--field", "data",
                topic]),
            timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None
    return parse_scalar(result.stdout) if result.returncode == 0 else None


def topic_present(config, runner, topic, timeout=1.2):
    try:
        result = runner.run(
            local_ros_argv(config, [
                "ros2", "topic", "echo", "--once", topic]),
            timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return False
    return result.returncode == 0 and bool(result.stdout.strip())


def parse_observer_output(stdout, returncode=0, stderr="", timed_out=False):
    """Turn helper output into an explicit observer result contract."""
    lines = str(stdout or "").splitlines()
    payload = None
    for line in reversed(lines):
        try:
            candidate = json.loads(line)
        except json.JSONDecodeError:
            continue
        if isinstance(candidate, dict) and isinstance(
                candidate.get("topics"), dict):
            payload = candidate
            break
    if timed_out:
        error_type = "timeout"
        message = "snapshot helper timed out"
    elif returncode != 0:
        error_type = "process_exit"
        message = f"snapshot helper exited rc={returncode}"
    elif payload is None:
        error_type = "malformed_output"
        message = "snapshot helper returned no valid JSON topic payload"
    else:
        return {
            "observer_ok": True, "observer_error_type": None,
            "observer_returncode": returncode, "observer_stderr": "",
            "observer_stdout": "", "observer_timed_out": False,
            "topics": payload["topics"],
            "complete": bool(payload.get("complete", False)),
        }
    return {
        "observer_ok": False, "observer_error_type": error_type,
        "observer_returncode": returncode,
        "observer_stderr": str(stderr or "").strip()[-2000:],
        "observer_stdout": str(stdout or "").strip()[-2000:],
        "observer_timed_out": bool(timed_out), "topics": {},
        "complete": False, "message": message,
    }


def observer_complete(received, mode):
    required = (set(STARTUP_REQUIRED_TOPIC_KEYS) if mode == "startup"
                else set((*TOPICS, *PRESENCE_TOPICS)))
    return required.issubset(set(received))


def _snapshot_topics(config, runner, timeout, mode="full"):
    helper = observer_helper_path()
    python = config.get("OBSERVER_PYTHON", "/usr/bin/python3")
    result = runner.run(
        observer_argv(config, [
            python, str(helper), "--timeout", str(float(timeout)),
            "--mode", mode]),
        timeout=float(timeout) + 2.0)
    return parse_observer_output(
        result.stdout, result.returncode, result.stderr,
        timed_out=result.returncode == 124)


def snapshot(config, runner=None, timeout=1.2, hosts=None, mode="full",
             observer_result=None):
    runner = runner or Runner()
    # One bounded observer subscribes to every field.  The previous design
    # kept three ros2 topic echo processes alive at a time for up to 12 s and
    # serialized 23 probes, producing minute-long snapshots and continuous
    # DDS participant churn on the RPis.
    observer = observer_result or _snapshot_topics(
        config, runner, timeout, mode=mode)
    values = {key: None for key in TOPICS}
    values.update({key: False for key in PRESENCE_TOPICS})
    sampled = observer["topics"]
    if observer["observer_ok"]:
        for key in values:
            if key in sampled:
                values[key] = sampled[key]
    for key in ("fleet", "target_status", "vehicle_spec", "merge", "front_loc", "rear_loc", "sync"):
        if isinstance(values[key], str):
            try:
                values[key] = json.loads(values[key])
            except json.JSONDecodeError:
                pass
    fleet = values["fleet"] if isinstance(values["fleet"], dict) else {}
    sync = values["sync"] if isinstance(values["sync"], dict) else {}
    blockers = []
    for role in ("front", "rear"):
        state = values[f"{role}_state"]
        fault = values[f"{role}_fault"]
        if state in (None, "UNKNOWN"):
            blockers.append(f"{role.upper()} STATE UNAVAILABLE")
        elif state == "FAULT":
            blockers.append(f"{role.upper()} ROBOT FAULT")
        if values[f"{role}_hw"] is not True:
            blockers.append(f"{role.upper()} HARDWARE NOT READY")
        hardware_status = values[f"{role}_hw_status"]
        if isinstance(hardware_status, str) and "ERR" in hardware_status.upper():
            blockers.append(f"{role.upper()} HARDWARE: {hardware_status}")
        if not values[f"{role}_odom"]:
            blockers.append(f"{role.upper()} ODOM NOT FRESH")
        if values[f"{role}_loc"] is None:
            blockers.append(f"{role.upper()} LOCALIZATION UNAVAILABLE")
        if values[f"{role}_marker"] is not True:
            blockers.append(f"{role.upper()} CCTV MARKER NOT VISIBLE")
        if fault not in (None, "", "OK", "-"):
            blockers.append(f"{role.upper()} MOTION FAULT: {fault}")
    sync_error = sync.get("error") if isinstance(sync, dict) else sync
    if sync_error not in (None, "", "OK", "ARRIVED"):
        blockers.append(f"SYNC: {sync_error}")
    if values["id0_marker"] is not True:
        blockers.append("ID0 MARKER NOT VISIBLE")
    if not fleet:
        blockers.append("FLEET STATE UNAVAILABLE")
    if fleet and not fleet.get("vehicle_spec_ready", False):
        blockers.append("VEHICLE DIMENSION NOT READY")
    merge = values["merge"] if isinstance(values["merge"], dict) else {}
    cameras = merge.get("cameras", {}) if merge else {}
    for camera in ("cam0", "cam2"):
        if not cameras.get(camera, {}).get("alive", False):
            blockers.append(f"{camera.upper()} NOT FRESH")
    if not values["map_stream"]:
        blockers.append("MAP NOT FRESH")
    if not observer["observer_ok"]:
        blockers.insert(0, "OBSERVER FAILURE: " + observer["message"])
    data = {
        "timestamp": dt.datetime.now(dt.timezone.utc).astimezone().isoformat(),
        "hosts": hosts or {}, "topics": values, "fleet_state": fleet.get("state", "UNKNOWN"),
        "mission_id": fleet.get("mission_id", ""), "empty_slots": fleet.get("empty_count"),
        "blockers": blockers, "overall": "READY FOR PARK" if not blockers else "NOT READY",
        "observer": {key: value for key, value in observer.items()
                     if key != "topics"},
    }
    return data


def _shown(value):
    if value is None:
        return "UNAVAILABLE"
    if isinstance(value, bool):
        return "TRUE" if value else "FALSE"
    if isinstance(value, dict):
        return str(value.get("state", value.get("error", "OK")))
    return str(value)


def format_snapshot(data):
    v = data["topics"]
    fleet = v["fleet"] if isinstance(v["fleet"], dict) else {}
    spec = v["vehicle_spec"] if isinstance(v["vehicle_spec"], dict) else {}
    sync = v["sync"] if isinstance(v["sync"], dict) else {}
    merge = v["merge"] if isinstance(v["merge"], dict) else {}
    cameras = merge.get("cameras", {})
    observer = data.get("observer", {"observer_ok": True})
    if observer.get("observer_ok"):
        observer_line = "OK"
    else:
        detail = observer.get("observer_stderr") or observer.get("observer_stdout")
        observer_line = "FAIL: " + observer.get(
            "message", "unknown observer error")
        if detail:
            observer_line += " | " + detail.splitlines()[-1]
    lines = [
        f"PARKINGBOT STATUS  {data['timestamp']}", "=" * 56, "",
        "OBSERVER", observer_line, "",
        "HOSTS",
        f"Jetson       {data.get('hosts', {}).get('jetson', 'UNKNOWN')}",
        f"Front RPi    {data.get('hosts', {}).get('front', 'UNKNOWN')}",
        f"Rear RPi     {data.get('hosts', {}).get('rear', 'UNKNOWN')}", "",
        "MISSION", f"Fleet        {data['fleet_state']}",
        f"Mission ID   {data['mission_id'] or '-'}",
        f"Target       {_shown(v['target_ready'])}",
        f"Vehicle Spec {'VALID' if fleet.get('vehicle_spec_ready') else 'NOT READY'}",
        f"Empty Slots  {_shown(data['empty_slots'])}", "",
    ]
    for role in ("front", "rear"):
        lines.extend([
            role.upper(), f"State        {_shown(v[role + '_state'])}",
            f"Aligned Hold {_shown(v[role + '_aligned_hold'])}",
            f"HW Ready     {_shown(v[role + '_hw'])}",
            f"Localization {_shown(v[role + '_loc'])}",
            f"Motion Fault {_shown(v[role + '_fault'])}",
            f"CCTV Marker  {_shown(v[role + '_marker'])}", "",
        ])
    lines.extend([
        "SYNC", f"ID0 Marker   {_shown(v['id0_marker'])}",
        f"Reference    {_shown(sync.get('reference_state'))}",
        f"Sync Error   {_shown(sync.get('error'))}", "",
        "VISION", f"Merge        {_shown(v['merge'])}",
        f"CCTV0        {'OK / FRESH' if cameras.get('cam0', {}).get('alive') else 'NOT READY'}",
        f"CCTV2        {'OK / FRESH' if cameras.get('cam2', {}).get('alive') else 'NOT READY'}",
        f"Map          {'OK' if v['map_stream'] else 'NOT READY'}",
        f"Dimension    {'VALID' if spec.get('dimension_valid') else 'NOT READY'}", "",
        "FAULTS",
        f"Front        {_shown(v['front_fault'])}",
        f"Rear         {_shown(v['rear_fault'])}",
        f"Sync         {_shown(sync.get('error'))}", "",
        "OVERALL", data["overall"],
    ])
    if data["blockers"]:
        lines.extend(["", "BLOCKERS:"] + [f"- {item}" for item in data["blockers"]])
    return "\n".join(lines)


def incident_snapshot(run_dir: Path, reason: str, state: dict, config,
                      runner=None, metadata=None):
    runner = runner or Runner()
    safe = re.sub(r"[^A-Z0-9_-]+", "_", reason.upper()).strip("_")[:64] or "FAULT"
    target = run_dir / "incidents" / (
        dt.datetime.now().strftime("%Y%m%d_%H%M%S_%f_") + safe)
    target.mkdir(parents=True, exist_ok=False)
    incident = dict(metadata or {})
    incident.setdefault("reason", reason)
    incident.setdefault(
        "timestamp", dt.datetime.now(dt.timezone.utc).astimezone().isoformat())
    recorded_state = dict(state)
    recorded_state["incident"] = incident
    (target / "state.json").write_text(
        json.dumps(recorded_state, indent=2, ensure_ascii=False) + "\n")
    (target / "incident.json").write_text(
        json.dumps(incident, indent=2, ensure_ascii=False) + "\n")
    (target / "summary.txt").write_text(format_snapshot(state) + "\n")
    for command, filename in ((["ros2", "node", "list"], "ros_node_list.txt"),
                              (["ros2", "topic", "list", "-t"], "ros_topic_list.txt")):
        try:
            result = runner.run(local_ros_argv(config, command), timeout=3)
            (target / filename).write_text(result.stdout + result.stderr)
        except (subprocess.TimeoutExpired, FileNotFoundError) as exc:
            (target / filename).write_text(f"unavailable: {exc}\n")
    for role in ROLES:
        source = run_dir / role / f"{role}_robot.log"
        if role == "jetson":
            source = run_dir / role / "cctv_server_dual.log"
        lines = source.read_text(errors="replace").splitlines()[-200:] if source.exists() else []
        (target / f"{role}_tail.log").write_text("\n".join(lines) + "\n")
    return target
