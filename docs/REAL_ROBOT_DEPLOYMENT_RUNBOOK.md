# 실차 탑재·실행 Runbook

이 문서는 배포, 분산 기동, 7인치 UI와 장애 복구의 기준 절차다. Calibration,
Homography와 preflight는 [pipeline](pipeline.md), 시험 단계와 최종 운용 판정은
[실차 준비도](REAL_WORLD_READINESS.md)를 따른다. 전체 문서 관계는
[문서 안내](README.md)에 정리돼 있다.

소프트웨어가 동작해도 하중 안전은 보장되지 않는다. 보호 지그, 물리 ESTOP,
사람 감독과 실제 파지/하중 확인 수단 없이 차량을 들어 올리는 무인 운용은
**NO-GO**다. `GRIP_DONE`은 서보 목표각 도달 신호일 뿐 하중 확인이 아니다.

## 1. 배포

Jetson과 Front/Rear Raspberry Pi는 Ubuntu 22.04, ROS 2 Humble, 같은
`ROS_DOMAIN_ID`, 동기화된 NTP/chrony 시각과 신뢰 가능한 격리 LAN을 사용한다.
저장소 기본 branch를 clone하거나 검증된 release/tag/commit을 명시한다. 과거
`feature/exit-mission-integration` branch를 배포 기준으로 사용하지 않는다.

```bash
git clone git@github.com:choonpal/parkingbot.git ~/parkingbot
mkdir -p ~/parkingbot_ws/src
ln -s ~/parkingbot/ros2/cooperative_parking_robot \
  ~/parkingbot_ws/src/cooperative_parking_robot

source /opt/ros/humble/setup.bash
cd ~/parkingbot_ws
~/parkingbot/ros2/cooperative_parking_robot/scripts/humble_build_check.sh \
  ~/parkingbot_ws
```

이 script는 Humble system Python을 사용하도록 build/test subprocess에서 user-site를
차단하고, 테스트가 0개 수집되면 실패한다. 현재 suite의 정확한 개수를 문서에
고정하지는 않지만 결과가 반드시 0보다 크고 failure가 없어야 한다. 실제 실행
결과를 확인한 뒤에만 다음 단계로 간다. 업데이트 전에는 미션을 종료하고 모터
전원을 차단한 뒤 `git pull --ff-only` 또는 검증된 release checkout 후 다시
빌드한다.

### Jetson ML runtime

`rosdep`은 Ultralytics와 Jetson용 PyTorch/TensorRT를 설치하지 않는다. CUDA,
cuDNN, TensorRT와 PyTorch는 설치된 JetPack/L4T에 맞는 NVIDIA 배포본을 사용하고,
그 위에 검증된 Ultralytics 버전을 고정한다. x86용 일반 PyPI wheel이나 임의의
`pip install --upgrade`로 Jetson system CUDA stack을 덮어쓰지 않는다.

2026-08-26 현장 Jetson import baseline은 다음과 같다. 장비 image를 바꾸면 이
조합을 그대로 가정하지 말고 NVIDIA 호환표에 맞춰 새 조합을 검증·기록한다.

| 모듈 | 검증 baseline |
|---|---:|
| Python | 3.10 |
| PyTorch | 2.8.0 |
| Ultralytics | 8.4.116 |
| TensorRT | 10.3.0 |
| OpenCV | 4.5.4 (`cv2.aruco` 포함) |
| NumPy | 1.26.4 |

설치 뒤 runtime shell에서 실제 import와 preflight를 확인한다. build script 안의
`PYTHONNOUSERSITE=1`은 build/test subprocess에만 적용되며, ML package를 user-site에
설치한 runtime shell에 전역 export하지 않는다.

```bash
python3 -c 'import torch, ultralytics, tensorrt, cv2, numpy; print(torch.__version__, ultralytics.__version__, tensorrt.__version__, cv2.__version__, numpy.__version__)'
```

각 장비의 shell 또는 service에 다음 환경을 동일하게 적용한다.

```bash
source /opt/ros/humble/setup.bash
source ~/parkingbot_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 2. STM32 플래시와 전기 안전

authoritative source는 `stm32/parking_robot` 하나지만 flash image는 로봇별로
다르다. 서보 OPEN/GRIP/MIN/MAX와 RL/RR encoder timer mapping이 compile-time
profile에 들어 있으므로 같은 `.bin`을 양쪽 보드에 사용하지 않는다. 저장소
루트에서 두 production image를 명시적으로 빌드한다.

```bash
export ARM_NONE_EABI_ROOT=/path/to/gcc-arm-none-eabi
tools/build_stm32_firmware.sh all
sha256sum stm32/parking_robot/build/production/artifacts/parking_robot_*.bin
```

- Front/robot-2 STM32: `parking_robot_front.bin`만 flash한다.
- Rear/robot-1 STM32: `parking_robot_rear.bin`만 flash한다.

빌드 script는 profile 없는 generic image를 만들지 않으며, source header도
`PARKING_ROBOT_PROFILE`이 없으면 compile을 거부한다. CubeIDE를 사용할 때도
Front configuration에는 `PARKING_ROBOT_PROFILE=1`, Rear configuration에는
`PARKING_ROBOT_PROFILE=2`를 명시하고 산출물 이름을 역할별로 분리한다. 어느
방식이든 flash 전 파일명, SHA256, 대상 로봇 label을 함께 대조한다.

메인 전원을 끈 상태로 각 보드에 해당 image를 flash한다. 로봇 공통 전원이
필요한 검증은 모든 바퀴를 견고하게 띄운 뒤 수행하고, UART 115200 8N1의
heartbeat/ACK, encoder와 ultrasonic frame을 먼저 확인한다.

펌웨어 소스의 현재 `ENCODER_PPR`은 `5182.0f`지만 운용값으로 고정해 믿지 않는다.
로봇별 출력축 1회전 count를 측정하고 ROS `encoder_ppr`와 해당 보드 펌웨어 값을
같게 맞춘다. wheel radius, `lx`, `ly`, motor/encoder sign도 로봇별로 실측한다.

전원 인가 전 필수 조건:

- HC-SR04 ECHO 5 V를 STM32에 직접 연결하지 않고 3.3 V level shifter 또는
  검증된 저항분압을 사용한다.
- 현재 조립 상태는 단일 메인 전원이라 RPi/카메라와 motor rail을 독립적으로
  ON/OFF할 수 없다. 적정 fuse와 비상정지를 확인하고 정적 통전 시 전 바퀴를
  띄운다.
- RPi, STM32와 motor driver는 공통 GND를 사용한다. servo를 RPi 5 V pin에서
  공급하지 않는다.
- 물리 ESTOP이 motor power를 실제 차단하는지 시험한다. 소프트웨어 ESTOP은
  인증된 기능 안전 장치가 아니다.

STM32 watchdog은 command 무갱신 **250 ms**, heartbeat 단절 **300 ms**에 정지해야
한다. 두 조건을 모터를 든 상태에서 각각 시험한다.

## 3. 장치 경로와 runtime asset

STM32 serial은 재부팅에도 안정적인 `/dev/serial/by-id/...`를 사용한다.

```bash
ls -l /dev/serial/by-id/
ls -l /dev/v4l/by-path/
```

카메라는 현장 USB topology를 확인해 기록한 `/dev/v4l/by-path/...`를 우선한다.
현재 launch의 by-path 기본값은 보편적인 cam0/cam2 매핑이 아니다. 숫자
`camera*_id`는 `camera*_device:=''`를 명시했을 때만 사용되며, 잘못된 by-path에서
숫자 ID로 자동 fallback하지 않는다. 숫자 ID를 임시로 사용하면 재부팅 뒤 영상과
marker 역할을 다시 확인한다. Rear 카메라는 가능하면 by-path를 지원하는 외부 ROS
camera driver로 열고 `enable_rear_camera:=false`로 연결한다.

현장 로봇 카메라 기준은 다음과 같다. 개발 PC에 보관한 원본은
`/home/guitest/ov2710_calib_23mm_*.npz`이고, 로봇에서 실행할 때는 각 로봇
사용자의 `$HOME`에 필요한 파일을 복사한다.

- Rear `robot-1`: 흰색 OV2710, 640x480,
  `$HOME/ov2710_calib_23mm_white.npz` 배포·존재 확인 완료
- Front `robot-2`: 검은색 OV2710, 640x480,
  원본 `ov2710_calib_23mm_black.npz` (현재 상대 pose 제어에는 미사용·미배포)

### Rear ID0 간편 정적시험

현재 전원은 motor rail만 따로 끌 수 없다. 공통 전원을 켜기 전에 robot-1을
견고한 받침대에 올려 네 바퀴를 모두 띄우고, 작업구역을 비우며 물리 ESTOP에
즉시 접근할 수 있게 한다. 운용 domain 42와 분리된 domain에서 다음
perception-only launch만 실행한다. 이 launch에는 STM32 bridge, 상태기계,
motion controller와 `cmd_vel` publisher가 없고, 실시간 ArUco 진단 화면을
5005번 포트로 함께 제공한다.

```bash
source /opt/ros/humble/setup.bash
source "$HOME/cooperative_parking_robot_ws/install/setup.bash"
export ROS_DOMAIN_ID=142

ros2 launch cooperative_parking_robot rear_aruco_static_check.launch.py
```

robot-1 자체 브라우저에서는 `http://127.0.0.1:5005`, 같은 내부망의 노트북에서는
`http://ROBOT1_IP:5005`를 연다. 화면에는 640x480 영상과 FPS, 중심 십자선,
검출된 marker ID·테두리·픽셀 크기·찌그러짐 정도가 실시간으로 표시된다. 웹
상단의 `거리`는 두 로봇 진행축 방향 간격, `좌우`는 중심선 오차, `틀어짐`은
상대 yaw이며, 이 세 값은 tracker의 `/sync/relative_pose`에서 가져온다. ID0
테두리와 상단 상대 pose 배지가 안정적으로 초록색에 가깝게 유지되는지 먼저
확인한다. 픽셀 표의 `각도오차`는 마커 사각형의 영상 왜곡 지표이므로 로봇 간
`틀어짐`과는 다른 값이다.

흰색 카메라가 `/dev/video0`이 아니면 확인한 안정 경로를 지정한다.

```bash
ros2 launch cooperative_parking_robot rear_aruco_static_check.launch.py \
  camera_device:=/dev/v4l/by-path/REPLACE_WITH_WHITE_CAMERA-video-index0
```

다른 터미널에도 같은 `ROS_DOMAIN_ID=142`를 적용하고 다음만 확인한다.

```bash
ros2 node list
ros2 topic list | grep cmd_vel
ros2 topic echo /sync/marker_visible
ros2 topic echo --once /sync/relative_pose
```

정상 상태에서는 실행 노드가 카메라, tracker와 진단 preview뿐이고 `cmd_vel`
검색 결과가 없어야 한다. 50 mm ID0을 카메라 정면 약 0.30 m에 두면
`marker_visible=true`,
`frame_id=rear_base`, `position.x`가 약 0.30 m로 나온다. 중앙·평행 배치에서는
`position.y`와 yaw가 0에 가까워야 한다. 값이 다르면 motion launch로 넘어가지
않고 영상, 실제 검은 정사각형 크기, 카메라 해상도와 장착 방향부터 확인한다.

### 두 로봇 10 cm 협동 직진 시험

Rear ID0 정적시험과 양쪽 STM32 바퀴 공중 방향시험이 모두 통과한 다음에는 전체
주차 상태기계 대신 전용 협동 직진 launch를 사용한다. robot-2 Front에서는
STM32 bridge만, robot-1 Rear에서는 흰색 카메라·ArUco·Rear bridge·시험
대시보드만 실행된다. `rigid_body_sync`, `individual_move`, state machine과
그리퍼 제어는 실행되지 않는다.

브라우저에서 `http://robot-1.local:5006/`을 열면 카메라 화면, ArUco 거리·좌우·
상대 각도, 양쪽 hardware/manual/odometry 상태, 현재 명령과 정지 이유를 한 번에
볼 수 있다. 기본 상태는 정지이며 `시험 준비` 후 `10 cm 시작`을 눌러야 움직인다.
간격·좌우 3 cm, 상대 각도 5도, 양쪽 이동거리 차이 3 cm, 센서 freshness와
4초 timeout을 넘으면 0속도를 유지한다. 구체적인 양쪽 명령과 공중→바닥 순서는
[두 로봇 10 cm 협동 직진 시험](../ros2/cooperative_parking_robot/docs/COOPERATIVE_DRIVE_TEST.md)을
따른다.

직진 시험을 통과한 뒤에는 별도
[키보드 ArUco 추종 시험](../ros2/cooperative_parking_robot/docs/KEYBOARD_FOLLOW_TEST.md)을
사용할 수 있다. 이 모드는 준비 순간의 ArUco forward 값을 차량 축간 거리의 정확한
목표로 저장하며, 로봇 길이나 명목 wheelbase를 더하지 않는다. 5006 직진 제어기와
5007 키보드 추종 제어기를 동시에 실행하지 않는다.

Jetson runtime asset은 `~/.ros/adaptive_valet_bot/`에 둔다.

```text
cctv0_camera_calibration.npz
cctv2_camera_calibration.npz
homography_cam0_rectified.npy
homography_cam2_rectified.npy
parking_layout.yaml
parking_registry.db
```

Homography와 등록 layout이 준비되기 전에는 motion을 허용하지 않는다. 생성과
검증 절차는 [pipeline](pipeline.md)을 따른다.

## 4. 분산 기동

### Production operation commands

긴 launch 명령을 매번 입력하지 않도록 중앙 운용 PC에 다음 명령을 설치한다.

```bash
cd /absolute/path/to/parkingbot
bash tools/install_robot_commands.sh
```

Installer는 shell startup file을 수정하지 않는다. 최초 한 번
`~/.config/parkingbot/production_hosts.env`에 검증된 SSH host, 세 장비와 운용
PC의 절대 colcon workspace 경로, stable device path 및 아래 launch에 필요한
실측값을 입력한다. 저장소에는 실제 SSH 주소가 없으며 Rear=`robot-1`,
Front=`robot-2` 역할만 확인되므로 빈 값을 추측해 채우지 않는다.
Rear가 외부 camera driver를 쓰면 현장에서 이미 검증한 정확한 실행 명령을
`REAR_EXTERNAL_CAMERA_COMMAND`에 넣는다. 명령을 알 수 없으면 start는 차단된다.

```bash
robot_doctor
robot_start
robot_state --watch
```

`robot_start`는 Jetson → Rear → Front 순서로 각 장비의
`parkingbot-production` tmux session을 만들고, 이 절의 기존 production launch
argument를 그대로 사용한다. PARK request는 자동 발행하지 않는다. 기동 후
Jetson의 `http://JETSON_HOST:5000/kiosk`에서 PARK를 승인한다.

```bash
robot_state
robot_state --json
robot_logs
robot_logs rear
robot_stop
```

`robot_stop`은 기존 `/emergency_stop`을 먼저 발행한 다음 Front → Rear → Jetson
session을 종료한다. STM32 ESTOP/FAULT latch를 해제하거나 Registry를 rollback하지
않는다. 따라서 출력의 `MANUAL RESET MAY BE REQUIRED`는 아래 FAULT 복구 계약을
따르라는 의미다.

로그는 각 장비의 `~/.ros/parkingbot_logs/<run-id>/<role>/`에 저장되고 중앙
운용 PC에는 state JSONL과 incident snapshot이 저장된다. Front/Rear motion
fault, sync fatal error, robot FAULT, hardware-ready loss 또는 tmux process exit가
새로 발생하면 `incidents/<timestamp>_<reason>/`에 상태, ROS graph와 세 장비
최근 로그 tail을 수집한다.

현재는 motor rail만 따로 끌 수 없으므로, 작업구역을 비우고 두 로봇의 모든
바퀴를 견고하게 띄운 상태에서 공통 전원을 인가한 뒤 Jetson → Rear → Front
순서로 기동한다. Production marker는 Front 상판 **ID2**, Rear 상판 **ID1**, Rear
카메라가 보는 Front 후면 상대 marker **ID0**이다. 실험용 ID2/ID3을 production
launch나 asset에 사용하지 않는다.

### Jetson

현장에서 검증한 절대 device path와 model path를 먼저 설정한다. 필수 변수가
비어 있으면 아래 guard가 launch 전에 중단시킨다.

```bash
: "${CAM0_DEVICE:?set CAM0_DEVICE to the site cam0 by-path}"
: "${CAM2_DEVICE:?set CAM2_DEVICE to the site cam2 by-path}"
: "${MODEL_PATH:?set MODEL_PATH to the validated vehicle model}"
: "${CAM0_GROUND_X_M:?set measured cam0 optical-axis ground X}"
: "${CAM0_GROUND_Y_M:?set measured cam0 optical-axis ground Y}"
: "${CAM0_HEIGHT_M:?set measured cam0 height}"
: "${CAM2_GROUND_X_M:?set measured cam2 optical-axis ground X}"
: "${CAM2_GROUND_Y_M:?set measured cam2 optical-axis ground Y}"
: "${CAM2_HEIGHT_M:?set measured cam2 height}"
: "${FRONT_MARKER_HEIGHT_M:?set measured Front marker height}"
: "${REAR_MARKER_HEIGHT_M:?set measured Rear marker height}"
RUNTIME_DIR="${HOME}/.ros/adaptive_valet_bot"

ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  camera0_device:="${CAM0_DEVICE}" \
  camera2_device:="${CAM2_DEVICE}" \
  cctv0_camera_calib:="${RUNTIME_DIR}/cctv0_camera_calibration.npz" \
  cctv2_camera_calib:="${RUNTIME_DIR}/cctv2_camera_calibration.npz" \
  homography_cam0_file:="${RUNTIME_DIR}/homography_cam0_rectified.npy" \
  homography_cam2_file:="${RUNTIME_DIR}/homography_cam2_rectified.npy" \
  layout_config:="${RUNTIME_DIR}/parking_layout.yaml" \
  model_path:="${MODEL_PATH}" \
  parking_registry_db_path:="${RUNTIME_DIR}/parking_registry.db" \
  cam0_ground_x_m:="${CAM0_GROUND_X_M}" \
  cam0_ground_y_m:="${CAM0_GROUND_Y_M}" \
  cam0_height_m:="${CAM0_HEIGHT_M}" \
  cam2_ground_x_m:="${CAM2_GROUND_X_M}" \
  cam2_ground_y_m:="${CAM2_GROUND_Y_M}" \
  cam2_height_m:="${CAM2_HEIGHT_M}" \
  camera_ground_points:="[${CAM0_GROUND_X_M}, ${CAM0_GROUND_Y_M}, ${CAM2_GROUND_X_M}, ${CAM2_GROUND_Y_M}]" \
  front_marker_height_m:="${FRONT_MARKER_HEIGHT_M}" \
  rear_marker_height_m:="${REAR_MARKER_HEIGHT_M}" \
  enable_operator_ui:=true \
  enable_debug_overlay:=false \
  simultaneous_entry:=false \
  require_all_cameras:=true \
  require_exact_camera_resolution:=true
```

두 camera 중 하나가 timeout이면 target/empty/map이 즉시 fail-closed하고,
live coverage 밖 map cell은 `unknown(-1)`이다. 부분 시야 운용을 위해
`require_all_cameras:=false`로 낮추는 것은 현장 위험성 검토 없이 허용하지 않는다.
Cam2를 debug 화면으로 선택하면 `debug_image_topic:=/cctv2/image_rect`와
`debug_camera_calib:="${RUNTIME_DIR}/cctv2_camera_calibration.npz"`를 함께 바꾼다.

### Rear Raspberry Pi

아래 예시는 외부 camera driver가 `/rear/marker_camera/image`를 발행하는 권장
구성이다. 대문자 변수에는 해당 장비에서 검증한 값만 넣는다.

```bash
: "${REAR_SERIAL:?set stable Rear STM32 by-id}"
: "${WHEELBASE:?set measured vehicle wheelbase}"
: "${REAR_WHEEL_RADIUS:?set measured Rear wheel radius}"
: "${REAR_ENCODER_PPR:?set measured Rear encoder PPR}"
: "${REAR_LX:?set measured Rear lx}"
: "${REAR_LY:?set measured Rear ly}"
: "${REAR_LEFT_SENSOR_X:?set measured left sensor offset}"
: "${REAR_RIGHT_SENSOR_X:?set measured right sensor offset}"
REAR_CALIB="${REAR_CALIB:-${HOME}/.ros/adaptive_valet_bot/rear_camera_calibration.npz}"

ros2 launch cooperative_parking_robot rear_robot.launch.py \
  serial_port:="${REAR_SERIAL}" \
  enable_serial:=true require_serial:=true \
  require_hardware_ready:=true require_ultrasonic_for_ready:=true \
  enable_rear_camera:=false \
  rear_camera_topic:=/rear/marker_camera/image \
  camera_calib:="${REAR_CALIB}" \
  wheelbase:="${WHEELBASE}" \
  wheel_radius:="${REAR_WHEEL_RADIUS}" \
  encoder_ppr:="${REAR_ENCODER_PPR}" \
  lx:="${REAR_LX}" ly:="${REAR_LY}" \
  left_sensor_to_gripper_x_m:="${REAR_LEFT_SENSOR_X}" \
  right_sensor_to_gripper_x_m:="${REAR_RIGHT_SENSOR_X}" \
  simultaneous_entry:=false
```

외부 driver가 없을 때만 내장 OpenCV camera와 숫자 `rear_camera_id`를 fallback으로
사용하고, 그 부팅에서 실제 영상을 확인한다.

### Front Raspberry Pi

```bash
: "${FRONT_SERIAL:?set stable Front STM32 by-id}"
: "${WHEELBASE:?set measured vehicle wheelbase}"
: "${FRONT_WHEEL_RADIUS:?set measured Front wheel radius}"
: "${FRONT_ENCODER_PPR:?set measured Front encoder PPR}"
: "${FRONT_LX:?set measured Front lx}"
: "${FRONT_LY:?set measured Front ly}"
: "${FRONT_LEFT_SENSOR_X:?set measured left sensor offset}"
: "${FRONT_RIGHT_SENSOR_X:?set measured right sensor offset}"

ros2 launch cooperative_parking_robot front_robot.launch.py \
  serial_port:="${FRONT_SERIAL}" \
  enable_serial:=true require_serial:=true \
  require_hardware_ready:=true require_ultrasonic_for_ready:=true \
  wheelbase:="${WHEELBASE}" \
  wheel_radius:="${FRONT_WHEEL_RADIUS}" \
  encoder_ppr:="${FRONT_ENCODER_PPR}" \
  lx:="${FRONT_LX}" ly:="${FRONT_LY}" \
  left_sensor_to_gripper_x_m:="${FRONT_LEFT_SENSOR_X}" \
  right_sensor_to_gripper_x_m:="${FRONT_RIGHT_SENSOR_X}" \
  use_aruco_distance:=true \
  simultaneous_entry:=false
```

ID0 중심거리 offset은 `config/id0_calibration.yaml`에서 중앙 관리한다.
현재 `0.570m` 값은 2026-08-27의 정렬 실측(중심간 0.785m, raw 약 0.215m)에
근거하므로 production은 `use_aruco_distance=true`를 사용한다. 카메라 또는 ID0
장착을 변경했다면 이 파일을 재측정하기 전까지 X correction을 비활성화하고,
mission reference가 wheel X를 사용한다는 점을 확인한다.

### 분산 장비 시계 동기화 확인

Jetson, Front RPi, Rear RPi 세 장비에서 모두 다음을 확인한다.

```bash
timedatectl
chronyc tracking
chronyc sources -v
```

`command_future_tolerance_s=0.10`, `command_source_timeout_s=0.25`보다 충분히
작은 여유를 확보하기 위해 장비 간 절대 skew 목표는 20ms 이하로 둔다.
`hardware_status`의 `ERR,CLOCK_SKEW`가 한 번이라도 발생하면 실제 미션을
시작하지 말고 NTP/chrony source와 네트워크를 복구한다.

### FAULT 복구 계약

FAULT는 자동 clear하지 않는다. 현재 STM32 firmware의 ESTOP은 전원/MCU reset까지
latched되므로 ROS state만 임의로 IDLE로 바꾸는 것은 금지한다.

| 발생 단계 | 허용되는 조치 |
|---|---|
| PARK 승인 전 | 원인 제거 후 노드를 재기동한다. Registry 변경은 없다. |
| WAIT_LIFT / 실제 Lift 전 | 차량이 바닥에 있고 두 로봇이 정지했음을 확인한 뒤 전체 mission process와 두 STM32를 재기동한다. |
| Lift 후 / NAVIGATING | 자동 reset·Registry rollback 금지. 차량을 안전 지지하고 수동 회수한 뒤 operator가 현장 상태와 DB를 대조한다. |
| Release 후 | 슬롯의 실제 차량 유무를 확인하기 전 EMPTY/OCCUPIED를 자동 변경하지 않는다. |

복구 전에는 `/robot/lifted`, 양쪽 `/robot_state`, `/hardware_status`, 실제 바퀴
정지, 그리퍼/차량 지지 상태를 함께 확인한다. ESTOP latch를 해제하는 firmware
protocol이 아직 없으므로 이 절차는 P0 운영 제약이며 단순 UI reset 버튼으로
대체할 수 없다.

## 5. 기동 확인과 모터 인가

```bash
ros2 topic echo /front/hardware_ready
ros2 topic echo /rear/hardware_ready
ros2 topic hz /front/wheel_odom
ros2 topic hz /rear/wheel_odom
ros2 topic hz /front/ultrasonic_left
ros2 topic hz /rear/ultrasonic_left
ros2 topic echo /front/localization_status
ros2 topic echo /rear/localization_status
ros2 topic echo /cctv/merge_status
ros2 topic echo /fleet/state
ros2 topic info /parking/map --verbose
```

양쪽 `hardware_ready=true`, fresh sensor/localization, 단일 `/parking/map`
publisher, 올바른 marker 역할, 등록된 layout, fault 없는 Fleet/UI를 확인한 뒤에만
로봇을 받침대에서 내려 바닥 시험으로 넘어간다. 공통 전원이 이미 인가된 동안에는
받침대를 제거하지 않는다. 단계별 시험 gate를 건너뛰지 않는다.

## 6. 7인치 UI와 Registry

Jetson에서 `http://127.0.0.1:5000/kiosk`를 연다.

```bash
chromium-browser --kiosk --noerrdialogs http://127.0.0.1:5000/kiosk
```

- 입차: 차량번호, 4~64자 비밀번호와 `EMPTY` slot을 제출하고 Fleet 승인과 양쪽
  HOME 완료 후 `OCCUPIED` 전환을 확인한다.
- 출차: 같은 차량번호와 비밀번호를 제출한다. Fleet가 source slot을 찾아
  구현된 retrieve 흐름을 실행하며 양쪽 HOME 후 `EMPTY` 전환을 확인한다.
- UI는 요청만 제출하며 pose, mission ID, source slot과 승인 여부는 Fleet가
  소유한다.

비밀번호 원문은 DB나 로그에 저장하지 않는다. HTTP와 ROS String transport는
암호화되지 않으므로 UI는 trusted LAN에서만 사용한다. SQLite Registry는 동일
layout의 안정 `EMPTY/OCCUPIED` 상태만 복원한다. `RESERVED`, `EXIT_RESERVED`,
`EXITING`, 손상 DB, schema/layout 불일치에서는 **fail-closed**로 시작을
차단해야 한다.

## 7. 정지와 복구

위험 시 웹 버튼보다 **물리 ESTOP과 motor power 차단을 먼저** 사용한다.

```bash
ros2 topic pub --once /emergency_stop std_msgs/msg/Bool "{data: true}"
```

복구 순서:

1. 물리 motor power를 차단하고 사람, 차량과 기구 상태를 확인한다.
2. 원인과 걸림을 제거하고 ROS command가 0인지 확인한다.
3. STM32 ESTOP latch는 원인 제거 후 보드 전원 재인가로 해제한다.
4. 모든 preflight, `hardware_ready`, sensor/localization/Fleet 상태를 다시 확인한다.
5. 실패한 단계부터 보호 지그와 감독 아래 재시험한다.

Registry startup이 차단되면 SQL로 상태를 임의 수정하지 않는다. DB를 보존하고
실제 차량/slot 상태를 확인한다. 주차장을 완전히 비운 경우에만 새 DB를 만들며,
차량이 남아 있으면 수동 제거와 재등록 없이 미션을 재개하지 않는다. 출차는
forward로 주차한 차량만 지원하며, 경로가 막히면 거부하고 우회하거나 주행 중
동적으로 재계획하지 않는다.
