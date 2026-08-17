# ROS 2 Humble 배포·실행 가이드

## 1. 기준 환경

| 장비 | 기준 OS/ROS | 역할 |
|---|---|---|
| Jetson Orin Nano | JetPack 6.x Ubuntu 22.04 + ROS 2 Humble | CCTV 보정, YOLO/BEV, 상판 마커, A*, fleet manager |
| Front Raspberry Pi 4 | Ubuntu 22.04 arm64 + ROS 2 Humble | 강체 Master, STM32 bridge, 초음파 에지 판단, pose fusion |
| Rear Raspberry Pi 4 | Ubuntu 22.04 arm64 + ROS 2 Humble | Rear ArUco, STM32 bridge, 초음파 에지 판단, pose fusion |

```bash
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

세 장비의 시계는 NTP/chrony로 동기화한다.

## 2. 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/cooperative_parking_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
colcon test --packages-select cooperative_parking_robot
colcon test-result --verbose
```

## 3. 카메라 좌표계

천장 카메라는 다음 순서로만 사용한다.

```text
/cctv/image_raw
  -> cctv_rectify_node
  -> /cctv/image_rect
       -> yolo_bev_map_node
       -> cctv_robot_marker_node
```

패키지에 `config/cctv_camera_calibration.npz`가 포함되어 있다. 이 파일은 천장 카메라용이며 Rear 카메라에 사용하지 않는다.

Homography는 반드시 `/cctv/image_rect`에서 다시 생성한 `homography_rectified.npy`를 사용한다. Raw 영상에서 만든 Homography와 섞지 않는다.

## 4. 별도 준비 항목

| 항목 | 장비 | 상태 |
|---|---|---|
| `cctv_camera_calibration.npz` | Jetson | 패키지 포함; 동일 카메라·해상도 확인 필요 |
| `homography_rectified.npy` | Jetson | `bev_layout_calibration.launch.py`로 생성 필수 |
| `vehicle_seg.engine`(권장) 또는 `yolov8n.pt` | Jetson | 차량 Seg 모델은 별도 준비; COCO 모델은 box 폴백 |
| `rear_camera_calibration.npz` | Rear RPi | 별도 생성 필수 |
| 천장/Rear Image publisher | Jetson/Rear | 별도 준비 필수 |
| STM32CubeMX/HAL 완성 프로젝트 | 각 STM32 | 별도 준비 필수 |

## 5. 사전 점검

Jetson:

```bash
hardware_preflight --role jetson \
  --model-path yolov8n.pt \
  --model-mode coco \
  --allow-model-download \
  --homography-file /absolute/homography_rectified.npy
```

외부 천장 calibration을 직접 지정하려면:

```bash
hardware_preflight --role jetson \
  --cctv-camera-calib /absolute/calibration_data.npz \
  --model-path /absolute/parking_seg.engine \
  --model-mode parking_seg \
  --homography-file /absolute/homography_rectified.npy
```

차량 mask+고정 주차면 방식(권장):

```bash
hardware_preflight --role jetson \
  --cctv-camera-calib /absolute/calibration_data.npz \
  --model-path /absolute/vehicle_seg.engine \
  --model-mode vehicle_seg \
  --homography-file /absolute/homography_rectified.npy
```

Rear:

```bash
hardware_preflight --role rear \
  --serial-port /dev/serial/by-id/<rear-stm32> \
  --rear-camera-calib /absolute/rear_camera_calibration.npz
```

Rear 카메라 없이 UART/STM32 초음파만 시험할 때는 `--disable-rear-aruco`와 `enable_aruco_tracker:=false`를 함께 사용한다.

## 6. 분산 실행

### Rear RPi

```bash
ros2 launch cooperative_parking_robot rear_robot.launch.py \
  rear_camera_topic:=/camera/image_raw \
  camera_calib:=/absolute/rear_camera_calibration.npz \
  serial_port:=/dev/serial/by-id/<rear-stm32> \
  wheelbase:=0.70 wheel_radius:=0.05 encoder_ppr:=2600 \
  lx:=0.10 ly:=0.10 \
  left_sensor_to_gripper_x_m:=<실측값> \
  right_sensor_to_gripper_x_m:=<실측값>
```

### Front RPi

```bash
ros2 launch cooperative_parking_robot front_robot.launch.py \
  serial_port:=/dev/serial/by-id/<front-stm32> \
  wheelbase:=0.70 wheel_radius:=0.05 encoder_ppr:=2600 \
  lx:=0.10 ly:=0.10 \
  left_sensor_to_gripper_x_m:=<실측값> \
  right_sensor_to_gripper_x_m:=<실측값> \
  aruco_distance_offset_m:=0.565 use_aruco_distance:=true
```

### Jetson

캘리브레이션과 실시간 영상 해상도가 같을 때:

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_opencv_camera:=false \
  cctv_raw_topic:=/camera/image_raw \
  model_path:=yolov8n.pt \
  model_mode:=coco \
  allow_model_download:=true \
  homography_file:=/absolute/homography_rectified.npy \
  homography_scale_to_m:=1.0 \
  fixed_wheelbase_m:=0.70 \
  front_marker_id:=10 rear_marker_id:=11
```

캘리브레이션 해상도를 확인했다면 명시한다.

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_opencv_camera:=true \
  camera_id:=0 \
  calibration_width_px:=1280 \
  calibration_height_px:=720 \
  model_path:=yolov8n.pt \
  model_mode:=coco \
  allow_model_download:=true \
  homography_file:=/absolute/homography_rectified.npy \
  homography_scale_to_m:=1.0
```

`bev_layout_calibrator`가 만든 H는 metre를 직접 출력하므로 `1.0`이다.
과거 cm 단위로 만든 H를 그대로 재사용할 때만 `0.01`을 쓴다.

1280×720은 파일 자체로 확정된 값이 아니므로 실제 생성 조건을 확인한 뒤 넣는다.

선택 웹 모니터는 `enable_debug_web:=true`로 켠다. Mission YOLO와 중복 추론을 피하려면 `debug_enable_yolo:=false`를 유지한다.

## 6-1. 초음파 UART/ROS 확인

STM32는 다음 프레임을 교대로 보낸다.

```text
U,L,83
U,R,86
U,L,TIMEOUT
```

RPi에서 확인한다.

```bash
ros2 topic hz /front/ultrasonic_left
ros2 topic hz /front/ultrasonic_right
ros2 topic echo /front/ultrasonic_status
```

정상 목표는 각 센서 약 14Hz이며, `stm32_bridge_node`가 좌우 프레임을
0.5초 이상 받지 못하면 `/front|rear/hardware_ready=false`로 전환한다.

## 7. 토픽 확인

```bash
ros2 topic hz /cctv/image_raw
ros2 topic hz /cctv/image_rect
ros2 topic info -v /cctv/image_rect
```

Jetson 핵심 토픽:

```text
/cctv/image_raw
/cctv/image_rect
/parking/map
/parking/target_pose
/parking/empty_slots
/virtual_robot/waypoints
```

## 8. 한-PC smoke 모드

```bash
ros2 launch cooperative_parking_robot full_system.launch.py
```

기본값은 `enable_opencv_camera=false`, `enable_cctv_rectify=false`, `enable_vision=false`, `enable_debug_web=false`, `enable_serial=false`다. 실제 영상까지 확인하려면 Rectifier와 Vision을 함께 활성화하고 모델·Homography 경로를 제공한다.

## 9. 실행 가능성 판정

- 천장 calibration의 파일 구조와 OpenCV 입력 형식은 유효하다.
- Raw→Rectified→YOLO/Homography 연결은 코드에 반영했다.
- 실제 카메라 해상도와 calibration 생성 해상도의 일치 여부는 아직 확인되지 않았다.
- `homography_rectified.npy`가 없으므로 실제 world 좌표 변환은 아직 실행할 수 없다.
- Rear 카메라 calibration은 별도로 필요하다.
- 무인 실차 주차는 STM32 완성 프로젝트와 하중 시험 전까지 미검증이다.
