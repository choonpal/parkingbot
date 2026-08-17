# 협동 주차 로봇 — ROS 2 Humble v1.10

천장 CCTV가 차량·빈 주차칸·Front/Rear 로봇의 위치를 인식하고, 두 메카넘 로봇이 고정 휠베이스 모형차를 들어 이동시키는 ROS 2 Humble 패키지다.

기준 환경:

- Jetson Orin Nano: Ubuntu 22.04 기반 JetPack 6.x + ROS 2 Humble
- Raspberry Pi 4 ×2: Ubuntu 22.04 arm64 + ROS 2 Humble
- Python 3.10
- 시연 범위: 모형차 한 대, 고정 휠베이스, 정해진 슬롯, 1회 작업 사이클

## Jetson 영상 파이프라인

```text
USB/CSI 카메라 또는 외부 ROS 카메라 드라이버
       ↓ /cctv/image_raw
cctv_rectify_node + cctv_camera_calibration.npz
       ↓ /cctv/image_rect
       ├─ yolo_bev_map_node
       │    ├─ 차량 검출
       │    ├─ 슬롯 점유 판정
       │    ├─ /parking/map
       │    ├─ /parking/target_pose
       │    └─ /parking/empty_slots
       ├─ cctv_robot_marker_node
       │    ├─ Front 상판 ArUco ID10
       │    ├─ Rear 상판 ArUco ID11
       │    └─ /front·rear/cctv_pose (각 로봇 전역 pose)
       └─ jetson_vision_web_node (선택)
            ├─ /cctv/debug/annotated
            └─ Flask MJPEG :5000
```

최초 설치 때는 별도 `bev_layout_calibration.launch.py`가 같은 rectified 영상을
구독해 브라우저 :5001에서 Homography·대기영역·주차면을 등록한다. 이때 YOLO와
Fleet Manager는 실행하지 않는다.

카메라를 여는 프로세스는 하나만 둔다. 기존 카메라 ROS 드라이버가 있으면 `enable_opencv_camera:=false`를 유지한다. 드라이버가 없을 때만 이 패키지의 `opencv_camera_node`를 켠다. 웹 모니터는 `/cctv/image_rect`를 구독할 뿐 카메라를 다시 열지 않는다.

## 카메라 캘리브레이션

`aruco_tracker_node`는 `rear_camera_calibration.npz`가 없으면 기동을 거부한다.
체커보드로 생성한다(내부 코너 개수 기준, 10x7칸 보드는 9x6).

```bash
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p image_topic:=/rear/marker_camera/image \
  -p output_path:=<workspace>/src/cooperative_parking_robot/config/rear_camera_calibration.npz \
  -p board_cols:=9 -p board_rows:=6 -p square_size_m:=0.025
```

보드를 화면 중앙·모서리, 정면·기울임으로 20장 수집하면 자동 계산 후 저장한다.
RMS 재투영 오차가 1 px을 넘으면 저장하지 않으므로 다시 촬영해야 한다.
저장 후 `colcon build`로 config를 install 경로에 반영한다.

CCTV용 `cctv_camera_calibration.npz`는 동봉돼 있고, Front 전면 카메라는 쓰지 않는다.
BEV Homography는 내부 파라미터와 별개이며 `bev_layout_calibration.launch.py`로 등록한다.

## 터치스크린 운용 UI (v1.10)

Jetson에 연결한 7인치 LCD로 입차를 승인한다. `jetson_vision_web_node`가
`/kiosk` 화면을 제공하고, 버튼은 `/ui/mission_request`만 발행한다. 임무 시작
판단은 `fleet_manager_node`가 하므로 UI 프로세스가 죽어도 로봇 거동은 변하지 않는다.

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_debug_web:=true debug_enable_yolo:=false enable_operator_ui:=true

chromium-browser --kiosk --incognito --noerrdialogs \
  http://localhost:5000/kiosk
xset s off; xset -dpms; xset s noblank
```

`require_ui_confirmation:=false`를 주면 v1.9처럼 차량 인식 즉시 자동 시작한다.
출차 버튼은 화면에 있으나 아직 동작하지 않는다(설계는 `docs/MASTER_PLAN.md` Part 5).

임무가 끝나면 Front 상태기계가 `/mission/complete`를 발행하고 Fleet과 YOLO가
latch를 풀어 다음 임무를 받는다. v1.9에서는 1회 사이클 후 재시작이 필요했다.

## 초음파 처리 구조

초음파 펄스 측정은 Raspberry Pi GPIO가 아니라 각 로봇의 STM32가 담당한다.

```text
HC-SR04 Left/Right
  → STM32 TIM9 1 MHz + GPIO EXTI
  → UART U,L|R,<distance_mm|TIMEOUT>
  → stm32_bridge_node
  → /front|rear/ultrasonic_left|right (sensor_msgs/Range)
  → ultrasonic_edge_node
  → /front|rear/axle_count, wheel_center_s, wheel_detected
  → /front|rear/wheel_lateral_offset, wheel_lateral_valid
```

STM32는 좌우 센서를 35 ms 간격으로 교대 트리거하고, RPi는 거리 필터·바퀴
진입/이탈 에지·축 중심 판단만 수행한다. 센서가 그리퍼 중심보다 앞/뒤에
설치되면 launch의 `left_sensor_to_gripper_x_m`,
`right_sensor_to_gripper_x_m`에 실측값을 넣는다. 자세한 CubeMX 설정과
시험 절차는 `docs/ULTRASONIC_STM32_INTEGRATION.md`를 본다.

기본값 `simultaneous_entry:=true`에서는 Front/Rear가 차량 뒤쪽 staging으로 함께
이동하고, 상대 ID0가 유효해진 뒤 각자 `PRE_ALIGN`에서 종방향 속도를 0으로
유지하며 횡오차와 yaw를 먼저 닫는다. 두 로봇 모두 `PREALIGNED` 장벽에 도착한
뒤 동시에 `SCAN_IN`하며,
Front는 두 번째 front axle, Rear는 첫 번째 rear axle에 정렬한다. 좌우 초음파가
동시에 바퀴를 보는 구간에서는 거리 차로 차량 기준 횡오차를 보정한다. 횡이탈이
지속되면 시작점으로 후퇴하고 에지 검출기를 초기화한 뒤 최대 2회 재시도한다.
동시 진입 중 한쪽이 후퇴하면 정렬 완료 전인 상대도 함께 후퇴해 로봇 간격을
유지한다. 한쪽이 이미 정렬 완료된 뒤 발생한 재시도 조건은 FAULT로 안전 정지한다.

축 중심의 최종 전후 제어권은 초음파에 있으며, Front 후면 ID0은 상대
lateral/yaw 유지, Rear 접근 감속, 최종 휠베이스 거리 검증에 사용한다.
`simultaneous_entry:=false`로 기존 Front 우선 순차 진입도 선택할 수 있다. 기본
복귀는 가까운 차량 끝으로 나뉘어 이탈하며, `same_direction_exit:=true`일 때는
peer barrier와 상대거리 속도 보정으로 같은 방향 동기 이탈을 수행한다.

## 전달받은 `yolo_and_aruco.py` 반영 사항

- 1280×720, 버퍼 1의 OpenCV 카메라 publisher 추가
- YOLO를 3프레임마다 처리하는 옵션 추가
- ArUco 면적 1000px 미만 오검출 제거
- `DICT_4X4_50` 지원
- 18cm 마커 PnP 거리와 축을 보여 주는 선택적 웹 모니터 추가
- Flask 클라이언트마다 카메라를 새로 여는 구조 제거
- Mission YOLO/ArUco와 웹 모니터의 역할 분리

웹 화면은 진단용이다. 경로계획에 필요한 ROS 토픽은 `yolo_bev_map_node`와 `cctv_robot_marker_node`가 발행한다.

## YOLO 모드 구분

### `model_mode:=vehicle_seg` — 권장

직접 학습한 YOLO11-Seg에서 차량 mask만 출력한다. `empty_slot` 클래스를 따로
학습하지 않고, 등록된 주차면과 차량 mask의 면적 겹침률로 점유 여부를 계산한다.
차량 mask에서는 중심·장축 Yaw·길이·폭도 계산해 `/parking/vehicle_spec`으로
전달한다.

### `model_mode:=coco`

기본 `yolov8n.pt`를 사용할 때의 모드다. COCO의 차량 클래스만 사용한다.

```text
2 car, 3 motorcycle, 5 bus, 7 truck
```

빈 주차칸은 YOLO 클래스가 아니라 설정된 `slot_coords`와 검출 차량 위치를 비교해 판단한다. 일반 COCO 모델에는 `empty_slot` 클래스가 없다.

### `model_mode:=parking_seg`

기존에 직접 학습한 `vehicle`/`empty_slot` 2클래스 모델의 호환 모드다.

```text
cls_vehicle:=0
cls_empty_slot:=1
```

학습 모델의 실제 클래스 순서와 반드시 일치해야 한다. 파일 이름으로 모델 종류를 추측하지 않는다.
차량 segmentation mask가 있으면 PCA와 EMA로 차량 중심축 yaw를 추정해
`/parking/target_pose` orientation에 싣는다. 기본 COCO detection 모델은 mask가
없으므로 yaw 0 폴백이며, 비껴 선 차량의 축 추정에는 segmentation 모델이 필요하다.

## 캘리브레이션과 Homography

포함된 `config/cctv_camera_calibration.npz`는 천장 카메라의 렌즈 왜곡 보정용이다. YOLO와 상판 ArUco는 모두 보정된 `/cctv/image_rect`를 사용한다.

Homography는 반드시 이 보정 영상 위에서 다시 만든다.

```text
/cctv/image_raw → lens rectification → /cctv/image_rect
                                      ↓
                              homography_rectified.npy
```

수동으로 Python 좌표를 고치지 말고 브라우저 등록 launch를 사용한다.

```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true
```

브라우저에서 `http://<JETSON-IP>:5001/`을 열고 바닥 기준점, 슬롯별 모서리
4개+통로점 1개, 차량 대기영역을 등록한다. 기본 출력은
`~/.ros/adaptive_valet_bot/` 아래의 Homography와 layout YAML이다. 자세한 절차는
`docs/BEV_SLOT_REGISTRATION_AND_PARKING.md`를 본다.

## 결합 footprint 기반 A*

일반 A* 구간에서는 yaw를 고정하고, Front·차량·Rear를 하나의 직사각형으로
계획한다. 슬롯 진입은 `외부 staging 이동 → 슬롯 Yaw로 제자리 회전 → 슬롯
축 직선 삽입`으로 나눈다. 로봇 1대의 실측 외곽은 차량 앞뒤 `0.565m`, 차량 좌우
`0.275m`로 설정돼 있다. 그리퍼/로봇 중심이 각 차량 축 중심과 일치한다는
전제에서 mission footprint는 다음과 같다.

```text
pair_half_length = (wheelbase + 0.565) / 2
length = 2 × max(vehicle_length/2, |centre_offset_x| + pair_half_length)
         + 2 × safety_margin
width  = 2 × max(vehicle_width/2, |centre_offset_y| + 0.275/2)
         + 2 × safety_margin
```

Fleet Manager는 `/parking/vehicle_spec`의 휠베이스를 받아 이 값을 mission별로
갱신하고, Front/Rear 최신 odometry의 중점을 실제 `base_virtual` 시작점으로
사용한다. A*는 이 직사각형으로 장애물을 팽창하며 미확인 셀과 맵 밖을 막고,
대각선 이동 시 양옆 직교 셀이 모두 비어 있어야 한다.

기본 휠베이스는 `0.70m`다. 길이 `0.565m`인 로봇 두 대가 각 축 중심에 있을 때
몸체 사이 `0.135m`를 남기며, 코드는 최소 `0.10m` 미만인 차량 제원을 거부한다.
`default_vehicle_length_m=0.90`, `default_vehicle_width_m=0.35`는 아직 모형차
외곽 실측 전 placeholder이므로 실제 차량 외곽으로 교체해야 한다.

브라우저 등록 도구가 만든 Homography는 metre를 직접 출력하므로
`homography_scale_to_m:=1.0`을 사용한다. 과거 cm 단위 H만 `0.01`이다.
등록·mission launch의 기본 경로는 둘 다
`~/.ros/adaptive_valet_bot/`이므로 정상 등록 후에는 경로 인자를 생략해도 된다.

- Homography 결과가 이미 m: `1.0`
- Homography 결과가 cm: `0.01`

이 값을 틀리면 모든 위치가 100배 어긋난다.

## 빌드

```bash
source /opt/ros/humble/setup.bash
cd ~/cooperative_parking_robot_ws
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
colcon test --packages-select cooperative_parking_robot
colcon test-result --verbose
```

## Jetson 사전 점검

COCO `.pt` 자동 다운로드를 허용하는 경우:

```bash
hardware_preflight --role jetson \
  --model-path yolov8n.pt \
  --model-mode coco \
  --allow-model-download \
  --homography-file /absolute/homography_rectified.npy
```

차량 mask 전용 YOLO11-Seg TensorRT 모델(권장):

```bash
hardware_preflight --role jetson \
  --model-path /absolute/vehicle_seg.engine \
  --model-mode vehicle_seg \
  --homography-file /absolute/homography_rectified.npy
```

기존 vehicle/empty_slot 2클래스 TensorRT 모델(하위호환):

```bash
hardware_preflight --role jetson \
  --model-path /absolute/parking_seg.engine \
  --model-mode parking_seg \
  --homography-file /absolute/homography_rectified.npy
```

웹 모니터도 사용할 경우 `--enable-debug-web`을 추가한다.

## Jetson 실행

### 이미 ROS 카메라 드라이버가 있는 경우

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_opencv_camera:=false \
  cctv_raw_topic:=/camera/image_raw \
  model_path:=yolov8n.pt \
  model_mode:=coco \
  allow_model_download:=true \
  homography_file:=/absolute/homography_rectified.npy \
  homography_scale_to_m:=1.0 \
  layout_config:=/home/<USER>/.ros/adaptive_valet_bot/parking_layout.yaml
```

### 이 패키지가 카메라를 직접 여는 경우

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_opencv_camera:=true \
  camera_id:=0 \
  camera_width_px:=1280 \
  camera_height_px:=720 \
  model_path:=yolov8n.pt \
  model_mode:=coco \
  allow_model_download:=true \
  homography_file:=/absolute/homography_rectified.npy \
  homography_scale_to_m:=1.0 \
  layout_config:=/home/<USER>/.ros/adaptive_valet_bot/parking_layout.yaml
```

### 웹 모니터

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_debug_web:=true \
  debug_enable_yolo:=false \
  debug_enable_aruco:=true \
  ...
```

브라우저에서 `http://<JETSON-IP>:5000/`을 연다. 인증 기능이 없으므로 신뢰할 수 있는 내부망에서만 사용한다.

`debug_enable_yolo:=true`로 설정하면 사용자 원본처럼 웹 화면에도 YOLO 박스를 그리지만, Mission YOLO와 별도로 한 번 더 추론하므로 Jetson GPU 사용량이 증가한다.

## ArUco 설정

- 천장 CCTV 마커: Front 상판 ID10, Rear 상판 ID11
- Rear 로봇 카메라가 보는 Front 후면 마커: ID0
- ID10/ID11은 천장 절대 pose, ID0은 로봇 간 상대 pose용이다.
- `min_marker_area_px:=1000`은 1280×720 실험값이므로 실제 설치 높이에서 조정한다.
- `marker_size_m:=0.18`은 웹 PnP 거리 표시용이며 인쇄한 검은 정사각형 변 길이와 같아야 한다.
- Mission 절대 위치는 PnP가 아니라 rectified Homography를 사용한다.

## 실행 전 아직 필요한 것

- 브라우저 등록 도구로 만든 `homography_rectified.npy`와 `parking_layout.yaml`
- 실제 천장 영상에서 모형차 `vehicle_seg` mask가 외곽을 안정적으로 따는지 확인
- 검출이 불안정하면 차량 전용 YOLO11-Seg 모델 학습 (`empty_slot` 불필요)
- Front ID10/Rear ID11의 실제 인식 면적·yaw offset·base 중심 offset 측정
- 두 로봇 축 정렬 상태에서 `aruco_distance_offset_m` 실측
- Rear 카메라 전용 calibration
- STM32CubeMX 프로젝트와 실측 wheel/PID/servo 상수

자세한 Jetson 통합 판단은 `docs/JETSON_VISION_INTEGRATION.md`를 본다.
