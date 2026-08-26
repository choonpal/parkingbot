# Calibration·Homography·preflight pipeline

이 문서는 실차 motion 전에 필요한 측정, camera calibration, rectified
Homography, layout 등록과 preflight만 다룬다. 배포·launch·UI·복구는
[실차 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), 최종 GO/NO-GO는
[실차 준비도](REAL_WORLD_READINESS.md)를 따른다. 전체 현재 문서는
[문서 안내](README.md)에서 찾는다.

## 1. 입력과 산출물

최종 asset은 Jetson의 `~/.ros/adaptive_valet_bot/`에 둔다.

```text
cctv0_camera_calibration.npz
cctv2_camera_calibration.npz
homography_cam0_rectified.npy
homography_cam2_rectified.npy
parking_layout.yaml
parking_registry.db
```

추가로 Rear RPi의 `rear_camera_calibration.npz`, 검증된 vehicle segmentation
model과 다음 실측값이 필요하다.

- 로봇별 wheel radius, 출력축 encoder PPR, `lx`, `ly`, motor/encoder sign
- 차량 wheelbase/length/width와 gripper를 포함한 robot 외곽
- 좌우 ultrasonic-to-gripper offset, lateral sign, threshold와 hysteresis
- marker 실측 크기, base offset, 부착 yaw, camera pose

펌웨어 소스의 `ENCODER_PPR=5182.0f`는 현재 값이지 모든 로봇의 보증값이 아니다.
출력축을 직접 측정해 로봇별 ROS와 firmware 값을 일치시킨다. Production marker는
Front 상판 **ID2**, Rear 상판 **ID1**, Rear가 보는 Front 후면 **ID0**이다.
Rear 단독 실험의 ID2/ID3을 production asset에 넣지 않는다.

## 2. 카메라 device 고정

카메라를 분리한 뒤 재연결·재부팅하고 다음 경로가 같은 물리 카메라를 가리키는지
확인한다.

```bash
ls -l /dev/v4l/by-path/
v4l2-ctl --list-devices
```

현장 USB topology에 맞는 `/dev/v4l/by-path/...`를 cam0/cam2/Rear 역할별로
기록한다. source의 site path를 다른 장비의 보편적 mapping으로 복사하지 않는다.
숫자 camera ID는 `camera*_device:=''`를 명시했을 때만 선택되며 잘못된 path에서
자동 fallback하지 않는다. 숫자 ID를 임시 사용하면 영상과 marker 역할을 매번
확인한다.

## 3. Intrinsic calibration

실제 운용 해상도와 focus를 고정한 뒤 카메라마다 별도로 보정한다. 보정 뒤
해상도, lens, focus 또는 설치 카메라를 바꾸면 intrinsic과 Homography를 모두
다시 만든다.

Rear camera 예:

```bash
mkdir -p "${HOME}/.ros/adaptive_valet_bot"
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p image_topic:=/rear/marker_camera/image \
  -p output_path:="${HOME}/.ros/adaptive_valet_bot/rear_camera_calibration.npz" \
  -p board_cols:=9 -p board_rows:=6 -p square_size_m:=0.025
```

각 CCTV도 해당 raw topic과 서로 다른 output path로 같은 도구를 실행한다.
Checkerboard를 중앙·모서리·기울임 자세로 충분히 수집하고 도구의 품질 gate를
통과한 결과만 사용한다. NPZ를 load해 frame size, camera matrix와 distortion
coefficients가 현재 영상과 일치하는지 확인한다.

## 4. Rectified Homography와 layout

Homography는 반드시 intrinsic으로 보정된 rectified 영상에서 만든다.

```text
camera raw → intrinsic rectification → rectified image → pixel-to-map Homography
```

원칙:

- 바닥 기준점은 줄자로 측정하고 한 직선이 아닌 화면 전체에 분산한다.
- 두 CCTV의 같은 물리점에는 같은 map `(X,Y)` metre 값을 준다.
- overlap에 공통 기준점을 두고 카메라 간 정합을 확인한다.
- slot, corridor, waiting zone, 고정 장애물 no-go가 같은 map frame을 사용한다.
- 도구가 metre를 출력하면 `homography_scale_to_m=1.0`을 사용한다. 과거 cm
  asset만 `0.01`이며 단위를 추측하지 않는다.
- 각 camera RMS가 0.02 m 이상이면 재등록한다.

두 카메라 Homography 등록은
[dual tile Homography 도구](../dual_tile_homography_tool/README.md)를 사용한다.
문서 속 예전 숫자 device 예시보다 현장 by-path를 우선한다.

```bash
: "${CAM0_DEVICE:?set CAM0_DEVICE to the verified site by-path}"
: "${CAM2_DEVICE:?set CAM2_DEVICE to the verified site by-path}"
cd dual_tile_homography_tool
./run_dual.sh --cam0 "${CAM0_DEVICE}" --cam2 "${CAM2_DEVICE}"
```

layout 등록은 by-path를 사용하는 외부 camera driver가 raw topic을 발행하는
상태에서 패키지가 카메라를 중복으로 열지 않게 실행한다.

```bash
RUNTIME_DIR="${HOME}/.ros/adaptive_valet_bot"
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=false \
  camera_label:=cam0 \
  cctv_raw_topic:=/cctv0/image_raw \
  cctv_rect_topic:=/cctv0/image_rect \
  cctv_camera_calib:="${RUNTIME_DIR}/cctv0_camera_calibration.npz" \
  homography_output_file:="${RUNTIME_DIR}/homography_cam0_rectified.npy" \
  layout_output_file:="${RUNTIME_DIR}/parking_layout.yaml"
```

cam2는 같은 map 원점과 물리 좌표를 사용하고
`append_existing_layout:=true`로 실행한다.

```bash
RUNTIME_DIR="${HOME}/.ros/adaptive_valet_bot"
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=false \
  camera_label:=cam2 \
  cctv_raw_topic:=/cctv2/image_raw \
  cctv_rect_topic:=/cctv2/image_rect \
  cctv_camera_calib:="${RUNTIME_DIR}/cctv2_camera_calibration.npz" \
  homography_output_file:="${RUNTIME_DIR}/homography_cam2_rectified.npy" \
  layout_output_file:="${RUNTIME_DIR}/parking_layout.yaml" \
  append_existing_layout:=true
```

저장 후 `layout_registered=true`, 모든 slot polygon, approach corridor, waiting
zone, no-go와 camera coverage를 검토한다. Homography/layout 등록이 끝나기 전에는
모터를 켜지 않는다.

## 5. 정적 검증

두 카메라 overlap에 같은 차량/marker를 놓고 다음을 확인한다.

- 같은 물체가 두 장애물로 남지 않음
- `/cctv/merge_status`의 multi-camera detection과 duplicate removal이 동작함
- `/parking/map` publisher가 merge node 하나뿐임
- 한 camera를 가렸을 때 관측하지 못한 slot을 빈자리로 만들지 않음
- Front ID2와 Rear ID1 absolute pose가 실제 base 위치·yaw에 맞음
- ID0 상대 yaw가 맞고, distance offset 실측 전에는
  `use_aruco_distance=false`임

Vehicle mask의 외곽, 중심, yaw와 footprint를 여러 위치에서 반복 측정한다. 벽,
기둥 등 model이 보장하지 않는 장애물은 layout no-go로 등록한다.

## 6. Hardware preflight

환경을 source하고 실제 절대 path를 변수에 지정한 뒤 role별로 실행한다.

```bash
RUNTIME_DIR="${HOME}/.ros/adaptive_valet_bot"
: "${MODEL_PATH:?set MODEL_PATH to the validated vehicle model}"

ros2 run cooperative_parking_robot hardware_preflight --role jetson \
  --dual-cctv \
  --cctv-camera-calib "${RUNTIME_DIR}/cctv0_camera_calibration.npz" \
  --cctv2-camera-calib "${RUNTIME_DIR}/cctv2_camera_calibration.npz" \
  --model-path "${MODEL_PATH}" \
  --model-mode vehicle_seg \
  --homography-file "${RUNTIME_DIR}/homography_cam0_rectified.npy" \
  --homography2-file "${RUNTIME_DIR}/homography_cam2_rectified.npy"
```

```bash
: "${REAR_SERIAL:?set REAR_SERIAL to the stable STM32 by-id}"
REAR_CALIB="${HOME}/.ros/adaptive_valet_bot/rear_camera_calibration.npz"
ros2 run cooperative_parking_robot hardware_preflight --role rear \
  --serial-port "${REAR_SERIAL}" \
  --rear-camera-calib "${REAR_CALIB}"
```

```bash
: "${FRONT_SERIAL:?set FRONT_SERIAL to the stable STM32 by-id}"
ros2 run cooperative_parking_robot hardware_preflight \
  --role front --serial-port "${FRONT_SERIAL}"
```

Preflight 실패를 launch flag로 우회하지 않는다. dependency, model, calibration,
Homography, serial path/permission 또는 환경을 수정하고 다시 실행한다. Preflight
통과는 motion 허가가 아니며 [실차 준비도](REAL_WORLD_READINESS.md)의 단계 gate를
계속 적용한다.
