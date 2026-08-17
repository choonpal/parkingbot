# BEV 기준점·주차면 등록 및 회전 진입

이 문서는 천장 CCTV 설치 후 브라우저에서 Homography와 주차면을 등록하고,
차량 크기에 맞는 슬롯을 골라 실제 회전 진입시키는 절차를 설명한다.

## 1. 이번 구조에서 BEV가 하는 일

`yolo_bev_map_node`는 별도의 BEV 영상을 계속 만들지 않는다. 보정 영상의
YOLO mask 픽셀을 Homography로 `map` 좌표(m)에 직접 변환한다. 브라우저에서
보이는 BEV 이미지는 기준점과 슬롯이 맞는지 확인하는 미리보기다.

```text
/cctv/image_raw
  -> 렌즈 왜곡 보정
  -> /cctv/image_rect
  -> pixel-to-metre Homography
  -> 차량 mask·슬롯을 map 좌표에서 비교
```

HSV로 주차면을 매 프레임 찾지 않는다. CCTV와 주차선은 고정돼 있으므로 최초
설치 때 실제 슬롯을 한 번 등록하고, 이후에는 차량 mask가 각 슬롯과 겹치는
면적만 계산한다.

## 2. 현장에서 먼저 측정할 값

- 바닥 기준점 최소 4개, 권장 6~12개의 실제 `(X,Y)m`
- 각 주차면 안쪽 네 모서리
- 각 주차면에서 차량이 들어오는 통로 쪽 위치
- 차량 대기영역 네 모서리
- 전체 map 폭·높이
- 천장 카메라 높이와 차량 mask가 잡히는 평균 높이

기준점은 넓게 퍼뜨린다. 한쪽 구석에 몰리거나 거의 한 직선 위에 놓인 점은
Homography가 불안정해진다. 입력 좌표는 줄자로 측정한 같은 `map` 좌표계를
사용한다.

## 3. 빌드와 등록 launch

```bash
source /opt/ros/humble/setup.bash
cd ~/cooperative_parking_robot_ws
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
```

패키지가 카메라를 직접 여는 경우:

```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true \
  camera_id:=0
```

이미 다른 ROS 카메라 드라이버가 `/camera/image_raw`를 발행하는 경우:

```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=false \
  cctv_raw_topic:=/camera/image_raw
```

이미 왜곡 보정된 `/cctv/image_rect`가 별도로 발행 중이라면
`enable_rectify:=false`로 실행한다. 하나의 카메라를 두 프로세스가 동시에 열지
않는다.

다른 PC의 브라우저에서 다음 주소를 연다.

```text
http://<JETSON-IP>:5001/
```

## 4. 브라우저 클릭 순서

1. **현재 영상 정지**를 누른다.
2. 바닥 기준점을 클릭한다.
3. 해당 점의 실측 `X,Y` metre를 입력하고 **현재 점 등록**을 누른다.
4. 기준점 4개 이상을 등록한 뒤 **Homography 계산**을 누른다.
5. 재투영 RMS·최대 오차와 BEV 격자를 확인한다.
6. 슬롯 ID를 입력한다.
7. 해당 슬롯의 모서리 네 점을 클릭한다. 순서는 상관없다.
8. 마지막 5번째 점은 슬롯 밖의 **차량 진입 통로 쪽**에 클릭한다.
9. **주차면 등록**을 누른다.
10. 모든 슬롯에 반복한다.
11. 차량 대기영역의 모서리 네 점을 클릭해 등록한다.
12. map 폭·높이·해상도를 확인한 뒤 저장한다.

브라우저에 축소된 영상 좌표를 그대로 저장하지 않는다. JavaScript가 다음처럼
원본 영상 해상도로 되돌린 픽셀을 서버에 전달한다.

```javascript
u = (event.clientX - rect.left) * canvas.width / rect.width
v = (event.clientY - rect.top) * canvas.height / rect.height
```

## 5. 생성 파일

기본 저장 위치:

```text
~/.ros/adaptive_valet_bot/homography_rectified.npy
~/.ros/adaptive_valet_bot/homography_rectified.json
~/.ros/adaptive_valet_bot/parking_layout.yaml
```

- `.npy`: 픽셀을 metre로 직접 바꾸는 3×3 행렬
- `.json`: 기준점, 원본 해상도, 재투영 오차, 클릭한 실제 슬롯 좌표
- `.yaml`: 두 ROS 노드가 읽는 슬롯 ID·중심·길이·폭·진입 Yaw
  및 점유률 계산용 실제 클릭 4점 polygon

저장 후 실행 중인 Mission 노드를 재시작해야 한다. 파일을 실시간 hot reload하지
않는다. `cctv_server.launch.py`와 `full_system.launch.py`의 기본 경로도
동일한 `~/.ros/adaptive_valet_bot/`이다. 패키지의 예시 layout은
`layout_registered: false`라서 실차 mission에서 거부된다.

## 6. 차량 전용 YOLO11-Seg 실행

권장 모델은 `vehicle` mask만 출력하는 YOLO11-Seg다. `empty_slot` 클래스는
필요 없다.

```bash
ros2 launch cooperative_parking_robot cctv_server.launch.py \
  enable_opencv_camera:=true \
  model_path:=/absolute/vehicle_seg.engine \
  model_mode:=vehicle_seg \
  homography_file:=/home/<USER>/.ros/adaptive_valet_bot/homography_rectified.npy \
  homography_scale_to_m:=1.0 \
  layout_config:=/home/<USER>/.ros/adaptive_valet_bot/parking_layout.yaml
```

새 Homography는 metre를 직접 출력한다. `homography_scale_to_m:=0.01`로 실행하면
모든 좌표가 100분의 1로 줄어드므로 사용하지 않는다.

## 7. 차량 크기와 슬롯 크기 비교

차량 mask의 map 좌표에 주축 분석을 적용해 길이·폭·Yaw를 계산한다. 상면 mask가
범퍼 외곽보다 작게 보일 수 있어 `vehicle_dimension_padding_m`를 양쪽에 더한다.

Fleet Manager는 차량만 비교하지 않고 다음 결합 footprint를 사용한다.

```text
loaded_length = max(vehicle_length, wheelbase + robot_length)
                + 2 * footprint_safety_margin
loaded_width  = max(vehicle_width, robot_width)
                + 2 * footprint_safety_margin
```

CCTV 차량 중심과 Front/Rear 중점에 body-frame offset이 있으면 그 양까지
대칭 외접 footprint에 더한다. 제어기도 회전 중 이 offset을 Yaw와
함께 회전시키고, 차량 중심이 staging에 고정되도록 로봇 중점 속도를
반대로 보상한다.

주차 가능 조건:

```text
slot_length >= loaded_length + 2 * slot_fit_longitudinal_margin
slot_width  >= loaded_width  + 2 * slot_fit_lateral_margin
```

따라서 차량만 들어가고 Front/Rear 로봇이 붙은 상태는 못 들어가는 슬롯은 자동
제외된다.

## 8. 실제 회전 진입 순서

메카넘 운반체이므로 자동차처럼 원호를 그리며 조향할 필요가 없다. 구현된 순서는
다음과 같다.

```text
고정 Yaw A* 평행이동
  -> 슬롯 밖 staging point 2cm 이내 도착
  -> 선속도 0, 슬롯 Yaw로 제자리 회전
  -> 슬롯 밖에서 중심선 횡오차 1cm 이내 정렬
  -> 슬롯 축을 따라 저속 직선/후진 삽입
  -> 슬롯 중심 2cm, Yaw 3도 이내에서 종료
```

Fleet Manager가 검사하는 항목:

- loaded footprint가 슬롯 길이·폭 안에 들어가는지
- 현재 Yaw를 유지한 A* 경로가 staging point까지 존재하는지
- staging point에서 회전 반대각 원이 장애물·맵 경계와 겹치지 않는지
- 정렬 후 슬롯 중심까지의 삽입 corridor가 비어 있는지

Segmentation 차량은 고정 `0.90m` 사각형이 아니라 실제 mask hull을
OccupancyGrid에 채운다. 슬롯은 0.75초 점유 유지와 5프레임 연속 empty
확인을 통과해야 빈자리로 발행된다.

`parking_direction`은 다음 중 하나다.

- `minimum_rotation`: 현재 Yaw에서 회전량이 작은 앞/뒤 방향
- `forward`: 차량 앞이 통로에서 슬롯 안쪽을 향함
- `reverse`: 차량 앞이 통로 쪽을 향한 채 후진 삽입

## 9. 실차에서 반드시 조정할 값

- `vehicle_detection_height_m`: 천장 CCTV가 보는 차량 상면 평균 높이
- `vehicle_dimension_padding_m`: 상면 mask와 실제 범퍼 외곽 차이
- `slot_occupancy_overlap_ratio`: 빈자리/점유 영상으로 결정
- `slot_fit_longitudinal_margin_m`, `slot_fit_lateral_margin_m`
- `slot_staging_gap_m`: 슬롯 입구와 회전 중심 사이 추가 간격
- `final_pos_tol`, `final_yaw_tol`

예시 YAML의 슬롯 좌표와 크기는 시연용 placeholder다. 브라우저로 생성한 현장
파일을 사용하기 전에는 실제 주차 정밀도를 보장하지 않는다.
벽·기둥 등 고정 장애물은 차량 Seg 모델이 보지 못하므로, 현장에 있다면
별도 OccupancyGrid/no-go 영역에 반드시 등록해야 한다.

## 10. 검증 명령

```bash
python3 -m compileall -q cooperative_parking_robot launch test
python3 -m pytest -q

source /opt/ros/humble/setup.bash
colcon build --symlink-install --packages-select cooperative_parking_robot
colcon test --packages-select cooperative_parking_robot \
  --event-handlers console_direct+
colcon test-result --verbose

curl -fsS http://127.0.0.1:5001/health
ros2 topic echo /parking/empty_slots
ros2 topic echo /parking/slot_pose
```
