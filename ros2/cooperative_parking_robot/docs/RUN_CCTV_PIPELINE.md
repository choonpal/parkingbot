# CCTV 파이프라인 실행 절차 (젯슨)

2026-08-29 통합 기준 · 패키지 배치부터 토픽 확인까지

Production은 `site_jetson.launch.py` 또는 저장소 루트의 `robot_start`를 우선한다.
현재 교체 카메라 asset은 **640x360 @ 30 fps**로 보정됐으며 camera·calibration·
Homography의 픽셀 frame을 함께 바꾸지 않는다. 5008 관제탑은 두 CCTV와 BEV를
보는 화면이고 Rear ID0 전용 5005 화면과 다르다.

---

## 0. 패키지 배치

ROS 워크스페이스에는 **`ros2/cooperative_parking_robot` 하나만** 넣는다.
`stm32/`와 최상위 `docs/`는 워크스페이스에 들어갈 것이 아니다.

```text
<repository>/ros2/cooperative_parking_robot/
        ↓ symlink 또는 검증된 배포
<colcon-workspace>/src/cooperative_parking_robot/
        ├── package.xml        ← 이 폴더 바로 밑에 있어야 함
        ├── setup.py
        ├── cooperative_parking_robot/
        ├── launch/
        ├── config/
        ├── models/
        ├── scripts/
        └── docs/
```

source와 install의 SHA를 배포 기록에 남긴다. 예전 디렉터리를 복사해 덮어쓰지
말고 [실차 Runbook](../../../docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md)의 clean build와
동일 SHA 확인 절차를 따른다.

---

## 1. 빌드

```bash
export PARKINGBOT_REPO=/absolute/path/to/parkingbot
export PARKINGBOT_WS=/absolute/path/to/colcon-workspace
source /opt/ros/humble/setup.bash
"${PARKINGBOT_REPO}/ros2/cooperative_parking_robot/scripts/humble_build_check.sh" \
  "${PARKINGBOT_WS}"
source "${PARKINGBOT_WS}/install/setup.bash"
```

`Unknown distribution option: 'tests_require'` 경고는 무시한다.

**확인:**

```bash
ros2 pkg executables cooperative_parking_robot
ls $(ros2 pkg prefix cooperative_parking_robot)/share/cooperative_parking_robot/models/
```

`parking_vehicle_yolo11n_seg.pt`가 보여야 한다. 이 모델은 `setup.py`의
`data_files`에 등록돼 있어 빌드 때 share로 복사된다.

---

## 2. 환경 변수 (새 터미널마다)

```bash
export PARKINGBOT_WS=/absolute/path/to/colcon-workspace
source /opt/ros/humble/setup.bash
source "${PARKINGBOT_WS}/install/setup.bash"
export ROS_DOMAIN_ID=42
export ROS_LOCALHOST_ONLY=0
```

workspace 경로는 장비별 deployment config에서 관리한다. `.bashrc`에 특정 임시
설치본을 고정하면 새 shell이 구 overlay를 import할 수 있으므로 넣지 않는다.

---

## 3. 사전 점검

### 3-1. 카메라

```bash
ls -l /dev/video*
v4l2-ctl --list-devices
```

현장 `CAM0_DEVICE`, `CAM2_DEVICE`에는 확인한 `/dev/v4l/by-path/...`를 쓴다.
숫자 `camera0_id:=2`, `camera2_id:=0`은 device path를 빈 문자열로 명시했을 때만
쓰는 fallback이며 재부팅 후 역할을 보장하지 않는다. 어느 장치가 어느 카메라인지
헷갈리면:

```bash
bash "${PARKINGBOT_WS}/src/cooperative_parking_robot/scripts/check_cameras.sh"
```

### 3-2. 캘리브레이션 파일

```bash
ls -lh ~/.ros/adaptive_valet_bot/
```

| 파일 | 용도 | 없으면 |
|---|---|---|
| `homography_cam0_rectified.npy` | 픽셀 → 미터 (cam0) | 비전 노드 기동 거부 |
| `homography_cam2_rectified.npy` | 픽셀 → 미터 (cam2) | 비전 노드 기동 거부 |
| `parking_layout.yaml` | 슬롯·대기영역·맵 크기 | `layout_registered=false` 로 거부 |

패키지 `config/`의 `cctv0/cctv2_camera_calibration.npz`는 빌드 때 share로 복사된다.

`parking_layout.yaml`에 `/**:` 블록이 있어야 한다. 없으면 예전 구조라
카메라별 sensor 노드(`yolo_bev_map_node_cam0`)에 값이 전달되지 않는다.

```bash
grep -n "^/\*\*:" ~/.ros/adaptive_valet_bot/parking_layout.yaml
```

### 3-3. 파이썬 환경

```bash
python3 -c "
import numpy, cv2, torch, ultralytics
from cv_bridge import CvBridge
print('numpy', numpy.__version__, '(2 미만)')
print('cv2  ', cv2.__version__, cv2.__file__)
print('torch', torch.__version__, '| CUDA', torch.cuda.is_available())
print('ultralytics', ultralytics.__version__)
CvBridge().cv2_to_imgmsg(numpy.zeros((4,4,3), numpy.uint8), encoding='bgr8')
print('cv_bridge OK')
"
```

`cv2.__file__`이 `/usr/lib/python3/dist-packages/...`여야 한다.
`~/.local/...`이면 pip OpenCV가 시스템 것을 가려 `KeyError: 16`이 난다.

---

## 4. 실행

### 4-A. 캘리브레이션 단계 — 카메라만

homography가 아직 없거나 기준점을 찍을 때 쓴다. YOLO를 끄면 프레임률이
30fps 가까이 나와 클릭이 훨씬 수월하다.

```bash
export CAM0_DEVICE=/dev/v4l/by-path/REPLACE_CAM0-video-index0
export CAM2_DEVICE=/dev/v4l/by-path/REPLACE_CAM2-video-index0
ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  camera0_device:="${CAM0_DEVICE}" camera2_device:="${CAM2_DEVICE}" \
  camera_width_px:=640 camera_height_px:=360 \
  calibration_width_px:=640 calibration_height_px:=360 \
  enable_vision:=false \
  enable_cctv_robot_markers:=false
```

띄우는 노드: `opencv_camera_node_cam0/cam2`, `cctv_rectify_node_cam0/cam2`

### 4-B. 전체 파이프라인

```bash
export CAM0_DEVICE=/dev/v4l/by-path/REPLACE_CAM0-video-index0
export CAM2_DEVICE=/dev/v4l/by-path/REPLACE_CAM2-video-index0
ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  camera0_device:="${CAM0_DEVICE}" camera2_device:="${CAM2_DEVICE}" \
  camera_width_px:=640 camera_height_px:=360 \
  calibration_width_px:=640 calibration_height_px:=360 \
  homography_cam0_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy \
  homography_cam2_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \
  layout_config:=$HOME/.ros/adaptive_valet_bot/parking_layout.yaml
```

`model_path`, `model_mode:=vehicle_seg`, `inference_imgsz:=640`은 launch
기본값이 이미 패키지의 학습 모델을 가리키므로 따로 줄 필요가 없다.

TensorRT engine을 만들었다면:

```bash
  model_path:=$HOME/ros2_ws/src/cooperative_parking_robot/models/parking_vehicle_yolo11n_seg.engine
```

**이 터미널은 닫지 않는다.** 닫으면 노드가 전부 죽는다.

#### 기동 로그에서 확인할 것

```
[opencv_camera_node_cam0] CCTV camera opened: camera_id=0 -> /cctv0/image_raw
[opencv_camera_node_cam0] CCTV first frame: 640x360, reported_fps=30.00
[cctv_rectify_node_cam0]  CCTV calibration loaded | fx=436.85 ...
[cctv_rectify_node_cam0]  CCTV undistort map ready: 640x360
[yolo_bev_map_node_cam0]  YOLO loaded: ... | mode=vehicle_seg | task=... | imgsz=640
[yolo_bev_map_node_cam0]  yolo_bev_map 시작 | camera_id=cam0 | mission_outputs=False
[yolo_bev_map_node_cam0]  [cam0] coverage polygon: (...)
[cctv_merge_node]         cctv_merge_node 시작 | cameras=['cam0','cam2'] | slots=[...]
[cctv_robot_marker_node]  cctv_robot_marker_node 시작 (markers={'front':10,'rear':11}, ...)
```

초반 몇 초간 `살아있는 CCTV sensor 노드가 없습니다`가 반복되는 것은 정상이다.
모델 로딩이 끝나고 `coverage polygon`이 찍히면 살아난다.

---

## 5. 토픽 확인 (터미널 2)

### 5-1. 노드가 다 떴는가

```bash
ros2 node list
```

전체 실행이면 9개:

```
/opencv_camera_node_cam0   /opencv_camera_node_cam2
/cctv_rectify_node_cam0    /cctv_rectify_node_cam2
/yolo_bev_map_node_cam0    /yolo_bev_map_node_cam2
/cctv_merge_node           /fleet_manager_node
/cctv_robot_marker_node
```

### 5-2. 영상

```bash
ros2 topic hz /cctv0/image_raw     # ~30
ros2 topic hz /cctv0/image_rect    # ~30
ros2 topic hz /cctv2/image_rect    # ~30
```

`image_raw`는 나오는데 `image_rect`가 없으면 rectify가 캘리브레이션을 못 읽은 것이다.

### 5-3. 카메라별 검출

```bash
ros2 topic echo /cctv0/detections --full-length --once
```

`coverage_polygon`이 채워져 있어야 한다. `--full-length`가 없으면 JSON이
`...`으로 잘려 내용을 못 본다.

### 5-4. 병합 진단 — 가장 먼저 볼 것

```bash
ros2 topic echo /cctv/merge_status --full-length
```

```json
{
  "cameras": {
    "cam0": {"alive": true, "age_s": 0.04, "detections": 1, "coverage_ready": true},
    "cam2": {"alive": true, "age_s": 0.05, "detections": 1, "coverage_ready": true}
  },
  "merged_detections": 1,
  "duplicates_removed": 1,
  "multi_camera_detections": 1,
  "slots": {"P1": {"observed": true, "occupied": false}, ...}
}
```

| 증상 | 원인 |
|---|---|
| `alive: false` | 그 카메라 sensor 노드가 죽었거나 영상이 안 옴 |
| `coverage_ready: false` | homography 미로드 |
| 겹침에 물체가 있는데 `duplicates_removed: 0` | **두 H가 서로 다른 map frame** |
| 특정 슬롯이 계속 `observed: false` | 어느 카메라 시야에도 없음 |

### 5-5. 임무 토픽

```bash
ros2 topic echo /parking/map --once | head -20      # 120x80 @ 0.05
ros2 topic echo /parking/empty_slots --once         # 빈자리 Pose
ros2 topic echo /parking/target_ready               # 대기영역 차량 정차 확정
ros2 topic echo /parking/target_pose                # 타겟 차량 위치
ros2 topic echo /parking/vehicle_spec --once        # 휠베이스·차량 치수
```

### 5-6. 로봇 절대 pose

```bash
ros2 topic echo /front/cctv_marker_visible
ros2 topic echo /rear/cctv_marker_visible
ros2 topic echo /rear/cctv_pose

# 발행자가 정확히 1개여야 한다 (2개면 EKF 가 같은 정보로 두 번 보정한다)
ros2 topic info /rear/cctv_pose --verbose | grep -c "Node name"
```

---

## 6. 눈으로 보기 (터미널 3)

### 6-1. 웹 프리뷰 — 권장

`cctv_server_dual.launch.py`와 `site_jetson.launch.py`는 기본적으로 이 관제탑을
함께 실행한다(`enable_control_tower_preview:=true`). 이미 dual-CCTV stack이 떠
있다면 아래 명령을 중복 실행하지 않는다. 관제탑을 별도로 껐을 때만 실행한다.

```bash
ros2 launch cooperative_parking_robot cctv_detection_preview.launch.py
```

같은 Wi-Fi의 다른 컴퓨터에서는 브라우저로
`http://robot-desktop.local:5008/`에 바로 접속한다. SSH나 포트 포워딩은
필요 없다. `.local` 이름이 안 열리면 Jetson에서 `hostname -I`를 실행하고,
첫 번째 Wi-Fi 주소를 사용해 `http://<Wi-Fi-IP>:5008/`로 접속한다.

다른 네트워크에서 VSCode 원격으로 볼 때만 **PORTS** 탭에서 5008을
forward한 뒤 `http://localhost:5008/`을 연다.

이 명령은 카메라와 YOLO를 새로 띄우지 않는다. 먼저 실행한
`site_jetson.launch.py`의 `/cctv0/image_rect`, `/cctv2/image_rect`와
`/cctv0/detections`, `/cctv2/detections`를 읽기만 한다. 따라서 카메라 busy나
TensorRT 모델 중복 로드가 없다.

상판 ArUco의 검은 외곽 한 변은 현재 24 cm가 기본값이다. 다른 크기를 쓸
때만 `marker_size_m`를 실제 검은 외곽 길이(m)로 덮어쓴다.

```bash
ros2 launch cooperative_parking_robot cctv_detection_preview.launch.py \
  marker_size_m:=0.20
```

보이는 것:

- 두 카메라 실시간 + 격자 + 중심 십자
- **ArUco 마커** — 원근에 따른 변 길이 차이, mm/px, world 좌표. 변 길이 차이는
  카메라 시점 진단값이며 주행 합격/실패 기준으로 사용하지 않는다.
- **Production 차량 검출** — 실제 sensor envelope의 윤곽·중심·world 좌표
- **검출 상태** — 토픽, sensor ID, 수신 Hz, 데이터 지연, sequence, H 상태,
  깨진 메시지와 순서가 뒤집힌 메시지 수
- **차량 상세** — 신뢰도, 길이·폭, yaw, 카메라 광축 거리, WAIT 영역 여부,
  차종 분류와 분류 휠베이스
- **이동 거리** — 차량을 움직이면 기준점 대비 직선거리와 누적 경로
- **BEV** — 카메라별 + 합성(색분리). 겹침이 회색이면 정합, 청록/빨강으로
  갈라지면 어긋난 것. 상관계수도 함께 표시

화면에 차량이 없을 때도 각 카메라의 `Production 수신 중`, 수신 Hz와
데이터 age가 계속 갱신되면 정상이다. `Production 수신 대기`라면 다음으로
원본 토픽부터 확인한다.

```bash
ros2 topic echo /cctv0/detections --once
ros2 topic echo /cctv2/detections --once
```

### 6-2. 터미널 맵 뷰어

모니터가 없을 때.

```bash
ros2 run cooperative_parking_robot show_map_ascii
```

### 6-3. RViz

젯슨에 **직접 연결된 모니터**에서만 된다. SSH에서 실행하면 죽는다.

```bash
# 터미널 A — TF (없으면 Fixed Frame 에러)
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link

# 터미널 B
rviz2
```

RViz 안에서: Fixed Frame `map` → Add **Map** → Topic에 `/parking/map`
**직접 입력** → Add **PoseArray** → `/parking/empty_slots`

토픽 이름을 안 넣으면 `Error subscribing: Empty topic name`이 뜬다. 가장 흔한 실수다.

---

## 7. 검증 두 가지

### 7-1. 겹침 중복 제거

겹침 구간에 물체를 놓고

```bash
ros2 topic echo /cctv/merge_status --full-length | grep -o '"duplicates_removed": [0-9]*'
```

**1 이상**이어야 두 좌표계가 정합된 것이다. 0이면 기준점 실측부터 다시 본다.

### 7-2. 시야 밖 슬롯 안전장치

가장 중요한 안전 로직이다. 못 보는 칸을 빈자리로 발행하면 차 있는 칸으로
로봇을 보내게 된다.

```bash
pkill -f "yolo_bev_map.*cam2"
ros2 topic echo /parking/empty_slots --once          # cam2 전용 슬롯이 빠져야 함
ros2 topic echo /cctv/merge_status --full-length --once   # 그 슬롯 observed:false
```

---

## 8. 정리

```bash
pkill -f cctv_server_dual
pkill -f opencv_camera
pkill -f camera_preview
sleep 2
sudo fuser -v /dev/video0 /dev/video2    # 비어야 함
```

**카메라를 직접 여는 프로그램은 동시에 하나만** 돌 수 있다.

| 도구 | 카메라 직접 열기 | 동시 실행 |
|---|---|---|
| `cctv_server_dual.launch.py` (`enable_opencv_camera:=true`) | O | 이것만 |
| `camera_preview` | X (토픽 구독) | 가능 |
| `tile_homography` | X (토픽 구독) | 가능 |
| 원본 `dual_tile_homography_gui.py` | O | **충돌** |

---

## 9. 자주 막히는 지점

| 증상 | 원인 / 조치 |
|---|---|
| `ros2: command not found` | `source /opt/ros/humble/setup.bash` 먼저 |
| `No executable found` | `setup.py` 미반영. 재빌드 |
| `can't open camera by index` | 이전 launch가 살아 있음. `pkill` 후 `sudo fuser -v /dev/video*` |
| `KeyError: 16` (cv_bridge) | numpy 2 또는 pip OpenCV. `pip3 install "numpy<2"`, pip opencv 제거 |
| `ImportError: cannot import name ...` | stale `__pycache__`. 지우고 클린 빌드 |
| `현장 등록 layout이 아닙니다` | `parking_layout.yaml`에 `/**:` 블록 없음. 재등록 |
| `ros2 param set` 반영 안 됨 | `__init__`에서 캐싱하는 값. launch 인자로 지정 |
| `qt.qpa.xcb: could not connect to display` | RViz를 SSH에서 실행. 젯슨 모니터에서 |
| `echo` 결과가 `...`로 잘림 | `--full-length` 추가 |
| `Address already in use` | 이전 웹 도구가 포트 점유. `pkill -f camera_preview` |

---

## 10. 터미널 배치 요약

```
[터미널 1] site_jetson.launch.py 또는 cctv_server_dual.launch.py  ← 닫지 말 것
[터미널 2] 토픽 확인 (merge_status, parking/*)
[터미널 3] camera_preview (5008) 또는 show_map_ascii
[터미널 4] tile_homography (5006)  — 기준점 등록할 때만
```
