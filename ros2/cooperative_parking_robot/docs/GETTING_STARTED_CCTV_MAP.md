# 신규 참여자 가이드 — 천장 CCTV 2대로 주차장 맵 만들기

작성 2026-08-14 · 대상 v1.11 · 선행 지식 불필요

이 문서 하나로 다음을 할 수 있게 하는 것이 목표다.

1. **오늘 당장** — 실제 주차장 없이, 카메라 2대만으로 RViz에 맵이 뜨는 것까지
2. **나중에** — 진짜 주차장을 만들고 실측값으로 바꿔 넣는 작업

처음 들어온 사람이 가장 많이 막히는 지점을 §7에 따로 모아뒀다. 에러가 나면 거기부터 보면 된다.

---

## 1. 이 시스템이 무엇을 하는가

주차 로봇 2대(Front/Rear)가 차량을 들어 올려 빈 주차면으로 옮긴다. 핵심 설계 사상은 **"로봇을 똑똑하게"가 아니라 "주차장을 똑똑하게"** 다.

- **인지·판단**은 천장에 달린 CCTV와 중앙 연산장치(Jetson Orin Nano)가 **전담**한다.
- **로봇**은 구동·정렬·파지만 한다. 그래서 로봇에 비싼 센서를 달 필요가 없다.

천장 CCTV가 하는 일은 네 가지다.

| 역할 | 설명 |
|---|---|
| 전역 인지 | 차량을 찾고, 어느 주차면이 비었는지 판정하고, 장애물 지도(OccupancyGrid)를 만든다 |
| 좌표 변환 | 카메라 **픽셀**을 주차장 바닥의 **미터**로 바꾼다 (Homography) |
| 절대 위치추정 | 로봇 상판의 ArUco 마커를 보고 로봇의 절대 위치·방향을 알려준다 |
| 안전 게이트 | 캘리브레이션이 없으면 아예 기동을 거부한다 |

v1.11부터 천장 카메라를 **2대** 쓴다. 한 대로는 주차장 전체를 못 보기 때문이다.

### 1-1. 데이터 흐름

```
 카메라0 ──→ /cctv0/image_raw ──→ [왜곡보정] ──→ /cctv0/image_rect
                                                      │
                                                 [YOLO 검출]
                                                      │
                                              /cctv0/detections
                                                      │
                                                      ├──→ [cctv_merge_node] ──→ /parking/map
 카메라2 ──→ /cctv2/image_raw ──→ [왜곡보정] ──→ /cctv2/image_rect          /parking/empty_slots
                                                      │                      /parking/target_pose
                                                 [YOLO 검출]                          │
                                                      │                               ▼
                                              /cctv2/detections ───────────→ [fleet_manager_node]
                                                                                  A* 경로계획
```

**왜 카메라마다 검출을 따로 하고 나중에 합치는가?**

두 카메라 영상을 먼저 이어붙여(스티칭) 하나로 만드는 방법도 있지만 쓰지 않았다. 차량처럼 **높이가 있는 물체**는 이어붙인 경계에서 잘리거나 두 겹으로 보이고, 그 오차가 장애물 지도에 그대로 들어간다. 대신 각 카메라가 독립적으로 "픽셀 → 미터"까지만 책임지고, 그 다음은 순수한 2D 기하 문제로 만든다. 카메라 하나가 죽어도 나머지가 계속 돈다.

**두 카메라의 좌표계는 어떻게 맞추는가?**

카메라 간 변환 행렬 같은 건 없다. 캘리브레이션할 때 **두 카메라 모두 같은 바닥 점에 같은 실측 (X, Y)m를 입력**하기 때문에, 두 Homography의 출력이 자동으로 같은 좌표계가 된다. 이것이 §6-3에서 줄자 작업을 강조하는 이유다.

---

## 2. 젯슨 환경 만들기 (최초 1회)

### 2-1. ROS 2 Humble

```bash
ls /opt/ros           # humble 이 있으면 건너뛴다
```

없다면:
```bash
sudo apt update && sudo apt install curl gnupg lsb-release -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
  -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" \
  | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null
sudo apt update
sudo apt install ros-humble-ros-base python3-colcon-common-extensions python3-rosdep -y
sudo rosdep init && rosdep update
```

### 2-2. 소스 배치

**폴더 구조를 정확히 맞춰야 한다.** `package.xml`이 `cooperative_parking_robot/` 바로 밑에 있어야 colcon이 패키지를 인식한다.

```
~/ros2_ws/
└── src/
    └── cooperative_parking_robot/     ← 이 폴더 안에
        ├── package.xml                ← 이게 바로 있어야 함
        ├── setup.py
        ├── cooperative_parking_robot/ (파이썬 코드)
        ├── launch/
        ├── config/
        ├── scripts/
        ├── test/
        └── docs/
```

폴더를 한 단계 더 감싸면 빌드가 안 된다.

### 2-3. 의존성

```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
pip3 install ultralytics
```

`ament_python`을 못 찾는다는 rosdep 경고는 정상이니 무시한다.

**그 다음 버전을 반드시 맞춰야 한다.** ultralytics 설치가 `~/.local`에 최신 패키지들을 밀어 넣으면서 ROS 2와 충돌한다. 이걸 안 하면 §7-1, §7-2에서 막힌다.

```bash
pip3 install --upgrade packaging      # setuptools가 요구하는 최신 packaging
pip3 install "setuptools<80"          # colcon-core가 요구
pip3 install "numpy<2"                # cv_bridge가 numpy 1.x로 컴파일돼 있음
```

확인:
```bash
python3 -c "
import numpy, setuptools, packaging
print('numpy', numpy.__version__, '(2 미만이어야 함)')
print('setuptools', setuptools.__version__, '(80 미만이어야 함)')
print('packaging', packaging.__version__, '(22 이상이어야 함)')
"
python3 -c "
import numpy as np
from cv_bridge import CvBridge
m = CvBridge().cv2_to_imgmsg(np.zeros((4,4,3), np.uint8), encoding='bgr8')
print('cv_bridge OK', m.encoding, m.width, m.height)
"
```

마지막 줄에서 `cv_bridge OK bgr8 4 4`가 나와야 한다. 여기서 `KeyError: 16`이 나면 §7-2로.

### 2-4. 빌드

```bash
cd ~/ros2_ws
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
ros2 pkg executables cooperative_parking_robot
```

`cctv_merge`, `yolo_bev_map`, `show_map_ascii` 등이 나오면 성공이다.

`Unknown distribution option: 'tests_require'` 경고는 무시해도 된다.

### 2-5. 편의 설정

새 터미널마다 source하는 게 번거로우면:
```bash
echo 'source /opt/ros/humble/setup.bash'   >> ~/.bashrc
echo 'source ~/ros2_ws/install/setup.bash' >> ~/.bashrc
echo 'export ROS_DOMAIN_ID=42'             >> ~/.bashrc
```

---

## 3. 오늘 당장 돌려보기 — 더미 캘리브레이션

> **이 배포본(parkingbot-main 2)에는 더미 캘리브레이션 생성기가 들어 있지
> 않다.** 가짜 좌표를 만드는 도구가 실배포 패키지에 있으면 실수로 돌려놓고
> 실측값으로 착각할 수 있어서, `test_repository_cleanup.py`가 그 존재 자체를
> 막는다. 아래 §3-3의 더미 절차는 **개발용 체크아웃에서만** 쓴다.
> 실제 등록은 §6-4의 `tile_homography`(포트 5006)로 한다.

### 3-1. 왜 더미부터 하는가

실제 캘리브레이션에는 주차장 바닥과 줄자 작업이 필요하다. 하지만 **배선이 맞는지, 노드가 다 뜨는지, 맵이 나오는지**는 그것 없이도 확인할 수 있다. 문제를 한 번에 하나씩만 상대하기 위해 순서를 나눈다.

더미 Homography는 "픽셀을 그냥 비례해서 미터로 늘린" 가짜다. **좌표값 자체는 아무 의미가 없다.** 대신 다음은 전부 진짜로 검증된다.

- 두 카메라가 각각 검출 결과를 내보내는가
- 병합 노드가 둘을 합치는가
- 겹치는 영역에서 같은 물체를 중복 제거하는가
- 어떤 카메라도 못 보는 주차면이 "빈자리"에서 제외되는가
- 맵이 생성되고 fleet_manager가 A*를 도는가

### 3-2. 카메라 확인

```bash
ls /dev/video*
v4l2-ctl --list-devices     # 없으면 sudo apt install v4l-utils
```

`/dev/video0`과 `/dev/video2`가 각각 다른 카메라인지 확인한다. USB 카메라는 한 대가 `video0`+`video1`(캡처+메타데이터) 두 개를 잡는 경우가 흔하다.

### 3-3. 임시 캘리브레이션 생성 (개발용 체크아웃 전용)

```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
cd ~/ros2_ws/src/cooperative_parking_robot
python3 scripts/make_dummy_calibration.py
```

기본값은 **640x480**이다. 패키지에 들어 있는 임시 intrinsic이 그 해상도로 만들어졌기 때문이다. 다른 해상도로 확인하려면 명시한다.
```bash
python3 scripts/make_dummy_calibration.py --width 1280 --height 720
```

> **해상도는 반드시 캘리브레이션과 맞춰야 한다.** npz의 주점(`cx`, `cy`)이 영상 중심 근처여야 정상이다. 640x480으로 캘리브레이션한 npz(`cx≈320`, `cy≈240`)를 1280x720 영상에 그대로 쓰면 왜곡 보정이 조용히 틀어지고, 그 위에서 만든 Homography에 그 오차가 그대로 박힌다. 4:3과 16:9는 화면비가 달라 초점거리 스케일링으로 변환되지 않으므로(`scale_camera_matrix`가 거부한다), 해상도를 바꾸려면 그 해상도로 **다시 캘리브레이션**해야 한다.

이미 파일이 있으면 실측 결과를 덮어쓰지 않으려고 중단한다. 다시 만들려면 `--force`.

출력은 이렇게 나온다.
```
map 6.0 x 4.0 m | 영상 640x480px
  cam0 시야: x 0.02~3.38 m, y 0.04~3.96 m
  cam2 시야: x 2.62~5.98 m, y 0.04~3.96 m
  겹침 구간: x 2.62~3.38 m (폭 0.76 m)

슬롯별 관측 카메라:
  P1 중심 (0.80, 3.00) -> cam0
  P2 중심 (1.80, 3.00) -> cam0
  P3 중심 (3.00, 3.00) -> cam0, cam2      ← 겹침 검증용
  P4 중심 (4.60, 3.00) -> cam2
  대기영역 중심 (3.00, 0.70) -> cam0, cam2
```

만들어지는 파일:
```
~/.ros/adaptive_valet_bot/
├── homography_cam0_rectified.npy   픽셀 → 미터 변환행렬 (cam0)
├── homography_cam0_rectified.json  메타데이터 (재투영 오차 등)
├── homography_cam2_rectified.npy
├── homography_cam2_rectified.json
└── parking_layout.yaml             주차면·대기영역·맵 크기
```

### 3-4. 실행

```bash
ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true camera0_id:=0 camera2_id:=2 \
  homography_cam0_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy \
  homography_cam2_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \
  layout_config:=$HOME/.ros/adaptive_valet_bot/parking_layout.yaml \
  model_path:=$HOME/yolov8n.pt \
  coco_vehicle_class_ids:="[0]" \
  process_every_n:=5
```

launch 기본 해상도도 640x480이다(`camera_width_px`, `calibration_width_px`). 카메라를 다른 해상도로 열려면 네 인자를 **함께** 바꿔야 한다.

인자 설명:

| 인자 | 의미 |
|---|---|
| `enable_opencv_camera:=true` | 이 패키지가 카메라를 직접 연다. 다른 노드가 이미 영상을 발행 중이면 false |
| `camera0_id`, `camera2_id` | `/dev/videoN`의 N |
| `homography_cam*_file` | 카메라별 좌표 변환행렬 |
| `layout_config` | 주차면·대기영역 정의. **두 카메라가 하나를 공유** |
| `model_path` | YOLO 모델. 없으면 자동 다운로드 |
| `coco_vehicle_class_ids:="[0]"` | **테스트용.** 0=person이라 사람이 장애물로 잡힌다. 실제로는 기본값 `[2,3,5,7]`(car/motorcycle/bus/truck) |
| `process_every_n:=5` | 5프레임에 1번만 추론. 카메라 2대분이라 부하가 크다 |

**이 터미널은 닫지 말 것.** 닫거나 Ctrl+C를 누르면 9개 노드가 전부 죽는다.

정상이면 이런 로그가 나온다.
```
[cctv_merge_node]: cctv_merge_node 시작 | cameras=['cam0', 'cam2'] | slots=['P1','P2','P3','P4']
[cctv_rectify_node_cam0]: CCTV calibration loaded | fx=436.85 fy=433.73
[opencv_camera_node_cam0]: CCTV first frame: 1280x720, reported_fps=30.00
[yolo_bev_map_node_cam0]: COCO vehicle classes: 0=person
[yolo_bev_map_node_cam0]: yolo_bev_map 시작 | camera_id=cam0 | mission_outputs=False | detections=/cctv0/detections
[yolo_bev_map_node_cam0]: [cam0] coverage polygon: (0.02,3.96), (3.38,3.96), (3.38,0.04), (0.02,0.04)
```

초반에 몇 초간 `살아있는 CCTV sensor 노드가 없습니다`가 반복되는 건 **정상**이다. CPU로 YOLO를 로딩하는 데 8~10초 걸린다. `coverage polygon` 로그가 나오면 그때부터 살아난다.

### 3-5. 상태 확인 (터미널 2)

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash

ros2 node list                       # 9개 나와야 함
ros2 topic hz /cctv0/image_rect      # ~30
ros2 topic hz /parking/map           # ~1
ros2 topic echo /cctv/merge_status --full-length --once
```

`--full-length`가 없으면 `ros2 topic echo`가 긴 문자열을 `...`로 잘라서 내용을 못 본다.

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

### 3-6. RViz 화면 만들기

RViz는 **젯슨에 직접 연결된 모니터**에서 실행해야 한다. SSH나 VSCode 원격에서 실행하면 `qt.qpa.xcb: could not connect to display`로 죽는다.

**터미널 3 — TF 발행 (먼저 해야 함)**

지금 시스템은 TF를 아무도 발행하지 않는다. RViz는 Fixed Frame이 TF 트리에 있어야 하므로, 이게 없으면 `Fixed Frame [map] does not exist` 에러가 난다.

```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```
(로봇까지 붙이면 TF는 제대로 발행해야 한다. 지금은 맵 확인용 임시 조치다.)

**터미널 4 — RViz**
```bash
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
rviz2
```

**RViz 안에서 설정**

1. 좌측 상단 **Global Options → Fixed Frame**을 `map`으로
2. 좌측 하단 **Add** → **Map**
   → 생성된 `Map` 항목에서 **`Topic`** 행의 값 칸을 클릭하고 `/parking/map` 입력
   *토픽을 안 넣으면 `Error subscribing: Empty topic name`이 뜬다. 가장 흔한 실수다.*
3. **Add** → **PoseArray** → `Topic`에 `/parking/empty_slots`
   빈 주차면이 화살표로 표시된다. 화살표 방향 = 차량 진입 방향
4. **Add** → **PointStamped** 또는 **Pose** → `/parking/target_pose`
   대기영역의 타겟 차량
5. 맵이 6×4m라 기본 시점에선 아주 작다. **휠로 확대**, **가운데 버튼 드래그**로 (3, 2) 근처를 화면 중앙에
6. **File → Save Config As**로 저장해두면 다음부터 바로 뜬다

**정상 판정**

`Map` 항목이 `Status: Ok`이고 `Resolution 0.05 / Width 120 / Height 80`이 채워지면 성공이다.
`Status: Warn` + `No map received` + `Width 0`이면 발행하는 쪽이 없다는 뜻이니 §3-4 launch가 살아 있는지 확인한다.

### 3-7. 모니터가 없다면 — 터미널 뷰어

SSH만으로도 맵을 볼 수 있다.
```bash
python3 ~/ros2_ws/src/cooperative_parking_robot/cooperative_parking_robot/show_map_ascii.py
```
(빌드 후에는 `ros2 run cooperative_parking_robot show_map_ascii`)

```
  y= 4.0m +--------------------------------------------------+
         |..........O.......O.........O..........O..........|   ← 빈 주차면
         |..................................................|
         |....................###########...................|   ← 장애물
         |..................................................|
  y= 0.0m +--------------------------------------------------+
          x=0.0m                                      x=6.0m

  카메라  : cam0=OK (1건, 0.07s)  cam2=OK (1건, 0.05s)
  병합    : 검출 1개 | 중복제거 1 | 2대관측 1
  슬롯    : P1:빈칸  P2:빈칸  P3:빈칸  P4:시야밖
```

---

## 4. 무엇을 확인해야 하는가

더미 캘리브레이션 단계에서 **반드시 눈으로 확인할 것 두 가지.**

### 4-1. 겹침 영역 중복 제거

겹침 구간(기본 x 2.62~3.38m)에 사람이 서면, 두 카메라가 같은 사람을 각각 보고한다. 병합 노드가 이걸 하나로 합쳐야 한다.

```bash
ros2 topic echo /cctv/merge_status --full-length | grep -o '"duplicates_removed": [0-9]*'
```

`1` 이상이 나와야 한다. **`0`이면 두 Homography가 서로 다른 곳을 가리키고 있다는 뜻이다.**

RViz에서는 검은 블록이 **하나만** 보여야 한다. 두 개로 보이면 좌표계가 어긋난 것이다.

> 지금은 더미 H라 어긋나도 정상이다. 실측 후에는 반드시 하나로 합쳐져야 한다.

### 4-2. 시야 밖 주차면 안전장치

이게 이 시스템에서 가장 중요한 안전 로직이다.

**cam0이 P4를 아예 못 보는데 "cam0이 차를 못 봤으니 빈자리"로 판정하면, 실제로는 차가 있는 칸으로 로봇을 보내게 된다.**

그래서 주차면마다 "지금 이 칸을 볼 수 있는 카메라가 살아 있는가"를 먼저 확인하고, 아무도 못 보면 빈자리 목록에서 **제외**한다.

확인:
```bash
# cam2 sensor만 죽인다
pkill -f "yolo_bev_map.*cam2"

# P4가 빠져야 정상
ros2 topic echo /parking/empty_slots --once
ros2 topic echo /cctv/merge_status --full-length --once   # P4의 observed: false
```

---

## 5. 여기까지가 준비운동이다

지금까지 확인한 것은 **배선**이다. 좌표는 전부 가짜다. 실제 주차장을 만들면 아래를 해야 한다.

---

## 6. 실제 주차장을 만들 때

### 6-1. 하드웨어 배치 원칙

카메라를 아무렇게나 달면 나중에 소프트웨어로 못 고친다.

| 항목 | 원칙 | 이유 |
|---|---|---|
| 카메라 방향 | 바닥을 **수직(90°)으로** 내려다보게 | 기울면 Homography 오차가 커지고 차량 높이에 의한 왜곡이 심해진다 |
| 두 카메라 높이 | **같게** | 현재 코드는 `camera_height_m`이 스칼라 하나다. 다르면 마커 보정이 부정확해진다 |
| 시야 겹침 | 최소 0.5~1.0m 폭 | 겹침이 없으면 두 좌표계가 맞는지 검증할 방법이 없다 |
| 주차면 위치 | 두 카메라 **경계에 걸치지 않게** | 경계에 걸친 칸은 어느 카메라도 온전히 못 봐서 판정이 불안정해진다 |
| 대기영역 | 가능하면 겹침 구간에 | 입차 시작 판정이 한 카메라 고장에 안 죽는다 |
| 조명 | 균일하게, 바닥 반사 최소화 | YOLO 검출률과 ArUco 인식률에 직접 영향 |

**설치 높이는 화각에서 역산한다.** 패키지의 임시 intrinsic 기준으로 화각은 약 72° x 58°다.

| 설치 높이 | 카메라 1대가 덮는 바닥 |
|---|---|
| 2.0 m | 2.93 x 2.21 m |
| 2.5 m | 3.66 x 2.77 m |
| 3.0 m | 4.40 x 3.32 m |

기본 맵이 6 x 4 m인데 2.5m 높이면 세로가 2.77m밖에 안 나온다. 4m 깊이를 덮으려면 **3.6m 이상**이 필요하다. 천장이 낮으면 맵을 줄이거나 카메라를 늘려야 한다. 자기 카메라의 화각은 npz에서 계산한다.

```bash
python3 -c "
import numpy as np, math
d = np.load('config/cctv0_camera_calibration.npz')
M = d['mtx'] if 'mtx' in d.files else d['camera_matrix']
W, H = 640, 480   # 캘리브레이션 해상도
hf = 2*math.degrees(math.atan(W/2/M[0,0])); vf = 2*math.degrees(math.atan(H/2/M[1,1]))
print('화각 %.0f x %.0f' % (hf, vf))
for h in (2.0, 2.5, 3.0, 3.6):
    print('  %.1fm -> %.2f x %.2f m' % (h, 2*h*math.tan(math.radians(hf/2)), 2*h*math.tan(math.radians(vf/2))))
"
```

### 6-2. 카메라 내부 파라미터 (npz) — 카메라마다 각각

렌즈 왜곡을 펴는 데 쓰는 값이다. **한 카메라의 값을 다른 카메라에 쓰면 안 된다.** 같은 모델이라도 개체차가 있다.

```bash
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p camera_id:=0 -p output_file:=$HOME/cctv0_camera_calibration.npz

ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p camera_id:=2 -p output_file:=$HOME/cctv2_camera_calibration.npz
```

체커보드를 여러 각도·거리에서 찍는다. 만든 npz를 `config/`에 넣고 **재빌드**해야 `install/`에 반영된다.

```bash
cp ~/cctv0_camera_calibration.npz ~/ros2_ws/src/cooperative_parking_robot/config/
cp ~/cctv2_camera_calibration.npz ~/ros2_ws/src/cooperative_parking_robot/config/
cd ~/ros2_ws && colcon build --symlink-install --packages-select cooperative_parking_robot
```

### 6-3. 바닥 기준점 실측 — 가장 중요한 단계

**코드가 아니라 줄자 작업이고, 여기서 틀리면 나머지가 전부 틀어진다.**

두 카메라의 좌표계를 하나로 묶는 것은 오직 이 숫자들이다.

바닥에 기준점을 표시하고, 원점 기준 (X, Y)를 미터로 재서 **종이에 적는다.**

```
예)  R1 (0.00, 0.00)   R2 (5.00, 0.00)   R3 (5.00, 3.50)   R4 (0.00, 3.50)
     R5 (2.50, 0.00)   R6 (2.50, 3.50)   ← 두 카메라가 다 보는 겹침 영역
```

규칙 세 가지:

1. 카메라마다 **최소 4점**, 권장 6~12점. 시야를 넓게 덮도록 퍼뜨린다.
2. **겹침 영역에 공통점 2~3개를 반드시 포함.** 두 카메라가 같은 물리 점을 같은 좌표로 보는지 확인할 유일한 수단이다.
3. 두 번의 등록에서 **같은 점에는 반드시 같은 숫자**를 입력한다.

> **원점을 어디로 잡을 것인가**: 주차장의 한쪽 구석을 (0,0)으로 잡고 +x, +y 방향을 정해서 바닥에 표시해둔다. 모든 좌표가 양수가 되게 잡아야 한다 — OccupancyGrid의 원점이 (0,0)에 고정되어 있어서 음수 좌표는 맵 밖으로 나간다.

### 6-4. Homography + 주차면 등록

카메라마다 한 번씩, 총 두 번 돌린다.

> ROS 토픽을 구독하는 등록 도구를 쓰면 카메라 노드를 껐다 켤 필요가 없다.
> ```bash
> # 터미널 1 — 카메라 + 왜곡보정만
> ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
>   enable_opencv_camera:=true camera0_id:=0 camera2_id:=2 \
>   enable_vision:=false enable_cctv_robot_markers:=false
> # 터미널 2 — 40cm 타일 격자 기반 등록 도구 (포트 5006)
> ros2 run cooperative_parking_robot tile_homography
> ```
> 브라우저형 단일 카메라 도구를 쓰려면 아래 절차를 따른다.

**1회차 — cam0**
```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true camera_id:=0 camera_label:=cam0 \
  cctv_raw_topic:=/cctv0/image_raw \
  cctv_rect_topic:=/cctv0/image_rect \
  cctv_camera_calib:=$HOME/ros2_ws/install/cooperative_parking_robot/share/cooperative_parking_robot/config/cctv0_camera_calibration.npz \
  homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy
```

브라우저로 `http://<젯슨IP>:5001/` 접속:

1. **현재 영상 정지** — 이후 클릭은 이 정지 화면 기준이다
2. **바닥 기준점** — 점을 클릭하고 실측 X(m), Y(m)를 입력해 등록. 최소 4개
3. **Homography 계산** — RMS 재투영 오차 확인. **2cm를 넘으면 기준점을 다시 찍는다**
4. **주차면** — cam0에 보이는 칸마다 모서리 4개 + **통로 쪽 점 1개**
   (통로점은 차량이 어느 방향에서 들어오는지 알려주는 것이다. 순서는 상관없지만 5번째 점이 반드시 통로 쪽이어야 한다)
5. **대기영역** — 모서리 4개. **1회차에서 반드시 등록**
6. **저장**

**2회차 — cam2**
```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true camera_id:=2 camera_label:=cam2 \
  cctv_raw_topic:=/cctv2/image_raw \
  cctv_rect_topic:=/cctv2/image_rect \
  cctv_camera_calib:=$HOME/ros2_ws/install/cooperative_parking_robot/share/cooperative_parking_robot/config/cctv2_camera_calibration.npz \
  homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \
  append_existing_layout:=true
```

- `layout_output_file`은 **지정하지 않는다.** 1회차와 같은 기본 경로를 써야 주차면이 합쳐진다.
- `append_existing_layout:=true`가 1회차 주차면을 지우지 않게 해준다.
- 기준점은 §6-3에서 적어둔 값을 **그대로** 입력한다.
- 주차면은 cam2에서만 보이는 것만 추가하면 된다.
- 대기영역은 재등록하지 않아도 된다.

### 6-5. 정합 검증 — 넘어가지 말 것

```bash
python3 -c "
import json
for c in ('cam0','cam2'):
    m = json.load(open(f'$HOME/.ros/adaptive_valet_bot/homography_{c}_rectified.json'))
    print(c, 'RMS %.4f m'%m['reprojection_rms_m'], '| max %.4f m'%m['reprojection_max_m'],
          '| 기준점', len(m['references']))
"
grep slot_ids ~/.ros/adaptive_valet_bot/parking_layout.yaml
```

- 두 카메라 모두 **RMS < 0.02m**
- `slot_ids`에 두 카메라의 주차면이 **모두** 들어 있을 것

그리고 §4-1의 중복 제거 테스트를 **실제 물체로** 다시 한다. 겹침 구간에 상자나 사람을 놓고 `duplicates_removed`가 1 이상이면 두 좌표계가 정합된 것이다. 0이면 §6-3 실측값부터 다시 본다.

### 6-6. 아직 남은 실측값들

맵 생성까지는 위로 충분하지만, **로봇을 실제로 움직이려면** 다음도 재야 한다. 지금은 전부 0(placeholder)이라 관련 보정이 꺼져 있다.

| 파라미터 | 무엇인가 | 재는 법 |
|---|---|---|
| `cam0_ground_x_m`, `cam0_ground_y_m` | 카메라 광축이 바닥과 만나는 점의 map 좌표 | 카메라 바로 아래 바닥 지점 (수직 설치 전제) |
| `cam0_height_m` | 카메라 설치 높이 | 바닥에서 렌즈까지 |
| `front_marker_height_m`, `rear_marker_height_m` | 로봇 상판 마커 높이 | 바닥에서 마커 표면까지 |
| `front_marker_offset_x_m`, `rear_marker_offset_x_m` | 마커 중심이 로봇 회전중심에서 진행축으로 떨어진 거리 | Front는 +, Rear는 − |
| `front_yaw_offset_deg`, `rear_yaw_offset_deg` | 마커 부착각 오차 | 로봇을 정방향으로 두고 CCTV yaw와 실제 방향 차이 |
| `vehicle_detection_height_m` | 차량 상면 높이 | 모형차 지붕 높이 |

**왜 필요한가**: Homography는 **바닥 평면** 기준이다. 바닥보다 높은 물체(로봇 상판 마커, 차량 지붕)는 카메라 광축에서 멀수록 실제보다 바깥으로 밀려 보인다.

```
오차 ≈ (물체 높이 / 카메라 높이) × (광축에서 물체까지 수평거리)
```

예: 카메라 2.5m, 마커 0.12m, 광축에서 2m 떨어진 위치 → 약 **9.6cm** 오차. 로봇 정렬 정밀도를 생각하면 무시할 수 없다.

이 값들을 넣으면 코드가 자동으로 보정한다. 0으로 두면 보정이 꺼진 채 동작한다(경고 로그가 나온다).

### 6-7. 실주행 전 남은 것

- **YOLO 모델**: COCO 기본 모델은 임시다. 실제로는 차량 세그멘테이션 모델(`vehicle_seg`)을 학습해서 쓴다. 차량 mask가 있어야 주차면 겹침률과 차량 치수를 정확히 계산할 수 있다.
- **CUDA**: 지금은 CPU로 추론 중일 가능성이 높다(`ros2 launch` 로그에 `CUDA initialization: driver too old`가 나오면 그렇다). `~/.local`의 pip PyTorch가 JetPack CUDA와 안 맞는 것이 원인이니, NVIDIA가 제공하는 Jetson용 PyTorch 휠로 교체해야 한다.
- **고정 장애물**: 벽·기둥은 차량 YOLO가 못 본다. 별도 no-go 영역으로 등록해야 한다.
- **시간 동기화**: 젯슨·Front RPi·Rear RPi를 NTP/chrony로 맞춰야 한다. CCTV 촬영시각을 로봇 쪽에서 신선도 판정에 쓴다.

---

## 7. 자주 막히는 지점

실제로 겪은 것들이다. 증상만 보고 바로 찾을 수 있게 정리했다.

### 7-1. `TypeError: canonicalize_version() got an unexpected keyword argument`

빌드가 실패한다. ultralytics가 새 setuptools를 깔았는데 `packaging`이 구버전이라 생기는 충돌이다.
```bash
pip3 install --upgrade packaging
pip3 install "setuptools<80"
```

### 7-2. `KeyError: 16` (cv_bridge, 카메라 노드가 죽음)

`cv2_to_imgmsg`에서 죽는다. cv_bridge가 만드는 타입 표에 `CV_8UC3(=16)`이 없다는 뜻이다.

원인 두 가지 — 둘 다 확인한다.
```bash
# (1) numpy 2.x
pip3 install "numpy<2"

# (2) pip으로 깔린 opencv-python이 ROS용 시스템 OpenCV를 가림
pip3 list | grep -i opencv
pip3 uninstall -y opencv-python opencv-python-headless opencv-contrib-python
sudo apt install --reinstall python3-opencv -y
python3 -c "import cv2; print(cv2.__file__)"   # /usr/lib/... 여야 정상
```

### 7-3. `ImportError: cannot import name '...' from cooperative_parking_robot....`

파일은 최신인데 예전 것으로 동작한다. 캐시가 남은 것이다.
```bash
find ~/ros2_ws/src -name __pycache__ -type d -exec rm -rf {} + 2>/dev/null
cd ~/ros2_ws && rm -rf build install log
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
```

파일 자체가 의심되면 md5로 비교한다.
```bash
md5sum ~/ros2_ws/src/cooperative_parking_robot/cooperative_parking_robot/bev_fusion_core.py
```

### 7-4. `RuntimeError: 현장 등록 layout이 아닙니다`

`parking_layout.yaml`이 없거나, 있어도 노드에 전달되지 않은 것이다.

파라미터 YAML은 **노드 이름이 정확히 일치**해야 값이 전달된다. 카메라 2대 구성에서는 노드 이름이 `yolo_bev_map_node_cam0` / `_cam2`로 갈리므로, 이 파일은 `/**:` 블록을 쓴다.
```bash
grep -n "^/\*\*:" ~/.ros/adaptive_valet_bot/parking_layout.yaml   # 나와야 함
```
없으면 예전 구조다. 다시 생성한다.
```bash
python3 scripts/make_dummy_calibration.py --force     # 또는 실측이면 §6-4 재실행
```

### 7-5. `ros2 param set`을 했는데 반영이 안 됨

`coco_vehicle_class_ids`처럼 노드 `__init__`에서 한 번만 읽고 캐싱하는 파라미터는 런타임 변경이 안 된다. "Set parameter successful"이 떠도 검출 루프는 예전 값을 쓴다.

**launch 인자로 지정해야 한다.**
```bash
ros2 launch ... coco_vehicle_class_ids:="[0]"
```

### 7-6. `qt.qpa.xcb: could not connect to display`

RViz를 SSH/원격에서 실행했다. **젯슨에 직접 연결된 모니터**의 터미널에서 실행한다.

원격에서 꼭 봐야 하면 대안은 셋이다.
- `ros2 run cooperative_parking_robot show_map_ascii` (가장 간단)
- Foxglove: 젯슨에 `sudo apt install ros-humble-foxglove-bridge` 후 `ros2 launch foxglove_bridge foxglove_bridge_launch.xml port:=8765`, PC에서 `ws://<젯슨IP>:8765` 접속
- X11 포워딩: `ssh -Y` (느리다)

### 7-7. RViz Map이 `No map received`, `Width 0`

두 가지 중 하나다.

1. **토픽 이름을 안 넣었다** — `Map` 항목의 `Topic` 행에 `/parking/map`을 직접 입력한다. `Error subscribing: Empty topic name`이 함께 뜬다.
2. **발행자가 없다** — `ros2 topic hz /parking/map`에 아무것도 안 나오면 launch가 꺼진 것이다.

### 7-8. `Fixed Frame [map] does not exist`

TF가 없다. 별도 터미널에서:
```bash
ros2 run tf2_ros static_transform_publisher 0 0 0 0 0 0 map base_link
```

### 7-9. `살아있는 CCTV sensor 노드가 없습니다`가 계속 나옴

- 기동 후 10초 이내면 정상(YOLO 로딩 중)
- 계속되면: `ros2 topic hz /cctv0/image_rect`로 영상이 흐르는지 → 안 흐르면 카메라 노드가 죽은 것(§7-2)
- 영상은 흐르는데 안 되면: `ros2 topic echo /cctv0/detections --once`로 검출이 나가는지 확인

### 7-10. `ros2 topic echo` 결과가 `...`로 잘림

```bash
ros2 topic echo /cctv/merge_status --full-length
```

### 7-11. `No executable found`

`setup.py`를 안 올렸거나 그 뒤 재빌드를 안 했다.
```bash
grep -n show_map_ascii ~/ros2_ws/src/cooperative_parking_robot/setup.py
cd ~/ros2_ws && colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
```

### 7-12. `ros2: command not found`

새 터미널에서 base ROS 2를 source하지 않았다.
```bash
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
```

---

## 8. 하면 안 되는 것

| 하지 말 것 | 이유 |
|---|---|
| 한 카메라의 npz를 다른 카메라에 쓰기 | 왜곡 보정이 틀어지고 그 위에 만든 Homography도 함께 틀어진다 |
| Raw 영상에서 Homography 만들기 | 반드시 왜곡 보정된 `/cctv*/image_rect`에서 만들어야 한다 |
| 두 등록에서 다른 기준점 값 넣기 | 두 카메라 좌표계가 어긋나 중복 제거가 실패한다 |
| `cctv_robot_marker_node`를 카메라마다 띄우기 | `/front/cctv_pose` 발행자가 둘이 되어 EKF가 같은 정보로 두 번 보정한다. 공분산이 과소평가되어 정상 측정이 기각되기 시작한다 (`CCTV_REJECTED_GATE`) |
| sensor 인스턴스에서 `/parking/*` 발행 | fleet_manager가 카메라별로 엇갈린 맵을 받는다. `publish_mission_outputs:=false`를 유지할 것 |
| 색상·블롭으로 yaw 추정 | yaw는 반드시 ArUco 마커로만 얻는다 (설계 원칙) |
| 차량에 마커 붙이기 | 임의 고객 차량이 전제다. 차량 인지는 항상 YOLO 세그멘테이션 |

---

## 9. 토픽 사전

| 토픽 | 타입 | 발행자 | 내용 |
|---|---|---|---|
| `/cctv0/image_raw`, `/cctv2/image_raw` | Image | opencv_camera | 카메라 원본 |
| `/cctv0/image_rect`, `/cctv2/image_rect` | Image | cctv_rectify | 왜곡 보정 영상 |
| `/cctv0/detections`, `/cctv2/detections` | String(JSON) | yolo_bev_map | 카메라별 차량 검출 + 시야 범위 |
| `/parking/map` | OccupancyGrid | **cctv_merge** | 장애물 지도 |
| `/parking/empty_slots` | PoseArray | **cctv_merge** | 빈 주차면 |
| `/parking/target_pose` | PoseStamped | **cctv_merge** | 대기영역 타겟 차량 |
| `/parking/vehicle_spec` | String(JSON) | **cctv_merge** | 차량 제원(휠베이스·치수) |
| `/parking/target_ready` | Bool | **cctv_merge** | 타겟 정차 확정 |
| `/cctv/merge_status` | String(JSON) | **cctv_merge** | 진단용 |
| `/front/cctv_pose`, `/rear/cctv_pose` | PoseStamped | cctv_robot_marker | 로봇 절대 위치 |
| `/parking/slot_pose` | PoseStamped | fleet_manager | 최종 선택된 목표 슬롯 |
| `/virtual_robot/waypoints` | — | fleet_manager | A* 경로 |

---

## 10. 알아둘 한계

이 프로젝트는 아직 완성이 아니다. 처음 들어온 사람이 오해하기 쉬운 부분이다.

- **출차는 구현됐다**(`parking_registry.py`, `retrieval_planning.py`). 다만 출차 목표는 CCTV 재탐지가 아니라 입차 때 Parking Registry에 저장한 보관 차량 자세를 쓴다 — "입차 후 사람이 차를 건드리지 않는다"는 전제이며, live 재검증은 향후 확장이다(`docs/adr/0003`).
- **A\*는 인양 직후 1회만 계산한다.** 주행 중 새 장애물에 대한 재계획이 없다. 통제된 정적 구역 전제다.
- **벽·기둥은 검출되지 않고, 등록할 방법도 아직 없다.** 차량 YOLO는 차량만 보는데 고정 장애물을 맵에 넣는 메커니즘이 코드에 없다. A*가 벽을 통과하는 경로를 낼 수 있으므로 실운영 전에 구현해야 한다.
- **두 카메라 설치 높이가 다르면** 마커 parallax 보정이 부정확하다. `camera_height_m`이 현재 스칼라 하나다.
- **인양 후 두 로봇이 같은 방향으로 함께 틀어지는 오차**는 현재 구조로 검출할 수 없다.

---

## 11. 더 읽을 것

| 문서 | 내용 |
|---|---|
| `DUAL_CCTV_MERGE_20260812.md` | 카메라 2대 병합의 설계 근거와 전체 변경 내역 |
| `CCTV_CALIBRATION.md` | 캘리브레이션 원칙 |
| `BEV_SLOT_REGISTRATION_AND_PARKING.md` | 주차면 등록과 점유 판정 |
| `localization_design.md` | 마커 체계(ID0/ID10/ID11)와 EKF |
| `HUMBLE_DEPLOYMENT.md` | 젯슨·Front·Rear 3대 장비 배포 |
| `system_spec.md` | 전체 시스템 사양 |
| `MASTER_PLAN.md` | 로드맵, 출차 설계 |
