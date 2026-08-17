# 천장 CCTV 2대 병합 (v1.11, 2026-08-12)

`/dev/video0`, `/dev/video2` 두 대의 천장 카메라를 임의 간격으로 설치하고, 두 시야가 겹치는 구간을 포함해 **하나의 map으로 합치는** 기능을 추가했다.

이 문서 하나로 다음을 모두 답한다.

1. 어떤 방식으로 합치는가 (그리고 왜 그 방식인가)
2. 무엇이 바뀌었는가 — 파일별·함수별 변경 내역
3. 내가 직접 만들어야 하는 npz / npy / yaml 파일과 그 생성 절차
4. 실행 명령과 확인(검증) 방법
5. 남은 한계와 주의사항

---

## 0. 3줄 요약

- 카메라마다 **자기 homography(H0, H2)** 를 갖되, **두 번 다 같은 바닥 점에 같은 실측 (X,Y)m를 입력**해서 등록한다. 그러면 두 H의 출력이 자동으로 같은 map 좌표계가 되므로 카메라 간 변환 행렬이 필요 없다.
- 카메라별 `yolo_bev_map_node`는 **자기가 본 차량 목록만** 발행하고(sensor 모드), 새로 만든 **`cctv_merge_node`** 가 두 목록을 겹침 제거해서 최종 `/parking/map`, `/parking/empty_slots` 등을 발행한다.
- 하류(`fleet_manager_node`, A*, `individual_move`, `rigid_body_sync`, UI)는 **코드 한 줄도 바뀌지 않았다.** 토픽 이름과 의미가 그대로이기 때문이다.

---

## 1. 왜 이 방식인가 — 선택하지 않은 대안들

### 1-1. 이미지 스티칭(파노라마)을 쓰지 않은 이유

두 영상을 먼저 이어붙여 하나의 큰 영상을 만들고 그 위에 H 하나를 얹는 방법도 있다. 채택하지 않았다.

- **스티칭 seam에서 기하가 깨진다.** 파노라마 스티칭은 "카메라 중심이 한 점"이거나 "장면이 평면"일 때만 정확하다. 천장 카메라 2대는 중심이 서로 떨어져 있고, 주차장에는 차량이라는 **높이 있는 물체**가 있다. 그래서 seam 근처에서 차량이 잘리거나 두 겹으로 보인다. 이 오차는 A* 장애물맵에 그대로 들어간다.
- **실시간 비용이 크다.** Jetson Orin Nano에서 매 프레임 warp+blend를 돌리면 YOLO에 쓸 자원이 줄어든다.
- **디버깅이 불가능해진다.** 좌표가 이상할 때 "스티칭이 문제인지 H가 문제인지 YOLO가 문제인지" 분리할 수 없다.

반면 world 좌표 병합은 각 카메라가 독립적으로 "픽셀 → metre"까지만 책임지고, 그 다음은 순수한 2D 기하 문제가 된다. 카메라 하나가 죽어도 나머지가 그대로 동작한다.

> 참고: 기존 문서 `CCTV_CALIBRATION.md`도 "카메라별로 `cctvN_camera_calibration.npz + HN_rectified.npy`가 각각 필요하고, 두 결과를 공통 world 좌표계로 병합해야 한다"고 이미 방향을 정해 두었다. 이번 구현은 그 미해결 항목을 실제 코드로 채운 것이다.

### 1-2. 카메라 간 변환 행렬(cam2 → cam0)을 따로 만들지 않은 이유

겹침 영역의 특징점을 매칭해서 2D 변환을 구하는 방법도 있다. 채택하지 않았다.

- 변환이 **2단(H2 → T → world)** 이 되면서 오차가 곱해진다.
- 등록 도구가 이미 "픽셀을 클릭하고 실측 metre를 타이핑"하는 구조다. 두 번째 카메라에서 **같은 바닥 점에 같은 숫자**를 입력하는 것만으로 정합이 끝난다. 추가 코드도, 추가 오차원도 없다.
- 실측 줄자 작업은 어차피 1회차에서 한 번 해야 한다. 2회차는 그 값을 **다시 입력**만 하면 되므로 추가 부담이 사실상 없다.

### 1-3. 노드 하나가 카메라 2대를 다 처리하지 않은 이유

`yolo_bev_map_node` 내부를 2채널로 고치는 방법도 있었다. 채택하지 않았다.

- 단일 카메라 경로와 코드가 뒤엉켜 "지금 어느 쪽이 도는 건지" 알 수 없게 된다.
- 카메라를 3대로 늘릴 때 또 고쳐야 한다. 현재 구조는 launch에 블록만 추가하면 된다.
- 한 카메라의 YOLO 추론이 다른 카메라의 콜백을 막는다(단일 스레드 executor).

---

## 2. 전체 구조

```
 /dev/video0 ─ opencv_camera(cam0) ─→ /cctv0/image_raw
                  └→ cctv_rectify(cam0, cctv0_camera_calibration.npz)
                        ─→ /cctv0/image_rect
                              ├→ yolo_bev_map(cam0, sensor) ─→ /cctv0/detections
                              └───────────────────────────────┐
 /dev/video2 ─ opencv_camera(cam2) ─→ /cctv2/image_raw        │
                  └→ cctv_rectify(cam2, cctv2_camera_calibration.npz)
                        ─→ /cctv2/image_rect                  │
                              ├→ yolo_bev_map(cam2, sensor) ─→ /cctv2/detections
                              └───────────────────────────────┤
                                                              ▼
                                                     cctv_merge_node
                                                       ├ /parking/map
                                                       ├ /parking/empty_slots
                                                       ├ /parking/target_pose
                                                       ├ /parking/vehicle_spec
                                                       ├ /parking/vehicle_pose_feedback
                                                       ├ /parking/target_ready
                                                       └ /cctv/merge_status  ← 신규 진단
                                                              ▼
                                                     fleet_manager_node (변경 없음)
                                                       └ A* → /virtual_robot/waypoints

 /cctv0/image_rect ┐
                   ├→ cctv_robot_marker_node (인스턴스 1개) → /front/cctv_pose, /rear/cctv_pose
 /cctv2/image_rect ┘
```

### 2-1. 상판 마커 노드는 왜 **하나만** 띄우는가

이 부분이 가장 중요한 설계 결정이다.

`cctv_robot_marker_node`를 카메라마다 하나씩 띄우면 `/front/cctv_pose`에 publisher가 둘이 된다. 그러면 `pose_fusion_node`의 EKF가 **같은 순간의 같은 로봇 위치**를 두 번 correct하게 된다. EKF는 "각 측정의 잡음이 서로 독립"이라고 가정하고 공분산을 줄이는데, 사실상 같은 정보를 두 번 먹으면 공분산이 실제보다 과도하게 작아진다. 그 결과 Mahalanobis 게이트가 좁아져 **정상 측정을 이상치로 기각**하기 시작한다 — `CCTV_REJECTED_GATE`가 반복되는 형태로 나타난다.

그래서 노드 하나가 두 영상을 모두 구독하고, **역할(front/rear)별로 카메라 하나만 골라서** 발행한다.

선택 기준은 **"그 카메라의 광축 지상점에서 마커까지의 거리"** 가 가장 작은 카메라다. 바닥 homography로 상판 마커(바닥보다 높음)를 투영할 때 생기는 parallax 오차가

```
오차 ≈ (마커 높이 / 카메라 높이) × (광축 지상점에서 마커까지의 수평거리)
```

이므로, 광축에 가까운 카메라를 쓰면 오차가 작아진다. `camera_ground_points`를 아직 실측하지 않았다면 **마커 픽셀이 영상 중심에서 얼마나 떨어졌는지**로 대신 판정한다(상대 비교만 하면 되므로 충분하다).

카메라가 바뀔 때 pose가 좌우로 튀지 않도록 `selection_hold_s`(기본 0.30초) 동안은 기존 카메라를 유지한다.

---

## 3. 파일별 변경 내역

### 3-1. 신규 파일 (3개)

#### `cooperative_parking_robot/bev_fusion_core.py` — 신규

ROS/OpenCV/YOLO에 의존하지 않는 순수 계산 모듈. 단위 테스트가 가능하도록 분리했다.

| 구성 요소 | 하는 일 |
|---|---|
| `CameraDetection` | 한 카메라가 본 차량 하나(world metre 중심, mask polygon, yaw, 길이/폭, 대기영역 여부, confidence, 광축거리, 차종) |
| `encode_detection_envelope` / `decode_detection_envelope` | 카메라 → 병합 노드로 보내는 JSON 직렬화. 버전 필드가 있어 스키마가 어긋나면 조용히 넘어가지 않고 거부한다 |
| `point_in_polygon` | 경계 포함 ray-casting. `yolo_bev_map_node`와 **같은 규칙**을 쓴다 |
| `image_corner_coverage` | **영상 네 귀퉁이를 H로 투영해 그 카메라의 바닥 시야(coverage polygon)를 자동 계산.** 사람이 자로 재서 넣으면 H와 반드시 어긋나므로 H에서 직접 유도한다 |
| `MergedDetection` / `merge_detections` | 카메라 간 중복 검출 제거 |
| `SlotOccupancyTracker` | **coverage를 인식하는** 슬롯 점유 debounce |
| `slot_observability` | 슬롯별로 "지금 살아있는 카메라 중 이 칸을 보는 카메라가 있는가" |
| `TargetLatchTracker` | 대기영역 차량 정차 latch (기존 규칙과 동일: 2cm / 2초) |
| `VehicleDimensionTracker` | 차량 길이/폭 EMA + 장축 yaw EMA (기존 규칙과 동일) |
| `summarize_merge` | `/cctv/merge_status` 진단 JSON |

**중복 제거 알고리즘** (`merge_detections`):

1. 모든 카메라의 검출을 `axis_dist_m`(광축 거리) 오름차순으로 정렬한다. 즉 **가장 정확한 관측이 앞에 온다.**
2. 앞에서부터 확정 목록에 넣되, 이미 확정된 것과
   - (a) 중심거리가 `duplicate_center_gate_m`(기본 0.35m) 이내이거나
   - (b) mask polygon의 **상호** 겹침률이 `duplicate_overlap_ratio`(기본 0.30) 이상이면
   중복으로 보고 흡수시킨다.
3. 흡수 시 위치는 정확한 쪽(primary)을 그대로 쓴다. `duplicate_center_blend`를 0보다 크게 주면 두 값을 섞는다.

> **gate 값을 0.35m로 잡은 근거**: 모형차 전장이 0.9m이므로 서로 다른 두 차량의 중심이 0.35m 안에 들어올 수 없다. 반대로 같은 차량을 두 카메라가 보면 parallax 때문에 최대 10cm 수준까지 벌어질 수 있으므로 gate는 그보다 넉넉해야 한다.

> **왜 "상호" 겹침률인가**: `polygon_overlap_ratio(subject, clip)`은 clip 면적 기준 비율을 준다. 한쪽 카메라에서 차량 mask가 영상 가장자리에 잘려 작게 나온 경우, 한 방향만 보면 겹침률이 낮게 나와 중복을 놓친다. 그래서 양방향을 다 계산해 큰 값을 쓴다.

**coverage 인식 슬롯 판정** (`SlotOccupancyTracker` + `slot_observability`) — 이번 변경에서 **가장 중요한 안전 장치**:

단순히 두 카메라 결과를 OR로 합치면 치명적인 버그가 생긴다. cam0이 P4 슬롯을 아예 보지 못하는데도 "cam0이 P4에서 차를 못 봤다 → 빈자리"로 판정하게 되기 때문이다. 실제로는 차가 있는 칸에 로봇을 보내게 된다.

그래서 슬롯마다 **"지금 살아있는 카메라 중 이 칸을 볼 수 있는 카메라가 있는가"** 를 먼저 확인한다.

- 볼 수 있는 카메라가 있음 → 기존 debounce 규칙 적용 (점유 유지 0.75초 / 빈칸 확정 5프레임)
- 아무도 못 봄 → **상태를 갱신하지 않고 직전 상태를 유지**하며, `empty_slot_ids()`에서 제외한다

초기 상태는 `occupied=True`다. 아직 아무것도 못 본 시점에 빈자리로 발행하면 로봇이 차 있는 칸으로 출발한다.

#### `cooperative_parking_robot/cctv_merge_node.py` — 신규

카메라별 검출을 받아 최종 `/parking/*`를 발행하는 노드. `merge_rate_hz`(기본 10Hz) 타이머로 다음을 반복한다.

1. 카메라별 최신 envelope의 나이를 확인해 `camera_timeout_s`(기본 1.0초) 이내면 살아있음으로 판단
2. 살아있는 카메라들의 검출을 `merge_detections`로 병합
3. 병합 후 최종 좌표로 대기영역 포함 여부를 **다시 확인**하고 타겟 latch 갱신
4. `slot_observability` → `SlotOccupancyTracker.update` → `/parking/empty_slots`
5. 운반 중 차량 피드백(Front/Rear 중점에 가장 가까운 mask, gate 0.45m)
6. OccupancyGrid 생성 (mask polygon은 `fillPoly`, mask 없으면 `car_size_m` 정사각형 폴백) + 로봇 self-mask + 운반 대상 제거
7. `/cctv/merge_status` 진단 발행

**안전 처리:**

| 상황 | 동작 |
|---|---|
| 모든 카메라가 죽음 | `/parking/*` 발행을 **중단**하고 에러 로그. 마지막 맵이 계속 흐르며 로봇이 오래된 정보로 움직이는 것을 막는다 |
| `require_all_cameras:=true`에서 한 대만 죽음 | 마찬가지로 발행 중단 |
| envelope의 `camera_id`가 구독한 카메라와 다름 | **거부.** 토픽 remap 실수로 좌표계가 조용히 섞이는 것을 막는다 |
| envelope의 `homography_ok=false` | 거부 |
| envelope JSON 파싱 실패 / 버전 불일치 | 거부 + 경고 |

#### `launch/cctv_server_dual.launch.py` — 신규

기존 `cctv_server.launch.py`는 **손대지 않았다.** 단일 카메라로 되돌리고 싶으면 그대로 쓰면 된다.

띄우는 노드: `opencv_camera` ×2, `cctv_rectify` ×2, `yolo_bev_map` ×2(sensor 모드), `cctv_merge` ×1, `fleet_manager` ×1, `cctv_robot_marker` ×1, `jetson_vision_web` ×1.

#### `test/test_dual_cctv_merge.py` — 신규

25개 회귀 테스트. 특히 다음 안전 성질을 고정한다.

- 겹침 영역의 같은 차량이 장애물 두 개로 남지 않는다
- 어떤 카메라도 보지 못하는 슬롯은 절대 빈자리로 발행되지 않는다
- sensor 인스턴스는 `/parking/*` publisher를 **아예 만들지 않는다**
- 상판 마커 노드는 카메라가 2대여도 인스턴스가 1개다
- 단일 카메라 기본값이 그대로 유지된다

---

### 3-2. 수정 파일 (6개)

#### `cooperative_parking_robot/yolo_bev_map_node.py`

**추가된 파라미터** (기본값은 모두 기존 단일 카메라 동작 그대로):

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `camera_id` | `'cam0'` | 병합 노드가 카메라를 구분하는 이름 |
| `publish_detections` | `False` | true면 자기 검출 목록을 `detection_topic`으로 발행 |
| `detection_topic` | `''` | 비우면 `/<camera_id>/detections` |
| `publish_mission_outputs` | `True` | **false면 `/parking/*` publisher를 아예 만들지 않는다** |
| `coverage_margin_px` | `8.0` | coverage polygon을 만들 때 잘라낼 영상 테두리 여유 |

**추가된 메서드:**

- `ensure_coverage_polygon(width, height)` — H로 영상 네 귀퉁이를 투영해 coverage polygon 계산 + 캐시. 광축 기준점(`_axis_reference`)도 여기서 정한다(실측값이 없으면 coverage 중심으로 대체하고 60초마다 경고).
- `axis_distance_m(x, y)` — 광축 기준점에서의 거리. 병합 노드의 중복 선택 기준.
- `publish_detection_envelope(...)` — envelope 발행.
- `_make_sensor_detection(...)` — `CameraDetection` 하나 생성.

**동작 변경:**

- `image_cb`에서 차량을 검출할 때마다 `sensor_detections` 리스트에도 담는다. 그 뒤 `publish_detections`면 envelope을 발행하고, `publish_mission_outputs=false`면 **거기서 리턴**한다(latch/슬롯/맵 로직을 돌리지 않음).
- `classify_vehicle`은 sensor 모드에서도 실행된다(분류기는 원본 crop 이미지가 필요하므로 카메라 노드에서만 가능). 다만 `/parking/vehicle_spec`을 발행하지 않고 결과를 `_classified_class` / `_classified_wheelbase`에 저장해 다음 프레임 envelope에 실어 보낸다.
- `publish_mission_outputs=false`면 `publish_map_periodic` 타이머 자체를 만들지 않는다.
- `publish_detections`와 `publish_mission_outputs`가 **둘 다 false면 시작 시 에러**(아무것도 발행하지 않는 노드가 되므로).

> **왜 publisher를 "만들지 않는" 것까지 하는가**: 두 인스턴스가 `/parking/map`에 publisher를 열어두면 `ros2 topic info`로 봐도 누가 진짜인지 알 수 없고, 실수로 한쪽이 발행되는 순간 fleet가 카메라별로 엇갈린 맵을 받는다.

#### `cooperative_parking_robot/cctv_robot_marker_node.py`

**추가된 파라미터:**

| 파라미터 | 기본값 | 의미 |
|---|---|---|
| `image_topics` / `homography_files` / `camera_ids` | `['']` | 배열 형태(직접 YAML 작성용) |
| `image_topics_csv` / `homography_files_csv` / `camera_ids_csv` | `''` | 쉼표 구분 문자열(launch용) |
| `camera_ground_points` | `[0.0]` | 카메라별 광축 지상점 `[x0,y0, x2,y2, ...]` |
| `selection_hold_s` | `0.30` | 다른 카메라로 넘어가기 전 현재 카메라 유지 시간 |
| `observation_timeout_s` | `0.30` | 이 시간 지난 관측은 선택 후보에서 제외 |

> **CSV 파라미터를 따로 둔 이유**: launch의 `PathJoinSubstitution` 결과는 "문자열 하나"로만 전달되므로, 배열 파라미터 안에 두 개를 넣으면 두 경로가 하나로 이어붙어버린다. 실제로 겪기 전에는 알기 어려운 함정이라 CSV 경로를 별도로 만들었다.

**구조 변경:**

- `_configure_cameras()` 신설 — 단일/멀티 파라미터를 하나의 `self.cameras` 리스트로 정규화. `image_topics`가 비어 있으면 기존 `image_topic`/`homography_file`을 그대로 쓴다(**하위호환**).
- `_load_homography(path, camera_id)` — 카메라별 H 로드. 없으면 즉시 시작 실패(기존과 동일한 fail-closed).
- `pixel_to_world(homography, px, py)` — 첫 인자로 H를 받도록 시그니처 변경.
- `image_cb(camera_id, msg)` — 카메라 ID를 받고, pose를 **바로 발행하지 않고** `self._observations[role][camera_id]`에 저장한 뒤 `_publish_selected()` 호출.
- `_selection_cost(...)` 신설 — 광축 거리(실측 있을 때) 또는 영상 중심 거리(없을 때).
- `_publish_selected(now)` 신설 — 만료 관측 정리 → 역할별 최적 카메라 선택 → hold 적용 → `/{role}/cctv_pose` + `/{role}/cctv_marker_visible` 발행. 카메라 전환 시 로그를 남긴다.
- `_correct_parallax(camera, ...)` — parallax 보정을 **그 카메라의** 광축 기준으로 수행. 두 카메라가 같은 map frame을 공유해도 광축 위치는 서로 다르다.
- `self.image_topic` 속성은 첫 카메라 토픽으로 유지(기존 코드/테스트 호환).

#### `cooperative_parking_robot/bev_layout_core.py`

- `render_parking_layout_yaml`에 **`cctv_merge_node:` 파라미터 블록 추가.** 병합 노드가 같은 등록 파일에서 슬롯/대기영역/맵 크기를 읽는다.
- `load_layout_yaml(path)` **신규** — 저장된 `parking_layout.yaml`에서 슬롯/폴리곤/대기영역/맵 크기를 복원.
- `merge_layout_registrations(existing, new_slots, new_polygons, new_waiting)` **신규** — 기존 layout에 이번 카메라에서 등록한 슬롯을 합친다. 같은 `slot_id`가 양쪽에 있으면 **새로 등록한 쪽이 이긴다**(현장에서 "다시 찍었는데 반영이 안 된다"가 가장 헷갈리므로). 대기영역은 새로 등록했으면 교체, 아니면 기존 유지.

#### `cooperative_parking_robot/bev_layout_calibrator_node.py`

- 파라미터 추가: `camera_label`(로그/메타데이터 표시용), `append_existing_layout`(2회차용).
- `/api/save`가 `append_existing_layout=true`일 때 기존 layout을 읽어 `merge_layout_registrations`로 합친 뒤 저장. append 모드에서는 이번 카메라에서 슬롯을 하나도 등록하지 않아도 되고, 대기영역도 재등록하지 않아도 된다.
- 저장 메타데이터 JSON에 `camera_label`, `appended_to_existing_layout` 추가.
- 저장 응답에 `slot_count`(합친 후 전체)와 `new_slot_count`(이번에 등록한 것) 구분.
- append 모드 시작 시 **"기준점은 1회차와 같은 바닥 점 / 같은 실측 (X,Y)m로 찍으세요"** 경고 로그.

#### `launch/bev_layout_calibration.launch.py`

- `camera_label`, `append_existing_layout` launch 인자 추가 + 노드에 전달.
- docstring에 카메라 2대 등록 명령 예시 추가.

#### `setup.py` / `package.xml` / `test/test_humble_port.py`

- `cctv_merge` console script 등록.
- 버전 `1.10.0` → `1.11.0` (setup.py, package.xml, 버전 핀 테스트).
- package.xml description에 "천장 CCTV 2대 병합" 명시.

---

## 4. 내가 만들어야 하는 파일

| 파일 | 위치 | 만드는 방법 | 필수 |
|---|---|---|---|
| `cctv0_camera_calibration.npz` | `config/` | `calibrate_camera` 노드(체커보드) | O |
| `cctv2_camera_calibration.npz` | `config/` | `calibrate_camera` 노드(체커보드) | O |
| `homography_cam0_rectified.npy` | `~/.ros/adaptive_valet_bot/` | `bev_layout_calibration.launch.py` 1회차 | O |
| `homography_cam2_rectified.npy` | `~/.ros/adaptive_valet_bot/` | `bev_layout_calibration.launch.py` 2회차 | O |
| `parking_layout.yaml` | `~/.ros/adaptive_valet_bot/` | 위 두 번의 등록으로 자동 생성/병합 | O |
| `homography_cam*_rectified.json` | 같은 폴더 | 자동 생성(재투영 오차 기록) | 자동 |
| `yolov8n.pt` 또는 `vehicle_seg.engine` | 아무 경로 | YOLO 모델. COCO는 자동 다운로드 가능 | O |

> 기존 `config/cctv_camera_calibration.npz`는 그대로 두었다. cam0에 같은 카메라를 쓴다면 `cctv0_camera_calibration.npz`로 복사해도 되지만, **해상도와 렌즈가 동일한지 반드시 확인**해야 한다. 두 카메라가 같은 모델(OV2710)이라도 개체차가 있으므로 각각 캘리브레이션하는 것을 권장한다.

### 4-1. 왜 카메라마다 npz가 따로 필요한가

`cctv_rectify_node`는 npz의 `fx, fy, cx, cy, dist`로 렌즈 왜곡을 편다. 카메라 개체가 다르면 이 값이 다르다. 잘못된 npz를 쓰면 rectified 영상의 픽셀 좌표가 틀어지고, 그 위에서 만든 H도 함께 틀어진다. **한 카메라의 npz를 다른 카메라에 쓰면 안 된다.**

### 4-2. 왜 카메라마다 npy(H)가 따로 필요한가

H는 "이 카메라의 rectified 픽셀 → map metre" 변환이다. 카메라 위치·각도가 다르면 당연히 다르다. 그런데 **출력 좌표계(map frame)는 같아야** 한다 — 그것을 보장하는 것이 "같은 바닥 점에 같은 실측값 입력"이다.

---

## 5. 설치 절차

### 5-1. 카메라 내부 파라미터 (각각 1회)

```bash
source ~/ros2_ws/install/setup.bash

# cam0
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p camera_id:=0 \
  -p output_file:=$HOME/cctv0_camera_calibration.npz

# cam2
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p camera_id:=2 \
  -p output_file:=$HOME/cctv2_camera_calibration.npz
```

생성된 npz를 패키지 `config/`에 넣고 다시 빌드하거나, launch에서 절대경로로 지정한다.

### 5-2. 바닥 기준점 실측 (한 번만, 두 카메라 공용)

바닥에 기준점을 표시하고 map 원점 기준 (X, Y) metre를 실측해 **적어 둔다.**

```
예)  R1 (0.00, 0.00)   R2 (5.00, 0.00)   R3 (5.00, 3.50)   R4 (0.00, 3.50)
     R5 (2.50, 0.00)   R6 (2.50, 3.50)   ← 겹침 영역의 공통점
```

**핵심 규칙 3가지:**

1. 각 카메라에서 최소 4점, 권장 6~12점.
2. **겹침 영역에 최소 2~3개의 공통점을 반드시 포함시킨다.** 두 카메라가 같은 물리 점을 같은 좌표로 보는지 확인할 유일한 수단이다.
3. 두 카메라 등록에서 **같은 점에는 반드시 같은 숫자**를 입력한다. 이것이 두 H를 같은 map frame으로 묶는 전부다.

### 5-3. Homography + 주차면 등록 — 1회차 (cam0)

```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true camera_id:=0 camera_label:=cam0 \
  cctv_raw_topic:=/cctv0/image_raw \
  cctv_rect_topic:=/cctv0/image_rect \
  cctv_camera_calib:=$HOME/cctv0_camera_calibration.npz \
  homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy
```

브라우저에서 `http://<JETSON-IP>:5001/` 접속 후:

1. **현재 영상 정지**
2. **바닥 기준점** — 점을 클릭하고 실측 X(m), Y(m)를 입력해 등록. 최소 4개.
3. **Homography 계산** — RMS 재투영 오차 확인. **2cm 이상이면 기준점을 다시 찍는다.**
4. **주차면** — cam0에 보이는 슬롯마다 모서리 4개 + 통로 쪽 점 1개.
5. **대기영역** — 모서리 4개 (1회차에서 반드시 등록).
6. **저장**

### 5-4. Homography + 주차면 등록 — 2회차 (cam2)

```bash
ros2 launch cooperative_parking_robot bev_layout_calibration.launch.py \
  enable_opencv_camera:=true camera_id:=2 camera_label:=cam2 \
  cctv_raw_topic:=/cctv2/image_raw \
  cctv_rect_topic:=/cctv2/image_rect \
  cctv_camera_calib:=$HOME/cctv2_camera_calibration.npz \
  homography_output_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \
  append_existing_layout:=true
```

`layout_output_file`은 지정하지 않으면 1회차와 같은 기본 경로(`parking_layout.yaml`)를 쓴다. **바꾸지 말 것.**

1. **현재 영상 정지**
2. **바닥 기준점** — 5-2에서 적어 둔 값을 **그대로** 입력. 겹침 영역 공통점 포함.
3. **Homography 계산** — RMS 확인.
4. **주차면** — cam2에서만 보이는 슬롯 등록. 이미 cam0에서 등록한 슬롯은 다시 찍지 않아도 된다.
5. **대기영역** — 재등록 불필요(1회차 값 유지).
6. **저장** — `append_existing_layout:=true` 덕분에 1회차 슬롯이 유지된다.

### 5-5. 정합 검증 (반드시 할 것)

두 H가 같은 map frame으로 가는지 확인하는 가장 확실한 방법:

```bash
# 두 카메라 영상 모두에 보이는 겹침 영역 바닥 점 하나를 정하고,
# 각 등록 세션의 .json 메타데이터에서 그 점의 world 좌표를 비교한다.
python3 - <<'EOF'
import json, numpy as np
for cam in ('cam0', 'cam2'):
    meta = json.load(open(f'/home/guitest/.ros/adaptive_valet_bot/homography_{cam}_rectified.json'))
    print(cam, 'RMS =', round(meta['reprojection_rms_m'], 4), 'm',
          '| max =', round(meta['reprojection_max_m'], 4), 'm',
          '| refs =', len(meta['references']))
EOF
```

- 두 카메라 모두 **RMS < 0.02m** 이면 양호.
- 실제 물체를 겹침 영역에 놓고 `/cctv/merge_status`의 `multi_camera_detections`가 1 이상, `duplicates_removed`가 1 이상이면 두 좌표계가 실제로 정합된 것이다. 0이면 두 H가 서로 다른 곳을 가리키고 있다는 뜻이므로 기준점부터 다시 확인해야 한다.

---

## 6. 실행

```bash
source ~/ros2_ws/install/setup.bash

ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  camera0_id:=0 camera2_id:=2 \
  cctv0_camera_calib:=$HOME/cctv0_camera_calibration.npz \
  cctv2_camera_calib:=$HOME/cctv2_camera_calibration.npz \
  homography_cam0_file:=$HOME/.ros/adaptive_valet_bot/homography_cam0_rectified.npy \
  homography_cam2_file:=$HOME/.ros/adaptive_valet_bot/homography_cam2_rectified.npy \
  layout_config:=$HOME/.ros/adaptive_valet_bot/parking_layout.yaml \
  model_path:=$HOME/yolov8n.pt \
  enable_debug_web:=true
```

### 6-1. 확인 명령

```bash
# 노드가 다 떴는지 — 카메라 관련 노드 이름에 _cam0 / _cam2 접미사가 붙는다
ros2 node list
#  /opencv_camera_node_cam0   /opencv_camera_node_cam2
#  /cctv_rectify_node_cam0    /cctv_rectify_node_cam2
#  /yolo_bev_map_node_cam0    /yolo_bev_map_node_cam2
#  /cctv_merge_node           /fleet_manager_node
#  /cctv_robot_marker_node    ← 1개만 있어야 정상

# 두 카메라 영상이 흐르는지
ros2 topic hz /cctv0/image_rect
ros2 topic hz /cctv2/image_rect

# 카메라별 검출이 나오는지 (coverage_polygon이 채워져 있어야 함)
ros2 topic echo /cctv0/detections --once
ros2 topic echo /cctv2/detections --once

# 병합 진단 — 가장 먼저 볼 것
ros2 topic echo /cctv/merge_status

# 최종 임무 토픽 (단일 카메라 시절과 이름/의미 동일)
ros2 topic echo /parking/map --once
ros2 topic echo /parking/empty_slots --once

# /parking/map publisher가 정확히 1개(cctv_merge_node)인지 확인
ros2 topic info /parking/map --verbose | grep -A2 "Publisher"

# 로봇 절대 pose (publisher가 1개여야 함)
ros2 topic info /front/cctv_pose --verbose | grep "Publisher count"
```

### 6-2. `/cctv/merge_status` 읽는 법

```json
{
  "stamp_ns": 1723440000000000000,
  "cameras": {
    "cam0": {"alive": true,  "age_s": 0.043, "detections": 2, "coverage_ready": true},
    "cam2": {"alive": true,  "age_s": 0.051, "detections": 1, "coverage_ready": true}
  },
  "merged_detections": 2,
  "duplicates_removed": 1,
  "multi_camera_detections": 1,
  "slots": {
    "P1": {"observed": true,  "occupied": false},
    "P4": {"observed": false, "occupied": true}
  }
}
```

| 증상 | 원인 | 조치 |
|---|---|---|
| `alive: false` | sensor 노드가 죽었거나 영상이 안 옴 | 해당 카메라의 `image_rect` hz 확인 |
| `coverage_ready: false` | H 미로드 또는 coverage 계산 실패 | 해당 카메라 npy 경로 확인 |
| `duplicates_removed: 0`인데 겹침 영역에 차가 있음 | **두 H가 서로 다른 map frame** | 기준점 실측값 재확인 (가장 흔한 실수) |
| 특정 슬롯이 계속 `observed: false` | coverage polygon 밖 | 카메라 각도 조정 또는 `coverage_margin_px` 축소 |
| `merged_detections`가 실제 차량 수의 2배 | dedup 실패 | `duplicate_center_gate_m`를 키움 |
| 빈자리가 안 뜸 | 관측 불가 슬롯 | `slots[].observed` 확인 |

---

## 7. 주요 튜닝 파라미터

| 파라미터 | 기본 | 언제 만지나 |
|---|---|---|
| `process_every_n` | 3 | 카메라 2대분 추론이 동시에 돌아 1대일 때보다 무겁다. Jetson이 버거우면 4~5로 올린다 |
| `duplicate_center_gate_m` | 0.35 | 같은 차가 2개로 보이면 ↑, 다른 차 2대가 1개로 합쳐지면 ↓ |
| `duplicate_overlap_ratio` | 0.30 | mask 기반 중복 판정 임계 |
| `duplicate_center_blend` | 0.0 | 0이면 광축 가까운 카메라 값만 사용(권장). 경계에서 위치가 튀면 0.2~0.3 |
| `camera_timeout_s` | 1.0 | 이 시간 넘게 envelope이 없으면 그 카메라를 죽은 것으로 판단 |
| `merge_rate_hz` | 10.0 | 병합 주기 |
| `require_all_cameras` | false | true면 한 대만 죽어도 `/parking/*` 발행 중단(더 보수적) |
| `require_full_slot_coverage` | false | true면 슬롯 네 모서리가 모두 한 카메라에 들어와야 판정(더 보수적) |
| `coverage_margin_px` | 8.0 | 영상 테두리 왜곡 잔차가 크면 ↑ |
| `selection_hold_s` | 0.30 | 마커 pose가 두 카메라 사이에서 자주 튀면 ↑ |

---

## 8. 하위 호환

**기존 단일 카메라 구성은 아무것도 바뀌지 않았다.**

- `cctv_server.launch.py`는 수정하지 않았다.
- `yolo_bev_map_node`의 새 파라미터 기본값은 `publish_detections=False`, `publish_mission_outputs=True` — 종전 동작 그대로.
- `cctv_robot_marker_node`는 `image_topics`가 비어 있으면 기존 `image_topic`/`homography_file` 경로를 그대로 탄다.
- `bev_layout_calibration.launch.py`는 `append_existing_layout` 기본값이 `false`라 종전과 동일하게 덮어쓴다.
- 기존 `parking_layout.yaml`에는 `cctv_merge_node:` 블록이 없다. **듀얼 구성으로 갈 때는 등록을 다시 해야 한다**(또는 해당 블록을 손으로 추가). 단일 카메라로는 그대로 써도 무방하다.
- `config/cctv_camera_calibration.npz`는 삭제하지 않았다.

---

## 9. 남은 한계 / 주의사항

1. **두 카메라 설치 높이가 다르면 마커 parallax 보정이 부정확하다.** `cctv_robot_marker_node`의 `camera_height_m`은 현재 스칼라 하나다. 높이를 다르게 설치할 계획이면 이 파라미터를 카메라별 배열로 확장해야 한다(위치 선택 로직 자체는 이미 카메라별로 동작한다).
2. **`camera_ground_points`(광축 지상점) 실측 전에는 대체 지표를 쓴다.** 카메라 선택과 중복 제거 우선순위 모두 근사값 기반이다. 동작은 하지만, 실측하면 정확도가 올라간다. 실측 방법: 카메라 바로 아래 바닥 지점의 map 좌표를 재면 된다(수직 90도 설치 전제).
3. **차량 mask는 평균하지 않는다.** 두 카메라가 본 mask를 평균하면 차량이 실제보다 커져 슬롯 적합성 판정이 보수적으로 망가지므로, 정확한 쪽 하나만 채택한다. 그래서 겹침 영역에서 차량 크기 추정은 광축 가까운 카메라 품질에 좌우된다.
4. **`require_full_slot_coverage=false`(기본)에서는 슬롯 중심만 본다.** 두 카메라 경계에 정확히 걸친 슬롯은 한 카메라만으로도 "관측 가능"이 된다. 슬롯이 카메라 경계에 걸치도록 배치하지 않는 것이 좋다.
5. **A*는 여전히 인양 직후 1회만 계산한다.** 카메라를 늘려도 주행 중 동적 재계획은 없다(`FRONT_FIRST_ENTRY_ID0_ULTRASONIC_20260725.md` 지적 사항 그대로).
6. **벽·기둥은 여전히 YOLO가 보지 못한다.** 차량 Seg 모델은 차량만 검출하므로 고정 장애물은 별도 no-go 영역으로 등록해야 한다(`BEV_SLOT_REGISTRATION_AND_PARKING.md` §9).
7. **출차(retrieve)는 여전히 미구현이다.** 이번 변경은 인지/맵 레이어만 확장했다. 출차 설계는 `MASTER_PLAN.md` Part 5 참조.
8. **Jetson 부하.** YOLO 추론이 2배가 된다. `process_every_n`을 올리거나 TensorRT engine(`vehicle_seg.engine`)을 쓰는 것을 권장한다.
9. **NTP/chrony 시간 동기화가 더 중요해졌다.** 병합 노드가 카메라 생존을 판단할 때 자기 monotonic 시계를 쓰므로 이 부분은 안전하지만, `/parking/vehicle_pose_feedback`의 `header.stamp`는 CCTV 촬영시각을 그대로 전파하므로 로봇 쪽 신선도 판정이 시계 오차에 영향을 받는다.

---

## 10. 테스트

```bash
cd ~/ros2_ws
colcon test --packages-select cooperative_parking_robot
colcon test-result --verbose

# 또는 직접
cd ~/ros2_ws/src/cooperative_parking_robot
python3 -m pytest test/test_dual_cctv_merge.py -v
```

검증 완료 상태(2026-08-12): 전체 131개 테스트 중 130개 통과, 1개는 테스트 하니스 한계로 인한 것이며 이번 변경과 무관.

---

## 11. 관련 문서

- `CCTV_CALIBRATION.md` — 캘리브레이션 원칙, 단일/듀얼 요구사항
- `BEV_SLOT_REGISTRATION_AND_PARKING.md` — 슬롯 등록, 점유 판정
- `localization_design.md` — 상판 마커 ID10/ID11, PoseEKF
- `HUMBLE_DEPLOYMENT.md` — 3대 장비 배포
- `MASTER_PLAN.md` — 전체 로드맵, 출차 설계
- `system_spec.md` — 시스템 사양, 토픽 목록
