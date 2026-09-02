# v1.10 변경 내역 — P0 블로커 / P1 정밀도 / P2 터치 UI / P3 임무 리셋

> **과거 기록 — v1.10 변경 스냅샷.** 본문의 미구현 항목과 테스트 수는 당시 이력이다.
> 현재 기준은 저장소의 `docs/README.md`, `docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`,
> `ros2/cooperative_parking_robot/docs/README.md`를
> 따르며, 출차 통합의 세부 이력은 `docs/RETRIEVAL_MISSION_INTEGRATION_PLAN.md`와 repository 최상위 ADR에 남아 있다.

기준: v1.9 (`adaptive_valet_bot_v1_9_bev_slot_parking_20260804`)
테스트: 128개 통과 (v1.9 117개 + 신규 11개)
**P4(출차)는 미구현이다.** 설계는 마스터 문서 Part 5 참조.

---

## P0-1. Rear 마커 카메라 발행 노드 추가 [블로커 해소]

**증상**: `aruco_tracker_node`가 구독하는 `/rear/marker_camera/image`에 발행자가 없어
ID0이 영원히 관측되지 않고 `WAIT_PEER_STAGED` → ALIGN_TIMEOUT → FAULT.

**변경**: `launch/rear_robot.launch.py`, `launch/full_system.launch.py`에
`opencv_camera` 노드 추가 (`rear_marker_camera_node`).

신규 launch 인자:

| 인자 | 기본값 | 비고 |
|---|---|---|
| `enable_rear_camera` | `true` | 외부 ROS 카메라 드라이버를 쓰면 `false` |
| `rear_camera_id` | `0` (rear launch) / `1` (full_system) | V4L2 index |
| `rear_camera_gst` | `''` | 채우면 GStreamer 파이프라인 사용 |
| `rear_camera_width` / `_height` / `_fps` | 1280 / 720 / 30.0 | |

확인: `ros2 topic hz /rear/marker_camera/image`

---

## P1-2. PoseEKF 첫 절대측정 초기화 [설계 수정 포함]

**증상**: `init_*`(도킹 홈 명목값)과 실제 배치 오차가 chi2 게이트(초기 P=0.25 기준
위치 약 1.7 m)를 넘으면, reject streak → 강제 재수렴 경로마저 reacquire 한계
(0.5 m / 45°)에 막혀 **필터가 영구 미수렴**. 증상은 `CCTV_REJECTED_GATE` 반복뿐.

**변경**: `PoseEKF.correct()`에 첫 유효 측정을 *보정*이 아니라 *절대 초기화*로 처리하는
분기 추가. 상태를 측정값으로 이식하고 `P = R`로 설정.

> **당초 계획에서 수정된 부분**: 마스터 문서에는 "첫 보정은 게이트 무시하고 무조건 수용"
> 이라고 적었으나, 구현 후 기존 테스트(`test_extreme_repeated_outlier_cannot_force_jump`)가
> 실패했다. 확인 결과 **테스트가 옳았다** — 무조건 수용이면 첫 프레임의 마커 오검출
> `(100,100)`이 그대로 초기 pose가 된다. 따라서 chi2 게이트만 건너뛰고
> **작업공간 범위 검사는 유지**하도록 스펙을 수정했다.

신규 파라미터: `PoseEKF(first_fix_max_position_m=10.0)`
— 주차장 전체가 6×4 m이므로 정상 배치는 절대 막지 않고 명백한 쓰레기 좌표만 거른다.

부가:
- `PoseEKF.initialized` 프로퍼티
- `pose_fusion_node`: 첫 수용 시 `source='CCTV_INITIALIZED'` + 좌표 로그
  (배치 오차를 현장에서 즉시 확인 가능), `localization_status`에 `initialized` 필드

테스트: `test_first_fix_initializes_beyond_chi2_gate`,
`test_first_fix_still_rejects_out_of_workspace` 추가.
`test_bounded_consistent_measurements_can_reacquire`는 "이미 수렴한 필터" 전제를
명시하도록 프라이밍 단계 추가 (재획득 게이트의 원래 의도는 보존).

---

## P1-3. `robot_mask_radius_m` 0.25 → 0.32

로봇 외곽 0.565×0.275 m의 반대각은 0.314 m. 기존 0.25 m는 오검출된 로봇의 모서리를
덮지 못해 팽창 후 A* 시작점을 막을 수 있었다.

---

## P2. 터치 UI + 입차 승인 게이트

### 구조 원칙
`jetson_vision_web_node`는 **view**다. 상태를 렌더링하고 두 개의 운용 의도
(`/ui/mission_request`, `/emergency_stop`)를 전달할 뿐, 임무 시작 여부는 판단하지 않는다.
판단은 `fleet_manager_node`가 소유하므로 **UI 프로세스를 kill해도 로봇 거동은 불변**이다.

### 신규 토픽
`/ui/mission_request` (std_msgs/String, JSON) — depth 1, RELIABLE, **VOLATILE**
```json
{"type":"park","request_id":"ui-<epoch_ms>","sequence":3,"stamp_ns":...}
```
latch를 쓰지 않는 이유: web_node 재시작 시 과거 버튼이 재생되면 안 된다.
비상정지는 중계 토픽 없이 `/emergency_stop`에 직접 발행한다 (web_node가 단일 실패점이
되는 것을 피한다).

### fleet_manager_node
- 파라미터 `require_ui_confirmation`(기본 true), `ui_request_timeout_s`(기본 10.0)
- `ui_request_cb`: JSON/타입/sequence 역행/stamp 신선도/현재 state 검증
- `manage_loop`의 WAIT_TARGET 전이에 승인 조건 추가.
  **승인은 WAIT_LIFT 진입 순간 즉시 소비** — 안 하면 다음 차량이 버튼 없이 실려 나간다.
- 차가 없는데 버튼이 먼저 눌린 경우 10 s 후 승인 만료
- `/fleet/state`에 `has_target`, `require_ui_confirmation`, `ui_approved`,
  `ui_request_id` 추가
- **회귀**: `require_ui_confirmation:=false`면 v1.9와 동일 동작

### jetson_vision_web_node
구독(전부 기존 토픽): `/fleet/state`, `/parking/target_ready`, `/sync/error_state`,
`/{role}/robot_state`, `/{role}/motion_phase`, `/{role}/motion_fault`,
`/{role}/localization_status`
- 모든 콜백은 (값, 수신시각)만 저장. 표시 판단은 `build_status()` 한 곳에서
  → staleness(기본 3 s) 기준이 일관된다. WiFi 너머 RPi 토픽이라 필수.
- Flask 스레드에서 직접 publish하지 않는다: `queue.Queue` → 20 Hz rclpy 타이머

엔드포인트:

| 경로 | 동작 |
|---|---|
| `/kiosk` | 1024×600 터치 화면 (외부 CDN 없음) |
| `/api/status` | 상태 JSON (500 ms 폴링) |
| `/api/park` | 서버측 조건 **재검사** 후 발행, 2 s 쿨다운 |
| `/api/estop` | `/emergency_stop` 발행 |

`park_enabled` = target_ready ∧ fleet WAIT_TARGET ∧ empty_count ≥ 1 ∧ 양측 IDLE
∧ fault 없음 ∧ 모든 상태 fresh. 클라이언트 JS는 표시만 하고, POST 시점에 서버가
다시 검사한다(폴링 사이 상태 변화 대비).

kiosk 화면: 좌 56% MJPEG / 우 44% 배너·로봇 2카드·입차(96 px+)·출차(disabled)·비상정지.
**비상정지에는 확인창을 두지 않는다.** "물리 비상정지 스위치가 우선" 문구 상시 노출.
localization 경고(`CCTV_REJECTED_GATE` 연속 5회)는 배너에 표시 — P1-2 조기 발견 수단.

신규 파라미터: `enable_operator_ui`(true), `status_stale_s`(3.0),
`ui_button_cooldown_s`(2.0), `localization_warning_streak`(5)

### launch (`cctv_server.launch.py`)
`require_ui_confirmation`, `ui_request_timeout_s`, `enable_operator_ui`,
`ui_status_stale_s`, `ui_button_cooldown_s` 노출.
후속 통합본에서는 kiosk를 `enable_operator_ui:=true` 기본값으로 독립 실행하고
`enable_debug_overlay:=false`를 유지한다(미션 노드와 YOLO 이중 추론 금지).

---

## P3. 다중 미션 리셋

v1.9는 1회 사이클 설계라 fleet이 NAVIGATING에서 영구 정지하고 yolo의 `target_latched`/
`_spec_sent`가 해제되지 않아 **두 번째 임무가 불가능**했다.

### 신규 토픽 `/mission/complete` (String JSON)
`{"mission_id": ..., "stamp_ns": ...}` — **Front 상태기계가 RETURN→IDLE 전이 시 발행.**
Front가 이미 `/mission/commit` 발행자(조정 권한자)이므로 완료 선언도 같은 노드가 맡는다.
transient-local을 쓰지 않는다: 나중에 기동한 노드가 과거 완료를 새 완료로 오인하면 안 된다.
`reset()`이 `active_mission_id`를 지우므로 **발행이 reset보다 먼저** 온다.

### 수신측 리셋
- `fleet_manager`: state→WAIT_TARGET, `mission_id`, `car_lifted`, `target_pose`,
  `empty_slots`, `path_published`, UI 승인, `vehicle_center_offset_body`(→0,0),
  loaded_footprint 재계산 + planner footprint 재설정, `target_gate.reset()`
  — mission_id 일치 + stamp 신선도 + state 검증 후에만 수행
- `yolo_bev_map`: `target_latched`, anchor/candidate/stable_since/last_seen,
  yaw·치수 EMA, `vehicle_dimension_valid`, `_spec_sent` 해제 + `target_ready=false` 발행
- `robot_state_machine` / `rigid_body_sync` / `ultrasonic_edge`: 기존 리셋 경로로 충분 (변경 없음)

---

## 신규 테스트 (`test/test_ui_gate_and_mission_reset.py`)

승인 게이트 6건 / 임무 리셋 3건. 특히 다음 두 가지는 회귀 시 증상이
"조용한 오작동"이라 명시적으로 고정했다:
- `test_approval_is_consumed_once` — 두 번째 차량이 버튼 없이 실려 나가는 것 방지
- `test_spec_is_republished_for_each_mission` — Fleet이 이전 차량 제원으로 계획하는 것 방지

---

## 검증 현황

| 항목 | 상태 |
|---|---|
| 전 노드/launch/test 문법 | ✓ |
| 단위 테스트 | ✓ 128 passed |
| 신규 토픽 pub↔sub seam | ✓ 양방향 연결 확인 |
| `build_status()` 시나리오 8종 (정상/차량없음/만차/이동중/운반중/FAULT/통신끊김/위치경고) | ✓ 의도대로 동작 |
| **ROS2 런타임 통합** | ✗ 미실시 — 실기 필요 |
| **Flask/kiosk 실기 렌더링** | ✗ 미실시 — Jetson+LCD 필요 |

---

## 실기 검증 항목 (이번 변경분)

1. `require_ui_confirmation:=false` 회귀 — v1.9와 동일 동작
2. true에서 target_ready 후 정지 유지 → 버튼 탭에 WAIT_LIFT 진입
3. 미션 1회 후 승인 잔존 없음 (두 번째 차량 자동 시작 안 됨)
4. target 없이 버튼 → 10 s 후 만료
5. 활성 표시 후 상태 변경 → POST 거부
6. **주행 중 web_node kill → 로봇 거동 무영향**
7. kiosk 비상정지 → 양측 STM32 ESTOP latch → 상태기계 FAULT 전파
8. RPi 1대 네트워크 차단 → 해당 카드 stale 표시 + park 비활성
9. **입차 2연속** (P3) — transient-local 잔존값 미혼입 텔레메트리 확인
10. `tegrastats`로 kiosk 추가 부하 확인 (P0-4 torch CUDA 확보 후)

---

## P0-1 보강. `calibrate_camera` 도구 추가

P0-1으로 rear 카메라 **발행 노드**는 해결했으나, `aruco_tracker_node`가
`rear_camera_calibration.npz`를 요구하고(`allow_uncalibrated` 기본 false)
그 파일을 만들 수단이 패키지에 없었다. 즉 카메라는 켜지지만 ArUco 노드가
FileNotFoundError로 죽어 ID0 경로가 그대로 막힌 상태였다.

**추가**: `cooperative_parking_robot/calibrate_camera_node.py`
(entry point `calibrate_camera`) — 체커보드 기반 내부 파라미터 캘리브레이션.

```bash
# Rear RPi에서 카메라 노드가 떠 있는 상태로 실행
ros2 run cooperative_parking_robot calibrate_camera --ros-args \
  -p image_topic:=/rear/marker_camera/image \
  -p output_path:=~/ros2_ws/src/cooperative_parking_robot/config/rear_camera_calibration.npz \
  -p board_cols:=9 -p board_rows:=6 -p square_size_m:=0.025
```

설계상 주의점:
- `board_cols`/`board_rows`는 **내부 코너 개수**다 (10x7 칸 보드 → 9x6).
- 정사각 보드(cols == rows)는 회전 모호성 때문에 거부한다.
- `min_sample_interval_s`(1.0)로 같은 자세 연사를 막는다 — 다양한 자세가
  없으면 해가 나빠진다.
- **RMS 재투영 오차가 `max_rms_error_px`(1.0)를 넘으면 저장하지 않는다.**
  나쁜 해를 파일로 남기면 이후 ArUco 거리 추정이 조용히 틀어진다.
- `camera_matrix`/`dist_coeffs` 키로 저장 — `camera_calibration.load_camera_calibration`
  왕복 검증 완료.

검증: 인코딩 변환 3종(bgr8/rgb8/mono8) + 미지원 거부, 가상 카메라
투영→복원으로 fx/cx 참값 복원 확인, 로더 왕복 확인.

### 캘리브레이션 필요 범위

| 대상 | 상태 |
|---|---|
| CCTV (`cctv_camera_calibration.npz`) | 동봉됨 — 실측값 (fx≈708, cx≈664, 1280x720). **단 현재 천장에 단 카메라와 동일 기종인지 확인 필요** |
| Rear 전면 (`rear_camera_calibration.npz`) | **없음 — 위 도구로 생성해야 함** |
| Front 전면 | 불필요 (설계상 미사용) |

내부 파라미터(intrinsic)와 BEV Homography는 별개다. Homography는
`bev_layout_calibration.launch.py`로 따로 등록한다 (P0-3).

---

## P0-1 보강 2. Rear 카메라 부하 실측 및 기본값 확정

**문제**: 추가한 rear 카메라 노드의 기본값을 1280x720@30fps로 뒀는데,
`aruco_tracker_node`에는 **프레임 스킵이 없다**(매 프레임 detectMarkers).
RPi 4에서 이는 적체를 일으킨다.

실측 (흑색 바닥 + 백색 테이프 + 마커 유사 장면, RPi4는 컨테이너 대비 4배로 추정):

| 해상도 | RPi4 추정 | 최대 fps |
|---|---|---|
| 1280x720 | 54.2 ms | 18.5 |
| 960x540 | 38.5 ms | 26.0 |
| 640x360 | 21.3 ms | 46.9 |

해상도를 낮추면 CPU는 남지만 마커 pose 정확도가 떨어진다
(0.05 m 마커, 운용 거리 약 0.7 m 기준):

| 해상도 | 0.7 m에서 마커 크기 | 1.0 m에서 |
|---|---|---|
| 1280x720 | 50.6 px | 35.4 px |
| 640x360 | 25.3 px | **17.7 px (위험)** |

`dist_error_limit`가 3 cm이므로 정확도를 희생할 여지가 없다.
따라서 **해상도는 유지하고 프레임률을 낮추는 쪽**으로 확정했다.

**기본값: 1280x720 @ 12 fps** → 1코어의 약 65 % (30 fps는 163 %로 적체).
상대 pose 신선도 게이트가 0.3 s이므로 12 Hz는 충분하다.

## P0-1 보강 3. 캘리브레이션 해상도 불일치 방어

`camera_calibration.scale_camera_matrix`가 있었으나 `aruco_tracker_node`가
**쓰지 않았다.** 캘리브레이션 해상도와 운용 해상도가 다르면 초점거리가 그만큼
틀린 채로 거리만 조용히 어긋난다.

**변경**: 첫 프레임에서 1회 검사.
- `calibrate_camera`가 저장한 `image_width`/`image_height`와 비교
- 같은 화면비면 초점거리 스케일 보정 후 warn 로그
- **화면비가 다르면 크롭이므로 보정 불가 → RuntimeError로 기동 중단**
- 해상도 정보가 없는 구형 npz면 경고만 남김

검증: 1280x720 → 640x360 스케일이 fx/cx 정확히 1/2, 16:9 → 4:3 요청은
ValueError로 거부됨을 확인.
