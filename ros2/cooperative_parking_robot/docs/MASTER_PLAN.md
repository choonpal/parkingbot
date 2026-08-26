# Adaptive Valet Bot v1.9 — 실차 투입 통합 마스터 문서
## (검토 이슈 · 병목 · UI 구현 스펙 · 출차 미션 설계 · 시운전 체크리스트)

> **과거 기록 — v1.9/2026-08-05 설계 스냅샷.** 현재 기준은
> 저장소의 `docs/README.md`, `docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`,
> `docs/REAL_WORLD_READINESS.md`, `ros2/cooperative_parking_robot/README.md`를 따른다.
> 구현된 출차 통합의 세부 이력은 `docs/RETRIEVAL_MISSION_INTEGRATION_PLAN.md`와 repository 최상위 ADR에 남아 있다.

기준 코드: `adaptive_valet_bot_v1_9_bev_slot_parking_20260804` | 작성일: 2026-08-05
검증 상태: 순수 로직 테스트 117개 통과, 전 노드 문법 정상, pub↔sub seam audit 미연결 없음,
QoS 호환성 전수 확인 (transient-local 미션 토픽 포함)

이 문서 하나로 Claude Code CLI 투입 가능하도록 전체 내용을 통합했다.

---

# 목차

- **Part 0** — 우선순위 로드맵 요약 (P0~P5)
- **Part 1** — P0: 블로커 (첫 시운전 전 필수)
- **Part 2** — P1: 정밀도/수렴 이슈 (첫 캘리브레이션 세션)
- **Part 3** — P2: UI 구현 스펙 전문 (jetson_vision_web_node 확장 + 입차 게이트)
- **Part 4** — P3: 다중 미션 리셋 (출차의 전제)
- **Part 5** — P4: 출차 미션 설계 스펙
- **Part 6** — P5: 선택 개선 항목
- **Part 7** — 코드 리뷰 결과 전체 (회귀 확인 포함)
- **Part 8** — 병목/성능 분석표
- **Part 9** — 통합 시운전 체크리스트

---

# Part 0. 우선순위 로드맵 요약

| 순위 | 항목 | 성격 | 규모 | 이걸 안 하면 |
|---|---|---|---|---|
| **P0** | 블로커 4건 (rear 카메라, chrony, BEV 등록, torch CUDA) | 코드 1건 + 환경 3건 | 소 | 첫 시운전 자체가 실패 |
| **P1** | 정밀도/수렴 (parallax, EKF 첫 보정, mask 반경, 배치 규약) | 코드 소규모 + 캘리브레이션 | 소~중 | 간헐 FAULT·영구 미수렴으로 원인 추적 소모 |
| **P2** | UI 대시보드 + 입차 버튼 게이트 (web_node 확장) | 신규 (Part 3 전문) | 중 | 자동 시작 그대로 — 시연 통제 불가 |
| **P3** | 다중 미션 리셋 | 신규 | 중 | 출차 불가 + 입차도 1회 후 노드 재시작 필요 |
| **P4** | 출차 미션 | 신규 설계 (Part 5) | 중 | 출차 버튼 영구 disabled |
| **P5** | 선택 개선 (peer 회피, 벡터화 등) | 선택 | 소 | 다중 차량 확장 시 제약 |

의존 관계: **P0 → P1 → (P2 ∥ P3) → P4**. P2와 P3은 병렬 가능.
구현 순서 제안: ① 상태 대시보드만(버튼 없이) ② 입차 게이트 ③ 미션 리셋 ④ 출차.
시연 일정이 빠듯하면 "출차 = 수동 재배치 + 재입차"로 범위 축소하는 fallback을 미리 합의.

---

# Part 1. P0 — 블로커 (이동/첫 통전 전 완료. 하나라도 빠지면 시운전 100% 실패)

### P0-1. Rear 마커 카메라 발행 노드 부재 [코드]
- `aruco_tracker_node`가 구독하는 `/rear/marker_camera/image`의 **발행자가
  `rear_robot.launch.py`에 없다.** `opencv_camera_node`는 패키지에 있으나 rear launch에
  인스턴스화되지 않았다. ID0 미관측 → `WAIT_PEER_STAGED`의 `relative_is_fresh()` 게이트
  통과 불가 → ALIGN_TIMEOUT → FAULT (100% 재현).
- **조치**: rear launch에 `opencv_camera` 인스턴스 추가:
  `output_topic:=/rear/marker_camera/image`, 전용 `camera_id`, 해상도/버퍼는 기존 관례
  (1280×720, buffer 1) 유지. 외부 드라이버를 쓸 경우 시동 절차에 명문화.

### P0-2. 3대 장비 시계 동기 [환경]
- StampGate가 기계 경계를 넘는 곳: Rear `stm32_bridge` cmd_vel(stale 0.25 s /
  **future 0.10 s**), odom(0.5 s), ArUco(0.3 s), cctv_pose(0.5 s), fleet/state(2.5 s).
- future_tolerance 0.1 s가 실질 한계: **Rear 시계가 Front보다 0.1 s만 뒤져도 Master 명령
  전량 FUTURE_STAMP 거부 → Rear만 정지** — 원인 추적이 가장 어려운 유형의 증상.
- **조치**: Jetson을 chrony 기준으로 RPi 2대 동기. 시동 체크 1번 = `chronyc tracking`
  skew < 50 ms. (`docs/HUMBLE_DEPLOYMENT.md` 절차 존재 — 체크리스트로 강제)

### P0-3. BEV 등록 — Homography 현재 미보유 [환경]
- `require_homography` / `require_registered_layout` / `cctv_robot_marker`의
  FileNotFoundError로 **등록 전에는 CCTV 스택이 기동 자체를 거부**한다 (의도된 fail-safe).
- **조치**: 현장 도착 즉시 `bev_layout_calibration.launch.py`(브라우저 :5001)로
  Homography·슬롯·대기영역 등록 (`~/.ros/adaptive_valet_bot/`).
  `homography_scale_to_m:=1.0` 확인 — 틀리면 전 좌표 100배 오차.

### P0-4. Jetson torch CUDA [환경 — 유일한 실질 병목 후보]
- pip 기본 torch는 CUDA 미포함 → yolov8n@imgsz320이 ARM CPU에서 150~300 ms/회
  → 콜백 적체. imgsz 320 / 3프레임당 1회 설정 자체는 적절하다.
- **조치**: `python3 -c "import torch; print(torch.cuda.is_available())"` == True 확인.
  False면 NVIDIA JetPack용 wheel 설치. 여유 시 `.engine` export 검토.
- UI(chromium kiosk) 추가 후 `tegrastats` 재측정 — 빠듯하면 `camera_fps:=15`
  (제어 요구 대비 충분).

---

# Part 2. P1 — 정밀도/수렴 이슈 (첫 캘리브레이션 세션에서 함께 처리)

### P1-1. Parallax 파라미터 실측 입력 [캘리브레이션]
- 현재 전부 0(비활성). 카메라 2.5 m / 상판 마커 0.12 m 가정 시 **두 로봇 간 차등 위치
  오차 ≈ 3 cm = `dist_error_limit`(3 cm)와 동일 차수.**
- ID0 가시 구간은 ArUco 거리가 필터를 지배해 안전. ID0 가림 구간의
  `CCTV_ID10_ID11` 앵커에서 계통 오차 유입 → 오탐 감속/DIST_ERROR 가능.
- **조치**: `camera_height_m`, `front/rear_marker_height_m`, `camera_ground_x/y_m`,
  `vehicle_detection_height_m` 실측 입력 (P0-3과 같은 세션에서).

### P1-2. PoseEKF 첫 보정 무조건 수용 패치 [코드 소규모]
- 초기 P=0.25에서 첫 CCTV 측정 허용 오차 ≈ 위치 1.7 m. 초과 시 5회 거부 후
  강제 재수렴 경로도 reacquire 한계(0.5 m / 45°)에 막혀 **영구 미수렴**.
  증상은 `localization_status`의 `CCTV_REJECTED_GATE` 반복뿐이라 알아채기 어렵다.
- **조치(코드)**: `PoseEKF.correct()`에 `self._ever_accepted` 플래그 —
  accept 이력이 한 번도 없으면 게이트 무시하고 첫 측정 무조건 수용.
- **조치(운용)**: 시동 시 로봇을 waiting 홈 좌표에, 진행방향 대략 +x로 배치.
  UI에 REJECTED_GATE 반복 경고 배너 (Part 3의 localization_warning과 연동).

### P1-3. `robot_mask_radius` 0.25 → 0.32 [파라미터]
- 로봇 반대각 0.314 m > 현재 마스크 0.25 m. YOLO가 로봇을 차량으로 오검출 시
  마스크 밖 모서리가 팽창되어 A* 시작점 봉쇄 가능.

### P1-4. 차량 대기 자세 규약 [운용]
- 차량은 대기구역에 **map x축 대략 정렬**로 진입해야 한다. yaw≈90°면
  (i) 홈이 차량 프레임 "후방"이 아니어서 `APPROACH_START_NOT_BEHIND_VEHICLE` FAULT,
  (ii) A* envelope가 y방향 ~1.4 m로 커져 맵 경계 밴드에 시작점 잠식.
- UI 안내 문구로도 노출 (Part 3 배너).

---

# Part 3. P2 — UI 구현 스펙 전문 (jetson_vision_web_node 확장 + 입차 버튼 게이트)

대상 파일: `jetson_vision_web_node.py`, `fleet_manager_node.py`, `launch/cctv_server.launch.py`
범위: **입차 버튼 + 상태 대시보드까지.** 출차 버튼은 화면에 배치하되
`disabled + "준비 중"` (출차 본체는 Part 5, 선행 조건은 Part 4).

## 3-0. 구조 원칙 (구현 전 합의사항)

1. **web_node는 view + 버튼 발행만 담당한다.** 미션 시작 판단·게이트는 fleet_manager 소유.
   web_node 프로세스가 죽어도 로봇 동작 안전성에 영향이 없어야 한다.
2. 버튼 → ROS 발행은 **반드시 rclpy 스레드에서** (Flask 핸들러 직접 publish 금지 —
   기존 web_node의 frame_condition 패턴처럼 `queue.Queue`로 넘긴다).
3. UI 비상정지는 보조 수단. **물리 E-Stop이 1차**임을 코드 주석과 화면에 명시.
4. 기존 web_node 역할(진단 MJPEG) 불변 — 카메라를 새로 열지 않고
   `/cctv/image_rect` 구독 구조 유지.

## 3-1. 신규 토픽 계약

### `/ui/mission_request` (std_msgs/String, JSON)
```json
{
  "type": "park",            // "park" | "retrieve"(P4에서 활성) | "cancel"(P5)
  "request_id": "ui-<epoch_ms>",
  "sequence": 3,             // web_node 프로세스 내 단조 증가
  "stamp_ns": 1780000000000000000
}
```
- QoS: depth 1, RELIABLE, **VOLATILE** — 버튼은 순간 이벤트. latch(transient_local) 금지:
  web_node 재시작 시 과거 요청 재생 방지.
- 발행: 버튼 1회 탭 = 1회 발행, **자동 재전송 없음** (수신 실패 시 사용자가 다시 누른다 —
  자동 재전송은 이중 승인 위험이 실패 확률보다 크다).

### 비상정지 — 중계 토픽 없음
- UI 비상정지 버튼은 **`/emergency_stop`(std_msgs/Bool, true)에 직접 발행.**
  전용 중계 토픽을 두면 web_node가 단일 실패점이 된다. 기존 구독자 변경 불요.

## 3-2. fleet_manager_node 변경 (게이트 삽입)

파라미터:
```python
self.declare_parameter('require_ui_confirmation', True)
self.declare_parameter('ui_request_timeout_s', 10.0)   # 버튼 stamp 신선도
```

구독:
```python
self.create_subscription(String, '/ui/mission_request', self.ui_request_cb, 10)
```

`ui_request_cb`:
- JSON 파싱 실패 / `type != 'park'`(P4 전) / stamp 신선도 실패(기존 `StampGate` 재사용,
  `ui_request_timeout_s`) / sequence 역행 → warn 후 무시
- 정상 수신 → `self.ui_park_approved = True`, `self.ui_approved_time = monotonic()`,
  `self.ui_request_id` 저장(로그용)

`manage_loop` 전이 변경:
```python
if self.state == 'WAIT_TARGET':
    if self.target_pose is not None:
        if (not require_ui_confirmation) or self.ui_park_approved:
            self.state = 'WAIT_LIFT'
            self.ui_park_approved = False   # ★ 1회성 소비 — 진입 순간 즉시 리셋
```
- **승인 소비 시점이 핵심**: WAIT_LIFT 진입 시 즉시 False. 안 하면 다음 미션에
  잔존 승인이 넘어가 게이트가 무의미해진다 (테스트 3-6-③이 이를 검증).
- target 전에 버튼이 먼저 눌린 경우: 승인을 유지하되 `ui_request_timeout_s`(10 s) 내
  target_ready 미도래 시 만료 (`manage_loop`에서 `ui_approved_time` 검사).
- `publish_state` JSON에 `"ui_approved": bool`, `"ui_request_id": str` 추가 (UI 표시용).
- **회귀 조건**: `require_ui_confirmation:=false`면 기존 v1.9와 100% 동일 동작
  (launch 인자로 노출).

## 3-3. jetson_vision_web_node 변경

### 구독 추가 (전부 기존 토픽 — 신규 개발 불요)
| 토픽 | 용도 |
|---|---|
| `/fleet/state` (String JSON) | 미션 단계, ui_approved, empty_count, lifted |
| `/front/robot_state`, `/rear/robot_state` | 로봇 상태 (FAULT 강조) |
| `/front/motion_phase`, `/rear/motion_phase` | 세부 진행 (PRE_ALIGN, SCAN_IN 등) |
| `/parking/target_ready` (Bool) | 입차 버튼 활성 조건 |
| `/sync/error_state` (String JSON) | 오류 사유 |
| `/front/motion_fault`, `/rear/motion_fault` | 오류 사유 |
| `/front/localization_status`, `/rear/localization_status` | CCTV_REJECTED_GATE 연속 감지 → 경고 배너 (P1-2 조기 발견 수단) |

- 콜백은 `self._status` dict에 **(값, receipt_monotonic)** 으로만 저장.
  UI 표시 계산은 `/api/status` 핸들러에서. 각 항목에 staleness 판정(예: 3 s) —
  오래된 값은 "연결 끊김" 표시. **WiFi 너머 RPi 토픽이므로 필수.**

### 발행 추가
```python
self.pub_ui_request = self.create_publisher(String, '/ui/mission_request', qos_depth1_reliable)
self.pub_estop = self.create_publisher(Bool, '/emergency_stop', 10)
```
- Flask 핸들러 → `queue.Queue` → rclpy 타이머(20 Hz)가 큐를 비우며 발행.

### Flask 엔드포인트
| 경로 | 메서드 | 동작 |
|---|---|---|
| `/` | GET | 기존 진단 페이지 유지 |
| `/kiosk` | GET | 터치스크린 풀스크린 페이지 (신규) |
| `/api/status` | GET | 상태 JSON (UI 500 ms 폴링) |
| `/api/park` | POST | **서버측 활성 조건 재검사** 후 mission_request 발행. 결과 JSON |
| `/api/estop` | POST | `/emergency_stop` true 발행 |
| `/video` | GET | 기존 MJPEG (kiosk에 임베드) |

`/api/status` 응답 형태:
```json
{
  "fleet": {"state": "WAIT_TARGET", "empty_count": 3, "ui_approved": false, "fresh": true},
  "front": {"state": "IDLE", "phase": "IDLE", "fresh": true},
  "rear":  {"state": "IDLE", "phase": "IDLE", "fresh": true},
  "target_ready": true,
  "park_enabled": true,
  "retrieve_enabled": false,
  "fault": null,
  "localization_warning": false,
  "banner": "차량 인식 완료 — 입차 가능"
}
```

### 버튼 활성 로직 — **서버측 판정** (클라이언트 JS는 표시만)
```
park_enabled =
    target_ready == true
AND fleet.state == 'WAIT_TARGET'
AND fleet.empty_count >= 1
AND front.state == 'IDLE' AND rear.state == 'IDLE'
AND fault is None
AND 모든 관련 상태 fresh

retrieve_enabled (P4 활성 후) =
    점유 슬롯 >= 1  AND 대기구역 비어 있음(target_ready == false)
AND front/rear IDLE AND fault is None AND fresh
```
- `/api/park` POST 시 **동일 조건 재검사** (폴링 500 ms 사이 상태 변화 대비).
  불충족 시 발행하지 않고 사유 반환.
- 연타 방지: 발행 후 2 s 서버측 쿨다운.

### `/kiosk` 페이지 (1024×600 고정, Waveshare 7" Display-C)
- 단일 HTML 문자열 또는 templates/ — **외부 CDN 의존 금지** (오프라인 시연장 전제)
- 좌측 55%: MJPEG 영상 (`/video`)
- 우측 45%:
  - 상단: 상태 배너 (단계 한국어 매핑: WAIT_TARGET→"차량 대기 중",
    PLAN_PATH→"경로 계산 중", NAVIGATING→"운반 중" 등) + P1-4 배치 안내 문구
  - 로봇 2행: Front/Rear 상태 + phase (FAULT 적색)
  - 중단: **[ 입차 ]** 대형 버튼 (터치 최소 높이 96 px), **[ 출차 ]** (disabled "준비 중")
  - 하단: **[ 비상정지 ]** 적색 — **confirm 없이 즉시 발동** (비상정지에 확인창 금지),
    옆에 "물리 비상정지 스위치가 우선입니다"
- FAULT 시: 전체 배너 적색 + 사유 + "복구는 운영자 절차 필요
  (ESTOP은 STM32 전원 재인가까지 latch)" 안내
- JS: 500 ms `/api/status` 폴링, fetch 실패 시 "UI 서버 연결 끊김" 오버레이

### web_node 실행 조건
- `enable_operator_ui=true`가 Fleet 승인용 kiosk/API를 독립 실행한다.
- `enable_debug_overlay=false`가 기본이며, 진단 영상이 필요할 때만 켠다.
  웹 진단은 ArUco/FPS만 처리하고 차량 YOLO는 미션 노드에서 한 번만 실행한다.

## 3-4. 터치스크린 / kiosk 실행 (Jetson)

```bash
# HDMI + USB(터치) 연결, 해상도 1024×600 확인 (xrandr)
# 터치 축 반전 시: xinput set-prop <id> "Coordinate Transformation Matrix" ...

# chromium kiosk (systemd user service 또는 autostart, Restart=always)
chromium-browser --kiosk --incognito --noerrdialogs --disable-translate \
  --check-for-update-interval=31536000 http://localhost:5000/kiosk

# 화면 절전/블랭킹 비활성
xset s off; xset -dpms; xset s noblank
```
- systemd 순서: cctv_server.launch(web_node 포함) 기동 → chromium.
- UI 렌더링이 영상 파이프라인과 CPU 공유 → `tegrastats`로 여유 확인 (P0-4 연동).

## 3-5. P2 테스트 계획

1. **회귀**: `require_ui_confirmation:=false`로 기존 자동 시작 플로우 그대로 동작
2. **게이트**: true에서 target_ready 후 정지 상태 유지 → 버튼 탭에 WAIT_LIFT 진입
3. **승인 소비**: 미션 1회 후 승인 플래그가 남아 다음 target에 자동 시작되지 않는지 ★
4. **만료**: target 없이 버튼 → 10 s 후 승인 만료
5. **서버측 재검사**: 활성 표시 후 상태를 강제로 바꾸고 POST → 거부
6. **UI kill 테스트**: 주행 중 web_node kill → 로봇 거동 무영향 ★
7. **estop 경로**: kiosk 비상정지 → 양측 stm32_bridge ESTOP latch → 상태기계 FAULT 전파
8. **staleness**: RPi 하나 네트워크 차단 → 해당 카드 "연결 끊김" + park 비활성
9. 터치 실기: 1024×600 오탭 여부, 폴링 중 CPU (`tegrastats`)

---

# Part 4. P3 — 다중 미션 리셋 (출차의 전제. P4보다 먼저)

현재 v1.9는 "1회 사이클" 설계 — 아래 latch들이 해제되지 않아 2번째 미션이 불가능하다.

### 미션 완료 판정
- 신규 토픽 `/mission/complete` (String JSON: `mission_id`, `stamp_ns`) —
  **Front 상태기계가 RETURN→IDLE 전이 시 발행** (Front가 이미 `/mission/commit`
  publisher = 조정 권한 보유자이므로 일관).
- **transient-local 금지** (과거 완료 재생 방지). 수신측은 mission_id 일치 검사 후 리셋.

### 노드별 리셋 항목
| 노드 | 리셋 대상 | 비고 |
|---|---|---|
| `fleet_manager` | state→WAIT_TARGET, `mission_id`, `car_lifted`, `target_pose`, `vehicle_center_offset_body`, loaded_footprint 재계산 | NAVIGATING 영구 정지 해제 |
| `yolo_bev_map` | `target_latched`, `target_anchor/candidate/stable_since`, `_spec_sent`, yaw/치수 EMA, `vehicle_dimension_valid` | `/mission/complete` 구독 추가 |
| `robot_state_machine` | 변경 불요 (`reset()` 기존재) — `/mission/complete` **발행만** 추가 | ✓ |
| `rigid_body_sync` | 변경 불요 (path_cb 임무별 재초기화 기존재) | 2번째 미션 rosbag 검증 필수 |
| `ultrasonic_edge` | 변경 불요 (APPROACH 진입 리셋 기존재) | ✓ |

### transient-local 잔존값 검증
- `/virtual_robot/waypoints` · `/parking/slot_pose` · `/mission/commit`의 이전 미션 값이
  새 미션과 섞이지 않는지 — 기존 path↔slot mission_stamp 매칭이 방어하나
  **2연속 미션 rosbag으로 실증** (rigid의 vehicle_lifted + 양측 DRIVE 게이트 포함).

### P3 테스트
1. 입차 2연속 (같은 차 수동 재배치) — 사람 개입은 차 배치뿐
2. 미션 1 완료 직후 미션 2 target latch 신규 획득 (`_spec_sent` 재발행 포함)
3. 미션 2 중 미션 1 waypoints 재생 없음

---

# Part 5. P4 — 출차 미션 설계 스펙

## 5-0. 핵심 설계 결정: "nose-in 주차 강제로 진입 기하 재사용"

당초 우려는 스캔 방향/축 인덱스 전면 일반화였다. 그러나 **입차 시
`parking_direction:=forward`(전진 주차, 차량 앞머리가 슬롯 안쪽)를 강제하면**
출차 시 차량 −s(후방)가 통로를 향하므로:

> **출차 접근 = 입차 접근과 동일한 기하** (통로에서 −s staging → +s 스캔 →
> Front 2번째 축 / Rear 1번째 축). `scan_direction`, `target_axle_index`,
> `approach_longitudinal` **변경 불요.**

바닥이 흑색 종이 + 백색 테이프라 슬롯에 물리 벽이 없으므로 Front가 슬롯 안쪽
(차량 전축 아래)까지 들어가는 물리 제약도 없다.

검산 (기본 레이아웃, 슬롯 P1=(1.5, 3.5), yaw 90°):
- 통로측 staging: Front s=−0.85 → world (1.5, 2.65), Rear s=−1.55 → (1.5, 1.95) — 통로 내 ✓
- 홈 시작 자세: Front (1.15, 0.6) → 차량 프레임 s=−2.9 < −0.73 ✓ "후방" 조건 통과,
  d=+0.35, 직선 TO_REAR_STAGING이 보호 사각형 미관통 ✓
- **결론: 1대 데모 레이아웃에서 `individual_move`의 APPROACH/ALIGN 코드 무변경 동작**

따라서 출차 구현은 아래 4개 변경으로 축소된다.

## 5-1. 변경 1 — 출차 타겟 latch (`yolo_bev_map_node`)

- `/ui/mission_request` 구독 추가. `type=="retrieve"` 수신 시:
  1. 점유 슬롯 결정: 1대 데모 = 유일 점유 슬롯 자동 (다수면 UI 슬롯 선택 — P5)
  2. 해당 슬롯 polygon 내부에 중심이 있는 차량 mask 탐색 → 중심 latch
     (`target_latched` 설정; 정차 2 cm/2 s 조건은 슬롯 차량엔 생략)
  3. **yaw는 등록 슬롯 entry_yaw 사용** (차량이 슬롯 축 정렬 상태 — PCA보다 안정).
     `target_yaw` 직접 설정, `target_yaw_valid=True`
  4. `publish_target` / `_spec_sent` 재발행 → 이후 파이프라인(개별 접근, offset 초기화,
     A* 시작점, 타겟 마스킹) 기존 코드 그대로 작동
- 타겟 마스킹: latch 중심 반경 0.30 m 게이트가 mask 중심(=latch점)을 제거하므로
  슬롯 차량이 A* 장애물에서 정상 제외 ✓ (기존 코드 재사용)

## 5-2. 변경 2 — Fleet 목적지 분기: "대기구역 = pseudo-slot" (`fleet_manager_node`)

- `/ui/mission_request` 구독, `mission_type ∈ {park, retrieve}` 상태 보유.
- **대기구역을 pseudo-slot으로 정의**해 기존 파이프라인 전체 재사용:
  `ParkingSlot(id='WAIT', center=대기구역 중심, yaw=0(x축 정렬), size=…)`.
  `make_approach_candidates` → staging/회전/삽입, `_rotation_space_free`,
  `_insertion_corridor_free`, `/parking/slot_pose` 발행까지 동일 경로.
- **fit 검사 예외 (함정)**: 대기구역 테이프(0.4×0.6 m) < loaded footprint(≈1.39×0.47 m).
  벽이 없으므로 retrieve 목적지에는 `check_slot_fit` **스킵** 또는 pseudo-slot 치수를
  `footprint+margin`으로 정의. 안 하면 출차가 "적합 슬롯 없음"으로 조용히 거부된다.
- retrieve일 때 `empty_slots` 요구 제거 (목적지가 슬롯이 아님).

## 5-3. 변경 3 — A* 시작점 함정 회피: "역-staged 추출" (**본 설계의 유일한 신규 기하**)

**문제**: 슬롯 중심(y=3.5)은 맵 상단(4.0 m)에 가깝고, yaw 90° 유지 시 loaded envelope
y 반폭 ≈ 0.69 m → A* `_inflate`의 맵 경계 밴드(y ≥ 3.3 m)가 **시작점을 봉쇄**한다.
슬롯 중심에서 A*를 직접 시작하면 `plan()`이 무조건 None.

**해법**: 입차의 staged 삽입을 그대로 뒤집는다.
- fleet은 A*를 **점유 슬롯의 통로측 staging point**(`make_approach_candidates`로 계산)에서
  시작해 대기 pseudo-slot의 staging까지 계획.
- **rigid_body_sync 무변경**: pure pursuit이 현재 위치(슬롯 중심)를 경로 polyline에
  투영하면 최근접점 = 경로 시작점(슬롯 staging)이 되어, 첫 이동이 자연스럽게
  **슬롯 축을 따라 통로로 빠져나오는 추출(extraction) 구간**이 된다.
  yaw는 hold_initial_yaw로 슬롯 yaw 유지 → 메카넘 후진/횡이동 추출 ✓.
- **추출 corridor 안전 검증**: 기존 `_insertion_corridor_free`를 슬롯 중심→staging
  방향으로 동일 호출 (같은 함수, 인자 역순). 실패 시 미션 거부.
- 속도: 추출 구간은 max_speed 0.08 m/s (삽입은 final_speed_ratio 0.30으로 더 느림).
  대칭 저속화를 원하면 P5 — 기능상 필수 아님.

**최종 정렬**: 대기 staging 도착 → `align_to_slot_yaw`로 yaw 0 회전 → x축 저속 삽입 →
대기구역 중심 도착. 기존 FINAL_APPROACH 코드 무변경.
(fleet `_rotation_space_free`가 대기 staging 회전 공간 사전 검증 ✓)

## 5-4. 변경 4 — 하차 후 복귀 방향 (`individual_move` 파라미터)

- 하차 지점 = 대기구역(yaw 0) → 분할 이탈: Front +x, Rear −x —
  기존 `exit_longitudinal_translation` 그대로 ✓.
- `plan_return_home` 회피 대상 = `/parking/slot_pose`(=대기 pseudo-slot pose) 주변 차량 —
  fleet이 slot_pose로 대기 pose를 발행하므로 **자동으로 올바른 차량 회피** ✓.
- 홈이 대기구역 −x측(0.45/1.15) → Rear는 −x 이탈 후 거의 제자리, Front는 차량 우회 복귀.
  `entry_side` 방향이 입차와 동일하게 유효한지 **리허설 확인**
  (필요시 미션별 `entry_side` 전환 파라미터).

## 5-5. 출차 강제 전제 조건 (설정 고정)

| 항목 | 값 | 이유 |
|---|---|---|
| `parking_direction` | `forward` (입차 시) | nose-in이어야 출차 진입 기하 재사용 (5-0) |
| P3 완료 | 필수 | latch 미해제 시 2번째 미션 자체 불가 |
| 대기구역 비어 있음 | UI 활성 조건 | 목적지 점유 시 출차 버튼 비활성 |
| 슬롯 staging 통로 확보 | 레이아웃 규약 | staging (1.5, 2.65)·(1.5, 1.95) 장애물 금지 |

## 5-6. 출차 잔여 리스크 (다중 차량 확장 시)

- `latch_target_and_plan`의 직선 TO_REAR_STAGING은 **타겟 차량 보호 사각형만** 검사 —
  다른 주차 차량 미고려. 다중 차량 운용 시 홈→슬롯 staging 구간에 로봇 단독 A*
  경유점 필요 (P5). 1대 데모는 5-0 검산대로 무변경 안전.
- 인접 슬롯 점유 시 Front의 슬롯 안쪽 스캔 구간 간섭 — 슬롯 폭 0.70 m vs 로봇 폭
  0.275 m로 기하 여유 있으나 리허설 확인 항목.

## 5-7. 출차 테스트 계획

1. 시뮬레이션(가능하면 gz_ws 포팅) 또는 dry-run: retrieve latch → staging 좌표 검증
2. 추출 구간 단독: 슬롯 중심에서 경로 투영이 staging으로 향하는지 (rosbag 궤적)
3. **A* 시작점 검증**: 슬롯 staging 시작 계획이 None이 아님 (5-3 함정 회피 증명)
4. 대기 pseudo-slot fit 스킵 경로 동작
5. 전체 순환: 입차(forward 강제) → 출차 → 대기 차량이 다시 target_ready로 잡히고
   **P2 게이트가 자동 재시작을 막는지**
6. 하차 후 복귀 궤적의 peer/차량 간섭

---

# Part 6. P5 — 선택 개선 (시연 후)

- RETURN/접근 경로의 peer + 전체 맵 장애물 회피 (다중 차량 전제)
- 출차 추출 구간 저속화 (삽입과 대칭)
- UI 슬롯 선택 화면 (다중 점유 슬롯)
- cancel 버튼 — 미션 중 안전 정지 시퀀스 설계 필요 (단순 estop과 별개)
- `mask_world_polygon` → `cv2.perspectiveTransform` 벡터화
- rigid PID dt 실측 반영 (현재 0.02 고정)

---

# Part 7. 코드 리뷰 결과 전체

## 7-1. 회귀 없음 확인 (기존 확정 버그들)

| 항목 | 상태 |
|---|---|
| Kalman predict delta 전파 (`reset(raw_value=...)` 포함) | ✓ 방어됨 |
| PID 보정 Front/Rear 50/50 분배 | ✓ |
| `/{role}/wheel_aligned` role 네임스페이스 | ✓ |
| STM32 16-bit 엔코더 wraparound / 카운터 리셋 방어 | ✓ (펌웨어 + `EncoderOdometry` 이중) |
| 로봇 self-mask / 타겟 차량 A* 마스킹 | ✓ (단 P1-3 반경 조정) |
| pub↔sub seam / QoS 호환성 (transient-local 포함) | ✓ 전수 확인, 미연결 없음 |
| `cmd_vel` 이중 발행자 (rigid_body_sync vs individual_move) | ✓ robot_state로 시간 분리 — 충돌 구간 없음 |
| 순수 로직 테스트 | 117개 전부 통과 |

## 7-2. 낮은 우선순위 / 참고

- rigid_body_sync PID dt 0.02 고정 (타이머 지터 미반영) — 현 이득에서 영향 미미
- Pure Pursuit 최종 감속 구간 저속 크리핑 가능 — `final_approach_dist`(2 cm)가
  먼저 걸려 실害 없음
- transient-local `vehicle_spec`을 놓친 노드는 기본 wheelbase 0.70 사용 —
  고정 휠베이스 운용이라 무해

---

# Part 8. 병목 / 성능 분석표

| 구간 | 판정 | 비고 |
|---|---|---|
| **Jetson YOLO 추론** | ⚠ 유일한 실질 병목 후보 | pip torch = CPU 추론 150~300 ms/회 → 콜백 적체. `torch.cuda.is_available()` 확인 필수 (P0-4). imgsz 320 / 3프레임당 1회 설정 자체는 적절 |
| Jetson 영상 전송 (raw 30 fps, 구독 3곳) | ✓ | Humble 기본 FastDDS shared-memory로 intra-host 감당. rectify remap + ArUco 매 프레임 CPU는 `tegrastats` 실측, 빠듯하면 `camera_fps:=15` |
| **UI 추가 부하 (chromium kiosk + MJPEG)** | ⚠ 측정 필요 | YOLO CUDA 미확보 상태에서 UI까지 얹으면 위험 — P0-4 선행 후 측정 |
| RPi 50 Hz 루프 (sync/bridge/move) | ✓ | 경량 연산 |
| UART 115200 | ✓ | 사용량 ~3 kB/s ≪ 11.5 kB/s |
| Python A* (120×80, 미션당 1회) | ✓ | 수백 ms 이내 |
| WiFi (cmd_vel/odom 50 Hz) | ✓ | 이미지 미경유. `/sync/relative_pose`는 BEST_EFFORT 드롭 허용 설계 |
| `mask_world_polygon` 점별 루프 | 참고 | 필요시 벡터화 (현재 규모 불요) |

---

# Part 9. 통합 시운전 체크리스트 (순서대로)

1. [ ] `chronyc tracking` 3대 skew < 50 ms **(P0-2)**
2. [ ] `ros2 topic hz /rear/marker_camera/image` 정상 **(P0-1)**
3. [ ] `torch.cuda.is_available() == True` **(P0-4)**
4. [ ] BEV 등록 완료 + `homography_scale_to_m:=1.0` **(P0-3)**
5. [ ] Parallax 실측값 입력 **(P1-1)**
6. [ ] 로봇 홈 좌표·+x 배치 → `localization_status` source=CCTV_ARUCO **(P1-2)**
7. [ ] `robot_mask_radius:=0.32` **(P1-3)**
8. [ ] `parking_direction:=forward` — 출차 대비 지금부터 고정 **(P4/5-5)**
9. [ ] `hardware_preflight` 3대 통과
10. [ ] 차량 없는 dry-run: APPROACH→ALIGN 진입 (초음파/ID0/상판 마커 스트림)
11. [ ] 차량 x축 정렬 배치 규약 **(P1-4)**
12. [ ] 1회 전체 사이클 rosbag → RETURN 궤적 교차 검토
13. [ ] UI 도입 후: 게이트/FAULT 표시/비상정지 + web_node kill 테스트 + `tegrastats` **(P2)**
14. [ ] P3 후: 입차 2연속 rosbag (transient-local 잔존값 검증)
15. [ ] P4 후: 입차→출차→재인식 순환 + A* 시작점(슬롯 staging) 로그 확인
