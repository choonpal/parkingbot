# 주차로봇 프로젝트 — 인수인계 문서

> 이 문서는 프로젝트 목표, 시스템 구조, 역할 분담, 소프트웨어·통신 설계를 설명하는 기준 문서입니다.
> 맥락: 대학 한이음 드림업(ICT 멘토링) 프로젝트.
> 최종 갱신: 2026-08-19
> 함께 보는 기준 문서: 부품·수량·구매 상태는 `BOM.md`, 조립·배선·핀맵은 `ELECTRICAL_WIRING.md`, 전체 문서 우선순위는 이 폴더의 `README.md`를 따른다. 호스트별 로그인·설치 정보는 비공개 운영 기록에서 관리하며, 상세 벤치 수치는 `TEST_LOG.md`에만 기록한다.

---

## 1. 프로젝트 개요

**천장 카메라(CCTV)가 주차장 전역을 인지·판단하고, 두 대의 로봇이 차량의 앞·뒤축을 각각 들어 올려 빈 주차칸까지 옮겨 주차하는 시스템.**

운전자가 차량을 입구(대기공간)에 두고 가면 인식·이동·주차의 전 과정이 자동으로 이루어진다. 차량은 축소 모형차를 사용한다.

핵심 발상은 **"로봇을 똑똑하게"가 아니라 "주차장을 똑똑하게"**. 상용 주차로봇은 로봇마다 LiDAR·자율주행 연산을 탑재해 비싸지만, 이 프로젝트는 그 지능을 천장 카메라와 중앙 컴퓨터로 옮겨 로봇 자체는 단순·저렴하게 만든다.

---

## 2. 시스템 구조

중앙집중형(인프라–로봇 분리) 구조.

| 구성 | 위치 | 역할 |
| --- | --- | --- |
| 천장 카메라 (OV2710)×2 | 주차장 천장, 시야 부분 중첩 | 담당 구역의 차량·로봇·빈자리 인식. 두 시야를 공통 좌표계로 정합해 하나의 전역 지도로 통합 |
| Jetson Orin Nano ×1 | 천장(중앙 서버) | 비전 처리(검출·분류), 좌표 변환, 경로 계획, 전체 관제 |
| 라즈베리파이 4 ×2 | 각 로봇 | 중앙 명령 수신, 센서 처리, 모터 지휘 (로봇 상위 두뇌) |
| STM32 (Nucleo F401RE) ×2 | 각 로봇 | 메카넘 4륜 PID 제어, 서보 제어 (하위 제어기) |
| WiFi | 전체 | 중앙↔로봇, 로봇↔로봇 통신 |

**데이터 흐름:** 천장 카메라 → Jetson(인식·좌표변환·경로계산) → WiFi → 라즈베리파이4(명령 수신·상태 관리·UART 변환) → STM32(메카넘 역기구학·모터 PID) → 엔코더 피드백

로봇은 스스로 길을 찾지 않는다. 천장 카메라가 모든 위치를 알고 "여기로 가라"고 지시하면 로봇이 따른다.

**마커 구성:** Front 로봇 후방에 ArUco ID0 부착. Rear 로봇 전방 카메라가 이 ID0을 관측. Rear에는 마커를 부착하지 않는다(단방향 관측).

---

## 3. 로봇 2대 역할 분담 (Front / Rear)

두 로봇은 대칭이 아니라 역할이 다르다.

|  | Front 로봇 | Rear 로봇 |
| --- | --- | --- |
| 현재 호스트명 | `robot-2` | `robot-1` |
| 담당 | 차량 앞축(앞바퀴) | 차량 뒷축(뒷바퀴) |
| ArUco 마커 | 로봇 **뒷면**에 부착(ID0) | 마커 없음 |
| 전면 카메라 용도 | 주행·정렬 보조 | **Front 뒷면 마커 관측** |
| 정렬 순서 | ② Rear 기준 상대위치로 앞바퀴 이동 | ① **먼저 정렬** — 기준 제공 |

각 로봇은 바퀴를 잡는 그리퍼(arm)가 좌우 하나씩 총 2개. 서보 1개가 그리퍼 1개를 닫아 바퀴 1개를 인양(로봇당 서보 2개).

**왜 이렇게 나누나:** Rear가 먼저 초음파로 뒷바퀴 중심을 잡아 기준이 된다. Rear 전면 카메라가 Front 후방의 ArUco ID0을 관측해 두 로봇의 상대 위치를 계산하고 WiFi로 공유하면, Front는 이 상대 위치와 차량 휠베이스 정보를 이용해 앞바퀴 위치로 이동한다. Front는 ArUco를 직접 관측하지 않는다.

---

## 4. 동작 과정 (6단계)

1. **인지·제원 파악** — 차량이 입구에 들어오면 천장 카메라가 YOLO11n으로 검출, 검출 영역을 잘라 EfficientNetV2-B0로 차종 분류. 차종을 알면 휠베이스·타이어 크기를 알 수 있다. 이 정보를 두 로봇에 전달, 로봇은 차량 측면으로 접근.
2. **순차 정렬** — ① Rear가 먼저 초음파로 뒷바퀴를 훑어(거리 변곡점) 타이어 중심을 역산해 정지. ② Rear가 Front 후방의 ArUco ID0을 관측해 상대 pose를 공유. ③ Front가 공유된 상대 pose와 휠베이스를 기준으로 앞바퀴 중심으로 이동한 뒤 자기 초음파로 재확인. 이 동안 천장 카메라는 빈자리를 미리 찾아둔다.
3. **인양·보고** — 두 로봇이 동시에 그리퍼를 조여 바퀴를 인양(동시에 들어야 차가 안 기울어짐). 각자 "들었음"을 보고하면 카메라가 가까운 빈 주차칸을 목표로 정한다.
4. **경로 계획** — 천장 카메라가 전역 점유 격자에서 목표까지 충돌 없는 경로를 계산해 waypoint 목록을 로봇에 내려보낸다.
5. **협조 주행** — 두 로봇이 차를 든 채 한 몸처럼 이동. Rear가 Front 마커로 위치 오차를 계산하고 엔코더와 합쳐(칼만 필터) 동기를 맞춘다. 천장 카메라는 든 차량을 추적해 실위치를 피드백하고 로봇 간격도 감시한다.
6. **하차·복귀** — 목표 도착 시 그리퍼를 풀어 차를 내려놓고 입구로 복귀.

---

## 5. 제어 구조 핵심 결정

최종 자동주행 구현에서 지켜야 할 목표 설계다. 현재 벤치용 STM32 수동 제어와 ROS2 전체 통합은 구분한다.

**5-1. 경로 계획 — Nav2 미사용**
천장 카메라가 전역 위치를 직접 제공하므로 로봇의 자기위치추정(AMCL)·로컬 플래너·Nav2 Controller를 쓰지 않는다. 중앙 Jetson이 OccupancyGrid 기반 A* 경로계획을 수행하고 waypoint 목록을 로봇에 전달한다.
`CCTV BEV Map → OccupancyGrid → A* → Waypoints → rigid_body_sync_node`

**5-2. ArUco — 단방향 구조**
`Front 후방 ArUco ID0 → Rear 전방 카메라 관측 → Rear가 상대 위치·yaw 오차 계산 → ROS2/WiFi로 상대 pose 공유 → rigid_body_sync_node가 동기 보정에 사용`. Front는 ArUco를 직접 관측하지 않고, Rear가 공유한 상대 위치 + 초음파로 앞바퀴 정렬·동기 주행을 수행한다. (ArUco pose는 경로계획용이 아니라 Front-Rear 동기 보정용.)

**5-3. 주행 제어 — `rigid_body_sync_node`가 주체**
중앙이 준 waypoint를 Pure Pursuit로 추종하고, 강체 기구학으로 Front/Rear 속도를 분배. 이후 엔코더와 ArUco 상대 위치를 칼만 필터로 융합해 거리·yaw 오차를 보정한다.

**5-4. STM32 통신 — `stm32_bridge_node`가 UART 변환 전담**
라즈베리파이가 `/cmd_vel`을 UART 문자열로 변환해 STM32에 전달하고, **메카넘 역기구학과 PID는 STM32의 `motor_pid_task`에서 수행**한다. 브리지 노드는 역기구학을 하지 않고 변환만 담당.

---

## 6. 소프트웨어 — 구현 상태와 목표 ROS2 노드

현재 두 Raspberry Pi의 실물 실행 환경은 ROS 2 Humble이다. STM32 수동 주행·엔코더·서보·초음파 제어와 Python 시험 도구는 확인됐다. 아래 ROS2 노드, Jetson 비전, ArUco 상대 측위, 강체 동기 주행은 **목표 구조이며 아직 전체 통합 완료 상태가 아니다.** Jetson과 개발 PC도 통합 시험 전에 Humble 환경과 `ROS_DOMAIN_ID=42`를 확인한다.

### 6-1. 목표 노드 구성

**CCTV 서버 (Jetson, Python/ROS2)**
- `yolo_bev_map_node` — 천장 2스트림 각각에서 차량 검출(YOLO11n), 공통 좌표 변환 후 겹침 영역 중복 검출 병합, 전역 점유 격자(BEV) 생성, 차량 추적. (겹침 구간은 두 관측 융합으로 측위가 오히려 안정적.)
- `fleet_manager_node` — 관제탑. 빈자리 선정, A* 경로 계획, waypoint·명령 전달.

**각 로봇 (라즈베리파이4, Python/ROS2)**
- `ultrasonic_edge_node` — 초음파로 타이어 에지 검출, 바퀴 중앙 정렬.
- `aruco_tracker_node` — (Rear만) 카메라로 상대 로봇 마커 관측, 상대 거리·각도 계산.
- `rigid_body_sync_node` — (Master: Front) 두 로봇을 한 몸으로 제어하는 강체 주행 제어기. 엔코더+ArUco 칼만 융합 후 Front/Rear의 목표 `/cmd_vel`을 생성한다. 바퀴별 목표 RPM 변환은 각 STM32의 `motor_pid_task`가 수행한다.
- `robot_state_machine_node` — 로봇 상태(대기/접근/인양/주행) 관리.
- `stm32_bridge_node` — ROS2↔STM32 UART 변환.

**각 로봇 (STM32 펌웨어, C)**
- `uart_comm_task` — 라즈베리파이와 시리얼 통신(속도 명령 수신, 엔코더 송신, Heartbeat 감시).
- `motor_pid_task` — 메카넘 역기구학 + 바퀴 모터 속도 PID → PWM 출력.
- `servo_lift_task` — 그리퍼 서보 제어(soft-start 포함).

현재 벤치용 STM32 펌웨어는 Nucleo USB 가상 COM으로 단일 문자 명령을 받아 4륜 메카넘 조합을 제어한다. 엔코더는 50ms 주기로 읽고 무부하 목표 속도는 약 12RPM이다. 키 해제·정지 명령 또는 250ms 명령 단절 시 네 모터의 PWM을 즉시 0으로 만든다. 최종 바퀴·하중 상태에서는 모터별 카운트와 PI/PID 게인을 다시 확인한다. 상세 실측값은 `TEST_LOG.md`를 따른다.

### 6-2. 목표 노드 간 통신 (ROS2 메시지 방향)

| 구간 | 내용 | 방식 | 방향 |
| --- | --- | --- | --- |
| 타겟·제원 전달 | 차량 위치 + 제원 | 토픽 | CCTV → 로봇 |
| 정렬 기준 | ArUco 마커 관측 | (시각) | Rear가 Front를 봄 |
| 인양 완료 | 들었음 신호 | 서비스 | 로봇 → CCTV |
| 경로 | waypoint 목록 | 토픽 | CCTV → 로봇 |
| 차량 위치 피드백 | 든 차량 실위치 | 토픽 | CCTV → 로봇 |
| 강체 동기 | 로봇 간 상대 위치 | 토픽 | Front ↔ Rear |
| 안전 감시 | 로봇 위치 보고 | 토픽 | 로봇 → CCTV |

> 통신 방식 구분: **토픽**=연속 데이터(위치·속도), **서비스**=1회성 요청-응답(인양 완료), **액션**=오래 걸리는 임무.

### 6-3. 목표 노드별 상세 (입출력·예외)

**[Jetson] yolo_bev_map_node** — 천장 영상으로 검출·분류·전역맵 생성
- 흐름: Image → Undistort → YOLO11n 검출 → Crop → EfficientNetV2-B0 분류 → 제원 DB 매핑 → Homography BEV 변환 → YOLOv8-seg 빈자리 인식 → OccupancyGrid 생성
- 입력: `/cctv/image_raw`, Camera Calibration, Homography Matrix, 차량 제원 DB, Parking Slot DB
- 출력: `/parking/map`, `/parking/target_pose`, `/parking/vehicle_spec`, `/parking/empty_slots`
- 예외: Calibration 없으면 시작 중단 / Homography 실패 시 Error / 차량 미검출 시 마지막 맵 유지

**[Jetson] fleet_manager_node** — 관제탑(빈자리 선정·A*·waypoint 발행)
- 흐름: Target 확인 → Lift 완료 확인 → 빈자리 탐색 → 대기공간 인접 빈자리 선정 → A* 경로 → waypoint 목록 → `/virtual_robot/waypoints` 발행 → 주행 중 CCTV 피드백으로 이탈 감시
- 입력: `/parking/target_pose`, `/parking/empty_slots`, `/parking/map`, `/robot/lifted`
- 출력: `/virtual_robot/waypoints`, `/fleet/state`
- 예외: 빈자리 없음 → 대기 / 경로 실패 → 재계획 / 이탈 → waypoint 재생성

**[RPi] ultrasonic_edge_node** — 초음파로 바퀴 중심 검출
- 흐름: 거리 측정 → Moving Average → 거리 감소 감지 → 증가 변곡점 감지 → 바퀴 중앙 검출
- 입력: `/front/ultrasonic`, `/rear/ultrasonic` / 출력: `/front/wheel_aligned`, `/rear/wheel_aligned`
- 예외: 노이즈 제거 / 1회 검출 후 reset 전까지 재검출 방지 / 비정상 거리 무시

**[RPi, Rear만] aruco_tracker_node** — Front 후방 ID0 관측해 상대 pose 계산
- 흐름: Rear Marker Camera → ArUco Detection → solvePnP → Front-Rear Relative Pose → `/sync/relative_pose` 발행
- 입력: Rear Marker Camera, Camera Calibration, ArUco Marker Size, ArUco ID0
- 출력: `/sync/relative_pose`, `/sync/marker_visible`
- 예외: Marker Loss → Encoder dead reckoning 임시 유지 / 지속 시 감속·정지 요청 / Calibration 없음 → 정확도 경고

**[RPi, Master: Front] rigid_body_sync_node** — 두 로봇을 강체 `base_virtual`로 제어(핵심)
- 흐름: waypoint 수신 → Pure Pursuit lookahead → base_virtual 목표 속도(vx,vy,w) → 강체 기구학으로 Front/Rear 분배 → 회전 시 ±w×L/2 횡속도 보정 → Front/Rear Odometry + ArUco Relative Pose 칼만 융합 → 거리 오차 PID → yaw 오차 PID → `/front/cmd_vel`, `/rear/cmd_vel` 발행
- 입력: `/virtual_robot/waypoints`, `/front/odom`, `/rear/odom`, `/sync/relative_pose`, `/sync/marker_visible`
- 출력: `/front/cmd_vel`, `/rear/cmd_vel`, `/sync/error_state`
- 예외: waypoint 종료 → 정지 / odom 0.5s 끊김 → 정지 / Marker Loss 짧음 → dead reckoning, 지속 → 감속·정지 / 거리 오차 초과 → 감속 / yaw 오차 초과 → 비틀림 위험 정지 / ESTOP → 즉시 cmd_vel=0

**[RPi] robot_state_machine_node** — 작업 상태 관리
- 상태: IDLE → APPROACH → ALIGN → LIFT → DRIVE → RELEASE → RETURN → IDLE
- 입력: `wheel_aligned`, `fleet_state`, `lift_status` / 출력: `robot_state`, `robot_lifted`, `grip_command`
- 예외: 단계별 Timeout / Front·Rear 동시 리프팅 상호 확인 / 비상정지 시 즉시 정지 전환

**[RPi] stm32_bridge_node** — ROS2↔STM32 UART 변환 (역기구학 안 함)
- 흐름: `/cmd_vel` 수신 → UART `V,vx,vy,w` / `grip_command` → `S,grip`|`S,release` / Heartbeat 주기 송신 → `HB,timestamp` / STM32 `E,fl,fr,rl,rr` 수신 → Odometry 계산 → `/odom` 발행 / `LIFT,GRIP_DONE` → `/lift_status` 발행
- 예외: UART 끊김 감지 / ERR 코드 상위 통보 / Heartbeat 미응답 시 정지 요청

**[STM32] uart_comm_task** — 문자열 파싱, 속도·서보 명령 수신, 엔코더 송신, Heartbeat 감시
- 예외: 명령/Heartbeat 250ms timeout → motor stop / Buffer overflow 방지 / 잘못된 포맷 무시 / ESTOP 수신 시 즉시 PWM 0

**[STM32] motor_pid_task** — 메카넘 역기구학 + 바퀴 PID
- 흐름: vx,vy,w → Mecanum Inverse Kinematics → Wheel Target RPM → Encoder Feedback → PID → PWM
- 예외: Anti-windup / PWM Clamp / Watchdog Stop / 속도 오차 과도 시 정지

**[STM32] servo_lift_task** — 그리퍼 서보 soft-start, 완료 응답
- 흐름: `S,grip`|`S,release` 수신 → 목표 각도 → soft-start 이동 → 도달 확인 → `LIFT,GRIP_DONE`|`LIFT,RELEASE_DONE` 송신
- 예외: 정지 상태 확인 후 리프팅 / Timeout / Emergency Stop / Servo hold·release-safe 모드

---

## 7. 목표 UART 프로토콜

아래 형식은 ROS2 통합용 목표 프로토콜이다. 현재 벤치 펌웨어의 단일 문자 명령과 혼동하지 않는다. 명령 또는 Heartbeat가 250ms 이상 끊기면 STM32가 모터를 정지하도록 통일한다.

**Raspberry Pi → STM32**
- `V,vx,vy,w` — 메카넘 속도 명령 (vx 전후, vy 좌우, w 회전 각속도)
- `S,grip` — 그리퍼 닫기(인양) / `S,release` — 그리퍼 열기(하차)
- `HB,timestamp` — Heartbeat(100ms 주기로 송신)
- `ESTOP` — 비상정지

**STM32 → Raspberry Pi**
- `E,fl,fr,rl,rr` — 바퀴별 엔코더 속도/카운트
- `LIFT,GRIP_DONE` — 리프팅 완료 / `LIFT,RELEASE_DONE` — 해제 완료
- `ACK,timestamp` — Heartbeat 응답
- `ERR,error_code` — 오류 코드

---

## 8. Fail-safe

| 상황 | 대응 |
| --- | --- |
| UART 명령/Heartbeat 250ms timeout | STM32가 자체 PWM 0, 모터 정지 |
| waypoint/경로 상실 | RPi가 cmd_vel 0 발행 |
| ArUco Marker Loss 짧게 | Encoder dead reckoning 유지 |
| ArUco Marker Loss 지속 | 감속 후 정지 |
| Encoder 값 고정 | 즉시 정지 |
| 좌우 속도차 과도 | 즉시 정지 |
| 목표 대비 실제 속도 오차 과도 | 즉시 정지 |
| Front-Rear 거리 오차 초과 | 감속 → 재보정 → 실패 시 정지 |
| Front-Rear yaw 오차 초과 | 차량 비틀림 위험, 즉시 정지 |
| 리프팅 정렬 확인 실패 | 리프팅 금지 |
| 리프팅 timeout | servo hold 또는 release-safe |
| Emergency Stop | RPi cmd_vel 0, STM32 PWM 0, Servo 안전 모드 |

---

## 9. 목표 적용 기술

- **YOLO11n** — 경량 객체 검출. 천장 영상에서 차량 실시간 검출(BBox).
- **EfficientNetV2-B0** — 이미지 분류. 검출 영역 crop → 차종 분류 → 차량 제원(휠베이스·타이어) 매핑.
- **YOLOv8-seg** — 세그멘테이션. 빈 주차칸 인식.
- **호모그래피 + 복수 카메라 정합** — 각 카메라 픽셀 좌표를 실평면 좌표로 변환하는 3×3 행렬. 설치 시 1회 캘리브레이션으로 H₁·H₂를 저장, 런타임엔 행렬 곱만 수행해 두 결과가 하나의 좌표계로 통합. 정합 기준점은 겹침 영역 바닥 특징점(주차선·코너)이며 런타임엔 바닥·차량에 마커 미부착. 영상 스티칭은 하지 않고 검출 좌표만 통합.
- **BEV + Occupancy Grid** — 천장 영상을 탑뷰로 정합해 전역 2D 점유 지도 생성. 빈자리 탐색·경로계획의 기반.
- **ROS 2 Humble** — 현재 두 Raspberry Pi에서 사용하는 로봇 미들웨어. 노드 분산, 토픽/서비스/액션 통신.
- **A* 경로 계획** — 점유 격자에서 목표까지 최단·충돌 회피 탐색. Nav2 풀스택 대신 경량 A* + waypoint 추종(천장 카메라가 전역 위치를 주므로 로봇 자기위치추정·로컬플래너 불필요).
- **Pure Pursuit** — waypoint 추종.
- **ArUco 마커** — 카메라 한 대로 상대 로봇의 거리·각도 산출. 로봇 상호에만 사용(단방향).
- **칼만 필터** — 엔코더(고빈도·단기)와 ArUco·카메라 관측(절대 보정)을 융합. 마커 가림·바퀴 슬립에도 측위 연속성 유지.
- **메카넘 휠 + 역기구학 + PID** — 4륜 메카넘으로 전방향 이동. STM32가 목표 속도를 바퀴 속도로 변환(역기구학) 후 엔코더 기반 PID 추종.
- **강체 동기 제어** — 차를 든 두 로봇을 하나의 강체로 보고 동기화. 카메라 피드백 + 상대 측위로 오차 실시간 보정.

---

## 10. 하드웨어 구성

부품 상세 내역(품명·수량·단가·구매 상태·합계)은 최신 **`BOM.md`**를 기준으로 한다. 여기서는 사용자 확인이 끝난 구성과 추가 확보 항목만 요약한다.

- **연산:** Jetson Orin Nano ×1(천장 중앙), 라즈베리파이 4 ×2(각 로봇), STM32 Nucleo F401RE ×2.
- **구동부(로봇당 4륜):** NEXUS 100mm 메카넘 휠 ×8, RB-35GM+엔코더 기어모터 ×8(DC24V, 1/100 감속), Cytron MDD10A 모터드라이버 ×4. 엔코더 풀업·실측 카운트와 최종 핀맵은 `ELECTRICAL_WIRING.md`를 따른다.
- **인양:** MG996R 디지털 서보 ×4(로봇당 2개, 그리퍼 1개당 서보 1개), 3D프린팅 V자 깔때기 그리퍼(횡방향 오차 기계적 자동 정렬). 서보 전원은 6.0V부터 설정한다.
- **센서:** OV2710은 총 4개(천장 2 + 로봇 2), HC-SR04는 총 4개(로봇당 2개)를 사용한다. 상세 구매·입고 상태는 `BOM.md`를 따른다. ※ LiDAR(LDS-08)는 보유하지만 천장 카메라가 측위를 대체하므로 미사용/예비.
- **전원(로봇 1대):** 납축 ES7-12(12V 7Ah) ×2 → 2직렬 24V 버스. 구동부(메카넘 모터)는 24V 직결, 벅 컨버터로 5.1V(RPi4)/6.0V(서보)를 분기한다. ATO 20A 퓨즈와 1NC 비상정지 스위치를 메인 전원선에 직렬로 연결하며, 비상정지 스위치는 사용자 결정에 따라 평상시 메인 전원 ON/OFF와 비상 차단을 겸한다. 부하 중 반복 조작은 피하고 실제 제품의 24V DC 차단 정격을 확인한다. ※ Jetson/천장 CCTV는 인프라측 별도 전원.
- **전원 분배 단자대:** TB-2506은 6극·12개 나사 체결점 구조이므로 로봇당 1개, 총 2개를 사용한다. 1~3극은 +24V BUS, 4~6극은 GND BUS로 나누고, 6P 쇼트바는 전기적으로 분리된 3P 두 구간으로 구성한다. 각 BUS는 입력 1개와 MDD10A ×2·XL4015 ×2·전압표시기 분기 5개를 합쳐 총 6개 체결점을 사용한다. 쇼트바 절단부는 서로 닿지 않도록 절연하고 통전 전에 3극과 4극 사이가 도통되지 않는지 확인한다.
- **프레임·체결:** DCBK2025 다이캐스팅 브라켓, 2020 L자 코너 브라켓, M3 2020 사각 너트는 도착완료로 확인됨.

---

## 11. 개발 환경

- **현재 로봇 OS:** Raspberry Pi 4 두 대 모두 Ubuntu Server 22.04.5 LTS ARM64 + ROS 2 Humble. Jetson·개발 PC의 최종 ROS 환경은 통합 전에 Humble 호환 여부를 확인한다.
- **IDE:** VSCode, STM32CubeIDE
- **도구:** ROS 2 Humble, RViz2, OpenCV, Ultralytics(YOLO11n·YOLOv8-seg), PyTorch(EfficientNetV2-B0), Git
- **언어:** Python(ROS2·비전), C(STM32 펌웨어)
- **통신:** ROS2 DDS(WiFi), UART(라즈베리파이4 ↔ STM32)
- **형상관리/협업:** Git·GitHub / Notion, 카카오톡

---

## 12. 목표 패키지 구조

아래는 구현할 패키지의 권장 구조다. 현재 파일이 모두 생성·구현됐다는 뜻이 아니다.

```
cooperative_parking_robot_ws/
└── src/
    └── cooperative_parking_robot/
        ├── cooperative_parking_robot/
        │   ├── yolo_bev_map_node.py
        │   ├── fleet_manager_node.py
        │   ├── ultrasonic_edge_node.py
        │   ├── aruco_tracker_node.py
        │   ├── rigid_body_sync_node.py
        │   ├── robot_state_machine_node.py
        │   ├── stm32_bridge_node.py
        │   ├── bev_transform.py
        │   ├── slot_iou.py
        │   ├── astar_planner.py
        │   ├── pure_pursuit.py
        │   ├── rigid_body_kinematics.py
        │   ├── kalman_filter.py
        │   ├── pid_controller.py
        │   ├── uart_protocol.py
        │   ├── encoder_odometry.py
        │   └── safety_monitor.py
        ├── launch/
        │   ├── cctv_server.launch.py
        │   ├── front_robot.launch.py
        │   ├── rear_robot.launch.py
        │   └── full_system.launch.py
        ├── config/
        │   ├── camera_calibration.yaml
        │   ├── homography.yaml
        │   ├── parking_slots.yaml
        │   ├── vehicle_specs.yaml
        │   ├── aruco_params.yaml
        │   ├── sync_params.yaml
        │   ├── serial_params.yaml
        │   └── safety_limits.yaml
        ├── models/
        │   ├── yolo11n_vehicle.pt
        │   ├── efficientnetv2_b0_vehicle_cls.pt
        │   └── yolo8n_parking_seg.pt
        ├── stm32_firmware/
        │   ├── Core/
        │   │   ├── Inc/  (uart_comm_task.h, motor_pid_task.h, servo_lift_task.h, mecanum_kinematics.h, safety_watchdog.h)
        │   │   └── Src/  (uart_comm_task.c, motor_pid_task.c, servo_lift_task.c, mecanum_kinematics.c, safety_watchdog.c)
        │   └── cooperative_parking_robot.ioc
        ├── docs/
        │   ├── system_spec.md
        │   ├── topic_list.md
        │   └── uart_protocol.md
        ├── package.xml
        ├── setup.py
        ├── setup.cfg
        └── resource/cooperative_parking_robot
```

---

## 13. 핵심 차별점 (상용 주차로봇 대비)

비교 대상은 기계식 설비가 아니라 상용 주차로봇(HL만도 파키, 현대위아). 핵심 차이는 **지능과 센서를 어디에 두느냐**.

- **차별점 ① 지능의 인프라 이전 → 저비용화:** 상용은 LiDAR·자율주행을 로봇에 탑재해 비쌈. 본 시스템은 이를 천장 카메라·중앙 연산으로 옮겨 로봇을 단순·저가로 구성. 여러 로봇이 천장 인프라 1식을 공유하므로 규모가 커질수록 로봇 1대당 비용이 낮아짐 → 비용 민감한 중·소형 주차장에 적합.
- **차별점 ② 중앙 카메라 기반 협조 주행 제어:** 두 로봇이 차를 든 채 한 몸처럼 동기화 이동. 천장 카메라가 전체를 내려다보며 오차를 보정해 정밀 협조 제어.
- **차별점 ③ 다양한 차종 대응:** 앞·뒤축을 나눠 들고 간격을 조절해 휠베이스가 다른 차종에 대응. LiDAR 없이 차종 분류 + 초음파 + V자 그리퍼로 안정 파지.

> **심사 대응 주의:** "로봇 2대가 앞·뒤축을 나눠 드는 방식" 자체는 HL만도 파키와 동일하므로 단독 차별점이 아님. 진짜 차별점은 ①(저비용 인프라 구조). 또 현재 제작은 로봇 1쌍(2대)이므로 ②는 "다수 로봇 운용"이 아니라 "1쌍의 정밀 동기 제어"로 표현해야 실물과 일치.

---
