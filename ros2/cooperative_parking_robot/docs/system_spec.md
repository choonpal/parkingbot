# 협동 자율 주차 로봇 시스템 — 최종 명세서 (v1.6, STM32 초음파 통합 — 검수 반영)

## 1. 환경

- 대기공간(Waiting Area), Target 차량(모형차), 주차장(Parking Lot), 목표 주차공간(Reach)
- Front·Rear 로봇 2대
- 천장 중앙 고정 CCTV 1대, 수직 90° 고정
- 실내 방 1개 크기, 장애물은 기둥 1개 정도의 단순·정적 환경
- 차량 높이가 충분하여 Rear 로봇이 차량 하부를 통해 Front 후방 ArUco 마커를 관측 가능
- **대기공간은 차량이 +x 방향(주차장 진행 방향)으로 정차하도록 규격화** — individual_move의 바퀴 위치 계산(차량중심 ± 휠베이스/2)이 이 전제에 기반

---

## 2. 핵심 컨셉

**"로봇을 똑똑하게"가 아니라 "주차장을 똑똑하게."**

인지·판단은 천장 카메라와 중앙 연산 장치(Jetson)가 담당하고, 로봇은 구동·정렬·파지에 집중한다. 이를 통해 로봇 구조를 단순화하고 저비용 시스템을 목표로 한다.

---

## 3. 하드웨어 구성

| 구성 | 위치 | 역할 |
|------|------|------|
| 천장 카메라 OV2710 | 천장 중앙 | 주차장 전역 인지 |
| Jetson Orin Nano | 중앙 서버 | 비전, 좌표변환, A* 경로계획, 관제 |
| Raspberry Pi 4 ×2 | 각 로봇 | 명령 수신, 센서 처리, ROS2 노드 실행 |
| STM32 F401RE ×2 | 각 로봇 | 메카넘 PID, 서보 제어 |

**로봇 1대 기준**
- 구동부: 24V DC Motor(RB-35GM + Encoder) ×4, MDD10A ×2, Mecanum Wheel ×4
- 그리퍼: Servo Motor ×2 (서보 1 = 그리퍼 1 = 바퀴 1)
- 센서: OV2710 Camera ×1(단, Front 카메라는 현 설계 미사용 — 예비 또는 제외 가능), HC-SR04 좌/우 ×2, Encoder ×4
- 전원:
  - 납축 12V 배터리 2개 직렬 = 24V 메인 전원
  - 24V는 모터 드라이버와 24V DC 모터 구동에 사용
  - 24V → 5V 벅 컨버터: Raspberry Pi 4, STM32, 센서 전원
  - 24V → 6~7.4V 벅 컨버터: 그리퍼 서보 전원

**전원 설계 주의사항**
- Raspberry Pi 4와 서보 전원은 모터 전원과 분리된 벅 컨버터를 사용한다.
- 서보는 순간 전류가 크므로 Raspberry Pi 5V 라인에서 직접 공급하지 않는다.
- 모터 전원 노이즈가 연산부로 유입되지 않도록 GND 공통 연결은 유지하되, 전원 라인은 분리한다.
- 최종 조립 후 STM32의 PWM 상한, PID 게인, 최대 속도, 가속 제한은 24V 모터 기준으로 튜닝한다.

**마커 구성**
- Front 상판 ID10, Rear 상판 ID11: 천장 CCTV 절대 pose
- Front 로봇 후방: ArUco ID0 부착
- Rear 로봇 전방 카메라: Front 후방 ArUco ID0 관측
- ID0 계산 상대 위치는 WiFi로 공유

---

## 4. 제어 구조 핵심 결정

### 4-1. 경로 계획 — 자체 A* (Nav2 미사용)
천장 카메라가 전역 위치를 직접 제공하므로 자기위치추정·로컬플래너·Nav2 Controller 불필요. Jetson에서 OccupancyGrid 기반 A* 경로계획 후 waypoint 목록을 로봇에 전달. 운반 중 yaw는 고정하며, `Front + 차량 + Rear` 결합 직사각형 footprint만큼 장애물을 팽창한다. 미확인 셀·맵 밖은 점유로 취급하고 대각선 corner cutting을 금지한다.

```
CCTV BEV Map → OccupancyGrid → 결합 footprint 팽창 → 2D A* → Waypoints → rigid_body_sync_node
```

### 4-2. ArUco — 단방향
Front 후방 ArUco ID0을 Rear 전방 카메라가 관측 → Rear가 상대 위치·yaw 계산 → WiFi 공유 → rigid_body_sync_node가 동기 보정에 사용. Front는 직접 관측하지 않고, Rear가 공유한 정보와 초음파로 정렬.

### 4-3. 주행 제어 — rigid_body_sync_node가 주체
waypoint를 Pure Pursuit로 추종, 강체 기구학으로 Front/Rear 속도 분배, Encoder + ArUco를 칼만 융합하여 거리/yaw 오차 보정.

### 4-4. STM32 통신 — stm32_bridge_node 전담
Raspberry Pi의 /cmd_vel을 UART 문자열로 변환하여 전달. 메카넘 역기구학은 STM32의 motor_pid_task에서 수행 (bridge는 변환만 담당).

### 4-5. 인지 모델 — YOLO11n-seg 단일 통합 (v1.2)
기존 YOLO11n(검출) + YOLOv8n-seg(빈자리) 2개 모델을 YOLO11n-seg 단일 모델로 통합. 커스텀 2클래스(vehicle, empty_slot) 학습으로 프레임당 추론 1회. 차종 분류(EfficientNetV2-B0)는 차량 최초 진입 시 crop 이미지 1회 분류로 제원 매핑.

---

## 5. 로봇 2대 역할 분담

| 구분 | Front | Rear |
|------|-------|------|
| 담당 위치 | 차량 앞축 | 차량 뒷축 |
| ArUco | 상판 ID10 + 후방 ID0 | 상판 ID11 |
| 카메라 용도 | **현 설계 미사용(예비)** | Front ArUco ID0 관측 |
| 정렬 순서 | ① 먼저 진입, 첫 축 통과 후 두 번째 앞축 정렬 | ② Front 완료 후 진입, 첫 번째 뒷축 정렬 |

**순차 정렬:** 두 로봇 모두 차량 뒤에서 진입한다. Front가 먼저 rear axle을
통과하고 초음파의 두 번째 wheel pair로 front axle 중심을 잡는다. 이후
`/align/front_done`을 받은 Rear가 진입해 초음파의 첫 wheel pair로 rear axle
중심을 잡고 ID0 거리/yaw를 최종 검증한다. 종방향 축 중심 제어권은 초음파다.

---

## 6. UART 프로토콜

**Raspberry Pi → STM32**
```
V,vx,vy,w        메카넘 속도 명령 (전후/좌우/회전)
S,grip           그리퍼 닫기 (인양)
S,release        그리퍼 열기 (하차)
HB,timestamp     Heartbeat
ESTOP            비상정지
```

**STM32 → Raspberry Pi**
```
E,fl,fr,rl,rr        바퀴별 엔코더 속도/카운트
LIFT,GRIP_DONE       리프팅 완료
LIFT,RELEASE_DONE    해제 완료
ACK,timestamp        Heartbeat 응답
ERR,error_code       오류 코드
```

---

**주요 토픽 책임 원칙**
- `/parking/empty_slots`: CCTV 인지부가 발행하는 빈자리 후보 목록
- `/parking/slot_pose`: `fleet_manager_node`가 최종 선택한 목표 주차칸 자세
- `/front/lifted`, `/rear/lifted`: 각 로봇의 개별 리프팅 완료 상태
- `/robot/lifted`: Front Master가 두 로봇의 리프팅 완료를 집계한 최종 신호
- `/{role}/cmd_vel` (`TwistStamped`): 상태에 따라 `individual_move_node` 또는 `rigid_body_sync_node`가 발행한다. STM32 bridge는 생성 timestamp가 250ms보다 오래됐거나 중복·역행한 명령을 거부한다.

---

## 7. 소프트웨어 구조 (WBS)

### [PART 1] CCTV 서버 — Jetson (Python/ROS2)

**7-1. yolo_bev_map_node**
- 역할: 천장 영상으로 차량 검출·차종 분류·전역 맵 생성 (YOLO11n-seg 통합)
- 처리: Image → Undistort → YOLO11n-seg 통합 검출(vehicle/empty_slot) → vehicle crop → EfficientNetV2-B0 차종 분류(최초 1회) → 제원 DB 매핑 → Homography BEV → OccupancyGrid 생성 → 협조주행 중 차량 실위치 피드백
- 입력: /cctv/image_rect, CCTV Camera Calibration, Rectified Homography Matrix, 차량 제원 DB
- 출력: /parking/map, /parking/target_pose, /parking/vehicle_spec, /parking/empty_slots, /parking/vehicle_pose_feedback
- 상태/예외: Calibration 없으면 시작 중단 / 커스텀 모델 없으면 COCO 프리트레인 폴백(빈자리는 슬롯DB 판정) / 차량 미검출 시 마지막 맵 유지

**7-2. fleet_manager_node**
- 역할: 중앙 관제탑. mission 결합 footprint 계산, 빈자리 선정, A* 경로계획, waypoint·slot_pose 발행
- 처리: Target·차량 제원 확인 → Lift 완료 확인 → Front/Rear 최신 odometry 중점으로 실제 base_virtual 시작점 계산 → 로봇 외곽·휠베이스·차량 크기·안전여유로 고정-yaw 직사각형 생성 → OccupancyGrid 팽창 → 최근접 빈자리까지 A* → waypoint와 목표 주차칸 자세 발행
- 입력: /parking/target_pose, /parking/vehicle_spec, /parking/empty_slots, /parking/map, /front/odom, /rear/odom, /robot/lifted
- 출력: /virtual_robot/waypoints, /parking/slot_pose, /fleet/state
- 책임 구분: 빈자리 후보(`/parking/empty_slots`)는 `yolo_bev_map_node`가 발행하고, 최종 선택된 주차칸 자세(`/parking/slot_pose`)는 `fleet_manager_node`가 발행한다.
- 상태/예외: 빈자리 없음 → 대기 / 경로 실패 → 재계획

### [PART 2] 로봇 두뇌 — Raspberry Pi (Python/ROS2)

**7-3. ultrasonic_edge_node**
- 역할: 초음파로 차량 바퀴 중심 검출
- 처리: STM32 Range 수신 → Moving Average → 진입/이탈 에지 중점 → 센서-그리퍼 X offset 보정 → 두 센서 중심 평균
- 입력: /{role}/ultrasonic_left, /{role}/ultrasonic_right (STM32 bridge 발행), /{role}/odom, /{role}/robot_state
- 출력: /{role}/wheel_detected, /{role}/wheel_center_x
- 상태/예외: APPROACH마다 reset / 두 센서 완료시각 1초 이내 확인 / 비정상 거리 무시
- individual_move_node가 검출된 center_x로 1cm 이내 복귀한 뒤 /{role}/wheel_aligned를 발행한다.

**7-4. aruco_tracker_node (Rear에서만 실행)**
- 역할: Rear 카메라로 Front ArUco ID0 관측 → 상대 위치·yaw 계산
- 처리: Rear Marker Camera → ArUco Detection → solvePnP → Relative Pose → Publish
- 입력: Rear Marker Camera, Camera Calibration, ArUco ID0
- 출력: /sync/relative_pose, /sync/marker_visible
- 상태/예외: Marker Loss 시 marker_visible=False 발행(감속·정지 판단과 실행은 rigid_body_sync_node 담당) / Calibration 없음 → 경고 / ArUco는 동기화 보정 전용(경로계획용 아님)

**7-5. individual_move_node (role 구분) — [신규 v1.1]**
- 역할: 차량 들기 전/후의 개별 이동 담당 (강체 주행은 rigid_body_sync가 담당)
- 처리:
  - APPROACH: 대기위치 → 담당 바퀴(Front=앞축, Rear=뒷축) 스캔 시작점으로 이동
  - ALIGN(SCAN): 차량 뒤에서 종축으로 저속 진입하며 바퀴 쌍 에지 검출
  - RETURN: `EXIT_UNDERBODY → EXIT_TO_SIDE → RETURN_HOME` 순서로 차량 아래에서 먼저 빠져나온 뒤 대기위치 복귀
- cmd_vel 게이팅: /{role}/robot_state를 구독해 APPROACH/ALIGN/RETURN 상태일 때만 발행 (DRIVE에선 침묵 → rigid_body_sync와 충돌 방지). Front SCAN은 /align/rear_done 이후 시작 (순차 보장)
- 입력: /{role}/robot_state, /{role}/odom, /parking/target_pose, /parking/vehicle_spec, /{role}/wheel_aligned
- 출력: /{role}/cmd_vel, /{role}/approach_done, /{role}/return_done

**7-6. rigid_body_sync_node (Master: Front Raspberry Pi)**
- 역할: 두 로봇을 강체(base_virtual)로 제어하는 핵심 노드
- 처리:
```
waypoint 수신 → 메카넘용 holonomic lookahead target
→ base_virtual 목표 속도(vx,vy,w)
→ 강체 기구학 Front/Rear 분배 (회전 시 ±w×L/2 횡속도)
→ CCTV 절대위치 보정 (엔코더 드리프트 오프셋)
→ Encoder + ArUco 칼만 융합 (거리/yaw)
→ 거리 PID + yaw PID 보정
→ 목표 30cm 이내 시 FINAL_APPROACH (저속 정밀 정렬)
→ /front/cmd_vel, /rear/cmd_vel 발행
```
- 입력: /virtual_robot/waypoints, /front/odom, /rear/odom, /sync/relative_pose, /sync/marker_visible, /parking/vehicle_pose_feedback, /parking/slot_pose, /parking/vehicle_spec, /emergency_stop
- 출력: /front/cmd_vel, /rear/cmd_vel, /sync/error_state
- 상태/예외:
  - waypoint 종료 → 정지
  - odom 0.5초 끊김 → 정지
  - Marker Loss 짧게 → Encoder dead reckoning
  - ID0 Loss 시 ID10+ID11 두 절대 pose가 있으면 상대 보정 유지
  - 모든 영상 상대정보 유실 0.75초 이후 감속, 1.50초 이후 정지
  - 거리 오차 초과 → 감속
  - yaw 오차 초과 → 차량 비틀림 위험, 정지
  - Emergency Stop → 즉시 cmd_vel = 0
- 부가 기능:
  - **CCTV 절대 보정:** vehicle_pose_feedback으로 엔코더 전역 드리프트를 오프셋 보정(스무딩 α=0.3)
  - **FINAL_APPROACH:** 목표 30cm 이내 + slot_pose 수신 시 저속 정밀 정렬, 위치 2cm·yaw 3° 도달 시 완료

**7-7. robot_state_machine_node (role 구분)**
- 역할: 전체 작업 상태 관리 + 순차 정렬 보장
- 상태: IDLE → APPROACH → ALIGN → LIFT → DRIVE → WAIT_RELEASE → RELEASE → RETURN → IDLE
- 순차 정렬: Front 두 번째 축 wheel_aligned → /align/front_done → Rear 접근·첫 번째 축 정렬 → 양쪽 LIFT
- 입력: 기존 상태 입력 + `/mission/{other_role}/ready`, `/mission/commit`
- 출력: 기존 상태 출력 + `/mission/{role}/ready`, `/mission/commit`(Front Master)
- 리프팅 완료 책임 구분:
  - Front/Rear state_machine은 각각 `/{role}/lifted`를 발행한다.
  - Front Master state_machine은 `/front/lifted`와 `/rear/lifted`가 모두 true일 때만 `/robot/lifted`를 true로 발행한다.
  - `fleet_manager_node`는 개별 리프팅 상태가 아니라 집계된 `/robot/lifted`만 보고 경로계획을 시작한다.
  - LIFT·DRIVE·RELEASE·RETURN은 동일 mission ID의 양쪽 READY를 Front가 확인한 뒤 발행한 COMMIT에서만 전이한다.
  - ready/commit에는 mission ID, sequence, timestamp가 포함되며 stale·중복·다른 임무 메시지는 무시한다.
- 상태/예외: 단계별 Timeout·fleet 상태 2.5초 timeout·하드웨어 ERR → latch된 FAULT+ESTOP

**7-8. stm32_bridge_node (role 구분)**
- 역할: ROS2 ↔ STM32 UART 변환 (역기구학 수행 안 함)
- 처리: timestamp 검증된 `TwistStamped` cmd_vel → "V,vx,vy,w" 송신 / grip_command → "S,grip/release" / Heartbeat 송신 / "E,..." → odom / "U,L|R,..." → Range / "LIFT,..." → lift_status / "ACK","ERR" 처리
- WiFi 지연 대비: cmd_vel을 50Hz 송신 루프에서 신선도 확인, 200ms 초과 시 선형 감쇠, 500ms 초과 시 정지 (급정지 대신 부드러운 감속, STM32 워치독 300ms는 최후 안전망)
- 입력: /{role}/cmd_vel, /{role}/grip_command, /emergency_stop
- 출력: /{role}/wheel_odom, /{role}/ultrasonic_left, /{role}/ultrasonic_right, /{role}/ultrasonic_status, /{role}/lift_status, /{role}/hardware_status, /{role}/hardware_ready
- 상태/예외: UART 끊김 감지 / ERR 수신 시 상위 통보 / 역기구학은 STM32가 수행

### [PART 3] STM32 Firmware (C)

**7-9. uart_comm_task** — UART 파싱, 속도/서보 명령 수신, 엔코더·초음파 송신, Heartbeat 300ms timeout → motor stop, ESTOP 즉시 PWM 0

**7-10. motor_pid_task** — 24V 메카넘 구동부의 역기구학 및 바퀴별 PID 속도 제어를 담당한다.

처리:
```
V,vx,vy,w 수신
→ Mecanum Inverse Kinematics
→ 바퀴별 목표 RPM 계산
→ Encoder Feedback 수신
→ 바퀴별 PID 제어
→ 24V Motor Driver PWM 출력
```

예외:
- Anti-windup 적용
- PWM 상한값 Clamp
- 목표 속도 대비 실제 속도 오차 과도 시 정지
- Heartbeat timeout 시 PWM 0
- Emergency Stop 수신 시 PWM 0
- 최종 PWM 제한값과 PID 게인은 24V 모터 실측 후 튜닝

**7-11. servo_lift_task** — 그리퍼 Soft-start 제어, GRIP_DONE/RELEASE_DONE 응답. 정지 상태 확인 후 리프팅, Timeout, servo hold/release-safe

**7-12. ultrasonic_task** — TIM9 1MHz 시간축과 GPIO EXTI로 좌/우 HC-SR04를 35ms 간격으로 교대 측정. 유효 거리(mm) 또는 TIMEOUT을 `U,L|R,...` UART 프레임으로 전송하며, 바퀴 에지 판단은 RPi가 수행한다.

---

## 8. 동작 흐름 (6단계)

1. **인지·제원 파악** — 차량 대기공간 진입 → 천장 카메라 YOLO11n-seg 검출 → vehicle crop → EfficientNetV2-B0 차종 분류 → 제원(휠베이스) 확보 → 로봇 전달
2. **순차 정렬** — ① Front가 뒤에서 먼저 진입해 첫 축을 통과하고 초음파 두 번째 축에 정렬 → ② Rear가 뒤에서 진입해 초음파 첫 축에 정렬 → ID0 거리/yaw 최종 확인
3. **인양·보고** — Front·Rear 동시 그리퍼 조여 인양 → STM32가 LIFT,GRIP_DONE 응답 → 각 로봇이 /front/lifted, /rear/lifted 발행 → Front Master가 둘 다 확인 후 /robot/lifted를 fleet_manager로 발행
4. **경로 계획** — fleet_manager가 실제 base_virtual 시작점과 mission 결합 직사각형을 계산 → 미확인·경계·corner cutting을 차단한 A* → waypoint + slot_pose 발행
5. **협조 주행** — rigid_body_sync가 waypoint를 Pure Pursuit 추종 + 강체 분배 + CCTV 절대보정 + Encoder/ArUco 칼만 융합 → 목표 근접 시 FINAL_APPROACH 정밀 정렬. 천장 카메라가 차량 실위치 지속 추적·피드백
6. **하차·복귀** — 목표 도착 → 그리퍼 풀어 하차 → individual_move_node가 대기공간 복귀

---

## 9. Fail-safe

| 상황 | 대응 |
|------|------|
| UART Heartbeat 300ms timeout | STM32 자체 PWM 0, 모터 정지 |
| 24V 모터 과속 또는 PWM 상한 초과 | STM32에서 PWM Clamp 적용, 최대 속도 제한 |
| 모터 전원 노이즈로 통신 불안정 | Heartbeat timeout 발생 시 STM32 자체 정지 |
| WiFi cmd 지연 200~500ms | stm32_bridge 선형 감쇠 (급정지 방지) |
| 좌/우 초음파 UART 프레임 0.5초 이상 미수신 | hardware_ready=false 및 상태 경고; ALIGN 완료 불가 |
| waypoint/경로 상실 | cmd_vel 0 |
| ArUco Marker Loss 짧게 | Encoder dead reckoning |
| ArUco Marker Loss 1초 지속 | ArUco yaw 보정 중단, Encoder 기반 추정(yaw 임계 완화) + 50% 감속 |
| ArUco Marker Loss 2초 이상 지속 또는 yaw 추정 불확실성 증가 | 즉시 정지 |
| Encoder 값 고정/속도차 과도 | **STM32 motor_pid_task가 감지**(목표≠0인데 엔코더 델타 0이 연속 N주기) → ERR 송신 + 즉시 정지 |
| Front-Rear 거리 오차 초과 | 감속 → 재보정 → 실패 시 정지 |
| Front-Rear yaw 오차 초과 | 차량 비틀림 위험, 즉시 정지 |
| 리프팅 정렬 확인 실패 | 리프팅 금지 |
| 리프팅 timeout | servo hold 또는 release-safe |
| Emergency Stop | RPi cmd_vel 0, STM32 PWM 0, Servo 안전모드 |

---

## 10. 적용 기술

- **YOLO11n-seg**: 차량·빈자리 통합 검출 (커스텀 2클래스, 단일 모델)
- **EfficientNetV2-B0**: 차종 분류 → 제원 매핑
- **Homography**: 천장 카메라 좌표 → BEV 좌표 변환 (설치 시 1회 캘리브)
- **BEV + OccupancyGrid**: 전역 맵 생성
- **A***: 전역 경로계획 (Nav2 미사용, 경량)
- **Pure Pursuit**: waypoint 추종
- **ArUco**: 단방향 상대측위 (Rear→Front)
- **Kalman Filter**: Encoder + ArUco 융합, CCTV 절대 보정
- **Mecanum Kinematics + PID**: 전방향 구동, 바퀴별 속도 제어
- **Rigid Body Sync Control**: Front/Rear 강체 동기 제어

---

## 11. 개발 환경

- OS: Ubuntu 22.04 + ROS 2 Humble (Jetson Orin Nano는 JetPack 6.x 기준)
- IDE: VSCode, STM32CubeIDE
- 언어: Python(ROS2·비전), C(STM32)
- 통신: ROS2 DDS over WiFi, UART(RPi↔STM32)
- 형상관리: Git, GitHub / 협업: Notion, 카카오톡

---

## 12. 파일 구조

```
cooperative_parking_robot_ws/
└── src/cooperative_parking_robot/
    ├── cooperative_parking_robot/
    │   ├── [노드]
    │   │   ├── yolo_bev_map_node.py         CCTV: YOLO11n-seg 통합 + 차종분류
    │   │   ├── fleet_manager_node.py        관제: A* 경로계획 + waypoint
    │   │   ├── ultrasonic_edge_node.py      초음파 바퀴 검출
    │   │   ├── aruco_tracker_node.py        Rear→Front 마커 관측 (단방향)
    │   │   ├── individual_move_node.py      개별 이동 (APPROACH/SCAN/RETURN)
    │   │   ├── rigid_body_sync_node.py      강체 동기 제어 (핵심)
    │   │   ├── robot_state_machine_node.py  상태머신 (순차 정렬)
    │   │   └── stm32_bridge_node.py         UART 변환 (역기구학 X, WiFi 감쇠)
    │   └── [모듈]
    │       ├── astar_planner.py             A* 경로계획
    │       ├── pure_pursuit.py              waypoint 추종
    │       ├── rigid_body_kinematics.py     강체 분배
    │       ├── kalman_filter.py             엔코더+ArUco 융합
    │       ├── pid_controller.py            거리/yaw 보정
    │       ├── uart_protocol.py             UART 인코딩/파싱
    │       └── encoder_odometry.py          엔코더→odom
    ├── launch/full_system.launch.py
    ├── config/ (sync_params, safety_limits)
    ├── models/ (parking_seg 또는 COCO YOLO, efficientnetv2_b0_vehicle)
    └── stm32_firmware/ (Core/Src, Core/Inc, .ioc)
```

---

## 13. 향후 보강 예정

- TF Tree(base_virtual, front/rear_base_link, 카메라/마커 프레임) 정의·구현
- 패키지 분리 (perception/fleet/sensing/sync_control/state_machine/stm32_bridge/interfaces)
- 커스텀 메시지(SyncError, LiftStatus, VehicleSpec) 및 Lift.action
- STM32 펌웨어 5파일 분리 (uart/motor/servo/kinematics/watchdog)
- UART CRC·재전송·오류 복구 보강
- 전류 센서(ACS712) 추가 시 토크 제한
- **배터리 저전압 보호: 전압분배+STM32 ADC 측정 회로 추가 시 저전압 경고·감속 구현** (현 하드웨어엔 전압 측정 수단 없음 — v1.4의 해당 Fail-safe 조항을 이곳으로 이동)
- 차량 1대 처리 시간(throughput) 목표치 산정
- 저비용 손익분기 계산 (자작 N대 + 인프라 vs 상용 로봇)


---

## 14. 실행 방법 (기기별 배포)

### 14-0. 사전 준비 — 세 기기 공통

세 기기(Jetson, Front RPi, Rear RPi)는 **같은 WiFi 네트워크**에 있어야 하며, 다음 설정을 각각 한 번씩 해준다.

**1) ROS_DOMAIN_ID 통일** — 같은 번호여야 서로 토픽이 보인다.

```bash
# 세 기기 모두 ~/.bashrc에 추가
echo "export ROS_DOMAIN_ID=42" >> ~/.bashrc
source ~/.bashrc
```

**2) 시간 동기화** — 칼만 필터 타임스탬프가 꼬이지 않도록.

```bash
# 세 기기 모두
sudo apt install chrony -y
# Jetson을 기준 시계로 삼으려면 RPi의 /etc/chrony/chrony.conf에
# server <Jetson_IP> iburst 추가 후 재시작
sudo systemctl restart chrony
```

**3) 패키지 빌드** — 세 기기 모두 동일하게.

```bash
mkdir -p ~/cpr_ws/src && cd ~/cpr_ws/src
# (Git clone 또는 tar 배포로 cooperative_parking_robot 배치)
cd ~/cpr_ws
colcon build --packages-select cooperative_parking_robot
echo "source ~/cpr_ws/install/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 14-1. Jetson Orin Nano (CCTV 서버)

천장 카메라가 USB로 연결되어 있어야 하며, 다음 파일이 필요하다.

| 준비물 | 생성 방법 |
|--------|----------|
| cctv_camera_calibration.npz | 천장 카메라 intrinsic/distortion; 패키지 포함 |
| homography_rectified.npy | /cctv/image_rect에서 바닥 기준점 클릭 (설치 시 1회) |
| parking_seg.engine | 선택: vehicle/empty_slot 커스텀 모델. 없을 때는 `model_mode=coco`와 일반 YOLO 모델을 명시적으로 사용 |
| efficientnetv2_b0_vehicle.pt | 차종 분류기 (없으면 기본 제원 사용) |

```bash
# Jetson에서
ros2 launch cooperative_parking_robot cctv_server.launch.py
```

실행되는 노드: `yolo_bev_map_node`, `fleet_manager_node`

### 14-2. Front 로봇 Raspberry Pi (Master)

STM32가 USB(UART)로 연결되어 있어야 한다. 포트 확인:

```bash
ls /dev/ttyUSB* /dev/ttyACM*   # 보통 /dev/ttyUSB0 또는 /dev/ttyACM0
# launch 파일의 serial_port 파라미터를 실제 포트로 맞춘다
```

```bash
# Front RPi에서
ros2 launch cooperative_parking_robot front_robot.launch.py
```

실행되는 노드: `rigid_body_sync_node`(Master — 두 로봇 cmd_vel 발행), `individual_move`(front), `state_machine`(front), `stm32_bridge`(front), `ultrasonic_edge`(front)

### 14-3. Rear 로봇 Raspberry Pi

전방 카메라(Front 마커 관측용)와 STM32가 연결되어 있어야 하며, ArUco용 Rear 카메라 캘리브레이션 파일(rear_camera_calibration.npz)이 필요하다.

```bash
# Rear RPi에서
ros2 launch cooperative_parking_robot rear_robot.launch.py
```

실행되는 노드: `aruco_tracker_node`(Front ID0 관측), `individual_move`(rear), `state_machine`(rear), `stm32_bridge`(rear), `ultrasonic_edge`(rear)

### 14-4. 실행 순서와 확인

권장 실행 순서: **Jetson → Rear → Front** (관제가 먼저 떠 있어야 로봇 상태머신이 fleet_state를 받는다)

```bash
# 아무 기기에서나 연결 확인
ros2 node list          # 12개 노드가 보여야 함
ros2 topic list         # /parking/map, /front/odom 등
ros2 topic echo /fleet/state        # 관제 상태
ros2 topic echo /front/robot_state  # 로봇 상태 (IDLE부터 진행)
ros2 topic echo /sync/error_state   # 동기 오차 모니터링
```

**노드가 서로 안 보일 때 점검 순서:**
1. `echo $ROS_DOMAIN_ID` — 세 기기 모두 같은 번호인가
2. 같은 WiFi/서브넷인가 (`ip addr`로 IP 대역 확인)
3. 방화벽: `sudo ufw disable` (또는 DDS 포트 허용)
4. 멀티캐스트 차단 공유기라면 `ROS_LOCALHOST_ONLY=0` 확인

### 14-5. 비상정지

어느 기기에서든:

```bash
ros2 topic pub --once /emergency_stop std_msgs/Bool "data: true"
```

RPi가 cmd_vel 0 발행 + stm32_bridge가 ESTOP 송신 + STM32가 PWM 0.

### 14-6. 기기별 노드 배치 요약

| 기기 | launch 파일 | 노드 |
|------|-------------|------|
| Jetson (CCTV) | cctv_server.launch.py | yolo_bev_map, fleet_manager |
| Front RPi (Master) | front_robot.launch.py | rigid_body_sync, individual_move, state_machine, stm32_bridge, ultrasonic_edge |
| Rear RPi | rear_robot.launch.py | aruco_tracker, individual_move, state_machine, stm32_bridge, ultrasonic_edge |

※ `full_system.launch.py`는 한 PC에서 전체를 띄우는 개발·시뮬레이션용. 실제 배포는 위 3개 launch를 기기별로 사용한다.

---

## 부록. 알고리즘 단위 시뮬레이션 검증 항목

| 항목 | 검증 결과 |
|------|----------|
| A* 경로계획 | 기둥 장애물 회피, 30 waypoint 생성 |
| Pure Pursuit | 목표 도착 오차 1cm |
| 강체 분배 | front/rear 횡속도 부호 반대 (회전 보정) |
| CCTV 절대 보정 | 5cm 드리프트 → 1.4mm 이내 수렴 |
| FINAL_APPROACH | 5° 틀어짐 → 2cm/0° 수렴 (556스텝) |
| WiFi 감쇠 | 250ms→84%, 450ms→16%, 600ms→0% |
| 개별 이동 | Front/Rear 담당 바퀴 위치 분리 계산 |
| **폐루프 협조주행 (외란 포함)** | 엔코더 3% 미끄러짐 + ArUco 9% 드롭 + WiFi 80ms 지연: **휠베이스 오차 평균 1.6mm/최대 8.0mm 통과**(한계 30mm) |
| **폐루프 — 나쁜 WiFi** | 150ms 지연 + 스파이크 15%: 최대 17.8mm, yaw 3.8° 통과 |
| **칼만 델타 전파 수정** | predict 덮어쓰기 버그 수정 — ArUco 보정 유지 확인 |
| **보정 분담** | 거리/yaw PID를 Front(로컬 즉시)+Rear 절반씩 분담 |
| **이음새 정적 감사** | pub↔sub 30개 전수 대조, 불일치 0건 (초음파 토픽 버그 수정 완료) |

*알고리즘 단위 + 폐루프 시뮬레이션 검증 완료. 하드웨어 검증은 모터 1개 → 로봇 1대 → 2대 빈손 동기주행 → 차량 인양 순으로 진행한다.*
