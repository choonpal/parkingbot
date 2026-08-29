# 주차로봇 시스템 인수인계

최종 갱신: 2026-08-29
범위: 모형차를 Front/Rear 로봇이 함께 운반하는 production 협동 미션

부품과 입고 상태는 [BOM](BOM.md), 전압·배선·핀은
[ELECTRICAL_WIRING](ELECTRICAL_WIRING.md), 실기 이력은
[TEST_LOG](TEST_LOG.md)을 우선한다. 설치와 운행 절차는
[실차 Runbook](../REAL_ROBOT_DEPLOYMENT_RUNBOOK.md)이 기준이다.

## 1. 목표와 현재 판정

천장 CCTV와 Jetson이 주차장 전역을 인지·계획하고, Front와 Rear 메카넘
로봇이 차량 앞·뒤축을 각각 인양해 지정 슬롯과 대기영역 사이로 운반한다.

현재 소프트웨어에는 park/retrieve, SQLite Registry, UI 승인, 접근·정렬,
운반·복귀 흐름이 구현돼 있다. 그러나 실제 파지·하중 확인 센서와 전체 하중
시험이 완료되지 않았으므로 사람 없는 무인 차량 인양은 **NO-GO**다.

현재 통합 감사 범위는 `stop_after_align=true`인 차량 하부 진입·차축 정렬·정지까지다.
Front에는 최신 통합본을 배포해 초음파-그리퍼 X offset `0.0m`를 확인했지만 Rear
두 값과 Rear/Jetson 동일 SHA 배포가 미확정이므로 자동 진입은 현재 **NO-GO**다.
최신 판정은
[현재 통합 상태](../CURRENT_INTEGRATION_STATUS.md)를 따른다.

## 2. 시스템 구성

| 구성 | 수량 | 역할 |
|---|---:|---|
| Jetson Orin Nano | 1 | 듀얼 CCTV, 차량·슬롯 인식, Fleet, 경로계획, UI, Registry |
| 천장 OV2710 | 운용 2 | 겹치는 시야를 공통 `map` 좌표로 변환 |
| Raspberry Pi 4 | 2 | 로봇별 ROS 2 상태기계, pose fusion, STM32 bridge |
| Nucleo F401RE | 2 | 메카넘 휠 PID, encoder, servo, ultrasonic, ESTOP latch |
| Front / `robot-2` | 1 | 차량 앞축, 강체 운반 master |
| Rear / `robot-1` | 1 | 차량 뒤축, Front 후면 마커 관측 |

모든 Linux 장비는 Ubuntu 22.04, ROS 2 Humble, `ROS_DOMAIN_ID=42`와 동기화된
시계를 사용한다.

```text
cam0/cam2
  → 렌즈 보정
  → YOLO vehicle mask + Homography
  → 차량·슬롯·OccupancyGrid
  → Fleet + UI 승인 + SQLite Registry
  → 접근 waypoint / 결합 footprint A*
  → Front master
  → /front/cmd_vel, /rear/cmd_vel
  → 각 RPi STM32 bridge
  → UART
  → 모터·encoder·servo·ultrasonic
```

## 3. 마커와 위치 추정

Production 협동 미션의 마커는 다음과 같다.

| 마커 | 관측자 | 목적 |
|---|---|---|
| Front 상판 ID 2 | 천장 CCTV | Front 절대 pose |
| Rear 상판 ID 1 | 천장 CCTV | Rear 절대 pose |
| Front 후면 ID 0 | Rear 전방 카메라 | 로봇 간 상대 yaw·거리 |

엔코더 예측을 CCTV 절대 pose와 ID 0 상대 pose로 보정한다. ID 0의 거리
offset을 실측하기 전에는 `use_aruco_distance=false`로 두고 상대 yaw만 사용한다.

Rear 단독 실험 branch에서 사용하는 Rear ID 2와 차량 ID 3은 이 production
구성에 적용하지 않는다.

## 4. 협동 미션

### 입차

```text
차량 인식과 UI 승인
→ Front/Rear staging
→ PRE_ALIGN
→ 초음파 SCAN_IN과 차축 중심 정렬
→ 양쪽 grip
→ 결합 footprint 경로계획·운반
→ 슬롯 축 정렬·하차
→ 두 로봇 HOME
→ mission complete와 OCCUPIED 기록
```

### 출차

UI는 차량번호와 주차 비밀번호만 제출한다. Fleet가 SQLite Registry에서
인증된 source slot, 저장 pose와 차량 제원을 찾고, 접근·인양·운반 흐름을
반대로 재사용해 고정 waiting pose에 내려놓는다. 현재 출차는 이 시스템이
forward로 주차한 차량만 지원한다.

기본 실증 layout에서는 두 로봇 동시 접근의 clearance가 부족하므로
`simultaneous_entry=false`인 Front-first 순차 접근을 사용한다.

## 5. 로봇별 책임

### Jetson

- cam0/cam2별 intrinsic, rectified Homography와 공통 layout 관리
- 차량 mask, 슬롯 점유, 전역 map과 로봇 상판 마커 pose 발행
- Fleet 단일 writer와 SQLite Parking Registry
- 차량번호·비밀번호 기반 park/retrieve 요청 검증
- 7인치 `/kiosk` UI 제공

### Raspberry Pi

- 로봇별 state machine과 `hardware_ready` gate
- encoder odometry와 CCTV/ArUco pose fusion
- ultrasonic axle edge·center 계산
- 안정적인 `/dev/serial/by-id/` 경로로 STM32 연결
- 수동 모드와 자동 명령의 상호 배제

### STM32

- 4륜 메카넘 역기구학과 휠 속도 폐루프
- encoder, 좌우 HC-SR04, 좌우 grip servo
- UART heartbeat와 명령 watchdog
- ESTOP latch 및 통신 단절 시 PWM 0

현재 firmware 기준 encoder는 출력축 1회전 약 5,182 count, 주행 명령
watchdog은 250 ms, heartbeat timeout은 300 ms다. ROS와 firmware에는 실제
로봇에서 다시 측정한 같은 값을 넣는다.

## 6. 안전 불변조건

- 물리 ESTOP과 모터 전원 차단이 소프트웨어 정지보다 우선한다.
- `hardware_ready=false`이면 구동하지 않는다.
- HC-SR04 ECHO 5 V는 level shifter 또는 검증된 분압을 거친다.
- 현재 로봇은 단일 메인 전원 구조라 RPi/카메라와 motor rail을 독립적으로
  ON/OFF할 수 없다. 공통 GND와 fuse를 확인하고, 정적 통전 시험은 모든 바퀴를
  띄운 뒤 격리 ROS domain과 perception-only 노드로만 수행한다.
- `GRIP_DONE`은 servo 목표각 도달일 뿐 실제 파지나 하중을 증명하지 않는다.
- 카메라 intrinsic, Homography와 layout을 현장에서 검증하기 전에는 주행하지 않는다.
- STM32 ESTOP은 latch되며 원인 제거 후 전원을 재인가해야 한다.
- 단계시험 하나라도 실패하면 다음 단계로 넘어가지 않는다.

## 7. 현재 하드웨어 상태

- 메인 스위치/비상정지가 전체 전원을 함께 제어하며 별도 motor-power enable은
  없다. 따라서 `motor OFF + camera/UART ON` 절차는 현재 하드웨어에서 불가능하다.
- Front(`robot-2`): ROS bridge, 잭업 폐루프 3축 주행과 무하중 저속 바닥
  주행까지 확인했다.
- Rear(`robot-1`): 교체 STM32 배포 뒤 2026-08-25 정상 단독 주행을 사용자가
  확인했다. 기존 강체 쌍 시험에서 `W/S`와 정지를 확인했으나 최신 통합본 재배포,
  잭업 재확인과 `A/D/Q/E` 수정 뒤 재시험은 남아 있다.
- 두 로봇의 최신본 빈손 동기주행, 초음파 차축 반복정밀도, 보호 지그 저하중,
  park→retrieve 전체 실차 cycle은 순서대로 검증해야 한다.

최신 상태는 [하드웨어 README](README.md)와 [TEST_LOG](TEST_LOG.md)에
추가하고, 절차나 안전 기준을 날짜별 로그에 중복 작성하지 않는다.

## 8. 인수인계 확인 순서

1. [문서 안내](../README.md)에서 현재 문서와 과거 기록을 구분한다.
2. [실차 준비도](../REAL_WORLD_READINESS.md)에서 현재 NO-GO를 확인한다.
3. [Runbook](../REAL_ROBOT_DEPLOYMENT_RUNBOOK.md)에 따라 배선·flash·분산
   기동을 준비한다.
4. [Pipeline](../pipeline.md)에 따라 intrinsic, Homography, layout과
   preflight를 완료한다.
5. 공통 전원 OFF에서 잭업·작업구역 격리 → perception-only 통전 → 로봇 1대 →
   두 로봇 빈손 → 초음파 → 무부하 grip → 보호 지그 저하중 순서를 지킨다.

로그인 정보, 내부 IP와 장치 일련번호는 공개 저장소 문서에 기록하지 않는다.
