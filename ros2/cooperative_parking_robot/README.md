# cooperative_parking_robot 1.11.0

ROS 2 Humble용 협동 주차 로봇 패키지다. Jetson의 CCTV/BEV/Fleet/UI와
Front/Rear Raspberry Pi의 localization, 협동 FSM, STM32 bridge를 제공한다.
입차와 차량번호·비밀번호 인증 기반 retrieve가 소프트웨어에 구현돼 있다.

실차 배포 명령은 [Runbook](../../docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md),
calibration은 [pipeline](../../docs/pipeline.md), 운용 허가는
[GO/NO-GO](../../docs/REAL_WORLD_READINESS.md)를 따른다. 전체 현재 문서는
[문서 안내](../../docs/README.md)에서 찾는다.

## 시스템 흐름

```text
CCTV raw → rectification → YOLO vehicle mask + ArUco absolute pose
         → per-camera Homography → merge/map → Fleet + SQLite Registry + UI
         → Front rigid-body coordinator → Front/Rear cmd_vel
         → STM32 bridge → motor/encoder/servo/ultrasonic

Rear camera → Front rear marker ID0 → relative pose
encoder + CCTV ID10/ID11 + relative ID0 → localization
```

Production marker 역할은 Front 상판 ID10, Rear 상판 ID11, 상대 pose ID0이다.
Rear 단독 실험의 ID2/ID3은 이 패키지의 production 설정이 아니다.

## Launch index

- `cctv_server_dual.launch.py` — production dual-CCTV perception, merge, Fleet,
  Registry와 선택적 kiosk/debug overlay
- `cctv_server.launch.py` — single-CCTV 구성
- `bev_layout_calibration.launch.py` — motion node 없이 rectified Homography와
  parking layout 등록
- `rear_robot.launch.py` — Rear STM32, ultrasonic, Rear camera/ID0, localization와 FSM
- `front_robot.launch.py` — Front STM32, localization, rigid-body coordinator와 FSM
- `full_system.launch.py` — 단일 호스트 smoke/integration 구성; 실차 분산 launch
  대신 사용하지 않음

실차에서는 Front/Rear launch에 `enable_serial=true`, `require_serial=true`,
`require_hardware_ready=true`, `require_ultrasonic_for_ready=true`를 명시한다.
ArUco 중심간 거리 offset을 실측하기 전에는 `use_aruco_distance=false`를
유지한다. 전체 명령과 기동 순서는 Runbook에만 유지한다.

## Runtime asset과 장치

Jetson 기본 asset 위치는 `~/.ros/adaptive_valet_bot/`이다.

- camera별 intrinsic NPZ와 rectified Homography NPY
- 등록된 `parking_layout.yaml`
- vehicle segmentation model
- `parking_registry.db`

카메라는 현장에 고정된 `/dev/v4l/by-path/...`를 우선한다. source의 by-path
기본값은 현장 WIP일 수 있으며 보편적인 cam0/cam2 mapping이 아니다. 숫자
camera ID는 `camera_device:=''`를 명시한 경우에만 사용하며, 지정한 장치 경로가
열리지 않을 때 자동 fallback하지 않는다. STM32는 `/dev/serial/by-id/...`를
사용한다.

## 주요 동작

- Vehicle mask와 등록 slot overlap으로 occupancy를 계산한다.
- Rectified 영상의 Homography가 모든 camera output을 공통 metre map으로 변환한다.
- Front/Rear absolute pose와 encoder, ID0 relative pose를 localization에 사용한다.
- 기본 접근은 `simultaneous_entry=false`인 Front-first 순차 진입이다.
- 초음파 edge로 axle center와 lateral offset을 계산한다.
- Park와 retrieve가 접근·파지·결합 footprint 이동·하차·양쪽 HOME barrier를 공유한다.
- Fleet가 mission 승인과 slot lifecycle을 소유한다. UI는 요청만 제출한다.
- SQLite는 같은 layout의 안정 `EMPTY/OCCUPIED`만 복원하며 불일치나 transient
  mission 상태에서는 fail-closed한다.

## 안전 경계

- Production dual-CCTV는 `require_all_cameras=true`와
  `require_exact_camera_resolution=true`가 기본이다. 카메라 누락·해상도 불일치와
  live coverage 밖은 fail-closed한다.
- `GRIP_DONE`은 servo 목표각 도달이지 실제 파지/하중 확인이 아니다.
- Firmware의 현재 PPR은 `5182.0f`지만 로봇별 실측값과 ROS 값을 일치시켜야 한다.
- STM32 command watchdog은 250 ms, heartbeat watchdog은 300 ms다.
- 물리 ESTOP, HC-SR04 5 V level protection, 전원 분리, fuse와 공통 GND가
  소프트웨어보다 먼저 검증돼야 한다.
- Retrieve는 forward로 주차한 차량만 지원한다. 막힌 source 접근은 거부한다.
- 경로는 주행 중 동적으로 재계획하지 않는다.
- UI는 trusted LAN 전용이며 비밀번호 원문을 DB나 로그에 저장하면 안 된다.
- 소프트웨어 ESTOP은 기능 안전 장치가 아니다.

보호 지그, 사람 감독과 실제 파지/하중 확인 수단 없는 무인 차량 인양은 NO-GO다.
