# 실차 적용 전 필수 점검 — v1.6

> **현재 배포 기준:** Ubuntu 22.04 + ROS 2 Humble. 저장소 루트의
> [실차 Runbook](../../../docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md)과
> `hardware_preflight`를 먼저 수행한다.

## 판정

코드의 제어 흐름과 fail-safe는 정리됐지만, **실측 상수와 센서 캘리브레이션을 넣지 않은 상태로 차량을 들어 올리면 안 된다.** 먼저 빈 차체·저속·잭업 시험을 통과시킨 뒤 하중을 단계적으로 올린다.

## 확정된 소프트웨어/펌웨어 구조

- RPi `stm32_bridge_node`: ROS2↔UART 변환, 엔코더 odom 및 초음파 Range 생성
- STM32: 메카넘 역기구학, 바퀴 PID, 모터 PWM, 서보 soft-start, HC-SR04 펄스 측정
- 250 ms command timeout 또는 300 ms heartbeat timeout 시 모터 정지
- ACK 중단, odom timeout, ArUco 장기 손실, 거리/yaw 과오차 시 ESTOP
- ESTOP 시 모터 PWM 0, 서보는 갑자기 해제하지 않고 현재 각도 유지
- `GRIP_DONE`/`RELEASE_DONE`: 서보 목표각 소프트웨어 도달 신호

## STM32F401RE 타이머 배치

저장소의 `stm32/parking_robot`에는 `parking_robot.ioc`, HAL/CMSIS,
`main.h`, startup/linker와 통합 제어 소스가 포함되어 있다. 실제 권위 프로젝트는
이 디렉터리다. 자동 GNU11 검사는 코드 정합성만 뜻하므로 CubeIDE ARM
build/link와 두 보드 ST-LINK flash는 여전히 실기에서 확인해야 한다.

현재 펌웨어가 기대하는 CubeMX 배치는 다음과 같다.

| 기능 | 타이머 |
|---|---|
| 모터 PWM 4채널 | TIM1 CH1~CH4 |
| Front-left encoder | TIM2, 32-bit |
| Front-right encoder | TIM3, 16-bit |
| Rear-left encoder | TIM4, 16-bit |
| Rear-right encoder | TIM5, 32-bit |
| 초음파 microsecond timebase | TIM9, 1 MHz free-running, period 65535 |
| Left grip servo | TIM10 CH1 |
| Right grip servo | TIM11 CH1 |
| RPi UART | USART2 |

TIM3/TIM4의 16-bit rollover는 펌웨어에서 부호 있는 16-bit 차분으로 처리한다. 실제 `.ioc`, 핀 배치, 보드 배선이 위 표와 다르면 코드와 함께 수정해야 한다.

CubeMX에서 검토할 수 있는 **예시 핀 배치**는 다음과 같다. 실제 Nucleo 확장보드와 카메라/센서 배선 충돌을 확인한 뒤 확정한다.

| 기능 | 예시 핀 |
|---|---|
| TIM1 CH1~CH4 모터 PWM | PA8, PA9, PA10, PA11 |
| TIM5 CH1/CH2 encoder | PA0, PA1 |
| TIM2 CH1/CH2 encoder | PA15, PB3 |
| TIM3 CH1/CH2 encoder | PB4, PB5 |
| TIM4 CH1/CH2 encoder | PB6, PB7 |
| TIM10 CH1 servo | PB8 |
| TIM11 CH1 servo | PB9 |
| USART2 TX/RX | PA2, PA3 |
| 초음파 TRIG Left/Right | PC8, PC5 (GPIO output) |
| 초음파 ECHO Left/Right | PC6, PC7 (EXTI rising+falling) |

PA15/PB3/PB4는 기본 JTAG 관련 핀과 겹칠 수 있다. CubeMX의 debug 설정은 full JTAG가 아니라 SWD 기준으로 확인하고, ST-LINK의 PA13/PA14 SWD 연결은 유지한다. 이 표는 `.ioc`를 대체하지 않는다.

## 반드시 실측할 상수

### 1. 바퀴·엔코더·차체

- `wheel_radius`: BOM의 100 mm 휠 기준 명목값은 0.05 m지만, 하중·롤러 변형을 포함한 **유효 구름 반경**을 직선 주행 실측으로 확정
- `encoder_ppr`: 모터축 PPR × 감속비 × quadrature 배수를 추정하지 말고, 출력축 한 바퀴를 돌려 STM32 누적 카운트로 확인
- `lx`, `ly`: 로봇 중심에서 좌우/전후 바퀴 접점 축까지 거리
- `kMotorCommandSign`, `kEncoderSign`: 양의 명령과 양의 엔코더 증가 방향이 각 바퀴에서 일치하도록 잭업 시험
- STM32 PID와 PWM 상한: 무부하→모형 하중 순으로 계단 입력을 주고 튜닝

ROS launch와 STM32의 `WHEEL_RADIUS`, `ENCODER_PPR`, `LX`, `LY`는 반드시 동일해야 한다.

### 2. 차량·ArUco

- 고정 `wheelbase`: 실제 모형차 앞·뒤 차축 중심 간 거리
- Rear 카메라–Front 마커 거리 보정:

```text
aruco_distance_offset_m
  = 정상 정렬 상태의 로봇 중심 간 거리
    - solvePnP raw camera-to-marker 거리
```

현재 실측 기준은 중심 `0.785m` - ID0 raw 약 `0.215m` = `0.570m`다.
값은 `config/id0_calibration.yaml` 한 곳에서 관리하며, 로봇 외곽 길이
`0.565m`를 대신 넣거나 Python 코드를 수정하지 않는다.

실측 전에는 `use_aruco_distance:=false`로 두고 ArUco 상대 yaw만 사용한다. 실측 후:

```bash
ros2 launch cooperative_parking_robot front_robot.launch.py \
  aruco_distance_offset_m:=<실측값> \
  use_aruco_distance:=true
```

- Rear 카메라 intrinsics, 마커 실제 한 변 길이, `yaw_offset_deg`
- CCTV Front ID2/Rear ID1의 base offset, 부착 yaw 오차
- CCTV 높이, 상판 마커 높이, 광축 바닥 교점: parallax 보정용

### 3. YOLO·BEV

- `model_mode=coco`: COCO 차량 클래스(2/3/5/7)만 사용하며 빈자리는 슬롯 DB와 차량 검출로 판정
- `model_mode=vehicle_seg`(권장): 차량 mask class 인덱스와 고정 슬롯 등록값 확인
- `model_mode=parking_seg`(하위호환): vehicle/empty_slot 클래스 인덱스 확인
- `homography_rectified.npy`: `/cctv/image_rect`에서 생성. 출력이 cm이면 `homography_scale_to_m=0.01`, 이미 m이면 `1.0`으로 설정
- 대기공간, 맵 크기, 슬롯 좌표가 같은 map 좌표계를 쓰는지 확인
- 카메라 드라이버가 원본 촬영시각을 담은 `/cctv/image_raw`를 발행하고 rectifier가 stamp를 보존해 `/cctv/image_rect`로 전달하는지 확인
- Jetson/RPi 시스템 시계 동기화: CCTV pose의 `header.stamp` 신선도 판정에 필요


### 4. 초음파 센서·그리퍼 오프셋

- `left_sensor_to_gripper_x_m`, `right_sensor_to_gripper_x_m`: `gripper_x - sensor_x`
- 양수는 그리퍼 중심이 해당 센서보다 로봇 +X 방향에 있다는 뜻이다.
- 두 센서는 가능한 한 같은 로봇 X선에 설치하고, 좌우별 값을 각각 실측한다.
- STM32 펌웨어는 거리만 측정하며 이 기구 offset은 RPi `ultrasonic_edge_node`에서 적용한다.

## 전원·배선 안전

- HC-SR04 ECHO는 STM32에 연결한다. 선택 핀의 5V tolerance를 확인하고 불확실하면 3.3V 레벨시프터 또는 저항분압을 사용
- 모터/서보/연산 전원을 분리 분기하고 공통 GND 확인
- 퓨즈, 물리 비상정지, 바퀴 들뜸 시험 지그 사용
- 서보 전원은 RPi 5 V 핀에서 직접 공급하지 않음

## 단계별 시험과 합격 기준

1. **정적 통신**: 두 STM32가 10 Hz heartbeat ACK, UART 오류 0건
2. **바퀴 1개 잭업**: 명령 방향=회전 방향=엔코더 부호 일치
3. **로봇 1대 빈 차체**: 전진·후진·횡이동·제자리 회전, 300 ms 통신 차단 정지
4. **두 로봇 빈손**: 뒤쪽 waypoint에서 `vx<0`, `omega≈yaw 복구분만`; 옆 waypoint에서 `vy` 이동
5. **상대동기**: 초기 간격, ArUco raw/offset/fused distance 로그 비교; 오차 임계 시험
6. **초음파 통신·정렬**: `U,L|R,...` 수신 주기와 TIMEOUT 확인 → Rear 완료 후 Front 시작 → 양쪽 에지 재현오차 기록
7. **서보 무부하**: 좌우 각도·방향·기구 간섭·ESTOP 현재각 유지 확인
8. **가벼운 모형 하중**: 양쪽 완료 전 `/robot/lifted=false`, 보호 지그와 사람 감독
9. **전체 1사이클**: 인식→정렬→인양→후진/횡이동→최종 접근→하차→분리 복귀

이전 단계가 실패하면 다음 단계로 넘어가지 않는다.

## 아직 코드만으로 해결되지 않는 안전 항목

- `GRIP_DONE`은 실제 바퀴 파지를 확인하지 않는다. 무인 인양 판정에는 리미트 스위치, 서보 전류 또는 위치/하중 센서가 필요하다.
- park/retrieve와 양쪽 HOME 뒤 mission reset은 구현됐지만 실제 하중 연속
  사이클은 아직 검증해야 한다.
- dual CCTV merge launch는 구현됐지만 두 실카메라의 공통 map 정합을 현장에서
  반복 검증해야 한다.
- 로컬 YOLO 모델, rectified Homography `.npy`와 Rear 카메라 calibration
  `.npz`는 현장 장비별로 별도 생성·배치해야 한다.
