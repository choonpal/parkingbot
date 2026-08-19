# STM32 초음파 측정 - Raspberry Pi 정렬 판단 통합 설계

## 1. 최종 책임 분리

```text
HC-SR04 Left/Right
  -> STM32 TIM9 1 MHz + ECHO EXTI
  -> UART U,L|R,<distance_mm|TIMEOUT>
  -> stm32_bridge_node
  -> /{role}/ultrasonic_left|right (sensor_msgs/Range)
  -> ultrasonic_edge_node
  -> /{role}/wheel_center_x, /{role}/wheel_detected
  -> individual_move_node
  -> /{role}/wheel_aligned
```

| 구성 | 책임 |
|---|---|
| STM32 | 10 us TRIG 생성, ECHO 펄스 폭 측정, 좌우 교대, timeout, mm 프레임 전송 |
| `stm32_bridge_node` | UART 파싱, `sensor_msgs/Range` 변환, 프레임 신선도와 hardware-ready 관리 |
| `ultrasonic_edge_node` | ALIGN 상태 게이트, 이동평균, 진입/이탈 에지, 센서-그리퍼 offset, 축 중심 판단 |
| `individual_move_node` | 검출된 중심으로 저속 복귀 후 `wheel_aligned=true` 발행 |

RPi는 더 이상 HC-SR04 GPIO를 직접 토글하거나 ECHO busy-wait를 수행하지 않는다.

## 2. UART 프로토콜

STM32에서 RPi로 전송한다.

```text
U,L,83          # left 83 mm
U,R,86          # right 86 mm
U,L,TIMEOUT     # left Echo 미수신/유효범위 밖
U,R,TIMEOUT
```

- 거리 단위: 정수 mm
- 유효 펌웨어 범위: 20~4000 mm
- 좌우 트리거 간격: 35 ms
- 센서 하나당 nominal update: 약 14.3 Hz
- Echo timeout: 25 ms

## 3. ROS 토픽 구조

| 토픽 | 타입 | 발행 | 구독 | 의미 |
|---|---|---|---|---|
| `/{role}/ultrasonic_left` | `sensor_msgs/Range` | `stm32_bridge_node` | `ultrasonic_edge_node` | 왼쪽 초음파 거리 |
| `/{role}/ultrasonic_right` | `sensor_msgs/Range` | `stm32_bridge_node` | `ultrasonic_edge_node` | 오른쪽 초음파 거리 |
| `/{role}/ultrasonic_status` | `std_msgs/String` | `stm32_bridge_node` | 진단 도구 | `left,OK,0.083`, `right,TIMEOUT` |
| `/{role}/wheel_center_x` | `std_msgs/Float64` | `ultrasonic_edge_node` | `individual_move_node` | 그리퍼 기준 최종 정렬 목표 X |
| `/{role}/wheel_detected` | `std_msgs/Bool` | `ultrasonic_edge_node` | `individual_move_node` | 좌우 에지 확정 |
| `/{role}/hardware_ready` | `std_msgs/Bool` | `stm32_bridge_node` | 상태머신 | UART ACK와 좌우 초음파 프레임 신선도 |

`TIMEOUT`은 `Range.range=+inf`로 발행한다. 에지 검출기는 비정상 거리를 표본에 넣지 않는다.

## 4. 센서-그리퍼 앞뒤 offset

각 센서가 그리퍼 중앙과 같은 로봇 X선에 있지 않다면 다음을 실측한다.

```text
sensor_to_gripper_x_m = gripper_center_x - sensor_x
```

- 양수: 그리퍼 중심이 센서보다 robot +X 앞쪽
- 음수: 그리퍼 중심이 센서보다 뒤쪽

에지 검출 시 적용 식:

```text
gripper_target_base_x = raw_sensor_center_base_x - sensor_to_gripper_x_m
```

좌우 센서에 각각 값을 넣는다.

```bash
ros2 launch cooperative_parking_robot front_robot.launch.py \
  left_sensor_to_gripper_x_m:=0.025 \
  right_sensor_to_gripper_x_m:=0.023
```

두 센서의 앞뒤 위치 차이는 가능한 한 작게 하고, 센서 면·높이·타이어까지의 기준 거리를 대칭으로 맞춘다.

## 5. STM32CubeMX 필수 설정

현재 권위 프로젝트는 저장소의 `stm32/parking_robot/parking_robot.ioc`다.
CubeIDE에서 다음 설정이 유지되는지 확인한다.

| 기능 | 설정 |
|---|---|
| TIM9 | 1 MHz counter tick, up-counting, period 65535, internal clock |
| Left/Right TRIG | GPIO output push-pull, 초기 LOW |
| Left/Right ECHO | GPIO EXTI rising + falling, 적절한 pull 설정 |
| USART2 | 115200 8N1, RPi UART |
| main loop | `Robot_Init()` 한 번, `Robot_MainLoop()` 지속 호출 |

현재 `.ioc`의 GPIO label과 핀은 다음과 같다.

```text
Left TRIG  PC8
Right TRIG PC5
Left ECHO  PC6
Right ECHO PC7
```

HC-SR04 ECHO는 5 V다. STM32 핀의 5 V tolerance를 확인하고 불확실하면 레벨시프터 또는 저항분압을 사용한다.

## 6. Launch 핵심 파라미터

| 파라미터 | 기본값 | 설명 |
|---|---:|---|
| `require_ultrasonic_for_ready` | `true` | 좌우 프레임이 신선해야 hardware ready |
| `ultrasonic_frame_timeout_s` | `0.50` | bridge/edge의 프레임 stale 한계 |
| `ultrasonic_threshold_m` | `0.10` | 바퀴 진입 판정 거리 |
| `ultrasonic_exit_hysteresis_m` | `0.02` | 이탈 판정 hysteresis |
| `left_sensor_to_gripper_x_m` | `0.0` | 왼쪽 센서-그리퍼 X offset |
| `right_sensor_to_gripper_x_m` | `0.0` | 오른쪽 센서-그리퍼 X offset |

## 7. 단계별 시험

### 7-1. STM32 단독

1. 오실로스코프로 TRIG가 약 10 us인지 확인
2. 좌/우가 35 ms 간격으로 교대하는지 확인
3. 고정 평판 50, 100, 200 mm에서 UART mm 오차 기록
4. 물체 제거 시 `TIMEOUT` 확인
5. 모터 PWM 작동 중 거리 노이즈 확인

### 7-2. UART와 ROS

```bash
ros2 topic hz /front/ultrasonic_left
ros2 topic hz /front/ultrasonic_right
ros2 topic echo /front/ultrasonic_status
ros2 topic echo /front/hardware_ready
```

합격 기준:

- 각 센서 12~16 Hz 범위
- 잘린 UART 프레임/파싱 오류 0건
- STM32 연결 후 0.5 s 안에 양쪽 Range 도착
- UART 분리 시 hardware ready가 false

### 7-3. 정렬

1. 로봇 고정, 바퀴 모형을 이동해 진입/이탈 거리 확인
2. 로봇을 0.03 m/s로 스캔해 좌우 중심 20회 기록
3. 센서 offset 0과 실측 offset 적용 결과 비교
4. 최종 그리퍼-바퀴 중심 오차를 측정

권장 합격 기준은 기구 허용범위보다 충분히 작은 오차로 정한다. 예를 들어 V자 그리퍼 허용오차가 ±15 mm라면 95% 정렬오차를 ±10 mm 이내로 둔다.

## 8. 아직 해야 할 것

- 실제 확장보드/driver와 현재 `.ioc` 핀 충돌 검토
- TIM9 1 MHz 설정 및 실제 counter 검증
- ECHO 입력 전압 보호회로
- 좌/우 센서 장착 위치와 offset 실측
- `threshold_m`, hysteresis, window size 현장 튜닝
- 타이어 곡면 반사와 그리퍼 가림 시험
- 모터 노이즈 환경에서 UART/거리 안정성 시험
- Front/Rear 각각 20회 반복 정렬 통계

## 9. 현재 검증 범위

코드 정합성 검증은 Python 문법, 단위시험, launch 정적 계약, UART parser, STM32 GNU11 문법 검사까지 수행한다. 실제 STM32 링크·플래시, TIM9 동작, GPIO EXTI, HC-SR04 실측과 ROS 2 Humble 런타임은 실제 장비에서 추가 검증해야 한다.


## 10. 정적 검증 결과

v1.6 소스 기준으로 다음을 확인했다.

- Python 전체 문법 및 Python 3.10 AST 호환
- Pytest 63개 통과
- UART `U,L|R,...` parser 단위검사
- bridge 발행 토픽과 edge-node 구독 토픽 일치
- Front/Rear/full-system launch 파라미터 전달 계약
- RPi GPIO 직접 측정 경로 제거
- STM32 GNU11 `-Wall -Wextra -Werror` 문법 검사
- package.xml, YAML, NPZ 및 Python wheel 구성 검사

이 결과는 코드와 패키지의 정합성을 의미한다. 실제 STM32CubeMX 링크·플래시,
TIM9/EXTI 동작, HC-SR04 정확도, ROS 2 Humble 분산 실행과 모터 노이즈 환경은
실제 장비에서 별도로 검증해야 한다.
