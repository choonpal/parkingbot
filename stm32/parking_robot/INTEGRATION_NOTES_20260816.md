# STM32 실차 코드 통합 기록 (2026-08-20 갱신)

## 적용 기준

- 하드웨어 기준: `real_robot_code/stm32/parking_robot_f401`
- 통합 대상: `stm32/parking_robot`
- `real_robot_code`는 당시 실차에서 동작한 원본 기록이므로 수정하지 않는다.
- CubeMX의 TIM9/EXTI 초음파 상태머신은 유지하되, 모터·엔코더·서보 값은
  실차 코드 기준으로 교체했다.

## 실차에서 가져온 값

- 논리 모터 순서: FL=`TIM1_CH1`/`TIM5`, FR=`TIM1_CH2`/`TIM2`,
  RL=`TIM1_CH4`/`TIM4`, RR=`TIM1_CH3`/`TIM3`
- 방향 핀: RL=`MOTOR4/PC3`, RR=`MOTOR3/PC2`. 뒤쪽 PWM이 4/3 순서이므로
  DIR도 같은 논리 순서로 맞춰야 한다.
- 모터 및 엔코더 부호: `{1, -1, 1, -1}`
- 엔코더: 출력축 1회전 5182 count, 입력 filter 15, GPIO pull-up
- 모터 PWM: TIM1 PSC 83, ARR 999, 명령 최대 350(약 35% duty)
- 속도 제어: 50 ms, 12 rpm 기준 feed-forward 200과 정수 PI
- 서보: 20 ms마다 최대 30 us 이동, 실차별 pulse 제한 적용
- 주행 명령 watchdog: 250 ms

## 로봇 프로필

`parking_robot_firmware.h`의 `PARKING_ROBOT_PROFILE`로 선택한다.

| 값 | 대상 | Servo 1 범위 | Servo 2 범위 |
|---:|---|---:|---:|
| 1 | front / robot-2 / ArUco 장착 | 1550~2600 us | 400~1450 us |
| 2 | rear / robot-1 | 400~1600 us | 1400~2600 us |

기본값은 1이다. Rear 바이너리를 만들 때 compiler preprocessor define에
`PARKING_ROBOT_PROFILE=2`를 추가한다.

## UART 호환 방식

- 기존 실차 시험 명령 `W/S/A/D/Q/E`, 소문자 open-loop 명령,
  `U/J/I/K/T/G/O/X`는 그대로 유지한다.
- ROS 프레임은 단일문자 `S`, `E`와 충돌하지 않도록 `@`로 시작한다.
  - `@V,vx,vy,w`
  - `@S,grip` / `@S,release`
  - `@HB,timestamp`
  - `@ESTOP`
- STM32는 누적 엔코더 `E,...`, 초음파 `U,...`, ACK/ERR와 함께 기존
  14-field `T,...` telemetry도 계속 보낸다.

## PC 확인 결과

- Python/launch 구문 검사 통과
- STM32·UART·엔코더·초음파 관련 테스트 32개 통과, 1개 skip
- ROS가 필요한 테스트 3개를 제외한 전체 PC 테스트 213개 통과,
  1개 skip. Windows에서만 의미가 다른 POSIX 실행권한/0600 권한 검사
  2개는 실패했으며 코드 회귀가 아니다.
- STM32CubeIDE 2.2.0 / ARM GCC 14.3으로 Front와 Rear 프로필 모두 전체
  컴파일·링크 완료: 각각 0 errors, 0 warnings.
- 메모리 사용량: text 33,780 B, data 92 B, bss 3,004 B.
- 플래시용 ELF/HEX/BIN은 저장소 밖 `stm32_build_outputs/front_robot_2`,
  `stm32_build_outputs/rear_robot_1`에 구분해 생성했다.

## Front(robot-2) 실차 확인 결과

- 최종 Front 바이너리 SHA256:
  `8E1FE87275733603CF9FF0B9E34F04D89479CD8349C388DF8FDA28A1921E91D0`
- 단일 바퀴 `±120 PWM`에서 RL/RR 모두 실제 방향과 엔코더 부호가 함께
  반전됨을 확인했다.
- 잭업 폐루프 직진 5초, 회전 12초, 횡이동 8초에서 네 바퀴가
  `11.3~12.0 rpm`, PWM 약 `±199~212`로 안정했다.
- 무하중 바닥에서 ROS 키보드 저속 주행과 입력 단절 정지, `Ctrl+C` 종료를
  확인했다.

## 남는 확인

1. Rear(robot-1) 프로필을 다시 빌드해 해당 STM32에 기록한다.
2. Rear의 단일 바퀴 방향, 잭업 폐루프 3축, 바닥 저속 주행을 같은 순서로
   확인한다.
3. 두 로봇의 초음파 좌/우 거리와 timeout을 최종 장착 상태에서 확인한다.
4. 두 로봇을 최종 인양 하중에서 시험하고 전압강하와 발열을 확인한다.
