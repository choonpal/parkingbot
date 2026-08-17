# STM32 v1.11 제어 펌웨어 통합 기록

## 적용 기준

- 기준 CubeIDE 프로젝트: parking_robot_stm32cubeide_20260724.zip
- 통합 제어 소스: Adaptive Valet Bot v1.11의 parking_robot_firmware.c
- 대상 MCU: STM32F401RE
- CubeIDE 원본 ZIP과 v1.11 원본 ZIP은 변경하지 않음

## 초음파 핀 매핑

CubeIDE의 parking_robot.ioc를 기준으로 다음처럼 연결했다.

| 역할 | CubeIDE 이름 | 핀 | 설정 |
|---|---|---|---|
| Left TRIG | ULTRASONIC1_TRIG | PC8 | GPIO output, 초기 LOW |
| Left ECHO | ULTRASONIC1_ECHO | PC6 | EXTI rising/falling |
| Right TRIG | ULTRASONIC2_TRIG | PC5 | GPIO output, 초기 LOW |
| Right ECHO | ULTRASONIC2_ECHO | PC7 | EXTI rising/falling |

HC-SR04 ECHO의 5V 신호를 STM32에 직접 연결하지 말고 저항분압 또는
레벨시프터를 사용한다.

## 모터 PWM 스케일

제어기의 PID 출력 범위는 0~999이고 CubeIDE TIM1의 현재 ARR는 65535다.
펌웨어는 아래 비율로 CCR을 계산한다.

    CCR = round(abs(command) / 999 * ARR)

현재 설정의 대표값은 다음과 같다.

| PID 명령 | TIM1 CCR | 대략적인 duty |
|---:|---:|---:|
| 0 | 0 | 0% |
| 100 | 6560 | 10.0% |
| 500 | 32800 | 50.1% |
| 999 | 65535 | 약 100% |

ARR를 변경해도 소스가 런타임에 ARR를 읽기 때문에 duty 비율은 유지된다.
현재 TIM1은 84 MHz, PSC 0, ARR 65535이므로 PWM 주파수는 약 1.282 kHz다.
모터 드라이버가 20 kHz를 요구하면 PSC 0에서 ARR 4199가 되지만, 실제 변경은
모터 드라이버 데이터시트와 발열 시험 후 결정한다.

## 제어 루프 통합

- Core/Inc/parking_robot_firmware.h 추가
- Core/Src/parking_robot_firmware.c 추가
- 주변장치 초기화 후 Robot_Init() 호출
- while 루프에서 Robot_MainLoop() 반복 호출

## 확인 결과

- 실제 프로젝트의 STM32 HAL/CMSIS 헤더를 사용한 GNU11 구문 검사 통과
- CubeIDE source path가 Core 전체를 포함하므로 새 C 소스는 빌드 대상
- 이 시스템에는 ARM GNU Toolchain 또는 STM32CubeIDE 실행기가 없어 실제
  ARM 링크, HEX 생성 및 보드 플래시는 아직 수행하지 못함

## 플래시 전 확인

- MOTOR1/2/3/4가 각각 FL/FR/RL/RR 배선인지 확인
- 초음파 1이 실제 Left, 초음파 2가 실제 Right인지 확인
- 각 바퀴의 kMotorCommandSign, kEncoderSign을 잭업 저속 시험으로 확정
- 유효 wheel radius, encoder PPR, LX, LY 실측
- 초음파 50/100/200 mm 측정 및 timeout 시험
- 물리 ESTOP과 모터 전원 차단 수단 준비
