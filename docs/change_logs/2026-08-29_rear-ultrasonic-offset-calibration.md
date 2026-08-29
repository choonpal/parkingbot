# Rear 초음파-그리퍼 X offset 수동 주행 보정

## 범위와 안전 구성

- 일시: 2026-08-29 15:39~15:42 KST
- 로봇: Rear `robot-1` 한 대
- source: `main` `9586439`
- 설치본: `/home/robot/parkingbot_main_9586439`
- ROS domain: `142`
- 실행: `stm32_bridge` 하나와 Rear `keyboard_teleop`, rosbag recorder
- 미실행: Front, 자동 state machine, `individual_move`, grip/lift, Fleet
- servo attach: robot-1 벌림값 `(400, 2600)` ACK 확인
- 시작/종료: `hardware_ready=true`, 네 바퀴 RPM/PWM 0 확인

사용자가 Rear를 수동으로 뒷차축과 앞차축에 각각 정렬하고, 차량을 완전히
통과한 뒤 후진했다. bag에는 wheel odometry, 좌·우 초음파, manual command와
hardware/motor/heartbeat 상태만 기록했다.

## 기록

- 원격 bag:
  `/home/robot/parkingbot_logs/rear-offset-main9586439-20260829-1538`
- 로컬 사본:
  `/home/guitest/parkingbot_analysis/rear-offset-main9586439-20260829-1538`
- 기간: `188.170 s`
- message: `26,589`
- DB3 SHA256:
  `3db3802731a33c95f6cdbf093814ccea1c6bc2b66829cbe5eb457245b97572bd`

최종 pose는 시작 기준 `x=+0.21821m`, `y=+0.01768m`, yaw `-0.626°`였다.
후진 뒤 완전 원점 복귀를 의도한 시험은 아니다.

## 분석 방법

초음파 0.10m threshold 구간의 최소·최대 odometry 중점을 전진과 후진에서 각각
계산했다. 정지 중 echo와 짧은 노이즈로 나뉜 구간은 같은 물리 바퀴 encounter로
합쳤다. 0.08/0.10/0.12m threshold sweep으로 결론의 민감도도 확인했다.

10cm threshold 결과:

| 차축 | 센서 | 전진 중심 X (m) | 후진 중심 X (m) | 평균 X (m) |
|---|---|---:|---:|---:|
| Rear | Left | 0.62350 | 0.63029 | 0.62690 |
| Rear | Right | 0.62960 | 0.63029 | 0.62995 |
| Front | Left | 1.38934 | 1.39390 | 1.39162 |
| Front | Right | 1.39234 | 1.39698 | 1.39466 |

- 같은 바퀴의 전진/후진 중심 반복 차이: 최대 약 `6.8mm`
- 센서 중심 기반 wheelbase: Left `0.76472m`, Right `0.76471m`
- threshold sweep wheelbase 범위: `0.76090~0.77245m`

사용자가 차축 중심에 맞춘 안정 정지 pose는 Rear `x=0.63669m`, Front
`x=1.39600m`로 식별했다. `offset = sensor-center event X - gripper-aligned X`는
코드 정의인 `gripper_x - sensor_x`와 같다. 두 차축과 양방향 평균은 다음과 같다.

| 센서 | 10cm threshold 추정 | 8/10/12cm sweep 범위 |
|---|---:|---:|
| Rear Left | 약 `-0.007m` | 약 `-0.007~-0.006m` |
| Rear Right | 약 `-0.004m` | 약 `-0.004~-0.003m` |

수동 육안 정렬의 실용 오차를 약 ±2cm로 보면 추정 offset은 그보다 작다.
millimetre 단위 음수를 설정 상수로 넣으면 이번 시험의 수동 정렬 오차를
과적합하게 된다.

## 결정과 남은 gate

Rear는 다음 값을 유지한다.

```text
REAR_LEFT_SENSOR_X=0.0
REAR_RIGHT_SENSOR_X=0.0
```

이 결정의 현장 유효 오차 범위는 약 `±0.02m`다. 이로써 Front/Rear 네 offset의
`0.0m`는 각각의 수동 왕복 시험으로 확인됐다.

센서 기반 wheelbase는 기존 설정·실측 기준 `0.785m`보다 약 `13~24mm` 짧았다.
이 차이는 offset 수 mm를 채택할 근거가 아니며, 이 시험도 wheel radius/PPR과
odometry scale 보정을 대신하지 않는다. 세 장비 동일 SHA 배포와 정적 통신 →
바퀴 공중 → 빈 차체 저속 gate를 통과하기 전 자동 하부 진입은 계속 NO-GO다.
