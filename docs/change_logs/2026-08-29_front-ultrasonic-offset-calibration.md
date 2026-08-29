# Front 초음파-그리퍼 X offset 수동 주행 보정

## 범위와 안전 구성

- 일시: 2026-08-29 15:14~15:18 KST
- 로봇: Front `robot-2` 한 대
- source: `main` `3f3ab73`
- 설치본: `/home/robot/parkingbot_main_3f3ab73`
- ROS domain: `142`
- 실행: `stm32_bridge` 하나와 Front `keyboard_teleop`, rosbag recorder
- 미실행: Rear, 자동 state machine, `individual_move`, grip/lift, Fleet
- servo attach: robot-2 벌림값 `(2600, 400)` ACK 확인
- 시작/종료: `hardware_ready=true`, 네 바퀴 RPM/PWM 0 확인

사용자가 Front를 수동으로 뒷차축과 앞차축에 각각 정렬하고, 차량을 완전히
통과한 뒤 후진했다. bag에는 wheel odometry, 좌·우 초음파, manual command와
hardware/motor/heartbeat 상태만 기록했다.

## 기록

- 원격 bag:
  `/home/robot/parkingbot_logs/front-offset-main3f3ab73-20260829-1515`
- 로컬 사본:
  `/home/guitest/parkingbot_analysis/front-offset-main3f3ab73-20260829-1515`
- 기간: `264.076 s`
- message: `34,909`
- DB3 SHA256:
  `6ddd2c7597c640980f3f1b6f598dc2184acab365043dd03112b0fd4437c973ae`

최종 pose는 시작 기준 `x=+0.42877m`, `y=+0.01714m`, yaw `-0.936°`였다.
사용자가 말한 것처럼 후진 뒤 완전 원점 복귀를 의도한 시험은 아니다.

## 분석 방법

초음파 0.10m threshold 진입·이탈 odometry의 중점을 전진과 후진에서 각각
계산했다. 정지 중 echo와 짧은 노이즈 때문에 구간이 나뉘어도 같은 물리 바퀴의
최소·최대 X를 하나의 encounter로 합쳤다. 0.08/0.10/0.12m threshold sweep으로
결론의 민감도도 확인했다.

접근 초기에 멈춘 `x=0.15528m` 지점은 좌·우 어느 센서에도 바퀴 echo가 없어
차축 정렬점에서 제외했다. 차축을 실제로 통과한 구간의 10cm threshold 결과는
다음과 같다.

| 차축 | 센서 | 전진 중심 X (m) | 후진 중심 X (m) | 평균 X (m) |
|---|---|---:|---:|---:|
| Rear | Left | 0.81396 | 0.81905 | 0.81651 |
| Rear | Right | 0.81396 | 0.81139 | 0.81267 |
| Front | Left | 1.58811 | 1.59266 | 1.59039 |
| Front | Right | 1.58504 | 1.59274 | 1.58889 |

- 같은 바퀴의 전진/후진 중심 반복 차이: 최대 약 `8mm`
- 센서 중심 기반 wheelbase: Left `0.77388m`, Right `0.77622m`
- threshold sweep wheelbase 범위: `0.77085~0.78094m`
- 기존 실측 wheelbase `0.785m` 대비 차이: 최대 약 `14mm`

사용자가 차축 중심에 맞춘 안정 정지 pose는 Rear `x=0.82982m`, Front
`x=1.58081m`로 식별했다. `offset = sensor-center event X - gripper-aligned X`를
두 차축과 양방향에서 평균하면 다음과 같다.

| 센서 | 10cm threshold 추정 | 8/10/12cm sweep 범위 |
|---|---:|---:|
| Front Left | 약 `-0.002m` | 약 `-0.003~0.000m` |
| Front Right | 약 `-0.005m` | 약 `-0.005~0.000m` |

수동 육안 정렬의 실용 오차를 약 ±2cm로 보면 추정 offset은 그보다 작고 차축별
부호도 일관되지 않는다. millimetre 단위 음수를 설정 상수로 넣으면 이번 시험의
수동 정렬 오차를 과적합하게 된다.

## 결정

Front는 다음 값을 유지한다.

```text
FRONT_LEFT_SENSOR_X=0.0
FRONT_RIGHT_SENSOR_X=0.0
```

이 결정의 현장 유효 오차 범위는 약 `±0.02m`다. Front의 `0.0m`는 이번 수동
왕복 시험에서 확인했지만 Rear 두 센서에는 이 결과를 복사하지 않았다. 이 Front
시험 시점에는 Rear 좌·우 offset이 미확정이었다.

후속 Rear 자체 측정은
[Rear offset 수동 보정](./2026-08-29_rear-ultrasonic-offset-calibration.md)에
별도로 기록했다.
