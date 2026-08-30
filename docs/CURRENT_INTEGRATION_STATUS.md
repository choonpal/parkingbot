# 현재 통합 상태 — 2026-08-30

이 문서는 지금 배포·시험할 통합 후보와 실기 허용 범위를 한 곳에서 요약한다.
세부 실행 절차는 [실차 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), 최종 안전
판정은 [실차 준비도](REAL_WORLD_READINESS.md)를 따른다.

## 2026-08-30 strict 복구 상태

현재 통합 후보는 `ddad9ac`를 기준으로 다시 만든
`recovery/ddad-strict-20260830`이며 현재 배포한 기능/운영 guard 기준점은
`89081ad`이다. 상세 변경과
실차 로그는 [ddad strict 복구·기동 부하 감사](change_logs/2026-08-30_ddad-strict-recovery-and-startup-audit.md)에
기록했다.

- 세 장비 `parkingbot_active` 동일 release 배포와 `robot_doctor`: PASS
- Jetson 카메라 각 단독 및 TensorRT cam0/cam2 각 단독: PASS
- Jetson dual TensorRT 순차 cold-load: 회복은 PASS, 두 번째 load 중 약 4초
  perception gap과 `NvMapMemAlloc error 12` 발생
- Jetson shared TensorRT 1개/dual-camera round-robin: 무구동 PASS. cam0/cam2
  각 5Hz 시험에서 70초간 재중단·NvMap 오류 없음, RAM 약 4.27/7.62GB
- stationary target snapshot과 mission 중 YOLO unload/reload: 구현·회귀 PASS,
  실제 `WAIT_LIFT` 전환 미검증
- Front/Rear UART bridge-only 및 shared DDS+Jetson CAM0/CAM2/YOLO0 join: PASS
- 전체 production cold-start: 두 차례 FAIL
- 현재 production stack: 모두 종료
- 이번 복구 작업의 차량 이동: 없음

전체 기동 중 RPi heartbeat gap, UART write timeout과 SSH 불안정이 재현됐으므로
차량 하부 자동 진입과 grip/lift/운반은 모두 **NO-GO**다. 다음 시험은 Jetson 없이
RPi 한 대씩 보조 노드를 하나씩 올리는 방식으로만 진행한다. 300ms watchdog을
늘려 통과시키지 않는다.

Jetson/control은 `${HOME}/parkingbot_active`, Front/Rear는 각 호스트의
`/home/robot/parkingbot_active`에서 같은 release를 사용한다. package-only 로봇
workspace는 `.parkingbot_revision`으로 revision을 고정하고, start 전 ROS prefix와
Python import realpath가 active workspace 안인지 검사한다.

## 이전 2026-08-29 기준

- 통합 감사 branch: `integration/unified-pregrip-20260829`
- 소프트웨어 감사 기준: `c118de8`
- 목표 시험: Front/Rear가 차량 하부로 진입해 각 차축에 정렬한 뒤 정지
- 시험 경계: `stop_after_align=true`; grip, lift, 운반은 범위 밖
- 최신 상세 감사: [2026-08-28~29 통합 감사](change_logs/2026-08-29_unified-pregrip-update-audit.md)

감사 뒤 문서만 바뀐 커밋이 추가될 수 있으므로 실제 배포 SHA는
`git rev-parse HEAD`로 기록한다. 양쪽 로봇과 Jetson은 반드시 같은 SHA에서 빌드한
설치본을 사용한다.

## 현재 판정

| 구분 | 판정 | 근거 |
|---|---|---|
| pregrip 소프트웨어 통합 | 회귀 PASS / 실기 보류 | 기존 회귀는 유지됐지만 현재 strict 전체 cold-start가 heartbeat FAULT로 차단됨 |
| clean ROS 2 Humble build/import | PASS | 격리 workspace에서 package build와 install-only import 확인 |
| Jetson CCTV 운용값 | PASS(제약 있음) | 카메라 2대가 TensorRT 1개를 round-robin 공유하는 경로는 무구동 PASS; production 기본은 총 6Hz(카메라별 약 3Hz), mission pause/resume 실차 전환은 미검증 |
| 현재 통합본 원격 배포 | PASS | 세 장비 active release, revision marker, runtime import/prefix와 `robot_doctor` 일치 |
| 차량 하부 실기 모션 | **NO-GO** | 전체 cold-start heartbeat/UART FAULT와 SSH 불안정이 남음 |
| grip/lift/운반 | **범위 밖·NO-GO** | `stop_after_align` 뒤 기능이며 현재 감사에서 실차 검증하지 않음 |

과거 `/home/robot/parkingbot` 또는
`/home/robot/parkingbot_unified_pregrip_20260829` 설치본은 경로 이름만으로 최신이라고
판정하지 않는다. 재배포 후 source/installed SHA와 launch 인자를 대조해야 한다.

## 초음파-그리퍼 X offset 상태

현재 현장 설정의 네 값은 모두 `0.0`이며, Front와 Rear 각각의 2026-08-29 수동
왕복 bag 분석에서 약 ±0.02m 현장 오차 범위로 확인했다.

```text
FRONT_LEFT_SENSOR_X
FRONT_RIGHT_SENSOR_X
REAR_LEFT_SENSOR_X
REAR_RIGHT_SENSOR_X
```

각 값은 `gripper_x - ultrasonic_sensor_x`다. 근거는
[Front offset 수동 보정](change_logs/2026-08-29_front-ultrasonic-offset-calibration.md)과
[Rear offset 수동 보정](change_logs/2026-08-29_rear-ultrasonic-offset-calibration.md)에
기록했다. 이 측정은 자동 진입 통과 판정이나 wheel odometry 보정을 대신하지
않는다. Runbook의 동일 SHA 배포, 정적 통신 → 바퀴 공중 → 빈 차체 저속 gate를
순서대로 통과해야 한다.

## 현재 UI와 영상 역할

- Jetson 듀얼 CCTV 관제탑: `http://robot-desktop.local:5008/`
- Rear ID0 카메라 진단: `http://robot-1.local:5005/`
- 강체 쌍 키보드 시험 UI: `http://robot-1.local:5007/`
- 10 cm 협동 직진 시험 UI: `http://robot-1.local:5006/`
- 운용자 mission UI: Jetson `5000`

5008은 주차장 전체를 두 CCTV와 BEV로 확인하는 화면이고, 5005는 Rear 로봇의
전방 ID0 카메라 화면이다. 서로 목적과 카메라가 다르다.
