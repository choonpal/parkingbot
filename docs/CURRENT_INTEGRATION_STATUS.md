# 현재 통합 상태 — 2026-08-29

이 문서는 지금 배포·시험할 통합 후보와 실기 허용 범위를 한 곳에서 요약한다.
세부 실행 절차는 [실차 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), 최종 안전
판정은 [실차 준비도](REAL_WORLD_READINESS.md)를 따른다.

## 기준

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
| pregrip 소프트웨어 통합 | PASS | UART/heartbeat, ArUco, 진입·정렬, 정렬 후 정지, CCTV/UI 및 운용 회귀 통과 |
| clean ROS 2 Humble build/import | PASS | 격리 workspace에서 package build와 install-only import 확인 |
| Jetson CCTV 운용값 | PASS(정적) | 640x360 intrinsic/Homography, 현장 광축 지상점 우선, 5008 관제탑 기본 기동 확인 |
| 현재 통합본 원격 배포 | 부분 완료 | Front `robot-2`에 `3f3ab73` 격리 배포·수동시험 완료; Rear/Jetson 미배포 |
| 차량 하부 실기 모션 | **NO-GO** | Rear 좌우 초음파-그리퍼 X offset과 Rear/Jetson 동일 SHA 배포가 미확정 |
| grip/lift/운반 | **범위 밖·NO-GO** | `stop_after_align` 뒤 기능이며 현재 감사에서 실차 검증하지 않음 |

과거 `/home/robot/parkingbot` 또는
`/home/robot/parkingbot_unified_pregrip_20260829` 설치본은 경로 이름만으로 최신이라고
판정하지 않는다. 재배포 후 source/installed SHA와 launch 인자를 대조해야 한다.

## 초음파-그리퍼 X offset 상태

현재 현장 설정의 네 값은 모두 `0.0`이다. Front 두 값은 2026-08-29 수동 왕복
bag 분석에서 약 ±0.02m 현장 오차 범위로 확인했다.

```text
FRONT_LEFT_SENSOR_X
FRONT_RIGHT_SENSOR_X
```

근거는 [Front offset 수동 보정](change_logs/2026-08-29_front-ultrasonic-offset-calibration.md)에
기록했다. Rear의 다음 두 값은 아직 실제로 0인지 확인되지 않았다.

```text
REAR_LEFT_SENSOR_X
REAR_RIGHT_SENSOR_X
```

각 값은 `gripper_x - ultrasonic_sensor_x`다. Rear를 직접 측정해 기록하거나
실측값으로 교체하기 전에는 자동 진입을 실행하지 않는다. 그다음에도 Runbook의
정적 통신 → 바퀴 공중 → 빈 차체 저속 gate를 순서대로 통과해야 한다.

## 현재 UI와 영상 역할

- Jetson 듀얼 CCTV 관제탑: `http://robot-desktop.local:5008/`
- Rear ID0 카메라 진단: `http://robot-1.local:5005/`
- 강체 쌍 키보드 시험 UI: `http://robot-1.local:5007/`
- 10 cm 협동 직진 시험 UI: `http://robot-1.local:5006/`
- 운용자 mission UI: Jetson `5000`

5008은 주차장 전체를 두 CCTV와 BEV로 확인하는 화면이고, 5005는 Rear 로봇의
전방 ID0 카메라 화면이다. 서로 목적과 카메라가 다르다.
