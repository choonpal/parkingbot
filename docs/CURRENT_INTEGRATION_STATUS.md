# 현재 통합 상태 — 2026-09-02

이 문서는 지금 배포·시험할 통합 후보와 실기 허용 범위를 한 곳에서 요약한다.
세부 실행 절차는 [실차 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), 최종 안전
판정은 [실차 준비도](REAL_WORLD_READINESS.md)를 따른다.

## 기준

- 통합 기준: `main`
- 운용 경계: `stop_after_align=true`; grip, lift, 운반은 별도 현장 승인 전 비활성
- 검증 기준: clean ROS 2 Humble build와 package 전체 자동 회귀

실제 배포 SHA는 `git rev-parse HEAD`로 기록한다. 양쪽 로봇과 Jetson은 반드시
같은 SHA에서 빌드한 설치본을 사용한다.

## 현재 판정

| 구분 | 판정 | 근거 |
|---|---|---|
| pregrip 소프트웨어 통합 | PASS | UART/heartbeat, ArUco, 진입·정렬, 정렬 후 정지, CCTV/UI 및 운용 회귀 통과 |
| clean ROS 2 Humble build/import | PASS | 격리 workspace에서 package build와 install-only import 확인 |
| Jetson CCTV 운용값 | PASS(정적) | 640x360 intrinsic/Homography, 현장 광축 지상점 우선, 5008 관제탑 기본 기동 확인 |
| 제출 후보 자동 검증 | PASS | 전체 pytest와 격리 feature suite 통과; 환경 의존 skip 검토 완료 |
| 현재 통합본 원격 배포 | 미실행 | 동일 SHA 배포와 장비별 preflight 필요 |
| 차량 하부 실기 모션 | **NO-GO** | 동일 SHA 배포와 단계별 현장 gate 미완료 |
| grip/lift/운반 | **범위 밖·NO-GO** | `stop_after_align` 뒤 기능이며 별도 현장 검증 필요 |

기존 장비 설치본은 경로 이름만으로 최신이라고 판정하지 않는다. 재배포 후
source/installed SHA와 launch 인자를 대조해야 한다.

## 초음파-그리퍼 X offset 상태

현재 현장 설정의 네 값은 모두 `0.0`이다. 장비 재조립이나 센서 위치 변경 후에는
역할별 저속 보정 절차로 다시 확인해야 한다.

```text
FRONT_LEFT_SENSOR_X
FRONT_RIGHT_SENSOR_X
REAR_LEFT_SENSOR_X
REAR_RIGHT_SENSOR_X
```

각 값은 `gripper_x - ultrasonic_sensor_x`다. 이 설정은 자동 진입 통과 판정이나
wheel odometry 보정을 대신하지 않는다. Runbook의 동일 SHA 배포, 정적 통신 →
바퀴 공중 → 빈 차체 저속 gate를 순서대로 통과해야 한다.

## 현재 UI와 영상 역할

- Jetson 듀얼 CCTV 관제탑: `http://robot-desktop.local:5008/`
- Rear ID0 카메라 진단: `http://robot-1.local:5005/`
- 10 cm 협동 직진 시험 UI: `http://robot-1.local:5006/`
- 운용자 mission UI: Jetson `5000`

5008은 주차장 전체를 두 CCTV와 BEV로 확인하는 화면이고, 5005는 Rear 로봇의
전방 ID0 카메라 화면이다. 서로 목적과 카메라가 다르다.
