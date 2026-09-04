# 현재 통합 상태 — 2026-09-04

이 문서는 현재 코드 구성과 축소형 로봇 시연을 정리한다.
시연 원본과 확인 범위는 [검증 기록](FINAL_VALIDATION_2026-09-04.md),
실제 기동 절차는 [Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), 현장 gate는
[실차 준비도](REAL_WORLD_READINESS.md)를 따른다.

## 기준

- 검토한 GitHub `main`: `6327c92e95fb3c960e42b05d44ea27e01d523077`
- ROS 패키지: `1.11.3` (`setup.py`, `package.xml` 일치)
- 생산용 제어 진입점: `rigid_body_sync_vehicle_global_node:main`
- 제공 시연: 2026-09-02 파일명 기준 영상 2건, 2026-09-04 검토

문서와 시연 자료는 [검증 기록](FINAL_VALIDATION_2026-09-04.md)의 제출 검토 태그로 관리한다.

## 현재 판정

| 구분 | 상태 | 근거·범위 |
|---|---|---|
| 접근·정렬·인양 명령·운반·복귀 | 소프트웨어 구현 | 생산용 launch, state machine, individual/rigid controller |
| 차량 전역 x/y 보정 | 소프트웨어 구현 | YOLO map bias, odom heading, ID0 formation의 분리 |
| 축소형 차량모형 시연 | 영상 확인 | 하부 진입, 차량 동반 이동, 이탈·복귀 장면 |
| Python CI | 기준 소스 SHA 통과 | [실행 결과](https://github.com/choonpal/parkingbot/actions/runs/33648148956); ROS 비의존 회귀와 compile, rclpy 미설치 시 ROS-node 검사는 skip |
| clean Humble build | 기존 통합 시험 통과 | 날짜별 결과는 [시험 기록](our_robot/TEST_LOG.md) 참조 |
| 현장 운용 | 단계별 안전 절차 적용 | 배포·통전·저하중 시험은 Runbook과 실차 준비도 기준 |
| 실차급·사람 없는 무인 인양·운반 | **NO-GO** | 현재 운용 범위는 축소형 시험기체이며 하중·안전 제한 적용 |

이전 문서의 “grip/lift/운반 범위 밖”은 당시 `stop_after_align=true`로 제한한
pregrip 시험 범위였다. 현재 소프트웨어는 전체 미션 흐름을 포함하며,
축소형 시연 기록은 위 표와 연결 문서에 정리했다.

## 정렬 시험 모드와 전체 미션

`front_robot.launch.py`, `rear_robot.launch.py`, `full_system.launch.py`의
`stop_after_align` 기본값은 `false`다. 정렬 단계만 확인할 때 양쪽 로봇에
`stop_after_align:=true`를 명시하면 정렬 후 hold하고 LIFT 진행을 차단한다.
`false`는 다른 ready/interlock 조건이 충족될 때 전체 미션을 진행하는 설정이다.
이 문서 갱신은 launch 값, safety limit 또는 로봇 실행 상태를 변경하지 않는다.

## 측정 기준

카메라 역할, 현장 calibration, 초음파-그리퍼 X offset과 마커 크기의 기준 위치는
[카메라·실측 기준](our_robot/CAMERA_CALIBRATION_BASELINE.md)에 모았다.
센서 재장착 뒤에는 과거 `0.0m` offset 확인을 그대로 재사용하지 않는다.

## 현재 UI와 영상 역할

- Jetson 듀얼 CCTV 관제탑: `http://robot-desktop.local:5008/`
- Rear ID0 카메라 진단: `http://robot-1.local:5005/`
- 10 cm 협동 직진 시험 UI: `http://robot-1.local:5006/`
- 운용자 mission UI: Jetson `5000`

5008은 주차장 전체를 두 CCTV와 BEV로 확인하는 화면이고, 5005는 Rear 로봇의
전방 ID0 카메라 화면이다. 서로 목적과 카메라가 다르다.
