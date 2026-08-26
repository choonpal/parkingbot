# 문서 안내

이 저장소의 문서는 현재 운용 기준과 과거 설계 기록을 구분한다. 실행 전에
아래의 현재 문서를 먼저 확인하고, 날짜나 버전이 붙은 문서는 당시 판단을
추적할 때만 참고한다.

## 현재 문서

| 문서 | 역할 |
|---|---|
| [실차 탑재·실행 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md) | 설치, 분산 기동, UI, 복구의 기준 절차 |
| [실차 준비도](REAL_WORLD_READINESS.md) | 현재 검증 범위와 GO/NO-GO |
| [배포 Pipeline](pipeline.md) | calibration, Homography, preflight 흐름 |
| [프로젝트 용어](../CONTEXT.md) | 미션·슬롯·Registry 공통 언어 |
| [하드웨어 문서](our_robot/README.md) | BOM, 배선, 실기 시험 기록 |
| [ADR](adr/) | 설계 결정과 변경 이유 |

ROS 패키지의 노드·launch 개요는
[cooperative_parking_robot README](../ros2/cooperative_parking_robot/README.md)를
본다. 듀얼 CCTV의 타일 등록 절차는
[Homography 도구 README](../dual_tile_homography_tool/README.md)가 기준이다.

## Rear 단독 실험

Rear 한 대와 차량 한 대의 접근·진입·grip 실험은 production 협동 미션과 다른
branch/worktree에서 진행한다. 해당 실험의 launch, 마커 ID, 등록 pose와 안전
게이트는 그 worktree의 `ros2/rear_single_vehicle_experiment/README.md`만
기준으로 삼는다. 실험값을 Front/Rear 협동 운용 문서에 그대로 적용하지 않는다.

## 과거 기록

`ros2/cooperative_parking_robot/docs/`의 날짜·버전 기반 계획, merge 기록,
변경 보고서는 당시 구현과 검증 결과를 보존한다. 문서 상단에 현재 대체 문서가
표시된 경우, 실행 명령이나 기본값은 대체 문서를 우선한다. ADR과 시험 로그는
기록 보존을 위해 본문을 소급 수정하지 않는다.

## 문서보다 우선하는 항목

- 실제 launch parameter와 config 기본값
- `stm32/parking_robot/parking_robot.ioc`, `main.h`, firmware 상수
- Jetson의 `~/.ros/adaptive_valet_bot/`에 배포된 현장 calibration
- 현장에서 확인한 배선, 차체 치수와 센서 offset

문서와 코드가 다르면 임의로 운행하지 말고 차이를 기록한 뒤 확인한다. 소프트웨어
ESTOP은 물리 비상정지나 모터 전원 차단을 대체하지 않는다.
