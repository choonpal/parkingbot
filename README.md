# parkingbot v1.11

ROS 2 Humble 협동 주차로봇과 STM32F401RE 제어 펌웨어를 한 배포본으로 묶은
패키지다.

## 먼저 읽을 문서

1. docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md — 탑재, 실행, STM32 핀맵, UI, 복구
2. docs/REAL_WORLD_READINESS.md — 현재 가능한 시험 범위와 NO-GO 조건
3. docs/pipeline.md — calibration부터 분산 기동·실차 검증까지의 상세 절차

## 구성

- ros2/cooperative_parking_robot: Adaptive Valet Bot v1.11 ROS 2 패키지
- stm32/parking_robot: CubeIDE 핀 설정과 v1.11 제어코드가 통합된 플래시 대상
- docs: 실차 준비도 및 배포 설명서

원본 ZIP은 변경하지 않았으며, 이 통합본의 STM32 기준은
stm32/parking_robot이다. 실제 ARM 링크·플래시와 하중 시험 전에는 무인 차량
인양에 사용하지 않는다.
