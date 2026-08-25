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

이 통합본의 STM32 기준은 `stm32/parking_robot`이다. 2026-08-25 기준
Front(`robot-2`)는 ARM 링크·플래시, ROS bridge, 잭업 폐루프 3축 주행과
무하중 바닥 저속 키보드 주행까지 확인했다. Rear(`robot-1`)는 교체 Nucleo의
Rear 펌웨어 기록·읽기 검증과 정지 UART까지 확인했지만 RR 엔코더 신호 경로
수리와 주행 검증이 남았다. 두 로봇의 차량 하중 시험 전에는 무인 차량 인양에
사용하지 않는다.
