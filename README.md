# parkingbot 1.11.0

ROS 2 Humble 협동 주차 로봇, Jetson 인지/UI, STM32F401RE 제어 펌웨어를
한 저장소에 묶은 프로젝트다. 입차와 인증 기반 출차(retrieve)는 소프트웨어에
구현되어 있지만, 실차 하중 안전이 검증됐다는 뜻은 아니다.

## 문서

- [현재 문서 안내](docs/README.md) — 실행 전에 먼저 확인
- [실차 탑재·실행 Runbook](docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md) — 배포, 기동, UI, 복구
- [실차 준비도](docs/REAL_WORLD_READINESS.md) — GO/NO-GO 판정
- [Calibration pipeline](docs/pipeline.md) — 카메라, Homography, layout, preflight
- [ROS 2 패키지](ros2/cooperative_parking_robot/README.md) — 노드 개요와 launch index

## 구성

- `ros2/cooperative_parking_robot` — ROS 2 Humble 패키지 1.11.0
- `stm32/parking_robot` — Front/Rear 공통 STM32CubeIDE 프로젝트
- `docs` — 현재 운용 문서와 과거 설계 기록

이 통합본의 STM32 기준은 `stm32/parking_robot`이다. 2026-08-25 기준
Front(`robot-2`)는 ARM 링크·플래시, ROS bridge, 잭업 폐루프 3축 주행과
무하중 바닥 저속 키보드 주행까지 확인했다. Rear(`robot-1`)는 교체 Nucleo의
Rear 펌웨어 기록·읽기 검증과 정지 UART까지 확인했지만 RR 엔코더 신호 경로
수리와 주행 검증이 남았다.

저장소 기본 branch 또는 검증된 release를 배포한다. 과거 실험 branch를 운용
기준으로 사용하지 않는다. 두 로봇의 차량 하중 시험, 보호 지그, 물리 ESTOP,
사람 감독과 실제 파지·하중 확인 수단이 준비되기 전 무인 차량 인양은 **NO-GO**다.
