# parkingbot 1.11.0

ROS 2 Humble 협동 주차 로봇, Jetson 인지/UI, STM32F401RE 제어 펌웨어를
한 저장소에 묶은 프로젝트다. 입차와 인증 기반 출차(retrieve)는 소프트웨어에
구현되어 있지만, 실차 하중 안전이 검증됐다는 뜻은 아니다.

## 문서

- [현재 문서 안내](docs/README.md) — 실행 전에 먼저 확인
- [현재 통합 상태](docs/CURRENT_INTEGRATION_STATUS.md) — 배포 기준과 실기 GO/NO-GO
- [실차 탑재·실행 Runbook](docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md) — 배포, 기동, UI, 복구
- [실차 준비도](docs/REAL_WORLD_READINESS.md) — GO/NO-GO 판정
- [Calibration pipeline](docs/pipeline.md) — 카메라, Homography, layout, preflight
- [ROS 2 패키지 문서](ros2/cooperative_parking_robot/docs/README.md) — 기능별 실행·시험 문서

## 구성

- `ros2/cooperative_parking_robot` — ROS 2 Humble 패키지 1.11.0
- `stm32/parking_robot` — Front/Rear 공통 STM32CubeIDE 프로젝트
- `docs` — 현재 운용 문서와 과거 설계 기록

이 통합본의 STM32 기준은 `stm32/parking_robot`이다. Front=`robot-2`,
Rear=`robot-1`이며 역할별 firmware profile을 따로 빌드·플래시한다. 2026-08-29
기준 pregrip 소프트웨어 통합과 clean build는 통과했다. Front에는 같은 `main`
설치본을 배포해 수동 왕복으로 좌·우 초음파-그리퍼 X offset `0.0m`를 확인했지만,
Rear 두 값과 Jetson/Rear 동일 SHA 배포가 남아 있어 차량 하부 자동 진입은
**NO-GO**다. 정확한 현재 판정은
[현재 통합 상태](docs/CURRENT_INTEGRATION_STATUS.md)를 따른다.

저장소 기본 branch 또는 검증된 release를 배포한다. 과거 실험 branch를 운용
기준으로 사용하지 않는다. 두 로봇의 차량 하중 시험, 보호 지그, 물리 ESTOP,
사람 감독과 실제 파지·하중 확인 수단이 준비되기 전 무인 차량 인양은 **NO-GO**다.
