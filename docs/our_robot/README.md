# Our Robot Hardware Profile

최종 갱신: 2026-08-25
적용 대상: 제작 중인 주차로봇 2대와 중앙 인프라

저장소 루트 `README.md`는 소프트웨어 패키지와 실행 문서를 안내한다. 이 폴더는 실제 제작 중인 로봇의 부품, 시스템 구조, 배선 및 시험 상태를 관리한다.

## 기준 문서

1. [`BOM.md`](BOM.md) — 부품명, 수량, 구매·입고 상태
2. [`SYSTEM_HANDOFF.md`](SYSTEM_HANDOFF.md) — 시스템 구조, Front/Rear 역할, 소프트웨어·통신 설계
3. [`ELECTRICAL_WIRING.md`](ELECTRICAL_WIRING.md) — 전압, 배선, 핀맵, 조립·통전 절차

보조 문서:

- [`TEST_LOG.md`](TEST_LOG.md) — 날짜별 벤치 시험 수치와 문제 해결 이력

문서가 충돌하면 부품·재고는 `BOM.md`, 전압·배선·핀은 `ELECTRICAL_WIRING.md`, 시스템 동작과 통신 구조는 `SYSTEM_HANDOFF.md`를 우선한다.

## 현재 확정 사항

- Front=`robot-2`(후방 ArUco ID0), Rear=`robot-1`(무마커)
- Raspberry Pi 4 두 대: Ubuntu Server 22.04.5 LTS ARM64 + ROS 2 Humble
- STM32 Nucleo F401RE: 메카넘 역기구학, 휠 PID, 서보 실시간 제어 담당
- 주행 모터: RB-35GM+Encoder DC24V, 1/100 ×8
- 기준 엔코더 실측값: STM32 4체배 기준 출력축 1회전당 약 5,182카운트, 내부 풀업 사용
- MG996R ×4, 서보 전원은 6.0V부터 설정
- TB-2506은 로봇당 1개, 1~3극 +24V BUS / 4~6극 GND BUS
- OV2710은 총 6개 확보: 운용 4개, 예비 2개
- HC-SR04는 총 4개 확보
- 모든 ROS 2 장치는 `ROS_DOMAIN_ID=42` 사용

## 현재 구현 범위

- 확인됨: STM32 수동 주행, 엔코더, 서보, 초음파 제어와 로봇별 방향·서보 한계. Front(`robot-2`)의 ROS 2 bridge, 잭업 폐루프 3축 주행, 무하중 바닥 저속 키보드 주행. 2026-08-22에는 Ramp 15를 적용해 전진·회전·횡이동을 다시 검증하고 Raspberry Pi에서 STM32를 직접 기록·검증하는 절차를 확인했다. 2026-08-25에는 Rear(`robot-1`) 교체 Nucleo에 Rear 펌웨어를 기록·읽기 검증하고 정지 UART와 400/2600µs 서보 프로파일을 확인했다.
- 통합 필요: Rear(`robot-1`) RR 엔코더 신호 하네스·커넥터 수리와 네 바퀴 배선 원복·모터/PID 검증, Jetson 비전·전역 경로계획, ArUco 상대 측위, 두 로봇 강체 동기 주행과 차량 하중 시험. `robot-1`의 기존 Nucleo는 소손·격리했고 교체품은 장착했다.
- 현재 STM32 명령 단절 정지 기준: 250ms

호스트 로그인 정보, 내부 IP, 장치 일련번호 등 운영 정보는 이 공개 저장소에서 관리하지 않는다.
