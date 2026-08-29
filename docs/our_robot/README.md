# Our Robot Hardware Profile

최종 갱신: 2026-08-29
적용 대상: 제작 중인 주차로봇 2대와 중앙 인프라

저장소 루트 `README.md`는 소프트웨어 패키지와 실행 문서를 안내한다. 이 폴더는 실제 제작 중인 로봇의 부품, 시스템 구조, 배선 및 시험 상태를 관리한다.

## 기준 문서

1. [`BOM.md`](BOM.md) — 부품명, 수량, 구매·입고 상태
2. [`SYSTEM_HANDOFF.md`](SYSTEM_HANDOFF.md) — 시스템 구조, Front/Rear 역할, 소프트웨어·통신 설계
3. [`ELECTRICAL_WIRING.md`](ELECTRICAL_WIRING.md) — 전압, 배선, 핀맵, 조립·통전 절차

보조 문서:

- [`TEST_LOG.md`](TEST_LOG.md) — 날짜별 벤치 시험 수치와 문제 해결 이력
- [`KNOWN_ISSUES.md`](KNOWN_ISSUES.md) — 재현된 미해결 문제와 해결 검증 상태

문서가 충돌하면 부품·재고는 `BOM.md`, 전압·배선·핀은 `ELECTRICAL_WIRING.md`, 시스템 동작과 통신 구조는 `SYSTEM_HANDOFF.md`를 우선한다.

## 현재 확정 사항

- Front=`robot-2`(상판 ID2, 후방 ID0), Rear=`robot-1`(상판 ID1)
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

- 확인됨: STM32 수동 주행, 엔코더, 서보, 초음파 제어와 로봇별 방향·서보 한계.
  Front(`robot-2`)의 잭업 폐루프 3축·무하중 바닥 저속 주행과 Rear(`robot-1`) 교체
  Nucleo의 flash/readback·단독 주행을 확인했다. 강체 쌍 시험은 `W/S`와 정지를
  확인했고 이후 `A/D/Q/E` 부호, ArUco 연속성, 진행각 유지 보정을 통합했다.
- 소프트웨어 통합됨: HELLO/session heartbeat 복구, phase별 command owner,
  Front-first 하부 진입, 초음파 차축 정렬, `stop_after_align`, 640x360 듀얼 CCTV와
  5008 관제탑. Front와 Rear에는 각각 격리 설치본을 배포해 수동 시험했다.
- 실기 차단: Front/Rear ultrasonic-to-gripper X offset 네 값 `0.0m`는 역할별 수동
  왕복으로 확인했다. 세 장비 동일 SHA 배포와 단계별 실기 gate가 남아 자동 진입은
  NO-GO다. `A/D/Q/E` 수정 뒤 실기, 초음파 차축 반복정밀도,
  보호 지그 저하중과 전체 park/retrieve cycle도 남아 있다.
- 현재 STM32 watchdog: 주행 명령 250 ms, heartbeat 300 ms

현재 배포·시험 판정은 [현재 통합 상태](../CURRENT_INTEGRATION_STATUS.md)를 우선한다.

호스트 로그인 정보, 내부 IP, 장치 일련번호 등 운영 정보는 이 공개 저장소에서 관리하지 않는다.
