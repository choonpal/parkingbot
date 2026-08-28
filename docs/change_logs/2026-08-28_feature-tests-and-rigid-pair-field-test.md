# 기능별 테스트·강체 쌍 실차시험 main 통합

## 목적

기능별 테스트와 강체 쌍 키보드 시험이 별도 branch에서만 동작해 최신 main의
안전·통신 계약과 달라지는 문제를 없앤다. 2026-08-28 현장에서 실제로 사용한
카메라, ArUco, UART 및 웹 진단 설정을 코드와 문서에 함께 남기되, 통과하지 않은
실차 항목은 완료로 기록하지 않는다.

## 통합 기준

- 기준 main: `d1f9417` (`Improve fault recovery and hardware communication safety`)
- 실차 시험 변경: `feature/rigid-pair-teleop`의 9개 커밋
- 통합 작업은 기존 dirty worktree를 건드리지 않는 별도 worktree에서 수행했다.
- main의 phase-scoped ultrasonic, UART scheduler, recoverable fault 정책을 유지했다.
- 시험 branch의 serial startup drain, 중복 SIGINT 종료 보호, session 재설정,
  ID0 720p 검출 및 marker dropout 처리를 그 위에 결합했다.

## 오늘 확인한 것

### CCTV

- `robot-1.local`, `robot-2.local` 이름을 사용해 DHCP 주소 의존을 제거했다.
- 카메라별 slot/차량 overlay가 반대로 표시되던 매핑을 수정했다.
- detection topic을 preview UI에 연결해 카메라 상태, detection 수, freshness,
  slot/marker 진단을 함께 볼 수 있게 했다.
- 대형 상판 marker는 `DICT_4X4_50`, ID1=Rear, ID2=Front, ID3/4=예비,
  검은 정사각형 `0.24 m` 운용값을 사용한다.
- 카메라 busy, web port 충돌, TensorRT OOM은 코드상 동일 원인이 아니며 각각
  중복 camera owner, 기존 web process, 동시 engine load 여부를 확인해야 한다.

### Rear ID0와 강체 쌍

- robot-1 흰색 OV2710: `1280x720 @ 12 fps`
- calibration: `/home/robot/ov2710_calib_23mm_white.npz`
- Front 후면 ID0: `DICT_4X4_50`, 검은 정사각형 `0.10 m`
- 720p에서 mounting board 외곽 사각형 때문에 ID0 후보가 제거되지 않도록
  `minMarkerDistanceRate=0.02`를 tracker와 preview에 적용했다.
- 일시적인 관측 누락은 최대 0.60초 동안 마지막 측정으로 명령을 계속하지 않고
  안전 상태를 유지하며, 3회 연속 정상 관측 후에만 복구한다.
- 바퀴를 띄운 실차에서 `W/S` 방향과 정지를 확인했다.
- `A/D/Q/E`, 지면 주행, 장시간 주행은 아직 확인하지 않았다.

### 현장에서 드러난 운용 문제

robot-1에서 격리 시험(domain 142)과 다른 사용자의 production Rear stack
(domain 42)이 동시에 실행됐다. ROS 토픽은 충돌하지 않았지만 두 ArUco tracker,
상태기계, bridge, preview가 CPU와 카메라 자원을 함께 사용했다. robot-1은
78.8°C까지 올라 웹/영상 응답이 간헐적으로 끊겼고 시험을 중단해 전원을 내렸다.
따라서 domain 분리만으로 중복 launch를 안전하게 만들 수 없으며, 시험 전에는
OS process 수준에서도 camera·bridge·주행 owner가 하나인지 확인해야 한다.

## 기능별 테스트가 main을 따라가는 방식

`scripts/run_feature_tests.sh`는 localhost-only domain 177에서 production node의
상위 입력만 가짜로 넣고 실제 downstream 결과를 검사한다. 현재 그룹은
perception, localization, fleet, entry, mission, rigid-sync, rigid-pair 및 전체 DDS
integration이다. 기존 local node가 발견되면 가짜 데이터를 발행하지 않고 실패한다.

이번 통합에서 최신 main이 `/rear/ultrasonic_ready` phase gate를 추가한 사실을
기존 entry 시나리오가 반영하지 못해 테스트가 실제로 실패했다. 시나리오가
`ultrasonic_ready=true`를 격리 토픽으로 발행한 뒤 Range를 넣도록 수정했다. 이는
테스트가 별도 구현을 흉내 내는 대신 production 계약 변화에 맞춰 깨지고 갱신되는
의도한 동작이다.

## 통합본 검증 결과

- 통신·펌웨어·CCTV·ArUco·강체 핵심 회귀: `110 passed`
- 전체 기능 테스트: `736 passed, 2 skipped`
- DDS production-boundary 전용 실행: `6 passed`
- ROS 2 Humble clean overlay: `1 package finished`
- install 확인: `rigid_pair_teleop`, `camera_preview`, `stm32_bridge` 실행 파일 존재
- Rear launch 확인: 1280x720, 12 fps, ID0 0.10 m,
  `minMarkerDistanceRate=0.02`, preview 중복 ArUco 기본 비활성,
  rigid-pair opt-in 인자 존재
- ARM production build: Front/Rear 두 profile 모두 성공
  - Front bin SHA256: `00fb66d3df69e133dceeb043fd216b82f922b22ac0d2c017d3f58498a736f008`
  - Rear bin SHA256: `1b1aacef05033e525bfe0df0cc3629c2f1a284db8eb6310eb2bb9c3112aa4044`

두 skip은 개발 PC의 선택적 Node.js/Torch 환경에 따른 항목이다. 같은 테스트에서
필수 ROS, firmware host harness, ArUco 및 DDS 시나리오는 통과했다. 위 검증은
소프트웨어 병합 결과이며 아래 실차 항목을 대체하지 않는다.

## 남은 실차 확인

- 냉각된 robot-1에서 중복 launch 없이 온도와 CPU를 관찰하며 재시험
- 바퀴 공중에서 `A/D/Q/E`, key timeout, Space, control release 확인
- marker 완전 가림과 짧은 dropout 각각에서 0속도 및 3-sample 복구 확인
- 바닥에서는 `W/S` 수 cm부터 시작하고 회전은 마지막에 확인
- 새 main 통합본 배포 후 Front/Rear HELLO, heartbeat, zero probe, servo attach 및
  hardware ready 순서를 다시 확인

이 문서는 코드상 검증과 실제 실차 확인을 구분한다. 위 남은 항목 전에는 강체 쌍
주행을 완료 상태로 표시하지 않는다.
