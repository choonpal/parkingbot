# P1 안전·정합성 수정 기록 (2026-07-24)

이 문서는 원본 `v8_fix_work.zip`을 보존한 채 수정 작업본에 반영한 사항을 요약한다.

## 반영 결과

- **P1-2 — 목표 차량 휠베이스 연결**
  - 로봇의 제어 기준점은 담당 축의 파지 중심으로 본다.
  - 초음파 센서-그리퍼 장착 오프셋은 바퀴 중심 검출 단계에서 이미 보정한다.
  - 따라서 Front-Rear 목표 간격은 `target_vehicle_wheelbase`만 사용한다.
  - `/parking/vehicle_spec`의 휠베이스를 접근·정렬 노드와 강체 동기 제어기에 연결했다.

- **P1-5 — 메카넘 주행 적합성 확인**
  - 기존 추종기는 경로 오차를 로봇 좌표계의 `vx`, `vy`로 변환하고 yaw를 별도로 제어한다.
  - 전진만 가정하는 자동차형 추종기가 아니라 메카넘 횡이동·후진을 사용하는 홀로노믹 구현임을 확인했다.
  - 별도 수정 대상에서 제외했다.

- **P1-6 — mission 단위 상태 barrier**
  - `LIFT`, `DRIVE`, `RELEASE`, `RETURN`에 mission ID별 `READY → COMMIT` 절차를 적용했다.
  - Front가 commit coordinator 역할을 수행한다.
  - 다른 mission, 중복·역순 이벤트, timeout을 거부한다.

- **P1-7 — timestamp·stale 방어**
  - 속도 명령을 `TwistStamped`로 변경했다.
  - 경로, 차량·슬롯·제원, odometry, ArUco, CCTV 입력에 timestamp/sequence 검사를 적용했다.
  - zero, stale, future, duplicate, out-of-order 데이터를 거부한다.
  - 주행 중 odometry와 fleet 상태 timeout 시 정지한다.
  - 실제 Jetson과 Raspberry Pi에는 NTP/Chrony 설정이 별도로 필요하다.

- **P1-4 — 결합 직사각형 footprint A***
  - 로봇 1대의 실측 방향별 크기 `0.565m(차량 앞뒤) × 0.275m(차량 좌우)`를 설정했다.
  - 그리퍼·로봇 중심이 각 차량 축 중심과 일치한다는 전제로 `max(차량 길이, 휠베이스+로봇 길이) × max(차량 폭, 로봇 폭)`에 양쪽 안전여유를 더한다.
  - 고정 4칸 원형 팽창을 mission별 축 정렬 직사각형 팽창으로 교체했다.
  - 미확인 셀, footprint가 넘어가는 맵 경계, 격자 대각선 corner cutting을 차단한다.
  - A* 시작점은 고정 대기좌표가 아니라 Front/Rear 최신 odometry 중점이다.
  - 차량 길이·폭과 실제 휠베이스는 아직 실측값 입력이 필요하다.

- **P1-10 — 차량 아래 이탈 순서**
  - 기존 코드의 `EXIT_UNDERBODY → EXIT_TO_SIDE → RETURN_HOME` 구현을 확인했다.
  - 이 순서가 유지되는지 검사하는 회귀 테스트를 추가했다.

- **2026-07-25 마커 결정으로 대체됨**
  - Front 상판 ID10과 Rear 상판 ID11은 천장 절대 pose에 사용한다.
  - Front 후면 ID0은 Rear 전면 카메라가 보는 상대 pose에 사용한다.
  - 최신 진입·fallback 기준은 `FRONT_FIRST_ENTRY_ID0_ULTRASONIC_20260725.md`를 따른다.

## P2로 유지한 항목

- 실제 파지 확인 센서는 기구 동작시험 후 필요 여부를 결정한다.
- 전원 상실 시 차량 유지 문제는 안전한 받침대 위의 실물시험 결과와 최종 안전 주장 범위에 따라 보강한다.

## 검증

- Python compile check 통과
- 최신 통합본 `pytest`: 89개 전체 통과(2026-07-25)
- 하드웨어 연결, ROS 2 다중 장비 통신, NTP/Chrony, 실차·모형차 운반 시험은 별도로 수행해야 한다.
