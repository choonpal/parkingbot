# 실차 적용 준비도 — 1.11.0

이 문서는 현재 운용 범위와 GO/NO-GO만 정의한다. 배포·launch·UI·복구는
[실차 Runbook](REAL_ROBOT_DEPLOYMENT_RUNBOOK.md), calibration·Homography·preflight는
[pipeline](pipeline.md)을 따른다. 다른 현재 문서는 [문서 안내](README.md)에서
찾는다.

## 현재 판정

소프트웨어에는 park와 인증 기반 retrieve, Front/Rear 협동 FSM, CCTV/ArUco
localization, 초음파 정렬, 결합 footprint A*, UI 승인과 SQLite Registry가
구현돼 있다. 이는 실차 하중 검증 완료를 의미하지 않는다.

- 정적 인지와 바퀴 공중시험: 현장 calibration과 실측값 적용 후 진행 가능
- 빈 차체 저속 단독/협동 시험: 앞 단계 통과 후 진행 가능
- 보호 지그 저하중 park/retrieve: 모든 interlock과 단계 gate 통과 후 조건부
- 사람 없는 무인 차량 인양·운반: **현재 NO-GO**

2026-08-29 통합 후보는 소프트웨어 감사와 clean build를 통과했다. Front와 Rear를
각각 격리 배포해 수동 왕복 bag으로 좌·우 `sensor_to_gripper_x=0.0m`, 총 네 값을
확인했다. 세 장비 동일 SHA 배포와 단계별 실기 gate는 아직 미완료다. 따라서 차량
하부 자동 진입은 계속 **NO-GO**다.
상세 기준은 [현재 통합 상태](CURRENT_INTEGRATION_STATUS.md)에 있다.

2026-08-30 `ddad9ac` 기준 strict 복구에서는 TensorRT 두 engine의 순차 로딩과
양쪽 bridge-only heartbeat는 통과했다. 그러나 전체 production cold-start 중
Front/Rear heartbeat ACK timeout, UART write timeout과 SSH 불안정이 재현됐다.
mission target snapshot 뒤 YOLO unload 기능도 실제 mission 상태까지 도달하지
못해 실차 미검증이다. 전체 stack 반복 기동을 중단하고 RPi 한 대씩 cold-start
부하를 격리하기 전까지 단계 gate 2를 통과한 것으로 간주하지 않는다.

현재 로봇은 메인 전원이 RPi/카메라와 모터 계통에 함께 인가되며 motor rail만
독립적으로 켜고 끌 수 없다. 그러므로 정적 인지/UART 시험도 바닥에서 수행하지
않고, 모든 바퀴를 견고하게 띄운 상태에서 격리 ROS domain과 perception-only
노드만 사용한다.

특정 test 개수를 합격 근거로 쓰지 않는다. 목표 Humble 환경에서 다음 명령이
실제로 성공하고 결과가 검토돼야 하는 software gate다.

```bash
~/parkingbot/ros2/cooperative_parking_robot/scripts/humble_build_check.sh \
  ~/parkingbot_ws
```

이 gate는 카메라, UART, DDS, STM32, 모터, 전기와 하중 시험을 대체하지 않는다.

## 절대 NO-GO 조건

다음 중 하나라도 해당하면 모터를 켠 접근 또는 차량 인양을 시작하지 않는다.

- Homography/layout이 없거나 현장 rectified 영상과 map frame에 맞지 않음
- 두 CCTV 중 하나가 stale이거나 실제 frame 크기가 Homography 기준 해상도와 다름
- CCTV live coverage 밖 `/parking/map` cell이 `unknown(-1)`으로 유지되지 않음
- `layout_registered=true`가 아니거나 slot·corridor·waiting·no-go가 미검증
- camera/marker 역할, wheel/encoder/kinematic 값 또는 motor sign이 미확정
- ROS와 각 STM32의 wheel radius/PPR/`lx`/`ly`가 일치하지 않음
- `enable_serial`, `require_serial`, `require_hardware_ready`,
  `require_ultrasonic_for_ready`가 실차 안전값이 아님
- `use_aruco_distance=true`인데 중심간 거리와 offset을 실측하지 않음
- 좌우 ultrasonic-to-gripper X offset이 미측정이거나, 측정 근거 없이 임시
  `0.0`으로 둔 값임
- 물리 ESTOP, fuse, 공통 GND 또는 HC-SR04 5 V level protection이 없음
- 단일 메인 전원 상태에서 바퀴를 띄우지 않거나, 격리 ROS domain·구동 노드
  미실행·`cmd_vel` publisher 0개를 확인하지 않고 정적 통전 시험을 시작함
- 실제 파지/하중 확인 수단이 없음. `GRIP_DONE`만으로는 통과할 수 없음
- UART, command, sensor, pose 또는 camera 단절 시 정지가 미검증
- 단계시험 실패가 남아 있거나 보호 지그와 사람 감독이 없음
- SQLite가 불일치 또는 미션 중간 상태에서 fail-closed하지 않음

## 단계 gate

한 단계가 실패하면 다음 단계는 NO-GO다. 초기 바닥 명령은 현재 펌웨어에서
바퀴가 덜컥거리지 않고 확인된 12 rpm 상당의 `0.0628 m/s`를 쓰되, 10 cm·4초
전용 시험 한도 밖으로 늘리지 않는다. 단순히 더 낮은 명령을 안전하다고 간주하지
않고 모터가 실제로 부드럽게 회전하는 구간인지 바퀴 공중시험에서 먼저 확인한다.

1. **정적 전기** — 단일 메인 전원 제약을 확인하고 모든 바퀴를 견고하게 띄운 뒤,
   물리 ESTOP 차단, HC-SR04 3.3 V 보호, fuse와 공통 GND가 확인됨
2. **Perception/UART only, common power ON** — 격리 ROS domain에서 motion node와
   `cmd_vel` publisher가 0개이며 heartbeat/ACK와 sensor frame이 안정적이고,
   command 무갱신 250 ms 및 heartbeat 단절 300 ms의 PWM 0이 확인됨
3. **바퀴별 jack-up** — 네 motor command sign과 encoder sign, ESTOP PWM 0 확인
4. **로봇별 빈 차체 저속** — 전후·횡·회전, odometry 축척, sensor/pose timeout 확인
5. **두 로봇 빈손** — 먼저 전용 10 cm 협동 직진 시험으로 상대 yaw, 간격 유지,
   marker/odom 손실 정지를 확인한 뒤 Front-first 접근으로 진행
6. **초음파/servo 무부하** — 차축 중심 반복성, offset/sign, 간섭과 ESTOP hold 확인
7. **보호 지그 저하중** — 실제 파지/하중 sensor, 미끄럼·낙하 방지와 전원 차단 확인
8. **전체 cycle** — park→양쪽 HOME→Fleet restart→인증 retrieve→양쪽 HOME,
   Registry `OCCUPIED/EMPTY`와 mission reset 확인

## 실차 GO 체크리스트

- [ ] Production marker가 Front **ID2**, Rear **ID1**, 상대 **ID0**이며 실험용
      ID2/ID3이 production asset에 없음
- [ ] 카메라마다 현장 `/dev/v4l/by-path/...` 역할이 기록·검증됨
- [ ] CCTV/Rear intrinsic이 실제 해상도에서 검증됨
- [ ] `require_all_cameras=true`, `require_exact_camera_resolution=true`이며 camera
      단절/해상도 불일치가 target·empty·map을 fail-closed함
- [ ] Rectified Homography RMS < 0.02 m, 동일 map frame과 overlap 검증 완료
- [ ] `layout_registered=true`, slot/corridor/waiting/no-go와 coverage 확인
- [ ] vehicle mask 외곽·yaw와 실제 vehicle/robot footprint 반복성 확인
- [ ] Front/Rear marker offset, ID0 yaw와 선택적 distance offset 실측
- [ ] 로봇별 wheel radius, encoder PPR, `lx`, `ly`, PID와 sign 확정
- [ ] 펌웨어의 현재 명목 PPR `5182.0f`를 그대로 믿지 않고 로봇별 측정값으로
      ROS/firmware를 일치시킴
- [ ] 좌우 ultrasonic-to-gripper offset, lateral sign, threshold와 freshness 확인
- [ ] 현재 ID0 장착에서 실측한 `aruco_distance_offset_m=0.570`을 재확인한 뒤에만
      `use_aruco_distance=true`; 카메라/마커 장착 변경 또는 미확인 시 `false`
- [ ] 단일 메인 전원 제약, 물리 ESTOP, fuse, 공통 GND, level shift, 전 바퀴
      공중 고정과 보호 지그 확인
- [ ] software와 단계 gate 1~8 결과를 현장 기록으로 검토
- [ ] `GRIP_DONE`과 독립된 실제 파지/하중 확인 수단 확보

마지막 항목까지 모두 충족되지 않으면 사람 없는 무인 인양은 NO-GO다.

## 구조적 운용 한계

- retrieve는 구현돼 있지만 이 시스템이 **forward로 주차한 차량**만 지원한다.
- 저장된 final pose를 사용하므로 주차 뒤 사람이 차량을 움직이지 않아야 한다.
- source 접근이 막히면 거부한다. 우회 planner와 주행 중 동적 재계획은 없다.
- 장애물은 layout no-go에 등록해야 하며 운반 중 기능 안전 obstacle stop은 없다.
- SQLite는 안정 `EMPTY/OCCUPIED`만 자동 복원한다. transient mission 자동 복구나
  perception reconciliation은 없다.
- UI transport는 암호화되지 않으며 별도 사용자 인증이 없으므로 trusted LAN
  전용이다. 비밀번호 원문을 DB나 로그에 저장해서는 안 된다.
- 소프트웨어 ESTOP은 인증된 기능 안전 장치가 아니며 물리 ESTOP을 대체하지 않는다.
