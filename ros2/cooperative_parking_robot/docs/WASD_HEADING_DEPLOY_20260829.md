# W/A/S/D 자세 유지 변경 및 임시 배포 기록

기록 시각: 2026-08-29 00:38 KST

## 현재 결론

- W/A/S/D 보정 코드는 로컬 작업 트리에 반영했다.
- 로컬 전체 테스트는 `752 passed, 1 skipped`로 통과했다.
- 변경본은 robot-1의 새 격리 경로에 빌드하여 실행 중이다.
- robot-2 Front와 robot-1 Rear/5007 스택은 모두 실행 중이다.
- 아직 Arm하거나 바퀴를 움직이는 실차 시험은 하지 않았다.
- 다른 Codex 세션이 작업 중이므로 이 변경은 아직 commit/push하지 않았다.

## 코드 변경 내용

변경 파일:

- `cooperative_parking_robot/rigid_pair_teleop_core.py`
- `cooperative_parking_robot/keyboard_follow_node.py`
- `launch/cooperative_drive_test_rear.launch.py`
- `test/test_rigid_pair_teleop.py`

핵심 동작:

1. 간격 오차가 데드밴드를 넘는 순간 `0.010 m/s`를 강제로 넣던 최소 보정을 제거했다.
2. 간격 보정은 `4 mm` 데드밴드를 넘은 오차만큼 연속적으로 증가한다.
3. Arm 시 현재 앞뒤 간격은 유지하되 lateral과 상대 yaw의 목표는 `0`으로 둔다.
4. W/A/S/D 중에는 두 wheel odom yaw의 평균 진행각을 Arm 시점 값으로 유지한다.
5. 상대 yaw 보정은 양쪽에 반대로, 전체 진행각 보정은 양쪽에 동일하게 적용한다.
6. Q/E 회전 궤적은 이번 변경에서 수정하지 않았다.
7. 초기 Arm 허용값을 lateral `2 cm`, 상대 yaw `3°`로 강화했다.
8. 5007 UI에 `전체 진행각 오차`를 추가했다.

기본 보정값:

| 항목 | 값 |
| --- | ---: |
| gap deadband | `0.004 m` |
| gap kp | `1.8` |
| gap correction limit | `0.025 m/s` |
| relative yaw deadband | `0.8°` |
| relative yaw kp | `1.5` |
| pair heading deadband | `0.5°` |
| pair heading kp | `1.2` |
| pair heading correction limit | `0.04 rad/s` |

## 소프트웨어 검증

실행 결과:

```text
python3 -m pytest -q
752 passed, 1 skipped in 112.53s
```

추가 확인:

- 변경 Python/launch 파일 `py_compile` 통과
- `git diff --check` 통과
- W/A/S/D에만 진행각 보정이 적용되고 Q/E 명령은 바뀌지 않는 테스트 포함
- 간격 보정이 데드밴드 경계에서 불연속적으로 튀지 않는 테스트 포함

## 배포 상태

기존 실행본은 보존했다.

```text
/home/robot/parkingbot_test_434be2a
```

robot-1 새 격리 배포본:

```text
/home/robot/parkingbot_test_wasd_heading_20260829
```

새 경로에서 다음 빌드가 성공했다.

```text
Summary: 1 package finished
```

실행 중인 tmux 세션 이름은 양쪽 모두 다음과 같다.

```text
rigid-pair-wasd-20260829
```

- robot-2: 기존 `parkingbot_test_434be2a` 설치본으로 Front STM32 bridge 실행
- robot-1: 새 `parkingbot_test_wasd_heading_20260829` 설치본으로 Rear 카메라, ID0 tracker, STM32 bridge, rigid-pair controller 실행
- ROS domain: `142`
- 5005 응답: HTTP `200`
- 5007 응답: HTTP `200`
- robot-1 온도: 마지막 확인 `66.2°C`
- throttling: `0x0`

실행 노드에서 확인된 적용값:

```text
gap_deadband_m = 0.004
yaw_deadband_deg = 0.8
heading_kp = 1.2
initial_lateral_limit_m = 0.02
```

## 마지막 5007 상태

```text
state: IDLE
Front command: 0 / 0 / 0
Rear command: 0 / 0 / 0
blocker: ID0 ArUco pose가 보이지 않거나 오래됨
```

양쪽 hardware/odom 때문에 생긴 blocker는 없었고, 현재 남은 것은 ID0 관측뿐이다.
따라서 로봇 배치 또는 영상/마커 관측을 확인하기 전에는 Arm하지 않는다.

## 아직 확인하지 못한 항목

마지막 카메라 sensor-data QoS 확인 명령은 세션 전환 때문에 중단됐다. 다음 항목은
아직 완료로 간주하지 않는다.

- `/rear/marker_camera/image` 실제 프레임 수신 여부
- `/rear/marker_camera/preview` 실제 프레임 수신 여부
- ID0 안정 검출 및 3프레임 안정화
- `initial_yaw_limit_deg=3.0` 실행 파라미터 직접 조회
- 바퀴를 띄운 W/S/A/D 방향 및 보정 확인
- 바닥에서 짧은 W/S/A/D 주행 확인

## 다음 재개 순서

1. `http://robot-1.local:5005/`에서 영상과 ID0를 확인한다.
2. `http://robot-1.local:5007/`에서 lateral `2 cm` 이내, yaw `3°` 이내로 배치한다.
3. 바퀴를 띄운 상태에서 Arm한다.
4. W, S, A, D를 각각 짧게 누르고 매번 Space로 멈춘다.
5. UI에서 `간격 오차`, `각도 오차`, `전체 진행각 오차`, Front/Rear 명령을 기록한다.
6. 정상일 때만 바닥에서 짧게 반복한다.

실행 로그 확인:

```bash
ssh robot@robot-1.local
tmux attach -t rigid-pair-wasd-20260829
```

시험을 종료할 때는 양쪽에서 다음 세션에 `Ctrl-C`를 보낸다.

```bash
tmux send-keys -t rigid-pair-wasd-20260829 C-c
```

