# 변경 이력: Production 강체 lateral 폐루프 제어

- 작업 일시: 2026-08-27 13:58
- 작업 범위: 실제 차량 Lift 이후 production Front/Rear 강체 동기 제어
- 관련 기능: 상대상태 추정, ArUco/CCTV sensor fusion, lateral PID, visual safety
- 주요 대상 파일: `rigid_body_sync_safe_node.py`, `relative_sync_filter.py`, `rigid_body_kinematics.py`, `sync_params.yaml`, 관련 테스트

## 1. 작업 목적

차량을 Lift한 Front/Rear 로봇이 메카넘 주행 중 longitudinal distance와 relative yaw뿐 아니라 lateral 상대 변위도 폐루프로 복원하도록 production 강체제어를 확장한다. 동시에 ArUco distance/lateral/yaw를 축별로 독립 검증하고, 마커가 사라진 경우와 보이지만 특정 축 correction이 신뢰할 수 없는 경우를 구분한다.

키보드 leader/follower 시험 기능과 legacy `rigid_body_sync_node.py`는 범위에서 제외했다. production `rigid_body_sync` entrypoint가 `rigid_body_sync_safe_node:main`을 사용하는 상태를 확인하고 유지했다.

## 2. 기존 문제

기존 production estimator는 distance와 relative yaw 상태만 가졌다. ID0의 `position.y`는 lateral envelope 검사와 telemetry에만 사용하고 `vy` 제어에는 사용하지 않았다.

- distance/yaw가 정상이어도 메카넘 lateral slip으로 Front/Rear 중심선이 어긋날 수 있었다.
- 결합형 gate 때문에 solvePnP yaw 하나가 reject되면 정상 distance도 폐기됐다.
- correction acceptance timestamp를 visibility 판단에도 사용해, marker가 계속 보여도 yaw reject가 지속되면 `MARKER_LOST`로 오인할 수 있었다.
- wheel/CCTV 상대상태에 lateral predictor와 correction이 없었다.

## 3. 수정 내용

### `ros2/cooperative_parking_robot/cooperative_parking_robot/relative_sync_filter.py`

수정한 클래스/함수:

- `ScalarGateDecision`
- `ScalarObservationGate`
- `visual_safety_state()`

변경 내용:

- distance/lateral/yaw별 scalar innovation gate를 추가했다.
- 각 축이 독립적으로 `ACCEPT`, `REJECT`, `REACQUIRE`를 결정한다.
- consistency 기반 bounded reacquire와 yaw wrap-aware residual을 유지했다.
- marker loss와 축별 correction stale을 분리하는 ROS 독립 safety helper를 추가했다.
- 기존 `DeltaKalman1D`와 `OncePerStamp`는 유지했다.

### `ros2/cooperative_parking_robot/cooperative_parking_robot/rigid_body_kinematics.py`

수정한 클래스/함수:

- `RigidBodyKinematics.relative_pose_in_rear_frame()`
- `RigidBodyKinematics.apply_relative_correction()`

변경 내용:

- Front-Rear map displacement를 Rear yaw 기준 body frame으로 변환한다.
- distance/lateral/yaw correction을 Front/Rear 명령에 대칭 적용한다.
- 기존 `split()` feed-forward와 `limit_twist_pair()` 공통 scale 제한은 유지했다.

### `ros2/cooperative_parking_robot/cooperative_parking_robot/rigid_body_sync_safe_node.py`

수정한 클래스/함수:

- `RigidBodySyncNode`
- `aruco_cb()`
- `_raw_wheel_relative()`, `_relative_predictor()`
- `_initialize_sync_filters()`, `_new_cctv_pair()`
- `_apply_visual_measurement()`, `_consume_visual_measurement()`
- `apply_sync_and_publish()`
- `path_cb()`, `send_stop()`

변경 내용:

- lateral `DeltaKalman1D`와 PID를 추가했다.
- raw wheel predictor 및 fused odom fallback에 lateral을 추가했다.
- ID0/CCTV pair의 세 축을 독립 gate/update한다.
- visual seen time과 각 축 correction time을 분리했다.
- lateral error에 따라 Front/Rear `vy`를 대칭 보정한다.
- path reset과 stop에서 lateral PID도 reset한다.
- 기존 path following, final approach, vehicle control-point 보정은 변경하지 않았다.

### `ros2/cooperative_parking_robot/config/sync_params.yaml`

lateral process/measurement noise, gate/reacquire/consistency, target, deadband, PID와 correction limit 기본값을 추가했다.

### 테스트 파일

- `test_relative_sync_filter.py`: one-frame-one-update, 독립 gate, lateral update, visibility/correction stale, 실제 visual loss, raw predictor 회귀
- `test_motion_control.py`: lateral correction 부호, zero lateral, 좌표계, pair saturation 회귀

## 4. 변경 전 / 변경 후

```text
[변경 전]

raw wheel distance/yaw predict
              +
ArUco distance/yaw combined gate
              ↓
한 축 reject → 관측 전체 reject
              ↓
Front/Rear vx, omega correction

[변경 후]

raw wheel distance/lateral/yaw delta predict
              ↓
distance gate ──→ distance update
lateral gate  ──→ lateral update
yaw gate      ──→ yaw update
              ↓
distance PID → symmetric vx correction
lateral PID  → symmetric vy correction
yaw PID      → symmetric omega correction
              ↓
Front/Rear 전체 command 동일 saturation scale
```

## 5. 주요 알고리즘 및 제어 로직

### 좌표계와 error

lateral은 Rear 중심에서 Front 중심으로 향하는 상대 변위를 `rear_base`에 표현한 Y 성분이다. ROS body convention에 따라 +Y는 Rear 로봇 좌측이다. ID0 tracker가 camera X를 반전해 `position.y`로 발행하는 정의와 동일하다.

```text
lateral_error = fused_relative_lateral - sync_target_lateral_m
```

### Sensor fusion

- raw `/front/wheel_odom`, `/rear/wheel_odom`: 세 축 delta predict의 우선 입력
- fused `/front/odom`, `/rear/odom`: raw wheel이 없거나 stale일 때만 fallback
- ID0: timestamp당 한 번, distance(설정 시)/lateral/yaw 독립 update
- CCTV top-marker pair: paired observation당 한 번, 세 축 독립 fallback update

`DeltaKalman1D`는 raw absolute 값을 덮어쓰지 않고 raw delta만 누적한다. yaw에는 `normalize_angle()`을 유지한다.

### PID와 correction 방향

```text
front_vy -= 0.5 * lateral_correction
rear_vy  += 0.5 * lateral_correction
```

정렬 근방에서 상대 lateral 변화율은 `front_vy - rear_vy`다. 따라서 positive error는 감소 방향 command를 만들고 negative error에서는 반대로 작동한다.

### Feed-forward, saturation, stale

기존 `split()` 강체 회전 feed-forward 이후 상대 correction을 적용한다. `limit_twist_pair()`가 두 로봇 여섯 성분에 동일 scale을 적용한다.

- visual loss: 기존 grace → slowdown → `MARKER_HOLD`
- visual visible, 축 correction reject: `*_CORRECTION_STALE`
- correction 불량 지속: slowdown → `*_CORRECTION_HOLD`
- wheel-only 고속 운행을 무기한 허용하지 않음

초기 filter는 fused odom에서 시작한다. 단일 ArUco frame을 무조건 초기값으로 신뢰하지 않고 innovation gate 또는 consistency reacquire를 거치게 했다.

## 6. 추가/변경된 파라미터

| Parameter | Default | 의미 |
|---|---:|---|
| `sync_target_lateral_m` | 0.0 | 장착 offset을 포함한 목표 lateral |
| `sync_lateral_process_sigma_m_sqrt_s` | 0.003 | lateral 시간 기반 process noise |
| `sync_lateral_process_gain` | 0.03 | lateral motion process noise gain |
| `sync_lateral_measurement_sigma_m` | 0.015 | visual lateral measurement sigma |
| `aruco_lateral_innovation_gate_m` | 0.04 | lateral innovation gate |
| `aruco_reacquire_lateral_m` | 0.10 | lateral reacquire envelope |
| `aruco_consistency_lateral_m` | 0.015 | 연속 관측 consistency 한계 |
| `sync_lateral_deadband_m` | 0.003 | lateral PID deadband |
| `sync_lateral_kp` | 1.2 | PID proportional gain |
| `sync_lateral_ki` | 0.1 | PID integral gain |
| `sync_lateral_kd` | 0.05 | PID derivative gain |
| `sync_lateral_max_correction_mps` | 0.08 | correction 절대 제한 |

기존 parameter 값은 변경하지 않았다. lateral Q/R 기본값은 실측 확정값이 아니다.

## 7. Topic / Interface 변경

ROS topic과 message type 변경 없음.

기존 `_info`/`/sync/error_state` telemetry에 다음 field를 추가했다.

- `raw_relative_lateral`, `fused_relative_lateral`
- `lateral_error`, `lateral_correction`, `lateral_std`
- 세 축 `*_gate_decision`과 `*_correction_age`
- `visual_seen_age`
- 기존 `relative_predictor` 유지

## 8. Safety 영향

- `OncePerStamp`, raw wheel 우선, innovation/reacquire, deadband와 marker-loss hold를 유지했다.
- yaw reject가 정상 distance/lateral update를 막지 않는다.
- visibility와 correction validity를 분리했지만 correction 불량을 허용하지 않고 degraded/hold로 처리한다.
- 실제 visual loss는 계속 `MARKER_HOLD`로 이어진다.
- global emergency stop 조건은 추가하거나 완화하지 않았다.

새 failure mode는 잘못된 target lateral 또는 PID 설정이 횡방향 힘을 만들 수 있다는 점이다. 코드 부호는 unit test로 검증했지만 실제 장착 부호와 offset은 실차 확인이 필요하다.

## 9. 테스트

실행 명령:

```bash
source /opt/ros/humble/setup.bash
export ROS_LOG_DIR=/tmp/parkingbot-ros-logs
PYTHONNOUSERSITE=1 python3 -m pytest -q -k 'not test_package_and_setup_versions_match'
python3 -m compileall -q cooperative_parking_robot launch test
ament_flake8 cooperative_parking_robot/relative_sync_filter.py \
  cooperative_parking_robot/rigid_body_kinematics.py \
  cooperative_parking_robot/rigid_body_sync_safe_node.py \
  test/test_relative_sync_filter.py test/test_motion_control.py
git diff --check
```

결과:

```text
412 passed, 1 deselected
compileall 통과
변경 파일 flake8: No problems found
git diff --check 통과
```

제외한 테스트는 기존 package/setup이 `1.11.1`인데 `1.11.0`을 고정 기대하는 범위 밖 불일치다. 제외 전 전체 실행에서 이 assertion 하나가 실패했다. sandbox의 기본 ROS log 경로가 read-only여서 `ROS_LOG_DIR`를 `/tmp`로 지정했다.

## 10. 실차 검증 필요 항목

- `sync_target_lateral_m` 장착 offset calibration
- ID0 lateral 실제 부호와 rear-base +Y 일치 여부
- lateral PID 및 maximum correction 저속 튜닝
- 정지 로그 기반 lateral Q/R과 gate 산정
- CCTV marker와 로봇 중심 간 offset
- encoder lateral slip 및 predictor drift
- visibility 불안정 시 stale/slowdown/hold 전이
- Lift 무하중 운반 후 실제 차량 하중 거동과 횡하중

## 11. 남은 문제 / 위험요소

- lateral noise와 PID 기본값은 실차 로그로 확정하지 않았다.
- relative yaw가 커지면 두 body-frame `vy` 축이 완전히 평행하지 않다.
- CCTV marker가 로봇 중심에서 벗어나면 lateral bias가 생긴다.
- fused lateral의 별도 즉시 fatal limit은 없고 correction stale은 slowdown/hold로 처리한다.
- 실제 차량과 lift의 lateral compliance 및 허용 횡하중이 정의되지 않았다.
- package/setup 버전과 기존 version test 기대값 불일치가 남아 있다.

## 12. 수정 요약

- production estimator에 lateral Kalman state를 추가했다.
- raw wheel lateral delta predict와 visual lateral update를 추가했다.
- 세 축 gate/reacquire를 독립시켰다.
- lateral PID로 Front/Rear `vy`를 대칭 보정한다.
- 기존 feed-forward와 common-scale saturation을 유지했다.
- visual seen과 축별 correction freshness를 분리했다.
- marker loss와 correction degraded hold reason을 구분했다.
- 관련 회귀 테스트, compile, lint를 수행했다.
- 실차 offset calibration과 PID/Q/R 튜닝은 남아 있다.
