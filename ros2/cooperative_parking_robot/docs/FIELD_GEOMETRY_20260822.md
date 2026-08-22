# 현장 기하 변경: 나란한 HOME + 차량 전용 1.2 m 슬롯

## 적용 조건

- 전체 맵: 4.40 m × 3.83 m
- 물리 주차 슬롯: 차량 전용 길이 1.20 m, 폭 0.80 m
- 슬롯 뒤 자유공간: 0.23 m
- Front/Rear HOME: 차량 후방에서 나란히 배치
  - Front 중심: `(3.60, 0.60)`, yaw `180°`
  - Rear 중심: `(3.60, 0.20)`, yaw `180°`
- 접근 순서: 기존과 동일한 Front-first
- 운반 및 회전 충돌검사: 기존 loaded footprint 유지

## 핵심 분리

물리 테이프 슬롯 적합성은 차량 본체만 검사한다.

```text
vehicle length/width + vehicle margins <= taped slot
```

다음 검사는 계속 Front + 차량 + Rear 결합체 전체를 사용한다.

- OccupancyGrid A*
- staging 회전공간
- 슬롯 삽입 corridor
- 최종 release 자세
- 출차 extraction

따라서 1.20 m 슬롯을 통과시키기 위해 A* footprint를 차량 크기로 줄이지
않는다. 슬롯 의미만 차량 전용으로 분리한다.

## 기본 23 cm 검산

기존 기본치:

```text
loaded footprint length              1.385 m
additional final collision margin    0.06 m each end
effective loaded length              1.505 m
slot length                           1.200 m
overhang each end                    0.1525 m
reserved extra space                 0.0300 m
required back clearance              0.1825 m
measured back clearance              0.2300 m
remaining clearance                  0.0475 m
```

따라서 기본 형상에서는 계산상 통과한다. 실제 차량 중심 offset 또는 실측
로봇 길이가 커지면 임무마다 다시 계산하며, 23 cm를 넘으면 슬롯 후보를
거부한다.

## 나란한 HOME 접근

`individual_move` 실행 엔트리를 field adapter로 변경했다. 기존 Front-first
barrier는 그대로 유지한다.

1. Front가 움직이고 Rear는 HOME에 정지한다.
2. Front가 차량 뒤 staging에 도착한다.
3. Rear가 움직이고 Front는 staging에 정지한다.
4. 이후 기존 PRE_ALIGN/SCAN_IN/초음파 중심정렬을 그대로 사용한다.

각 이동 로봇은 vehicle frame에서 다음 두 보호 사각형을 모두 피하는
visibility-graph 경로를 만든다.

1. 차량 + 이동 로봇 clearance 보호 사각형
2. 현재 정지해 있는 동료 로봇 보호 사각형

Fleet Manager도 입차 승인 전과 출차 승인 전에 같은 순서로 경로를
preflight한다. 지도 경계, 다른 주차 차량, unknown 셀까지 통과해야 승인된다.

## 실행 엔트리

이 브랜치의 기본 console script는 field adapter를 사용한다.

```text
fleet_manager  -> field_fleet_manager_node
individual_move -> field_individual_move_node
pose_fusion -> field_pose_fusion_node
```

기존 구현도 롤백용으로 남겼다.

```text
fleet_manager_legacy
individual_move_legacy
pose_fusion_legacy
```

## 현장 실행

브랜치 checkout 및 빌드:

```bash
git switch feature/side-by-side-home-vehicle-only-slots
cd ~/parkingbot/ros2
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
```

Jetson은 기존 듀얼 CCTV launch를 사용한다. `fleet_manager` 엔트리가 field
adapter로 연결되어 있으므로 별도 Jetson launch 복사는 필요 없다.

```bash
ros2 launch cooperative_parking_robot cctv_server_dual.launch.py \
  enable_opencv_camera:=true \
  layout_config:=$HOME/.ros/adaptive_valet_bot/parking_layout.yaml \
  simultaneous_entry:=false
```

Front Raspberry Pi:

```bash
ros2 launch cooperative_parking_robot front_robot_field.launch.py
```

Rear Raspberry Pi:

```bash
ros2 launch cooperative_parking_robot rear_robot_field.launch.py
```

Field wrapper는 RETURN 목적지와 PoseFusion 초기 중심을 실제 HOME 좌표로
설정한다. field PoseFusion은 첫 CCTV fix 전 명목 yaw를 180°로 시작하고,
첫 유효 상판 마커 관측이 들어오면 그 관측을 권위값으로 사용한다.

## 새 파라미터

Fleet Manager:

```text
vehicle_slot_longitudinal_margin_m = 0.05
vehicle_slot_lateral_margin_m      = 0.05
slot_back_clearance_m              = 0.23
slot_back_clearance_reserve_m      = 0.03
approach_robot_clearance_m         = 0.06
approach_corner_margin_m           = 0.03
```

Individual Move:

```text
approach_corner_margin_m           = 0.03
require_peer_odom_for_approach     = true
```

Pose Fusion:

```text
field_initial_yaw_deg              = 180.0
```

## 실차 적용 전 확인

- 실제 차량 길이·폭과 두 로봇 길이는 placeholder가 아닌 실측값을 넣는다.
- 슬롯 뒤 0.23 m는 테이프 끝부터 벽 또는 고정 장애물까지 실제 자유거리로 잰다.
- `layout_registered: true`인 현장 등록 YAML을 런타임에 사용한다.
- Front/Rear의 실제 HOME 중심이 `(3.60,0.60)`, `(3.60,0.20)`와 맞는지 확인한다.
- HOME 중심 간격은 0.40 m이고 기본 로봇 폭/최소 간격 조건에 대한 추가
  위치 여유가 약 2.5 cm뿐이다. 실제 배치 오차가 크면 HOME 중심 간격을 더
  넓히거나 실측 좌표를 wrapper와 layout에 같이 반영한다.
- 첫 시험은 차량 없이 Front 단독 접근 → Rear 단독 접근 → 두 로봇 순차 접근
  → 차량 인양 없는 슬롯 삽입 순으로 한다.
- `simultaneous_entry`는 이 현장 branch에서 사용하지 않는다.
