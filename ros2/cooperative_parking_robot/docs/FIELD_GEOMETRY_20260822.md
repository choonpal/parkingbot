# 현장 기하 변경: 나란한 HOME + 차량 전용 1.2m 슬롯

## 1. 적용한 실제 배치

- 전체 map: `4.40m × 3.83m`
- 물리 WAITING 테이프: `(0.00,0.00) ~ (1.20,0.80)`
- 안전 차량 중심 ROI: `(0.82,0.32) ~ (1.10,0.48)`
- retrieve 차량 중심: `(0.85,0.40)`, yaw `180°`
- Front HOME: `(3.60,0.60)`, yaw `180°`
- Rear HOME: `(3.60,0.20)`, yaw `180°`
- 주차 슬롯 P1~P4:
  - 중심 x: `1.20, 2.00, 2.80, 3.60m`
  - 중심 y: `3.00m`
  - 크기: `1.20m × 0.80m`
  - 입구선: `y=2.40m`
  - 뒤쪽 선: `y=3.60m`
  - map 끝: `y=3.83m`
  - 실제 뒤쪽 여유: `0.23m`
- 접근 순서: `simultaneous_entry=false`, Front-first

WAITING 테이프 전체를 target ROI로 쓰지 않는다. 차량 중심이 너무 왼쪽 또는
위·아래에 있으면 Front+차량+Rear loaded footprint가 map 경계를 넘는다. 따라서
차량은 물리 테이프 안에서도 안전 중심 ROI에 맞춰 세운 경우에만 latch한다.

## 2. 슬롯 적합성과 충돌검사의 의미 분리

물리 테이프 슬롯은 차량 본체만 검사한다.

```text
vehicle length/width + vehicle margins <= taped slot 1.20×0.80m
```

다음 검사는 계속 Front+차량+Rear 전체 loaded footprint를 사용한다.

- OccupancyGrid A*
- staging 회전원
- 메카넘 횡정렬 corridor
- 슬롯 종방향 삽입 corridor
- 최종 release 자세
- 출차 extraction

즉 1.20m 슬롯을 통과시키기 위해 A* footprint를 차량 크기로 축소하지 않는다.

## 3. 23cm 뒤 여유 검산

기본 placeholder 형상 기준:

```text
loaded footprint length              1.3850m
final collision margin               0.0600m each end
effective loaded length              1.5050m
slot length                           1.2000m
overhang each end                    0.1525m
reserved extra space                 0.0300m
required back clearance              0.1825m
measured back clearance              0.2300m
remaining physical clearance         0.0475m
```

계산상 가능하다. 다만 5cm OccupancyGrid는 `3.83/0.05`를 76셀로 처리하므로
계획상 map 높이는 3.80m가 된다. 최종 자세 검사는 이 더 보수적인 경계에서도
loaded footprint가 들어가는지 다시 검사한다.

실제 차량 중심 offset, 휠베이스 또는 로봇 실측 길이로 required clearance가
0.23m를 넘으면 해당 슬롯은 임무 전에 제외된다.

## 4. 나란한 HOME에서 차량 접근

기존 Front-first barrier를 유지한다.

1. Front가 움직이고 Rear는 HOME에 정지한다.
2. Front가 차량 뒤 staging에 도착한다.
3. Rear가 움직이고 Front는 staging에 정지한다.
4. 기존 PRE_ALIGN → SCAN_IN → 초음파 차축 중심 정렬을 수행한다.

각 이동 로봇은 다음 두 보호 사각형을 피하는 visibility-graph 경로를 만든다.

1. 차량 외곽 + 이동 로봇 반폭 + clearance
2. 정지한 동료 로봇 외곽 + 이동 로봇 반폭 + 최소 로봇 간격

Fleet Manager도 UI 입차 승인과 출차 승인 전에 같은 Front-first 접근 경로를
preflight한다. Front/Rear odom이 없거나 stale이면 승인하지 않는다.

HOME 중심 간격은 `0.40m`이고 기본 로봇 폭 `0.275m` + 최소 간격 `0.10m`보다
`0.025m`만 크다. 실차 배치 오차가 2.5cm를 넘으면 HOME 간격을 넓히고
`parking_layout.yaml`과 field launch wrapper를 함께 수정해야 한다.

## 5. 실제 23cm에 맞춘 슬롯 좌표

초기 템플릿의 슬롯이 `y=1.6~2.8m`이면 map 끝까지 1.03m가 남아 실제 23cm와
맞지 않는다. field branch는 실제 측정값대로 슬롯을 `y=2.4~3.6m`로 옮겼다.

```text
bottom HOME/WAITING zone: y=0.0~0.8m
main aisle:               y=0.8~2.4m
parking slots:            y=2.4~3.6m
measured rear clearance:  y=3.6~3.83m
```

## 6. 슬롯 앞 회전 staging

기본 loaded 회전 반경:

```text
0.5 × hypot(1.385+0.12, 0.470+0.12) = 약 0.8083m
boundary reserve 0.03m 포함 inset     = 약 0.8383m
```

실제 슬롯 중심 y=3.00m일 때 nominal staging y는 약 `1.6075m`라 위·아래
경계는 충분하다. P1~P3는 그대로 사용한다. P4만 오른쪽 map 경계 때문에:

```text
P4 nominal stage   (3.6000, 1.6075)
P4 safe stage      (3.5617, 1.6075)
shift              약 0.0383m
```

으로 이동한다.

staging에서 슬롯 중심으로 대각선 이동하지 않는다. 실제 RigidBodySync와 같은
다음 L자 경로로 검사한다.

```text
safe staging에서 슬롯 yaw로 제자리 회전
→ 메카넘 횡이동으로 슬롯 중심선 정렬
→ 슬롯 종축을 따라 차량 중심까지 삽입
```

## 7. WAITING ZONE의 map 경계 처리

기존 중심 `(0.60,0.40)`에서는 loaded footprint가 x=0 경계를 넘기 때문에 A*
시작점 또는 retrieve 최종 자세가 막힌다. field branch는 retrieve 중심을
`(0.85,0.40)`으로 옮기고 안전 중심 ROI만 target으로 인정한다.

기본 loaded footprint와 5cm boundary reserve 기준:

```text
safe centre x=0.85  → left clearance 약 0.1075m
old centre x=0.60   → left boundary 약 0.1425m 침범
```

입차 승인 시 실제 YOLO target yaw까지 반영한 oriented rectangle로 다시
검사한다. 안전 ROI 안이라도 차량 yaw가 많이 틀어져 loaded footprint가 map
경계를 넘으면 `APPROACH_CORRIDOR_BLOCKED`로 거부된다.

retrieve waiting nominal rotation staging은 `(2.335,0.400)`이며 아래쪽 경계
때문에 약 `(2.335,0.8383)`으로 옮긴다. 횡정렬 후 x축 방향으로 WAITING
중심까지 들어간다.

## 8. release 후 빠져나오기와 HOME 복귀

기존 split exit는 Front를 슬롯 뒤쪽으로 더 밀기 때문에 0.23m 폐쇄단 여유에서
사용할 수 없다. field wrapper는 두 로봇이 wheelbase를 유지한 채 모두 aisle
방향으로 빠져나오도록 설정한다.

```text
same_direction_exit=true
same_direction_exit_sign=-1
exit_distance_m=0.65
```

기본 wheelbase 0.70m에서 두 로봇의 공통 translation은 `-1.35m`다.

```text
Front final s = +0.35 - 1.35 = -1.00m
Rear final s  = -0.35 - 1.35 = -1.70m
```

Front도 차량 외곽과 회전 반경을 벗어난 뒤에만 HOME 방향 회전을 시작한다.

HOME 복귀는 좁은 나란한 공간에서 동시에 들어가지 않는다.

1. 두 로봇이 aisle 방향으로 함께 clear한다.
2. 두 로봇이 같은 side lane으로 이동한다.
3. Rear가 먼저 안전한 내부 위치에서 yaw `180°`로 회전한다.
4. Rear가 `(3.60,0.20)` HOME으로 복귀한다.
5. Front가 Rear의 `RETURNED`를 확인한다.
6. Front가 yaw `180°`로 회전한 뒤 `(3.60,0.60)` HOME으로 복귀한다.

순차 복귀 시간을 반영해 field wrapper의 `return_timeout_s`는 180초다.

## 9. 실행 엔트리

이 브랜치의 기본 실행 파일은 최종 field adapter를 사용한다.

```text
fleet_manager   -> field_runtime_fleet_manager_node
individual_move -> field_runtime_individual_move_node
pose_fusion     -> field_pose_fusion_node
```

롤백용 엔트리:

```text
fleet_manager_field_policy
fleet_manager_legacy
individual_move_field_policy
individual_move_legacy
pose_fusion_legacy
```

## 10. 빌드 및 실행

```bash
git fetch origin
git switch feature/side-by-side-home-vehicle-only-slots
git pull --ff-only
cd ~/parkingbot/ros2
colcon build --symlink-install --packages-select cooperative_parking_robot
source install/setup.bash
```

Jetson:

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

## 11. runtime layout 반영

패키지 템플릿은 오동작 방지를 위해 `layout_registered:false`다. 현장 실행 전
branch의 좌표를 runtime 파일에 반영하고 세 위치의 값을 모두 true로 바꾼다.
기존 runtime layout은 먼저 백업한다.

```bash
cp ~/.ros/adaptive_valet_bot/parking_layout.yaml \
   ~/.ros/adaptive_valet_bot/parking_layout.backup_$(date +%Y%m%d_%H%M%S).yaml

cp ~/parkingbot/ros2/cooperative_parking_robot/config/parking_layout.yaml \
   ~/.ros/adaptive_valet_bot/parking_layout.yaml

sed -i 's/layout_registered: false/layout_registered: true/g' \
   ~/.ros/adaptive_valet_bot/parking_layout.yaml
```

Homography `.npy` 두 개는 이 작업으로 변경되지 않는다.

## 12. 실차 적용 전 순서

1. 모터 OFF 상태에서 `/front/odom`, `/rear/odom`이 실제 HOME과 맞는지 확인
2. 차량 없이 Front 단독 HOME→staging 경로 확인
3. 차량 없이 Rear 단독 경로 확인
4. 두 로봇 Front-first 접근 및 HOME 간격 확인
5. 차량 없이 P1 staging 회전·횡정렬·삽입 확인
6. P4 오른쪽 경계 보정 확인
7. release 없이 shared aisle exit 확인
8. Rear-first HOME 복귀와 Front 대기 확인
9. 마지막에 저하중 차량으로 P1부터 시험

실차 차량/로봇 치수는 placeholder 대신 반드시 측정값으로 넣는다. 4.75cm
후방 계산 여유와 2.5cm HOME 간격 여유는 크지 않으므로 테이프 좌표만으로
안전하다고 간주하면 안 된다.
