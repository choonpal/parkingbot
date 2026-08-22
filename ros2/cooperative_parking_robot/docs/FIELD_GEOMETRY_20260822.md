# 현장 기하 변경: 나란한 HOME + 차량 전용 1.2 m 슬롯

## 적용 조건

- 전체 맵: 4.40 m × 3.83 m
- 물리 주차 슬롯: 차량 전용 길이 약 1.20 m
- 슬롯 뒤 자유공간: 0.23 m
- Front/Rear HOME: 차량 후방에서 나란히 배치
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

## 나란한 HOME

`individual_move` 실행 엔트리를 field adapter로 변경했다. 각 로봇은 기존
Front-first barrier를 유지하되, vehicle frame에서 다음 두 장애물을 모두
피하는 visibility-graph 경로를 만든다.

1. 차량 + 이동 로봇 clearance 보호 사각형
2. 현재 정지해 있는 동료 로봇 보호 사각형

따라서 HOME을 억지로 종방향 일렬로 배치할 필요가 없다.

## 롤백

기존 실행 파일도 별도 이름으로 남겼다.

```text
fleet_manager_legacy
individual_move_legacy
```

문제가 생기면 `setup.py`의 기본 entry point만 legacy 쪽으로 되돌릴 수 있다.

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

## 실차 적용 전 확인

- 실제 차량 길이·폭과 두 로봇 길이는 placeholder가 아닌 실측값을 넣는다.
- 슬롯 뒤 0.23 m는 테이프 끝부터 벽 또는 고정 장애물까지 실제 자유거리로 잰다.
- `layout_registered: true`인 현장 등록 YAML을 런타임에 사용한다.
- Front/Rear의 실제 HOME odom이 config의 side-by-side 위치와 맞는지 확인한다.
- 첫 시험은 차량 없이 Front 단독 접근 → Rear 단독 접근 → 두 로봇 순차 접근 순으로 한다.
