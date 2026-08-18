---
status: accepted
---

# Fleet Manager가 고정 waiting pose를 소유한다

출차 목적지는 Fleet Manager가 `map` frame의 `waiting_x`, `waiting_y`, `waiting_yaw_deg`로 소유하는 고정 pose다. UI는 차량번호와 주차 비밀번호만 전달하고 Fleet가 Registry에서 `source_slot_id`를 도출한다. UI는 destination pose를 만들지 않으며 Parking Registry도 destination을 중복 저장하지 않는다.

기존 `waiting_x=2.3`, `waiting_y=0.6` 기본값은 하위 호환을 위해 유지하고 `waiting_yaw_deg` parameter를 추가한다. 현장 layout 등록 도구는 waiting polygon의 중심을 Fleet 설정의 `waiting_x/y`에도 기록한다. yaw는 polygon에서 추정하지 않고 현장에서 차량이 최종적으로 바라볼 방향을 명시적으로 입력·저장한다. 등록 설정이 있으면 기본값보다 등록값을 우선한다.

Fleet는 이 pose를 retrieve mission의 pseudo destination으로 만들고 기존 staging 및 직선 insertion 기하를 최대한 재사용한다. waiting pose가 finite한 `map` 좌표인지, 등록 영역 및 map 안에 있는지, 접근 corridor와 loaded footprint가 장애물·map boundary와 충돌하지 않는지 검증한다. UI와 Registry에는 destination 사본을 두지 않는다.

기존 park mission의 Perception은 waiting polygon을 차량 최초 감지 영역으로 계속 사용하므로 Fleet destination 설정 추가가 그 동작을 바꾸지 않아야 한다.

waiting polygon은 차량 중심이 waiting zone에 들어왔는지를 판정하는 allowed vehicle-center detection ROI로 기존 의미를 유지한다. Perception의 center-in-polygon 판정은 변경하지 않고 vehicle 또는 loaded footprint 전체가 polygon 안에 들어가야 한다는 containment 조건도 추가하지 않는다.

Fleet는 `waiting_x/y`가 waiting polygon 내부인지 확인하여 최종 차량 중심과 detection ROI의 일관성만 검증한다. 차량과 Front/Rear를 포함한 loaded footprint의 물리적 안전성은 polygon 크기가 아니라 OccupancyGrid, map boundary, oriented footprint 및 `waiting staging -> waiting pose` insertion corridor로 검증한다. 별도 physical waiting polygon은 이번 범위에 추가하지 않는다.
