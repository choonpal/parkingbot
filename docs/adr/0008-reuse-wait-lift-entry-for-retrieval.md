---
status: accepted
---

# 출차도 기존 WAIT_LIFT 접근 entry point를 사용한다

Fleet Manager가 출차 요청을 승인하면 먼저 새 `mission_id`와 `mission_type=retrieve`를 확정하고, Parking Registry에서 선택한 source slot의 최종 차량 자세와 vehicle spec을 읽는다. Fleet는 fresh한 timestamp와 `map` frame으로 기존 `/parking/target_pose`와 `/parking/vehicle_spec`을 발행한 뒤 상태를 `WAIT_LIFT`로 변경한다. target freshness window 안에 양쪽 Robot FSM이 수신하도록 timer를 기다리지 않고 변경 직후 `/fleet/state`를 즉시 한 번 발행한다.

양쪽 Robot FSM은 기존과 동일하게 `IDLE -> APPROACH -> ALIGN -> LIFT`를 수행한다. `IndividualMoveNode`의 후방 접근 및 차축 탐색 알고리즘에는 park/retrieve 분기를 추가하지 않는다. `mission_type=retrieve`는 이후 Fleet의 extraction, 경로 계획, waiting destination과 Parking Registry lifecycle 처리에만 사용한다. retrieve 전용 Robot FSM state와 새 topic은 만들지 않는다.

현재 `/parking/target_pose`와 `/parking/vehicle_spec`은 Perception도 발행한다. 이번 실증에서는 출차 중 waiting zone에 새 입차 차량을 두지 않아 Fleet의 retrieve target과 Perception의 waiting target이 동시에 활성화되지 않는다고 운영 조건으로 보장한다. Fleet는 추가로 fresh한 `/parking/target_ready=true`가 있으면 retrieve 시작을 거부한다.

`target_ready=false`는 waiting zone이 물리적으로 비었다는 증명이 아니라 정차 차량 target이 latch되지 않았다는 뜻일 뿐이다. 따라서 이 검사는 보조 충돌 방지이며, 이번 실증의 waiting zone 비점유 운영 조건을 대체하지 않는다. 실제 운영 환경의 명시적 waiting occupancy 관측은 후속 확장으로 남긴다.
