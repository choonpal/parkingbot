---
status: accepted
---

# park 목적 슬롯은 경로계획 성공 후 발행 전에 예약한다

park mission의 destination 후보는 Perception이 제공한 빈 슬롯과 Parking Registry의 `EMPTY` 슬롯의 교집합으로 제한한다. Fleet Manager는 기존 `PLAN_PATH` 흐름에서 슬롯 geometry, loaded footprint, source/destination staging, insertion corridor와 A* 경로가 모두 유효한 최종 목적 슬롯을 선택한다.

계획이 완성된 직후 기존 path와 `/parking/slot_pose`를 발행하기 전에, Fleet는 해당 Registry record를 현재 park `mission_id`에 연결해 `EMPTY -> RESERVED`로 원자적으로 전환하고 `active_destination_slot_id`를 보존한다. 예약 전에는 목적 슬롯을 선택하거나 기존 slot-selection 흐름을 앞당기지 않는다.

예약 뒤 path 발행 전에 오류가 발생해 mission path가 시작되지 않았다면, 같은 mission이 만든 `RESERVED`만 `EMPTY`로 되돌릴 수 있다. path가 발행된 뒤 중단되면 차량 위치가 불확실하므로 자동으로 `EMPTY`로 되돌리지 않고 `RESERVED`를 유지한다.

park destination의 `pending_final_vehicle_pose`는 matching park `RETURN` commit까지 active mission에 유지한다. 현재 `mission_id`와 `mission_type=park`가 모두 일치하고 Front/Rear 양쪽 `RELEASE_DONE`으로 만들어진 `RETURN` commit에서만 pose, vehicle spec과 parking direction을 Registry에 저장하고 `RESERVED -> OCCUPIED`로 확정한다. 다른 mission 또는 retrieve commit은 이 record를 변경하지 않는다.
