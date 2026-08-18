---
status: accepted
---

# 슬롯 lifecycle을 mission-scoped commit에 연결한다

park mission은 기존 `PLAN_PATH`에서 경로계획이 성공한 뒤 path와 `/parking/slot_pose`를 발행하기 전에 목적 슬롯을 `EMPTY -> RESERVED`로 전환한다. 예약 뒤 path 발행 전에 실패한 경우에만 같은 mission의 예약을 `EMPTY`로 되돌릴 수 있으며, path 발행 뒤에는 자동 해제하지 않는다.

출차 요청을 승인하면 선택한 슬롯을 `OCCUPIED -> EXIT_RESERVED`로 전환한다. 현재 retrieve mission의 양쪽 로봇이 인양을 마친 뒤 발행된 `DRIVE` commit과 `mission_id`가 일치할 때 `EXIT_RESERVED -> EXITING`으로 전환한다.

`EXITING -> EMPTY`는 다음 조건이 모두 성립하는 `RETURN` commit에서만 수행한다.

- commit의 `mission_id`가 현재 active mission과 일치한다.
- active `mission_type`이 `retrieve`다.
- active mission의 source가 해당 Registry 슬롯이다.
- active mission의 destination이 구성된 출차 waiting pose다.
- commit이 같은 mission ID에 속한 Front/Rear 양쪽 `RELEASE_DONE` ready로 생성되었다.

park mission의 matching `RETURN` commit은 기존 입차 의미를 유지하며, `pending_final_vehicle_pose`를 해당 `RESERVED` 슬롯에 저장하고 `OCCUPIED`로 확정한다. `HOME` commit은 슬롯 lifecycle을 변경하지 않고 양쪽 로봇 복귀 완료와 전체 미션 완료만 확정한다.

`DRIVE` commit 전 장애에서는 슬롯을 자동으로 비우지 않고 `EXIT_RESERVED`로 유지하며, 차량이 슬롯에 정상적으로 복구된 것이 확인된 뒤에만 `OCCUPIED`로 되돌린다. `DRIVE` commit 이후 장애에서는 차량 위치가 불확실하므로 `EXITING`을 유지하고 자동으로 `EMPTY` 처리하지 않는다.
