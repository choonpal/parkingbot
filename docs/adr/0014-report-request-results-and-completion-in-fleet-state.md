---
status: accepted
---

# 요청 결과와 완료 기록을 fleet state로 전달한다

`/api/park`와 `/api/retrieve`의 즉시 HTTP 응답은 Web queue 제출만 뜻하므로 `accepted`가 아니라 `submitted=true`와 동일한 `request_id`를 반환한다. 실제 `ACCEPTED` 또는 `REJECTED` 판단 권한은 Fleet Manager만 가진다.

Fleet는 park와 retrieve 공통 구조의 optional `/fleet/state.request_status`를 유지한다. `request_id`, type, optional source slot, status와 안정적인 reason code를 포함한다. 거부 reason은 `INVALID_REQUEST`, `MISSION_ALREADY_ACTIVE`, `SOURCE_SLOT_NOT_FOUND`, `SOURCE_SLOT_NOT_OCCUPIED`, `UNSUPPORTED_PARKING_DIRECTION`, `MISSING_VEHICLE_RECORD`, `APPROACH_CORRIDOR_BLOCKED` 같은 code로 제공하고 UI가 사용자 문구로 변환한다. 요청 처리 뒤 상태를 즉시 발행한다.

`/mission/complete`는 현재 active `mission_id`와 일치하고 앞서 확정한 HOME commit 이후에 발생한 경우에만 유효하다. Fleet는 유효한 completion을 받으면 active reset 전에 mission ID, mission type, optional source slot과 stamp를 `/fleet/state.last_completed`로 snapshot한다. Fleet 프로세스 수명 동안 단조 증가하는 `completion_sequence`를 붙인 뒤 active mission을 reset하고 상태를 즉시 발행한다.

UI는 `last_completed`의 존재만으로 알림하지 않고 마지막으로 처리한 `completion_sequence`보다 큰 이벤트를 한 번만 표시한다. retrieve source slot의 `EXITING -> EMPTY`는 앞선 RETURN commit에서 이미 수행하며, `last_completed`는 그보다 뒤의 양쪽 HOME 복귀까지 끝난 전체 미션 완료를 뜻한다.

기존 Robot FSM은 필요한 `/fleet/state` field만 읽고 optional `request_status`와 `last_completed`를 무시할 수 있어야 하며 park 회귀 테스트로 호환성을 고정한다. 새 topic은 추가하지 않는다.
