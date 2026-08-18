---
status: accepted
---

# HOME commit 이후에 미션을 완료한다

현재 `RETURN` commit은 Front와 Rear가 모두 차량 `RELEASE_DONE`을 보고하여 차량을 내려놓은 시점을 뜻한다. 이 시점에 입차 슬롯은 `OCCUPIED`, 출차 슬롯은 `EMPTY`로 확정할 수 있지만 로봇은 아직 복귀 중일 수 있다.

기존 `/mission/{role}/ready`와 `/mission/commit` 프로토콜에 `HOME` stage를 추가한다. 각 로봇은 자기 `return_done` 이후 현재 `mission_id`로 `HOME` ready를 발행하고 `RETURN` 상태에서 대기한다. Front는 양쪽의 fresh한 `HOME` ready가 같은 미션에 속할 때만 `HOME` commit을 발행한다. 두 로봇은 이 commit 이후에 reset하고 `IDLE`로 전이하며, Front만 그 이후 기존 `/mission/complete`를 발행한다.

현재 coordination stage validator의 허용 목록에도 `HOME`을 추가하고 `RETURN` FSM이 `HOME` ready/commit을 실제로 발행·소비하도록 함께 변경한다. 이 규칙은 park와 retrieve에 동일하게 적용하되 모든 ready, commit과 completion은 현재 active `mission_id`와 일치해야 한다.

Fleet Manager는 현재 활성 미션과 일치하는 `/mission/complete`를 받은 뒤에만 활성 미션을 reset하고 다음 요청을 받을 수 있다. 별도의 HOME topic이나 Fleet의 Front/Rear `return_done` 직접 집계는 추가하지 않는다.
