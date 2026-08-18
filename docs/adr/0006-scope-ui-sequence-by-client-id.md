---
status: accepted
---

# UI 요청 sequence를 client_id별로 관리한다

Web UI는 프로세스 시작 시 UUID 기반 optional `client_id`를 한 번 생성하고 같은 프로세스의 모든 park/retrieve 요청에 이를 포함한다. 요청의 `sequence`는 해당 UI 세션 안에서 증가한다.

Fleet Manager는 `client_id`가 있는 요청에 대해 `(client_id, sequence)` 단위로 순서를 검사한다. 같은 client에서는 이전 값보다 큰 sequence만 허용하고, 처음 보는 client는 새 UI 세션으로 취급한다. `client_id`가 없는 기존 요청은 현재의 global sequence 검사를 사용하여 하위 호환성을 유지한다.

park와 retrieve는 같은 요청 전처리를 사용한다. 기존 `stamp_ns` freshness 검사를 유지하고 최근 `request_id`도 중복 검사하여 동일 요청이 다시 실행되지 않게 한다. client별 마지막 sequence와 최근 request ID는 현재 코드 규모에 맞는 작은 bounded LRU/최근 목록으로 관리해 무한히 쌓이지 않게 한다.

이 중복 방지 기록은 활성 미션 상태가 아니므로 mission reset에서 지우지 않는다. 새 ROS topic은 추가하지 않고 `/ui/mission_request`의 optional JSON field만 확장한다.
