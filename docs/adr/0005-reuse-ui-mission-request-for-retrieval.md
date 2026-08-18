---
status: superseded by ADR-0018
---

# 출차도 기존 UI 미션 요청 topic을 사용한다

출차 요청은 새 topic, service 또는 action을 만들지 않고 기존 `/ui/mission_request`의 `std_msgs/String` JSON 계약을 확장한다. 기존 입차 payload와 동작은 유지한다.

출차 payload는 다음 필드를 사용한다.

```json
{
  type: retrieve,
  source_slot_id: A3,
  request_id: ui-...,
  sequence: 7,
  stamp_ns: 123456789
}
```

UI는 사용자가 선택한 `source_slot_id`만 업무 데이터로 전달하며 vehicle pose, vehicle spec 또는 `mission_id`를 만들지 않는다. Fleet Manager는 `park`와 `retrieve`를 구분하고, 출차 요청의 슬롯이 Parking Registry에 존재하며 정확히 `OCCUPIED`인지 검증한다. `EMPTY`, `RESERVED`, `EXIT_RESERVED`, `EXITING` 슬롯, 누락되거나 알 수 없는 슬롯, 다른 활성 미션이 있는 경우에는 승인하지 않는다.

Fleet Manager는 승인된 Registry record에서 최종 차량 자세, 주차 방향과 차량 제원을 가져오고 고정 waiting 목적지를 결정한다. 승인 시 새 `mission_id`를 만들고 active mission의 type을 `retrieve`로 저장하며 슬롯을 `EXIT_RESERVED`로 전환한다. `EXIT_RESERVED -> EXITING`의 정확한 전이 barrier는 별도 결정으로 남긴다.

기존 QoS, freshness, sequence 및 request ID 구조를 최대한 재사용하고 반복 전달이 동일 미션을 두 번 시작하지 않도록 한다. 요청 중복 판정의 세부 기준은 별도 결정으로 남긴다.
