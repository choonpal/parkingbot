---
status: accepted
---

# 차량번호와 주차 비밀번호로 출차 차량을 식별한다

운영자는 입차 때 차량번호, 주차 비밀번호와 원하는 빈 슬롯을 제출하고, 출차 때는 차량번호와 같은 비밀번호만 제출한다. Fleet Manager는 기존 `/ui/mission_request` 안에서 이 값을 받아 Parking Registry record를 인증한 뒤 `source_slot_id`를 자체 결정한다. UI가 source slot, pose, spec 또는 mission ID를 출차 권한으로 지정하지 못하게 하며, `source_slot_id` 단독 요청은 비밀번호 우회가 되므로 거부한다.

차량번호는 공백 제거와 영문 대문자화 후 세션에서 유일해야 한다. 주차 비밀번호 원문은 요청 처리 중에만 존재하고 Registry에는 무작위 salt를 사용한 PBKDF2-SHA256 검증값만 저장한다. `/fleet/state`, request status, completion record와 로그에는 차량번호·비밀번호·검증값을 싣지 않는다. 이번 demo의 Flask/ROS transport 자체는 암호화하거나 사용자 계정을 제공하지 않으므로 계속 trusted LAN 전용이며, 네트워크 인증과 영속 credential recovery는 후속 운영 범위다.
