# 기능별 소프트웨어 테스트

자동 테스트는 실제 카메라·STM32를 열지 않고, 격리된 localhost ROS domain에서
가짜 상위 입력을 실제 production 로직에 넣어 변환·계획·상태 전이를 확인한다.
Pose fusion, CCTV merge, 초음파 edge, mission FSM, rigid sync와 Fleet 경로
계획은 실제 배포 node와 DDS 입출력 경계를 통과한다. Fleet은 가짜 map·slot·
target·vehicle spec·paired odom을 받은 뒤 실제 A*·footprint·임시 slot registry로
계획하고 Path와 slot pose topic을 발행한다.
실행기는 localhost 전용 domain을 사용하고, 그 domain에 기존 local ROS node가
보이면 가짜 데이터를 보내지 않고 즉시 중단한다.

## 실행

```bash
cd ~/parkingbot/ros2/cooperative_parking_robot
scripts/run_feature_tests.sh all
```

필요한 기능만 실행할 수도 있다.

```bash
scripts/run_feature_tests.sh perception
scripts/run_feature_tests.sh localization
scripts/run_feature_tests.sh fleet
scripts/run_feature_tests.sh entry
scripts/run_feature_tests.sh mission
scripts/run_feature_tests.sh rigid-sync
scripts/run_feature_tests.sh rigid-pair
```

`PASS`가 아니면 다음 단계나 실차 시험으로 넘어가지 않는다. Perception 자동
테스트는 병합·안전 정책을 검사하며, 실제 카메라/YOLO 화질은 대체하지 않는다.
STM32 UART, 모터 방향, 센서 freshness는 실제 장비에서 별도 확인해야 한다.

## 실제 강체 키보드 주행

가짜 토픽을 쓰지 않는다. ID0 ArUco, 양쪽 wheel odom, hardware ready와 manual
ACK를 모두 받아야 arm 된다. 명령과 안전 제한은
[강체 쌍 키보드 주행](KEYBOARD_FOLLOW_TEST.md)을 따른다.
