# 강체주행 차량 global pose 보정 — 2026-09-02

차량 인양 후 Rear 카메라의 Front ID0가 가려질 수 있고, 메카넘 wheel odom의
상대 횡이동은 실측보다 크게 누적됐다. production 역할을 다음처럼 분리한다.

- CCTV YOLO + Homography: 운반 차량의 map-frame `x/y` 보정
- Rear ID0: Front/Rear 상대 `x/y/yaw` 측정과 formation 보정
- Front/Rear wheel odom: 고주기 진행량과 상대 `x/yaw` 예측

YOLO 프레임은 Lift 때 확정한 물리 `vehicle_offset_body`를 다시 쓰지 않는다.
별도 map translation bias만 보정한다. transport yaw는 Front/Rear odom heading의
circular mean을 사용하며, Front-Rear 위치 연결축은 formation geometry로만 남긴다.

실측 과대누적이 확인된 wheel relative-y는 production에서 hold한다. ID0가 stale한
동안 마지막 lateral 오차를 open-loop로 계속 보정하지 않고 해당 제어주기에는
상대 Y 명령을 0으로 만든다. fresh YOLO vehicle x/y가 있으면 전체 강체의 global
경로는 저속으로 계속 보정하고, ID0와 YOLO가 모두 사라지면 bounded grace 뒤
경로를 보존한 recoverable HOLD로 전환한다.

CCTV merge는 `/robot/lifted=true`일 때 차량 중심이 아직 waiting ROI 안에 있어도
`/parking/vehicle_pose_feedback`을 발행한다. 따라서 ID0가 가려지는 초기 이탈
구간부터 global correction이 연결된다.
