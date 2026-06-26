# 협동 자율 주차 로봇 시스템 (WBS 기반)

## 작업 분할 구조 (WBS)

```
[파트1] CCTV 서버 인지부 (Python/ROS2)
  1-1. yolo_bev_map_node     천장 카메라 → 맵 + 빈자리/타겟
  1-2. fleet_manager_node    관제탑, 빈자리 선정 + Nav2 명령

[파트2] 로봇 두뇌 - 라즈베리파이 (Python/ROS2)
  2-1. ultrasonic_edge_node      초음파 바퀴 엣지 검출
  2-2. aruco_tracker_node        ArUco 마커 보정
  2-3. rigid_body_sync_node      강체 주행 제어 (Nav2 + 칼만)
  2-4. robot_state_machine_node  로봇 상태 관리

[파트3] STM32 (C 펌웨어)
  3-1. uart_comm_task    라파 통신
  3-2. motor_pid_task    모터 PID
  3-3. servo_lift_task   arm Soft-start
```

## 노드별 토픽 맵

| 노드 | 입력 | 출력 |
|------|------|------|
| 1-1 yolo_bev_map | /cctv/image_raw | /parking/map, /parking/target_pose, /parking/empty_slots |
| 1-2 fleet_manager | target_pose, empty_slots, /robot/lifted | NavigateToPose 액션 |
| 2-1 ultrasonic | /robot/ultrasonic (또는 GPIO) | /robot/wheel_aligned |
| 2-2 aruco_tracker | /robot/front_camera/image | /sync/relative_pose, /sync/marker_visible |
| 2-3 rigid_body_sync | /cmd_vel(Nav2), /front,rear/odom, /sync/relative_pose | /front,rear/cmd_vel, /virtual_robot/odom |
| 2-4 state_machine | /robot/wheel_aligned, /fleet/state | /robot/state, /robot/lifted, /robot/grip_cmd |

## 데이터 흐름

```
천장 카메라 → [1-1] → /parking/map ─────────┐
                    → target/empty_slots    │
                          │                 │
                       [1-2] 관제탑          │
                          │ Nav2 goal        │
                          ▼                  ▼
                        Nav2 (Omni) ◄── /parking/map
                          │ /cmd_vel
                          ▼
        [2-3] rigid_body_sync ◄── [2-2] aruco
                          │       ◄── /front,rear/odom
              ┌───────────┴───────────┐
         /front/cmd_vel          /rear/cmd_vel
              │                       │
           STM32 [3-2]            STM32 [3-2]
           모터 PID                모터 PID
```

## 빌드 & 실행

```bash
cd ~/ros2_ws  # 또는 parking_robot_ws
colcon build --packages-select parking_robot
source install/setup.bash

ros2 launch parking_robot system_launch.py
```

## STM32 펌웨어

`stm32_firmware/parking_robot_firmware.c`를 STM32CubeIDE 프로젝트에 통합.
필요한 CubeMX 설정:
- USART2: 라즈베리파이 통신 (115200)
- TIM1: 모터 PWM 4채널
- TIM2: 서보 PWM
- TIM3/4/5/8: 엔코더 모드

main()에서:
```c
Robot_Init();
while (1) { Robot_MainLoop(); }
```

## 개별 노드 실행 (디버깅)

```bash
ros2 run parking_robot yolo_bev_map
ros2 run parking_robot fleet_manager
ros2 run parking_robot ultrasonic_edge
ros2 run parking_robot aruco_tracker
ros2 run parking_robot rigid_body_sync
ros2 run parking_robot state_machine --ros-args -p role:=front
```

## 남은 작업

1. YOLO 모델 학습 (모형 차량 + 빈자리)
2. 호모그래피 캘리브레이션 (calibrate.py)
3. 카메라 캘리브레이션 (ArUco용, camera_calibration.npz)
4. Nav2 bringup 통합
5. STM32 CubeMX 핀 설정 + 펌웨어 플래시
6. 실제 하드웨어 통합 테스트
