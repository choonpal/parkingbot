# 코드·문서 점검 이슈 (2026-08-26)

## 점검 범위

- 브랜치: `fix/site-map-aruco-calibration`
- 코드, 테스트, 실행 설정, 사용자·배포 문서 간 일관성
- 기존 작업 트리 변경은 보존하며 점검

## 발견 이슈

### I-001 [해결] — ROS 설치 환경에서 테스트용 clock이 잘못된 메시지 타입을 반환함

- 심각도: Medium (회귀 테스트 신뢰성)
- 위치: `ros2/cooperative_parking_robot/test/test_fleet_retrieval_integration.py:41-51`
- 증상: 실제 ROS 2 `geometry_msgs/PoseStamped`를 사용하는 환경에서
  `AdjustableClock.now().to_msg()`가 `builtin_interfaces.msg.Time`이 아닌
  `SimpleNamespace`를 반환한다. `header.stamp` 대입 시 type assertion으로 실패한다.
- 재현: `python3 -m pytest -q`
- 영향: 실제 ROS 설치 여부에 따라 같은 테스트가 통과하거나 실패하여 CI/현장
  검증 결과가 달라진다.
- 조치: 테스트 clock이 실제 `builtin_interfaces.msg.Time` 또는 프로젝트의 ROS
  fallback과 호환되는 Time 객체를 반환하도록 수정한다.

### I-002 [해결] — 출차 preflight 통합 테스트가 계산 중 odometry timeout으로 실패함

- 심각도: Medium (회귀 테스트 신뢰성 및 preflight 성능 가시성)
- 위치: `ros2/cooperative_parking_robot/test/test_fleet_retrieval_integration.py:416-444`
- 증상: `odom_timeout=1.0`인 단일 timestamp를 넣은 뒤 비용이 큰 preflight를 여러
  slot에 연속 수행한다. 첫 검사들에 약 1초 이상이 걸려 이후
  `current_virtual_start()`가 stale odometry로 판정되고 sequential 정책 assertion이
  실패한다.
- 재현: `python3 -m pytest -q test/test_fleet_retrieval_integration.py::test_demo_p1_p4_preflight_uses_front_first_clearance_policy -vv`
- 영향: 기하 정책 회귀가 없어도 CPU 속도와 실행 순서에 따라 테스트가 실패한다.
- 조치: monotonic clock을 고정/주입하거나 각 case마다 receipt를 갱신하고, 별도
  성능 테스트로 단일 preflight의 실행 시간 budget을 검증한다.

### I-003 [해결] — ROS 2 Humble 기본 pytest에서 지원하지 않는 설정 사용

- 심각도: Low (검증 로그 오염/호환성)
- 위치: `ros2/cooperative_parking_robot/pytest.ini:2`
- 증상: Ubuntu 22.04/ROS 2 Humble 환경의 pytest 6.2.5가 `pythonpath` 옵션을
  인식하지 못해 모든 테스트 실행에서 `Unknown config option: pythonpath` 경고를
  낸다.
- 영향: 실제 새 경고를 식별하기 어려워지고, 문서가 기준으로 삼는 Humble 환경과
  테스트 설정이 맞지 않는다.
- 조치: 패키지 루트 실행/설치 환경에서 불필요한 `pythonpath = .`를 제거한다.

### I-004 [해결] — 실제 CCTV 해상도 불일치가 Homography 좌표 오류로 이어져도 계속 발행됨

- 심각도: High (실차 위치/장애물 오인 가능)
- 위치: `ros2/cooperative_parking_robot/cooperative_parking_robot/opencv_camera_node.py:138-153`,
  `ros2/cooperative_parking_robot/launch/cctv_server_dual.launch.py:156-168`
- 증상: 카메라가 요청한 640×480을 거부하고 다른 해상도로 frame을 반환해도 camera
  node는 경고만 남기고 publish한다. Dual CCTV Homography는 640×480 rectified pixel
  좌표계에 묶여 있으므로 다른 크기의 rectified frame에 그대로 적용하면 map 좌표가
  틀어진다. 외부 camera driver 사용 시에도 같은 문제가 생길 수 있다.
- 영향: 차량/slot/로봇 위치가 잘못된 map 위치로 변환된 뒤 Fleet 계획 입력으로
  사용될 수 있다.
- 조치: 실운용 rectifier에 기대 frame 크기와 `require_exact_resolution` fail-closed
  검사를 추가하고 dual launch에서는 기본 활성화한다. 불일치 frame은 downstream에
  발행하지 않아 camera freshness gate가 미션을 차단하게 한다.

### I-005 [잔여: Low] — 전체 ament flake8 gate가 108건으로 실패함

- 심각도: Low (정적 품질 gate 부재)
- 위치: 패키지 Python/launch/test 전반; 실행 관련 F 오류는
  `cooperative_parking_robot/individual_move_node.py:33`
- 최초 증상: `ament_flake8 cooperative_parking_robot launch test setup.py`가
  74개 docstring(D), 33개 formatting(E), 1개 unused import(F401)로 실패했다.
- 현재 상태: 실행 관련 F 1건과 formatting E 33건은 모두 해결됐다. 전체 lint에는
  실행에 영향을 주지 않는 기존 docstring 스타일 D 74건만 남아 있다.
- 영향: 현재 저장소는 일반적인 ROS 2 Python 정적 gate를 통과하지 못해 신규 결함과
  기존 부채를 구분하기 어렵다.
- 조치: unused import와 formatting 오류는 제거했다. 남은 docstring 문체 부채는
  동작 변경과 분리해 일괄 정리하거나 명시적인 프로젝트 lint 기준을 정한다.

### I-006 [해결] — 카메라 장애 후에도 latch된 입차 target을 계속 fresh/ready로 재발행함

- 심각도: High (stale mission input)
- 위치: `ros2/cooperative_parking_robot/cooperative_parking_robot/bev_fusion_core.py:580-607`,
  `ros2/cooperative_parking_robot/cooperative_parking_robot/cctv_merge_node.py:402-516`
- 증상: 한 번 latch된 target은 mission complete 전까지 검출 소실/카메라 timeout과
  무관하게 유지된다. `require_all_cameras=false` 기본에서는 target을 본 카메라가
  죽고 다른 카메라만 살아도 과거 좌표를 새 timestamp와 `target_ready=true`로
  계속 발행한다. 모든 카메라가 죽는 조기 return도 즉시 false를 발행하지 않는다.
- 영향: UI/Fleet가 실제로는 관측되지 않는 차량 좌표로 입차 미션을 승인할 수 있다.
- 조치: 실운용 기본을 `require_all_cameras=true`로 바꾸고 required camera 누락 시
  target/empty/map을 즉시 fail-closed한다. 명시적 degraded mode에서도 latch 위치를
  보는 live coverage가 없으면 latch를 해제한다.

### I-007 [해결] — CCTV 미관측 영역이 OccupancyGrid에서 unknown이 아니라 free로 생성됨

- 심각도: High (미관측 장애물로 경로 계획 가능)
- 위치: `ros2/cooperative_parking_robot/cooperative_parking_robot/cctv_merge_node.py:604-642`
- 증상: 맵 전체를 0(free)으로 초기화한 뒤 검출 차량만 100으로 칠한다. 살아있는
  camera coverage 밖도 -1(unknown)이 아니므로 Fleet의
  `unknown_is_occupied=true` 정책이 적용되지 않는다.
- 영향: 죽은 카메라 영역이나 원래 시야 밖의 미관측 공간을 A*/출차 preflight가
  안전한 빈 공간으로 사용할 수 있다.
- 조치: 전체를 -1로 시작하고 live coverage 안만 0으로 rasterize한 뒤 장애물을
  100으로 표시한다. robot self-mask도 unknown을 free로 바꾸지 않게 한다.

### I-008 [해결] — 복사 실행 가능한 shell 문서 블록 2개에 syntax error가 있음

- 심각도: Low (운용 문서 정확성)
- 위치: `dual_tile_homography_tool/README.md:66`,
  `ros2/cooperative_parking_robot/docs/HUMBLE_DEPLOYMENT.md:113`
- 증상: `/dev/v4l/by-id/<CAM2-ID>`와 `:=<실측값>`의 `<...>`가 Bash redirection으로
  해석돼 `bash -n`에서 실패한다.
- 조치: 검증 가능한 환경변수 placeholder로 교체한다.

### I-009 [해결] — Jetson 필수 ML runtime 설치 절차가 배포 문서에 없음

- 심각도: High (신규 장비 배포 실패)
- 위치: `docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md`,
  `ros2/cooperative_parking_robot/package.xml:18-24`,
  `ros2/cooperative_parking_robot/cooperative_parking_robot/hardware_preflight.py:144-157`
- 증상: Runbook은 `rosdep install`만 안내하지만 dual launch/preflight가 요구하는
  Ultralytics와 PyTorch/TensorRT는 package.xml/rosdep만으로 설치되지 않는다.
- 영향: 문서 그대로 구성한 Jetson에서 preflight 또는 vision node가 시작되지 않는다.
- 조치: JetPack과 맞는 pinned ML runtime 설치·검증 절차를 Runbook에 명시한다.

### I-010 [해결] — 듀얼 CCTV preflight가 cam0 자산만 검사함

- 심각도: High (미검증 calibration/Homography로 기동 가능)
- 위치: `docs/pipeline.md:159-163`,
  `ros2/cooperative_parking_robot/cooperative_parking_robot/hardware_preflight.py:186-241`
- 증상: 실제 launch는 cam0/cam2 각각의 calibration과 Homography를 요구하지만
  preflight CLI와 문서 예시는 한 쌍만 검사한다.
- 영향: cam2 자산 누락/손상/잘못된 행렬을 사전검사에서 놓친다.
- 조치: preflight가 두 번째 asset 쌍을 선택적으로 필수 검사하도록 확장하고 dual
  운용 문서에서 네 파일을 모두 넘긴다.

### I-011 [해결] — camera ID “fallback” 설명과 실제 선택 규칙이 모호함

- 심각도: Medium (현장 기동 실패/역할 혼동)
- 위치: `ros2/cooperative_parking_robot/cooperative_parking_robot/opencv_camera_node.py:22-25`,
  `ros2/cooperative_parking_robot/launch/cctv_server_dual.launch.py:145-155`
- 증상: `camera_device`가 비어 있지 않으면 경로가 존재하지 않아도 ID로 자동
  fallback하지 않는다. 반면 이름/문서 일부는 ID fallback으로만 표현한다.
- 영향: site by-path가 다른 장비에서 ID가 설정돼 있어도 open이 실패한다.
- 조치: 안전상 자동 숫자 fallback은 하지 않고, ID는 `camera_device:=''`를 명시한
  경우에만 사용된다고 문서와 설명을 정확히 맞춘다.

### I-012 [해결] — cam2 debug 영상을 선택해도 cam0 intrinsic을 사용함

- 심각도: Medium (진단 pose/거리 오표시)
- 위치: `ros2/cooperative_parking_robot/launch/cctv_server_dual.launch.py:473-488`
- 증상: `debug_image_topic`은 cam0/cam2를 선택할 수 있지만 Web debug node의
  `camera_calib`은 항상 `cctv0_camera_calib`이다.
- 영향: cam2 디버그 ArUco overlay/PnP의 pose와 거리가 잘못 표시될 수 있다.
- 조치: 별도 `debug_camera_calib` 인자를 추가해 선택한 topic과 같은 카메라 자산을
  명시하게 한다.

### I-013 [해결] — 듀얼 marker에서 cam2 카메라 높이 파라미터가 사용되지 않음

- 심각도: Medium (카메라별 parallax 보정 오류)
- 위치: `ros2/cooperative_parking_robot/launch/cctv_server_dual.launch.py:207-213,450-466`,
  `ros2/cooperative_parking_robot/cooperative_parking_robot/cctv_robot_marker_node.py`
- 증상: cam0/cam2 높이를 따로 선언하지만 marker node에는 cam0 값 하나만 전달한다.
- 영향: 두 CCTV 설치 높이가 다르면 cam2 marker의 parallax 보정이 틀린다.
- 조치: marker node에 카메라별 높이 배열을 전달하고 선택된 관측의 높이를 사용한다.

### I-014 [해결] — user-site setuptools 오염 시 colcon이 0 tests를 성공으로 처리함

- 심각도: High (배포 gate false positive)
- 위치: `ros2/cooperative_parking_robot/scripts/humble_build_check.sh`, 배포 문서
- 증상: 현재 Humble shell에서 `~/.local`의 setuptools 79.0.1이 시스템
  setuptools보다 우선되면 `setup.py` 경고 후 `colcon test`가 테스트를 하나도
  수집하지 않고 성공할 수 있다. `colcon test-result`도 `0 tests`를 실패로 보지 않는다.
- 영향: 실제 284개 회귀 테스트를 실행하지 않은 배포가 검증 통과로 오인된다.
- 조치: Humble build/test에 `PYTHONNOUSERSITE=1`을 적용하고, build check script가
  테스트 결과 수가 0이면 명시적으로 실패하도록 한다.

### I-015 [해결] — 설치된 ROS package 문서에서 상위 저장소 링크 24개가 깨짐

- 심각도: Low (설치 산출물 문서 탐색)
- 위치: `ros2/cooperative_parking_robot/docs/*.md`, `setup.py`의 docs data_files
- 증상: source tree에서는 유효한 `../../../docs/...` 링크가 install tree의
  `share/cooperative_parking_robot/docs`에서는 존재하지 않는 package 밖 경로를
  가리킨다.
- 영향: `ros2 pkg prefix` 아래 설치 문서를 열면 current Runbook/README 링크가
  깨진다.
- 조치: 설치 package 밖을 가리키던 archive 문서의 Markdown 링크를 경로 안내용
  inline code로 바꿨다. 설치 산출물의 문서 링크 검사에서 누락 0건을 확인했다.

### I-016 [해결] — 현재 pipeline의 preflight 복사 명령이 설치 환경에서 실행되지 않음

- 심각도: Medium (운용 문서 정확성)
- 위치: `docs/pipeline.md:159,172,179`
- 증상: ament Python console script인 `hardware_preflight`를 bare command로
  실행하도록 안내했다. workspace를 source해도 해당 libexec 경로는 `PATH`에 없어
  clean install에서 `command not found`가 발생한다.
- 영향: 문서의 Jetson/후륜/전륜 사전검사 명령을 그대로 실행할 수 없다.
- 조치: 세 예시를 모두
  `ros2 run cooperative_parking_robot hardware_preflight ...`로 수정하고 clean install
  환경에서 `--help` 실행을 확인했다.

### I-017 [해결] — 최신 main의 setup.py가 폐기된 파일을 다시 패키징함

- 심각도: High (설치 산출물·배포 회귀)
- 위치: `ros2/cooperative_parking_robot/setup.py`
- 증상: 최신 `origin/main`을 병합하자 존재하지 않는 `show_map_ascii` 실행 항목과
  폐기된 `stm32_firmware`, Python utility script 패키징이 다시 추가되고, production
  YOLO model과 `keyboard_teleop` 패키징은 빠졌다.
- 영향: Humble 회귀 테스트 284개 중 4개가 실패하고, 설치본에서 mission model이나
  필요한 teleop 실행 항목이 누락될 수 있다.
- 조치: main의 새 `camera_preview`·`tile_homography` 항목은 유지하되, 존재하지
  않거나 폐기된 항목은 제거하고 model·shell script·keyboard teleop의 authoritative
  package 구성을 복원했다.

### I-018 [잔여: Low] — 최신 main의 신규 카메라 도구가 lint gate를 통과하지 못함

- 심각도: Low (정적 품질·유지보수성)
- 위치: `camera_preview_node.py`, `tile_homography_node.py`
- 증상: 최신 `origin/main`에서 추가된 두 파일에 압축된 HTML/JavaScript와 한 줄
  Python 코드가 포함되어 E/F/W 383건과 docstring D 2건이 추가된다.
- 영향: 전체 `ament_flake8` gate가 다시 실패하며, 실제 unused import/variable과
  생성형 UI의 줄 길이 부채를 구분하기 어렵다.
- 조치: 기능·병합 회귀와 분리해 두 파일을 별도 formatting PR에서 정리한다.
  embedded web asset은 필요한 경우 명시적인 per-file E501 기준을 둔다.

### I-019 [해결] — camera_preview가 존재하지 않는 YOLO loader를 import함

- 심각도: Medium (신규 진단 도구 실행 불가)
- 위치: `camera_preview_node.py:96`, `vision_utils.py`
- 증상: `camera_preview` import 즉시 존재하지 않는
  `vision_utils.load_yolo_model` 때문에 `ImportError`가 발생했다.
- 영향: console entry point가 설치되어도 카메라 프리뷰 노드를 시작할 수 없다.
- 조치: 로컬 모델 파일만 허용하고 model mode에 맞는 task를 지정하는 공용 loader를
  구현했다. 모듈 import, segmentation task 선택, 누락 모델의 network-download 차단
  회귀 테스트를 추가했다.

## 검증 기록

- `python3 -m compileall -q cooperative_parking_robot launch test`: 통과
- `python3 -m pytest -q`: **285 passed**
- clean ROS 2 Humble `colcon build`/`colcon test`: **285 tests, 0 errors,
  0 failures, 0 skipped**
- 설치된 launch 6개의 `--show-args`: 모두 통과
- Python AST 71개, XML/YAML/NPZ, shell script 문법: 통과
- source 문서 로컬 링크 68개: 누락 0건
- 복사 실행 가능한 Bash 문서 블록 72개: 문법 오류 0건
- 설치 package 문서 링크: 누락 0건
- `git diff --check`: 통과
- `ament_flake8` (main 동기화 전): E/F/W 0건, 기존 docstring D 74건
- `ament_flake8` (최신 main 동기화 후): 신규 카메라 도구의 E/F/W 383건과
  docstring D 2건 추가(I-018)
