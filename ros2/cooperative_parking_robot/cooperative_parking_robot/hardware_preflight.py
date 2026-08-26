#!/usr/bin/env python3
"""ROS 2 Humble 실차 실행 전 환경·파일·장치 사전 점검."""

from __future__ import annotations

import argparse
import importlib
import os
from pathlib import Path
import platform
import sys
from typing import Mapping, MutableSequence, Optional

from cooperative_parking_robot.camera_calibration import (
    load_camera_calibration,
)
from cooperative_parking_robot.vision_utils import normalize_model_mode


EXPECTED_ROS_DISTRO = 'humble'


def default_cctv_camera_calib_path() -> str:
    try:
        from ament_index_python.packages import get_package_share_directory
        return str(Path(get_package_share_directory(
            'cooperative_parking_robot')) / 'config' /
            'cctv_camera_calibration.npz')
    except Exception:
        return 'cctv_camera_calibration.npz'


def check_ros_environment(
        errors: MutableSequence[str],
        warnings: MutableSequence[str],
        environ: Optional[Mapping[str, str]] = None) -> None:
    env = os.environ if environ is None else environ
    distro = env.get('ROS_DISTRO', '')
    if distro != EXPECTED_ROS_DISTRO:
        errors.append(
            f'ROS_DISTRO={distro or "<unset>"}; '
            f'/opt/ros/{EXPECTED_ROS_DISTRO}/setup.bash를 source해야 함')

    localhost_only = str(env.get('ROS_LOCALHOST_ONLY', '0')).lower()
    if localhost_only in ('1', 'true', 'yes'):
        errors.append(
            'ROS_LOCALHOST_ONLY가 활성화됨 — Jetson/RPi 간 DDS 통신 불가')

    if not env.get('ROS_DOMAIN_ID'):
        warnings.append(
            'ROS_DOMAIN_ID 미설정 — 세 장비에서 같은 값으로 명시 권장')
    if not env.get('RMW_IMPLEMENTATION'):
        warnings.append(
            'RMW_IMPLEMENTATION 미설정 — 세 장비에서 같은 RMW 사용 권장')


def check_host_runtime(warnings: MutableSequence[str]) -> None:
    if sys.version_info[:2] != (3, 10):
        warnings.append(
            f'Python {platform.python_version()} 감지 — Ubuntu 22.04의 '
            'ROS 2 Humble binary 환경은 Python 3.10 기준')
    if platform.system() != 'Linux':
        warnings.append(f'Linux가 아닌 호스트 감지: {platform.system()}')


def require_import(
        errors: MutableSequence[str], label: str, module_name: str):
    try:
        return importlib.import_module(module_name)
    except Exception as exc:  # ImportError뿐 아니라 ABI mismatch도 잡는다.
        errors.append(f'{label} import 실패 ({module_name}): {exc}')
        return None


def validate_opencv_aruco(errors: MutableSequence[str], cv2_module) -> None:
    if cv2_module is None:
        return
    aruco = getattr(cv2_module, 'aruco', None)
    if aruco is None:
        errors.append(
            'cv2.aruco 없음 — ArUco 포함 OpenCV 빌드가 필요함')
        return
    if not hasattr(aruco, 'DICT_4X4_50'):
        errors.append('cv2.aruco.DICT_4X4_50 없음')
    if not (hasattr(aruco, 'ArucoDetector') or
            hasattr(aruco, 'detectMarkers')):
        errors.append('지원 가능한 OpenCV ArUco detector API 없음')


def require_file(errors: MutableSequence[str], label: str, path: str) -> bool:
    file_path = Path(path).expanduser()
    if not path or not file_path.is_file():
        errors.append(f'{label} 없음: {path}')
        return False
    if file_path.stat().st_size == 0:
        errors.append(f'{label} 빈 파일: {path}')
        return False
    return True


def validate_homography(
        errors: MutableSequence[str], path: str,
        label: str = 'homography') -> None:
    if not require_file(errors, label, path):
        return
    try:
        import numpy as np
        matrix = np.load(Path(path).expanduser())
        if matrix.shape != (3, 3) or not np.all(np.isfinite(matrix)):
            errors.append(f'{label}는 finite 3x3 행렬이어야 함')
        elif abs(float(np.linalg.det(matrix))) < 1e-12:
            errors.append(f'{label} 행렬이 singular임')
    except Exception as exc:
        errors.append(f'{label} 로드 실패: {exc}')


def validate_camera_calib(
        errors: MutableSequence[str], path: str, label: str) -> None:
    if not require_file(errors, label, path):
        return
    try:
        load_camera_calibration(path)
    except Exception as exc:
        errors.append(f'{label} 로드 실패: {exc}')


def validate_second_cctv_assets(
        errors: MutableSequence[str], calibration_path: str,
        homography_path: str, required: bool = False) -> None:
    """Validate the second camera pair without accepting a half-configured pair."""
    calibration_path = str(calibration_path or '').strip()
    homography_path = str(homography_path or '').strip()
    if required and (not calibration_path or not homography_path):
        errors.append(
            'dual CCTV는 --cctv2-camera-calib와 --homography2-file이 모두 필요함')
        return
    if bool(calibration_path) != bool(homography_path):
        errors.append(
            '두 번째 CCTV calibration과 homography는 함께 지정해야 함')
        return
    if calibration_path:
        validate_camera_calib(
            errors, calibration_path, 'CCTV2 camera calibration')
        validate_homography(errors, homography_path, 'CCTV2 homography')


def validate_serial(errors: MutableSequence[str], path: str) -> None:
    serial_path = Path(path).expanduser()
    if not serial_path.exists():
        errors.append(f'STM32 serial 장치 없음: {path}')
    elif not os.access(serial_path, os.R_OK | os.W_OK):
        errors.append(
            f'STM32 serial 읽기/쓰기 권한 없음: {path} '
            '(dialout 그룹/udev rule 확인)')


def validate_role_dependencies(
        errors: MutableSequence[str], role: str,
        require_cctv_markers: bool, require_rear_aruco: bool,
        skip_serial: bool,
        model_path: str = '', enable_operator_ui: bool = True,
        enable_debug_overlay: bool = False) -> None:
    require_import(errors, 'ROS 2 Python', 'rclpy')
    require_import(errors, 'NumPy', 'numpy')

    if role == 'jetson':
        cv2_module = require_import(errors, 'OpenCV', 'cv2')
        require_import(errors, 'cv_bridge', 'cv_bridge')
        require_import(errors, 'Ultralytics', 'ultralytics')
        suffix = Path(model_path).suffix.lower()
        if suffix == '.engine':
            require_import(errors, 'TensorRT Python', 'tensorrt')
        else:
            require_import(errors, 'PyTorch', 'torch')
        if require_cctv_markers:
            validate_opencv_aruco(errors, cv2_module)
        if enable_operator_ui or enable_debug_overlay:
            require_import(errors, 'Flask', 'flask')
            require_import(errors, 'Werkzeug', 'werkzeug')
        return

    if not skip_serial:
        require_import(errors, 'pyserial', 'serial')
    if role == 'rear' and require_rear_aruco:
        cv2_module = require_import(errors, 'OpenCV', 'cv2')
        require_import(errors, 'cv_bridge', 'cv_bridge')
        validate_opencv_aruco(errors, cv2_module)


def parse_args(argv=None):
    parser = argparse.ArgumentParser(
        description='ROS 2 Humble cooperative parking robot preflight')
    parser.add_argument('--role', required=True,
                        choices=('jetson', 'front', 'rear'))
    parser.add_argument('--serial-port', default='/dev/ttyACM0')
    parser.add_argument('--model-path', default='yolov8n.pt')
    parser.add_argument(
        '--model-mode', default='coco',
        choices=('coco', 'vehicle_seg', 'parking_seg'),
        help=('coco: COCO 차량 box, vehicle_seg: 차량 mask+고정 슬롯(권장), '
              'parking_seg: 기존 vehicle/empty_slot 커스텀 모델'))
    parser.add_argument(
        '--disable-operator-ui', action='store_true',
        help='운용 kiosk/API를 사용하지 않을 때 웹 의존성 검사를 생략')
    parser.add_argument(
        '--enable-debug-overlay', action='store_true',
        help='진단 영상 overlay 실행 구성을 함께 검사')
    parser.add_argument('--homography-file',
                        default='homography_rectified.npy')
    parser.add_argument(
        '--cctv-camera-calib', default=default_cctv_camera_calib_path(),
        help='Jetson 천장 카메라 calibration .npz')
    parser.add_argument('--dual-cctv', action='store_true',
                        help='두 번째 CCTV calibration/Homography를 필수 검사')
    parser.add_argument('--cctv2-camera-calib', default='',
                        help='두 번째 천장 카메라 calibration .npz')
    parser.add_argument('--homography2-file', default='',
                        help='두 번째 천장 카메라 rectified Homography .npy')
    parser.add_argument(
        '--rear-camera-calib', default='rear_camera_calibration.npz',
        help='Rear ArUco 카메라 전용 calibration .npz')
    parser.add_argument(
        '--camera-calib', default='',
        help='하위호환 alias; 해당 role의 calibration 경로를 덮어씀')
    parser.add_argument(
        '--software-only', action='store_true',
        help='파일/UART 장치 검사를 건너뛰고 ROS/Python 의존성만 검사')
    parser.add_argument(
        '--skip-serial', action='store_true',
        help='UART 없는 bench 점검 시 pyserial/장치 검사를 생략')
    parser.add_argument(
        '--disable-cctv-markers', action='store_true',
        help='Jetson 상판 ArUco 노드를 비활성화할 때 ArUco 검사를 생략')
    parser.add_argument(
        '--disable-rear-aruco', action='store_true',
        help='Rear ArUco tracker를 비활성화할 때 카메라 검사를 생략')
    return parser.parse_args(argv)


def main(argv=None):
    args = parse_args(argv)
    errors = []
    warnings = []

    check_ros_environment(errors, warnings)
    check_host_runtime(warnings)
    validate_role_dependencies(
        errors, args.role,
        require_cctv_markers=not args.disable_cctv_markers,
        require_rear_aruco=not args.disable_rear_aruco,
        skip_serial=args.skip_serial or args.software_only,
        model_path=args.model_path,
        enable_operator_ui=not args.disable_operator_ui,
        enable_debug_overlay=args.enable_debug_overlay)

    if not args.software_only:
        if args.role == 'jetson':
            model_mode = normalize_model_mode(args.model_mode)
            model_path = Path(args.model_path).expanduser()
            if not model_path.is_file():
                errors.append(f'YOLO model 없음: {args.model_path}')
            if (model_mode in ('vehicle_seg', 'parking_seg') and
                    not model_path.is_file()):
                errors.append(
                    f'{model_mode} 모드는 학습된 로컬 모델 파일이 필요함')
            validate_camera_calib(
                errors, args.camera_calib or args.cctv_camera_calib,
                'CCTV camera calibration')
            validate_homography(errors, args.homography_file)
            validate_second_cctv_assets(
                errors, args.cctv2_camera_calib, args.homography2_file,
                required=args.dual_cctv)
        else:
            if not args.skip_serial:
                validate_serial(errors, args.serial_port)
            if args.role == 'rear' and not args.disable_rear_aruco:
                validate_camera_calib(
                    errors, args.camera_calib or args.rear_camera_calib,
                    'Rear camera calibration')

    for warning in warnings:
        print(f'WARNING: {warning}')

    if errors:
        print('HUMBLE PREFLIGHT: FAIL')
        for error in errors:
            print(f'  - {error}')
        return 1

    mode = 'software-only' if args.software_only else 'hardware'
    print(
        f'HUMBLE PREFLIGHT: PASS '
        f'(role={args.role}, mode={mode}, python={platform.python_version()})')
    return 0


if __name__ == '__main__':
    sys.exit(main())
