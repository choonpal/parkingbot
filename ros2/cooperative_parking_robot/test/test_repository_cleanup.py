"""Repository hygiene and production-only runtime regression tests."""

import hashlib
from pathlib import Path


PACKAGE_ROOT = Path(__file__).resolve().parents[1]
REPOSITORY_ROOT = PACKAGE_ROOT.parents[1]


def _project_text_files():
    roots = (
        REPOSITORY_ROOT / 'README.md',
        REPOSITORY_ROOT / 'CONTEXT.md',
    )
    yield from (path for path in roots if path.is_file())
    yield from (REPOSITORY_ROOT / 'docs').rglob('*.md')
    for pattern in ('*.py', '*.md', '*.xml', '*.yaml', '*.cfg', '*.sh'):
        yield from PACKAGE_ROOT.rglob(pattern)


def test_main_tree_has_no_nav2_residue():
    offenders = []
    for path in _project_text_files():
        if path.resolve() == Path(__file__).resolve():
            continue
        if 'nav2' in path.read_text(encoding='utf-8').lower():
            offenders.append(str(path.relative_to(REPOSITORY_ROOT)))
    assert offenders == []


def test_colcon_generated_directories_are_ignored_at_repository_root():
    ignore = (REPOSITORY_ROOT / '.gitignore').read_text(encoding='utf-8')
    for entry in ('/ros2/build/', '/ros2/install/', '/ros2/log/'):
        assert entry in ignore


def test_only_authoritative_stm32_project_remains():
    authoritative = REPOSITORY_ROOT / 'stm32' / 'parking_robot'
    assert (authoritative / 'parking_robot.ioc').is_file()
    assert (authoritative / 'Core/Src/parking_robot_firmware.c').is_file()
    assert not (authoritative / 'backup.ioc').exists()
    assert not (PACKAGE_ROOT / 'stm32_firmware').exists()
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert 'stm32_firmware' not in setup_source


def test_dummy_homography_generator_is_not_in_production_package():
    assert not (PACKAGE_ROOT / 'scripts/make_dummy_calibration.py').exists()
    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert "glob('scripts/*.py')" not in setup_source
    for relative in (
            'docs/CCTV_CALIBRATION.md',
            '../../docs/REAL_ROBOT_DEPLOYMENT_RUNBOOK.md'):
        source = (PACKAGE_ROOT / relative).resolve().read_text(encoding='utf-8')
        assert 'make_dummy_calibration' not in source


def test_mission_yolo_requires_a_local_model_without_download_switch():
    yolo = (
        PACKAGE_ROOT / 'cooperative_parking_robot/yolo_bev_map_node.py'
    ).read_text(encoding='utf-8')
    preflight = (
        PACKAGE_ROOT / 'cooperative_parking_robot/hardware_preflight.py'
    ).read_text(encoding='utf-8')
    launches = [
        path.read_text(encoding='utf-8')
        for path in (PACKAGE_ROOT / 'launch').glob('*.launch.py')
    ]
    combined = '\n'.join([yolo, preflight, *launches])
    assert 'allow_model_download' not in combined
    assert 'allow-model-download' not in combined
    assert 'if not os.path.isfile(mp):' in yolo
    assert 'model_path.is_file()' in preflight


def test_web_debug_overlay_never_loads_or_runs_yolo():
    web = (
        PACKAGE_ROOT / 'cooperative_parking_robot/jetson_vision_web_node.py'
    ).read_text(encoding='utf-8')
    assert 'from ultralytics import YOLO' not in web
    assert "declare_parameter('enable_yolo'" not in web
    assert 'def _run_yolo' not in web
    assert 'def _draw_yolo' not in web
    for launch in (PACKAGE_ROOT / 'launch').glob('*.launch.py'):
        assert 'debug_enable_yolo' not in launch.read_text(encoding='utf-8')


def test_trained_vehicle_seg_model_is_packaged_and_is_camera_default():
    model_name = 'parking_vehicle_yolo11n_seg.pt'
    model_path = PACKAGE_ROOT / 'models' / model_name
    assert model_path.is_file()
    assert model_path.stat().st_size == 6_031_189
    assert hashlib.sha256(model_path.read_bytes()).hexdigest() == (
        'e60179f0ad4a1b346b1b464dbc0cf93075f1c91385820683b384e238e8c7d896')

    setup_source = (PACKAGE_ROOT / 'setup.py').read_text(encoding='utf-8')
    assert 'models' in setup_source
    assert 'glob(\'models/*.pt\')' in setup_source

    for launch_name in (
            'full_system.launch.py',
            'cctv_server.launch.py',
            'cctv_server_dual.launch.py'):
        launch_source = (
            PACKAGE_ROOT / 'launch' / launch_name
        ).read_text(encoding='utf-8')
        assert '\'models\', \'parking_vehicle_yolo11n_seg.pt\'' in launch_source
        assert 'model_mode\', default_value=\'vehicle_seg\'' in launch_source
        assert 'inference_imgsz\', default_value=\'640\'' in launch_source
