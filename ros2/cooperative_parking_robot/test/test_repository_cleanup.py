"""Repository hygiene and production-only runtime regression tests."""

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
