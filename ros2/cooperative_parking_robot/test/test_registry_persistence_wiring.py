from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]


def test_fleet_and_all_server_launches_wire_registry_database_path():
    fleet = (
        ROOT / 'cooperative_parking_robot/fleet_manager_node.py').read_text()
    assert "'parking_registry_db_path'" in fleet
    assert 'registered_slots_fingerprint(' in fleet
    assert 'database_path=self.registry_database_path' in fleet

    for filename in (
            'cctv_server.launch.py',
            'cctv_server_dual.launch.py',
            'full_system.launch.py'):
        launch = (ROOT / 'launch' / filename).read_text()
        assert "'parking_registry_db_path'" in launch
        assert "'parking_registry.db'" in launch
        assert (
            "'parking_registry_db_path': LaunchConfiguration(" in launch)


def test_runtime_registry_database_files_are_ignored():
    gitignore = (ROOT / '.gitignore').read_text()
    assert 'parking_registry.db' in gitignore
    assert 'parking_registry.db-*' in gitignore
