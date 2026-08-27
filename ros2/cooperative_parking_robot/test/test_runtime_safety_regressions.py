"""Focused regressions for mission/runtime safety fixes."""

from pathlib import Path

from cooperative_parking_robot.sync_faults import is_fatal_sync_error


ROOT = Path(__file__).resolve().parents[1]


def test_lateral_and_reference_faults_propagate_as_fatal():
    assert is_fatal_sync_error('LATERAL_ERROR_FATAL +41mm')
    assert is_fatal_sync_error('LATERAL_ERROR_TIMEOUT +22mm')
    assert is_fatal_sync_error('REFERENCE_CAPTURE_FAILED sample_dispersion')
    assert not is_fatal_sync_error('LATERAL_ERROR +21mm')
    assert not is_fatal_sync_error('ARRIVED')


def test_wait_target_does_not_create_mission_in_target_callback():
    source = (ROOT / 'cooperative_parking_robot' /
              'fleet_manager_node.py').read_text()
    callback = source.split('    def target_cb', 1)[1].split(
        '    def slots_cb', 1)[0]
    assert 'mission_id = str(uuid.uuid4())' not in callback
    assert 'target_candidate_receipt_time' in callback


def test_cctv_preservation_requires_lift_confirmation_in_wait_lift():
    source = (ROOT / 'cooperative_parking_robot' /
              'cctv_merge_node.py').read_text()
    assert "self.fleet_state == 'WAIT_LIFT' and self.vehicle_lifted" in source


def test_map_safety_features_are_wired():
    source = (ROOT / 'cooperative_parking_robot' /
              'cctv_merge_node.py').read_text()
    assert 'math.ceil(self.map_w_m / self.resolution)' in source
    assert 'robot_odom_freshness_s' in source
    assert 'static_obstacle_polygons_json' in source
    assert 'region[(region >= 0) & ~region_static] = 0' in source


def test_vehicle_spec_waits_for_real_segmentation_dimensions():
    source = (ROOT / 'cooperative_parking_robot' /
              'cctv_merge_node.py').read_text()
    helper = source.split('    def _publish_vehicle_spec', 1)[1].split(
        '    def _publish_empty_slots', 1)[0]
    assert 'if not self.dimension_tracker.dimension_valid' in helper
    assert "'dimension_source': 'segmentation_mask'" in helper


def test_production_entrypoint_remains_p0_wrapper_over_safe_node():
    setup = (ROOT / 'setup.py').read_text()
    production = (ROOT / 'cooperative_parking_robot' /
                  'rigid_body_sync_production_node.py').read_text()
    assert 'rigid_body_sync_production_node:main' in setup
    assert 'MissionReferenceRigidBodySyncNode' in production
