import json
import math
import time
from types import SimpleNamespace

import pytest

import cooperative_parking_robot.fleet_manager_node as fleet_manager_module
from cooperative_parking_robot.fleet_manager_node import (
    FleetManagerNode,
    normalize_planning_validation_mode,
)
from cooperative_parking_robot.parking_geometry import Pose2D, RegisteredSlot
from cooperative_parking_robot.parking_registry import (
    ParkingCredential,
    ParkingRegistry,
    SlotLifecycle,
    normalize_vehicle_number,
)
from cooperative_parking_robot.loaded_footprint import (
    compute_loaded_footprint,
)
from cooperative_parking_robot.mission_protocol import make_arrival_status
from cooperative_parking_robot.robot_state_machine_node import (
    RobotStateMachineNode,
)


SPEC = {
    'wheelbase': 0.70,
    'vehicle_length_m': 0.90,
    'vehicle_width_m': 0.35,
}
POSE = Pose2D(1.5, 3.0, 1.5707963267948966)
VEHICLE_NUMBER = '12가3456'
PASSWORD = '2468'


def test_planning_validation_mode_accepts_only_explicit_contract_values():
    assert normalize_planning_validation_mode('ENFORCE') == 'enforce'
    assert normalize_planning_validation_mode(' warn_only ') == 'warn_only'
    with pytest.raises(ValueError, match='planning_validation_mode'):
        normalize_planning_validation_mode('disabled')


def test_enforce_candidate_finding_waits_for_final_global_blocker():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.planning_validation_mode = 'enforce'
    fleet.validation_warnings = []
    fleet.planning_blocker = None
    published = []
    fleet.publish_state = lambda: published.append(True)

    assert not fleet._validation_allows(
        False, 'PARK_ROTATION_SPACE_BLOCKED', 'PLAN_PATH')
    assert fleet.planning_blocker is None
    assert published == []


def test_warn_only_warnings_are_deduplicated_and_bounded():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.planning_validation_mode = 'warn_only'
    fleet.validation_warnings = []
    fleet.get_logger = lambda: SimpleNamespace(
        warn=lambda *args, **kwargs: None)

    for index in range(20):
        assert fleet._validation_allows(
            False, f'CHECK_{index}', 'PLAN_PATH')
    assert fleet._validation_allows(False, 'CHECK_19', 'PLAN_PATH')

    assert len(fleet.validation_warnings) == 16
    assert fleet.validation_warnings[0]['code'] == 'CHECK_4'
    assert fleet.validation_warnings[-1]['code'] == 'CHECK_19'


class RecordingPublisher:
    def __init__(self):
        self.messages = []

    def publish(self, message):
        self.messages.append(message)


class AdjustableClock:
    def __init__(self, nanoseconds):
        self.nanoseconds = nanoseconds

    def now(self):
        nanoseconds = self.nanoseconds
        return SimpleNamespace(
            nanoseconds=nanoseconds,
            to_msg=lambda: SimpleNamespace(
                sec=nanoseconds // 1_000_000_000,
                nanosec=nanoseconds % 1_000_000_000))


class RecordingPlanner:
    def __init__(self):
        self.map_geometry = None

    def set_map_geometry(self, resolution, origin_x_m, origin_y_m):
        self.map_geometry = (resolution, origin_x_m, origin_y_m)


class DirectPlanner:
    def __init__(self):
        self.footprint = None

    def set_footprint(self, half_length, half_width):
        self.footprint = (half_length, half_width)

    def plan(self, grid, width, height, start, goal):
        return [start, goal]


def test_fleet_uses_occupancy_grid_origin_for_world_planning():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.planner = RecordingPlanner()
    fleet.get_logger = lambda: SimpleNamespace(warn=lambda *args, **kwargs: None)
    message = SimpleNamespace(
        info=SimpleNamespace(
            width=96,
            height=77,
            resolution=0.05,
            origin=SimpleNamespace(
                position=SimpleNamespace(x=-0.40, y=0.0))),
        data=[0] * (96 * 77),
    )

    fleet.map_cb(message)

    assert fleet.grid_origin_x_m == -0.40
    assert fleet.grid_origin_y_m == 0.0
    assert fleet.planner.map_geometry == (0.05, -0.40, 0.0)


def occupied(direction='forward'):
    registry = ParkingRegistry(['A1'])
    registry.reserve_park(
        'A1', 'park-1', VEHICLE_NUMBER,
        ParkingCredential.create(PASSWORD))
    registry.complete_park('A1', 'park-1', POSE, direction, SPEC)
    return registry


def request_harness(registry):
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.registry = registry
    fleet.state = 'WAIT_TARGET'
    fleet.mission_id = ''
    fleet.mission_type = ''
    fleet.target_pose = None
    fleet.front_robot_state = 'IDLE'
    fleet.rear_robot_state = 'IDLE'
    fleet.front_motion_fault = ''
    fleet.rear_motion_fault = ''
    fleet.request_status = None
    fleet.ui_request_id = ''
    fleet.active_source_slot_id = ''
    fleet.destination_kind = ''
    fleet.active_parking_direction = ''
    fleet.active_vehicle_spec = None
    fleet.grid = [0]
    now = time.monotonic()
    fleet.front_odom = {
        'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'receipt': now,
    }
    fleet.rear_odom = {
        'x': 0.0, 'y': 0.7, 'yaw': 0.0, 'receipt': now,
    }
    fleet.odom_timeout = 10.0
    fleet.publish_count = 0
    fleet.publish_state = lambda: setattr(
        fleet, 'publish_count', fleet.publish_count + 1)
    fleet._retrieve_approach_preflight = lambda record: True
    fleet._apply_active_vehicle_spec = lambda: None
    fleet._publish_retrieve_target = lambda pose: pose
    fleet.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=123456789))
    fleet.get_logger = lambda: SimpleNamespace(
        info=lambda *args: None,
        warn=lambda *args, **kwargs: None)
    return fleet


def park_request_harness(registry=None):
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.registry = registry or ParkingRegistry(['A1', 'A2'])
    fleet.registered_slots = [
        SimpleNamespace(slot_id='A1'), SimpleNamespace(slot_id='A2')]
    fleet.empty_slots = list(fleet.registered_slots)
    fleet.state = 'WAIT_TARGET'
    fleet.mission_id = 'mission-park'
    fleet.mission_type = 'park'
    fleet.front_robot_state = 'IDLE'
    fleet.rear_robot_state = 'IDLE'
    fleet.front_motion_fault = ''
    fleet.rear_motion_fault = ''
    fleet.ui_park_approved = False
    fleet.ui_approved_time = 0.0
    fleet.ui_request_id = ''
    fleet.request_status = None
    fleet.requested_destination_slot_id = ''
    fleet.active_vehicle_number = ''
    fleet.active_parking_credential = None
    fleet.publish_state = lambda: None
    fleet.get_logger = lambda: SimpleNamespace(info=lambda *args: None)
    return fleet


def test_park_request_binds_vehicle_password_and_requested_empty_slot():
    fleet = park_request_harness()
    payload = {
        'type': 'park',
        'request_id': 'ui-park',
        'vehicle_number': '12가 3456',
        'password': '2468',
        'destination_slot_id': 'A2',
    }

    fleet._handle_park_request(payload)

    assert fleet.request_status['status'] == 'ACCEPTED'
    assert fleet.requested_destination_slot_id == 'A2'
    assert fleet.active_vehicle_number == normalize_vehicle_number('12가 3456')
    assert fleet.active_parking_credential.verify('2468')
    assert not hasattr(fleet, 'active_parking_password')
    assert 'password' not in fleet.request_status


def test_park_request_rejects_duplicate_vehicle_or_unavailable_slot():
    registry = occupied()
    fleet = park_request_harness(registry)
    fleet._handle_park_request({
        'type': 'park', 'request_id': 'duplicate',
        'vehicle_number': VEHICLE_NUMBER, 'password': '1357',
        'destination_slot_id': 'A2',
    })
    assert fleet.request_status['reason'] == 'VEHICLE_ALREADY_PARKED'

    fleet = park_request_harness()
    fleet.empty_slots = [SimpleNamespace(slot_id='A1')]
    fleet._handle_park_request({
        'type': 'park', 'request_id': 'unavailable',
        'vehicle_number': '34나5678', 'password': '1357',
        'destination_slot_id': 'A2',
    })
    assert fleet.request_status['reason'] == 'DESTINATION_SLOT_UNAVAILABLE'


def test_park_planning_candidates_are_limited_to_requested_empty_slot():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.registry = ParkingRegistry(['A1', 'A2'])
    fleet.empty_slots = [
        SimpleNamespace(slot_id='A1'), SimpleNamespace(slot_id='A2')]
    fleet.requested_destination_slot_id = 'A2'

    candidates = FleetManagerNode._eligible_park_slots(fleet)

    assert [slot.slot_id for slot in candidates] == ['A2']


def park_plan_harness(validation_mode):
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    slot = RegisteredSlot('P2', 2.0, 2.2, 1.2, 0.8, math.pi / 2.0)
    fleet.mission_type = 'park'
    fleet.mission_id = 'park-warn-only'
    fleet.grid_w = 120
    fleet.grid_h = 100
    fleet.resolution = 0.05
    fleet.grid = [0] * (fleet.grid_w * fleet.grid_h)
    fleet.grid_origin_x_m = 0.0
    fleet.grid_origin_y_m = 0.0
    fleet.empty_slots = [slot]
    fleet.registered_slots = [slot]
    fleet.registry = ParkingRegistry(['P2'])
    fleet.requested_destination_slot_id = 'P2'
    fleet.current_virtual_start = lambda: Pose2D(0.6, 0.4, math.pi)
    fleet.target_pose = None
    fleet.current_wheelbase = SPEC['wheelbase']
    fleet.robot_length = 0.565
    fleet.robot_width = 0.275
    fleet.vehicle_length = SPEC['vehicle_length_m']
    fleet.vehicle_width = SPEC['vehicle_width_m']
    fleet.footprint_margin = 0.06
    fleet.vehicle_center_offset_body = [0.0, 0.0]
    fleet.loaded_footprint = compute_loaded_footprint(
        fleet.current_wheelbase,
        fleet.robot_length,
        fleet.robot_width,
        fleet.vehicle_length,
        fleet.vehicle_width,
        fleet.footprint_margin)
    fleet.slot_fit_long_margin = 0.06
    fleet.slot_fit_lat_margin = 0.06
    fleet.slot_staging_gap = 0.10
    fleet.use_staged_slot_entry = True
    fleet.parking_direction = 'forward'
    fleet.initial_target_offset_gate = 0.50
    fleet.planner = DirectPlanner()
    fleet._rotation_space_free = lambda x, y: True
    fleet._insertion_corridor_free = lambda start, goal: True
    fleet.active_vehicle_spec = dict(SPEC)
    fleet.active_vehicle_number = VEHICLE_NUMBER
    fleet.active_parking_credential = ParkingCredential.create(PASSWORD)
    fleet.active_destination_slot_id = ''
    fleet.destination_kind = ''
    fleet.active_parking_direction = ''
    fleet.path_published = False
    fleet.active_plan_stamp_ns = 0
    fleet.pub_slot_pose = RecordingPublisher()
    fleet.pub_waypoints = RecordingPublisher()
    fleet.publish_state = lambda: None
    fleet.get_clock = lambda: AdjustableClock(1_000_000_000)
    fleet.get_logger = lambda: SimpleNamespace(
        info=lambda *args, **kwargs: None,
        warn=lambda *args, **kwargs: None,
        error=lambda *args, **kwargs: None)
    fleet.planning_validation_mode = validation_mode
    fleet.validation_warnings = []
    return fleet


def test_warn_only_park_keeps_too_short_slot_and_publishes_existing_path():
    fleet = park_plan_harness('warn_only')

    planned = fleet.plan_and_publish()

    assert planned
    assert fleet.active_destination_slot_id == 'P2'
    assert len(fleet.pub_waypoints.messages) == 1
    assert fleet.validation_warnings == [
        {
            'code': 'SLOT_TOO_SHORT',
            'mission_phase': 'PLAN_PATH',
        }
    ]


def test_enforce_park_rejects_too_short_slot_with_visible_blocker():
    fleet = park_plan_harness('enforce')

    planned = fleet.plan_and_publish()

    assert not planned
    assert fleet.pub_waypoints.messages == []
    assert fleet.planning_blocker == {
        'code': 'PARK_SLOT_FIT_BLOCKED',
        'mission_phase': 'PLAN_PATH',
    }


def test_warn_only_park_reports_rotation_and_insertion_but_publishes():
    fleet = park_plan_harness('warn_only')
    # Isolate this slice from the already-covered slot-fit warning.
    fleet.registered_slots[0] = RegisteredSlot(
        'P2', 2.0, 2.2, 2.0, 0.8, math.pi / 2.0)
    fleet.empty_slots = list(fleet.registered_slots)
    fleet._rotation_space_free = lambda x, y: False
    fleet._insertion_corridor_free = lambda start, goal: False

    planned = fleet.plan_and_publish()

    assert planned
    assert [warning['code'] for warning in fleet.validation_warnings] == [
        'PARK_ROTATION_SPACE_BLOCKED',
        'PARK_INSERTION_CORRIDOR_BLOCKED',
    ]


def retrieve_plan_harness(validation_mode):
    fleet = park_plan_harness(validation_mode)
    slot = RegisteredSlot('P2', 2.0, 2.2, 1.2, 0.8, math.pi / 2.0)
    registry = ParkingRegistry(['P2'])
    registry.reserve_park(
        'P2', 'park-1', VEHICLE_NUMBER,
        ParkingCredential.create(PASSWORD))
    registry.complete_park(
        'P2', 'park-1', Pose2D(2.0, 2.2, math.pi / 2.0),
        'forward', SPEC)
    registry.reserve_retrieve('P2', 'retrieve-warn-only')
    fleet.registry = registry
    fleet.registered_slots = [slot]
    fleet.mission_type = 'retrieve'
    fleet.mission_id = 'retrieve-warn-only'
    fleet.active_source_slot_id = 'P2'
    fleet.rigid_body_lookahead = 0.15
    fleet.source_vehicle_fallback_mask = 0.90
    fleet.unknown_is_occupied = True
    fleet.wait_x = 0.6
    fleet.wait_y = 0.4
    fleet.wait_yaw = math.pi
    fleet.pub_slot_pose = RecordingPublisher()
    fleet.pub_waypoints = RecordingPublisher()
    fleet.validation_warnings = []
    return fleet


def test_warn_only_retrieve_reports_corridors_but_publishes(monkeypatch):
    fleet = retrieve_plan_harness('warn_only')
    fleet._rotation_space_free = lambda x, y: False
    monkeypatch.setattr(
        fleet_manager_module, 'corridor_is_free',
        lambda *args, **kwargs: False)

    planned = fleet.plan_retrieve_and_publish()

    assert planned
    assert len(fleet.pub_waypoints.messages) == 1
    assert [warning['code'] for warning in fleet.validation_warnings] == [
        'RETRIEVE_EXTRACTION_CORRIDOR_BLOCKED',
        'WAITING_ROTATION_SPACE_BLOCKED',
        'WAITING_INSERTION_CORRIDOR_BLOCKED',
    ]


def test_warn_only_still_refuses_missing_astar_path_and_reports_blocker():
    fleet = park_plan_harness('warn_only')
    fleet.planner.plan = lambda *args, **kwargs: None

    planned = fleet.plan_and_publish()

    assert not planned
    assert fleet.pub_waypoints.messages == []
    assert fleet.planning_blocker == {
        'code': 'ASTAR_NO_PATH',
        'mission_phase': 'PLAN_PATH',
    }


def request(slot_id='A1', request_id='ui-1'):
    return {
        'type': 'retrieve',
        'source_slot_id': slot_id,
        'request_id': request_id,
    }


def authenticated_request(
        vehicle_number=VEHICLE_NUMBER, password=PASSWORD,
        request_id='ui-credential'):
    return {
        'type': 'retrieve',
        'vehicle_number': vehicle_number,
        'password': password,
        'request_id': request_id,
    }


def test_retrieve_request_resolves_slot_by_vehicle_number_and_password():
    fleet = request_harness(occupied())

    fleet._handle_retrieve_request(authenticated_request('12가 3456'))

    assert fleet.request_status == {
        'request_id': 'ui-credential',
        'type': 'retrieve',
        'source_slot_id': 'A1',
        'destination_slot_id': '',
        'status': 'ACCEPTED',
        'reason': '',
    }
    assert fleet.active_source_slot_id == 'A1'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED


def test_retrieve_request_hides_vehicle_lookup_and_password_failures():
    fleet = request_harness(occupied())

    fleet._handle_retrieve_request(authenticated_request(password='0000'))
    assert fleet.request_status['status'] == 'REJECTED'
    assert fleet.request_status['reason'] == 'VEHICLE_OR_PASSWORD_INVALID'
    assert 'password' not in fleet.request_status
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED


def test_source_slot_only_request_cannot_bypass_vehicle_password():
    fleet = request_harness(occupied())

    fleet._handle_retrieve_request(request())

    assert fleet.request_status['status'] == 'REJECTED'
    assert fleet.request_status['reason'] == 'VEHICLE_OR_PASSWORD_INVALID'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED


def test_retrieve_approach_preflight_is_blocking_in_enforce_mode():
    fleet = request_harness(occupied())
    fleet.planning_validation_mode = 'enforce'
    fleet.validation_warnings = []
    fleet._retrieve_approach_preflight = lambda record: False

    fleet._handle_retrieve_request(authenticated_request())

    assert fleet.request_status['status'] == 'REJECTED'
    assert fleet.request_status['reason'] == 'APPROACH_CORRIDOR_BLOCKED'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED


def test_retrieve_approach_preflight_warn_only_starts_mission_with_warning():
    fleet = request_harness(occupied())
    fleet.planning_validation_mode = 'warn_only'
    fleet.validation_warnings = []
    fleet._retrieve_approach_preflight = lambda record: False

    fleet._handle_retrieve_request(authenticated_request())

    assert fleet.request_status['status'] == 'ACCEPTED'
    assert fleet.state == 'WAIT_LIFT'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED
    assert fleet.validation_warnings == [
        {
            'code': 'APPROACH_CORRIDOR_BLOCKED',
            'mission_phase': 'REQUEST',
        }
    ]


def test_warn_only_retrieve_rejects_missing_map_before_model_preflight():
    fleet = request_harness(occupied())
    fleet.planning_validation_mode = 'warn_only'
    fleet.validation_warnings = []
    fleet.grid = None

    fleet._handle_retrieve_request(authenticated_request())

    assert fleet.request_status['status'] == 'REJECTED'
    assert fleet.request_status['reason'] == 'MAP_MISSING'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED
    assert fleet.validation_warnings == []


def test_warn_only_retrieve_rejects_missing_odom_before_model_preflight():
    fleet = request_harness(occupied())
    fleet.planning_validation_mode = 'warn_only'
    fleet.validation_warnings = []
    fleet.front_odom = None

    fleet._handle_retrieve_request(authenticated_request())

    assert fleet.request_status['status'] == 'REJECTED'
    assert fleet.request_status['reason'] == 'ODOM_MISSING_OR_STALE'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED
    assert fleet.validation_warnings == []


def test_retrieve_request_accepts_only_complete_forward_occupied_record():
    fleet = request_harness(occupied())

    fleet._handle_retrieve_request(authenticated_request())

    assert fleet.request_status['status'] == 'ACCEPTED'
    assert fleet.state == 'WAIT_LIFT'
    assert fleet.mission_type == 'retrieve'
    assert fleet.active_source_slot_id == 'A1'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED
    assert fleet.target_pose == POSE
    assert fleet.publish_count == 1


def test_wait_lift_refreshes_retrieve_target_for_sequential_rear():
    fleet = request_harness(occupied())
    clock = AdjustableClock(1_000_000_000)
    fleet.get_clock = lambda: clock
    fleet.pub_target_pose = RecordingPublisher()
    fleet.pub_vehicle_spec = RecordingPublisher()
    fleet._publish_retrieve_target = (
        lambda pose: FleetManagerNode._publish_retrieve_target(fleet, pose))
    fleet.car_lifted = False

    fleet._handle_retrieve_request(authenticated_request())
    assert len(fleet.pub_target_pose.messages) == 1
    assert fleet.pub_target_pose.messages[-1].header.stamp.sec == 1

    # Rear는 Front-first 접근에서 2초 이상 WAIT_FRONT_STAGED에 머물 수 있다.
    # Fleet의 WAIT_LIFT tick은 Registry target을 새 stamp로 유지해야 한다.
    clock.nanoseconds = 4_000_000_000
    FleetManagerNode.manage_loop(fleet)

    assert len(fleet.pub_target_pose.messages) == 2
    assert fleet.pub_target_pose.messages[-1].header.stamp.sec == 4
    spec = json.loads(fleet.pub_vehicle_spec.messages[-1].data)
    assert spec['stamp_ns'] == 4_000_000_000


def test_retrieve_request_rejects_unknown_or_empty_registry_identity():
    fleet = request_harness(ParkingRegistry(['A1']))
    fleet._handle_retrieve_request(authenticated_request())
    assert fleet.request_status['reason'] == 'VEHICLE_OR_PASSWORD_INVALID'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EMPTY


def test_retrieve_request_rejects_unsupported_or_missing_record():
    fleet = request_harness(occupied('reverse'))
    fleet._handle_retrieve_request(authenticated_request())
    assert fleet.request_status['reason'] == 'UNSUPPORTED_PARKING_DIRECTION'

    fleet = request_harness(occupied())
    incomplete = SimpleNamespace(
        slot_id='A1',
        lifecycle=SlotLifecycle.OCCUPIED,
        parking_direction='forward',
        final_vehicle_pose=None,
        vehicle_spec=None)
    fleet.registry = SimpleNamespace(
        authenticate_vehicle=lambda vehicle_number, password: incomplete)
    fleet._handle_retrieve_request(authenticated_request())
    assert fleet.request_status['reason'] == 'MISSING_VEHICLE_RECORD'


def commit_harness():
    fleet = SimpleNamespace(
        registry=occupied(),
        mission_id='retrieve-1',
        mission_type='retrieve',
        active_source_slot_id='A1',
        destination_kind='WAITING',
        active_committed_stages=set(),
        last_commit_sequence=-1,
        pending_completion=None,
        publish_state=lambda: None,
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        get_logger=lambda: SimpleNamespace(error=lambda *args: None),
    )
    fleet.registry.reserve_retrieve('A1', 'retrieve-1')
    return fleet


def emit_commit(fleet, stage, sequence, mission_id='retrieve-1'):
    message = SimpleNamespace(data=json.dumps({
        'mission_id': mission_id,
        'role': 'front',
        'stage': stage,
        'sequence': sequence,
        'stamp_ns': 1_000_000_000,
    }))
    FleetManagerNode.mission_commit_cb(fleet, message)


def test_retrieve_drive_and_return_commits_advance_and_clear_registry():
    fleet = commit_harness()
    emit_commit(fleet, 'DRIVE', 1)
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXITING

    emit_commit(fleet, 'RELEASE', 2)
    emit_commit(fleet, 'RETURN', 3)
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EMPTY


def test_wrong_mission_commit_cannot_change_retrieve_lifecycle():
    fleet = commit_harness()
    emit_commit(fleet, 'DRIVE', 1, mission_id='other')
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED


def test_robot_arrival_requires_valid_map_frame_final_vehicle_pose():
    robot = RobotStateMachineNode.__new__(RobotStateMachineNode)
    robot.state = 'DRIVE'
    robot.active_plan_stamp_ns = 123
    robot.arrived = False
    invalid = make_arrival_status(1.5, 3.0, math.pi / 2.0, 123)
    invalid['final_vehicle_pose']['frame_id'] = 'odom'

    RobotStateMachineNode.sync_cb(
        robot, SimpleNamespace(data=json.dumps(invalid)))
    assert not robot.arrived

    valid = make_arrival_status(1.5, 3.0, math.pi / 2.0, 123)
    RobotStateMachineNode.sync_cb(
        robot, SimpleNamespace(data=json.dumps(valid)))
    assert robot.arrived


def test_fleet_cannot_finalize_park_before_registry_is_occupied():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park('A1', 'park-1')
    fleet = SimpleNamespace(
        registry=registry,
        mission_id='park-1',
        mission_type='park',
        active_source_slot_id='',
        active_destination_slot_id='A1',
        active_committed_stages={'HOME'},
        pending_completion=None,
        finalized=False,
        _finalize_mission=lambda payload: setattr(
            fleet, 'finalized', True),
        get_clock=lambda: SimpleNamespace(
            now=lambda: SimpleNamespace(nanoseconds=1_000_000_000)),
        get_logger=lambda: SimpleNamespace(error=lambda *args: None),
    )
    message = SimpleNamespace(data=json.dumps({
        'mission_id': 'park-1',
        'stamp_ns': 1_000_000_000,
    }))

    FleetManagerNode.mission_complete_cb(fleet, message)

    assert not fleet.finalized
    assert registry.lifecycle('A1') is SlotLifecycle.RESERVED


def test_late_arrived_pose_completes_park_after_return_commit():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.registry = ParkingRegistry(['A1'])
    fleet.registry.reserve_park('A1', 'park-1')
    fleet.state = 'NAVIGATING'
    fleet.mission_id = 'park-1'
    fleet.mission_type = 'park'
    fleet.active_source_slot_id = ''
    fleet.active_destination_slot_id = 'A1'
    fleet.destination_kind = 'PARKING_SLOT'
    fleet.active_parking_direction = 'forward'
    fleet.active_vehicle_spec = dict(SPEC)
    fleet.active_plan_stamp_ns = 123
    fleet.pending_final_vehicle_pose = None
    fleet.active_committed_stages = {'RELEASE'}
    fleet.last_commit_sequence = 1
    fleet.pending_completion = None
    fleet.publish_state = lambda: None
    fleet.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    fleet.get_logger = lambda: SimpleNamespace(
        info=lambda *args: None, error=lambda *args: None)

    emit_commit(fleet, 'RETURN', 2, mission_id='park-1')
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.RESERVED

    arrival = make_arrival_status(POSE.x_m, POSE.y_m, POSE.yaw_rad, 123)
    FleetManagerNode.sync_status_cb(
        fleet, SimpleNamespace(data=json.dumps(arrival)))

    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED
    assert fleet.registry.get('A1').final_vehicle_pose == POSE


def test_demo_p1_p4_preflight_uses_front_first_clearance_policy():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.grid_w = 120
    fleet.grid_h = 80
    fleet.resolution = 0.05
    fleet.grid = [0] * (fleet.grid_w * fleet.grid_h)
    fleet.odom_timeout = 1.0
    now = time.monotonic()
    fleet.front_odom = {
        'x': 3.60, 'y': 0.60, 'yaw': math.pi, 'receipt': now}
    fleet.rear_odom = {
        'x': 3.60, 'y': 0.20, 'yaw': math.pi, 'receipt': now}
    fleet.entry_standoff = 0.85
    fleet.robot_length = 0.565
    fleet.robot_width = 0.275
    fleet.unknown_is_occupied = True
    fleet.approach_speed = 0.035
    fleet.approach_yaw_gain = 1.5
    fleet.approach_max_yaw_rate = 0.15
    fleet.minimum_inter_robot_gap = 0.10

    records = [SimpleNamespace(
        final_vehicle_pose=Pose2D(x, 2.20, math.pi / 2.0),
        vehicle_spec=SPEC) for x in (1.20, 2.00, 2.80, 3.60)]

    fleet.simultaneous_entry = True
    assert not any(fleet._retrieve_approach_preflight(r) for r in records)
    fleet.simultaneous_entry = False
    assert all(fleet._retrieve_approach_preflight(r) for r in records)


def test_complete_park_home_ui_retrieve_home_cycle():
    fleet = FleetManagerNode.__new__(FleetManagerNode)
    fleet.registry = ParkingRegistry(['A1'])
    credential = ParkingCredential.create(PASSWORD)
    fleet.registry.reserve_park(
        'A1', 'park-1', VEHICLE_NUMBER, credential)
    fleet.state = 'NAVIGATING'
    fleet.mission_id = 'park-1'
    fleet.mission_type = 'park'
    fleet.active_source_slot_id = ''
    fleet.active_destination_slot_id = 'A1'
    fleet.requested_destination_slot_id = 'A1'
    fleet.destination_kind = 'PARKING_SLOT'
    fleet.active_parking_direction = 'forward'
    fleet.active_vehicle_spec = dict(SPEC)
    fleet.active_vehicle_number = VEHICLE_NUMBER
    fleet.active_parking_credential = credential
    fleet.active_plan_stamp_ns = 123
    fleet.pending_final_vehicle_pose = None
    fleet.active_committed_stages = set()
    fleet.last_commit_sequence = -1
    fleet.pending_completion = None
    fleet.completion_sequence = 0
    fleet.last_completed = None
    fleet.request_status = {
        'request_id': 'ui-park', 'status': 'ACCEPTED'}
    fleet.ui_request_id = 'ui-park'
    fleet.car_lifted = True
    fleet.target_pose = object()
    fleet.empty_slots = []
    fleet.path_published = True
    fleet.ui_park_approved = False
    fleet.front_robot_state = 'IDLE'
    fleet.rear_robot_state = 'IDLE'
    fleet.front_motion_fault = ''
    fleet.rear_motion_fault = ''
    fleet.current_wheelbase = SPEC['wheelbase']
    fleet.robot_length = 0.565
    fleet.robot_width = 0.275
    fleet.vehicle_length = SPEC['vehicle_length_m']
    fleet.vehicle_width = SPEC['vehicle_width_m']
    fleet.footprint_margin = 0.06
    fleet.vehicle_center_offset_body = [0.0, 0.0]
    fleet.loaded_footprint = compute_loaded_footprint(
        fleet.current_wheelbase, fleet.robot_length, fleet.robot_width,
        fleet.vehicle_length, fleet.vehicle_width, fleet.footprint_margin)
    fleet.planner = SimpleNamespace(set_footprint=lambda *args: None)
    fleet.target_gate = SimpleNamespace(reset=lambda: None)
    fleet.spec_gate = SimpleNamespace(reset=lambda: None)
    fleet.publish_state = lambda: None
    fleet.get_clock = lambda: SimpleNamespace(
        now=lambda: SimpleNamespace(nanoseconds=1_000_000_000))
    fleet.get_logger = lambda: SimpleNamespace(
        info=lambda *args: None, error=lambda *args: None,
        warn=lambda *args, **kwargs: None)

    arrival = make_arrival_status(POSE.x_m, POSE.y_m, POSE.yaw_rad, 123)
    FleetManagerNode.sync_status_cb(
        fleet, SimpleNamespace(data=json.dumps(arrival)))
    emit_commit(fleet, 'RELEASE', 1, mission_id='park-1')
    emit_commit(fleet, 'RETURN', 2, mission_id='park-1')
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED
    emit_commit(fleet, 'HOME', 3, mission_id='park-1')
    FleetManagerNode.mission_complete_cb(fleet, SimpleNamespace(data=json.dumps({
        'mission_id': 'park-1', 'stamp_ns': 1_000_000_000})))
    assert fleet.state == 'WAIT_TARGET'
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.OCCUPIED
    assert fleet.registry.authenticate_vehicle(
        VEHICLE_NUMBER, PASSWORD).slot_id == 'A1'

    fleet.grid = [0]
    odom_receipt = time.monotonic()
    fleet.front_odom = {
        'x': 0.0, 'y': 0.0, 'yaw': 0.0, 'receipt': odom_receipt,
    }
    fleet.rear_odom = {
        'x': 0.0, 'y': 0.7, 'yaw': 0.0, 'receipt': odom_receipt,
    }
    fleet.odom_timeout = 10.0
    fleet._retrieve_approach_preflight = lambda record: True
    fleet._apply_active_vehicle_spec = lambda: None
    fleet._publish_retrieve_target = lambda pose: pose
    fleet._handle_retrieve_request(authenticated_request(
        request_id='ui-retrieve'))
    retrieve_id = fleet.mission_id
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED
    emit_commit(fleet, 'DRIVE', 1, mission_id=retrieve_id)
    emit_commit(fleet, 'RELEASE', 2, mission_id=retrieve_id)
    emit_commit(fleet, 'RETURN', 3, mission_id=retrieve_id)
    assert fleet.registry.lifecycle('A1') is SlotLifecycle.EMPTY
    emit_commit(fleet, 'HOME', 4, mission_id=retrieve_id)
    FleetManagerNode.mission_complete_cb(fleet, SimpleNamespace(data=json.dumps({
        'mission_id': retrieve_id, 'stamp_ns': 1_000_000_000})))

    assert fleet.state == 'WAIT_TARGET'
    assert fleet.last_completed['mission_type'] == 'retrieve'
    assert fleet.last_completed['source_slot_id'] == 'A1'
    assert fleet.last_completed['completion_sequence'] == 2
