import pytest

from cooperative_parking_robot.parking_geometry import Pose2D
from cooperative_parking_robot.parking_registry import (
    ParkingCredential,
    ParkingRegistry,
    RegistryTransitionError,
    SlotLifecycle,
    normalize_vehicle_number,
)


SPEC = {
    'wheelbase': 0.70,
    'vehicle_length_m': 0.90,
    'vehicle_width_m': 0.35,
}
POSE = Pose2D(1.5, 3.0, 1.5707963267948966)


def test_registry_binds_vehicle_number_and_verifies_password_without_exposure():
    registry = ParkingRegistry(['A1'])
    credential = ParkingCredential.create('2468')

    registry.reserve_park(
        'A1', 'park-1',
        vehicle_number='12가 3456', credential=credential)
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)

    record = registry.get('A1')
    assert record.vehicle_number == normalize_vehicle_number('12가 3456')
    assert record.credential is not None
    assert record.credential.verify('2468')
    assert not record.credential.verify('0000')
    assert '2468' not in repr(record)
    assert 'vehicle_number' not in registry.summaries()[0]
    assert 'credential' not in registry.summaries()[0]


def test_registry_rejects_duplicate_normalized_vehicle_number():
    registry = ParkingRegistry(['A1', 'A2'])
    registry.reserve_park(
        'A1', 'park-1', '12가 3456', ParkingCredential.create('2468'))
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)

    with pytest.raises(RegistryTransitionError):
        registry.reserve_park(
            'A2', 'park-2', '12가3456', ParkingCredential.create('1357'))


def test_registry_authenticates_only_matching_vehicle_and_password():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park(
        'A1', 'park-1', '12가3456', ParkingCredential.create('2468'))
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)

    assert registry.authenticate_vehicle('12가 3456', '2468').slot_id == 'A1'
    assert registry.authenticate_vehicle('12가3456', '0000') is None
    assert registry.authenticate_vehicle('99나9999', '2468') is None


def test_password_verifiers_use_unique_salts_and_reject_short_passwords():
    first = ParkingCredential.create('2468')
    second = ParkingCredential.create('2468')
    assert first.salt != second.salt
    assert first.digest != second.digest
    with pytest.raises(ValueError):
        ParkingCredential.create('123')


def occupied_registry():
    registry = ParkingRegistry(['A1', 'A2'])
    registry.reserve_park(
        'A1', 'park-1', '12가3456', ParkingCredential.create('2468'))
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)
    return registry


def test_registry_starts_every_registered_slot_empty():
    registry = ParkingRegistry(['A1', 'A2'])
    assert registry.lifecycle('A1') is SlotLifecycle.EMPTY
    assert registry.lifecycle('A2') is SlotLifecycle.EMPTY
    assert registry.empty_slot_ids() == ('A1', 'A2')


def test_park_lifecycle_preserves_vehicle_record_after_active_reset():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park('A1', 'park-1')
    assert registry.lifecycle('A1') is SlotLifecycle.RESERVED
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)

    active_mission = {'mission_id': 'park-1'}
    active_mission.clear()
    record = registry.get('A1')
    assert record.lifecycle is SlotLifecycle.OCCUPIED
    assert record.final_vehicle_pose == POSE
    assert record.parking_direction == 'forward'
    assert record.vehicle_spec == SPEC


def test_retrieve_lifecycle_clears_record_only_after_release():
    registry = occupied_registry()
    selected = registry.reserve_retrieve('A1', 'retrieve-1')
    assert selected.final_vehicle_pose == POSE
    assert registry.lifecycle('A1') is SlotLifecycle.EXIT_RESERVED

    registry.mark_retrieve_exiting('A1', 'retrieve-1')
    assert registry.lifecycle('A1') is SlotLifecycle.EXITING
    assert registry.get('A1').final_vehicle_pose == POSE

    registry.complete_retrieve('A1', 'retrieve-1')
    record = registry.get('A1')
    assert record.lifecycle is SlotLifecycle.EMPTY
    assert record.final_vehicle_pose is None
    assert record.vehicle_spec is None


@pytest.mark.parametrize(
    ('operation', 'slot_id', 'mission_id'),
    [
        ('complete_park', 'A1', 'wrong-mission'),
        ('reserve_retrieve', 'A2', 'retrieve-1'),
        ('mark_retrieve_exiting', 'A1', 'wrong-mission'),
        ('complete_retrieve', 'A1', 'wrong-mission'),
    ],
)
def test_invalid_lifecycle_or_mission_binding_is_rejected(
        operation, slot_id, mission_id):
    registry = ParkingRegistry(['A1', 'A2'])
    registry.reserve_park('A1', 'park-1')
    if operation in ('mark_retrieve_exiting', 'complete_retrieve'):
        registry.complete_park(
            'A1', 'park-1', POSE, 'forward', SPEC)
        registry.reserve_retrieve('A1', 'retrieve-1')
        if operation == 'complete_retrieve':
            registry.mark_retrieve_exiting('A1', 'retrieve-1')

    with pytest.raises(RegistryTransitionError):
        if operation == 'complete_park':
            registry.complete_park(
                slot_id, mission_id, POSE, 'forward', SPEC)
        else:
            getattr(registry, operation)(slot_id, mission_id)


def test_ui_summary_hides_internal_record_and_marks_only_forward_record():
    registry = occupied_registry()
    registry.reserve_park(
        'A2', 'park-2', '34나5678', ParkingCredential.create('1357'))
    registry.complete_park(
        'A2', 'park-2', Pose2D(2.5, 3.0, -1.57), 'reverse', SPEC)

    summary = registry.summaries()
    assert summary == [
        {'slot_id': 'A1', 'lifecycle': 'OCCUPIED', 'retrievable': True},
        {'slot_id': 'A2', 'lifecycle': 'OCCUPIED', 'retrievable': False},
    ]
    assert 'final_vehicle_pose' not in summary[0]
    assert 'vehicle_spec' not in summary[0]
    assert 'parking_direction' not in summary[0]


def test_only_same_unpublished_park_reservation_can_roll_back():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park('A1', 'park-1')
    with pytest.raises(RegistryTransitionError):
        registry.rollback_unpublished_park('A1', 'park-other')
    registry.rollback_unpublished_park('A1', 'park-1')
    assert registry.lifecycle('A1') is SlotLifecycle.EMPTY


def test_complete_demo_park_home_retrieve_home_cycle_keeps_session_truth():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park(
        'A1', 'park-1', '12가3456', ParkingCredential.create('2468'))
    registry.complete_park(
        'A1', 'park-1', POSE, 'forward', SPEC)
    assert registry.summaries()[0]['retrievable']

    selected = registry.reserve_retrieve('A1', 'retrieve-2')
    assert selected.parked_by_mission_id == 'park-1'
    registry.mark_retrieve_exiting('A1', 'retrieve-2')
    registry.complete_retrieve('A1', 'retrieve-2')
    assert registry.lifecycle('A1') is SlotLifecycle.EMPTY


def test_registry_normalizes_final_vehicle_yaw():
    registry = ParkingRegistry(['A1'])
    registry.reserve_park('A1', 'park-1')
    registry.complete_park(
        'A1', 'park-1', Pose2D(1.5, 3.0, 4.0), 'forward', SPEC)

    assert -3.141592653589793 <= (
        registry.get('A1').final_vehicle_pose.yaw_rad
    ) <= 3.141592653589793
