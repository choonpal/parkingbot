from cooperative_parking_robot.mvp_runtime_nodes import (
    minimum_entry_side_offset,
    rigid_drive_owns_command,
)


def test_measured_vehicle_geometry_requires_0595_m_side_lane():
    assert minimum_entry_side_offset(0.310, 0.420, 0.060) == 0.595


def test_rigid_controller_owns_only_active_drive():
    common = dict(
        has_path=True,
        vehicle_lifted=True,
        front_ready=True,
        rear_ready=True,
        estop=False,
    )
    assert rigid_drive_owns_command(
        front_state='DRIVE', rear_state='DRIVE', **common)
    assert not rigid_drive_owns_command(
        front_state='APPROACH', rear_state='APPROACH', **common)
    assert not rigid_drive_owns_command(
        front_state='DRIVE', rear_state='ALIGN', **common)


def test_estop_never_grants_rigid_command_ownership():
    assert not rigid_drive_owns_command(
        has_path=True,
        vehicle_lifted=True,
        front_state='DRIVE',
        rear_state='DRIVE',
        front_ready=True,
        rear_ready=True,
        estop=True,
    )
