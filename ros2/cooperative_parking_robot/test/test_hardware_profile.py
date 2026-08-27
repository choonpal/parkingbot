from pathlib import Path

import pytest

from cooperative_parking_robot.hardware_profile import (
    command_sign_for,
    resolve_hardware_profile,
)


ROOT = Path(__file__).resolve().parents[1]


def test_auto_profile_preserves_normal_front_rear_assignment():
    assert resolve_hardware_profile('front') == 'robot-2'
    assert resolve_hardware_profile('rear') == 'robot-1'


def test_profile_is_independent_of_logical_role():
    profile = resolve_hardware_profile('rear', 'robot-2')
    assert profile == 'robot-2'
    assert command_sign_for(profile) == (-1.0, 1.0, 1.0)


@pytest.mark.parametrize('role', ('front', 'rear'))
def test_both_measured_profiles_keep_positive_lateral_and_yaw(role):
    profile = resolve_hardware_profile(role)
    assert command_sign_for(profile) == (-1.0, 1.0, 1.0)


def test_unknown_hardware_profile_is_rejected():
    with pytest.raises(ValueError):
        resolve_hardware_profile('rear', 'robot-3')
    with pytest.raises(ValueError):
        resolve_hardware_profile('middle')
    with pytest.raises(ValueError):
        command_sign_for('robot-3')


@pytest.mark.parametrize(
    ('filename', 'argument', 'default'),
    (
        ('front_robot.launch.py', 'hardware_profile', 'robot-2'),
        ('rear_robot.launch.py', 'hardware_profile', 'robot-1'),
        ('cooperative_drive_test_front.launch.py',
         'hardware_profile', 'robot-2'),
        ('cooperative_drive_test_rear.launch.py',
         'hardware_profile', 'robot-1'),
        ('full_system.launch.py', 'front_hardware_profile', 'robot-2'),
        ('full_system.launch.py', 'rear_hardware_profile', 'robot-1'),
    ),
)
def test_real_launches_expose_physical_profile(filename, argument, default):
    source = (ROOT / 'launch' / filename).read_text()
    compact = ''.join(source.split())
    assert f"'{argument}',default_value='{default}'" in compact or \
        f'"{argument}",default_value="{default}"' in compact
    assert f"LaunchConfiguration('{argument}')" in compact or \
        f'LaunchConfiguration("{argument}")' in compact
