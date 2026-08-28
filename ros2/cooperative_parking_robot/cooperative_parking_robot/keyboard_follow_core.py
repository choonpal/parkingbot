"""Compatibility names for the former ``keyboard_follow`` implementation.

New code must import :mod:`rigid_pair_teleop_core`: its model is a virtual
rigid pair-centre, not a leader/follower controller.  These aliases preserve
existing scripts and tests without maintaining a second safety implementation.
"""

from cooperative_parking_robot.rigid_pair_teleop_core import (
    ZERO_COMMAND,
    RigidPairDecision as FollowDecision,
    RigidPairTeleopLimits as KeyboardFollowLimits,
    angle_norm,
    capture_pair_reference as capture_aruco_reference,
    clamp,
    evaluate_rigid_pair as evaluate_follow,
    is_zero,
    median_relative_pose,
    OdomPathAccumulator,
    relative_pose_is_stable,
    split_pair_centre_twist as follow_pair_commands,
)

__all__ = [
    'ZERO_COMMAND', 'KeyboardFollowLimits', 'FollowDecision', 'angle_norm',
    'capture_aruco_reference', 'clamp', 'evaluate_follow',
    'follow_pair_commands', 'is_zero', 'median_relative_pose',
    'OdomPathAccumulator', 'relative_pose_is_stable',
]
