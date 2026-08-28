#!/usr/bin/env python3
"""Canonical entry point for virtual rigid-body keyboard teleoperation.

The implementation remains in ``keyboard_follow_node`` so the legacy
``keyboard_follow`` executable stays compatible.  It is a rigid-pair
controller: it never designates Front or Rear as a leader/follower.
"""

from cooperative_parking_robot.keyboard_follow_node import RigidPairTeleopNode


def main(args=None):
    """Run the canonical rigid-pair keyboard teleop node."""
    from cooperative_parking_robot.keyboard_follow_node import main as _main
    return _main(args=args)

__all__ = ['RigidPairTeleopNode', 'main']


if __name__ == '__main__':
    main()
