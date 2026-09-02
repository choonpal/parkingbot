#!/usr/bin/env python3
"""
==================================================
rigid_body_kinematics.py
==================================================
가상 강체 기구학: 중심점 속도 → Front/Rear 속도 분배.
두 로봇이 차량을 들고 하나의 강체처럼 움직일 때 사용.
"""

import math


class RigidBodyKinematics:
    def __init__(self, wheelbase=0.70):
        self.set_wheelbase(wheelbase)

    def set_wheelbase(self, wheelbase):
        '''Update the measured axle-to-axle separation for this mission.'''
        wheelbase = float(wheelbase)
        if not math.isfinite(wheelbase) or wheelbase <= 0.0:
            raise ValueError('wheelbase must be finite and positive')
        self.wheelbase = wheelbase
        self.half_L = wheelbase / 2.0

    def virtual_pose(self, front, rear):
        """Return midpoint and Front-Rear position-line geometry.

        ``theta`` is the angle of the line joining the two robot positions.
        It is useful for formation diagnostics, but it is not necessarily the
        transported vehicle heading: a relative lateral displacement rotates
        this line even if both robot headings remain unchanged. Global path and
        yaw control should use :meth:`transport_pose`.
        """
        cx = (front['x'] + rear['x']) / 2
        cy = (front['y'] + rear['y']) / 2
        dx = front['x'] - rear['x']
        dy = front['y'] - rear['y']
        theta = math.atan2(dy, dx)
        return cx, cy, theta

    @staticmethod
    def circular_mean_yaw(first_yaw, second_yaw):
        """Return the wrap-safe mean of two finite robot headings.

        The rigid pair should never have antipodal headings. If such invalid
        geometry is presented, return the first normalized heading
        deterministically; the independent relative-yaw guard remains the
        authority for stopping unsafe formation states.
        """
        first_yaw = float(first_yaw)
        second_yaw = float(second_yaw)
        if not all(math.isfinite(value) for value in
                   (first_yaw, second_yaw)):
            raise ValueError('robot headings must be finite')
        sin_sum = math.sin(first_yaw) + math.sin(second_yaw)
        cos_sum = math.cos(first_yaw) + math.cos(second_yaw)
        if math.hypot(sin_sum, cos_sum) < 1.0e-9:
            return math.atan2(math.sin(first_yaw), math.cos(first_yaw))
        return math.atan2(sin_sum, cos_sum)

    def transport_pose(self, front, rear):
        """Return pair midpoint with heading from the two robot odom yaws."""
        cx = (front['x'] + rear['x']) / 2
        cy = (front['y'] + rear['y']) / 2
        yaw = self.circular_mean_yaw(front['theta'], rear['theta'])
        return cx, cy, yaw

    def encoder_distance(self, front, rear):
        """엔코더 기반 두 로봇 거리"""
        dx = front['x'] - rear['x']
        dy = front['y'] - rear['y']
        return math.hypot(dx, dy)

    @classmethod
    def relative_pose_in_rear_frame(cls, front, rear):
        """Return Front centre displacement expressed in ``rear_base``.

        The returned lateral component follows ROS body axes: positive is to
        the Rear robot's left. This is the same convention as ID0
        ``/sync/relative_pose.position.y``.
        """
        dx = front['x'] - rear['x']
        dy = front['y'] - rear['y']
        longitudinal, lateral = cls.world_offset_to_body(
            dx, dy, rear['theta'])
        relative_yaw = math.atan2(
            math.sin(front['theta'] - rear['theta']),
            math.cos(front['theta'] - rear['theta']))
        return longitudinal, lateral, relative_yaw

    @staticmethod
    def apply_relative_correction(front_velocity, rear_velocity,
                                  corr_x, corr_y, corr_yaw):
        """Apply symmetric Front/Rear corrections in their common body axes."""
        return (
            (front_velocity[0] - 0.5 * corr_x,
             front_velocity[1] - 0.5 * corr_y,
             front_velocity[2] - 0.5 * corr_yaw),
            (rear_velocity[0] + 0.5 * corr_x,
             rear_velocity[1] + 0.5 * corr_y,
             rear_velocity[2] + 0.5 * corr_yaw),
        )

    def split(self, vx, vy, omega):
        """
        중심점 속도 → Front/Rear 속도 분배
        회전 시 앞은 +ω×L/2, 뒤는 -ω×L/2 횡속도 보정
        """
        front_vel = (vx, vy + omega * self.half_L, omega)
        rear_vel = (vx, vy - omega * self.half_L, omega)
        return front_vel, rear_vel

    @staticmethod
    def body_offset_to_world(offset_x, offset_y, yaw):
        """차량 body-frame offset을 map-frame 벡터로 회전한다."""
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * offset_x - s * offset_y,
                s * offset_x + c * offset_y)

    @staticmethod
    def world_offset_to_body(offset_x, offset_y, yaw):
        """map-frame offset을 차량 body-frame으로 회전한다."""
        c, s = math.cos(yaw), math.sin(yaw)
        return (c * offset_x + s * offset_y,
                -s * offset_x + c * offset_y)

    @classmethod
    def control_point_pose(cls, centre_x, centre_y, yaw,
                           offset_body_x, offset_body_y):
        """로봇 중점에서 body offset된 차량 제어점 pose를 구한다."""
        dx, dy = cls.body_offset_to_world(
            offset_body_x, offset_body_y, yaw)
        return centre_x + dx, centre_y + dy, yaw

    @staticmethod
    def control_point_twist_to_centre(
            vx, vy, omega, offset_body_x, offset_body_y):
        """차량 제어점 명령을 Front/Rear 중점 명령으로 바꾼다.

        차량 중심이 로봇 두 대 중점에서 ``d=(dx,dy)``만큼 떨어져
        있으면 ``v_vehicle = v_pair + omega x d``다. 따라서 제자리
        회전 시에도 차량 중심이 표류하지 않게 반대 평행속도를 더한다.
        """
        return (vx + omega * offset_body_y,
                vy - omega * offset_body_x,
                omega)

    @staticmethod
    def limit_twist_pair(front_command, rear_command,
                         linear_limit, angular_limit):
        """두 로봇 명령을 *같은 비율*로 축소한다.

        Front와 Rear를 각각 따로 제한하면 한쪽 선속도만 잘리면서
        ``v = omega x r`` 강체 관계가 깨진다. 특히 차량 중심이 로봇
        중점에서 벗어난 상태로 제자리 회전할 때 차량 중심이 옆으로
        밀릴 수 있다. 아래 방식은 여섯 성분 전체에 하나의 scale을
        적용하므로 궤적 형상과 차량 중심 보상을 그대로 보존한다.
        """
        linear_limit = max(0.0, float(linear_limit))
        angular_limit = max(0.0, float(angular_limit))
        commands = (tuple(front_command), tuple(rear_command))

        max_planar = max(math.hypot(cmd[0], cmd[1]) for cmd in commands)
        max_angular = max(abs(cmd[2]) for cmd in commands)
        scale = 1.0
        if max_planar > linear_limit:
            scale = min(scale, (linear_limit / max_planar)
                        if max_planar > 0.0 else 1.0)
        if max_angular > angular_limit:
            scale = min(scale, (angular_limit / max_angular)
                        if max_angular > 0.0 else 1.0)

        return tuple(
            tuple(value * scale for value in command)
            for command in commands)
