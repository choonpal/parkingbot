"""물리 로봇의 실측 축 설정을 논리 ROS 역할과 분리한다.

``role``은 front/rear 토픽 namespace를 고르고, ``hardware_profile``은
실제로 연결된 RPi/STM32 기체의 배선 방향을 고른다. 한 기체를 다른 역할로
시험하더라도 실측 축 보정이 역할과 함께 뒤집히지 않아야 한다.
"""


COMMAND_SIGN_BY_PROFILE = {
    # ROS REP-103 (x forward, y left, yaw CCW) -> 각 기체 STM32 속도축.
    'robot-1': (-1.0, 1.0, 1.0),
    # 2026-08-25 cam2 map 기준 바닥 pulse 실측: ROS +y/+yaw는 STM32
    # +vy/+omega로 전달해야 한다. 예전 front 역할값 -1은 오차를 키웠다.
    'robot-2': (-1.0, 1.0, 1.0),
}

DEFAULT_PROFILE_BY_ROLE = {
    'front': 'robot-2',
    'rear': 'robot-1',
}


def resolve_hardware_profile(role, requested='auto'):
    """명시한 프로필을 검증하거나 현재 실차 기본 배치를 사용한다."""
    if role not in DEFAULT_PROFILE_BY_ROLE:
        raise ValueError("role must be 'front' or 'rear'")
    value = str(requested).strip().lower()
    if value in ('', 'auto'):
        return DEFAULT_PROFILE_BY_ROLE[role]
    if value not in COMMAND_SIGN_BY_PROFILE:
        raise ValueError(
            "hardware_profile must be 'auto', 'robot-1', or 'robot-2'")
    return value


def command_sign_for(profile):
    """ROS 명령축과 STM32/encoder 축 사이의 부호 변환을 반환한다."""
    try:
        return COMMAND_SIGN_BY_PROFILE[profile]
    except KeyError as exc:
        raise ValueError(f'unknown hardware profile: {profile}') from exc
