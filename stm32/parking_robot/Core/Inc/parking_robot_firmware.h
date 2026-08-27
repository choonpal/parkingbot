#ifndef PARKING_ROBOT_FIRMWARE_H
#define PARKING_ROBOT_FIRMWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 두 실차가 mirror 구조라 서보 안전 범위와 뒤쪽 encoder mapping이 다르다.
 * Production binary에는 반드시 profile을 명시한다. 무지정 build를 Front로
 * 간주하면 같은 image를 Rear에 flash하는 사고를 compiler가 막을 수 없다. */
#define PARKING_ROBOT_PROFILE_FRONT 1
#define PARKING_ROBOT_PROFILE_REAR  2

#ifndef PARKING_ROBOT_PROFILE
#error "PARKING_ROBOT_PROFILE must be explicitly defined as FRONT(1) or REAR(2)"
#endif

#if PARKING_ROBOT_PROFILE != PARKING_ROBOT_PROFILE_FRONT && \
    PARKING_ROBOT_PROFILE != PARKING_ROBOT_PROFILE_REAR
#error "PARKING_ROBOT_PROFILE must be FRONT(1) or REAR(2)"
#endif

void Robot_Init(void);
void Robot_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* PARKING_ROBOT_FIRMWARE_H */
