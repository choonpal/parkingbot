#ifndef PARKING_ROBOT_FIRMWARE_H
#define PARKING_ROBOT_FIRMWARE_H

#ifdef __cplusplus
extern "C" {
#endif

/* 두 실차가 mirror 구조라 서보 안전 범위가 다르다.
 * CubeIDE의 C/C++ Build > Settings > MCU GCC Compiler > Preprocessor에서
 * PARKING_ROBOT_PROFILE을 1(front/robot-2) 또는 2(rear/robot-1)로 지정한다. */
#define PARKING_ROBOT_PROFILE_FRONT 1
#define PARKING_ROBOT_PROFILE_REAR  2

#ifndef PARKING_ROBOT_PROFILE
#define PARKING_ROBOT_PROFILE PARKING_ROBOT_PROFILE_FRONT
#endif

void Robot_Init(void);
void Robot_MainLoop(void);

#ifdef __cplusplus
}
#endif

#endif /* PARKING_ROBOT_FIRMWARE_H */
