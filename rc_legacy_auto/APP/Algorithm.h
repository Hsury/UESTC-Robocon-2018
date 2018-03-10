#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "Includes.h"

#define PI (3.14159265358979323846f)

#define WHEEL_RADIUS       (0.075f) // 轮子半径（米）
#define SIDE_LENGTH        (0.914f) // 三角边长（米）
#define REDUCTION_RATIO    (19)     // 减速比

#define SUBWHEEL_DIAMETER  (50.7f)  // 码盘从动轮直径（毫米）

#define w2v(w) ((w) * SIDE_LENGTH / sqrt(3))
#define v2jv(v) ((v) / (2 * PI * WHEEL_RADIUS) * REDUCTION_RATIO * 2000)

#define rad2deg(rad) ((rad) / PI * 180)
#define deg2rad(deg) ((deg) * PI / 180)

#define Real2Elmo_Head ((int)(v2jv(- VelX * cos(deg2rad(AngZ)) - VelY * sin(deg2rad(AngZ)) + w2v(deg2rad(VelZ)))))
#define Real2Elmo_Left ((int)(v2jv(VelX * sin(PI / 6 + deg2rad(AngZ)) - VelY * cos(PI / 6 + deg2rad(AngZ)) + w2v(deg2rad(VelZ)))))
#define Real2Elmo_Right ((int)(v2jv(VelX * sin(PI / 6 - deg2rad(AngZ)) + VelY * cos(PI / 6 - deg2rad(AngZ)) + w2v(deg2rad(VelZ)))))

#define Real2Encoder_X (PosX / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2Encoder_Y (PosY / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2Gyro_Z (AngZ) //(AngZ / PI * 180)

#define Encoder2Real_X(x) (x * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Encoder2Real_Y(y) (y * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Gyro2Real_Z(z) (fmod(z, 360)) //(z * PI / 180)

#define DeltaAng(z) (z < -180 ? z + 360 : (z >= 180 ? z - 360 : z))

#endif
