#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "Includes.h"

#define PI (3.14159265358979323846f)

#define WHEEL_RADIUS           (0.075f)          // 轮子半径（米）
#define SIDE_LENGTH            (0.914f)          // 三角边长（米）
#define REDUCTION_RATIO        (19)              // 减速比

#define SUBWHEEL_DIAMETER      (50.7f)           // 码盘从动轮直径（毫米）

#define DT35_X_STATIC_DIST     (0.460f)          // 左侧DT35传感器距离车Y轴距离
#define DT35_Y_STATIC_DIST     (0.490f)          // 后侧DT35传感器距离车X轴距离

#define PHOTO_SENSOR_X_DIST    (0.43f)           // X轴光电传感器至过机器中心Y轴的距离
#define PHOTO_SENSOR_Y_DIST    (0.17f)           // Y轴光电传感器至过机器中心X轴的距离

#define clamp(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

#define w2v(w) ((w) * SIDE_LENGTH / sqrt(3))
#define v2jv(v) ((v) / (2 * PI * WHEEL_RADIUS) * REDUCTION_RATIO * 2000)

#define rad2deg(rad) ((rad) / PI * 180)
#define deg2rad(deg) ((deg) * PI / 180)

#define Real2ElmoHead ((int)(v2jv(- VelX * cos(deg2rad(PosZ)) - VelY * sin(deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))
#define Real2ElmoLeft ((int)(v2jv(VelX * sin(PI / 6 + deg2rad(PosZ)) - VelY * cos(PI / 6 + deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))
#define Real2ElmoRight ((int)(v2jv(VelX * sin(PI / 6 - deg2rad(PosZ)) + VelY * cos(PI / 6 - deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))

#define Real2EncoderX (PosX / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2EncoderY (PosY / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2GyroZ (PosZ)

#define Encoder2RealX(x) ((x) * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Encoder2RealY(y) ((y) * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Gyro2RealZ(z) (fmod(z, 360))

#define DeltaPos(dx, dy) (sqrt((dx) * (dx) + (dy) * (dy)))
#define DeltaAng(dz) (dz < -180 ? dz + 360 : (dz >= 180 ? dz - 360 : dz))

#define DT352RealX (DT35X * cos(deg2rad(PosZ)))
#define DT352RealY (DT35Y * cos(deg2rad(PosZ)))

#define PhotoSensorXOffset (- PHOTO_SENSOR_X_DIST * (1 - cos(deg2rad(PosZ))))
#define PhotoSensorYOffset (- PHOTO_SENSOR_Y_DIST * (1 - cos(deg2rad(PosZ))))

#endif
