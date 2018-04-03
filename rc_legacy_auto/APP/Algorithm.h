#ifndef __ALGORITHM_H
#define __ALGORITHM_H

#include "Includes.h"

#define PI (3.14159265358979323846f)

#define WHEEL_RADIUS           (0.075f)          // 轮子半径（米）
#define SIDE_LENGTH            (0.914f)          // 三角边长（米）
#define REDUCTION_RATIO        (19)              // 减速比

#define SUBWHEEL_DIAMETER      (50.7f)           // 码盘从动轮直径（毫米）

#define GY53_A_STATIC_DIST     (0.434f - 0.12f)  // 左前侧GY53传感器距离车X轴距离
#define GY53_B_STATIC_DIST     (0.434f - 0.09f)  // 右前侧GY53传感器距离车X轴距离
#define GY53_A_B_GAP           (0.234f)          // 两颗GY53的间距

#define DT35_H_STATIC_DIST     (0.0f)            // 头部DT35传感器距离车X轴距离
#define DT35_F_STATIC_DIST     (0.0f)            // 左前侧DT35传感器距离车Y轴距离
#define DT35_R_STATIC_DIST     (0.0f)            // 左后侧DT35传感器距离车Y轴距离
#define DT35_F_R_GAP           (0.0f)            // 两颗DT35的间距

#define PHOTO_SENSOR_X_DIST    (0.39f)           // X轴光电传感器至过机器中心Y轴的距离
#define PHOTO_SENSOR_Y_DIST    (0.26f)           // Y轴光电传感器至过机器中心X轴的距离

#define clamp(x, lower, upper) (x >= upper ? upper : (x <= lower ? lower : x))

#define w2v(w) ((w) * SIDE_LENGTH / sqrt(3))
#define v2jv(v) ((v) / (2 * PI * WHEEL_RADIUS) * REDUCTION_RATIO * 2000)

#define rad2deg(rad) ((rad) / PI * 180)
#define deg2rad(deg) ((deg) * PI / 180)

#define Real2ElmoHead ((int)(v2jv(- VelX * cos(deg2rad(PosZ)) - VelY * sin(deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))
#define Real2ElmoLeft ((int)(v2jv(VelX * sin(PI / 6 + deg2rad(PosZ)) - VelY * cos(PI / 6 + deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))
#define Real2ElmoRight ((int)(v2jv(VelX * sin(PI / 6 - deg2rad(PosZ)) + VelY * cos(PI / 6 - deg2rad(PosZ)) + w2v(deg2rad(VelZ)))))

#define AbsEncoder2Ang(val) (1.0f * (val) / 4096 * 360)

#define Real2EncoderX (PosX / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2EncoderY (PosY / (SUBWHEEL_DIAMETER * PI) * 2000 * 1000)
#define Real2GyroZ (PosZ)

#define Encoder2RealX(x) ((x) * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Encoder2RealY(y) ((y) * (SUBWHEEL_DIAMETER * PI) / 2000 / 1000)
#define Gyro2RealZ(z) (fmod(z, 360))

#define DeltaPos(dx, dy) (sqrt((dx) * (dx) + (dy) * (dy)))
#define DeltaAng(dz) (dz < -180 ? dz + 360 : (dz >= 180 ? dz - 360 : dz))

#define GY532RealY ((GY53A + GY53B) / 2.0f * cos(deg2rad(PosZ)))
#define GY532RealZ (rad2deg(atan2(GY53A - GY53B, GY53_A_B_GAP)))

#define DT502RealX (DT50 * cos(deg2rad(PosZ)))

#define DT352RealX ((DT35F + DT35R) / 2.0f * cos(deg2rad(PosZ)))
#define DT352RealY (DT35H * cos(deg2rad(PosZ)))
#define DT352RealZ (rad2deg(atan2(DT35R - DT35F, DT35_F_R_GAP)))

#define PhotoSensorXOffset (- PHOTO_SENSOR_X_DIST * (1 - cos(deg2rad(PosZ))))
#define PhotoSensorYOffset (- PHOTO_SENSOR_Y_DIST * (1 - cos(deg2rad(PosZ))))

#endif
