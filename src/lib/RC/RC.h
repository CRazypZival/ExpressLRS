#ifndef RC_H
#define RC_H

#include <Arduino.h>

extern int RC_CHANNEL[10];

// 引脚定义
#define ACCGYRO_SCL 4
#define ACCGYRO_SDA 5

// 可调参数
#define ACCGYRO_YAW_ALIGN 180
#define ACCGYRO_DEADZONE 5
#define PITCH_RANGE 90
#define ROLL_RANGE 90
#define YAW_RANGE 90


// 通道值范围定义
#define CHANNEL_MAX 2100
#define CHANNEL_MID 1500
#define CHANNEL_MIN 900

void rc_init();

void channel_update();

// IMU 实时任务（在主循环中高频调用）
void rc_gyro_fast_update();

// 姿态打印任务（在主循环中调用，内部按 250ms 定时）
void rc_gyro_slow_update(uint32_t nowMs);

// 获取当前姿态角（单位：度），值来自 ACCGYRO 融合结果
void rc_gyro_get_angles(float &rollDeg, float &pitchDeg, float &yawDeg);

#endif // RC_H
