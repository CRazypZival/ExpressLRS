#include <Arduino.h>
#include "RC.h"
#include "logging.h"  // 添加日志支持
#include "crsf_protocol.h"  // 用于CRSF值转换
#include "ACCGYRO.h"

ACCGYRO accgyro;

// CRSF通道数量（ch0-ch15 = 16个通道）
#define CRSF_NUM_CHANNELS 16

// 外部声明ChannelData（定义在common.cpp中）
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];

// RC通道数组定义（全局变量）
int RC_CHANNEL[10] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

void rc_init() {
    accgyro.begin();
}

void channel_update() {
    // 读取姿态数据
    float roll_val = accgyro.roll();
    float pitch_val = accgyro.pitch();
    float yaw_val = accgyro.yaw();

    // 映射函数：将角度值(-45~45)映射到RC通道值(900~2100)
    auto map_angle_to_channel = [](float angle) -> int {
        // 角度范围：-45 ~ +45
        const float angle_min = -45.0f;
        const float angle_max = 45.0f;
        // 通道范围：900 ~ 2100
        const int channel_min = CHANNEL_MIN;
        const int channel_max = CHANNEL_MAX;

        // 限制角度在有效范围内
        if (angle < angle_min) angle = angle_min;
        if (angle > angle_max) angle = angle_max;

        // 线性映射
        float normalized = (angle - angle_min) / (angle_max - angle_min);
        int channel_value = channel_min + (int)((channel_max - channel_min) * normalized);

        return channel_value;
    };

    // 更新RC通道
    RC_CHANNEL[0] = map_angle_to_channel(roll_val);   // 横滚
    RC_CHANNEL[1] = map_angle_to_channel(pitch_val);  // 俯仰
    RC_CHANNEL[3] = map_angle_to_channel(yaw_val);    // 偏航

    // 其他通道保持最低值(900)
    RC_CHANNEL[2] = CHANNEL_MIN;  // Throttle
    RC_CHANNEL[4] = CHANNEL_MIN;  // AUX1
    RC_CHANNEL[5] = CHANNEL_MIN;  // AUX2
    RC_CHANNEL[6] = CHANNEL_MIN;  // AUX3
    RC_CHANNEL[7] = CHANNEL_MIN;  // AUX4
    RC_CHANNEL[8] = CHANNEL_MIN;  // AUX5
    RC_CHANNEL[9] = CHANNEL_MIN;  // AUX6
}

void rc_gyro_fast_update() {
    // 高频更新：读取原始数据 + 姿态融合
    accgyro.read();
    accgyro.posture();
}

void rc_gyro_slow_update(uint32_t nowMs) {
    static uint32_t lastPrintMs = 0;
    if (nowMs - lastPrintMs >= 250) {
        lastPrintMs = nowMs;
        accgyro.posture_print();
    }
}

