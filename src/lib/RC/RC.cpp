#include <Arduino.h>
#include "RC.h"
#include "logging.h"  // 添加日志支持
#include "crsf_protocol.h"  // 用于CRSF值转换
#include "ACCGYRO.h"

ACCGYRO accgyro;

// 外部声明ChannelData（定义在common.cpp中）
extern uint32_t ChannelData[CRSF_NUM_CHANNELS];


void rc_init() {
    accgyro.begin();
    accgyro.setAlign(ACCGYRO_YAW_ALIGN); // 设置陀螺仪对齐角度
}

void channel_update() {
    // 读取姿态数据
    float roll_val = accgyro.roll();
    float pitch_val = accgyro.pitch();
    float yaw_val = accgyro.yaw();

    // 映射函数：将角度差值映射到CRSF通道值范围（用于ROLL和PITCH）
    auto map_angle_to_crsf = [](float angle_diff, float range) -> uint32_t {
        // 计算角度范围：-range/2 ~ +range/2
        const float angle_min = -range / 2.0f;
        const float angle_max = range / 2.0f;

        // CRSF通道值范围
        const uint32_t crsf_min = CRSF_CHANNEL_VALUE_MIN;
        const uint32_t crsf_max = CRSF_CHANNEL_VALUE_MAX;

        // 限制角度差值在有效范围内
        if (angle_diff < angle_min) angle_diff = angle_min;
        if (angle_diff > angle_max) angle_diff = angle_max;

        // 线性映射到CRSF范围
        float normalized = (angle_diff - angle_min) / (angle_max - angle_min);
        uint32_t crsf_value = crsf_min + (uint32_t)((crsf_max - crsf_min) * normalized);

        return crsf_value;
    };

    // YAW轴特殊映射函数（考虑圆形角度0-360°）
    auto map_yaw_to_crsf = [](float yaw_angle, float mid_angle, float range) -> uint32_t {
        // YAW轴是圆形角度，需要特殊处理
        // 计算与mid_angle的角度差值（考虑圆形角度）
        float angle_diff = yaw_angle - mid_angle;

        // 将角度差值归一化到-180°到180°范围
        while (angle_diff > 180.0f) angle_diff -= 360.0f;
        while (angle_diff < -180.0f) angle_diff += 360.0f;

        // 对于YAW轴，range表示检测范围的一半
        // 例如：range=90时，检测范围是mid_angle ±45°
        const float angle_min = -range / 2.0f;
        const float angle_max = range / 2.0f;

        // CRSF通道值范围
        const uint32_t crsf_min = CRSF_CHANNEL_VALUE_MIN;
        const uint32_t crsf_max = CRSF_CHANNEL_VALUE_MAX;

        // 限制角度差值在有效范围内
        if (angle_diff < angle_min) angle_diff = angle_min;
        if (angle_diff > angle_max) angle_diff = angle_max;

        // 线性映射到CRSF范围
        float normalized = (angle_diff - angle_min) / (angle_max - angle_min);
        uint32_t crsf_value = crsf_min + (uint32_t)((crsf_max - crsf_min) * normalized);

        return crsf_value;
    };

    // 计算角度差值并映射到ChannelData
    ChannelData[0] = map_angle_to_crsf(roll_val - ROLL_MID, ROLL_RANGE);   // 横滚
    ChannelData[1] = map_angle_to_crsf(pitch_val - PITCH_MID, PITCH_RANGE); // 俯仰
    ChannelData[3] = map_yaw_to_crsf(yaw_val, YAW_MID, YAW_RANGE);          // 偏航（特殊处理）

    ChannelData[4] = CRSF_CHANNEL_VALUE_MIN;

    // 其他通道保持中间值(CRSF_CHANNEL_VALUE_MID)
    for (int i = 0; i < CRSF_NUM_CHANNELS; i++) {
        if (i != 0 && i != 1 && i != 3 && i != 4) {
            ChannelData[i] = CRSF_CHANNEL_VALUE_MID;
        }
    }
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

void rc_gyro_get_angles(float &rollDeg, float &pitchDeg, float &yawDeg)
{
    rollDeg = accgyro.roll();
    pitchDeg = accgyro.pitch();
    yawDeg = accgyro.yaw();
}
