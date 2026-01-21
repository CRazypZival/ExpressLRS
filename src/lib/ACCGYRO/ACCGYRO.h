#ifndef ACCGYRO_H
#define ACCGYRO_H

#include "config.h"
#include <Arduino.h>
#include <Wire.h>
#include <MPU6050.h>
#include "logging.h"
#include "RC.h"

class ACCGYRO {
public:
    ACCGYRO();
    void begin();  // 初始化 I2C 和 MPU6050
    void setAlign(int alignDeg); // 设置陀螺仪对齐角度 (0, 90, 180, 270)
    void read();   // 读取一次数据并通过 DBG() 打印
    void posture(); //根据读取到的传感器数据进行计算姿态
    void posture_print();
    float roll() const { return m_rollDeg; }
    float pitch() const { return m_pitchDeg; }
    float yaw() const { return m_yawDeg; }

private:
    MPU6050 mpu;
    int16_t m_ax = 0, m_ay = 0, m_az = 0;
    int16_t m_gx = 0, m_gy = 0, m_gz = 0;
    bool m_hasReading = false;
    unsigned long m_prevMicros = 0;
    float m_rollDeg = 0.0f;
    float m_pitchDeg = 0.0f;
    float m_yawDeg = 0.0f;
    int m_alignDeg = 0; // 对齐角度 (0, 90, 180, 270)
    static constexpr float GYRO_SENS_500DPS = 65.5f; // LSB/deg/s for 500 dps full scale
    static constexpr float COMPLEMENTARY_ALPHA = 0.98f;
    static constexpr float YAW_OFFSET_LSB = 165.0f; // YAW轴偏移补偿，基于调试数据计算得出
};

#endif