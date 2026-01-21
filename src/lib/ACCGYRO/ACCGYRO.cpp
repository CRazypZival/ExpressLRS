#include "ACCGYRO.h"
#include <math.h>

ACCGYRO::ACCGYRO() : mpu(MPU6050_DEFAULT_ADDRESS, &Wire1) {
    m_prevMicros = micros();
}

void ACCGYRO::begin() {
    Wire1.begin(ACCGYRO_SDA, ACCGYRO_SCL);

    Wire1.setClock(400000);

    mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_4);
    mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_500);
    mpu.initialize();

    bool ok = mpu.testConnection();
    if (!ok) {
        DBGLN("MPU6050 connection: FAILED");
    } else {
        DBGLN("MPU6050 connection: OK");
    }
}

void ACCGYRO::read() {
    mpu.getMotion6(&m_ax, &m_ay, &m_az, &m_gx, &m_gy, &m_gz);
    m_hasReading = true;
}

void ACCGYRO::posture() {
    if (!m_hasReading) {
        return;
    }

    // 计算时间间隔（秒）
    unsigned long now = micros();
    float dt = (now - m_prevMicros) / 1000000.0f;
    if (dt <= 0.0f) {
        return;
    }
    m_prevMicros = now;

    // 将陀螺仪原始值转换为角速度（deg/s）
    float gxDeg = static_cast<float>(m_gx) / GYRO_SENS_500DPS;
    float gyDeg = static_cast<float>(m_gy) / GYRO_SENS_500DPS;
    float gzDeg = static_cast<float>(m_gz) / GYRO_SENS_500DPS;

    // 积分陀螺仪角度
    m_rollDeg += gxDeg * dt;
    m_pitchDeg += gyDeg * dt;
    m_yawDeg += gzDeg * dt;

    // 加速度计计算俯仰、横滚（单位：度）
    float accRoll = atan2(static_cast<float>(m_ay), static_cast<float>(m_az)) * RAD_TO_DEG;
    float accPitch = atan2(-static_cast<float>(m_ax), sqrtf(static_cast<float>(m_ay) * m_ay + static_cast<float>(m_az) * m_az)) * RAD_TO_DEG;

    // 互补滤波融合
    m_rollDeg = COMPLEMENTARY_ALPHA * m_rollDeg + (1.0f - COMPLEMENTARY_ALPHA) * accRoll;
    m_pitchDeg = COMPLEMENTARY_ALPHA * m_pitchDeg + (1.0f - COMPLEMENTARY_ALPHA) * accPitch;

    // 限幅
    m_pitchDeg = constrain(m_pitchDeg, -90.0f, 90.0f);
    if (m_rollDeg > 180.0f) m_rollDeg -= 360.0f;
    if (m_rollDeg < -180.0f) m_rollDeg += 360.0f;

    // 偏航角包络到 [-180, 180]
    if (m_yawDeg > 180.0f || m_yawDeg < -180.0f) {
        m_yawDeg = fmodf(m_yawDeg + 180.0f, 360.0f);
        if (m_yawDeg < 0) m_yawDeg += 360.0f;
        m_yawDeg -= 180.0f;
    }
}

void ACCGYRO::posture_print(){
    DBGLN("Roll: %f, Pitch: %f, Yaw: %f", m_rollDeg, m_pitchDeg, m_yawDeg);
}