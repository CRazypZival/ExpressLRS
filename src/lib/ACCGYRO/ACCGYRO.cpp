#include "ACCGYRO.h"
#include <math.h>

ACCGYRO::ACCGYRO() : mpu(MPU6050_DEFAULT_ADDRESS, &Wire1) {
    m_prevMicros = micros();
    m_alignDeg = 0; // 默认对齐角度为0度
}

void ACCGYRO::setAlign(int alignDeg) {
    // 只接受0, 90, 180, 270度
    if (alignDeg == 0 || alignDeg == 90 || alignDeg == 180 || alignDeg == 270) {
        m_alignDeg = alignDeg;
        DBGLN("ACCGYRO align set to: %d degrees", m_alignDeg);
    } else {
        DBGLN("ACCGYRO align: invalid angle %d, keeping current %d", alignDeg, m_alignDeg);
    }
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

    // YAW轴补偿和反向处理（先补偿，再取反）
    m_gz = -(m_gz - YAW_OFFSET_LSB);

    // 根据对齐角度进行坐标变换
    if (m_alignDeg != 0) {
        int16_t ax_temp = m_ax, ay_temp = m_ay, az_temp = m_az;
        int16_t gx_temp = m_gx, gy_temp = m_gy, gz_temp = m_gz;

        switch (m_alignDeg) {
            case 90:  // 顺时针旋转90度: X=Y, Y=-X, Z=Z
                m_ax = ay_temp;
                m_ay = -ax_temp;
                m_az = az_temp;
                m_gx = gy_temp;
                m_gy = -gx_temp;
                m_gz = gz_temp;
                break;
            case 180:  // 旋转180度: X=-X, Y=-Y, Z=Z
                m_ax = -ax_temp;
                m_ay = -ay_temp;
                m_az = az_temp;
                m_gx = -gx_temp;
                m_gy = -gy_temp;
                m_gz = gz_temp;
                break;
            case 270:  // 逆时针旋转90度: X=-Y, Y=X, Z=Z
                m_ax = -ay_temp;
                m_ay = ax_temp;
                m_az = az_temp;
                m_gx = -gy_temp;
                m_gy = gx_temp;
                m_gz = gz_temp;
                break;
            default:  // 0度或其他无效值，无变换
                break;
        }
    }

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

    // 将陀螺仪原始值转换为角速度（deg/s），YAW轴的补偿已在read()中处理
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

    // 偏航角包络到 [0, 360]
    if (m_yawDeg >= 360.0f) {
        m_yawDeg = fmodf(m_yawDeg, 360.0f);
    } else if (m_yawDeg < 0.0f) {
        m_yawDeg = fmodf(m_yawDeg, 360.0f) + 360.0f;
    }
}

void ACCGYRO::posture_print(){
    DBGLN("Roll: %f, Pitch: %f, Yaw: %f", m_rollDeg, m_pitchDeg, m_yawDeg);
}