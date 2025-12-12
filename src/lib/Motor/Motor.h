#ifndef MOTOR_H
#define MOTOR_H

#include <Arduino.h>
#include <SimpleFOC.h>
#include "RC.h"

// PID参数结构体
struct PIDParams {
    float velocity_P;
    float velocity_I;
    float velocity_D;
    float angle_P;
    float velocity_limit;
    float angle_limit;
};

// 电压限制参数结构体
struct VoltageLimitParams {
    float motor_voltage_limit;
    float sensor_align_voltage;
    float current_limit;
};

// 电机控制范围参数结构体
struct MotorControlRangeParams {
    float center_angle;              // 中心角度，默认0度
    float span_zone;                 // 活动范围，默认150度
    float transition_zone;           // 过渡区域，默认10度
    float voltage_inzone;            // 活动区域电压，默认0.5V
    float voltage_outzone;           // 外部区域电压，默认3.0V
    float transition_curve_ratio;    // 过渡曲线比例，默认0.3 (前段占30%，后段占70%)
};

// 电机状态结构体
struct MotorStatus {
    bool enabled;
    bool angle_control_mode;
    float target_angle;
    float current_angle;
    float shaft_velocity;
    float output_voltage;
};

class MotorController {
public:
    // 构造函数
    MotorController();

    // 初始化电机系统
    void init();

    // 主循环调用
    void loop();

    // ===== 电机控制 =====
    void enable();
    void disable();
    void enterAngleControlMode();
    void exitAngleControlMode();
    void setTargetAngle(float angle_deg);

    // ===== 外部可访问的参数 =====
    // PID参数 (直接访问)
    PIDParams pid_params;

    // 电压限制参数 (直接访问)
    VoltageLimitParams voltage_params;

    // 电机控制范围参数 (直接访问)
    MotorControlRangeParams control_range_params;

    // 电机状态 (只读)
    const MotorStatus& getStatus() const { return motor_status; }

    // ===== 便捷设置方法 =====
    void setVelocityPID(float p, float i, float d);
    void setAnglePID(float p);
    void setVoltageLimit(float voltage);
    void setVoltageAlign(float voltage);
    void setAngleLimit(float limit);
    void setVelocityLimit(float limit);
    void setCenterAngle(float center_deg);
    void setSpanZone(float span_zone_deg);
    void setTransitionZone(float transition_zone_deg);
    void setVoltageInzone(float voltage);
    void setVoltageOutzone(float voltage);
    void setTransitionCurveRatio(float ratio);

    // ===== 状态查询 =====
    bool isEnabled() const { return motor_status.enabled; }
    bool isAngleControlMode() const { return motor_status.angle_control_mode; }
    float getCurrentAngle() const { return motor_status.current_angle; }
    float getTargetAngle() const { return motor_status.target_angle; }
    float getShaftVelocity() const { return motor_status.shaft_velocity; }
    float getOutputVoltage() const { return motor_status.output_voltage; }

    // ===== 调试和维护 =====
    void showStatus();
    void handlePIDCommand(String command);

private:
    // 电机硬件对象
    MagneticSensorI2C sensor;
    BLDCDriver3PWM driver;
    BLDCMotor motor;

    // 电机状态 (内部维护)
    MotorStatus motor_status;

    // 根据角度范围动态调整电压限制
    void updateVoltageLimitByAngle();

    // 更新状态信息
    void updateStatus();
};

#endif // MOTOR_H
