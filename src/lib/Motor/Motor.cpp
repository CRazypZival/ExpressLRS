#include "Motor.h"

MotorController::MotorController()
    : sensor(AS5600_I2C),
      driver(DRV8313_IN1, DRV8313_IN2, DRV8313_IN3, DRV8313_EN),
      motor(7) {  // 假设电机有7对极

    // 初始化电机状态
    motor_status.enabled = false;
    motor_status.angle_control_mode = false;
    motor_status.target_angle = 0;
    motor_status.current_angle = 0;
    motor_status.shaft_velocity = 0;
    motor_status.output_voltage = 0;

    // 初始化PID参数
    pid_params.velocity_P = 0.08;
    pid_params.velocity_I = 0.08;
    pid_params.velocity_D = 0.0;
    pid_params.angle_P = 50;
    pid_params.velocity_limit = 1.0;
    pid_params.angle_limit = 50;

    // 初始化电压限制参数
    voltage_params.motor_voltage_limit = 0.5;
    voltage_params.sensor_align_voltage = 5.0;
    voltage_params.current_limit = 1.0;

    // 初始化电机控制范围参数
    control_range_params.center_angle = 0.0;       // 中心角度，默认0度
    control_range_params.span_zone = 150.0;        // 活动范围，默认150度
    control_range_params.transition_zone = 10.0;   // 过渡区域，默认10度
    control_range_params.voltage_inzone = 0.5;     // 活动区域电压，默认0.5V
    control_range_params.voltage_outzone = 3.0;    // 外部区域电压，默认3.0V
    control_range_params.transition_curve_ratio = 0.3; // 过渡曲线比例，前段占30%
}

void MotorController::init() {
    Wire1.begin(AS5600_SDA, AS5600_SCL);
    Wire1.setClock(400000);  // 设置I2C时钟为400kHz

    // 初始化传感器 - 使用Wire1
    sensor.init(&Wire1);

    // 初始化驱动器
    driver.init();

    // 连接传感器和驱动器到电机
    motor.linkSensor(&sensor);
    motor.linkDriver(&driver);

    // 配置电机参数 (使用结构体参数)
    motor.voltage_limit = voltage_params.motor_voltage_limit;
    motor.voltage_sensor_align = voltage_params.sensor_align_voltage;
    motor.current_limit = voltage_params.current_limit;

    // 设置控制模式
    motor.controller = MotionControlType::angle;  // 角度控制
    motor.torque_controller = TorqueControlType::voltage;  // 电压力矩控制

    // 设置PID参数
    motor.P_angle.P = pid_params.angle_P;
    motor.PID_velocity.P = pid_params.velocity_P;
    motor.PID_velocity.I = pid_params.velocity_I;
    motor.PID_velocity.D = pid_params.velocity_D;

    // 设置PID输出限制
    motor.P_angle.limit = pid_params.angle_limit;
    motor.PID_velocity.limit = pid_params.velocity_limit;

    // 添加低通滤波减少震动
    motor.LPF_velocity.Tf = 0.01;      // 速度低通滤波时间常数
    motor.LPF_angle.Tf = 0.005;        // 角度低通滤波时间常数
    motor.PID_velocity.output_ramp = 1000;  // 限制输出变化率

    // 初始化电机
    motor.init();

    // 初始化FOC算法
    motor.initFOC();

    // 设置默认控制模式和目标角度
    enterAngleControlMode();
    setTargetAngle(0.0);

}

void MotorController::loop() {
    // 运行FOC算法
    motor.loopFOC();

    // 如果电机启用且在角度控制模式下，设置目标角度
    if (motor_status.enabled && motor_status.angle_control_mode) {
        motor.move(motor_status.target_angle);
    }

    // 根据角度范围动态调整电压限制
    updateVoltageLimitByAngle();

    // 更新状态信息
    updateStatus();
}

void MotorController::enable() {
    motor_status.enabled = true;
    motor.enable();
}

void MotorController::disable() {
    motor_status.enabled = false;
    motor.disable();
}

void MotorController::enterAngleControlMode() {
    motor_status.angle_control_mode = true;
    motor_status.enabled = true;
    motor.enable();
    motor.controller = MotionControlType::angle;
}

void MotorController::exitAngleControlMode() {
    motor_status.angle_control_mode = false;
    motor_status.enabled = false;
    motor.disable();
}

void MotorController::setTargetAngle(float angle_deg) {
    if (angle_deg >= 0 && angle_deg <= 360) {
        motor_status.target_angle = angle_deg * PI / 180.0; // 转换为弧度
    }
}


void MotorController::setVelocityPID(float p, float i, float d) {
    pid_params.velocity_P = p;
    pid_params.velocity_I = i;
    pid_params.velocity_D = d;
    motor.PID_velocity.P = p;
    motor.PID_velocity.I = i;
    motor.PID_velocity.D = d;
}

void MotorController::setAnglePID(float p) {
    pid_params.angle_P = p;
    motor.P_angle.P = p;
}

void MotorController::setVoltageLimit(float voltage) {
    voltage_params.motor_voltage_limit = voltage;
    motor.voltage_limit = voltage;
    // 同时更新PID输出限制，使其不超过电压限制
    if (pid_params.velocity_limit > voltage) {
        pid_params.velocity_limit = voltage;
        motor.PID_velocity.limit = voltage;
    }
}

void MotorController::setVoltageAlign(float voltage) {
    voltage_params.sensor_align_voltage = voltage;
    motor.voltage_sensor_align = voltage;
}

void MotorController::setAngleLimit(float limit) {
    pid_params.angle_limit = limit;
    motor.P_angle.limit = limit;
}

void MotorController::setVelocityLimit(float limit) {
    pid_params.velocity_limit = limit;
    motor.PID_velocity.limit = limit;
}

void MotorController::setCenterAngle(float center_deg) {
    control_range_params.center_angle = center_deg;
}

void MotorController::setSpanZone(float span_zone_deg) {
    control_range_params.span_zone = span_zone_deg;
}

void MotorController::setTransitionZone(float transition_zone_deg) {
    control_range_params.transition_zone = transition_zone_deg;
}

void MotorController::setVoltageInzone(float voltage) {
    control_range_params.voltage_inzone = voltage;
}

void MotorController::setVoltageOutzone(float voltage) {
    control_range_params.voltage_outzone = voltage;
}

void MotorController::setTransitionCurveRatio(float ratio) {
    // 限制在合理范围内 (0.1 到 0.9)
    if (ratio < 0.1) ratio = 0.1;
    if (ratio > 0.9) ratio = 0.9;
    control_range_params.transition_curve_ratio = ratio;
}

void MotorController::showStatus() {
    // 调试输出功能已被移除
}

void MotorController::handlePIDCommand(String command) {
    // 移除"set "前缀
    command = command.substring(4);
    command.trim();

    // 查找等号
    int equalPos = command.indexOf('=');
    if (equalPos == -1) {
        return;
    }

    String paramName = command.substring(0, equalPos);
    String valueStr = command.substring(equalPos + 1);

    paramName.trim();
    valueStr.trim();

    float value = valueStr.toFloat();

    // 处理不同的参数
    if (paramName == "p" || paramName == "P") {
        setVelocityPID(value, pid_params.velocity_I, pid_params.velocity_D);
    }
    else if (paramName == "i" || paramName == "I") {
        setVelocityPID(pid_params.velocity_P, value, pid_params.velocity_D);
    }
    else if (paramName == "d" || paramName == "D") {
        setVelocityPID(pid_params.velocity_P, pid_params.velocity_I, value);
    }
    else if (paramName == "angle_p" || paramName == "angle_P") {
        setAnglePID(value);
    }
    else if (paramName == "voltage_limit" || paramName == "vlim") {
        setVoltageLimit(value);
    }
    else if (paramName == "voltage_align" || paramName == "valign") {
        setVoltageAlign(value);
    }
    else if (paramName == "angle_limit" || paramName == "alim") {
        setAngleLimit(value);
    }
    else if (paramName == "velocity_limit" || paramName == "vlim_pid") {
        setVelocityLimit(value);
    }
    else if (paramName == "c" || paramName == "center") {
        setCenterAngle(value);
    }
    else if (paramName == "sz" || paramName == "span_zone") {
        setSpanZone(value);
    }
    else if (paramName == "tz" || paramName == "transition_zone") {
        setTransitionZone(value);
    }
    else if (paramName == "iv" || paramName == "voltage_inzone") {
        setVoltageInzone(value);
    }
    else if (paramName == "ov" || paramName == "voltage_outzone") {
        setVoltageOutzone(value);
    }
    else if (paramName == "tcr" || paramName == "transition_curve_ratio") {
        setTransitionCurveRatio(value);
    }
    else {
        // 未知参数，调试输出已被移除
    }
}

void MotorController::updateVoltageLimitByAngle() {
    if (!motor_status.angle_control_mode || !motor_status.enabled) {
        return;
    }

    // 计算目标角度和当前角度的角度差（考虑角度循环）
    float current_angle = const_cast<MagneticSensorI2C&>(sensor).getAngle();
    float angle_diff = motor_status.target_angle - current_angle;

    // 将角度差规范化到 [-PI, PI] 范围
    while (angle_diff > PI) angle_diff -= 2 * PI;
    while (angle_diff < -PI) angle_diff += 2 * PI;
    angle_diff = abs(angle_diff);  // 取绝对值

    // 将弧度转换为度数进行区域判断
    float angle_diff_deg = angle_diff * 180.0 / PI;

    // 计算各个区域的边界（度数）
    float span_boundary = control_range_params.span_zone / 2.0;         // 活动区域边界
    float transition_boundary = span_boundary + control_range_params.transition_zone;  // 过渡区域边界

    // 根据角度误差确定所在区域并计算电压限制
    float new_voltage_limit;

    if (angle_diff_deg <= span_boundary) {
        // 活动区域：|error| <= span_zone/2，统一使用 voltage_inzone
        new_voltage_limit = control_range_params.voltage_inzone;
    }
    else if (angle_diff_deg <= transition_boundary) {
        // 过渡区域：span_zone/2 < |error| <= span_zone/2 + transition_zone
        // 使用分段非线性过渡曲线：前段斜率低，后段斜率高
        float transition_progress = (angle_diff_deg - span_boundary) / control_range_params.transition_zone;

        // 计算非线性过渡
        float ratio = control_range_params.transition_curve_ratio;  // 前段比例
        float adjusted_progress;

        if (transition_progress <= ratio) {
            // 前段：平缓上升 (0-30%范围，0-20%进度)
            adjusted_progress = (transition_progress / ratio) * 0.2;
        } else {
            // 后段：陡峭上升 (30-100%范围，20-100%进度)
            adjusted_progress = 0.2 + ((transition_progress - ratio) / (1.0 - ratio)) * 0.8;
        }

        new_voltage_limit = control_range_params.voltage_inzone +
                           (control_range_params.voltage_outzone - control_range_params.voltage_inzone) * adjusted_progress;
    }
    else {
        // 外部区域：|error| > span_zone/2 + transition_zone
        new_voltage_limit = control_range_params.voltage_outzone;
    }

    // 对电压限制进行低通滤波，避免突变导致震动
    static float smoothed_voltage_limit = -1.0;
    const float voltage_filter_alpha = 0.05;  // 滤波系数，越小越平滑
    
    if (smoothed_voltage_limit < 0) {
        // 首次初始化
        smoothed_voltage_limit = new_voltage_limit;
    } else {
        // 低通滤波: smoothed = alpha * new + (1-alpha) * old
        smoothed_voltage_limit = voltage_filter_alpha * new_voltage_limit + 
                                 (1.0 - voltage_filter_alpha) * smoothed_voltage_limit;
    }

    // 只有当电压限制变化超过阈值时才更新
    static float last_applied_voltage = -1.0;
    if (abs(smoothed_voltage_limit - last_applied_voltage) > 0.005) {
        motor.voltage_limit = smoothed_voltage_limit;
        motor.PID_velocity.limit = smoothed_voltage_limit;
        last_applied_voltage = smoothed_voltage_limit;
    }
}

void MotorController::updateStatus() {
    // 更新实时状态信息
    motor_status.current_angle = const_cast<MagneticSensorI2C&>(sensor).getAngle();
    motor_status.shaft_velocity = motor.shaft_velocity;
    motor_status.output_voltage = motor.voltage.q;
}

