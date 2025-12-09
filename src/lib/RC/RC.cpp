#include <Arduino.h>
#include "RC.h"
#include "logging.h"  // 添加日志支持

// RC通道数组定义（全局变量）
int RC_CHANNEL[10] = {1500, 1500, 900, 1500, 1500, 1500, 1500, 1500, 1500, 1500};

// ADC滤波器状态（每个通道独立）
static int filtered_values[4] = {2048, 2048, 2048, 2048};

/**
 * @brief 读取并处理ADC值
 * @param pin ADC引脚
 * @param channel_index 通道索引（用于滤波器）
 * @return 映射后的通道值 (CHANNEL_MIN 到 CHANNEL_MAX)
 */
int adc_get(int pin, int channel_index){
    int raw_value = analogRead(pin);
    
    // 应用低通滤波器 (7/8旧值 + 1/8新值)
    if (channel_index >= 0 && channel_index < 4) {
        filtered_values[channel_index] = (filtered_values[channel_index] * 7 + raw_value) / 8;
        raw_value = filtered_values[channel_index];
    }

    // 获取校准参数
    int max, mid, min;
    switch(pin) {
        case ROLL_ADC:
            max = ROLL_CALIBRATION_MAX;
            mid = ROLL_CALIBRATION_MID;
            min = ROLL_CALIBRATION_MIN;
            break;
        case PITCH_ADC:
            max = PITCH_CALIBRATION_MAX;
            mid = PITCH_CALIBRATION_MID;
            min = PITCH_CALIBRATION_MIN;
            break;
        case THROTTLE_ADC:
            max = THROTTLE_CALIBRATION_MAX;
            mid = THROTTLE_CALIBRATION_MID;
            min = THROTTLE_CALIBRATION_MIN;
            break;
        case YAW_ADC:
            max = YAW_CALIBRATION_MAX;
            mid = YAW_CALIBRATION_MID;
            min = YAW_CALIBRATION_MIN;
            break;
        case SLI10:
            max = SLIDE_CALIBRATION_MAX;
            mid = SLIDE_CALIBRATION_MID;
            min = SLIDE_CALIBRATION_MIN;
            break;
        default:
            max = 4095;
            mid = 2048;
            min = 0;
            break;
    }
    
    // 应用死区并映射到输出范围
    int channel_value;
    if (raw_value > mid + ADC_DEADZONE && raw_value <= max){
        channel_value = map(raw_value, mid + ADC_DEADZONE, max, CHANNEL_MID, CHANNEL_MAX);
    } else if (raw_value < mid - ADC_DEADZONE && raw_value >= min){
        channel_value = map(raw_value, min, mid - ADC_DEADZONE, CHANNEL_MIN, CHANNEL_MID);
    } else{
        channel_value = CHANNEL_MID;
    }
    
    return channel_value;
}

/**
 * @brief 更新所有RC通道值
 */
void channel_update(){
    // CH1-4: 摇杆ADC通道
    RC_CHANNEL[0] = adc_get(ROLL_ADC, 0);      // 副翼
    RC_CHANNEL[1] = adc_get(PITCH_ADC, 1);     // 升降
    RC_CHANNEL[2] = adc_get(THROTTLE_ADC, 2);  // 油门
    RC_CHANNEL[3] = adc_get(YAW_ADC, 3);       // 方向
    
    // CH5: 两段开关
    if (digitalRead(SW5) == HIGH){
        RC_CHANNEL[4] = CHANNEL_MAX;
    } else {
        RC_CHANNEL[4] = CHANNEL_MIN;
    }
    
    // CH6: 三段开关
    if (digitalRead(SW6_1) == HIGH && digitalRead(SW6_2) == LOW){
        RC_CHANNEL[5] = CHANNEL_MAX;
    } else if (digitalRead(SW6_1) == LOW && digitalRead(SW6_2) == HIGH){
        RC_CHANNEL[5] = CHANNEL_MIN;
    } else{
        RC_CHANNEL[5] = CHANNEL_MID;
    }
    
    // CH7: 三段开关
    if (digitalRead(SW7_1) == HIGH && digitalRead(SW7_2) == LOW){
        RC_CHANNEL[6] = CHANNEL_MAX;
    } else if (digitalRead(SW7_1) == LOW && digitalRead(SW7_2) == HIGH){
        RC_CHANNEL[6] = CHANNEL_MIN;
    } else{
        RC_CHANNEL[6] = CHANNEL_MID;
    }
    
    // CH8: 两段开关
    if (digitalRead(SW8) == HIGH){
        RC_CHANNEL[7] = CHANNEL_MAX;
    } else {
        RC_CHANNEL[7] = CHANNEL_MIN;
    }
    
    // CH9: 两段开关
    if (digitalRead(SW9) == HIGH){
        RC_CHANNEL[8] = CHANNEL_MAX;
    } else {
        RC_CHANNEL[8] = CHANNEL_MIN;
    }
    
    // CH10: 滑动开关/电位器
    RC_CHANNEL[9] = adc_get(SLI10, -1);  // -1表示不使用滤波器索引
    
    static uint32_t lastDebugTime = 0;
    if (millis() - lastDebugTime > 250) // 每秒输出一次
    {
        lastDebugTime = millis();
        DBGLN("=== RC通道值 ===");
        DBGLN("摇杆: 副翼=%d 升降=%d 油门=%d 方向=%d", 
              RC_CHANNEL[0], RC_CHANNEL[1], RC_CHANNEL[2], RC_CHANNEL[3]);
        DBGLN("开关: CH5=%d CH6=%d CH7=%d CH8=%d CH9=%d CH10=%d",
              RC_CHANNEL[4], RC_CHANNEL[5], RC_CHANNEL[6], 
              RC_CHANNEL[7], RC_CHANNEL[8], RC_CHANNEL[9]);
    }
}

/**
 * @brief 初始化RC系统
 */
void rc_init(){
    DBGLN("RC: 初始化RC系统...");
    
    // 初始化ADC引脚
    pinMode(ROLL_ADC, INPUT);
    pinMode(PITCH_ADC, INPUT);
    pinMode(THROTTLE_ADC, INPUT);
    pinMode(YAW_ADC, INPUT);
    pinMode(SLI10, INPUT);
    
    // 初始化开关引脚（使用内部上拉）
    pinMode(SW5, INPUT_PULLUP);
    pinMode(SW6_1, INPUT_PULLUP);
    pinMode(SW6_2, INPUT_PULLUP);
    pinMode(SW7_1, INPUT_PULLUP);
    pinMode(SW7_2, INPUT_PULLUP);
    pinMode(SW8, INPUT_PULLUP);
    pinMode(SW9, INPUT_PULLUP);
    
    // 初始化电源控制引脚
    pinMode(POWER_BTN, INPUT_PULLUP);
    pinMode(POWER_EN, OUTPUT);
    digitalWrite(POWER_EN, HIGH);  // 默认使能电源
    
    // 初始化通道值为中点
    for(int i = 0; i < 10; i++){
        RC_CHANNEL[i] = CHANNEL_MID;
    }
    
    DBGLN("RC: 初始化完成");
    DBGLN("RC: ADC引脚 - 副翼:%d 升降:%d 油门:%d 方向:%d SLI10:%d", 
          ROLL_ADC, PITCH_ADC, THROTTLE_ADC, YAW_ADC, SLI10);
    DBGLN("RC: 开关引脚 - SW5:%d SW6:%d,%d SW7:%d,%d SW8:%d SW9:%d ",
          SW5, SW6_1, SW6_2, SW7_1, SW7_2, SW8, SW9);
}
