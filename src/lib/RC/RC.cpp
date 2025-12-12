#include <Arduino.h>
#include "RC.h"
#include "logging.h"  // 添加日志支持
#include "crsf_protocol.h"  // 用于CRSF值转换

#include "Motor.h"
// 创建电机控制器对象
MotorController motorController;

int adc_get(int pin, int channel_index){

}

/**
 * @brief 更新所有RC通道值
 */
void channel_update(){

    motorController.loop();

}

/**
 * @brief 初始化RC系统
 */
void rc_init(){

    motorController.init();

}
