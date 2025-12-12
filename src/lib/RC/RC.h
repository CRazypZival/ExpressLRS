#ifndef RC_H
#define RC_H

// AS5600 磁编码器 I2C 引脚 - 与屏幕共享I2C总线
#define AS5600_SCL      15     // 与屏幕共享 SCK
#define AS5600_SDA      16     // 与屏幕共享 SDA

// DRV8313 电机驱动引脚
#define DRV8313_EN      4      // 使能引脚 (注意: 与ROLL_ADC共用GPIO4)
#define DRV8313_IN1     7      // PWM输入1 (注意: 与YAW_ADC共用GPIO7)
#define DRV8313_IN2     6      // PWM输入2 (注意: 与THROTTLE_ADC共用GPIO6)
#define DRV8313_IN3     5      // PWM输入3 (注意: 与PITCH_ADC共用GPIO5)

// ============================================================
// 通用定义 (两种模式共用)
// ============================================================

// RC通道数组 (10个通道)
// CH1-4: 摇杆 (副翼、升降、油门、方向)
// CH5-9: 开关
// CH10: 滑动开关/电位器
extern int RC_CHANNEL[10];

// 通道值范围定义
#define CHANNEL_MAX 2100
#define CHANNEL_MID 1500
#define CHANNEL_MIN 900

// ADC校准值 (可根据实际硬件调整)
#define ROLL_CALIBRATION_MAX    3600
#define ROLL_CALIBRATION_MID    1930
#define ROLL_CALIBRATION_MIN    400

#define PITCH_CALIBRATION_MAX   3400
#define PITCH_CALIBRATION_MID   1970
#define PITCH_CALIBRATION_MIN   490

#define THROTTLE_CALIBRATION_MAX 3360
#define THROTTLE_CALIBRATION_MID 1940
#define THROTTLE_CALIBRATION_MIN 490

#define YAW_CALIBRATION_MAX     3620
#define YAW_CALIBRATION_MID     1945
#define YAW_CALIBRATION_MIN     500

#define SLIDE_CALIBRATION_MAX   3323
#define SLIDE_CALIBRATION_MID   2040
#define SLIDE_CALIBRATION_MIN   488

// 死区设置 (ADC值)
#define ADC_DEADZONE 20

// ============================================================
// 函数声明
// ============================================================

/**
 * @brief 初始化RC系统
 * 根据当前模式(DRONE_MODE/RC_MODE)配置所有GPIO引脚，初始化通道值
 */
void rc_init();

/**
 * @brief 更新所有RC通道值
 * 读取ADC和开关状态，更新RC_CHANNEL数组
 * 应该在主循环中定期调用
 */
void channel_update();

/**
 * @brief 读取并处理单个ADC通道
 * @param pin ADC引脚编号
 * @param channel_index 通道索引（用于滤波器，-1表示不使用）
 * @return 映射后的通道值 (CHANNEL_MIN 到 CHANNEL_MAX)
 */
int adc_get(int pin, int channel_index);

#endif // RC_H
