#ifndef RC_H
#define RC_H

// ============================================================
// RC 模式选择
// ============================================================
// 取消注释以下其中一行来选择模式:
// DRONE_MODE: 无人机模式 - 带电机控制，使用特定引脚配置
// RC_MODE: 遥控器模式 - 纯遥控器，使用不同引脚配置
// ============================================================

#define DRONE_MODE
// #define RC_MODE

// ============================================================
// GPIO 引脚定义 - 根据模式自动选择
// ============================================================

#ifdef DRONE_MODE
// ==================== 无人机模式引脚配置 ====================
// 此模式下，摇杆ADC引脚避开电机驱动引脚 (GPIO 4,5,6,7)
// AS5600与屏幕共享I2C总线 (GPIO 11, 12)

// AETR 通道 (副翼、升降、油门、方向) - 摇杆ADC引脚
#define ROLL_ADC        13     // 副翼通道 ADC 引脚
#define PITCH_ADC       14     // 升降通道 ADC 引脚
#define THROTTLE_ADC    21     // 油门通道 ADC 引脚
#define YAW_ADC         47     // 方向通道 ADC 引脚

// 开关引脚定义
#define SW5             17     // 开关5 (两段)
#define SW6_1           18     // 开关6位置1 (三段)
#define SW6_2           8      // 开关6位置2 (三段)
#define SW7_1           3      // 开关7位置1 (三段)
#define SW7_2           46     // 开关7位置2 (三段)
#define SW8             9      // 开关8 (两段)
#define SW9             45     // 开关9 (两段)
#define SLI10           35     // 滑动开关10 (电位器)

// AS5600 磁编码器 I2C 引脚 - 与屏幕共享I2C总线
#define AS5600_SCL      12     // 与屏幕共享 SCK
#define AS5600_SDA      11     // 与屏幕共享 SDA

// DRV8313 电机驱动引脚
#define DRV8313_EN      4      // 使能引脚
#define DRV8313_IN1     7      // PWM输入1
#define DRV8313_IN2     6      // PWM输入2
#define DRV8313_IN3     5      // PWM输入3

// 电源控制引脚
#define POWER_BTN       1      // 电源按钮
#define POWER_EN        2      // 电源使能

#endif // DRONE_MODE

#ifdef RC_MODE
// ==================== 遥控器模式引脚配置 ====================
// 此模式下，无电机控制，摇杆可以使用更多GPIO

// AETR 通道 (副翼、升降、油门、方向) - 摇杆ADC引脚
#define ROLL_ADC        4      // 副翼通道 ADC 引脚
#define PITCH_ADC       5      // 升降通道 ADC 引脚
#define THROTTLE_ADC    6      // 油门通道 ADC 引脚
#define YAW_ADC         7      // 方向通道 ADC 引脚

// 开关引脚定义
#define SW5             15     // 开关5 (两段)
#define SW6_1           18     // 开关6位置1 (三段)
#define SW6_2           8      // 开关6位置2 (三段)
#define SW7_1           3      // 开关7位置1 (三段)
#define SW7_2           46     // 开关7位置2 (三段)
#define SW8             9      // 开关8 (两段)
#define SW9             10     // 开关9 (两段)
#define SLI10           16     // 滑动开关10 (电位器)

// 电源控制引脚
#define POWER_BTN       21     // 电源按钮
#define POWER_EN        47     // 电源使能

#endif // RC_MODE

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
