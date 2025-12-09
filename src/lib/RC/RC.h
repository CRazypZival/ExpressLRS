#ifndef RC_H
#define RC_H

// GPIO 引脚定义
// AETR 通道 (副翼、升降、油门、方向)
#define ROLL_ADC        4      // 副翼通道 ADC 引脚
#define PITCH_ADC       5      // 升降通道 ADC 引脚
#define THROTTLE_ADC    6      // 油门通道 ADC 引脚
#define YAW_ADC         7      // 方向通道 ADC 引脚

// 开关引脚定义
#define SW5             15      // 开关5
#define SW6_1           18      // 开关6位置1
#define SW6_2           8      // 开关6位置2
#define SW7_1           3      // 开关7位置1
#define SW7_2           46      // 开关7位置2
#define SW8             9      // 开关8
#define SW9             10      // 开关9
#define SLI10           16      // 滑动开关10

// 电源控制引脚
#define POWER_BTN       21       // 电源按钮
#define POWER_EN        47       // 电源使能

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

// 函数声明

/**
 * @brief 初始化RC系统
 * 配置所有GPIO引脚，初始化通道值
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
