#ifndef YUNTAI_GIMBAL_APP_H
#define YUNTAI_GIMBAL_APP_H

#include "main.h"
#include "adc_input.h"
#include "mpu6050_app.h"
#include "servo_app.h"

/* ==========================================================================
 * 云台控制层（主控端）
 *
 * 两种控制方式，KEY1 实时切换；KEY2 在陀螺仪模式下重新归零：
 *   1) 电位器模式：PC0 / PC1 两个电位器 -> 水平 / 俯仰目标脉宽（全量程线性）
 *   2) 陀螺仪模式：MPU6050 相对姿态变化 -> 目标脉宽（相对角镜像）
 *
 * 统一目标源：算出的 (CH1_US, CH2_US) 同时喂给
 *   - 本地 TIM1 舵机（单板演示）
 *   - 蓝牙发送（执行端）
 * 因此本地和远端永远一致。
 * ========================================================================== */

/* --- 电位器 -> 目标脉宽（全量程线性） --- */
#define GIMBAL_ADC_RAW_MIN        0U
#define GIMBAL_ADC_RAW_MAX        4095U

/* ADC 一阶低通权重 1/(2^SHIFT)，10 ms 一次：2 -> 约 40 ms 时间常数 */
#define GIMBAL_ADC_FILTER_SHIFT   2U

/* 方向：0 = 电位器值 / 姿态角增大 -> 脉宽增大；1 = 反向 */
#define GIMBAL_HORIZONTAL_INVERT  0U
#define GIMBAL_PITCH_INVERT       0U

/* --- 陀螺仪姿态跟随：姿态每变化 1 度对应目标脉宽变化多少微秒 ---
 *   5.0f  -> 面包板转约 ±100 度才到脉宽限位，手感柔和（起步建议）
 *   11.0f -> 接近 1:1 角度镜像（按 SG90 约 11 us/度估算），更灵敏 */
#define GIMBAL_GYRO_US_PER_DEG    5.0f

/* 每个节拍目标脉宽最大变化量（us）：限制猛甩、保护舵机、平滑模式切换 */
#define GIMBAL_SLEW_US_PER_TICK   50U

typedef enum
{
  GIMBAL_MODE_POTENTIOMETER = 0,
  GIMBAL_MODE_GYRO = 1
} GimbalMode;

void Gimbal_Init(void);

/* 主循环每个 10 ms 节拍调用：处理按键、计算目标、驱动本地舵机、发送蓝牙帧 */
void Gimbal_Update(const ADC_InputData *adc, const MPU6050_AppState *mpu);

GimbalMode Gimbal_GetMode(void);

#endif /* YUNTAI_GIMBAL_APP_H */
