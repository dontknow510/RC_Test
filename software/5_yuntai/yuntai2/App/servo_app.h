#ifndef YUNTAI_SERVO_APP_H
#define YUNTAI_SERVO_APP_H

#include "main.h"

/* ==========================================================================
 * 舵机执行层（TIM1 PWM 输出，纯执行器）
 *
 * 通道映射：
 *   SERVO_CHANNEL_HORIZONTAL  PE9  / TIM1_CH1
 *   SERVO_CHANNEL_PITCH       PE11 / TIM1_CH2
 *
 * 本层只负责"把目标脉宽输出到硬件"：脉宽限幅、微秒 <-> 计数换算、角度换算。
 * 控制逻辑（电位器映射、陀螺仪跟随、模式切换、蓝牙发送）在 App/gimbal_app.c。
 *
 * 脉宽一律以微秒表示，运行时按 htim1.Init.Prescaler 换算成定时器计数，
 * 因此改变 TIM1 分辨率时本文件无需修改：
 *   Prescaler 1679 -> 1 计数 = 10 us
 *   Prescaler  167 -> 1 计数 =  1 us（当前）
 * ========================================================================== */

typedef enum
{
  SERVO_CHANNEL_HORIZONTAL = 0,
  SERVO_CHANNEL_PITCH,
  SERVO_CHANNEL_COUNT
} ServoChannel;

/* --- 脉宽范围（微秒），防止输出越界撞机械限位 --- */
#define SERVO_PULSE_MIN_US        1000U
#define SERVO_PULSE_MAX_US        2000U
#define SERVO_PULSE_CENTER_US     1500U

/* --- TIM1 计数器时钟（APB2 定时器时钟 168 MHz），用于微秒 -> 计数换算 --- */
#define SERVO_TIM1_CLOCK_MHZ      168U

/* --- 角度范围：SERVO_ANGLE_MIN_DEG 对应 SERVO_PULSE_MIN_US --- */
#define SERVO_ANGLE_MIN_DEG       0.0f
#define SERVO_ANGLE_MAX_DEG       180.0f

/* 启动 TIM1 CH1/CH2 PWM 并输出中位脉宽；任一通道启动失败返回错误码。 */
HAL_StatusTypeDef Servo_Init(void);

/* 按微秒设置脉宽，内部限幅到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]。 */
void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us);

/* 直接写 TIM1 比较值（原始计数），内部限幅到换算后的最小/最大计数。 */
void Servo_SetPulse(ServoChannel channel, uint16_t pulse_counts);

/* 按角度设置脉宽，内部限幅到 [SERVO_ANGLE_MIN_DEG, SERVO_ANGLE_MAX_DEG]。 */
void Servo_SetAngle(ServoChannel channel, float angle_deg);

/* 返回当前实际输出的脉宽，单位为微秒（已按定时器分辨率量化）。 */
uint16_t Servo_GetPulseUs(ServoChannel channel);

/* 返回当前 TIM1 比较值（原始计数）。 */
uint16_t Servo_GetPulse(ServoChannel channel);

float Servo_GetAngle(ServoChannel channel);

#endif /* YUNTAI_SERVO_APP_H */
