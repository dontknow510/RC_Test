#ifndef YUNTAI_SERVO_APP_H
#define YUNTAI_SERVO_APP_H

#include "main.h"

/* ==========================================================================
 * 舵机执行层（TIM3 PWM 输出，纯执行器）
 *
 * 通道映射：
 *   SERVO_CHANNEL_HORIZONTAL  PA6 / TIM3_CH1
 *   SERVO_CHANNEL_PITCH       PA7 / TIM3_CH2
 *
 * 本层只负责"把目标脉宽输出到硬件"：脉宽限幅、微秒 <-> 计数换算。
 * 目标脉宽来自蓝牙链路层（见 App/slave_app.c）。
 *
 * 脉宽一律以微秒表示，运行时按 htim3.Init.Prescaler 换算成定时器计数：
 *   Prescaler 71 -> 1 计数 = 1 us（F103 @ 72 MHz）
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

/* --- 定时器计数器时钟（APB1 定时器时钟 72 MHz @ F103） --- */
#define SERVO_TIM_CLOCK_MHZ       72U

/* 启动 TIM3 CH1/CH2 PWM 并输出中位脉宽；任一通道启动失败返回错误码。 */
HAL_StatusTypeDef Servo_Init(void);

/* 按微秒设置脉宽，内部限幅到 [SERVO_PULSE_MIN_US, SERVO_PULSE_MAX_US]。 */
void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us);

/* 直接写 TIM3 比较值（原始计数），内部限幅到换算后的最小/最大计数。 */
void Servo_SetPulse(ServoChannel channel, uint16_t pulse_counts);

uint16_t Servo_GetPulseUs(ServoChannel channel);
uint16_t Servo_GetPulse(ServoChannel channel);

#endif /* YUNTAI_SERVO_APP_H */