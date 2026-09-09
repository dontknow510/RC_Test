#ifndef YUNTAI_SERVO_APP_H
#define YUNTAI_SERVO_APP_H

#include "main.h"

/* ==========================================================================
 * 舵机执行层：TIM3_CH1(PA6) / TIM3_CH2(PA7) 输出 50Hz PWM
 *
 * 脉宽统一用微秒表示，运行时按 htim3.Init.Prescaler 换算成计数
 * （Prescaler 71 -> 1 计数 = 1us）。
 * ========================================================================== */

typedef enum
{
  SERVO_CHANNEL_HORIZONTAL = 0,
  SERVO_CHANNEL_PITCH,
  SERVO_CHANNEL_COUNT
} ServoChannel;

/* 脉宽限幅（微秒），防止输出越界撞机械限位 */
#define SERVO_PULSE_MIN_US      1000U
#define SERVO_PULSE_MAX_US      2000U
#define SERVO_PULSE_CENTER_US   1500U

/* APB1 定时器时钟 72MHz @ F103 */
#define SERVO_TIM_CLOCK_MHZ     72U

/* 启动 PWM 并输出中位脉宽 */
HAL_StatusTypeDef Servo_Init(void);

/* 设置脉宽（微秒），内部限幅到 [MIN, MAX] */
void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us);

#endif /* YUNTAI_SERVO_APP_H */