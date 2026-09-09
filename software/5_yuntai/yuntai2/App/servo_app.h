#ifndef YUNTAI_SERVO_APP_H
#define YUNTAI_SERVO_APP_H

#include "main.h"

/* ==========================================================================
 * 舵机执行层：TIM1_CH1(PE9) / TIM1_CH2(PE11) 输出 50Hz PWM
 *
 * 脉宽统一用微秒表示，运行时按 htim1.Init.Prescaler 换算成计数
 * （Prescaler 167 -> 1 计数 = 1us），所以改定时器分辨率不用动本文件。
 * 控制逻辑（电位器映射、陀螺仪跟随、模式切换）在 gimbal_app.c。
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

/* APB2 定时器时钟 168MHz @ F407 */
#define SERVO_TIM_CLOCK_MHZ     168U

/* 启动 PWM 并输出中位脉宽 */
HAL_StatusTypeDef Servo_Init(void);

/* 设置脉宽（微秒），内部限幅到 [MIN, MAX] */
void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us);

/* 当前实际输出的脉宽（微秒，已按定时器分辨率量化） */
uint16_t Servo_GetPulseUs(ServoChannel channel);

#endif /* YUNTAI_SERVO_APP_H */
