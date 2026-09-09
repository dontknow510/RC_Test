#include "servo_app.h"
#include "tim.h"

static const uint32_t servo_channel[SERVO_CHANNEL_COUNT] =
{
  TIM_CHANNEL_1,
  TIM_CHANNEL_2
};

/* 当前脉宽（微秒，已按定时器分辨率量化），供显示使用 */
static uint16_t g_pulse_us[SERVO_CHANNEL_COUNT];

/* 微秒 -> 计数：计数 = us * 168MHz / (Prescaler + 1) */
static uint16_t Servo_UsToCounts(uint16_t us)
{
  uint32_t divider = htim1.Init.Prescaler + 1U;
  uint32_t counts;

  if (divider == 0U)
  {
    divider = 1U;
  }

  counts = ((uint32_t)us * SERVO_TIM_CLOCK_MHZ) / divider;

  return (counts > 0xFFFFU) ? 0xFFFFU : (uint16_t)counts;
}

static uint16_t Servo_CountsToUs(uint16_t counts)
{
  uint32_t divider = htim1.Init.Prescaler + 1U;

  if (divider == 0U)
  {
    divider = 1U;
  }

  return (uint16_t)(((uint32_t)counts * divider) / SERVO_TIM_CLOCK_MHZ);
}

HAL_StatusTypeDef Servo_Init(void)
{
  HAL_StatusTypeDef status;
  uint16_t center_counts = Servo_UsToCounts(SERVO_PULSE_CENTER_US);
  uint32_t i;

  for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
  {
    g_pulse_us[i] = SERVO_PULSE_CENTER_US;
    __HAL_TIM_SET_COMPARE(&htim1, servo_channel[i], center_counts);
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
}

void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us)
{
  uint16_t counts;

  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return;
  }

  if (pulse_us < SERVO_PULSE_MIN_US)
  {
    pulse_us = SERVO_PULSE_MIN_US;
  }
  else if (pulse_us > SERVO_PULSE_MAX_US)
  {
    pulse_us = SERVO_PULSE_MAX_US;
  }

  counts = Servo_UsToCounts(pulse_us);
  g_pulse_us[channel] = Servo_CountsToUs(counts);
  __HAL_TIM_SET_COMPARE(&htim1, servo_channel[channel], counts);
}

uint16_t Servo_GetPulseUs(ServoChannel channel)
{
  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return SERVO_PULSE_CENTER_US;
  }

  return g_pulse_us[channel];
}
