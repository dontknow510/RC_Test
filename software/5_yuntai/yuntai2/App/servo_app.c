#include "servo_app.h"
#include "tim.h"

typedef struct
{
  uint32_t tim_channel;
  uint16_t pulse_counts;  /* 当前写入 TIM1 的比较值 */
  uint16_t pulse_us;      /* 当前脉宽（微秒，已按定时器分辨率量化） */
} ServoRuntime;

static ServoRuntime servo_runtime[SERVO_CHANNEL_COUNT] =
{
  {TIM_CHANNEL_1, 0U, SERVO_PULSE_CENTER_US},
  {TIM_CHANNEL_2, 0U, SERVO_PULSE_CENTER_US}
};

static uint32_t g_min_counts;
static uint32_t g_max_counts;
static uint32_t g_center_counts;

/* 微秒 -> TIM1 计数：计数 = us * 168 MHz / (Prescaler + 1) */
static uint16_t Servo_UsToCounts(uint16_t us)
{
  uint32_t divider = htim1.Init.Prescaler + 1U;
  uint32_t counts;

  if (divider == 0U)
  {
    divider = 1U;
  }

  counts = ((uint32_t)us * SERVO_TIM1_CLOCK_MHZ) / divider;
  if (counts > 0xFFFFU)
  {
    counts = 0xFFFFU;
  }

  return (uint16_t)counts;
}

static uint16_t Servo_CountsToUs(uint16_t counts)
{
  uint32_t divider = htim1.Init.Prescaler + 1U;

  if (divider == 0U)
  {
    divider = 1U;
  }

  return (uint16_t)(((uint32_t)counts * divider) / SERVO_TIM1_CLOCK_MHZ);
}

HAL_StatusTypeDef Servo_Init(void)
{
  HAL_StatusTypeDef status;
  uint32_t i;

  g_min_counts = Servo_UsToCounts(SERVO_PULSE_MIN_US);
  g_max_counts = Servo_UsToCounts(SERVO_PULSE_MAX_US);
  g_center_counts = Servo_UsToCounts(SERVO_PULSE_CENTER_US);

  for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
  {
    servo_runtime[i].pulse_us = SERVO_PULSE_CENTER_US;
    servo_runtime[i].pulse_counts = (uint16_t)g_center_counts;
    __HAL_TIM_SET_COMPARE(&htim1, servo_runtime[i].tim_channel, g_center_counts);
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_1);
  if (status != HAL_OK)
  {
    return status;
  }

  status = HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_2);
  if (status != HAL_OK)
  {
    return status;
  }

  return HAL_OK;
}

void Servo_SetPulse(ServoChannel channel, uint16_t pulse_counts)
{
  uint32_t clamped;

  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return;
  }

  clamped = (uint32_t)pulse_counts;
  if (clamped < g_min_counts)
  {
    clamped = g_min_counts;
  }
  else if (clamped > g_max_counts)
  {
    clamped = g_max_counts;
  }

  servo_runtime[channel].pulse_counts = (uint16_t)clamped;
  servo_runtime[channel].pulse_us = Servo_CountsToUs((uint16_t)clamped);
  __HAL_TIM_SET_COMPARE(&htim1, servo_runtime[channel].tim_channel, clamped);
}

void Servo_SetPulseUs(ServoChannel channel, uint16_t pulse_us)
{
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

  Servo_SetPulse(channel, Servo_UsToCounts(pulse_us));
}

void Servo_SetAngle(ServoChannel channel, float angle_deg)
{
  float pulse_us_f;

  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return;
  }

  if (angle_deg < SERVO_ANGLE_MIN_DEG)
  {
    angle_deg = SERVO_ANGLE_MIN_DEG;
  }
  if (angle_deg > SERVO_ANGLE_MAX_DEG)
  {
    angle_deg = SERVO_ANGLE_MAX_DEG;
  }

  pulse_us_f = (float)SERVO_PULSE_MIN_US +
               ((angle_deg - SERVO_ANGLE_MIN_DEG) *
                ((float)SERVO_PULSE_MAX_US - (float)SERVO_PULSE_MIN_US) /
                (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG));

  Servo_SetPulseUs(channel, (uint16_t)(pulse_us_f + 0.5f));
}

uint16_t Servo_GetPulseUs(ServoChannel channel)
{
  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return SERVO_PULSE_CENTER_US;
  }

  return servo_runtime[channel].pulse_us;
}

uint16_t Servo_GetPulse(ServoChannel channel)
{
  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return (uint16_t)g_center_counts;
  }

  return servo_runtime[channel].pulse_counts;
}

float Servo_GetAngle(ServoChannel channel)
{
  float pulse_us_f;

  if (channel >= SERVO_CHANNEL_COUNT)
  {
    return SERVO_ANGLE_MIN_DEG;
  }

  pulse_us_f = (float)servo_runtime[channel].pulse_us;

  return SERVO_ANGLE_MIN_DEG +
         ((pulse_us_f - (float)SERVO_PULSE_MIN_US) *
          (SERVO_ANGLE_MAX_DEG - SERVO_ANGLE_MIN_DEG) /
          ((float)SERVO_PULSE_MAX_US - (float)SERVO_PULSE_MIN_US));
}
