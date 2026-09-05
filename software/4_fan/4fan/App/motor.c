#include "motor.h"
#include "gpio.h"

#define MOTOR_ENCODER_CPR             1456L
#define MOTOR_MAX_TARGET_RPM          300U
#define MOTOR_PI_KP                   1.2f
#define MOTOR_PI_KI                   2.7f
#define MOTOR_PI_INTEGRAL_LIMIT       1000.0f
#define MOTOR_MIN_PWM                 10U
#define MOTOR_ADC_AVERAGE_SAMPLES     16U

static TIM_HandleTypeDef *motorPwmTimer;
static TIM_HandleTypeDef *motorEncoderTimer;
static ADC_HandleTypeDef *motorAdc;
static MotorMode motorMode = MOTOR_MODE_IDLE;

static volatile uint16_t speedPreviousCount;
static volatile int32_t speedRpm;
static volatile int32_t speedSpeedCount;
static volatile uint8_t speedSpeedSamples;
static volatile uint16_t speedTargetRpm;
static volatile uint8_t speedPwmEnabled;
static volatile uint8_t speedPwmDuty;
static volatile float speedIntegral;
static volatile float speedKp = MOTOR_PI_KP;
static volatile float speedKi = MOTOR_PI_KI;
static uint32_t adcLastUpdateTick;

static void Motor_ResetEncoderState(void)
{
  __HAL_TIM_SET_COUNTER(motorEncoderTimer, 0U);
  speedPreviousCount = 0U;
  speedRpm = 0;
  speedSpeedCount = 0;
  speedSpeedSamples = 0U;
}

static void Motor_SetForwardDirection(void)
{
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
}

static uint16_t Motor_ReadAdc(void)
{
  uint32_t sum = 0U;
  uint8_t validSamples = 0U;
  uint8_t sample;

  for (sample = 0U; sample < MOTOR_ADC_AVERAGE_SAMPLES; sample++)
  {
    if (HAL_ADC_Start(motorAdc) == HAL_OK)
    {
      if (HAL_ADC_PollForConversion(motorAdc, 10U) == HAL_OK)
      {
        sum += (uint32_t)HAL_ADC_GetValue(motorAdc);
        validSamples++;
      }
      (void)HAL_ADC_Stop(motorAdc);
    }
  }

  return (validSamples == 0U) ? 0U : (uint16_t)(sum / validSamples);
}

void Motor_Init(TIM_HandleTypeDef *pwmTimer,
                TIM_HandleTypeDef *encoderTimer,
                ADC_HandleTypeDef *adc)
{
  motorPwmTimer = pwmTimer;
  motorEncoderTimer = encoderTimer;
  motorAdc = adc;
  Motor_Stop();
}

void Motor_SetMode(MotorMode mode)
{
  Motor_Stop();
  motorMode = mode;

  if (mode == MOTOR_MODE_IDLE)
  {
    return;
  }

  Motor_ResetEncoderState();
  Motor_SetForwardDirection();

  if (HAL_TIM_PWM_Start(motorPwmTimer, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_Encoder_Start(motorEncoderTimer, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  if (mode == MOTOR_MODE_SPEED)
  {
    speedTargetRpm = 0U;
    speedPwmEnabled = 0U;
    speedPwmDuty = 0U;
    speedIntegral = 0.0f;
  }
  __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
  adcLastUpdateTick = HAL_GetTick();
}

void Motor_Stop(void)
{
  if (motorPwmTimer != NULL)
  {
    __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
    (void)HAL_TIM_PWM_Stop(motorPwmTimer, TIM_CHANNEL_1);
  }
  if (motorEncoderTimer != NULL)
  {
    (void)HAL_TIM_Encoder_Stop(motorEncoderTimer, TIM_CHANNEL_ALL);
  }
  HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_4, GPIO_PIN_RESET);
  speedPwmEnabled = 0U;
  speedPwmDuty = 0U;
  speedIntegral = 0.0f;
  motorMode = MOTOR_MODE_IDLE;
}

void Motor_SpeedToggle(void)
{
  speedPwmEnabled = (speedPwmEnabled == 0U) ? 1U : 0U;
  speedIntegral = 0.0f;
  speedPwmDuty = (speedPwmEnabled != 0U && speedTargetRpm > 0U) ?
                 MOTOR_MIN_PWM : 0U;
  if (speedPwmEnabled == 0U)
  {
    speedRpm = 0;
    speedSpeedCount = 0;
    speedSpeedSamples = 0U;
  }
  __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, speedPwmDuty);
}

void Motor_SetPidGains(float kp, float ki)
{
  speedKp = kp;
  speedKi = ki;
  speedIntegral = 0.0f;
}

void Motor_SetKp(float kp)
{
  speedKp = kp;
  speedIntegral = 0.0f;
}

void Motor_SetKi(float ki)
{
  speedKi = ki;
  speedIntegral = 0.0f;
}

uint8_t Motor_MainLoopUpdate(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - adcLastUpdateTick) < 100U)
  {
    return 0U;
  }
  adcLastUpdateTick = now;

  if (motorMode == MOTOR_MODE_SPEED)
  {
    uint16_t adcValue = Motor_ReadAdc();
    speedTargetRpm = (uint16_t)(((uint32_t)adcValue *
                                 MOTOR_MAX_TARGET_RPM + 2047U) / 4095U);

    if (speedTargetRpm == 0U && speedPwmEnabled != 0U)
    {
      speedPwmEnabled = 0U;
      speedPwmDuty = 0U;
      speedIntegral = 0.0f;
      __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
    }
  }

  return (motorMode == MOTOR_MODE_SPEED) ? 1U : 0U;
}

void Motor_TIM6_Update(void)
{
  if (motorMode == MOTOR_MODE_SPEED)
  {
    uint16_t currentCount = (uint16_t)__HAL_TIM_GET_COUNTER(motorEncoderTimer);
    int16_t delta = -(int16_t)(currentCount - speedPreviousCount);
    float actualRpm = ((float)delta * 6000.0f) /
                      (float)MOTOR_ENCODER_CPR;
    speedPreviousCount = currentCount;
    speedSpeedCount += delta;
    speedSpeedSamples++;

    if (speedPwmEnabled == 0U || speedTargetRpm == 0U)
    {
      speedIntegral = 0.0f;
      speedPwmDuty = 0U;
      __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
    }
    else
    {
      float error = (float)speedTargetRpm - actualRpm;
      float output;
      speedIntegral += error * 0.01f;

      if (speedIntegral > MOTOR_PI_INTEGRAL_LIMIT)
      {
        speedIntegral = MOTOR_PI_INTEGRAL_LIMIT;
      }
      else if (speedIntegral < -MOTOR_PI_INTEGRAL_LIMIT)
      {
        speedIntegral = -MOTOR_PI_INTEGRAL_LIMIT;
      }

      output = (speedKp * error) + (speedKi * speedIntegral);
      if (output < (float)MOTOR_MIN_PWM)
      {
        output = (float)MOTOR_MIN_PWM;
      }
      else if (output > 100.0f)
      {
        output = 100.0f;
      }
      speedPwmDuty = (uint8_t)(output + 0.5f);
      __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, speedPwmDuty);
    }

    if (speedSpeedSamples >= 10U)
    {
      speedRpm = (speedSpeedCount * 600L) / MOTOR_ENCODER_CPR;
      speedSpeedCount = 0;
      speedSpeedSamples = 0U;
    }
  }
}

void Motor_GetSpeedData(MotorSpeedData *data)
{
  __disable_irq();
  data->targetRpm = speedTargetRpm;
  data->rpm = speedRpm;
  data->pwmEnabled = speedPwmEnabled;
  data->pwmDuty = speedPwmDuty;
  data->kp100 = (uint16_t)(speedKp * 100.0f + 0.5f);
  data->ki100 = (uint16_t)(speedKi * 100.0f + 0.5f);
  __enable_irq();
}
