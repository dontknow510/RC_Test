#include "motor.h"
#include "gpio.h"

#define MOTOR_ENCODER_CPR             1456L
#define MOTOR_MAX_TARGET_RPM          300U
#define MOTOR_PI_KP                   0.10f
#define MOTOR_PI_KI                   0.03f
#define MOTOR_PI_INTEGRAL_LIMIT       1000.0f
#define MOTOR_MIN_PWM                 10U

static TIM_HandleTypeDef *motorPwmTimer;
static TIM_HandleTypeDef *motorEncoderTimer;
static ADC_HandleTypeDef *motorAdc;
static MotorMode motorMode = MOTOR_MODE_IDLE;

static volatile int32_t devTotalCount;
static volatile uint16_t devPreviousCount;
static volatile int16_t devDeltaCount;
static volatile uint8_t devDirection;
static volatile int32_t devRpm;
static volatile int32_t devSpeedCount;
static volatile uint8_t devSpeedSamples;
static volatile uint8_t devPwmEnabled;
static volatile uint8_t devPwmDuty;

static volatile uint16_t speedPreviousCount;
static volatile int32_t speedRpm;
static volatile int32_t speedSpeedCount;
static volatile uint8_t speedSpeedSamples;
static volatile uint16_t speedTargetRpm;
static volatile uint8_t speedPwmEnabled;
static volatile uint8_t speedPwmDuty;
static volatile float speedIntegral;
static uint32_t adcLastUpdateTick;

static void Motor_ResetEncoderState(void)
{
  __HAL_TIM_SET_COUNTER(motorEncoderTimer, 0U);
  devTotalCount = 0;
  devPreviousCount = 0U;
  devDeltaCount = 0;
  devDirection = 0U;
  devRpm = 0;
  devSpeedCount = 0;
  devSpeedSamples = 0U;
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
  uint16_t value = 0U;

  if (HAL_ADC_Start(motorAdc) == HAL_OK)
  {
    if (HAL_ADC_PollForConversion(motorAdc, 10U) == HAL_OK)
    {
      value = (uint16_t)HAL_ADC_GetValue(motorAdc);
    }
    (void)HAL_ADC_Stop(motorAdc);
  }

  return value;
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
  else
  {
    devPwmEnabled = 0U;
    devPwmDuty = 0U;
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
  devPwmEnabled = 0U;
  devPwmDuty = 0U;
  speedPwmEnabled = 0U;
  speedPwmDuty = 0U;
  speedIntegral = 0.0f;
  motorMode = MOTOR_MODE_IDLE;
}

void Motor_DevelopmentIncreaseDuty(void)
{
  devPwmDuty = (devPwmDuty >= 95U) ? 100U : (uint8_t)(devPwmDuty + 5U);
  if (devPwmEnabled != 0U)
  {
    __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, devPwmDuty);
  }
}

void Motor_DevelopmentDecreaseDuty(void)
{
  devPwmDuty = (devPwmDuty <= 5U) ? 0U : (uint8_t)(devPwmDuty - 5U);
  if (devPwmEnabled != 0U)
  {
    __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, devPwmDuty);
  }
}

void Motor_DevelopmentToggle(void)
{
  devPwmEnabled = (devPwmEnabled == 0U) ? 1U : 0U;
  __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1,
                        devPwmEnabled ? devPwmDuty : 0U);
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
  if (motorMode == MOTOR_MODE_DEVELOPMENT)
  {
    uint16_t currentCount = (uint16_t)__HAL_TIM_GET_COUNTER(motorEncoderTimer);
    int16_t delta = -(int16_t)(currentCount - devPreviousCount);
    devPreviousCount = currentCount;
    devDeltaCount = delta;
    devTotalCount += delta;
    devSpeedCount += delta;
    devSpeedSamples++;

    if (devSpeedSamples >= 10U)
    {
      devRpm = (devSpeedCount * 600L) / MOTOR_ENCODER_CPR;
      devSpeedCount = 0;
      devSpeedSamples = 0U;
    }

    devDirection = (delta > 0) ? 1U : (delta < 0) ? 2U : 0U;
  }
  else if (motorMode == MOTOR_MODE_SPEED)
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

      output = (MOTOR_PI_KP * error) + (MOTOR_PI_KI * speedIntegral);
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

void Motor_GetDevelopmentData(MotorDevelopmentData *data)
{
  __disable_irq();
  data->totalCount = devTotalCount;
  data->deltaCount = devDeltaCount;
  data->rpm = devRpm;
  data->direction = devDirection;
  data->pwmEnabled = devPwmEnabled;
  data->pwmDuty = devPwmDuty;
  __enable_irq();
}

void Motor_GetSpeedData(MotorSpeedData *data)
{
  __disable_irq();
  data->targetRpm = speedTargetRpm;
  data->rpm = speedRpm;
  data->pwmEnabled = speedPwmEnabled;
  data->pwmDuty = speedPwmDuty;
  data->kp = MOTOR_PI_KP;
  data->ki = MOTOR_PI_KI;
  __enable_irq();
}
