#include "motor.h"
#include "gpio.h"

#define MOTOR_ENCODER_CPR             1456L
#define MOTOR_MAX_TARGET_RPM          300U
#define MOTOR_PI_KP                   1.2f
#define MOTOR_PI_KI                   2.7f
#define MOTOR_PI_INTEGRAL_LIMIT       1000.0f
#define MOTOR_MIN_PWM                 10U
#define MOTOR_ADC_AVERAGE_SAMPLES     16U
#define MOTOR_POSITION_MAX_TARGET_RPM 60
#define MOTOR_POSITION_KP              1.3f
#define MOTOR_POSITION_KD              0.60f
#define MOTOR_POSITION_DEADBAND_COUNT 4L
#define MOTOR_POSITION_MAX_RPM_LIMIT   300U
#define MOTOR_POSITION_KP_MAX          100.0f
#define MOTOR_POSITION_PI_KP           1.2f
#define MOTOR_POSITION_PI_KI           2.7f

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
static volatile uint16_t positionPreviousCount;
static volatile int32_t positionCount;
static volatile uint16_t positionTargetAngle;
static volatile int32_t positionTargetCount;
static volatile int16_t positionTargetRpm;
static volatile uint8_t positionPwmEnabled;
static volatile uint8_t positionPwmDuty;
static volatile float positionKp = MOTOR_POSITION_KP;
static volatile float positionKd = MOTOR_POSITION_KD;
static volatile uint16_t positionMaxTargetRpm = MOTOR_POSITION_MAX_TARGET_RPM;
static volatile int32_t positionDeadbandCount = MOTOR_POSITION_DEADBAND_COUNT;
static volatile float positionSpeedIntegral;
static volatile float positionSpeedKp = MOTOR_POSITION_PI_KP;
static volatile float positionSpeedKi = MOTOR_POSITION_PI_KI;
static volatile float positionActualRpmFiltered;
static int8_t positionDirection;
static uint32_t adcLastUpdateTick;

static void Motor_ResetEncoderState(void)
{
  __HAL_TIM_SET_COUNTER(motorEncoderTimer, 0U);
  speedPreviousCount = 0U;
  speedRpm = 0;
  speedSpeedCount = 0;
  speedSpeedSamples = 0U;
  positionPreviousCount = 0U;
  positionCount = 0;
  positionTargetAngle = 0U;
  positionTargetCount = 0;
  positionTargetRpm = 0;
  positionPwmEnabled = 0U;
  positionPwmDuty = 0U;
  positionSpeedIntegral = 0.0f;
  positionActualRpmFiltered = 0.0f;
  positionDirection = 0;
}

static void Motor_SetDirection(int8_t direction)
{
  if (direction > 0)
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_SET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_RESET);
  }
  else if (direction < 0)
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_4, GPIO_PIN_RESET);
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOE, GPIO_PIN_2 | GPIO_PIN_4, GPIO_PIN_RESET);
  }
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

  if (HAL_TIM_PWM_Start(motorPwmTimer, TIM_CHANNEL_1) != HAL_OK ||
      HAL_TIM_Encoder_Start(motorEncoderTimer, TIM_CHANNEL_ALL) != HAL_OK)
  {
    Error_Handler();
  }

  if (mode == MOTOR_MODE_SPEED)
  {
    Motor_SetDirection(1);
    speedTargetRpm = 0U;
    speedPwmEnabled = 0U;
    speedPwmDuty = 0U;
    speedIntegral = 0.0f;
  }
  else
  {
    Motor_SetDirection(0);
    positionSpeedIntegral = 0.0f;
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
  positionPwmEnabled = 0U;
  positionPwmDuty = 0U;
  positionTargetRpm = 0;
  positionSpeedIntegral = 0.0f;
  positionDirection = 0;
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

void Motor_PositionToggle(void)
{
  positionPwmEnabled = (positionPwmEnabled == 0U) ? 1U : 0U;
  positionTargetRpm = 0;
  positionSpeedIntegral = 0.0f;

  if (positionPwmEnabled == 0U)
  {
    positionPwmDuty = 0U;
    positionDirection = 0;
    Motor_SetDirection(0);
  }

  __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, positionPwmDuty);
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

void Motor_SetPositionKp(float kp)
{
  if (!(kp >= 0.0f && kp <= MOTOR_POSITION_KP_MAX))
  {
    return;
  }

  positionKp = kp;
  positionSpeedIntegral = 0.0f;
}

void Motor_SetPositionKd(float kd)
{
  if (!(kd >= 0.0f && kd <= MOTOR_POSITION_KP_MAX))
  {
    return;
  }

  positionKd = kd;
  positionSpeedIntegral = 0.0f;
}

void Motor_SetPositionMaxRpm(uint16_t maxRpm)
{
  if (maxRpm > MOTOR_POSITION_MAX_RPM_LIMIT)
  {
    return;
  }

  positionMaxTargetRpm = maxRpm;
  positionSpeedIntegral = 0.0f;
}

void Motor_SetPositionDeadband(int32_t deadbandCount)
{
  if (deadbandCount < 0 || deadbandCount > MOTOR_ENCODER_CPR)
  {
    return;
  }

  positionDeadbandCount = deadbandCount;
  positionSpeedIntegral = 0.0f;
}

void Motor_SetPositionSpeedKp(float kp)
{
  if (!(kp >= 0.0f && kp <= MOTOR_POSITION_KP_MAX))
  {
    return;
  }

  positionSpeedKp = kp;
  positionSpeedIntegral = 0.0f;
}

void Motor_SetPositionSpeedKi(float ki)
{
  if (!(ki >= 0.0f && ki <= MOTOR_POSITION_KP_MAX))
  {
    return;
  }

  positionSpeedKi = ki;
  positionSpeedIntegral = 0.0f;
}

MotorMode Motor_GetMode(void)
{
  return motorMode;
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
  else if (motorMode == MOTOR_MODE_POSITION)
  {
    uint16_t adcValue = Motor_ReadAdc();
    positionTargetAngle = (uint16_t)(((uint32_t)adcValue * 360U + 2047U) /
                                     4095U);
    positionTargetCount = (int32_t)(((uint32_t)adcValue *
                                     (uint32_t)MOTOR_ENCODER_CPR + 2047U) /
                                    4095U);
  }

  return (motorMode == MOTOR_MODE_SPEED || motorMode == MOTOR_MODE_POSITION) ?
         1U : 0U;
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
  else if (motorMode == MOTOR_MODE_POSITION)
  {
    uint16_t currentCount = (uint16_t)__HAL_TIM_GET_COUNTER(motorEncoderTimer);
    int16_t delta = -(int16_t)(currentCount - positionPreviousCount);
    int32_t positionError;
    float targetRpm;
    float actualRpm;
    float speedError;
    float output;
    int8_t direction;

    positionPreviousCount = currentCount;
    positionCount += (int32_t)delta;

    actualRpm = ((float)delta * 6000.0f) /
                (float)MOTOR_ENCODER_CPR;
    positionActualRpmFiltered +=
        0.25f * (actualRpm - positionActualRpmFiltered);
    positionError = positionTargetCount - positionCount;

    if (positionPwmEnabled == 0U || positionKp <= 0.0f ||
        positionMaxTargetRpm == 0U ||
        (positionError <= positionDeadbandCount &&
         positionError >= -positionDeadbandCount))
    {
      positionTargetRpm = 0;
      positionPwmDuty = 0U;
      positionSpeedIntegral = 0.0f;
      positionDirection = 0;
      Motor_SetDirection(0);
      __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
      return;
    }

    targetRpm = ((float)positionError * positionKp) -
                (positionActualRpmFiltered * positionKd);
    if (targetRpm > (float)positionMaxTargetRpm)
    {
      targetRpm = (float)positionMaxTargetRpm;
    }
    else if (targetRpm < -(float)positionMaxTargetRpm)
    {
      targetRpm = -(float)positionMaxTargetRpm;
    }
    else if (targetRpm > 0.0f && targetRpm < 1.0f)
    {
      targetRpm = 1.0f;
    }
    else if (targetRpm < 0.0f && targetRpm > -1.0f)
    {
      targetRpm = -1.0f;
    }
    positionTargetRpm = (int16_t)targetRpm;
    direction = (positionTargetRpm > 0) ? 1 : -1;

    if (direction != positionDirection)
    {
      __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, 0U);
      Motor_SetDirection(direction);
      positionSpeedIntegral = 0.0f;
      positionDirection = direction;
    }

    speedError = (float)positionTargetRpm - actualRpm;
    positionSpeedIntegral += speedError * 0.01f;
    if (positionSpeedIntegral > MOTOR_PI_INTEGRAL_LIMIT)
    {
      positionSpeedIntegral = MOTOR_PI_INTEGRAL_LIMIT;
    }
    else if (positionSpeedIntegral < -MOTOR_PI_INTEGRAL_LIMIT)
    {
      positionSpeedIntegral = -MOTOR_PI_INTEGRAL_LIMIT;
    }

    output = (positionSpeedKp * speedError) +
             (positionSpeedKi * positionSpeedIntegral);
    if (output < (float)MOTOR_MIN_PWM)
    {
      output = (float)MOTOR_MIN_PWM;
    }
    else if (output > 100.0f)
    {
      output = 100.0f;
    }

    positionPwmDuty = (uint8_t)(output + 0.5f);
    __HAL_TIM_SET_COMPARE(motorPwmTimer, TIM_CHANNEL_1, positionPwmDuty);
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

void Motor_GetPositionData(MotorPositionData *data)
{
  if (data == NULL)
  {
    return;
  }

  __disable_irq();
  data->targetAngle = positionTargetAngle;
  data->targetCount = positionTargetCount;
  data->positionCount = positionCount;
  data->positionAngle = (positionCount * 360L) / MOTOR_ENCODER_CPR;
  data->errorCount = positionTargetCount - positionCount;
  data->errorAngle = (data->errorCount * 360L) / MOTOR_ENCODER_CPR;
  data->targetRpm = positionTargetRpm;
  data->positionKp100 = (uint16_t)(positionKp * 100.0f + 0.5f);
  data->pwmEnabled = positionPwmEnabled;
  data->pwmDuty = positionPwmDuty;
  __enable_irq();
}
