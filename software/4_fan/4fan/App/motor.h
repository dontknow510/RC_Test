#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "stm32f4xx_hal.h"

typedef enum
{
  MOTOR_MODE_IDLE = 0,
  MOTOR_MODE_SPEED,
  MOTOR_MODE_POSITION
} MotorMode;

typedef struct
{
  uint16_t targetRpm;
  int32_t rpm;
  uint8_t pwmEnabled;
  uint8_t pwmDuty;
  uint16_t kp100;
  uint16_t ki100;
} MotorSpeedData;

typedef struct
{
  uint16_t targetAngle;
  int32_t targetCount;
  int32_t positionCount;
  int32_t positionAngle;
  int32_t errorCount;
  int32_t errorAngle;
  int16_t targetRpm;
  uint16_t positionKp100;
  uint8_t pwmEnabled;
  uint8_t pwmDuty;
} MotorPositionData;

void Motor_Init(TIM_HandleTypeDef *pwmTimer,
                TIM_HandleTypeDef *encoderTimer,
                ADC_HandleTypeDef *adc);
void Motor_SetMode(MotorMode mode);
void Motor_Stop(void);
void Motor_SpeedToggle(void);
void Motor_PositionToggle(void);
void Motor_SetPidGains(float kp, float ki);
void Motor_SetKp(float kp);
void Motor_SetKi(float ki);
void Motor_SetPositionKp(float kp);
void Motor_SetPositionKd(float kd);
void Motor_SetPositionMaxRpm(uint16_t maxRpm);
void Motor_SetPositionDeadband(int32_t deadbandCount);
void Motor_SetPositionSpeedKp(float kp);
void Motor_SetPositionSpeedKi(float ki);
MotorMode Motor_GetMode(void);
uint8_t Motor_MainLoopUpdate(void);
void Motor_TIM6_Update(void);
void Motor_GetSpeedData(MotorSpeedData *data);
void Motor_GetPositionData(MotorPositionData *data);

#endif
