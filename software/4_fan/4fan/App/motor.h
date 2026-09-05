#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "stm32f4xx_hal.h"

typedef enum
{
  MOTOR_MODE_IDLE = 0,
  MOTOR_MODE_SPEED,
  MOTOR_MODE_DEVELOPMENT
} MotorMode;

typedef struct
{
  int32_t totalCount;
  int16_t deltaCount;
  int32_t rpm;
  uint8_t direction;
  uint8_t pwmEnabled;
  uint8_t pwmDuty;
} MotorDevelopmentData;

typedef struct
{
  uint16_t targetRpm;
  int32_t rpm;
  uint8_t pwmEnabled;
  uint8_t pwmDuty;
  float kp;
  float ki;
} MotorSpeedData;

void Motor_Init(TIM_HandleTypeDef *pwmTimer,
                TIM_HandleTypeDef *encoderTimer,
                ADC_HandleTypeDef *adc);
void Motor_SetMode(MotorMode mode);
void Motor_Stop(void);
void Motor_DevelopmentIncreaseDuty(void);
void Motor_DevelopmentDecreaseDuty(void);
void Motor_DevelopmentToggle(void);
void Motor_SpeedToggle(void);
uint8_t Motor_MainLoopUpdate(void);
void Motor_TIM6_Update(void);
void Motor_GetDevelopmentData(MotorDevelopmentData *data);
void Motor_GetSpeedData(MotorSpeedData *data);

#endif
