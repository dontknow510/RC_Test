#ifndef APP_MOTOR_H
#define APP_MOTOR_H

#include "stm32f4xx_hal.h"

typedef enum
{
  MOTOR_MODE_IDLE = 0,
  MOTOR_MODE_SPEED
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

void Motor_Init(TIM_HandleTypeDef *pwmTimer,
                TIM_HandleTypeDef *encoderTimer,
                ADC_HandleTypeDef *adc);
void Motor_SetMode(MotorMode mode);
void Motor_Stop(void);
void Motor_SpeedToggle(void);
void Motor_SetPidGains(float kp, float ki);
void Motor_SetKp(float kp);
void Motor_SetKi(float ki);
uint8_t Motor_MainLoopUpdate(void);
void Motor_TIM6_Update(void);
void Motor_GetSpeedData(MotorSpeedData *data);

#endif
