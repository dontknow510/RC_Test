#ifndef YUNTAI_MPU6050_APP_H
#define YUNTAI_MPU6050_APP_H

#include "main.h"

typedef struct
{
  uint8_t initialized;
  uint8_t sample_valid;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
} MPU6050_AppState;

HAL_StatusTypeDef MPU6050_App_Init(void);
HAL_StatusTypeDef MPU6050_App_Update(void);
const MPU6050_AppState *MPU6050_App_GetState(void);

#endif /* YUNTAI_MPU6050_APP_H */
