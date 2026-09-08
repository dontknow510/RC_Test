#ifndef YUNTAI_MPU6050_APP_H
#define YUNTAI_MPU6050_APP_H

#include "mpu6050.h"

typedef struct
{
  uint8_t initialized;
  uint8_t sample_valid;
  uint8_t calibration_active;
  uint8_t calibration_valid;
  uint8_t static_valid;
  uint8_t who_am_i;
  uint16_t calibration_samples;
  I2C_HandleTypeDef *i2c;
  HAL_StatusTypeDef init_status;
  HAL_StatusTypeDef read_status;
  float accel_norm_g;
  float roll_deg;
  float pitch_deg;
  float yaw_deg;
  float roll_zero_deg;
  float pitch_zero_deg;
  float yaw_zero_deg;
  MPU6050_t data;
} MPU6050_AppState;

HAL_StatusTypeDef MPU6050_App_Init(void);
HAL_StatusTypeDef MPU6050_App_Update(void);
const MPU6050_AppState *MPU6050_App_GetState(void);

#endif /* YUNTAI_MPU6050_APP_H */
