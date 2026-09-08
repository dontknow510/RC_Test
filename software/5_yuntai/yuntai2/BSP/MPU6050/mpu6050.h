#ifndef INC_MPU6050_H_
#define INC_MPU6050_H_

#include <stdint.h>
#include "i2c.h"

#define MPU6050_I2C_HANDLE hi2c1
#define MPU6050_I2C        (&MPU6050_I2C_HANDLE)

// MPU6050 structure
typedef struct
{

    int16_t Accel_X_RAW;
    int16_t Accel_Y_RAW;
    int16_t Accel_Z_RAW;
    double Ax;
    double Ay;
    double Az;

    int16_t Gyro_X_RAW;
    int16_t Gyro_Y_RAW;
    int16_t Gyro_Z_RAW;
    double Gx;
    double Gy;
    double Gz;

    float Temperature;

    double KalmanAngleX;
    double KalmanAngleY;
		double KalmanAngleZ;
} MPU6050_t;

// Kalman structure
typedef struct
{
    double Q_angle;
    double Q_bias;
    double R_measure;
    double angle;
    double bias;
    double P[2][2];
} Kalman_t;

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2Cx);
uint8_t MPU6050_GetLastWhoAmI(void);
void MPU6050_SetGyroBiasRaw(double bias_x, double bias_y, double bias_z);
void MPU6050_SetGyroZDeadbandRaw(double deadband_raw);
void MPU6050_ResetAttitude(MPU6050_t *DataStruct);

HAL_StatusTypeDef MPU6050_ReadAccelYRaw(I2C_HandleTypeDef *I2Cx, int16_t *raw_y);

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct);

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx,
                                   MPU6050_t *DataStruct);

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt);

#endif
