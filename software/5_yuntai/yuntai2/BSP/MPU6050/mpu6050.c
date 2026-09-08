/*By 楂樿繃_Gaoguo*/
#include <math.h>
#include "mpu6050.h"
#define RAD_TO_DEG 57.295779513082320876798154814105
//寮у害杞搴?rad = 57.2958掳
#define RAD_TO_DEG 57.295779513082320876798154814105

#define WHO_AM_I_REG 0x75
#define PWR_MGMT_1_REG 0x6B
#define SMPLRT_DIV_REG 0x19
#define CONFIG_REG 0x1A
#define ACCEL_CONFIG_REG 0x1C
#define ACCEL_XOUT_H_REG 0x3B
#define ACCEL_YOUT_H_REG 0x3D
#define TEMP_OUT_H_REG 0x41
#define GYRO_CONFIG_REG 0x1B
#define GYRO_XOUT_H_REG 0x43

#define MPU6050_WHO_AM_I_VALUE 0x68U
#define MPU6500_WHO_AM_I_VALUE 0x70U

// Setup MPU6050

#define MPU6050_ADDR 0xD0
const uint16_t i2c_timeout = 100;
const double Accel_Z_corrector = 14418.0;
uint32_t timer;
static double yaw_angle;
static uint8_t last_who_am_i;
static double gyro_bias_x_raw;
static double gyro_bias_y_raw;
static double gyro_bias_z_raw;
static double gyro_z_deadband_raw;

static uint8_t MPU6050_IsSupportedWhoAmI(uint8_t who_am_i)
{
    return (who_am_i == MPU6050_WHO_AM_I_VALUE) ||
           (who_am_i == MPU6500_WHO_AM_I_VALUE);
}

Kalman_t KalmanX = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f
};
Kalman_t KalmanY = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};
Kalman_t KalmanZ = {
    .Q_angle = 0.001f,
    .Q_bias = 0.003f,
    .R_measure = 0.03f,
};

uint8_t MPU6050_InitLegacy(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t Data;

    // check device ID WHO_AM_I

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG, 1, &check, 1, i2c_timeout);

    if (check == 104) // 0x68 will be returned by the sensor if everything goes well
    {
        // power management register 0X6B we should write all 0's to wake the sensor up
        Data = 0;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG, 1, &Data, 1, i2c_timeout);

        // Set DATA RATE of 1KHz by writing SMPLRT_DIV register
        Data = 0x07;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG, 1, &Data, 1, i2c_timeout);

        // Set accelerometer configuration in ACCEL_CONFIG Register
        // XA_ST=0,YA_ST=0,ZA_ST=0, FS_SEL=0 -> 锟?2g
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG, 1, &Data, 1, i2c_timeout);

        // Set Gyroscopic configuration in GYRO_CONFIG Register
        // XG_ST=0,YG_ST=0,ZG_ST=0, FS_SEL=0 -> 锟?250 锟?s
        Data = 0x00;
        HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG, 1, &Data, 1, i2c_timeout);
        timer = HAL_GetTick();
        return 0;
    }
    return 1;
}

HAL_StatusTypeDef MPU6050_Init(I2C_HandleTypeDef *I2Cx)
{
    uint8_t check;
    uint8_t data;
    HAL_StatusTypeDef status;
    uint8_t attempt;

    last_who_am_i = 0U;
    gyro_bias_x_raw = 0.0;
    gyro_bias_y_raw = 0.0;
    gyro_bias_z_raw = 0.0;
    gyro_z_deadband_raw = 0.0;
    yaw_angle = 0.0;
    if (I2Cx == NULL)
    {
        return HAL_ERROR;
    }

    /* Read WHO_AM_I several times so a slow-starting sensor is not rejected. */
    status = HAL_ERROR;
    for (attempt = 0U; attempt < 3U; attempt++)
    {
        check = 0U;
        status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, WHO_AM_I_REG,
                                  I2C_MEMADD_SIZE_8BIT, &check, 1U,
                                  i2c_timeout);
        if (status == HAL_OK)
        {
            last_who_am_i = check;
            /* 0x68 is MPU-6050; 0x70 is MPU-6500. */
            if (MPU6050_IsSupportedWhoAmI(check) != 0U)
            {
                break;
            }
            status = HAL_ERROR;
        }
        HAL_Delay(5U);
    }

    if ((status != HAL_OK) ||
        (MPU6050_IsSupportedWhoAmI(last_who_am_i) == 0U))
    {
        return status;
    }

    /* Match the tested reference project. Configuration writes are best effort. */
    data = 0x00U;
    (void)HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, PWR_MGMT_1_REG,
                            I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
    HAL_Delay(10U);

    data = 0x07U;
    (void)HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, SMPLRT_DIV_REG,
                            I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);

    data = 0x00U;
    (void)HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, ACCEL_CONFIG_REG,
                            I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);
    (void)HAL_I2C_Mem_Write(I2Cx, MPU6050_ADDR, GYRO_CONFIG_REG,
                            I2C_MEMADD_SIZE_8BIT, &data, 1U, i2c_timeout);

    timer = HAL_GetTick();
    return HAL_OK;
}

uint8_t MPU6050_GetLastWhoAmI(void)
{
    return last_who_am_i;
}

void MPU6050_SetGyroBiasRaw(double bias_x, double bias_y, double bias_z)
{
    gyro_bias_x_raw = bias_x;
    gyro_bias_y_raw = bias_y;
    gyro_bias_z_raw = bias_z;
}

void MPU6050_SetGyroZDeadbandRaw(double deadband_raw)
{
    gyro_z_deadband_raw = (deadband_raw > 0.0) ? deadband_raw : 0.0;
}

void MPU6050_ResetAttitude(MPU6050_t *DataStruct)
{
    double roll;
    double pitch;
    double roll_sqrt;

    if (DataStruct == NULL)
    {
        return;
    }

    roll_sqrt = sqrt((double)DataStruct->Accel_X_RAW *
                     (double)DataStruct->Accel_X_RAW +
                     (double)DataStruct->Accel_Z_RAW *
                     (double)DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
    {
        roll = atan((double)DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    }
    else
    {
        roll = 0.0;
    }

    pitch = atan2(-(double)DataStruct->Accel_X_RAW,
                  (double)DataStruct->Accel_Z_RAW) * RAD_TO_DEG;

    KalmanX.angle = roll;
    KalmanX.bias = 0.0;
    KalmanX.P[0][0] = 0.0;
    KalmanX.P[0][1] = 0.0;
    KalmanX.P[1][0] = 0.0;
    KalmanX.P[1][1] = 0.0;

    KalmanY.angle = pitch;
    KalmanY.bias = 0.0;
    KalmanY.P[0][0] = 0.0;
    KalmanY.P[0][1] = 0.0;
    KalmanY.P[1][0] = 0.0;
    KalmanY.P[1][1] = 0.0;

    KalmanZ.angle = 0.0;
    KalmanZ.bias = 0.0;
    KalmanZ.P[0][0] = 0.0;
    KalmanZ.P[0][1] = 0.0;
    KalmanZ.P[1][0] = 0.0;
    KalmanZ.P[1][1] = 0.0;

    yaw_angle = 0.0f;
    timer = HAL_GetTick();
    DataStruct->KalmanAngleX = roll;
    DataStruct->KalmanAngleY = pitch;
    DataStruct->KalmanAngleZ = 0.0;
}

HAL_StatusTypeDef MPU6050_ReadAccelYRaw(I2C_HandleTypeDef *I2Cx, int16_t *raw_y)
{
    uint8_t data[2];

    if ((I2Cx == NULL) || (raw_y == NULL))
    {
        return HAL_ERROR;
    }

    if (HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_YOUT_H_REG,
                         I2C_MEMADD_SIZE_8BIT, data, sizeof(data), i2c_timeout) != HAL_OK)
    {
        return HAL_ERROR;
    }

    *raw_y = (int16_t)(((uint16_t)data[0] << 8) | (uint16_t)data[1]);
    return HAL_OK;
}

void MPU6050_Read_Accel(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];
	
    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
}

void MPU6050_Read_Gyro(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[6];

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, GYRO_XOUT_H_REG, 1, Rec_Data, 6, i2c_timeout);

    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
	
    DataStruct->Gx = DataStruct->Gyro_X_RAW / 131.0;
    DataStruct->Gy = DataStruct->Gyro_Y_RAW / 131.0;
    DataStruct->Gz = DataStruct->Gyro_Z_RAW / 131.0;
}

void MPU6050_Read_Temp(I2C_HandleTypeDef *I2Cx, MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[2];
    int16_t temp;

    HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, TEMP_OUT_H_REG, 1, Rec_Data, 2, i2c_timeout);

    temp = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
}

double Kalman_getAngle(Kalman_t *Kalman, double newAngle, double newRate, double dt)
{
    double rate = newRate - Kalman->bias;
    double S;
    double K[2];
    double gap;
    double P00_temp;
    double P01_temp;

    Kalman->angle += dt * rate;
    Kalman->P[0][0] += dt * (dt * Kalman->P[1][1] - Kalman->P[0][1] - Kalman->P[1][0] + Kalman->Q_angle);
    Kalman->P[0][1] -= dt * Kalman->P[1][1];
    Kalman->P[1][0] -= dt * Kalman->P[1][1];
    Kalman->P[1][1] += Kalman->Q_bias * dt;

    S = Kalman->P[0][0] + Kalman->R_measure;
    K[0] = Kalman->P[0][0] / S;
    K[1] = Kalman->P[1][0] / S;

    gap = newAngle - Kalman->angle;
    Kalman->angle += K[0] * gap;
    Kalman->bias += K[1] * gap;

    P00_temp = Kalman->P[0][0];
    P01_temp = Kalman->P[0][1];
    Kalman->P[0][0] -= K[0] * P00_temp;
    Kalman->P[0][1] -= K[0] * P01_temp;
    Kalman->P[1][0] -= K[1] * P00_temp;
    Kalman->P[1][1] -= K[1] * P01_temp;
		
    return Kalman->angle;
}

HAL_StatusTypeDef MPU6050_Read_All(I2C_HandleTypeDef *I2Cx,
                                   MPU6050_t *DataStruct)
{
    uint8_t Rec_Data[14];
    int16_t temp;
    double gyro_z_corrected_raw;
    HAL_StatusTypeDef status;

    if ((I2Cx == NULL) || (DataStruct == NULL))
    {
        return HAL_ERROR;
    }

    status = HAL_I2C_Mem_Read(I2Cx, MPU6050_ADDR, ACCEL_XOUT_H_REG,
                              I2C_MEMADD_SIZE_8BIT, Rec_Data,
                              sizeof(Rec_Data), i2c_timeout);
    if (status != HAL_OK)
    {
        return status;
    }

    DataStruct->Accel_X_RAW = (int16_t)(Rec_Data[0] << 8 | Rec_Data[1]);
    DataStruct->Accel_Y_RAW = (int16_t)(Rec_Data[2] << 8 | Rec_Data[3]);
    DataStruct->Accel_Z_RAW = (int16_t)(Rec_Data[4] << 8 | Rec_Data[5]);
    temp = (int16_t)(Rec_Data[6] << 8 | Rec_Data[7]);
    DataStruct->Gyro_X_RAW = (int16_t)(Rec_Data[8] << 8 | Rec_Data[9]);
    DataStruct->Gyro_Y_RAW = (int16_t)(Rec_Data[10] << 8 | Rec_Data[11]);
    DataStruct->Gyro_Z_RAW = (int16_t)(Rec_Data[12] << 8 | Rec_Data[13]);

    DataStruct->Ax = DataStruct->Accel_X_RAW / 16384.0;
    DataStruct->Ay = DataStruct->Accel_Y_RAW / 16384.0;
    DataStruct->Az = DataStruct->Accel_Z_RAW / Accel_Z_corrector;
    DataStruct->Temperature = (float)((int16_t)temp / (float)340.0 + (float)36.53);
    DataStruct->Gx = ((double)DataStruct->Gyro_X_RAW - gyro_bias_x_raw) / 131.0;
    DataStruct->Gy = ((double)DataStruct->Gyro_Y_RAW - gyro_bias_y_raw) / 131.0;
    gyro_z_corrected_raw =
        (double)DataStruct->Gyro_Z_RAW - gyro_bias_z_raw;
    DataStruct->Gz = gyro_z_corrected_raw / 131.0;

    // Kalman angle solve
    uint32_t now = HAL_GetTick();
    double dt = (double)(now - timer) / 1000.0;
    double yaw_rate_dps = DataStruct->Gz;
    timer = now;

    if (fabs(gyro_z_corrected_raw) <= gyro_z_deadband_raw)
    {
        yaw_rate_dps = 0.0;
    }
		
    double roll;
    double roll_sqrt = sqrt(DataStruct->Accel_X_RAW * DataStruct->Accel_X_RAW + DataStruct->Accel_Z_RAW * DataStruct->Accel_Z_RAW);
    if (roll_sqrt != 0.0)
        roll = atan(DataStruct->Accel_Y_RAW / roll_sqrt) * RAD_TO_DEG;
    else
        roll = 0.0;
		
    double pitch = atan2(-DataStruct->Accel_X_RAW, DataStruct->Accel_Z_RAW) * RAD_TO_DEG;
    if ((pitch < -90 && DataStruct->KalmanAngleY > 90) || (pitch > 90 && DataStruct->KalmanAngleY < -90))
    {
        KalmanY.angle = pitch;
        DataStruct->KalmanAngleY = pitch;
    }
    else
       DataStruct->KalmanAngleY = Kalman_getAngle(&KalmanY, pitch, DataStruct->Gy, dt);
    if (fabs(DataStruct->KalmanAngleY) > 90)
        DataStruct->Gx = -DataStruct->Gx;
    DataStruct->KalmanAngleX = Kalman_getAngle(&KalmanX, roll, DataStruct->Gx, dt);
		
		// --- Yaw (Z杞?瑙掑害浼拌 ---
    // 娌℃湁纾佸姏璁℃棤娉曚粠鍔犻€熷害璁℃帹鏂璝aw瑙掞紝鍙兘闈犻檧铻轰华瑙掗€熷害绉垎
		
		yaw_angle += yaw_rate_dps * dt;
		DataStruct->KalmanAngleZ = yaw_angle;

    return HAL_OK;
}
