#include "mpu6050_app.h"
#include "mpu6050.h"

#include <math.h>
#include <stdint.h>

#define MPU6050_APP_CALIBRATION_DURATION_MS 5000U
#define MPU6050_APP_CALIBRATION_INTERVAL_MS 10U
#define MPU6050_APP_ACCEL_NORM_MIN_G    0.80
#define MPU6050_APP_ACCEL_NORM_MAX_G    1.20
#define MPU6050_APP_ACCEL_STDDEV_MAX_G  0.08
#define MPU6050_APP_GYRO_STDDEV_MAX_RAW 250.0
#define MPU6050_APP_YAW_DEADBAND_MIN_RAW 3.0
#define MPU6050_APP_YAW_DEADBAND_MAX_RAW 12.0

static MPU6050_AppState mpu6050_app_state;
static MPU6050_t mpu6050_data;
static float roll_zero_deg;
static float pitch_zero_deg;
static float yaw_zero_deg;

typedef struct
{
  int64_t gyro_sum_x;
  int64_t gyro_sum_y;
  int64_t gyro_sum_z;
  uint64_t gyro_square_sum_x;
  uint64_t gyro_square_sum_y;
  uint64_t gyro_square_sum_z;
  double accel_norm_sum;
  double accel_norm_square_sum;
} MPU6050_CalibrationStats;

static double MPU6050_App_AccelNormG(const MPU6050_t *data)
{
  double accel_x;
  double accel_y;
  double accel_z;

  accel_x = (double)data->Accel_X_RAW / 16384.0;
  accel_y = (double)data->Accel_Y_RAW / 16384.0;
  accel_z = (double)data->Accel_Z_RAW / 16384.0;
  return sqrt(accel_x * accel_x + accel_y * accel_y + accel_z * accel_z);
}

static double MPU6050_App_Variance(int64_t sum, uint64_t square_sum,
                                   uint16_t sample_count)
{
  double mean;
  double variance;

  mean = (double)sum / (double)sample_count;
  variance = (double)square_sum / (double)sample_count - mean * mean;
  return variance > 0.0 ? variance : 0.0;
}

static uint8_t MPU6050_App_CalibrationIsStatic(
    const MPU6050_CalibrationStats *stats, uint16_t sample_count)
{
  double sample_count_double;
  double mean_accel_norm;
  double accel_variance;
  double accel_stddev;
  double gyro_stddev_x;
  double gyro_stddev_y;
  double gyro_stddev_z;

  if (sample_count == 0U)
  {
    return 0U;
  }

  sample_count_double = (double)sample_count;
  mean_accel_norm = stats->accel_norm_sum / sample_count_double;
  accel_variance = (stats->accel_norm_square_sum / sample_count_double) -
                   (mean_accel_norm * mean_accel_norm);

  if (accel_variance < 0.0)
  {
    accel_variance = 0.0;
  }
  accel_stddev = sqrt(accel_variance);
  gyro_stddev_x = sqrt(MPU6050_App_Variance(
      stats->gyro_sum_x, stats->gyro_square_sum_x, sample_count));
  gyro_stddev_y = sqrt(MPU6050_App_Variance(
      stats->gyro_sum_y, stats->gyro_square_sum_y, sample_count));
  gyro_stddev_z = sqrt(MPU6050_App_Variance(
      stats->gyro_sum_z, stats->gyro_square_sum_z, sample_count));

  return (mean_accel_norm >= MPU6050_APP_ACCEL_NORM_MIN_G) &&
         (mean_accel_norm <= MPU6050_APP_ACCEL_NORM_MAX_G) &&
         (accel_stddev <= MPU6050_APP_ACCEL_STDDEV_MAX_G) &&
         (gyro_stddev_x <= MPU6050_APP_GYRO_STDDEV_MAX_RAW) &&
         (gyro_stddev_y <= MPU6050_APP_GYRO_STDDEV_MAX_RAW) &&
         (gyro_stddev_z <= MPU6050_APP_GYRO_STDDEV_MAX_RAW);
}

static void MPU6050_App_UpdateAngles(void)
{
  mpu6050_app_state.roll_deg =
      (float)mpu6050_data.KalmanAngleX - roll_zero_deg;
  mpu6050_app_state.pitch_deg =
      (float)mpu6050_data.KalmanAngleY - pitch_zero_deg;
  mpu6050_app_state.yaw_deg =
      (float)mpu6050_data.KalmanAngleZ - yaw_zero_deg;
}

static void MPU6050_App_AccumulateCalibrationSample(
    MPU6050_CalibrationStats *stats)
{
  double accel_norm_g = MPU6050_App_AccelNormG(&mpu6050_data);

  stats->gyro_sum_x += mpu6050_data.Gyro_X_RAW;
  stats->gyro_sum_y += mpu6050_data.Gyro_Y_RAW;
  stats->gyro_sum_z += mpu6050_data.Gyro_Z_RAW;
  stats->gyro_square_sum_x += (uint64_t)((int64_t)
      mpu6050_data.Gyro_X_RAW * mpu6050_data.Gyro_X_RAW);
  stats->gyro_square_sum_y += (uint64_t)((int64_t)
      mpu6050_data.Gyro_Y_RAW * mpu6050_data.Gyro_Y_RAW);
  stats->gyro_square_sum_z += (uint64_t)((int64_t)
      mpu6050_data.Gyro_Z_RAW * mpu6050_data.Gyro_Z_RAW);
  stats->accel_norm_sum += accel_norm_g;
  stats->accel_norm_square_sum += accel_norm_g * accel_norm_g;
}

static void MPU6050_App_ResetInitialAttitude(void)
{
  MPU6050_ResetAttitude(&mpu6050_data);
  roll_zero_deg = (float)mpu6050_data.KalmanAngleX;
  pitch_zero_deg = (float)mpu6050_data.KalmanAngleY;
  yaw_zero_deg = (float)mpu6050_data.KalmanAngleZ;
  MPU6050_App_UpdateAngles();
}

static void MPU6050_App_RunStartupCalibration(void)
{
  uint32_t start_tick = HAL_GetTick();
  uint16_t sample_count = 0U;
  MPU6050_CalibrationStats stats = {0};

  /* Keep sampling for wall-clock time, rather than stopping at a sample count. */
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         MPU6050_APP_CALIBRATION_DURATION_MS)
  {
    if (MPU6050_Read_All(MPU6050_I2C, &mpu6050_data) == HAL_OK)
    {
      MPU6050_App_AccumulateCalibrationSample(&stats);
      sample_count++;
    }

    HAL_Delay(MPU6050_APP_CALIBRATION_INTERVAL_MS);
  }

  mpu6050_app_state.sample_valid = (sample_count != 0U) ? 1U : 0U;
  if (sample_count == 0U)
  {
    return;
  }

  MPU6050_SetGyroBiasRaw(
      (double)stats.gyro_sum_x / (double)sample_count,
      (double)stats.gyro_sum_y / (double)sample_count,
      (double)stats.gyro_sum_z / (double)sample_count);

  {
    double yaw_deadband_raw = MPU6050_APP_YAW_DEADBAND_MIN_RAW;

    if (MPU6050_App_CalibrationIsStatic(&stats, sample_count) != 0U)
    {
      yaw_deadband_raw = sqrt(MPU6050_App_Variance(
          stats.gyro_sum_z, stats.gyro_square_sum_z, sample_count)) * 3.0;
      if (yaw_deadband_raw < MPU6050_APP_YAW_DEADBAND_MIN_RAW)
      {
        yaw_deadband_raw = MPU6050_APP_YAW_DEADBAND_MIN_RAW;
      }
      else if (yaw_deadband_raw > MPU6050_APP_YAW_DEADBAND_MAX_RAW)
      {
        yaw_deadband_raw = MPU6050_APP_YAW_DEADBAND_MAX_RAW;
      }
    }

    MPU6050_SetGyroZDeadbandRaw(yaw_deadband_raw);
  }

  MPU6050_App_ResetInitialAttitude();
}

HAL_StatusTypeDef MPU6050_App_Init(void)
{
  HAL_StatusTypeDef status;

  mpu6050_app_state = (MPU6050_AppState){0};
  mpu6050_data = (MPU6050_t){0};
  roll_zero_deg = 0.0f;
  pitch_zero_deg = 0.0f;
  yaw_zero_deg = 0.0f;

  status = MPU6050_Init(MPU6050_I2C);
  if (status != HAL_OK)
  {
    return status;
  }

  mpu6050_app_state.initialized = 1U;
  /* Allow the sensor output registers to settle before calibration. */
  HAL_Delay(100U);
  MPU6050_App_RunStartupCalibration();

  return HAL_OK;
}

HAL_StatusTypeDef MPU6050_App_Update(void)
{
  HAL_StatusTypeDef status;

  if (mpu6050_app_state.initialized == 0U)
  {
    mpu6050_app_state.sample_valid = 0U;
    return HAL_ERROR;
  }

  status = MPU6050_Read_All(MPU6050_I2C, &mpu6050_data);
  mpu6050_app_state.sample_valid =
      (status == HAL_OK) ? 1U : 0U;

  if (status == HAL_OK)
  {
    MPU6050_App_UpdateAngles();
  }

  return status;
}

const MPU6050_AppState *MPU6050_App_GetState(void)
{
  return &mpu6050_app_state;
}
