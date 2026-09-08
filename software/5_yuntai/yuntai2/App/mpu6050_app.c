#include "mpu6050_app.h"

#include <math.h>
#include <stdint.h>

#define MPU6050_APP_CALIBRATION_DURATION_MS 2000U
#define MPU6050_APP_CALIBRATION_INTERVAL_MS 10U
#define MPU6050_APP_ACCEL_NORM_MIN_G    0.80
#define MPU6050_APP_ACCEL_NORM_MAX_G    1.20
#define MPU6050_APP_ACCEL_STDDEV_MAX_G  0.08
#define MPU6050_APP_GYRO_STDDEV_MAX_RAW 250.0
#define MPU6050_APP_YAW_DEADBAND_MIN_RAW 3.0
#define MPU6050_APP_YAW_DEADBAND_MAX_RAW 12.0

static MPU6050_AppState mpu6050_app_state;
static int64_t gyro_sum_x;
static int64_t gyro_sum_y;
static int64_t gyro_sum_z;
static uint64_t gyro_square_sum_x;
static uint64_t gyro_square_sum_y;
static uint64_t gyro_square_sum_z;
static double accel_norm_sum;
static double accel_norm_square_sum;

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

static void MPU6050_App_ResetCalibration(void)
{
  gyro_sum_x = 0;
  gyro_sum_y = 0;
  gyro_sum_z = 0;
  gyro_square_sum_x = 0U;
  gyro_square_sum_y = 0U;
  gyro_square_sum_z = 0U;
  accel_norm_sum = 0.0;
  accel_norm_square_sum = 0.0;
  mpu6050_app_state.calibration_samples = 0U;
}

static void MPU6050_App_BeginCalibration(void)
{
  MPU6050_App_ResetCalibration();
  mpu6050_app_state.calibration_active = 1U;
  mpu6050_app_state.calibration_valid = 0U;
  mpu6050_app_state.static_valid = 0U;
}

static uint8_t MPU6050_App_CalibrationIsStatic(uint16_t sample_count)
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
  mean_accel_norm = accel_norm_sum / sample_count_double;
  accel_variance = (accel_norm_square_sum / sample_count_double) -
                   (mean_accel_norm * mean_accel_norm);

  if (accel_variance < 0.0)
  {
    accel_variance = 0.0;
  }
  accel_stddev = sqrt(accel_variance);
  gyro_stddev_x = sqrt(MPU6050_App_Variance(
      gyro_sum_x, gyro_square_sum_x, sample_count));
  gyro_stddev_y = sqrt(MPU6050_App_Variance(
      gyro_sum_y, gyro_square_sum_y, sample_count));
  gyro_stddev_z = sqrt(MPU6050_App_Variance(
      gyro_sum_z, gyro_square_sum_z, sample_count));

  return (mean_accel_norm >= MPU6050_APP_ACCEL_NORM_MIN_G) &&
         (mean_accel_norm <= MPU6050_APP_ACCEL_NORM_MAX_G) &&
         (accel_stddev <= MPU6050_APP_ACCEL_STDDEV_MAX_G) &&
         (gyro_stddev_x <= MPU6050_APP_GYRO_STDDEV_MAX_RAW) &&
         (gyro_stddev_y <= MPU6050_APP_GYRO_STDDEV_MAX_RAW) &&
         (gyro_stddev_z <= MPU6050_APP_GYRO_STDDEV_MAX_RAW);
}

static void MPU6050_App_UpdateDerivedData(void)
{
  mpu6050_app_state.accel_norm_g =
      (float)MPU6050_App_AccelNormG(&mpu6050_app_state.data);
  mpu6050_app_state.roll_deg =
      (float)mpu6050_app_state.data.KalmanAngleX -
      mpu6050_app_state.roll_zero_deg;
  mpu6050_app_state.pitch_deg =
      (float)mpu6050_app_state.data.KalmanAngleY -
      mpu6050_app_state.pitch_zero_deg;
  mpu6050_app_state.yaw_deg =
      (float)mpu6050_app_state.data.KalmanAngleZ -
      mpu6050_app_state.yaw_zero_deg;
}

static void MPU6050_App_RunStartupCalibration(void)
{
  uint32_t start_tick;
  uint16_t sample_count = 0U;
  uint8_t valid_sample_seen = 0U;
  HAL_StatusTypeDef read_status = HAL_OK;
  double mean_bias_x;
  double mean_bias_y;
  double mean_bias_z;

  MPU6050_App_BeginCalibration();
  start_tick = HAL_GetTick();

  /* Keep sampling for wall-clock time, rather than stopping at a sample count. */
  while ((uint32_t)(HAL_GetTick() - start_tick) <
         MPU6050_APP_CALIBRATION_DURATION_MS)
  {
    read_status = MPU6050_Read_All(
        mpu6050_app_state.i2c, &mpu6050_app_state.data);
    mpu6050_app_state.read_status = read_status;

    if (read_status == HAL_OK)
    {
      valid_sample_seen = 1U;
      mpu6050_app_state.sample_valid = 1U;
      MPU6050_App_UpdateDerivedData();

      gyro_sum_x += mpu6050_app_state.data.Gyro_X_RAW;
      gyro_sum_y += mpu6050_app_state.data.Gyro_Y_RAW;
      gyro_sum_z += mpu6050_app_state.data.Gyro_Z_RAW;
      gyro_square_sum_x += (uint64_t)((int64_t)
          mpu6050_app_state.data.Gyro_X_RAW *
          mpu6050_app_state.data.Gyro_X_RAW);
      gyro_square_sum_y += (uint64_t)((int64_t)
          mpu6050_app_state.data.Gyro_Y_RAW *
          mpu6050_app_state.data.Gyro_Y_RAW);
      gyro_square_sum_z += (uint64_t)((int64_t)
          mpu6050_app_state.data.Gyro_Z_RAW *
          mpu6050_app_state.data.Gyro_Z_RAW);
      accel_norm_sum += mpu6050_app_state.accel_norm_g;
      accel_norm_square_sum +=
          (double)mpu6050_app_state.accel_norm_g *
          (double)mpu6050_app_state.accel_norm_g;
      sample_count++;
    }

    HAL_Delay(MPU6050_APP_CALIBRATION_INTERVAL_MS);
  }

  mpu6050_app_state.calibration_samples = sample_count;
  mpu6050_app_state.calibration_active = 0U;
  mpu6050_app_state.sample_valid = valid_sample_seen;

  if (sample_count == 0U)
  {
    mpu6050_app_state.calibration_valid = 0U;
    return;
  }

  mean_bias_x = (double)gyro_sum_x / (double)sample_count;
  mean_bias_y = (double)gyro_sum_y / (double)sample_count;
  mean_bias_z = (double)gyro_sum_z / (double)sample_count;
  mpu6050_app_state.static_valid =
      MPU6050_App_CalibrationIsStatic(sample_count);

  if (mpu6050_app_state.static_valid != 0U)
  {
    double gyro_stddev_z = sqrt(MPU6050_App_Variance(
        gyro_sum_z, gyro_square_sum_z, sample_count));
    double yaw_deadband_raw = gyro_stddev_z * 3.0;

    if (yaw_deadband_raw < MPU6050_APP_YAW_DEADBAND_MIN_RAW)
    {
      yaw_deadband_raw = MPU6050_APP_YAW_DEADBAND_MIN_RAW;
    }
    else if (yaw_deadband_raw > MPU6050_APP_YAW_DEADBAND_MAX_RAW)
    {
      yaw_deadband_raw = MPU6050_APP_YAW_DEADBAND_MAX_RAW;
    }

    MPU6050_SetGyroBiasRaw(mean_bias_x, mean_bias_y, mean_bias_z);
    MPU6050_SetGyroZDeadbandRaw(yaw_deadband_raw);
    MPU6050_ResetAttitude(&mpu6050_app_state.data);
    mpu6050_app_state.roll_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleX;
    mpu6050_app_state.pitch_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleY;
    mpu6050_app_state.yaw_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleZ;
    mpu6050_app_state.calibration_valid = 1U;
    MPU6050_App_UpdateDerivedData();
  }
  else
  {
    /* Use the measured startup average whenever at least one sample exists. */
    MPU6050_SetGyroBiasRaw(mean_bias_x, mean_bias_y, mean_bias_z);
    MPU6050_SetGyroZDeadbandRaw(MPU6050_APP_YAW_DEADBAND_MIN_RAW);
    MPU6050_ResetAttitude(&mpu6050_app_state.data);
    mpu6050_app_state.roll_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleX;
    mpu6050_app_state.pitch_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleY;
    mpu6050_app_state.yaw_zero_deg =
        (float)mpu6050_app_state.data.KalmanAngleZ;
    mpu6050_app_state.calibration_valid = 1U;
    MPU6050_App_UpdateDerivedData();
  }
}

HAL_StatusTypeDef MPU6050_App_Init(void)
{
  mpu6050_app_state.initialized = 0U;
  mpu6050_app_state.sample_valid = 0U;
  mpu6050_app_state.calibration_active = 0U;
  mpu6050_app_state.calibration_valid = 0U;
  mpu6050_app_state.static_valid = 0U;
  mpu6050_app_state.who_am_i = 0U;
  mpu6050_app_state.calibration_samples = 0U;
  mpu6050_app_state.roll_zero_deg = 0.0f;
  mpu6050_app_state.pitch_zero_deg = 0.0f;
  mpu6050_app_state.yaw_zero_deg = 0.0f;
  mpu6050_app_state.i2c = MPU6050_I2C;
  mpu6050_app_state.init_status = MPU6050_Init(MPU6050_I2C);
  mpu6050_app_state.who_am_i = MPU6050_GetLastWhoAmI();
  mpu6050_app_state.read_status = mpu6050_app_state.init_status;

  if (mpu6050_app_state.init_status == HAL_OK)
  {
    mpu6050_app_state.initialized = 1U;
  }

  if (mpu6050_app_state.initialized != 0U)
  {
    /* Allow the sensor output registers to settle before calibration. */
    HAL_Delay(100U);
    MPU6050_App_RunStartupCalibration();
  }

  return mpu6050_app_state.init_status;
}

HAL_StatusTypeDef MPU6050_App_Update(void)
{
  if (mpu6050_app_state.initialized == 0U)
  {
    mpu6050_app_state.sample_valid = 0U;
    mpu6050_app_state.read_status = HAL_ERROR;
    return mpu6050_app_state.read_status;
  }

  mpu6050_app_state.read_status = MPU6050_Read_All(
      mpu6050_app_state.i2c, &mpu6050_app_state.data);
  mpu6050_app_state.sample_valid =
      (mpu6050_app_state.read_status == HAL_OK) ? 1U : 0U;

  if (mpu6050_app_state.sample_valid == 0U)
  {
    return mpu6050_app_state.read_status;
  }

  MPU6050_App_UpdateDerivedData();

  return mpu6050_app_state.read_status;
}

const MPU6050_AppState *MPU6050_App_GetState(void)
{
  return &mpu6050_app_state;
}
