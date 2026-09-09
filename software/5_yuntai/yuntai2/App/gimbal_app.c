#include "gimbal_app.h"
#include "bt_link.h"
#include "key.h"

static GimbalMode g_mode;

/* 电位器一阶低通状态（Q8 定点，避免整数截断造成的收敛死区） */
static int32_t g_adc_filter_q8[SERVO_CHANNEL_COUNT];
static uint8_t g_adc_primed[SERVO_CHANNEL_COUNT];

/* 进入陀螺仪模式时的姿态基准与脉宽基准 */
static float g_gyro_yaw_base;
static float g_gyro_pitch_base;
static uint16_t g_gyro_base_us[SERVO_CHANNEL_COUNT];

static uint8_t g_tx_seq;

static uint16_t Gimbal_ClampUs(int32_t us)
{
  if (us < (int32_t)SERVO_PULSE_MIN_US)
  {
    return SERVO_PULSE_MIN_US;
  }
  if (us > (int32_t)SERVO_PULSE_MAX_US)
  {
    return SERVO_PULSE_MAX_US;
  }

  return (uint16_t)us;
}

static uint16_t Gimbal_AdcLowPass(ServoChannel channel, uint16_t raw)
{
  int32_t error_q8;

  if (g_adc_primed[channel] == 0U)
  {
    /* 第一帧直接置位，避免从上电初值缓慢爬升 */
    g_adc_filter_q8[channel] = (int32_t)raw << 8;
    g_adc_primed[channel] = 1U;
    return raw;
  }

  error_q8 = ((int32_t)raw << 8) - g_adc_filter_q8[channel];
  g_adc_filter_q8[channel] += error_q8 >> GIMBAL_ADC_FILTER_SHIFT;

  return (uint16_t)((g_adc_filter_q8[channel] + (1 << 7)) >> 8);
}

static uint16_t Gimbal_PotToUs(ServoChannel channel, uint16_t raw)
{
  const int32_t span = (int32_t)GIMBAL_ADC_RAW_MAX - (int32_t)GIMBAL_ADC_RAW_MIN;
  int32_t position = (int32_t)raw - (int32_t)GIMBAL_ADC_RAW_MIN;
  uint8_t invert = (channel == SERVO_CHANNEL_HORIZONTAL) ? GIMBAL_HORIZONTAL_INVERT
                                                         : GIMBAL_PITCH_INVERT;

  if (position < 0)
  {
    position = 0;
  }
  else if (position > span)
  {
    position = span;
  }

  if (invert != 0U)
  {
    position = span - position;
  }

  return (uint16_t)((int32_t)SERVO_PULSE_MIN_US +
                    (position * ((int32_t)SERVO_PULSE_MAX_US -
                                 (int32_t)SERVO_PULSE_MIN_US)) / span);
}

static float Gimbal_Sign(uint8_t invert)
{
  return (invert != 0U) ? -1.0f : 1.0f;
}

static uint16_t Gimbal_SlewLimit(uint16_t current, uint16_t wanted)
{
  int32_t difference = (int32_t)wanted - (int32_t)current;
  const int32_t limit = (int32_t)GIMBAL_SLEW_US_PER_TICK;

  if (difference > limit)
  {
    difference = limit;
  }
  else if (difference < -limit)
  {
    difference = -limit;
  }

  return (uint16_t)((int32_t)current + difference);
}

static void Gimbal_ResetAdcFilter(void)
{
  uint32_t i;

  for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
  {
    g_adc_primed[i] = 0U;
  }
}

static void Gimbal_CaptureGyroBase(const MPU6050_AppState *mpu)
{
  g_gyro_yaw_base = mpu->yaw_deg;
  g_gyro_pitch_base = mpu->pitch_deg;
  /* 脉宽基准取当前目标，保证切换模式时舵机不跳变 */
  g_gyro_base_us[SERVO_CHANNEL_HORIZONTAL] =
      Servo_GetPulseUs(SERVO_CHANNEL_HORIZONTAL);
  g_gyro_base_us[SERVO_CHANNEL_PITCH] = Servo_GetPulseUs(SERVO_CHANNEL_PITCH);
}

void Gimbal_Init(void)
{
  uint32_t i;

  g_mode = GIMBAL_MODE_POTENTIOMETER;
  g_tx_seq = 0U;
  g_gyro_yaw_base = 0.0f;
  g_gyro_pitch_base = 0.0f;

  for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
  {
    g_gyro_base_us[i] = SERVO_PULSE_CENTER_US;
    g_adc_primed[i] = 0U;
  }
}

void Gimbal_Update(const ADC_InputData *adc, const MPU6050_AppState *mpu)
{
  uint16_t raw[SERVO_CHANNEL_COUNT];
  uint16_t wanted[SERVO_CHANNEL_COUNT];
  BtLinkFrame frame;
  uint8_t gyro_ok;
  uint32_t i;

  if ((adc == NULL) || (mpu == NULL))
  {
    return;
  }

  raw[SERVO_CHANNEL_HORIZONTAL] = adc->horizontal_raw;
  raw[SERVO_CHANNEL_PITCH] = adc->pitch_raw;

  gyro_ok = ((mpu->initialized != 0U) && (mpu->sample_valid != 0U)) ? 1U : 0U;

  /* 陀螺仪失效时自动降级回电位器模式，避免目标角失控 */
  if ((g_mode == GIMBAL_MODE_GYRO) && (gyro_ok == 0U))
  {
    g_mode = GIMBAL_MODE_POTENTIOMETER;
    Gimbal_ResetAdcFilter();
  }

  /* --- 按键 --- */
  if (KEY_GetEvent(KEY_ID_1) == KEY_EVENT_PRESSED)
  {
    if (g_mode == GIMBAL_MODE_POTENTIOMETER)
    {
      /* 陀螺仪不可用时拒绝进入，避免云台乱动 */
      if (gyro_ok != 0U)
      {
        g_mode = GIMBAL_MODE_GYRO;
        Gimbal_CaptureGyroBase(mpu);
      }
    }
    else
    {
      g_mode = GIMBAL_MODE_POTENTIOMETER;
      Gimbal_ResetAdcFilter();
    }
  }

  if ((KEY_GetEvent(KEY_ID_2) == KEY_EVENT_PRESSED) &&
      (g_mode == GIMBAL_MODE_GYRO) && (gyro_ok != 0U))
  {
    Gimbal_CaptureGyroBase(mpu);
  }

  /* --- 目标脉宽 --- */
  if ((g_mode == GIMBAL_MODE_GYRO) && (gyro_ok != 0U))
  {
    float delta_yaw = mpu->yaw_deg - g_gyro_yaw_base;
    float delta_pitch = mpu->pitch_deg - g_gyro_pitch_base;
    float horizontal_us;
    float pitch_us;

    horizontal_us = (float)g_gyro_base_us[SERVO_CHANNEL_HORIZONTAL] +
                    (delta_yaw * GIMBAL_GYRO_US_PER_DEG *
                     Gimbal_Sign(GIMBAL_HORIZONTAL_INVERT));
    pitch_us = (float)g_gyro_base_us[SERVO_CHANNEL_PITCH] +
               (delta_pitch * GIMBAL_GYRO_US_PER_DEG *
                Gimbal_Sign(GIMBAL_PITCH_INVERT));

    wanted[SERVO_CHANNEL_HORIZONTAL] = Gimbal_ClampUs((int32_t)horizontal_us);
    wanted[SERVO_CHANNEL_PITCH] = Gimbal_ClampUs((int32_t)pitch_us);
  }
  else
  {
    for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
    {
      wanted[i] = Gimbal_PotToUs((ServoChannel)i,
                                 Gimbal_AdcLowPass((ServoChannel)i, raw[i]));
    }
  }

  /* --- 速率限制 + 本地舵机输出 --- */
  for (i = 0U; i < (uint32_t)SERVO_CHANNEL_COUNT; i++)
  {
    Servo_SetPulseUs((ServoChannel)i,
                     Gimbal_SlewLimit(Servo_GetPulseUs((ServoChannel)i),
                                      wanted[i]));
  }

  /* --- 蓝牙发送：每个节拍一帧（主循环 10ms 一次 -> 100Hz） --- */
  frame.seq = g_tx_seq;
  g_tx_seq++;
  frame.mode = (uint8_t)g_mode;
  frame.ch1_us = Servo_GetPulseUs(SERVO_CHANNEL_HORIZONTAL);
  frame.ch2_us = Servo_GetPulseUs(SERVO_CHANNEL_PITCH);
  (void)BtLink_Send(&frame);
}

GimbalMode Gimbal_GetMode(void)
{
  return g_mode;
}
