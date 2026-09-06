#include "vofa.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_FRAME_FLOAT_COUNT 2U
#define VOFA_FRAME_SIZE        (VOFA_FRAME_FLOAT_COUNT * sizeof(float) + 4U)
#define VOFA_SEND_PERIOD_MS    100U
#define VOFA_GAIN_MAX          100.0f
#define VOFA_POSITION_MAX_RPM  300L
#define VOFA_POSITION_MAX_COUNT 1456L

static UART_HandleTypeDef *vofaUart;
static uint8_t vofaRxByte;
static uint8_t vofaRxFrame[VOFA_FRAME_SIZE];
static uint8_t vofaRxLength;
static char vofaTextLine[24];
static uint8_t vofaTextLength;
static uint32_t vofaLastSendTick;

static uint8_t VOFA_IsFrameTail(const uint8_t *frame)
{
  return frame[VOFA_FRAME_SIZE - 4U] == 0x00U &&
         frame[VOFA_FRAME_SIZE - 3U] == 0x00U &&
         frame[VOFA_FRAME_SIZE - 2U] == 0x80U &&
         frame[VOFA_FRAME_SIZE - 1U] == 0x7FU;
}

static uint8_t VOFA_IsValueEnd(const char *text)
{
  return *text == '\0' || *text == '\r';
}

static uint8_t VOFA_ParseFloat(const char *text, float *value)
{
  char *end;

  *value = strtof(text, &end);
  return end != text && VOFA_IsValueEnd(end) && isfinite(*value);
}

static uint8_t VOFA_ParseLong(const char *text, long *value)
{
  char *end;

  *value = strtol(text, &end, 10);
  return end != text && VOFA_IsValueEnd(end);
}

static void VOFA_ProcessFrame(void)
{
  float gains[VOFA_FRAME_FLOAT_COUNT];

  if (!VOFA_IsFrameTail(vofaRxFrame))
  {
    return;
  }

  memcpy(gains, vofaRxFrame, sizeof(gains));
  if (isfinite(gains[0]) && isfinite(gains[1]) &&
      gains[0] >= 0.0f && gains[0] <= VOFA_GAIN_MAX &&
      gains[1] >= 0.0f && gains[1] <= VOFA_GAIN_MAX)
  {
    if (Motor_GetMode() == MOTOR_MODE_POSITION)
    {
      Motor_SetPositionSpeedKp(gains[0]);
      Motor_SetPositionSpeedKi(gains[1]);
    }
    else if (Motor_GetMode() == MOTOR_MODE_SPEED)
    {
      Motor_SetPidGains(gains[0], gains[1]);
    }
  }
}

static void VOFA_ProcessTextLine(void)
{
  float value;
  long integerValue;

  if (strncmp(vofaTextLine, "Kp:", 3U) == 0 ||
      strncmp(vofaTextLine, "Ki:", 3U) == 0)
  {
    if (!VOFA_ParseFloat(&vofaTextLine[3], &value) ||
        value < 0.0f || value > VOFA_GAIN_MAX)
    {
      return;
    }

    if (vofaTextLine[1] == 'p')
    {
      Motor_SetKp(value);
    }
    else
    {
      Motor_SetKi(value);
    }
    return;
  }

  if (strncmp(vofaTextLine, "PosKp:", 6U) == 0)
  {
    if (VOFA_ParseFloat(&vofaTextLine[6], &value) &&
        value >= 0.0f && value <= VOFA_GAIN_MAX)
    {
      Motor_SetPositionKp(value);
    }
    return;
  }

  if (strncmp(vofaTextLine, "PosSpeedKp:", 11U) == 0)
  {
    if (VOFA_ParseFloat(&vofaTextLine[11], &value) &&
        value >= 0.0f && value <= VOFA_GAIN_MAX)
    {
      Motor_SetPositionSpeedKp(value);
    }
    return;
  }

  if (strncmp(vofaTextLine, "PosSpeedKi:", 11U) == 0)
  {
    if (VOFA_ParseFloat(&vofaTextLine[11], &value) &&
        value >= 0.0f && value <= VOFA_GAIN_MAX)
    {
      Motor_SetPositionSpeedKi(value);
    }
    return;
  }

  if (strncmp(vofaTextLine, "PosMaxRpm:", 10U) == 0)
  {
    if (VOFA_ParseLong(&vofaTextLine[10], &integerValue) &&
        integerValue >= 0L && integerValue <= VOFA_POSITION_MAX_RPM)
    {
      Motor_SetPositionMaxRpm((uint16_t)integerValue);
    }
  }
  else if (strncmp(vofaTextLine, "PosDeadband:", 12U) == 0)
  {
    if (VOFA_ParseLong(&vofaTextLine[12], &integerValue) &&
        integerValue >= 0L && integerValue <= VOFA_POSITION_MAX_COUNT)
    {
      Motor_SetPositionDeadband((int32_t)integerValue);
    }
  }
}

void VOFA_Init(UART_HandleTypeDef *uart)
{
  vofaUart = uart;
  vofaRxLength = 0U;
  vofaTextLength = 0U;
  vofaLastSendTick = HAL_GetTick();
  (void)HAL_UART_Receive_IT(vofaUart, &vofaRxByte, 1U);
}

void VOFA_MainLoopUpdate(void)
{
  uint32_t now = HAL_GetTick();
  MotorMode mode;
  MotorSpeedData speed;
  MotorPositionData position;
  float values[VOFA_FRAME_FLOAT_COUNT];
  uint8_t frame[VOFA_FRAME_SIZE];

  if ((uint32_t)(now - vofaLastSendTick) < VOFA_SEND_PERIOD_MS)
  {
    return;
  }
  vofaLastSendTick = now;

  mode = Motor_GetMode();
  if (mode == MOTOR_MODE_SPEED)
  {
    Motor_GetSpeedData(&speed);
    values[0] = (float)speed.rpm;
    values[1] = (float)speed.targetRpm;
  }
  else if (mode == MOTOR_MODE_POSITION)
  {
    Motor_GetPositionData(&position);
    values[0] = (float)position.positionAngle;
    values[1] = (float)position.targetAngle;
  }
  else
  {
    return;
  }
  memcpy(frame, values, sizeof(values));
  frame[8] = 0x00U;
  frame[9] = 0x00U;
  frame[10] = 0x80U;
  frame[11] = 0x7FU;
  (void)HAL_UART_Transmit(vofaUart, frame, sizeof(frame), 10U);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart == vofaUart)
  {
    if (vofaTextLength == 0U &&
        (vofaRxByte == 'K' || vofaRxByte == 'P'))
    {
      vofaTextLine[0] = (char)vofaRxByte;
      vofaTextLength = 1U;
    }
    else if (vofaTextLength != 0U && vofaRxByte == '\n')
    {
      vofaTextLine[vofaTextLength] = '\0';
      VOFA_ProcessTextLine();
      vofaTextLength = 0U;
    }
    else if (vofaTextLength != 0U &&
             vofaTextLength < (uint8_t)(sizeof(vofaTextLine) - 1U) &&
             vofaRxByte >= 0x20U && vofaRxByte <= 0x7EU)
    {
      vofaTextLine[vofaTextLength++] = (char)vofaRxByte;
    }
    else if (vofaTextLength != 0U)
    {
      vofaTextLength = 0U;
    }

    if (vofaRxLength < VOFA_FRAME_SIZE)
    {
      vofaRxFrame[vofaRxLength++] = vofaRxByte;
    }

    if (vofaRxLength == VOFA_FRAME_SIZE)
    {
      VOFA_ProcessFrame();
      if (!VOFA_IsFrameTail(vofaRxFrame))
      {
        memmove(vofaRxFrame, &vofaRxFrame[1], VOFA_FRAME_SIZE - 1U);
        vofaRxLength = VOFA_FRAME_SIZE - 1U;
      }
      else
      {
        vofaRxLength = 0U;
      }
    }

    (void)HAL_UART_Receive_IT(vofaUart, &vofaRxByte, 1U);
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart == vofaUart)
  {
    vofaRxLength = 0U;
    (void)HAL_UART_Receive_IT(vofaUart, &vofaRxByte, 1U);
  }
}
