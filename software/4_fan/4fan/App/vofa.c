#include "vofa.h"
#include "motor.h"
#include <math.h>
#include <stdlib.h>
#include <string.h>

#define VOFA_FRAME_FLOAT_COUNT 2U
#define VOFA_FRAME_SIZE        (VOFA_FRAME_FLOAT_COUNT * sizeof(float) + 4U)
#define VOFA_SEND_PERIOD_MS    100U
#define VOFA_GAIN_MAX          100.0f

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
    Motor_SetPidGains(gains[0], gains[1]);
  }
}

static void VOFA_ProcessTextLine(void)
{
  char *end;
  float value;

  if (strncmp(vofaTextLine, "Kp:", 3U) != 0 &&
      strncmp(vofaTextLine, "Ki:", 3U) != 0)
  {
    return;
  }

  value = strtof(&vofaTextLine[3], &end);
  if (end == &vofaTextLine[3] || !isfinite(value) ||
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
  MotorSpeedData speed;
  float values[VOFA_FRAME_FLOAT_COUNT];
  uint8_t frame[VOFA_FRAME_SIZE];

  if ((uint32_t)(now - vofaLastSendTick) < VOFA_SEND_PERIOD_MS)
  {
    return;
  }
  vofaLastSendTick = now;

  Motor_GetSpeedData(&speed);
  values[0] = (float)speed.rpm;
  values[1] = (float)speed.targetRpm;
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
    if (vofaRxByte == 'K')
    {
      vofaTextLine[0] = 'K';
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
