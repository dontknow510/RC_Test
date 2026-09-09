#include "slave_app.h"
#include "bt_link.h"
#include "servo_app.h"
#include "usart.h"

static uint8_t g_rx_byte;

static void Slave_ArmReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
}

void Slave_Init(void)
{
  (void)Servo_Init();
  BtLink_DecodeReset();
  Slave_ArmReceive();
}

void Slave_OnByte(uint8_t byte)
{
  BtLinkFrame frame;

  if (BtLink_DecodeByte(byte, &frame) == 0U)
  {
    return;
  }

  /* 只有帧头、长度、CRC 全部正确才会走到这里，脉宽限幅在 servo_app 内部 */
  Servo_SetPulseUs(SERVO_CHANNEL_HORIZONTAL, frame.ch1_us);
  Servo_SetPulseUs(SERVO_CHANNEL_PITCH, frame.ch2_us);
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    Slave_OnByte(g_rx_byte);
    Slave_ArmReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    /* 噪声/帧错误/溢出后重新武装接收，避免链路卡死 */
    Slave_ArmReceive();
  }
}