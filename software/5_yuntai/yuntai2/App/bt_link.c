#include "bt_link.h"
#include "usart.h"

/* 发送卡死保护：超过该时间仍未收到 TX 完成回调就强制复位忙标志 */
#define BT_LINK_TX_TIMEOUT_MS 50U

static uint8_t g_tx_buffer[BT_LINK_FRAME_LEN];
static volatile uint8_t g_tx_busy;
static uint32_t g_tx_start_tick;

static uint8_t BtLink_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t i;
  uint8_t bit;

  for (i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      crc = ((crc & 0x80U) != 0U) ? (uint8_t)((crc << 1) ^ 0x07U)
                                  : (uint8_t)(crc << 1);
    }
  }

  return crc;
}

static void BtLink_Encode(const BtLinkFrame *frame)
{
  g_tx_buffer[0] = BT_LINK_HEADER_0;
  g_tx_buffer[1] = BT_LINK_HEADER_1;
  g_tx_buffer[2] = BT_LINK_PAYLOAD_LEN;
  g_tx_buffer[3] = frame->seq;
  g_tx_buffer[4] = frame->mode;
  g_tx_buffer[5] = (uint8_t)(frame->ch1_us >> 8);
  g_tx_buffer[6] = (uint8_t)frame->ch1_us;
  g_tx_buffer[7] = (uint8_t)(frame->ch2_us >> 8);
  g_tx_buffer[8] = (uint8_t)frame->ch2_us;
  g_tx_buffer[9] = BtLink_Crc8(&g_tx_buffer[2], BT_LINK_PAYLOAD_LEN + 1U);
}

void BtLink_Init(void)
{
  g_tx_busy = 0U;
  g_tx_start_tick = 0U;
}

uint8_t BtLink_Send(const BtLinkFrame *frame)
{
  if (frame == NULL)
  {
    return 0U;
  }

  if (g_tx_busy != 0U)
  {
    if ((uint32_t)(HAL_GetTick() - g_tx_start_tick) < BT_LINK_TX_TIMEOUT_MS)
    {
      return 0U;
    }

    /* 超时：强制清理，避免一次异常把链路永久卡死 */
    (void)HAL_UART_AbortTransmit(&huart3);
    g_tx_busy = 0U;
  }

  BtLink_Encode(frame);

  g_tx_busy = 1U;
  g_tx_start_tick = HAL_GetTick();

  if (HAL_UART_Transmit_IT(&huart3, g_tx_buffer, BT_LINK_FRAME_LEN) != HAL_OK)
  {
    g_tx_busy = 0U;
    return 0U;
  }

  return 1U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    g_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART3)
  {
    g_tx_busy = 0U;
  }
}
