#include "bt_link.h"
#include "usart.h"

/* 发送超时保护：超过该时间仍未收到 TX 完成回调就强制复位忙标志 */
#define BT_LINK_TX_TIMEOUT_MS 50U

/* 解码缓冲区：LEN + 载荷 + CRC */
#define BT_LINK_RX_BUF_LEN    (BT_LINK_PAYLOAD_LEN + 2U)

typedef enum
{
  BT_DECODE_WAIT_HEADER_0 = 0,
  BT_DECODE_WAIT_HEADER_1,
  BT_DECODE_WAIT_LENGTH,
  BT_DECODE_PAYLOAD
} BtDecodeState;

static uint8_t g_tx_buffer[BT_LINK_FRAME_LEN];
static volatile uint8_t g_tx_busy;
static uint32_t g_tx_start_tick;

static BtDecodeState g_decode_state;
static uint8_t g_decode_buffer[BT_LINK_RX_BUF_LEN];
static uint8_t g_decode_index;

uint8_t BtLink_Crc8(const uint8_t *data, uint8_t length)
{
  uint8_t crc = 0U;
  uint8_t i;
  uint8_t bit;

  if (data == NULL)
  {
    return 0U;
  }

  for (i = 0U; i < length; i++)
  {
    crc ^= data[i];
    for (bit = 0U; bit < 8U; bit++)
    {
      if ((crc & 0x80U) != 0U)
      {
        crc = (uint8_t)((crc << 1) ^ 0x07U);
      }
      else
      {
        crc = (uint8_t)(crc << 1);
      }
    }
  }

  return crc;
}

uint8_t BtLink_Encode(const BtLinkFrame *frame, uint8_t *out, uint8_t out_size)
{
  if ((frame == NULL) || (out == NULL) || (out_size < BT_LINK_FRAME_LEN))
  {
    return 0U;
  }

  out[0] = BT_LINK_HEADER_0;
  out[1] = BT_LINK_HEADER_1;
  out[2] = BT_LINK_PAYLOAD_LEN;
  out[3] = frame->seq;
  out[4] = frame->mode;
  out[5] = (uint8_t)(frame->ch1_us >> 8);
  out[6] = (uint8_t)(frame->ch1_us & 0xFFU);
  out[7] = (uint8_t)(frame->ch2_us >> 8);
  out[8] = (uint8_t)(frame->ch2_us & 0xFFU);
  out[9] = BtLink_Crc8(&out[2], BT_LINK_PAYLOAD_LEN + 1U);

  return BT_LINK_FRAME_LEN;
}

void BtLink_Init(void)
{
  g_tx_busy = 0U;
  g_tx_start_tick = 0U;
  BtLink_DecodeReset();
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

  if (BtLink_Encode(frame, g_tx_buffer, BT_LINK_FRAME_LEN) == 0U)
  {
    return 0U;
  }

  g_tx_busy = 1U;
  g_tx_start_tick = HAL_GetTick();

  if (HAL_UART_Transmit_IT(&huart3, g_tx_buffer, BT_LINK_FRAME_LEN) != HAL_OK)
  {
    g_tx_busy = 0U;
    return 0U;
  }

  return 1U;
}

void BtLink_DecodeReset(void)
{
  g_decode_state = BT_DECODE_WAIT_HEADER_0;
  g_decode_index = 0U;
}

uint8_t BtLink_DecodeByte(uint8_t byte, BtLinkFrame *out)
{
  if (out == NULL)
  {
    return 0U;
  }

  switch (g_decode_state)
  {
    case BT_DECODE_WAIT_HEADER_0:
      if (byte == BT_LINK_HEADER_0)
      {
        g_decode_state = BT_DECODE_WAIT_HEADER_1;
      }
      break;

    case BT_DECODE_WAIT_HEADER_1:
      if (byte == BT_LINK_HEADER_1)
      {
        g_decode_state = BT_DECODE_WAIT_LENGTH;
      }
      else if (byte != BT_LINK_HEADER_0)
      {
        /* 不是 0xAA 0x55：重新找帧头；是 0xAA 时保持在当前状态以兼容 0xAA 0xAA 0x55 */
        g_decode_state = BT_DECODE_WAIT_HEADER_0;
      }
      break;

    case BT_DECODE_WAIT_LENGTH:
      if (byte == BT_LINK_PAYLOAD_LEN)
      {
        g_decode_buffer[0] = byte;
        g_decode_index = 1U;
        g_decode_state = BT_DECODE_PAYLOAD;
      }
      else
      {
        g_decode_state = BT_DECODE_WAIT_HEADER_0;
      }
      break;

    case BT_DECODE_PAYLOAD:
      g_decode_buffer[g_decode_index] = byte;
      g_decode_index++;

      if (g_decode_index >= BT_LINK_RX_BUF_LEN)
      {
        g_decode_state = BT_DECODE_WAIT_HEADER_0;

        if (BtLink_Crc8(g_decode_buffer, BT_LINK_PAYLOAD_LEN + 1U) ==
            g_decode_buffer[BT_LINK_PAYLOAD_LEN + 1U])
        {
          out->seq = g_decode_buffer[1];
          out->mode = g_decode_buffer[2];
          out->ch1_us = ((uint16_t)g_decode_buffer[3] << 8) |
                        (uint16_t)g_decode_buffer[4];
          out->ch2_us = ((uint16_t)g_decode_buffer[5] << 8) |
                        (uint16_t)g_decode_buffer[6];
          return 1U;
        }
      }
      break;

    default:
      BtLink_DecodeReset();
      break;
  }

  return 0U;
}

void HAL_UART_TxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART3))
  {
    g_tx_busy = 0U;
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART3))
  {
    g_tx_busy = 0U;
  }
}
