#include "bt_link.h"

/* 解码缓冲区：LEN + 载荷 + CRC */
#define BT_LINK_RX_BUF_LEN    (BT_LINK_PAYLOAD_LEN + 2U)

typedef enum
{
  BT_DECODE_WAIT_HEADER_0 = 0,
  BT_DECODE_WAIT_HEADER_1,
  BT_DECODE_WAIT_LENGTH,
  BT_DECODE_PAYLOAD
} BtDecodeState;

static BtDecodeState g_decode_state;
static uint8_t g_decode_buffer[BT_LINK_RX_BUF_LEN];
static uint8_t g_decode_index;

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
        /* 不是 0xAA 0x55：重新找帧头；是 0xAA 时保持状态以兼容 0xAA 0xAA 0x55 */
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