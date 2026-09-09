#ifndef YUNTAI_BT_LINK_H
#define YUNTAI_BT_LINK_H

#include "main.h"

/* ==========================================================================
 * 蓝牙链路层（执行端）
 *
 * 帧格式与主控端 yuntai2 严格一致（固定 10 字节）：
 *   [0] 0xAA        帧头
 *   [1] 0x55        帧头
 *   [2] LEN = 6     载荷长度（SEQ..CH2_L）
 *   [3] SEQ         帧序号，溢出回绕
 *   [4] MODE        0 = 电位器模式，1 = 陀螺仪模式
 *   [5] CH1_US_H    水平目标脉宽（微秒，高字节在前）
 *   [6] CH1_US_L
 *   [7] CH2_US_H    俯仰目标脉宽（微秒，高字节在前）
 *   [8] CH2_US_L
 *   [9] CRC8        对 [2..8] 计算（poly 0x07，初值 0）
 *
 * 执行端只做"收帧 -> 校验 -> 输出脉宽"，所有解算都在主控端完成。
 * ========================================================================== */

#define BT_LINK_HEADER_0      0xAAU
#define BT_LINK_HEADER_1      0x55U
#define BT_LINK_PAYLOAD_LEN   6U
#define BT_LINK_FRAME_LEN     10U

typedef enum
{
  BT_LINK_MODE_POTENTIOMETER = 0U,
  BT_LINK_MODE_GYRO = 1U
} BtLinkMode;

typedef struct
{
  uint8_t seq;
  uint8_t mode;
  uint16_t ch1_us;
  uint16_t ch2_us;
} BtLinkFrame;

uint8_t BtLink_Crc8(const uint8_t *data, uint8_t length);
uint8_t BtLink_Encode(const BtLinkFrame *frame, uint8_t *out, uint8_t out_size);

/* 逐字节解码状态机，返回 1 表示解析出一帧且 CRC 正确。 */
void BtLink_DecodeReset(void);
uint8_t BtLink_DecodeByte(uint8_t byte, BtLinkFrame *out);

#endif /* YUNTAI_BT_LINK_H */