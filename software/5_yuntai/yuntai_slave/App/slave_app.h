#ifndef YUNTAI_SLAVE_APP_H
#define YUNTAI_SLAVE_APP_H

#include "main.h"

/* ==========================================================================
 * 执行端应用层
 *
 * 职责：
 *   1) 中断逐字节接收 USART1 数据
 *   2) 用蓝牙链路层解码（帧头 + 长度 + CRC8 校验）
 *   3) 校验通过才把目标脉宽输出到 TIM3 的两路舵机
 *   4) 失联保护：超过 SLAVE_LINK_TIMEOUT_MS 没有有效帧则保持最后位置
 *   5) PC13 指示链路状态：常亮 = 正常，闪烁 = 失联
 * ========================================================================== */

void Slave_Init(void);

/* 主循环每约 10 ms 调用一次：失联检测 + LED 指示 */
void Slave_Update(void);

/* 供 USART1 接收中断调用 */
void Slave_OnByte(uint8_t byte);

uint8_t Slave_IsLinkAlive(void);
uint8_t Slave_GetLastMode(void);
uint32_t Slave_GetLastSeq(void);

#endif /* YUNTAI_SLAVE_APP_H */