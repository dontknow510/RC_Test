#ifndef YUNTAI_SLAVE_APP_H
#define YUNTAI_SLAVE_APP_H

#include "main.h"

/* ==========================================================================
 * 执行端应用层
 *
 * 全部工作在 USART1 接收中断里完成：
 *   收字节 -> 蓝牙链路层解帧（帧头 + 长度 + CRC8）-> 校验通过才输出脉宽。
 * 收不到帧时舵机自然保持最后位置，主循环不需要做事。
 * ========================================================================== */

void Slave_Init(void);

/* 供 USART1 接收中断调用 */
void Slave_OnByte(uint8_t byte);

#endif /* YUNTAI_SLAVE_APP_H */