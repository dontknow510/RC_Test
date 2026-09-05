#ifndef APP_VOFA_H
#define APP_VOFA_H

#include "stm32f4xx_hal.h"

void VOFA_Init(UART_HandleTypeDef *uart);
void VOFA_MainLoopUpdate(void);

#endif
