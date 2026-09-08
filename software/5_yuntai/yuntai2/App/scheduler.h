#ifndef YUNTAI_SCHEDULER_H
#define YUNTAI_SCHEDULER_H

#include "main.h"

extern volatile uint32_t g_10ms_tick;

HAL_StatusTypeDef Scheduler_Start(void);
uint32_t Scheduler_GetTick(void);

#endif
