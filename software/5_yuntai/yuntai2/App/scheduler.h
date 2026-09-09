#ifndef YUNTAI_SCHEDULER_H
#define YUNTAI_SCHEDULER_H

#include "main.h"

HAL_StatusTypeDef Scheduler_Start(void);
uint32_t Scheduler_GetTick(void);

#endif
