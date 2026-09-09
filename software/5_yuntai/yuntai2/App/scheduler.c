#include "scheduler.h"
#include "tim.h"

static volatile uint32_t g_10ms_tick;

HAL_StatusTypeDef Scheduler_Start(void)
{
  g_10ms_tick = 0U;
  return HAL_TIM_Base_Start_IT(&htim6);
}

uint32_t Scheduler_GetTick(void)
{
  return g_10ms_tick;
}

void HAL_TIM_PeriodElapsedCallback(TIM_HandleTypeDef *htim)
{
  if ((htim != NULL) && (htim->Instance == TIM6))
  {
    g_10ms_tick++;
  }
}
