#ifndef YUNTAI_ADC_INPUT_H
#define YUNTAI_ADC_INPUT_H

#include "main.h"

typedef struct
{
  uint16_t horizontal_raw;
  uint16_t pitch_raw;
} ADC_InputData;

HAL_StatusTypeDef ADC_Input_Update(void);
void ADC_Input_Get(ADC_InputData *data);

#endif
