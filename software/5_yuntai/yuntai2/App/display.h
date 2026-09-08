#ifndef YUNTAI_DISPLAY_H
#define YUNTAI_DISPLAY_H

#include "adc_input.h"

void Display_Init(void);
void Display_Update(const ADC_InputData *data);

#endif
