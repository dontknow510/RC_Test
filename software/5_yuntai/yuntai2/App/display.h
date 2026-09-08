#ifndef YUNTAI_DISPLAY_H
#define YUNTAI_DISPLAY_H

#include "adc_input.h"
#include "mpu6050_app.h"

void Display_Init(void);
void Display_Update(const ADC_InputData *data,
                    const MPU6050_AppState *mpu_state);

#endif
