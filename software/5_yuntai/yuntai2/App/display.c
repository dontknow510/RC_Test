#include "display.h"
#include "OLED.h"
#include <stdio.h>

void Display_Init(void)
{
  OLED_Init();
  OLED_NewFrame();
  OLED_PrintASCIIString(0U, 0U, "YUNTAI2", &afont8x6,
                        OLED_COLOR_NORMAL);
  OLED_PrintASCIIString(0U, 16U, "READY", &afont8x6,
                        OLED_COLOR_NORMAL);
  OLED_ShowFrame();
}

void Display_Update(const ADC_InputData *data,
                    const MPU6050_AppState *mpu_state)
{
  char line[22];

  if ((data == NULL) || (mpu_state == NULL))
  {
    return;
  }

  OLED_NewFrame();

  (void)snprintf(line, sizeof(line), "H:%4u V:%4u",
                 (unsigned int)data->horizontal_raw,
                 (unsigned int)data->pitch_raw);
  OLED_PrintASCIIString(0U, 0U, line, &afont8x6, OLED_COLOR_NORMAL);

  if ((mpu_state->sample_valid != 0U) &&
      (mpu_state->initialized != 0U))
  {
    (void)snprintf(line, sizeof(line), "R:%4d P:%4d",
                   (int)mpu_state->roll_deg,
                   (int)mpu_state->pitch_deg);
    OLED_PrintASCIIString(0U, 16U, line, &afont8x6, OLED_COLOR_NORMAL);

    (void)snprintf(line, sizeof(line), "Y:%6d",
                   (int)mpu_state->yaw_deg);
    OLED_PrintASCIIString(0U, 24U, line, &afont8x6, OLED_COLOR_NORMAL);
  }

  OLED_ShowFrame();
}
