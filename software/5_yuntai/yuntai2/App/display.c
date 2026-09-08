#include "display.h"
#include "OLED.h"
#include "key.h"
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

void Display_Update(const ADC_InputData *data)
{
  char line[22];

  if (data == NULL)
  {
    return;
  }

  OLED_NewFrame();

  (void)snprintf(line, sizeof(line), "H:%4u V:%4u",
                 (unsigned int)data->horizontal_raw,
                 (unsigned int)data->pitch_raw);
  OLED_PrintASCIIString(0U, 0U, line, &afont8x6, OLED_COLOR_NORMAL);

  (void)snprintf(line, sizeof(line), "KEY:%u%u%u%u",
                 (unsigned int)KEY_IsDown(KEY_ID_1),
                 (unsigned int)KEY_IsDown(KEY_ID_2),
                 (unsigned int)KEY_IsDown(KEY_ID_3),
                 (unsigned int)KEY_IsDown(KEY_ID_4));
  OLED_PrintASCIIString(0U, 16U, line, &afont8x6, OLED_COLOR_NORMAL);

  OLED_ShowFrame();
}
