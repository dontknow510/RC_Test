#ifndef BSP_KEY_H
#define BSP_KEY_H

#include "main.h"

typedef enum
{
  KEY_ID_1 = 0,
  KEY_ID_2,
  KEY_ID_3,
  KEY_ID_4,
  KEY_ID_COUNT
} KeyId;

typedef enum
{
  KEY_EVENT_NONE = 0,
  KEY_EVENT_PRESSED,
  KEY_EVENT_RELEASED,
  KEY_EVENT_LONG_PRESSED
} KeyEvent;

void KEY_Init(void);
void KEY_Scan(void);
KeyEvent KEY_GetEvent(KeyId key);
uint8_t KEY_IsPressed(KeyId key);

#endif /* BSP_KEY_H */
