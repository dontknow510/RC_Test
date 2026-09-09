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
  KEY_EVENT_RELEASED
} KeyEvent;

void KEY_Init(void);
void KEY_Scan(void);
KeyEvent KEY_GetEvent(KeyId key);
uint8_t KEY_IsDown(KeyId key);
uint8_t KEY_GetPressed(KeyId key);

#endif /* BSP_KEY_H */
