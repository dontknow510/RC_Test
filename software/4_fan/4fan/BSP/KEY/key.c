#include "key.h"

#define KEY_DEBOUNCE_TIME_MS 20U
#define KEY_LONG_PRESS_TIME_MS 800U

typedef enum
{
  KEY_STATE_IDLE = 0,
  KEY_STATE_DEBOUNCE_PRESS,
  KEY_STATE_PRESSED,
  KEY_STATE_DEBOUNCE_RELEASE
} KeyState;

typedef struct
{
  GPIO_TypeDef *port;
  uint16_t pin;
} KeyConfig;

typedef struct
{
  KeyState state;
  uint8_t stablePressed;
  uint8_t longPressReported;
  uint32_t stateTick;
  uint32_t pressTick;
  KeyEvent event;
} KeyRuntime;

static const KeyConfig keyConfig[KEY_ID_COUNT] =
{
  {GPIOE, GPIO_PIN_12},
  {GPIOE, GPIO_PIN_13},
  {GPIOE, GPIO_PIN_14},
  {GPIOE, GPIO_PIN_15}
};

static KeyRuntime keyRuntime[KEY_ID_COUNT];

static uint8_t KEY_ReadRaw(KeyId key)
{
  return HAL_GPIO_ReadPin(keyConfig[key].port, keyConfig[key].pin) == GPIO_PIN_SET;
}

void KEY_Init(void)
{
  uint32_t now = HAL_GetTick();

  for (KeyId key = KEY_ID_1; key < KEY_ID_COUNT; key++)
  {
    keyRuntime[key].state = KEY_STATE_IDLE;
    keyRuntime[key].stablePressed = 0U;
    keyRuntime[key].longPressReported = 0U;
    keyRuntime[key].stateTick = now;
    keyRuntime[key].pressTick = now;
    keyRuntime[key].event = KEY_EVENT_NONE;
  }
}

void KEY_Scan(void)
{
  uint32_t now = HAL_GetTick();

  for (KeyId key = KEY_ID_1; key < KEY_ID_COUNT; key++)
  {
    uint8_t rawPressed = KEY_ReadRaw(key);
    KeyRuntime *runtime = &keyRuntime[key];

    switch (runtime->state)
    {
      case KEY_STATE_IDLE:
        if (rawPressed)
        {
          runtime->state = KEY_STATE_DEBOUNCE_PRESS;
          runtime->stateTick = now;
        }
        break;

      case KEY_STATE_DEBOUNCE_PRESS:
        if (!rawPressed)
        {
          runtime->state = KEY_STATE_IDLE;
        }
        else if ((uint32_t)(now - runtime->stateTick) >= KEY_DEBOUNCE_TIME_MS)
        {
          runtime->state = KEY_STATE_PRESSED;
          runtime->stablePressed = 1U;
          runtime->pressTick = now;
          runtime->longPressReported = 0U;
          runtime->event = KEY_EVENT_PRESSED;
        }
        break;

      case KEY_STATE_PRESSED:
        if (!rawPressed)
        {
          runtime->state = KEY_STATE_DEBOUNCE_RELEASE;
          runtime->stateTick = now;
        }
        else if (!runtime->longPressReported &&
                 (uint32_t)(now - runtime->pressTick) >= KEY_LONG_PRESS_TIME_MS)
        {
          runtime->longPressReported = 1U;
          runtime->event = KEY_EVENT_LONG_PRESSED;
        }
        break;

      case KEY_STATE_DEBOUNCE_RELEASE:
        if (rawPressed)
        {
          runtime->state = KEY_STATE_PRESSED;
        }
        else if ((uint32_t)(now - runtime->stateTick) >= KEY_DEBOUNCE_TIME_MS)
        {
          runtime->state = KEY_STATE_IDLE;
          runtime->stablePressed = 0U;
          runtime->event = KEY_EVENT_RELEASED;
        }
        break;

      default:
        runtime->state = KEY_STATE_IDLE;
        runtime->stablePressed = 0U;
        runtime->event = KEY_EVENT_NONE;
        break;
    }
  }
}

KeyEvent KEY_GetEvent(KeyId key)
{
  KeyEvent event;

  if (key >= KEY_ID_COUNT)
  {
    return KEY_EVENT_NONE;
  }

  event = keyRuntime[key].event;
  keyRuntime[key].event = KEY_EVENT_NONE;
  return event;
}

uint8_t KEY_IsPressed(KeyId key)
{
  if (key >= KEY_ID_COUNT)
  {
    return 0U;
  }

  return keyRuntime[key].stablePressed;
}
