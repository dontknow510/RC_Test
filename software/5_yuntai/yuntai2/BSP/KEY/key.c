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
  uint8_t stable_pressed;
  uint8_t long_press_reported;
  uint32_t state_tick;
  uint32_t press_tick;
  KeyEvent event;
} KeyRuntime;

static const KeyConfig key_config[KEY_ID_COUNT] =
{
  {GPIOE, GPIO_PIN_12},
  {GPIOE, GPIO_PIN_13},
  {GPIOE, GPIO_PIN_14},
  {GPIOE, GPIO_PIN_15}
};

static KeyRuntime key_runtime[KEY_ID_COUNT];

static uint8_t KEY_ReadRaw(KeyId key)
{
  /* This board matches 4_fan: pull-down inputs, high level means pressed. */
  return (HAL_GPIO_ReadPin(key_config[key].port, key_config[key].pin) ==
          GPIO_PIN_SET) ? 1U : 0U;
}

void KEY_Init(void)
{
  uint32_t now = HAL_GetTick();

  for (KeyId key = KEY_ID_1; key < KEY_ID_COUNT; key++)
  {
    key_runtime[key].state = KEY_STATE_IDLE;
    key_runtime[key].stable_pressed = 0U;
    key_runtime[key].long_press_reported = 0U;
    key_runtime[key].state_tick = now;
    key_runtime[key].press_tick = now;
    key_runtime[key].event = KEY_EVENT_NONE;
  }
}

void KEY_Scan(void)
{
  uint32_t now = HAL_GetTick();

  for (KeyId key = KEY_ID_1; key < KEY_ID_COUNT; key++)
  {
    uint8_t raw_pressed = KEY_ReadRaw(key);
    KeyRuntime *runtime = &key_runtime[key];

    switch (runtime->state)
    {
      case KEY_STATE_IDLE:
        if (raw_pressed != 0U)
        {
          runtime->state = KEY_STATE_DEBOUNCE_PRESS;
          runtime->state_tick = now;
        }
        break;

      case KEY_STATE_DEBOUNCE_PRESS:
        if (raw_pressed == 0U)
        {
          runtime->state = KEY_STATE_IDLE;
        }
        else if ((uint32_t)(now - runtime->state_tick) >=
                 KEY_DEBOUNCE_TIME_MS)
        {
          runtime->state = KEY_STATE_PRESSED;
          runtime->stable_pressed = 1U;
          runtime->press_tick = now;
          runtime->long_press_reported = 0U;
          runtime->event = KEY_EVENT_PRESSED;
        }
        break;

      case KEY_STATE_PRESSED:
        if (raw_pressed == 0U)
        {
          runtime->state = KEY_STATE_DEBOUNCE_RELEASE;
          runtime->state_tick = now;
        }
        else if ((runtime->long_press_reported == 0U) &&
                 ((uint32_t)(now - runtime->press_tick) >=
                  KEY_LONG_PRESS_TIME_MS))
        {
          runtime->long_press_reported = 1U;
          runtime->event = KEY_EVENT_LONG_PRESSED;
        }
        break;

      case KEY_STATE_DEBOUNCE_RELEASE:
        if (raw_pressed != 0U)
        {
          runtime->state = KEY_STATE_PRESSED;
        }
        else if ((uint32_t)(now - runtime->state_tick) >=
                 KEY_DEBOUNCE_TIME_MS)
        {
          runtime->state = KEY_STATE_IDLE;
          runtime->stable_pressed = 0U;
          runtime->event = KEY_EVENT_RELEASED;
        }
        break;

      default:
        runtime->state = KEY_STATE_IDLE;
        runtime->stable_pressed = 0U;
        runtime->long_press_reported = 0U;
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

  event = key_runtime[key].event;
  key_runtime[key].event = KEY_EVENT_NONE;
  return event;
}

uint8_t KEY_IsDown(KeyId key)
{
  if (key >= KEY_ID_COUNT)
  {
    return 0U;
  }

  return key_runtime[key].stable_pressed;
}

uint8_t KEY_GetPressed(KeyId key)
{
  return (KEY_GetEvent(key) == KEY_EVENT_PRESSED) ? 1U : 0U;
}
