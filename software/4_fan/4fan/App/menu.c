#include "menu.h"
#include "motor.h"
#include "OLED.h"
#include "key.h"
#include <stdio.h>

static MenuPage menuPage = MENU_PAGE_MAIN;
static uint8_t menuIndex;
static volatile uint8_t menuRenderPending = 1U;

static char menuSpeedText[] = "\xE5\xAE\x9A\xE9\x80\x9F\xE6\xA8\xA1\xE5\xBC\x8F";
static char menuPositionText[] = "\xE5\xAE\x9A\xE4\xBD\x8D\xE6\xA8\xA1\xE5\xBC\x8F";

void Menu_Init(void)
{
  menuPage = MENU_PAGE_MAIN;
  menuIndex = 0U;
  menuRenderPending = 1U;
}

MenuPage Menu_GetPage(void)
{
  return menuPage;
}

void Menu_RequestRender(void)
{
  menuRenderPending = 1U;
}

uint8_t Menu_IsRenderPending(void)
{
  return menuRenderPending;
}

void Menu_ClearRenderPending(void)
{
  menuRenderPending = 0U;
}

static void Menu_EnterPage(MenuPage page)
{
  menuPage = page;
  if (page == MENU_PAGE_SPEED)
  {
    Motor_SetMode(MOTOR_MODE_SPEED);
  }
  else if (page == MENU_PAGE_POSITION)
  {
    Motor_SetMode(MOTOR_MODE_POSITION);
  }
  Menu_RequestRender();
}

static void Menu_LeavePage(void)
{
  Motor_Stop();
  menuPage = MENU_PAGE_MAIN;
  Menu_RequestRender();
}

void Menu_HandleKeyEvents(void)
{
  KeyEvent key1 = KEY_GetEvent(KEY_ID_1);
  KeyEvent key2 = KEY_GetEvent(KEY_ID_2);
  KeyEvent key3 = KEY_GetEvent(KEY_ID_3);
  KeyEvent key4 = KEY_GetEvent(KEY_ID_4);

  if (menuPage == MENU_PAGE_MAIN)
  {
    if (key1 == KEY_EVENT_PRESSED)
    {
      menuIndex = (menuIndex == 0U) ? 1U : (uint8_t)(menuIndex - 1U);
      Menu_RequestRender();
    }
    else if (key2 == KEY_EVENT_PRESSED)
    {
      menuIndex = (menuIndex >= 1U) ? 0U : (uint8_t)(menuIndex + 1U);
      Menu_RequestRender();
    }
    else if (key3 == KEY_EVENT_PRESSED)
    {
      Menu_EnterPage((MenuPage)(menuIndex + 1U));
    }
  }
  else if (menuPage == MENU_PAGE_SPEED && key3 == KEY_EVENT_PRESSED)
  {
    Motor_SpeedToggle();
    Menu_RequestRender();
  }
  else if (menuPage == MENU_PAGE_POSITION && key3 == KEY_EVENT_PRESSED)
  {
    Motor_PositionToggle();
    Menu_RequestRender();
  }

  if (key4 == KEY_EVENT_PRESSED && menuPage != MENU_PAGE_MAIN)
  {
    Menu_LeavePage();
  }
}

void Menu_MainLoopUpdate(void)
{
  if (Motor_MainLoopUpdate() != 0U)
  {
    Menu_RequestRender();
  }
}

void Menu_Render(void)
{
  char line[24];
  MotorSpeedData speed;
  MotorPositionData position;

  OLED_NewFrame();

  if (menuPage == MENU_PAGE_MAIN)
  {
    OLED_PrintString(24U, 0U, menuSpeedText, &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintString(24U, 24U, menuPositionText, &font16x16, OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(0U, (uint8_t)(menuIndex * 24U), ">", &afont16x8,
                          OLED_COLOR_NORMAL);
  }
  else if (menuPage == MENU_PAGE_SPEED)
  {
    Motor_GetSpeedData(&speed);
    (void)snprintf(line, sizeof(line), "TGT:%3u",
                   (unsigned int)speed.targetRpm);
    OLED_PrintASCIIString(0U, 0U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "RPM:%4ld", (long)speed.rpm);
    OLED_PrintASCIIString(0U, 16U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "Kp:%u.%02u",
                   (unsigned int)(speed.kp100 / 100U),
                   (unsigned int)(speed.kp100 % 100U));
    OLED_PrintASCIIString(0U, 32U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "Ki:%u.%02u",
                   (unsigned int)(speed.ki100 / 100U),
                   (unsigned int)(speed.ki100 % 100U));
    OLED_PrintASCIIString(0U, 48U, line, &afont12x6, OLED_COLOR_NORMAL);
  }
  else if (menuPage == MENU_PAGE_POSITION)
  {
    Motor_GetPositionData(&position);
    (void)snprintf(line, sizeof(line), "TGT:%3u %s",
                   (unsigned int)position.targetAngle,
                   position.pwmEnabled != 0U ? "RUN" : "STOP");
    OLED_PrintASCIIString(0U, 0U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "PosKp:%u.%02u",
                   (unsigned int)(position.positionKp100 / 100U),
                   (unsigned int)(position.positionKp100 % 100U));
    OLED_PrintASCIIString(0U, 16U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "ANG:%+5ld",
                   (long)position.positionAngle);
    OLED_PrintASCIIString(0U, 32U, line, &afont12x6, OLED_COLOR_NORMAL);
    (void)snprintf(line, sizeof(line), "ERR:%+6ld",
                   (long)position.errorCount);
    OLED_PrintASCIIString(0U, 48U, line, &afont12x6, OLED_COLOR_NORMAL);
  }

  OLED_ShowFrame();
}
