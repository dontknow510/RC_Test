#ifndef APP_MENU_H
#define APP_MENU_H

#include <stdint.h>

typedef enum
{
  MENU_PAGE_MAIN = 0,
  MENU_PAGE_SPEED,
  MENU_PAGE_POSITION
} MenuPage;

void Menu_Init(void);
void Menu_HandleKeyEvents(void);
void Menu_Render(void);
void Menu_MainLoopUpdate(void);
void Menu_RequestRender(void);
uint8_t Menu_IsRenderPending(void);
void Menu_ClearRenderPending(void);
MenuPage Menu_GetPage(void);

#endif
