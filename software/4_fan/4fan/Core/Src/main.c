/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file           : main.c
  * @brief          : Main program body
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2026 STMicroelectronics.
  * All rights reserved.
  *
  * This software is licensed under terms that can be found in the LICENSE file
  * in the root directory of this software component.
  * If no LICENSE file comes with this software, it is provided AS-IS.
  *
  ******************************************************************************
  */
/* USER CODE END Header */
/* Includes ------------------------------------------------------------------*/
#include "main.h"
#include "adc.h"
#include "i2c.h"
#include "tim.h"
#include "usart.h"
#include "gpio.h"
#include "OLED.h"
#include "key.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */

typedef enum
{
  UI_PAGE_MENU = 0,
  UI_PAGE_SPEED_TEST,
  UI_PAGE_POSITION_TEST
} UiPage;

static UiPage uiPage = UI_PAGE_MENU;
static uint8_t uiMenuIndex = 0U;
static uint8_t uiNeedsRender = 1U;

static char uiSpeedMode[] = "\xE5\xAE\x9A\xE9\x80\x9F\xE6\xA8\xA1\xE5\xBC\x8F";
static char uiPositionMode[] = "\xE5\xAE\x9A\xE4\xBD\x8D\xE6\xA8\xA1\xE5\xBC\x8F";
static char uiCursor[] = ">";
static char uiTestText[] = "TEST";

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */

static void UI_Render(void);
static void UI_HandleKeyEvents(void);

/* USER CODE END PFP */

/* Private user code ---------------------------------------------------------*/
/* USER CODE BEGIN 0 */
/* USER CODE END 0 */

/**
  * @brief  The application entry point.
  * @retval int
  */
int main(void)
{

  /* USER CODE BEGIN 1 */

  /* USER CODE END 1 */

  /* MCU Configuration--------------------------------------------------------*/

  /* Reset of all peripherals, Initializes the Flash interface and the Systick. */
  HAL_Init();

  /* USER CODE BEGIN Init */

  /* USER CODE END Init */

  /* Configure the system clock */
  SystemClock_Config();

  /* USER CODE BEGIN SysInit */

  /* USER CODE END SysInit */

  /* Initialize all configured peripherals */
  MX_GPIO_Init();
  MX_I2C2_Init();
  MX_TIM4_Init();
  MX_TIM9_Init();
  MX_USART1_UART_Init();
  MX_ADC1_Init();
  MX_TIM6_Init();
  /* USER CODE BEGIN 2 */
  OLED_Init();
  KEY_Init();
  UI_Render();
  uiNeedsRender = 0U;
  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */
    /* USER CODE BEGIN 3 */
    KEY_Scan();
    UI_HandleKeyEvents();

    if (uiNeedsRender)
    {
      UI_Render();
      uiNeedsRender = 0U;
    }

    HAL_Delay(5U);
  }
  /* USER CODE END 3 */
}

/**
  * @brief System Clock Configuration
  * @retval None
  */
void SystemClock_Config(void)
{
  RCC_OscInitTypeDef RCC_OscInitStruct = {0};
  RCC_ClkInitTypeDef RCC_ClkInitStruct = {0};

  /** Configure the main internal regulator output voltage
  */
  __HAL_RCC_PWR_CLK_ENABLE();
  __HAL_PWR_VOLTAGESCALING_CONFIG(PWR_REGULATOR_VOLTAGE_SCALE1);

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSI;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.HSICalibrationValue = RCC_HSICALIBRATION_DEFAULT;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSI;
  RCC_OscInitStruct.PLL.PLLM = 8;
  RCC_OscInitStruct.PLL.PLLN = 168;
  RCC_OscInitStruct.PLL.PLLP = RCC_PLLP_DIV2;
  RCC_OscInitStruct.PLL.PLLQ = 4;
  if (HAL_RCC_OscConfig(&RCC_OscInitStruct) != HAL_OK)
  {
    Error_Handler();
  }

  /** Initializes the CPU, AHB and APB buses clocks
  */
  RCC_ClkInitStruct.ClockType = RCC_CLOCKTYPE_HCLK|RCC_CLOCKTYPE_SYSCLK
                              |RCC_CLOCKTYPE_PCLK1|RCC_CLOCKTYPE_PCLK2;
  RCC_ClkInitStruct.SYSCLKSource = RCC_SYSCLKSOURCE_PLLCLK;
  RCC_ClkInitStruct.AHBCLKDivider = RCC_SYSCLK_DIV1;
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV4;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV2;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_5) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */

static void UI_Render(void)
{
  OLED_NewFrame();

  if (uiPage == UI_PAGE_MENU)
  {
    OLED_PrintString(24U, 0U, uiSpeedMode, &font16x16,
                     OLED_COLOR_NORMAL);
    OLED_PrintString(24U, 24U, uiPositionMode, &font16x16,
                     OLED_COLOR_NORMAL);

    if (uiMenuIndex == 0U)
    {
      OLED_PrintASCIIString(0U, 0U, uiCursor, &afont16x8,
                            OLED_COLOR_NORMAL);
    }
    else
    {
      OLED_PrintASCIIString(0U, 24U, uiCursor, &afont16x8,
                            OLED_COLOR_NORMAL);
    }
  }
  else if (uiPage == UI_PAGE_SPEED_TEST)
  {
    OLED_PrintString(32U, 0U, uiSpeedMode, &font16x16,
                     OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(48U, 32U, uiTestText, &afont16x8,
                          OLED_COLOR_NORMAL);
  }
  else
  {
    OLED_PrintString(32U, 0U, uiPositionMode, &font16x16,
                     OLED_COLOR_NORMAL);
    OLED_PrintASCIIString(48U, 32U, uiTestText, &afont16x8,
                          OLED_COLOR_NORMAL);
  }

  OLED_ShowFrame();
}

static void UI_HandleKeyEvents(void)
{
  KeyEvent key1Event = KEY_GetEvent(KEY_ID_1);
  KeyEvent key2Event = KEY_GetEvent(KEY_ID_2);
  KeyEvent key3Event = KEY_GetEvent(KEY_ID_3);
  KeyEvent key4Event = KEY_GetEvent(KEY_ID_4);

  if (uiPage == UI_PAGE_MENU)
  {
    if (key1Event == KEY_EVENT_PRESSED)
    {
      uiMenuIndex = (uiMenuIndex == 0U) ? 1U : 0U;
      uiNeedsRender = 1U;
    }
    else if (key2Event == KEY_EVENT_PRESSED)
    {
      uiMenuIndex = (uiMenuIndex == 0U) ? 1U : 0U;
      uiNeedsRender = 1U;
    }
    else if (key3Event == KEY_EVENT_PRESSED)
    {
      uiPage = (uiMenuIndex == 0U) ? UI_PAGE_SPEED_TEST
                                   : UI_PAGE_POSITION_TEST;
      uiNeedsRender = 1U;
    }
  }
  else if (key4Event == KEY_EVENT_PRESSED)
  {
    uiPage = UI_PAGE_MENU;
    uiNeedsRender = 1U;
  }
}

/* USER CODE END 4 */

/**
  * @brief  This function is executed in case of error occurrence.
  * @retval None
  */
void Error_Handler(void)
{
  /* USER CODE BEGIN Error_Handler_Debug */
  /* User can add his own implementation to report the HAL error return state */
  __disable_irq();
  while (1)
  {
  }
  /* USER CODE END Error_Handler_Debug */
}
#ifdef USE_FULL_ASSERT
/**
  * @brief  Reports the name of the source file and the source line number
  *         where the assert_param error has occurred.
  * @param  file: pointer to the source file name
  * @param  line: assert_param error line source number
  * @retval None
  */
void assert_failed(uint8_t *file, uint32_t line)
{
  /* USER CODE BEGIN 6 */
  /* User can add his own implementation to report the file name and line number,
     ex: printf("Wrong parameters value: file %s on line %d\r\n", file, line) */
  /* USER CODE END 6 */
}
#endif /* USE_FULL_ASSERT */
