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
#include "tim.h"
#include "usart.h"
#include "gpio.h"

/* Private includes ----------------------------------------------------------*/
/* USER CODE BEGIN Includes */

/* USER CODE END Includes */

/* Private typedef -----------------------------------------------------------*/
/* USER CODE BEGIN PTD */

/* USER CODE END PTD */

/* Private define ------------------------------------------------------------*/
/* USER CODE BEGIN PD */
#define SERVO_MIN_ANGLE_DEG      0.0f
#define SERVO_MAX_ANGLE_DEG      180.0f
#define SERVO_MIN_PULSE_US       500.0f
#define SERVO_MAX_PULSE_US       2500.0f
#define SERVO_PWM_PERIOD_US      20000.0f
#define VOFA_CHANNEL_COUNT       2U
#define UART_COMMAND_BUFFER_SIZE 16U

/* USER CODE END PD */

/* Private macro -------------------------------------------------------------*/
/* USER CODE BEGIN PM */

/* USER CODE END PM */

/* Private variables ---------------------------------------------------------*/

/* USER CODE BEGIN PV */
static float current_angle = 90.0f;
static uint8_t uart_rx_byte = 0U;
static char uart_command_buffer[UART_COMMAND_BUFFER_SIZE] = {0};
static uint8_t uart_command_index = 0U;

/* USER CODE END PV */

/* Private function prototypes -----------------------------------------------*/
void SystemClock_Config(void);
/* USER CODE BEGIN PFP */
static float Servo_LimitAngle(float angle);
static uint16_t Servo_AngleToPulse(float angle);
static void Servo_SetAngle(float angle);
static float Servo_GetDuty(float angle);
static void VOFA_SendFloat(float angle, float duty);
static void UART_CommandProcessByte(uint8_t byte);
static void UART_CommandProcessLine(const char *line);
static uint8_t UART_CommandParseAngle(const char *line, float *angle);

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
  MX_TIM1_Init();
  MX_USART1_UART_Init();
  /* USER CODE BEGIN 2 */
  HAL_TIM_PWM_Start(&htim1, TIM_CHANNEL_4);
  Servo_SetAngle(current_angle);
  HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);

  /* USER CODE END 2 */

  /* Infinite loop */
  /* USER CODE BEGIN WHILE */
  while (1)
  {
    /* USER CODE END WHILE */

    /* USER CODE BEGIN 3 */
    VOFA_SendFloat(current_angle, Servo_GetDuty(current_angle));
    HAL_Delay(20);
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

  /** Initializes the RCC Oscillators according to the specified parameters
  * in the RCC_OscInitTypeDef structure.
  */
  RCC_OscInitStruct.OscillatorType = RCC_OSCILLATORTYPE_HSE;
  RCC_OscInitStruct.HSEState = RCC_HSE_ON;
  RCC_OscInitStruct.HSEPredivValue = RCC_HSE_PREDIV_DIV1;
  RCC_OscInitStruct.HSIState = RCC_HSI_ON;
  RCC_OscInitStruct.PLL.PLLState = RCC_PLL_ON;
  RCC_OscInitStruct.PLL.PLLSource = RCC_PLLSOURCE_HSE;
  RCC_OscInitStruct.PLL.PLLMUL = RCC_PLL_MUL9;
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
  RCC_ClkInitStruct.APB1CLKDivider = RCC_HCLK_DIV2;
  RCC_ClkInitStruct.APB2CLKDivider = RCC_HCLK_DIV1;

  if (HAL_RCC_ClockConfig(&RCC_ClkInitStruct, FLASH_LATENCY_2) != HAL_OK)
  {
    Error_Handler();
  }
}

/* USER CODE BEGIN 4 */
static float Servo_LimitAngle(float angle)
{
  if (angle < SERVO_MIN_ANGLE_DEG)
  {
    return SERVO_MIN_ANGLE_DEG;
  }
  if (angle > SERVO_MAX_ANGLE_DEG)
  {
    return SERVO_MAX_ANGLE_DEG;
  }
  return angle;
}

static uint16_t Servo_AngleToPulse(float angle)
{
  const float limited_angle = Servo_LimitAngle(angle);
  const float pulse_range = SERVO_MAX_PULSE_US - SERVO_MIN_PULSE_US;
  const float angle_range = SERVO_MAX_ANGLE_DEG - SERVO_MIN_ANGLE_DEG;
  const float pulse = SERVO_MIN_PULSE_US + limited_angle * pulse_range / angle_range;

  return (uint16_t)(pulse + 0.5f);
}

static void Servo_SetAngle(float angle)
{
  current_angle = Servo_LimitAngle(angle);
  __HAL_TIM_SET_COMPARE(&htim1, TIM_CHANNEL_4, Servo_AngleToPulse(current_angle));
}

static float Servo_GetDuty(float angle)
{
  return (float)Servo_AngleToPulse(angle) * 100.0f / SERVO_PWM_PERIOD_US;
}

static void VOFA_SendFloat(float angle, float duty)
{
  float data[VOFA_CHANNEL_COUNT] = {angle, duty};
  uint8_t tail[4] = {0x00, 0x00, 0x80, 0x7F};

  HAL_UART_Transmit(&huart1, (uint8_t *)data, sizeof(data), 100);
  HAL_UART_Transmit(&huart1, tail, sizeof(tail), 100);
}

static void UART_CommandProcessByte(uint8_t byte)
{
  if (byte == '\r')
  {
    return;
  }

  if (byte == '\n')
  {
    uart_command_buffer[uart_command_index] = '\0';
    UART_CommandProcessLine(uart_command_buffer);
    uart_command_index = 0U;
    return;
  }

  if (uart_command_index < (UART_COMMAND_BUFFER_SIZE - 1U))
  {
    uart_command_buffer[uart_command_index] = (char)byte;
    uart_command_index++;
  }
  else
  {
    uart_command_index = 0U;
  }
}

static void UART_CommandProcessLine(const char *line)
{
  float angle = 0.0f;

  if (UART_CommandParseAngle(line, &angle) != 0U)
  {
    Servo_SetAngle(angle);
  }
}

static uint8_t UART_CommandParseAngle(const char *line, float *angle)
{
  float value = 0.0f;
  float decimal = 0.1f;
  uint8_t has_digit = 0U;

  while ((*line != '\0') && ((*line < '0') || (*line > '9')) && (*line != '.'))
  {
    line++;
  }

  while ((*line >= '0') && (*line <= '9'))
  {
    value = value * 10.0f + (float)(*line - '0');
    has_digit = 1U;
    line++;
  }

  if (*line == '.')
  {
    line++;
    while ((*line >= '0') && (*line <= '9'))
    {
      value += (float)(*line - '0') * decimal;
      decimal *= 0.1f;
      has_digit = 1U;
      line++;
    }
  }

  if (has_digit == 0U)
  {
    return 0U;
  }

  *angle = value;
  return 1U;
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if (huart->Instance == USART1)
  {
    UART_CommandProcessByte(uart_rx_byte);
    HAL_UART_Receive_IT(&huart1, &uart_rx_byte, 1);
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
