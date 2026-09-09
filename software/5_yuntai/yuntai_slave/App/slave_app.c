#include "slave_app.h"
#include "bt_link.h"
#include "servo_app.h"
#include "usart.h"

/* 超过该时间没有收到有效帧就认为失联（保持最后位置） */
#define SLAVE_LINK_TIMEOUT_MS   500U
/* 失联时 LED 闪烁半周期 */
#define SLAVE_LED_BLINK_MS      250U

static BtLinkFrame g_last_frame;
static uint32_t g_last_frame_tick;
static uint8_t g_link_alive;
static uint8_t g_rx_byte;

static void Slave_ArmReceive(void)
{
  (void)HAL_UART_Receive_IT(&huart1, &g_rx_byte, 1U);
}

void Slave_Init(void)
{
  (void)Servo_Init();

  BtLink_DecodeReset();
  g_last_frame = (BtLinkFrame){0};
  g_last_frame_tick = HAL_GetTick();
  g_link_alive = 0U;

  Slave_ArmReceive();
}

void Slave_OnByte(uint8_t byte)
{
  BtLinkFrame frame;

  if (BtLink_DecodeByte(byte, &frame) != 0U)
  {
    g_last_frame = frame;
    g_last_frame_tick = HAL_GetTick();
    g_link_alive = 1U;

    /* 只有 CRC 正确的帧才会走到这里；脉宽限幅在 servo_app 内部完成 */
    Servo_SetPulseUs(SERVO_CHANNEL_HORIZONTAL, frame.ch1_us);
    Servo_SetPulseUs(SERVO_CHANNEL_PITCH, frame.ch2_us);
  }
}

void Slave_Update(void)
{
  uint32_t now = HAL_GetTick();

  if ((uint32_t)(now - g_last_frame_tick) > SLAVE_LINK_TIMEOUT_MS)
  {
    /* 失联：保持最后位置，不对舵机做任何操作 */
    g_link_alive = 0U;
  }

  if (g_link_alive != 0U)
  {
    /* PC13 低电平点亮：常亮表示链路正常 */
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
  else if (((now / SLAVE_LED_BLINK_MS) & 1U) != 0U)
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_SET);
  }
  else
  {
    HAL_GPIO_WritePin(GPIOC, GPIO_PIN_13, GPIO_PIN_RESET);
  }
}

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    Slave_OnByte(g_rx_byte);
    Slave_ArmReceive();
  }
}

void HAL_UART_ErrorCallback(UART_HandleTypeDef *huart)
{
  if ((huart != NULL) && (huart->Instance == USART1))
  {
    /* 噪声/帧错误/溢出后重新武装接收，避免链路永久卡死 */
    Slave_ArmReceive();
  }
}

uint8_t Slave_IsLinkAlive(void)
{
  return g_link_alive;
}

uint8_t Slave_GetLastMode(void)
{
  return g_last_frame.mode;
}

uint32_t Slave_GetLastSeq(void)
{
  return (uint32_t)g_last_frame.seq;
}