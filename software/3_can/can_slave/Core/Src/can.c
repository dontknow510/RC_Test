/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    can.c
  * @brief   This file provides code for the configuration
  *          of the CAN instances.
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
#include "can.h"

/* USER CODE BEGIN 0 */
#define CAN_RX_QUEUE_SIZE  8U

static CAN_Message_t can_rx_queue[CAN_RX_QUEUE_SIZE];
static volatile uint8_t can_rx_head = 0U;
static volatile uint8_t can_rx_tail = 0U;

/* USER CODE END 0 */

CAN_HandleTypeDef hcan;

/* CAN init function */
void MX_CAN_Init(void)
{

  /* USER CODE BEGIN CAN_Init 0 */

  /* USER CODE END CAN_Init 0 */

  /* USER CODE BEGIN CAN_Init 1 */

  /* USER CODE END CAN_Init 1 */
  hcan.Instance = CAN1;
  hcan.Init.Prescaler = 9;
  hcan.Init.Mode = CAN_MODE_NORMAL;
  hcan.Init.SyncJumpWidth = CAN_SJW_1TQ;
  hcan.Init.TimeSeg1 = CAN_BS1_6TQ;
  hcan.Init.TimeSeg2 = CAN_BS2_1TQ;
  hcan.Init.TimeTriggeredMode = DISABLE;
  hcan.Init.AutoBusOff = DISABLE;
  hcan.Init.AutoWakeUp = DISABLE;
  hcan.Init.AutoRetransmission = ENABLE;
  hcan.Init.ReceiveFifoLocked = DISABLE;
  hcan.Init.TransmitFifoPriority = DISABLE;
  if (HAL_CAN_Init(&hcan) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN CAN_Init 2 */

  /* USER CODE END CAN_Init 2 */

}

void HAL_CAN_MspInit(CAN_HandleTypeDef* canHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspInit 0 */

  /* USER CODE END CAN1_MspInit 0 */
    /* CAN1 clock enable */
    __HAL_RCC_CAN1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* CAN1 interrupt Init */
    HAL_NVIC_SetPriority(USB_LP_CAN1_RX0_IRQn, 0, 0);
    HAL_NVIC_EnableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspInit 1 */

  /* USER CODE END CAN1_MspInit 1 */
  }
}

void HAL_CAN_MspDeInit(CAN_HandleTypeDef* canHandle)
{

  if(canHandle->Instance==CAN1)
  {
  /* USER CODE BEGIN CAN1_MspDeInit 0 */

  /* USER CODE END CAN1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_CAN1_CLK_DISABLE();

    /**CAN GPIO Configuration
    PA11     ------> CAN_RX
    PA12     ------> CAN_TX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_11|GPIO_PIN_12);

    /* CAN1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USB_LP_CAN1_RX0_IRQn);
  /* USER CODE BEGIN CAN1_MspDeInit 1 */

  /* USER CODE END CAN1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
HAL_StatusTypeDef CAN_AppInit(void)
{
  CAN_FilterTypeDef filter = {0};

  /* Accept all standard data frames and route them to RX FIFO0. */
  filter.FilterBank = 0;
  filter.FilterMode = CAN_FILTERMODE_IDMASK;
  filter.FilterScale = CAN_FILTERSCALE_32BIT;
  filter.FilterIdHigh = 0x0000;
  filter.FilterIdLow = CAN_ID_STD | CAN_RTR_DATA;
  filter.FilterMaskIdHigh = 0x0000;
  filter.FilterMaskIdLow = CAN_ID_EXT | CAN_RTR_REMOTE;
  filter.FilterFIFOAssignment = CAN_RX_FIFO0;
  filter.FilterActivation = ENABLE;
  filter.SlaveStartFilterBank = 14;

  if (HAL_CAN_ConfigFilter(&hcan, &filter) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_Start(&hcan) != HAL_OK)
  {
    return HAL_ERROR;
  }

  if (HAL_CAN_ActivateNotification(&hcan, CAN_IT_RX_FIFO0_MSG_PENDING) != HAL_OK)
  {
    (void)HAL_CAN_Stop(&hcan);
    return HAL_ERROR;
  }

  return HAL_OK;
}

HAL_StatusTypeDef CAN_SendMessage(uint32_t std_id, const uint8_t *data,
                                  uint8_t length)
{
  CAN_TxHeaderTypeDef tx_header = {0};
  uint32_t tx_mailbox = 0U;
  uint8_t tx_data[CAN_MAX_DATA_LENGTH] = {0};
  uint8_t index;

  if ((std_id > 0x7FFU) || (length > CAN_MAX_DATA_LENGTH) ||
      ((data == NULL) && (length > 0U)))
  {
    return HAL_ERROR;
  }

  for (index = 0U; index < length; index++)
  {
    tx_data[index] = data[index];
  }

  tx_header.StdId = std_id;
  tx_header.ExtId = 0U;
  tx_header.IDE = CAN_ID_STD;
  tx_header.RTR = CAN_RTR_DATA;
  tx_header.DLC = length;
  tx_header.TransmitGlobalTime = DISABLE;

  return HAL_CAN_AddTxMessage(&hcan, &tx_header, tx_data, &tx_mailbox);
}

uint8_t CAN_TryReadMessage(CAN_Message_t *message)
{
  uint8_t index;
  uint8_t next_tail;

  if ((message == NULL) || (can_rx_tail == can_rx_head))
  {
    return 0U;
  }

  message->std_id = can_rx_queue[can_rx_tail].std_id;
  message->length = can_rx_queue[can_rx_tail].length;
  for (index = 0U; index < message->length; index++)
  {
    message->data[index] = can_rx_queue[can_rx_tail].data[index];
  }

  next_tail = can_rx_tail + 1U;
  if (next_tail >= CAN_RX_QUEUE_SIZE)
  {
    next_tail = 0U;
  }
  can_rx_tail = next_tail;

  return 1U;
}

void HAL_CAN_RxFifo0MsgPendingCallback(CAN_HandleTypeDef *can_handle)
{
  CAN_RxHeaderTypeDef rx_header = {0};
  uint8_t rx_data[CAN_MAX_DATA_LENGTH] = {0};
  uint8_t next_head;
  uint8_t index;

  if ((can_handle->Instance != CAN1) ||
      (HAL_CAN_GetRxMessage(can_handle, CAN_RX_FIFO0, &rx_header, rx_data) != HAL_OK))
  {
    return;
  }

  next_head = can_rx_head + 1U;
  if (next_head >= CAN_RX_QUEUE_SIZE)
  {
    next_head = 0U;
  }

  if (next_head == can_rx_tail)
  {
    return;
  }

  can_rx_queue[can_rx_head].std_id = rx_header.StdId;
  can_rx_queue[can_rx_head].length = (uint8_t)rx_header.DLC;
  for (index = 0U; index < can_rx_queue[can_rx_head].length; index++)
  {
    can_rx_queue[can_rx_head].data[index] = rx_data[index];
  }
  can_rx_head = next_head;
}

/* USER CODE END 1 */
