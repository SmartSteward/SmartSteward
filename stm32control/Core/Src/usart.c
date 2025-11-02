/* USER CODE BEGIN Header */
/**
 ******************************************************************************
 * @file    usart.c
 * @brief   This file provides code for the configuration
 *          of the USART instances.
 ******************************************************************************
 * @attention
 *
 * Copyright (c) 2025 STMicroelectronics.
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
#include "usart.h"

/* USER CODE BEGIN 0 */

int __io_putchar(int ch) {
  HAL_UART_Transmit(&huart1, (uint8_t *)&ch, 1, HAL_MAX_DELAY);
  return ch;
}

u8 Usart1_Receive_buf[1];

/* USER CODE END 0 */

UART_HandleTypeDef huart1;

/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */

  /* USER CODE END USART1_Init 0 */

  /* USER CODE BEGIN USART1_Init 1 */

  /* USER CODE END USART1_Init 1 */
  huart1.Instance = USART1;
  huart1.Init.BaudRate = 115200;
  huart1.Init.WordLength = UART_WORDLENGTH_8B;
  huart1.Init.StopBits = UART_STOPBITS_1;
  huart1.Init.Parity = UART_PARITY_NONE;
  huart1.Init.Mode = UART_MODE_TX_RX;
  huart1.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart1.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart1) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART1_Init 2 */

  /* USER CODE END USART1_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspInit 0 */

  /* USER CODE END USART1_MspInit 0 */
    /* USART1 clock enable */
    __HAL_RCC_USART1_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_9;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART1 interrupt Init */
    HAL_NVIC_SetPriority(USART1_IRQn, 1, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==USART1)
  {
  /* USER CODE BEGIN USART1_MspDeInit 0 */

  /* USER CODE END USART1_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART1_CLK_DISABLE();

    /**USART1 GPIO Configuration
    PA9     ------> USART1_TX
    PA10     ------> USART1_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_9|GPIO_PIN_10);

    /* USART1 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspDeInit 1 */

  /* USER CODE END USART1_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
void HAL_UART_RxCpltCallback(UART_HandleTypeDef *UartHandle) // 接收中断回调函数
{
  if (UartHandle->Instance == USART1) // 串口1接收：改为APP按键控制协议
  {
    static int serial_receive = 0; // 接收字节
    static u8 Flag_PID, i, j, Receive[50];
    static float Data; // APP参数调节数据

    serial_receive = Usart1_Receive_buf[0];

    // 强制为APP按键控制模式
    // 按键方向：0x41~0x48, 0x5A停止，其它兼容<=10的旧协议
    if (serial_receive >= 0x41 && serial_receive <= 0x48)
      Flag_Direction = serial_receive - 0x40;
    else if (serial_receive == 0x5A)
      Flag_Direction = 0;
    else if (serial_receive <= 10)
      Flag_Direction = serial_receive;

    // 速度调节
    else if (serial_receive == 0x59) { // 减速
      if ((RC_Velocity -= X_Step) < MINI_RC_Velocity)
        RC_Velocity = MINI_RC_Velocity;
    } else if (serial_receive == 0x58) { // 加速
      if ((RC_Velocity += X_Step) > MAX_RC_Velocity)
        RC_Velocity = MAX_RC_Velocity;
    }

    // APP 参数帧 { ... }
    if (serial_receive == 0x7B)
      Flag_PID = 1; // 起始
    else if (serial_receive == 0x7D)
      Flag_PID = 2; // 结束

    if (Flag_PID == 1) {
      Receive[i++] = serial_receive;
    }
    if (Flag_PID == 2) {
      if (Receive[3] == 0x50)
        PID_Send = 1;
      else if (Receive[1] != 0x23) {
        for (j = i; j >= 4; j--) {
          Data += (Receive[j - 1] - 48) * pow(10, i - j);
        }
        switch (Receive[1]) {
        case 0x30:
          Velocity_KP = Data;
          break;
        case 0x31:
          Velocity_KI = Data;
          break;
        case 0x32:
          RC_Velocity = Data;
          break;
        default:
          break;
        }
      }
      Flag_PID = 0;
      i = 0;
      j = 0;
      Data = 0;
      memset(Receive, 0, sizeof(u8) * 50);
    }

    HAL_UART_Receive_IT(&huart1, Usart1_Receive_buf,
                        sizeof(Usart1_Receive_buf));
  }
}
/* USER CODE END 1 */
