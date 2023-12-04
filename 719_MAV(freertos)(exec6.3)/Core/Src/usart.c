/* USER CODE BEGIN Header */
/**
  ******************************************************************************
  * @file    usart.c
  * @brief   This file provides code for the configuration
  *          of the USART instances.
  ******************************************************************************
  * @attention
  *
  * Copyright (c) 2023 STMicroelectronics.
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
#include "Drv_Of.h"
#include "Drv_Echo.h"
void NoUse(uint8_t data){}
#define U1GetOneByte	OpiticalFlow_Receive
#define U2GetOneByte	NoUse
#define U3GetOneByte	NoUse            //重定向目标，用于串口打印
#define U4GetOneByte	Echo_GetOneByte
#define U5GetOneByte	NoUse
/* USER CODE END 0 */

UART_HandleTypeDef huart4;
UART_HandleTypeDef huart5;
UART_HandleTypeDef huart1;
UART_HandleTypeDef huart2;
UART_HandleTypeDef huart3;

/* UART4 init function */
void MX_UART4_Init(void)
{

  /* USER CODE BEGIN UART4_Init 0 */

  /* USER CODE END UART4_Init 0 */

  /* USER CODE BEGIN UART4_Init 1 */

  /* USER CODE END UART4_Init 1 */
  huart4.Instance = UART4;
  huart4.Init.BaudRate = 9600;
  huart4.Init.WordLength = UART_WORDLENGTH_8B;
  huart4.Init.StopBits = UART_STOPBITS_1;
  huart4.Init.Parity = UART_PARITY_NONE;
  huart4.Init.Mode = UART_MODE_TX_RX;
  huart4.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart4.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart4) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART4_Init 2 */

  /* USER CODE END UART4_Init 2 */

}
/* UART5 init function */
void MX_UART5_Init(void)
{

  /* USER CODE BEGIN UART5_Init 0 */
//==================
//=====用于超声波
//==================
  /* USER CODE END UART5_Init 0 */

  /* USER CODE BEGIN UART5_Init 1 */

  /* USER CODE END UART5_Init 1 */
  huart5.Instance = UART5;
  huart5.Init.BaudRate = 115200;
  huart5.Init.WordLength = UART_WORDLENGTH_8B;
  huart5.Init.StopBits = UART_STOPBITS_1;
  huart5.Init.Parity = UART_PARITY_NONE;
  huart5.Init.Mode = UART_MODE_TX_RX;
  huart5.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart5.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart5) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN UART5_Init 2 */

  /* USER CODE END UART5_Init 2 */

}
/* USART1 init function */

void MX_USART1_UART_Init(void)
{

  /* USER CODE BEGIN USART1_Init 0 */
//==================
//=====用于光流
//==================
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
/* USART2 init function */

void MX_USART2_UART_Init(void)
{

  /* USER CODE BEGIN USART2_Init 0 */

  /* USER CODE END USART2_Init 0 */

  /* USER CODE BEGIN USART2_Init 1 */

  /* USER CODE END USART2_Init 1 */
  huart2.Instance = USART2;
  huart2.Init.BaudRate = 115200;
  huart2.Init.WordLength = UART_WORDLENGTH_8B;
  huart2.Init.StopBits = UART_STOPBITS_1;
  huart2.Init.Parity = UART_PARITY_NONE;
  huart2.Init.Mode = UART_MODE_TX_RX;
  huart2.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart2.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart2) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART2_Init 2 */

  /* USER CODE END USART2_Init 2 */

}
/* USART3 init function */

void MX_USART3_UART_Init(void)
{

  /* USER CODE BEGIN USART3_Init 0 */

  /* USER CODE END USART3_Init 0 */

  /* USER CODE BEGIN USART3_Init 1 */

  /* USER CODE END USART3_Init 1 */
  huart3.Instance = USART3;
  huart3.Init.BaudRate = 9600;
  huart3.Init.WordLength = UART_WORDLENGTH_8B;
  huart3.Init.StopBits = UART_STOPBITS_1;
  huart3.Init.Parity = UART_PARITY_NONE;
  huart3.Init.Mode = UART_MODE_TX_RX;
  huart3.Init.HwFlowCtl = UART_HWCONTROL_NONE;
  huart3.Init.OverSampling = UART_OVERSAMPLING_16;
  if (HAL_UART_Init(&huart3) != HAL_OK)
  {
    Error_Handler();
  }
  /* USER CODE BEGIN USART3_Init 2 */

  /* USER CODE END USART3_Init 2 */

}

void HAL_UART_MspInit(UART_HandleTypeDef* uartHandle)
{

  GPIO_InitTypeDef GPIO_InitStruct = {0};
  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspInit 0 */

  /* USER CODE END UART4_MspInit 0 */
    /* UART4 clock enable */
    __HAL_RCC_UART4_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    /* UART4 interrupt Init */
    HAL_NVIC_SetPriority(UART4_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspInit 1 */

  /* USER CODE END UART4_MspInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspInit 0 */

  /* USER CODE END UART5_MspInit 0 */
    /* UART5 clock enable */
    __HAL_RCC_UART5_CLK_ENABLE();

    __HAL_RCC_GPIOC_CLK_ENABLE();
    __HAL_RCC_GPIOD_CLK_ENABLE();
    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_12;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOC, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOD, &GPIO_InitStruct);

    /* UART5 interrupt Init */
    HAL_NVIC_SetPriority(UART5_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspInit 1 */

  /* USER CODE END UART5_MspInit 1 */
  }
  else if(uartHandle->Instance==USART1)
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
    HAL_NVIC_SetPriority(USART1_IRQn, 5, 0);
    HAL_NVIC_EnableIRQ(USART1_IRQn);
  /* USER CODE BEGIN USART1_MspInit 1 */

  /* USER CODE END USART1_MspInit 1 */
  }
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspInit 0 */

  /* USER CODE END USART2_MspInit 0 */
    /* USART2 clock enable */
    __HAL_RCC_USART2_CLK_ENABLE();

    __HAL_RCC_GPIOA_CLK_ENABLE();
    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_2;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_3;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOA, &GPIO_InitStruct);

    /* USART2 interrupt Init */
    HAL_NVIC_SetPriority(USART2_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspInit 1 */

  /* USER CODE END USART2_MspInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspInit 0 */

  /* USER CODE END USART3_MspInit 0 */
    /* USART3 clock enable */
    __HAL_RCC_USART3_CLK_ENABLE();

    __HAL_RCC_GPIOB_CLK_ENABLE();
    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    GPIO_InitStruct.Pin = GPIO_PIN_10;
    GPIO_InitStruct.Mode = GPIO_MODE_AF_PP;
    GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    GPIO_InitStruct.Pin = GPIO_PIN_11;
    GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
    GPIO_InitStruct.Pull = GPIO_NOPULL;
    HAL_GPIO_Init(GPIOB, &GPIO_InitStruct);

    /* USART3 interrupt Init */
    HAL_NVIC_SetPriority(USART3_IRQn, 6, 0);
    HAL_NVIC_EnableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspInit 1 */

  /* USER CODE END USART3_MspInit 1 */
  }
}

void HAL_UART_MspDeInit(UART_HandleTypeDef* uartHandle)
{

  if(uartHandle->Instance==UART4)
  {
  /* USER CODE BEGIN UART4_MspDeInit 0 */

  /* USER CODE END UART4_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART4_CLK_DISABLE();

    /**UART4 GPIO Configuration
    PC10     ------> UART4_TX
    PC11     ------> UART4_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_10|GPIO_PIN_11);

    /* UART4 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART4_IRQn);
  /* USER CODE BEGIN UART4_MspDeInit 1 */

  /* USER CODE END UART4_MspDeInit 1 */
  }
  else if(uartHandle->Instance==UART5)
  {
  /* USER CODE BEGIN UART5_MspDeInit 0 */

  /* USER CODE END UART5_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_UART5_CLK_DISABLE();

    /**UART5 GPIO Configuration
    PC12     ------> UART5_TX
    PD2     ------> UART5_RX
    */
    HAL_GPIO_DeInit(GPIOC, GPIO_PIN_12);

    HAL_GPIO_DeInit(GPIOD, GPIO_PIN_2);

    /* UART5 interrupt Deinit */
    HAL_NVIC_DisableIRQ(UART5_IRQn);
  /* USER CODE BEGIN UART5_MspDeInit 1 */

  /* USER CODE END UART5_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART1)
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
  else if(uartHandle->Instance==USART2)
  {
  /* USER CODE BEGIN USART2_MspDeInit 0 */

  /* USER CODE END USART2_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART2_CLK_DISABLE();

    /**USART2 GPIO Configuration
    PA2     ------> USART2_TX
    PA3     ------> USART2_RX
    */
    HAL_GPIO_DeInit(GPIOA, GPIO_PIN_2|GPIO_PIN_3);

    /* USART2 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART2_IRQn);
  /* USER CODE BEGIN USART2_MspDeInit 1 */

  /* USER CODE END USART2_MspDeInit 1 */
  }
  else if(uartHandle->Instance==USART3)
  {
  /* USER CODE BEGIN USART3_MspDeInit 0 */

  /* USER CODE END USART3_MspDeInit 0 */
    /* Peripheral clock disable */
    __HAL_RCC_USART3_CLK_DISABLE();

    /**USART3 GPIO Configuration
    PB10     ------> USART3_TX
    PB11     ------> USART3_RX
    */
    HAL_GPIO_DeInit(GPIOB, GPIO_PIN_10|GPIO_PIN_11);

    /* USART3 interrupt Deinit */
    HAL_NVIC_DisableIRQ(USART3_IRQn);
  /* USER CODE BEGIN USART3_MspDeInit 1 */

  /* USER CODE END USART3_MspDeInit 1 */
  }
}

/* USER CODE BEGIN 1 */
int fputc(int ch, FILE *f)
{
    HAL_UART_Transmit(&huart3, (uint8_t *)&ch, 1, 0xFFFF);
    return ch;
}

uint8_t USART1_RXbuff;
uint8_t USART2_RXbuff;
uint8_t USART3_RXbuff;
uint8_t USART4_RXbuff;
uint8_t USART5_RXbuff;
uint8_t SBUS_FLAG = 0;

_rc_input_st rc_in;

static uint8_t datatmp[25];
static uint8_t cnt = 0;
static uint8_t SBUS_RC_State = 0;	
#define SBUS_RC_Reset (SBUS_RC_State = 0)

void HAL_UART_RxCpltCallback(UART_HandleTypeDef *huart)
{
	uint8_t rdata;
	
	if(huart->Instance == USART1)
    {
			rdata = USART1_RXbuff;
//		receive_data(SBUS_DATA);
//	  receive_sbus_data(rdata);
			U1GetOneByte(rdata);
    }
	if(huart->Instance == USART2)
    {

    }	
    if(huart->Instance == USART3)
    {
		
    }	
	if(huart->Instance == UART4)
    {
			rdata=USART4_RXbuff;
			U4GetOneByte(rdata);
    }	
	if(huart->Instance == UART5)
    {
			
    }		
	
	HAL_UART_Receive_IT(&huart1,(void *)&USART1_RXbuff,1);  /*再次开启接收中断*/
	HAL_UART_Receive_IT(&huart2,(void *)&USART2_RXbuff,1);  /*再次开启接收中断*/	
	HAL_UART_Receive_IT(&huart3,(void *)&USART3_RXbuff,1);  /*再次开启接收中断*/
	HAL_UART_Receive_IT(&huart4,(void *)&USART4_RXbuff,1);  /*再次开启接收中断*/
	HAL_UART_Receive_IT(&huart5,(void *)&USART5_RXbuff,1);  /*再次开启接收中断*/

}


void receive_sbus_data(uint8_t rdata)
{
	if(SBUS_RC_State == 0 && rdata == 0x0f)
	{
		++SBUS_RC_State;
		datatmp[cnt++] = rdata;
	}
	
	else if(SBUS_RC_State <= 24)
	{
		datatmp[cnt++] = rdata;
		++SBUS_RC_State;
		
	}else if(SBUS_RC_State == 25 && (datatmp[24] == 0x00 || datatmp[24] == 0x04 || datatmp[24] == 0x14 || datatmp[24] == 0x24 || datatmp[24] == 0x34) )
	{
		SBUS_RC_Reset;
		//数据解析
		cnt = 0;
		rc_in.sbus_ch[0] = (int16_t)(datatmp[2] & 0x07) << 8 | datatmp[1];
		rc_in.sbus_ch[1] = (int16_t)(datatmp[3] & 0x3f) << 5 | (datatmp[2] >> 3);
		rc_in.sbus_ch[2] = (int16_t)(datatmp[5] & 0x01) << 10 | ((int16_t)datatmp[4] << 2) | (datatmp[3] >> 6);
		rc_in.sbus_ch[3] = (int16_t)(datatmp[6] & 0x0F) << 7 | (datatmp[5] >> 1);
		rc_in.sbus_ch[4] = (int16_t)(datatmp[7] & 0x7F) << 4 | (datatmp[6] >> 4);
		rc_in.sbus_ch[5] = (int16_t)(datatmp[9] & 0x03) << 9 | ((int16_t)datatmp[8] << 1) | (datatmp[7] >> 7);
		rc_in.sbus_ch[6] = (int16_t)(datatmp[10] & 0x1F) << 6 | (datatmp[9] >> 2);
		rc_in.sbus_ch[7] = (int16_t)datatmp[11] << 3 | (datatmp[10] >> 5);

		rc_in.sbus_ch[8] = (int16_t)(datatmp[13] & 0x07) << 8 | datatmp[12];
		rc_in.sbus_ch[9] = (int16_t)(datatmp[14] & 0x3f) << 5 | (datatmp[13] >> 3);
		rc_in.sbus_ch[10] = (int16_t)(datatmp[16] & 0x01) << 10 | ((int16_t)datatmp[15] << 2) | (datatmp[14] >> 6);
		rc_in.sbus_ch[11] = (int16_t)(datatmp[17] & 0x0F) << 7 | (datatmp[16] >> 1);
		rc_in.sbus_ch[12] = (int16_t)(datatmp[18] & 0x7F) << 4 | (datatmp[17] >> 4);
		rc_in.sbus_ch[13] = (int16_t)(datatmp[20] & 0x03) << 9 | ((int16_t)datatmp[19] << 1) | (datatmp[18] >> 7);
		rc_in.sbus_ch[14] = (int16_t)(datatmp[21] & 0x1F) << 6 | (datatmp[20] >> 2);
		rc_in.sbus_ch[15] = (int16_t)datatmp[22] << 3 | (datatmp[21] >> 5);
		
		for(uint8_t i = 0; i < 10; i++)
		{
			rc_in.sbus_buffer[i] = 0.644f * (rc_in.sbus_ch[i] - 1024) + 1500;
			SBUS_FLAG = 1;
		}
		
	}
	else
	{
		cnt = 0;
		SBUS_RC_Reset;
		for(uint8_t i = 0; i < 25; i++)
			datatmp[i] = 0;
	}	
}

void printf_CH(void)
{
	printf("CH[0]:%f\r\n",rc_in.sbus_buffer[0]);
	printf("CH[1]:%f\r\n",rc_in.sbus_buffer[1]);
	printf("CH[2]:%f\r\n",rc_in.sbus_buffer[2]);
	printf("CH[3]:%f\r\n",rc_in.sbus_buffer[3]);
	printf("CH[4]:%f\r\n",rc_in.sbus_buffer[4]);
	printf("CH[5]:%f\r\n",rc_in.sbus_buffer[5]);
	printf("CH[6]:%f\r\n",rc_in.sbus_buffer[6]);
	printf("CH[7]:%f\r\n",rc_in.sbus_buffer[7]);
	printf("CH[8]:%f\r\n",rc_in.sbus_buffer[8]);
	printf("CH[9]:%f\r\n",rc_in.sbus_buffer[9]);
}

void printf_temp(void)
{
    printf("temp1: %d\r\n",datatmp[0]);
	printf("temp2: %d\r\n",datatmp[1]);
    printf("temp3: %d\r\n",datatmp[2]);
    printf("temp4: %d\r\n",datatmp[3]);
    printf("temp5: %d\r\n",datatmp[4]);
    printf("temp6: %d\r\n",datatmp[5]);
	printf("temp7: %d\r\n",datatmp[6]);

    printf("temp8: %d\r\n",datatmp[7]);
    printf("temp9: %d\r\n",datatmp[8]);
    printf("temp10: %d\r\n",datatmp[9]);
    printf("temp11: %d\r\n",datatmp[10]);
    printf("temp12: %d\r\n",datatmp[11]);
    printf("temp13: %d\r\n",datatmp[12]);
    printf("temp14: %d\r\n",datatmp[13]);
    printf("temp15: %d\r\n",datatmp[14]);
    printf("temp16: %d\r\n",datatmp[15]);
    printf("temp17: %d\r\n",datatmp[16]);
    printf("temp18: %d\r\n",datatmp[17]);
    printf("temp19: %d\r\n",datatmp[18]);
    printf("temp20: %d\r\n",datatmp[19]);
    printf("temp21: %d\r\n",datatmp[20]);
    printf("temp22: %d\r\n",datatmp[21]);
    printf("temp23: %d\r\n",datatmp[22]);
    printf("temp24: %d\r\n",datatmp[23]);
    printf("temp25: %d\r\n",datatmp[24]);

}


void receive_data(uint8_t com_data)
{
    /*循环体变量*/
    uint8_t i;
    /*计数变量*/
    static uint8_t rx_counter = 0;//计数
    /*数据接收数组*/
    static uint8_t rx_buffer[30] = {0};
    /*数据传输状态位*/
    static uint8_t rx_state = 0;

    /*对数据进行校准，判断是否为有效数据*/
    if(rx_state == 0 && com_data == 0x0f)  //0x0f帧头
    {
        rx_state = 1;
        rx_buffer[rx_counter++] = com_data;
    }

    else if(rx_state == 1)
    {
        rx_buffer[rx_counter++] = com_data;
        if(rx_counter >= 25 && rx_buffer[rx_counter - 1] == 0x00)   //RxBuffer1接受满了,接收数据结束
        {
            rx_state = 3;
	    }
	}

    else if(rx_state == 3)//检测是否接受到结束标志
    {
		rx_state = 0;
		rx_counter = 0;
		for (i = 0; i < 30; i ++)
		    rx_buffer[i] = 0x00;      //将存放数据数组清零
    }

    else   //接收异常
    {
        rx_state = 0;
        rx_counter = 0;
        for(i = 0; i < 30; i++)
            rx_buffer[i] = 0x00;      //将存放数据数组清零
    }

}

/* USER CODE END 1 */
