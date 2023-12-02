#include "systerm.h"
#include "iic.h"
#include "usbd_cdc_if.h"
#include "719_PID.h"

void systerm_init(void)
{
	/********************************************************************
	开启串口接收中断
	********************************************************************/
	HAL_UART_Receive_IT(&huart1,(void *)&USART1_RXbuff,1); 
	HAL_UART_Receive_IT(&huart2,(void *)&USART2_RXbuff,1); 
	HAL_UART_Receive_IT(&huart3,(void *)&USART3_RXbuff,1); 
	HAL_UART_Receive_IT(&huart4,(void *)&USART4_RXbuff,1); 
	HAL_UART_Receive_IT(&huart5,(void *)&USART5_RXbuff,1); 
	/********************************************************************
	飞控系统计时器
	********************************************************************/
    HAL_TIM_Base_Start(&htim7); //打开定时器
    HAL_TIM_Base_Start(&htim4); //打开定时器
	
  /********************************************************************
	电机PWM输出初始化
	********************************************************************/
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	/********************************************************************
	定时器时基初始化
	********************************************************************/
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim5);
	
    IIC_Init();
    PidParameter_init(); //PID参数初始化

}
