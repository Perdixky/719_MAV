#include "systerm.h"
#include "iic.h"
#include "usbd_cdc_if.h"
#include "719_PID.h"

void systerm_init(void)
{
	/********************************************************************
	�������ڽ����ж�
	********************************************************************/
	HAL_UART_Receive_IT(&huart1,(void *)&USART1_RXbuff,1); 
	HAL_UART_Receive_IT(&huart2,(void *)&USART2_RXbuff,1); 
	HAL_UART_Receive_IT(&huart3,(void *)&USART3_RXbuff,1); 
	HAL_UART_Receive_IT(&huart4,(void *)&USART4_RXbuff,1); 
	HAL_UART_Receive_IT(&huart5,(void *)&USART5_RXbuff,1); 
	/********************************************************************
	�ɿ�ϵͳ��ʱ��
	********************************************************************/
    HAL_TIM_Base_Start(&htim7); //�򿪶�ʱ��
    HAL_TIM_Base_Start(&htim4); //�򿪶�ʱ��
	
  /********************************************************************
	���PWM�����ʼ��
	********************************************************************/
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_1);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_2);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_3);
	HAL_TIM_PWM_Start(&htim3,TIM_CHANNEL_4);
	
	/********************************************************************
	��ʱ��ʱ����ʼ��
	********************************************************************/
	__HAL_TIM_CLEAR_FLAG(&htim5, TIM_FLAG_UPDATE);
	HAL_TIM_Base_Start_IT(&htim5);
	
    IIC_Init();
    PidParameter_init(); //PID������ʼ��

}
