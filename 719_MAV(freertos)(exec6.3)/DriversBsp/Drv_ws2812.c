/******************************************************************************************
模块名：传感器之交互灯模块
输入量：unlock_f标志位
输出量：无
作者：	719-张木悦
*******************************************************************************************/
#include "Drv_ws2812.h"
#include "cmsis_os.h"

uint16_t  RGB_buffur[RESET_PULSE + WS2812_DATA_LEN] = { 0 };  //48+4*24
 
 
void ws2812_set_RGB(uint8_t R, uint8_t G, uint8_t B, uint16_t num)
{
    //指针偏移:需要跳过复位信号的N个0
    uint16_t* p = (RGB_buffur + RESET_PULSE) + (num * LED_DATA_LEN);
    
    for (uint16_t i = 0; i < 8; i++)
    {
        //填充数组
        p[i]      = (G << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 8]  = (R << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
        p[i + 16] = (B << i) & (0x80)?ONE_PULSE:ZERO_PULSE;
    }
 
}
void HAL_TIM_PWM_PulseFinishedCallback(TIM_HandleTypeDef *htim)
{
  if(htim->Instance == TIM8) {
    if(htim->Channel == HAL_TIM_ACTIVE_CHANNEL_4) 
	{
      HAL_TIM_PWM_Stop_DMA(&htim8, TIM_CHANNEL_4);
    }   
  } 
}

 
void ws2812_example(void)
{	
	for(uint8_t i = 0xf0; i < 0xff; i++)
	{
		ws2812_set_RGB(0x00, i+2, i+3, 0); 
		ws2812_set_RGB(0x00, i+2, i+3, 1);  
		ws2812_set_RGB(0x00, i+2, i+3, 2);  
		ws2812_set_RGB(0x00, i+2, i+3, 3);  
        HAL_TIM_PWM_Start_DMA(&htim8, TIM_CHANNEL_4,(uint32_t *)RGB_buffur,(144));	
		HAL_Delay(1);
	}
}

extern osEventFlagsId_t ClassisEventHandle;
void ws2812_Server(void)
{
	int retval;
	int flag_s[GroupOfFlag] = {0};
	
//	retval = osEventFlagsWait(ClassisEventHandle, 1<<unlock_f, osFlagsNoClear, 0);
//	if(retval >= 0)
//		flag_s[unlock_f] = (retval >> unlock_f) & 0x01;
//	
//	if(flag_s[unlock_f] == 1)
//	{
    ws2812_example();
//	}

}
