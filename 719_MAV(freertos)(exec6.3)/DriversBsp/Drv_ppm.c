/******************************************************************************************
模块名：接收机PPM解析模块
输入量：无
输出量：rc_control遥控器通道量
说明  ：
PPM_Databuf[0]-------------横滚
PPM_Databuf[1]-------------俯仰
PPM_Databuf[2]-------------油门
PPM_Databuf[3]-------------偏航
PPM_Databuf[4]-------------飞行模式
PPM_Databuf[5]-------------锁
作者：719-张木悦，程子峻
*******************************************************************************************/
#include "Drv_ppm.h"
#include "719_PID.h"

#define PPM_Chn_Max 8//最大通道数
static uint16_t PPM_Sample_Cnt = 0;//通道
static uint32_t PPM_Time = 0;//获取通道时间
static uint16_t PPM_Okay = 0;//下一次解析状态
static int16_t PPM_Databuf[8] = {0};//所有通道的数组


void get_rc_data(void)
{
	RC_TYPE  rc_control;		
	
	Classis_Interface_in(NULL);							//数据接口，从全局变量中获取数据
	
	rc_control.ROLL = PPM_Databuf[0];
	rc_control.PITCH = PPM_Databuf[1];
	rc_control.THROTTLE = PPM_Databuf[2];
	rc_control.YAW = PPM_Databuf[3];
	rc_control.BUTTON1 = PPM_Databuf[4];
	rc_control.BUTTON2 = PPM_Databuf[5];
	
//	printf("%d, %d, %d, %d, %d, %d\r\n",rc_control.ROLL, rc_control.PITCH, rc_control.THROTTLE, rc_control.YAW, rc_control.BUTTON1, rc_control.BUTTON2);
	
	Classis_Interface_out("a", &rc_control);		//数据接口，发送数据到全局变量处。
}

//PPM解析中断回调函数
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_7)//判断是否为接收器产生的中断，例程设置为PIN8
    {
        PPM_Time = TIM4 ->CNT;//将定时数转存
        TIM4 -> CNT = 0;//计数器归零
        if (PPM_Okay == 1)//判断是否是新的一轮解析
        {
            PPM_Sample_Cnt++;//通道数+1
            PPM_Databuf[PPM_Sample_Cnt - 1] = PPM_Time;//把每一个通道的数值存入数组
            if (PPM_Sample_Cnt >= PPM_Chn_Max)//判断是否超过额定通道数
                PPM_Okay = 0;
        }
        if (PPM_Time >= 2050)//长时间无下降沿即无通道数据，进入下一轮解析
        {
            PPM_Okay = 1;
            PPM_Sample_Cnt = 0;
        }
    }
}

