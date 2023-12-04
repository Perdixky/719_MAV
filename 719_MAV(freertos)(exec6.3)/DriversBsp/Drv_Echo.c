/******************************************************************************************
模块名：超声波传感器
作者：719-李京泽
*******************************************************************************************/
#include "Drv_Echo.h"
#include "usart.h"

_echo_st echo;
static uint8_t _datatemp[50];

Kalman KF;
void Kalman_Init()
{
		KF.Q = 0.0004;			//过程噪声可以认为是0
		KF.R = 0.0005;		//给一个较小的值，可以在debug中调节
		KF.Kg = 0;			
		KF.lastP = 1;		//lastP相当于上一次的值，初始值可以为1，不可以为0
		KF.x_hat = 0;		
}
void Echo_init(void)
{
	uint8_t data[1];
	data[0]=0XA0;
HAL_UART_Transmit(&huart5,data,1,0XFFFF);



}
void Kalman_Filter(Kalman *KF, float input)
{
		float output = 0, x_t;						//output为卡尔曼滤波计算值
		x_t = KF->x_hat;							//当前先验预测值 = 上一次最优值
		KF->nowP = KF->lastP + KF->Q;				//本次的协方差矩阵
		KF->Kg = KF->nowP / (KF->nowP + KF->R);		//卡尔曼增益系数计算
		output = x_t + KF->Kg*(input - x_t); 		//当前最优值
		KF->x_hat = output;							//更新最优值
		KF->lastP = (1 - KF->Kg) * KF->nowP;		//更新协方差矩阵
		
}
void Echo_GetOneByte(uint8_t data)
{
	static uint8_t rxstate = 0;

	if (rxstate == 0 )
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 )
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
	else if (rxstate == 2)
	{
		rxstate = 0;
		_datatemp[2] = data;

		Echo_DataAnl(_datatemp); //
	}
	else
	{
		rxstate = 0;
	}
}

static void Echo_DataAnl(uint8_t *data)
{
	Kalman_Filter(&KF,(float)((data[0]<<16)+(data[1]<<8)+data[2])/1000);
	echo.distance=KF.x_hat;//单位，mm
//	printf("%f                ",echo.distance);
}
