/******************************************************************************************
ģ������������������
���ߣ�719-���
*******************************************************************************************/
#include "Drv_Echo.h"
#include "usart.h"

_echo_st echo;
static uint8_t _datatemp[50];

Kalman KF;
void Kalman_Init()
{
		KF.Q = 0.0004;			//��������������Ϊ��0
		KF.R = 0.0005;		//��һ����С��ֵ��������debug�е���
		KF.Kg = 0;			
		KF.lastP = 1;		//lastP�൱����һ�ε�ֵ����ʼֵ����Ϊ1��������Ϊ0
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
		float output = 0, x_t;						//outputΪ�������˲�����ֵ
		x_t = KF->x_hat;							//��ǰ����Ԥ��ֵ = ��һ������ֵ
		KF->nowP = KF->lastP + KF->Q;				//���ε�Э�������
		KF->Kg = KF->nowP / (KF->nowP + KF->R);		//����������ϵ������
		output = x_t + KF->Kg*(input - x_t); 		//��ǰ����ֵ
		KF->x_hat = output;							//��������ֵ
		KF->lastP = (1 - KF->Kg) * KF->nowP;		//����Э�������
		
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
	echo.distance=KF.x_hat;//��λ��mm
//	printf("%f                ",echo.distance);
}
