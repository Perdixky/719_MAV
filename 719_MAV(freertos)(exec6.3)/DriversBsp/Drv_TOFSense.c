/******************************************************************************************
模块名：单点激光测距传感器
作者：719-李京泽
*******************************************************************************************/
#include "Drv_TOFSense.h"
#include "usart.h"

_TOFSense_st TOFSense;
static uint8_t _datatemp[50];


void TOFSense_init(void)
{
	uint8_t data[1];
	data[0]=0XA0;
HAL_UART_Transmit(&huart5,data,1,0XFFFF);



}

void TOFSense_GetOneByte(uint8_t data)
{
	static uint8_t rxstate = 0;

	if (rxstate == 0 &&data==0x57)
	{
		rxstate = 1;
		_datatemp[0] = data;
	}
	else if (rxstate == 1 )
	{
		rxstate = 2;
		_datatemp[1] = data;
	}
		else if (rxstate == 2 )
	{
		rxstate = 3;
		_datatemp[2] = data;
	}
		else if (rxstate == 3 )
	{
		rxstate = 4;
		_datatemp[3] = data;
	}
		else if (rxstate == 4 )
	{
		rxstate = 5;
		_datatemp[4] = data;
	}
		else if (rxstate == 5 )
	{
		rxstate = 6;
		_datatemp[5] = data;
	}
		else if (rxstate == 6 )
	{
		rxstate = 7;
		_datatemp[6] = data;
	}
		else if (rxstate == 7 )
	{
		rxstate = 8;
		_datatemp[7] = data;
	}
		else if (rxstate == 8 )
	{
		rxstate = 9;
		_datatemp[8] = data;
	}
		else if (rxstate == 9 )
	{
		rxstate = 10;
		_datatemp[9] = data;
	}
		else if (rxstate == 10 )
	{
		rxstate = 11;
		_datatemp[10] = data;
	}
		else if (rxstate == 11 )
	{
		rxstate = 12;
		_datatemp[11] = data;
	}
		else if (rxstate == 12 )
	{
		rxstate = 13;
		_datatemp[12] = data;
	}
		else if (rxstate == 13 )
	{
		rxstate = 14;
		_datatemp[13] = data;
	}
		else if (rxstate == 14 )
	{
		rxstate = 15;
		_datatemp[14] = data;
	}
		else if (rxstate == 15 )
	{
		rxstate = 16;
		_datatemp[15] = data;
	}

	
	else if (rxstate == 16&&data==0x3A)
	{
		rxstate = 0;
		_datatemp[16] = data;

		TOFSense_DataAnl(_datatemp); //
	}
	else
	{
		rxstate = 0;
	}
}

static void TOFSense_DataAnl(uint8_t *data)
{
	TOFSense.distance=data[8]+(data[9]<<8)+(data[10]<<16);//单位mm
	//printf("%f                ",TOFSense.distance);
}
