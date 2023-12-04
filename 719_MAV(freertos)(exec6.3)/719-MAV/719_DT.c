/******************************************************************************************
模块名：通信协议
输入量：底盘，传感器各种数据
输出量：暂无
说明	：
作者	：719-汤展博，罗梓龙
*******************************************************************************************/
#include "719_DT.h"
#include "usart.h"
#include "stm32f103xe.h"
#include <stdio.h>
#include "719_Interface.h"

static uint8_t ifmsgOK;
static uint8_t msg_pack[148];
ADPID usb_PID;
Status usb_rec_flag;						//存储USB接受usb_PID参数标志位

float U82Float(uint8_t* u8Arry, int offset)
{
	float result;
	uint8_t farray[4] = { 0 };

	farray[3] = u8Arry[3 + offset];
	farray[2] = u8Arry[2 + offset];
	farray[1] = u8Arry[1 + offset];
	farray[0] = u8Arry[offset];

	memcpy(&result, farray, 4);
	return result;
}

void Float2U8(uint8_t *u8Arry, float *floatdata, int offset)
{
    uint8_t farray[4];
    *(float *)farray = *floatdata;

    u8Arry[3+offset] = farray[3];
    u8Arry[2+offset] = farray[2];
    u8Arry[1+offset] = farray[1];
    u8Arry[offset] = farray[0];
}

int judgement = 0;

Status receive(uint8_t dataBuf)
{
	static int judgeNum = 0;
	static int indx = 0;
	if (dataBuf == 0xAA && judgeNum == 0)
	{
		judgeNum = 1;
		judgement = 1;
	}

	if (judgeNum == 1)
	{
		msg_pack[indx++] = dataBuf;
	}

	if (indx == 145)
	{
		judgeNum = indx = 0;
		judgement = 2;
		if (dataBuf == 0xFF)
		{
			judgement = 3;
			ifmsgOK = 0x01;
		}
	}
	if(ifmsgOK == 0x01)
	{
		unpackmsg(msg_pack);
		indx = ifmsgOK = 0;	
		return True;
	}
	return False;
}

void unpackmsg(uint8_t* dataArr)
{
	int index =1;
	usb_PID.angleRing_pitch.p = U82Float(dataArr, index);
	usb_PID.angleRing_pitch.i = U82Float(dataArr, index += 4);
	usb_PID.angleRing_pitch.d = U82Float(dataArr, index += 4);
	usb_PID.angleRing_pitch.limit = U82Float(dataArr, index += 4);

	usb_PID.angleRing_roll.p = U82Float(dataArr, index += 4);
	usb_PID.angleRing_roll.i = U82Float(dataArr, index += 4);
	usb_PID.angleRing_roll.d = U82Float(dataArr, index += 4);
	usb_PID.angleRing_roll.limit = U82Float(dataArr, index += 4);

	usb_PID.angularVelocity_pitch.p = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_pitch.i = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_pitch.d = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_pitch.limit = U82Float(dataArr, index += 4);

	usb_PID.angularVelocity_roll.p = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_roll.i = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_roll.d = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_roll.limit = U82Float(dataArr, index += 4);

	usb_PID.angularVelocity_yaw.p = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_yaw.i = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_yaw.d = U82Float(dataArr, index += 4);
	usb_PID.angularVelocity_yaw.limit = U82Float(dataArr, index += 4);

	usb_PID.heightRing.p = U82Float(dataArr, index += 4);
	usb_PID.heightRing.i = U82Float(dataArr, index += 4);
	usb_PID.heightRing.d = U82Float(dataArr, index += 4);
	usb_PID.heightRing.limit = U82Float(dataArr, index += 4);

	usb_PID.heightVelocity.p = U82Float(dataArr, index += 4);
	usb_PID.heightVelocity.i = U82Float(dataArr, index += 4);
	usb_PID.heightVelocity.d = U82Float(dataArr, index += 4);
	usb_PID.heightVelocity.limit = U82Float(dataArr, index += 4);

	usb_PID.positionVelocity_X.p = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_X.i = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_X.d = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_X.limit = U82Float(dataArr, index += 4);

	usb_PID.positionVelocity_Y.p = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_Y.i = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_Y.d = U82Float(dataArr, index += 4);
	usb_PID.positionVelocity_Y.limit = U82Float(dataArr, index += 4);
}

void drawLine(float num)
{
	uint8_t dataBuf[6] = {0};
	dataBuf[0] = 0xAA;
	Float2U8(dataBuf, &num, 1);
	dataBuf[5] = 0xBB;
	
//	HAL_UART_Transmit(&huart1 , dataBuf, 6, 0xFFFF);
}
