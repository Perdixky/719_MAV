#ifndef __719_DT_H
#define __719_DT_H

#include "stm32f103xe.h"
#define False 0
#define True  1

typedef struct {
	float p;
	float i;
	float d;
	float limit;
}pid_receive;

typedef int Status;
//数据存入点
typedef struct {
	pid_receive angleRing_pitch;
	pid_receive angleRing_roll;

	pid_receive angularVelocity_pitch;
	pid_receive angularVelocity_roll;
	pid_receive angularVelocity_yaw;

	pid_receive heightRing;
	pid_receive	heightVelocity;
	pid_receive positionVelocity_X;
	pid_receive positionVelocity_Y;
}ADPID;

Status receive(uint8_t dataBuf);//串口中断中调用此函数 可接收上位机的pid参数写入
void drawLine(float num);//调用此函数可以在上位机中显示num的波形(波形刷新率10HZ)

float U82Float(uint8_t* u8Arry, int offset);
void unpackmsg(uint8_t* dataArr);

extern uint8_t ifmsgOK;
extern uint8_t msg_pack[148];
extern int judgement;
extern Status usb_rec_flag;						//存储USB接受PID参数标志位
extern ADPID usb_PID;

#endif

