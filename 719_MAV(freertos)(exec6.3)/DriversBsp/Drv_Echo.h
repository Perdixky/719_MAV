#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H


//==定义/声明
#include "stdint.h"
typedef struct
{
	//
	float distance;  //超声波数据更新计数。
} _echo_st;
typedef struct
{
		float lastP;		//上次的协方差
		float nowP;			//本次的协方差
		float x_hat;		//卡尔曼滤波的计算值，即为后验最优值
		float Kg;			//卡尔曼增益系数
		float Q;			//过程噪声
		float R;			//测量噪声
}Kalman;
//飞控状态

//==数据声明
extern _echo_st echo;
//==函数声明
//static
static void Echo_DataAnl(uint8_t *data_buf);

//public
void Echo_GetOneByte(uint8_t data);
void Echo_init(void);
void Kalman_Init(void);
#endif
