/******************************************************************************************
模块名：单点激光雷达传感器
作者：719-李京泽
*******************************************************************************************/
#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H


//==定义/声明
#include "stdint.h"
typedef struct
{
	//
	uint32_t distance;  //激光雷达数据。
} _TOFSense_st;
//飞控状态

//==数据声明
extern _TOFSense_st TOFSense;
//==函数声明
//static
static void TOFSense_DataAnl(uint8_t *data_buf);

//public
void TOFSense_GetOneByte(uint8_t data);
#endif

