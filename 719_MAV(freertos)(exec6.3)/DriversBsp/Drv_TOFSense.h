/******************************************************************************************
ģ���������㼤���״ﴫ����
���ߣ�719-���
*******************************************************************************************/
#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H


//==����/����
#include "stdint.h"
typedef struct
{
	//
	uint32_t distance;  //�����״����ݡ�
} _TOFSense_st;
//�ɿ�״̬

//==��������
extern _TOFSense_st TOFSense;
//==��������
//static
static void TOFSense_DataAnl(uint8_t *data_buf);

//public
void TOFSense_GetOneByte(uint8_t data);
#endif

