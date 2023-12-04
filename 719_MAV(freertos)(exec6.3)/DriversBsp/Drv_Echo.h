#ifndef __DRV_ANO_OF_H
#define __DRV_ANO_OF_H


//==����/����
#include "stdint.h"
typedef struct
{
	//
	float distance;  //���������ݸ��¼�����
} _echo_st;
typedef struct
{
		float lastP;		//�ϴε�Э����
		float nowP;			//���ε�Э����
		float x_hat;		//�������˲��ļ���ֵ����Ϊ��������ֵ
		float Kg;			//����������ϵ��
		float Q;			//��������
		float R;			//��������
}Kalman;
//�ɿ�״̬

//==��������
extern _echo_st echo;
//==��������
//static
static void Echo_DataAnl(uint8_t *data_buf);

//public
void Echo_GetOneByte(uint8_t data);
void Echo_init(void);
void Kalman_Init(void);
#endif
