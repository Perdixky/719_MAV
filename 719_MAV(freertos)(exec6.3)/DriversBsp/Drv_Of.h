#include "stm32f103xe.h"

#ifndef __DRV_OF_H

#define __DRV_OF_H

typedef struct
{
	uint8_t head;//����֡ͷ
	uint8_t dev_id;//�豸ID
	uint8_t sys_id;//ϵͳID
	uint8_t mes_id;//��ϢID
	uint8_t pac_que;//������
	uint8_t pay_len;//���س���
	uint8_t payload[0x14];//���ݸ���
	uint8_t checksum;//У�飨ǰ����������֮�ͣ�
}OW_MESSAGE;

void OpiticalFlow_Receive(uint8_t rdata);
unsigned int fourCharToInt(unsigned char a, unsigned char b, unsigned char c, unsigned char d);
unsigned short twoCharToshort(unsigned char a, unsigned char b);
#endif
