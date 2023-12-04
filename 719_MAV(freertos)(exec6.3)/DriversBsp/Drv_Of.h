#include "stm32f103xe.h"

#ifndef __DRV_OF_H

#define __DRV_OF_H

typedef struct
{
	uint8_t head;//定义帧头
	uint8_t dev_id;//设备ID
	uint8_t sys_id;//系统ID
	uint8_t mes_id;//消息ID
	uint8_t pac_que;//包序列
	uint8_t pay_len;//负载长度
	uint8_t payload[0x14];//数据负载
	uint8_t checksum;//校验（前面所有数据之和）
}OW_MESSAGE;

void OpiticalFlow_Receive(uint8_t rdata);
unsigned int fourCharToInt(unsigned char a, unsigned char b, unsigned char c, unsigned char d);
unsigned short twoCharToshort(unsigned char a, unsigned char b);
#endif
