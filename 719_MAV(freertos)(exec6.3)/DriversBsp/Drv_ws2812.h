#ifndef __DRV_WS2812_H
#define __DRV_WS2812_H

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "719_Interface.h"
 
//�û��޸Ĳ�����
#define ONE_PULSE        (59)                           //1 ���������
#define ZERO_PULSE       (29)                           //0 ���������
#define RESET_PULSE      (48)                           //80 ��λ��ƽ���������ܵ���40��
#define LED_NUMS         (4)                            //LED ����
#define LED_DATA_LEN     (24)                           //LED ���ȣ�������Ҫ24���ֽ�
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812������Ҫ�����鳤��

void ws2812_example(void);
void ws2812_Server(void);

#endif


