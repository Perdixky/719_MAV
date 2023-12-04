#ifndef __DRV_WS2812_H
#define __DRV_WS2812_H

#include "main.h"
#include "dma.h"
#include "tim.h"
#include "719_Interface.h"
 
//用户修改参数区
#define ONE_PULSE        (59)                           //1 码计数个数
#define ZERO_PULSE       (29)                           //0 码计数个数
#define RESET_PULSE      (48)                           //80 复位电平个数（不能低于40）
#define LED_NUMS         (4)                            //LED 个数
#define LED_DATA_LEN     (24)                           //LED 长度，单个需要24个字节
#define WS2812_DATA_LEN  (LED_NUMS*LED_DATA_LEN)        //ws2812灯条需要的数组长度

void ws2812_example(void);
void ws2812_Server(void);

#endif


