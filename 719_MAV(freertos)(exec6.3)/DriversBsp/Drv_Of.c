/******************************************************************************************
模块名：光流模块
输入量：无
输出量：of光流数据,x,y轴速度以及z轴距离
作者：	719-罗梓龙
*******************************************************************************************/
#include "Drv_Of.h"
#include "usart.h"
#include "stm32f103xe.h"
#include <stdio.h>
#include "719_Interface.h"

static OF of_s;
unsigned int fourCharToInt(unsigned char a, unsigned char b, unsigned char c, unsigned char d)//4个char合成一个int
{
	unsigned int ret_val = 0;
	ret_val  =  a;
	ret_val <<= 8;
	ret_val |=  b;
	ret_val <<= 8;
	ret_val |=  c;
	ret_val <<= 8;
	ret_val |=  d;
	return ret_val;
}
unsigned short twoCharToshort(unsigned char a, unsigned char b)//两个char合成一个short
{
	unsigned short ret_val = 0;
	ret_val  |= a;
	ret_val <<= 8;
	ret_val |=  b;
	return ret_val;
}
typedef struct
{
	uint32_t sys_time;//系统时间
	uint32_t	distance;//距离值
	uint8_t power;//信号强度
	uint8_t	lev1;//预留
	uint8_t status;//状态
	uint8_t lev2;//预留
	int16_t speed_x;//光流速度x轴mm/s
	int16_t speed_y;//光流速度x轴mm/s
	uint8_t ow_mass;//光流质量
	uint8_t ow_status;//光流状态
	uint16_t lev3;//预留
	
}OW_PAYLOAD;
void OpiticalFlow_Receive(uint8_t rdata)
{	
		static OW_MESSAGE ow_data;//存放接受数据
		static OW_PAYLOAD ow_payload;//存放数据负载
		static uint8_t sum=0;//记录求和值
		static uint8_t rc_counter=0;//状态机
//		printf("%d\r\n", rdata);
		if(rc_counter==0)
		{
			if(rdata!=0xEF)//帧头检测0xEF
			{
				rc_counter =0;
			}
			else 
			{
				++rc_counter;
				ow_data.head =rdata;
				sum+=rdata;
			}
		}
		else if(rc_counter<6)//存放前5个数据
		{
			((uint8_t*)&ow_data)[rc_counter]=rdata;
			++rc_counter ;
			sum+=rdata;
		}
		else if(rc_counter<26)//存放数据负载
		{
			ow_data.payload[rc_counter-6]=rdata;
			sum+=rdata;
			++rc_counter;
		}
		else if(rc_counter==26)
		{	
			ow_data.checksum=rdata;
			if(sum==ow_data.checksum)//校验位检测
			{
				if(fourCharToInt(ow_data.payload[7],ow_data.payload[6],ow_data.payload[5],ow_data.payload[4])>=2)//距离值最小为2，存放在结构体的数据是char型的要转化为int
				{
					
					if(ow_data.payload[10]==1)//状态为1表示测距数据可以使用
					{
						ow_payload.distance=fourCharToInt(ow_data.payload[7],ow_data.payload[6],ow_data.payload[5],ow_data.payload[4]);
						of_s.Distance =ow_payload.distance;	//赋值给Distance
						//printf("Distance:%u\n\r",Distance);
						//printf("raw1:%d\n\r",ow_data.payload[4]);
						//printf("raw2:%d\n\r",ow_data.payload[5]);
						//printf("raw3:%d\n\r",ow_data.payload[6]);
						//printf("raw4:%d\n\r",ow_data.payload[7]);
					}
			
					if(ow_data.payload[17]==1)//状态为1表示光流数据可以使用
					{
						ow_payload.speed_x=twoCharToshort(ow_data.payload[13],ow_data.payload[12]);
						ow_payload.speed_y=twoCharToshort(ow_data.payload[15],ow_data.payload[14]);
//						printf("Distance:%u\n\r",of_s.Distance);						printf("Rx:%d\n\r",ow_payload.speed_x);
//						printf("Ry:%d\n\r",ow_payload.speed_y);
					}
					if(ow_data.payload[10]==1&&ow_data.payload[17]==1)//光流数据和测距数据都可用才能计算出速度
					{
						of_s.RealSpeed_x =ow_payload.speed_x*(signed)(of_s.Distance)/100;//实际速度=光流速度*高度
						of_s.RealSpeed_y =ow_payload.speed_y*(signed)(of_s.Distance)/100;
						printf("Distance:%u\n\r",of_s.Distance);
						printf("Rx:%d\n\r",of_s.RealSpeed_x);
						printf("Ry:%d\n\r",of_s.RealSpeed_y);
//						printf("RealSpeed_x:%d\n\r",RealSpeed_x);
//						printf("RealSpeed_y:%d\n\r",RealSpeed_y);
					}
				}
						rc_counter =0;//计数值清零
						sum=0;//校验位清零接受下一组数据
			}
		}
		
	Classis_Interface_out("g", &of_s);
}

void OpiticalFlow_Send(void)
{
	
}
