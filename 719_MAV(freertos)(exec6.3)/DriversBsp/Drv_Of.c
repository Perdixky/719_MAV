/******************************************************************************************
ģ����������ģ��
����������
�������of��������,x,y���ٶ��Լ�z�����
���ߣ�	719-������
*******************************************************************************************/
#include "Drv_Of.h"
#include "usart.h"
#include "stm32f103xe.h"
#include <stdio.h>
#include "719_Interface.h"

static OF of_s;
unsigned int fourCharToInt(unsigned char a, unsigned char b, unsigned char c, unsigned char d)//4��char�ϳ�һ��int
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
unsigned short twoCharToshort(unsigned char a, unsigned char b)//����char�ϳ�һ��short
{
	unsigned short ret_val = 0;
	ret_val  |= a;
	ret_val <<= 8;
	ret_val |=  b;
	return ret_val;
}
typedef struct
{
	uint32_t sys_time;//ϵͳʱ��
	uint32_t	distance;//����ֵ
	uint8_t power;//�ź�ǿ��
	uint8_t	lev1;//Ԥ��
	uint8_t status;//״̬
	uint8_t lev2;//Ԥ��
	int16_t speed_x;//�����ٶ�x��mm/s
	int16_t speed_y;//�����ٶ�x��mm/s
	uint8_t ow_mass;//��������
	uint8_t ow_status;//����״̬
	uint16_t lev3;//Ԥ��
	
}OW_PAYLOAD;
void OpiticalFlow_Receive(uint8_t rdata)
{	
		static OW_MESSAGE ow_data;//��Ž�������
		static OW_PAYLOAD ow_payload;//������ݸ���
		static uint8_t sum=0;//��¼���ֵ
		static uint8_t rc_counter=0;//״̬��
//		printf("%d\r\n", rdata);
		if(rc_counter==0)
		{
			if(rdata!=0xEF)//֡ͷ���0xEF
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
		else if(rc_counter<6)//���ǰ5������
		{
			((uint8_t*)&ow_data)[rc_counter]=rdata;
			++rc_counter ;
			sum+=rdata;
		}
		else if(rc_counter<26)//������ݸ���
		{
			ow_data.payload[rc_counter-6]=rdata;
			sum+=rdata;
			++rc_counter;
		}
		else if(rc_counter==26)
		{	
			ow_data.checksum=rdata;
			if(sum==ow_data.checksum)//У��λ���
			{
				if(fourCharToInt(ow_data.payload[7],ow_data.payload[6],ow_data.payload[5],ow_data.payload[4])>=2)//����ֵ��СΪ2������ڽṹ���������char�͵�Ҫת��Ϊint
				{
					
					if(ow_data.payload[10]==1)//״̬Ϊ1��ʾ������ݿ���ʹ��
					{
						ow_payload.distance=fourCharToInt(ow_data.payload[7],ow_data.payload[6],ow_data.payload[5],ow_data.payload[4]);
						of_s.Distance =ow_payload.distance;	//��ֵ��Distance
						//printf("Distance:%u\n\r",Distance);
						//printf("raw1:%d\n\r",ow_data.payload[4]);
						//printf("raw2:%d\n\r",ow_data.payload[5]);
						//printf("raw3:%d\n\r",ow_data.payload[6]);
						//printf("raw4:%d\n\r",ow_data.payload[7]);
					}
			
					if(ow_data.payload[17]==1)//״̬Ϊ1��ʾ�������ݿ���ʹ��
					{
						ow_payload.speed_x=twoCharToshort(ow_data.payload[13],ow_data.payload[12]);
						ow_payload.speed_y=twoCharToshort(ow_data.payload[15],ow_data.payload[14]);
//						printf("Distance:%u\n\r",of_s.Distance);						printf("Rx:%d\n\r",ow_payload.speed_x);
//						printf("Ry:%d\n\r",ow_payload.speed_y);
					}
					if(ow_data.payload[10]==1&&ow_data.payload[17]==1)//�������ݺͲ�����ݶ����ò��ܼ�����ٶ�
					{
						of_s.RealSpeed_x =ow_payload.speed_x*(signed)(of_s.Distance)/100;//ʵ���ٶ�=�����ٶ�*�߶�
						of_s.RealSpeed_y =ow_payload.speed_y*(signed)(of_s.Distance)/100;
						printf("Distance:%u\n\r",of_s.Distance);
						printf("Rx:%d\n\r",of_s.RealSpeed_x);
						printf("Ry:%d\n\r",of_s.RealSpeed_y);
//						printf("RealSpeed_x:%d\n\r",RealSpeed_x);
//						printf("RealSpeed_y:%d\n\r",RealSpeed_y);
					}
				}
						rc_counter =0;//����ֵ����
						sum=0;//У��λ���������һ������
			}
		}
		
	Classis_Interface_out("g", &of_s);
}

void OpiticalFlow_Send(void)
{
	
}
