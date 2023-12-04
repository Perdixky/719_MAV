#ifndef __719_PID_H
#define __719_PID_H

#include "main.h"
#include "structconfig.h"
#include "719_Interface.h"
#include "719_FC.h"

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )

void PID_Postion_Cal(float dT_s,						//���ڣ���λ���룩
										 PID_TYPE*PID,					
										 float target,					//����ֵ���趨ֵ��
										 float measure, 				//����ֵ������ֵ��
										 float feedforward,			//ǰ��ֵ
										 int Integral_Flag			//���ַ����־
										);
void UnControl_Check(void);

#endif
