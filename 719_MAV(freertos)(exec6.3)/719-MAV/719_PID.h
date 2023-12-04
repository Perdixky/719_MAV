#ifndef __719_PID_H
#define __719_PID_H

#include "main.h"
#include "structconfig.h"
#include "719_Interface.h"
#include "719_FC.h"

#define safe_div(numerator,denominator,safe_value) ( (denominator == 0)? (safe_value) : ((numerator)/(denominator)) )
#define LIMIT( x,min,max ) ( ((x) <= (min)) ? (min) : ( ((x) > (max))? (max) : (x) ) )

void PID_Postion_Cal(float dT_s,						//周期（单位：秒）
										 PID_TYPE*PID,					
										 float target,					//期望值（设定值）
										 float measure, 				//反馈值（测量值）
										 float feedforward,			//前馈值
										 int Integral_Flag			//积分分离标志
										);
void UnControl_Check(void);

#endif
