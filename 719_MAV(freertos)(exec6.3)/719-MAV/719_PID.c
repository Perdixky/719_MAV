/******************************************************************************************
模块名：PID控制器
作者：	719-程子峻
*******************************************************************************************/
#include "719_PID.h"

void PID_Postion_Cal(float dT_s,						//周期（单位：秒）
										 PID_TYPE*PID,					
										 float target,					//期望值（设定值）
										 float measure, 				//反馈值（测量值）
										 float feedforward,			//前馈值
										 int Integral_Flag			//积分分离标志
										)
{
	float hz = safe_div(1.0f,dT_s,0);
	
	PID->target = target;
	PID->feedback = measure;
	PID->Error  = target - measure;           					    								//误差
	
	PID->Pout = PID->P * PID->Error;                        								//比例控制
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  								//积分控制
	PID->Dout = PID->fb_k1 * (PID->target - PID->target_pre) * hz
						- PID->fb_k2 * (PID->feedback - PID->feedback_pre) * hz;			//微分控制（采取匿名做法）

	if( Integral_Flag == 0 )  //飞机解锁之后再加入积分,防止积分过调
	{
		if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //积分分离（当被控量和设定值偏差较大时，取消积分作用）
		{PID->Ilimit_flag = 0;}
		else
		{
			PID->Ilimit_flag = 1;                               //加入积分（积分分离标志为1，即积分项参与调节）
			PID->Integral += PID->Error;                        //对误差进行积分
			PID->Integral = LIMIT(PID->Integral, -PID->Irang, PID->Irang);		//积分限幅           
		}
	}else
	{
		PID->Integral = 0;
	}
	
	PID->OutPut = PID->feedforward_k *feedforward + PID->Pout + PID->Iout + PID->Dout; //前馈 + 比例 + 积分 + 微分总控制
	
	PID->PreError = PID->Error ;                            //前一个误差值
	PID->target_pre = PID->target;													//新旧期望值迭代
	PID->feedback_pre = PID->feedback;											//新旧反馈值迭代
}

