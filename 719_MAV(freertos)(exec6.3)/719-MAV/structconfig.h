#ifndef _STRUCTCONFIG_H
#define _STRUCTCONFIG_H
#include "stdio.h"
#include "stdint.h"


//typedef struct
//{
//	float X;
//	float Y;
//	float Z;
//}FLOAT_XYZ;

//typedef struct
//{
//	float rol;
//	float pit;
//	float yaw;
//}FLOAT_ANGLE;

////遥控器的数据结构 
//typedef struct
//{
//	int16_t ROLL;
//	int16_t PITCH;
//	int16_t THROTTLE;
//	int16_t YAW;
//	int16_t BUTTON1;
//}RC_TYPE;

//PID算法的数据结构 
//typedef struct PID
//{
//  float P;         //参数
//  float I;
//  float Error;     //比例项
//  float Integral;  //积分项
//  float Differ;    //微分项
//  float PreError;
//  float PrePreError;
//  float Ilimit; 
//  float Irang;
//  float Pout;
//  float Iout;
//  float Dout;
//	float Dout_pre;					//以上一个Dout进行迭代（新增）
//  float OutPut;   
//  uint8_t Ilimit_flag;    //积分分离
//	//新增
//	float err_i;						//误差的积分值

//	
//	//新增
//	float target;						//期望值
//	float target_pre;				//上一次期望值
//	float feedback;					//反馈值
//	float feedback_pre;			//上一次反馈值
//	float feedforward_k;		//前馈系数
//	float fb_k1;						//微分系数（C2）
//	float fb_k2;						//微分先行系数(C3)
//	float iteration_k;			//迭代系数（C1）

//}PID_TYPE;

////角度环PID
//extern PID_TYPE PID[ROL][ANGLE];
//extern PID_TYPE PID[PIT][ANGLE];
//extern PID_TYPE PID[YAW][ANGLE];
////角速度环PID
//extern PID_TYPE PID[ROL][RATE];
//extern PID_TYPE PID[PIT][RATE];
//extern PID_TYPE PID[YAW][RATE];

//extern RC_TYPE  RC_Control;
extern uint8_t lock;
extern uint8_t loop;
extern float   THROTTLE;

#endif

