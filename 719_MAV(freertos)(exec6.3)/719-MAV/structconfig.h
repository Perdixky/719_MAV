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

////ң���������ݽṹ 
//typedef struct
//{
//	int16_t ROLL;
//	int16_t PITCH;
//	int16_t THROTTLE;
//	int16_t YAW;
//	int16_t BUTTON1;
//}RC_TYPE;

//PID�㷨�����ݽṹ 
//typedef struct PID
//{
//  float P;         //����
//  float I;
//  float Error;     //������
//  float Integral;  //������
//  float Differ;    //΢����
//  float PreError;
//  float PrePreError;
//  float Ilimit; 
//  float Irang;
//  float Pout;
//  float Iout;
//  float Dout;
//	float Dout_pre;					//����һ��Dout���е�����������
//  float OutPut;   
//  uint8_t Ilimit_flag;    //���ַ���
//	//����
//	float err_i;						//���Ļ���ֵ

//	
//	//����
//	float target;						//����ֵ
//	float target_pre;				//��һ������ֵ
//	float feedback;					//����ֵ
//	float feedback_pre;			//��һ�η���ֵ
//	float feedforward_k;		//ǰ��ϵ��
//	float fb_k1;						//΢��ϵ����C2��
//	float fb_k2;						//΢������ϵ��(C3)
//	float iteration_k;			//����ϵ����C1��

//}PID_TYPE;

////�ǶȻ�PID
//extern PID_TYPE PID[ROL][ANGLE];
//extern PID_TYPE PID[PIT][ANGLE];
//extern PID_TYPE PID[YAW][ANGLE];
////���ٶȻ�PID
//extern PID_TYPE PID[ROL][RATE];
//extern PID_TYPE PID[PIT][RATE];
//extern PID_TYPE PID[YAW][RATE];

//extern RC_TYPE  RC_Control;
extern uint8_t lock;
extern uint8_t loop;
extern float   THROTTLE;

#endif

