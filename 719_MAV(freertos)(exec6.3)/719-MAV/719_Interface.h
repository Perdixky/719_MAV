#ifndef __719_INTERFACE_H
#define	__719_INTERFACE_H
#include "stdio.h"
#include "stdint.h"
#include "Drv_wit_c_sdk.h"
#include <stdarg.h>

//#include "XYZ_Velocity_Set.h"
enum
{
	ppm_analize = 1,
	imu_get = 2,
	control_model = 3,
	ws2812_model = 4,
};

enum
{
	ROL = 0,
	PIT = 1,
	YAW = 2,
	VEC_RPY,
};

enum
{
	ANGLE = 0,
	RATE = 1,
};

enum
{
	X = 0,
	Y = 1,
	Z = 2,
	VEC_XYZ,
};

enum
{
	unlock_f = 0,					//������־λ
	imu_init_f = 1,				//imu��ʼ����־λ
	rc_mode_f = 2,				//�ֶ�����ģʽ
	fix_f = 3,						//����ģʽ
	realframe_f = 4,			//ʵʱ֡ģʽ
	LocControl_1_f = 5,
	GroupOfFlag = 6,
};

//ң���������ݽṹ 
typedef struct
{
	int16_t ROLL;
	int16_t PITCH;
	int16_t THROTTLE;
	int16_t YAW;
	int16_t BUTTON1;
	int16_t BUTTON2;
}RC_TYPE;

typedef struct
{
	float X;
	float Y;
	float Z;
}FLOAT_XYZ;

typedef struct
{
	float rol;
	float pit;
	float yaw;
}FLOAT_ANGLE;

typedef struct PID
{
  float P;         //����
  float I;
  float Error;     //������
  float Integral;  //������
  float Differ;    //΢����
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
	float Dout_pre;					//����һ��Dout���е�����������
  float OutPut;   
  uint8_t Ilimit_flag;    //���ַ���
	//����
	float err_i;						//���Ļ���ֵ

	
	//����
	float target;						//����ֵ
	float target_pre;				//��һ������ֵ
	float feedback;					//����ֵ
	float feedback_pre;			//��һ�η���ֵ
	float feedforward_k;		//ǰ��ϵ��
	float fb_k1;						//΢��ϵ����C2��
	float fb_k2;						//΢������ϵ��(C3)
	float iteration_k;			//����ϵ����C1��

}PID_TYPE;
//�ǶȻ�+���ٶȻ�PID


typedef struct
{
	int alt_ctrl_speed_set;
	float speed_set_h[VEC_XYZ];	
	float speed_set_h_cms[VEC_XYZ];
	
	float distance_set_h[VEC_XYZ];
	
	float speed_set_h_norm[VEC_XYZ];
	float speed_set_h_norm_lpf[VEC_XYZ];
	
}_flight_state_st;

typedef struct
{
	float vel_cmps_h[3];
	float distance_cmps_h[3];
	
}_fly_ct_st;

typedef struct
{
	uint32_t Distance;
	int16_t RealSpeed_x; //��λmm/s
	int16_t RealSpeed_y;	//��λmm/s
	int16_t RealSpeed_z;	//��λmm/s
}OF;

extern int16_t sreg[REGSIZE];

typedef struct
{
	_flight_state_st fs;						//ң�����趨�ɿ�״̬�������ٶ�
	_fly_ct_st program_ctrl;			//�����趨�ɿ�״̬
	RC_TYPE  RC_Control;						//ң����ͨ����
	FLOAT_ANGLE oular_angle;				//����ŷ����
	FLOAT_XYZ gyro;									//������ٶ�	
	FLOAT_XYZ facc;									//������ٶ�				
	int16_t sreg[REGSIZE];					//ά�����ܴ�����
	OF Of;													//��������	
}Classis_Container;

void Classis_Interface_in(char* fmt, ...);

void Classis_Interface_out(char* fmt, ...);

void Flag_Interface_in(int Flag[GroupOfFlag]);

void Flag_Interface_out(int Flag[GroupOfFlag]);

#endif

