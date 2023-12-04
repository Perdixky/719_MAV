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
	unlock_f = 0,					//解锁标志位
	imu_init_f = 1,				//imu初始化标志位
	rc_mode_f = 2,				//手动控制模式
	fix_f = 3,						//定点模式
	realframe_f = 4,			//实时帧模式
	LocControl_1_f = 5,
	GroupOfFlag = 6,
};

//遥控器的数据结构 
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
  float P;         //参数
  float I;
  float Error;     //比例项
  float Integral;  //积分项
  float Differ;    //微分项
  float PreError;
  float PrePreError;
  float Ilimit; 
  float Irang;
  float Pout;
  float Iout;
  float Dout;
	float Dout_pre;					//以上一个Dout进行迭代（新增）
  float OutPut;   
  uint8_t Ilimit_flag;    //积分分离
	//新增
	float err_i;						//误差的积分值

	
	//新增
	float target;						//期望值
	float target_pre;				//上一次期望值
	float feedback;					//反馈值
	float feedback_pre;			//上一次反馈值
	float feedforward_k;		//前馈系数
	float fb_k1;						//微分系数（C2）
	float fb_k2;						//微分先行系数(C3)
	float iteration_k;			//迭代系数（C1）

}PID_TYPE;
//角度环+角速度环PID


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
	int16_t RealSpeed_x; //单位mm/s
	int16_t RealSpeed_y;	//单位mm/s
	int16_t RealSpeed_z;	//单位mm/s
}OF;

extern int16_t sreg[REGSIZE];

typedef struct
{
	_flight_state_st fs;						//遥控器设定飞控状态，三轴速度
	_fly_ct_st program_ctrl;			//程序设定飞控状态
	RC_TYPE  RC_Control;						//遥控器通道量
	FLOAT_ANGLE oular_angle;				//三轴欧拉角
	FLOAT_XYZ gyro;									//三轴角速度	
	FLOAT_XYZ facc;									//三轴角速度				
	int16_t sreg[REGSIZE];					//维特智能传感器
	OF Of;													//光流数据	
}Classis_Container;

void Classis_Interface_in(char* fmt, ...);

void Classis_Interface_out(char* fmt, ...);

void Flag_Interface_in(int Flag[GroupOfFlag]);

void Flag_Interface_out(int Flag[GroupOfFlag]);

#endif

