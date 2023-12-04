#ifndef   __719_FC_H
#define   __719_FC_H

#include "main.h"
#include "structconfig.h"
#include "719_Interface.h"
#include "tim.h"
#include "719_PID.h"
#include "Comprehensive_Hope_Set.h"
#include "719_DT.h"


void AttControl_2(float dT_s, int integral_flag_s);//外环
void AttControl_1(float dT_s, int integral_flag_s);//内环
void AltControl_2(float dT_s, float target_disance, int integral_flag_s);//外环
void AltControl_1(float dT_s, float target_distance, int integral_flag_s);//内环
void LocControl(float dT_s, int integral_flag_s);
//void Moto_Pwm(int16_t* Moto_PWM_f);
void PidParameter_init(void);
void Motcontrol(void);
void MotorPwm_Init(void);
void Control_Server(void);

//enum
//{
//	ROL = 0,
//	PIT = 1,
//	YAW = 2,
//	VEC_RPY,
//};

typedef struct
{
	float yaw_err;
  float exp_rol_adj;
	float exp_pit_adj;
	
	float exp_rol,exp_pit,exp_yaw;
	float fb_rol,fb_pit,fb_yaw;

}_att_2l_ct_st;
//extern _att_2l_ct_st att_2l_ct;

typedef struct
{
	float set_yaw_speed;
	
	float exp_angular_velocity[VEC_RPY];

	float fb_angular_velocity[VEC_RPY];
}_att_1l_ct_st;
//extern _att_1l_ct_st att_1l_ct;

extern RC_TYPE RC_Control;

#define G			9.80665f		      	// m/s^2	
#define RadtoDeg    57.324841f				//弧度到角度 (弧度 * 180/3.1415)
#define DegtoRad    0.0174533f				//角度到弧度 (角度 * 3.1415/180)
#define MAX_ANGLE 25.0f
#define MAX_SPEED 500.0f 
#define MAX_SPEED_YAW 250  //角度每秒
#define MAX_ROLLING_SPEED 1600  //角度每秒

#define ABS(x) ( (x)>0?(x):-(x) )

#endif

