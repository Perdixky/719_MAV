#ifndef __COMPREHENSIVE_HOPE_SET_H
#define __COMPREHENSIVE_HOPE_SET_H

#include "main.h"
#include "structconfig.h"
#include "719_Interface.h"
#include "tim.h"
#include "719_PID.h"
#include "719_FC.h"

#define my_2_norm(x,y) (my_sqrt(my_pow((x)) + my_pow((y))))
#define my_pow(a) ((a)*(a))
#define LPF_1_(hz,t,in,out) ((out) += ( 1 / ( 1 + 1 / ( (hz) *6.28f *(t) ) ) ) *( (in) - (out) ))	

#define MAX_Z_SPEED_UP 350 //厘米每秒 cm/s
#define MAX_Z_SPEED_DW 250 //厘米每秒 cm/s
#define CHANNEL_VALUE_TO_VELOCITY  (350/500)   //厘米每秒
//enum
//{
//	X = 0,
//	Y = 1,
//	Z = 2,
//	VEC_XYZ,
//};

//typedef struct
//{
//	int alt_ctrl_speed_set;
//	float speed_set_h[VEC_XYZ];	
//	float speed_set_h_cms[VEC_XYZ];
//	
//	float speed_set_h_norm[VEC_XYZ];
//	float speed_set_h_norm_lpf[VEC_XYZ];
//	
//}_flight_state_st;
//extern _flight_state_st fs;

typedef struct
{
	float vel_limit_xy;
	float vel_limit_z_p;
	float vel_limit_z_n;
	float yaw_pal_limit;
}_fc_sta_var_st; //state variable

typedef struct
{
	//
	float vel_cmps_set_h[3];
	float vel_cmps_set_w[3];
	float vel_cmps_set_ref[3];
	//
	float vel_cmps_set_z;
	float pal_dps_set;
}_pc_user_st;

//program_ctrl_s，pc_user_s虽然是全局变量，但是却并无具体用图，不会发生数据冒险。（还未补充）
//typedef struct
//{
//	//sta
//	int state_ok;
//	int cmd_state[2];
//	//time
//	float fb_process_t_ms[4];
//	float exp_process_t_ms[4];
//	
//	//ctrl
//	float ref_dir[2];
//	float vel_cmps_ref[3];
//	float vel_cmps_w[3];
//	float vel_cmps_h[3];
//	int yaw_pal_dps;
//	
//}_fly_ct_st;
//extern _fly_ct_st program_ctrl_s;

//typedef struct
//{
//	//
//	float vel_cmps_set_h[2];
//	float vel_cmps_set_w[2];
//	float vel_cmps_set_ref[2];
//	//
//	float vel_cmps_set_z;
//	float pal_dps_set;
//}_pc_user_st;
//extern _pc_user_st pc_user_s;

typedef struct
{
	float exp[VEC_XYZ];
	float fb[VEC_XYZ];

	
	float out[VEC_XYZ];
}_loc_ctrl_st;// loc_ctrl;
//extern _loc_ctrl_st loc_ctrl_1;

float my_sqrt(float number);
float my_sqrt_reciprocal(float number);
void length_limit(float *in1,float *in2,float limit,float out[2]);
float my_deadzone(float x,float ref,float zoom);
void XYZ_Velocity_Set(float dT_ms, RC_TYPE* rc_control_s, OF* of_s, _fly_ct_st* program_ctrl_s);
#endif
