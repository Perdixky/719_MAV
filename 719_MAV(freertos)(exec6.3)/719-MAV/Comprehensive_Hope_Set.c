/******************************************************************************************
模块名：速度分解限幅模块
输入量：RC通道量
输出量：x, y轴速度
作者：	719-程子峻
*******************************************************************************************/
#include "Comprehensive_Hope_Set.h"
#include "cmsis_os.h"
/******************************************************************************************
void XYZ_Velocity_Set(float dT_ms)
作用：将对应通道量映射到对应速度量
概述：rc_control_s.THROTTLE通道归一化后，乘以MAX_Z轴速度，得到对应Z轴速度
			将rc_control_s.YAW通道归一化后，乘以MAX_偏航角速度，得到对应偏航角速度
			将rc_control_s.ROLL和rc_control_s.PITCH通道归一化，后进行飞控系统XY速度目标量综合设定，画个图容易理解
来源：匿名飞控
*******************************************************************************************/
static _flight_state_st fs_s;
static _pc_user_st pc_user_s;
static _fc_sta_var_st fc_stv;
static int FC_Mode;

//static _fc_sta_var_st fc_stv;			
//static _flight_state_st fs_s;

extern osEventFlagsId_t ClassisEventHandle;

void XYZ_Velocity_Set(float dT_ms, RC_TYPE* rc_control_s, OF* of_s, _fly_ct_st* program_ctrl_s)
{
//	int thr_deadzone = 0;
//	_fc_sta_var_st fc_stv;			
//	printf("%d, %d, %d, %d\r\n", rc_control_s->THROTTLE, rc_control_s->PITCH,  rc_control_s->ROLL, rc_control_s->BUTTON1);
	
//	fs_s.speed_set_h_norm[Z] = my_deadzone((rc_control_s->THROTTLE - 1500),0,thr_deadzone) *0.0023f;	//死区时间暂时无用,且归一化
//	fs_s.speed_set_h_norm_lpf[Z] += 0.5f *(fs_s.speed_set_h_norm[Z] - fs_s.speed_set_h_norm_lpf[Z]);  //低通滤波

	float speed_set_tmp[3] = {0};
	static int time;
	
	time++;
	fc_stv.vel_limit_xy = MAX_SPEED;
	fc_stv.vel_limit_z_p = MAX_Z_SPEED_UP;
	fc_stv.vel_limit_z_n = -MAX_Z_SPEED_DW;	

	//XYZ轴速度设定量，正负参考坐标参考方向
	fs_s.speed_set_h_norm[X] = (my_deadzone((rc_control_s->ROLL - 1500),0,50) *0.0025f);		//归一化，是为了保
	fs_s.speed_set_h_norm[Y] = (my_deadzone((rc_control_s->PITCH - 1500),0,50) *0.0025f);			//证其值能大于1，后对其限幅。
	fs_s.speed_set_h_norm[Z] = (my_deadzone((rc_control_s->THROTTLE - 1500),0,50) *0.0025f);			//证其值能大于1，后对其限幅。
	fs_s.speed_set_h_norm_lpf[X] += 0.5f *(fs_s.speed_set_h_norm[X] - fs_s.speed_set_h_norm_lpf[X]);  //低通滤波
	fs_s.speed_set_h_norm_lpf[Y] += 0.5f *(fs_s.speed_set_h_norm[Y] - fs_s.speed_set_h_norm_lpf[Y]);  //低通滤波
	fs_s.speed_set_h_norm_lpf[Z] += 0.5f *(fs_s.speed_set_h_norm[Z] - fs_s.speed_set_h_norm_lpf[Z]);  //低通滤波
	fs_s.speed_set_h_norm_lpf[X] = LIMIT(fs_s.speed_set_h_norm_lpf[X],-1,1);												//限幅，将值限幅在[-1,1]内
	fs_s.speed_set_h_norm_lpf[Y] = LIMIT(fs_s.speed_set_h_norm_lpf[Y],-1,1);
	fs_s.speed_set_h_norm_lpf[Z] = LIMIT(fs_s.speed_set_h_norm_lpf[Z],-1,1);
	
	//飞控系统XY速度目标量综合设定（在XY平面内的速度分解成X轴方向上的速度和Y轴方向上的速度）
	if((rc_control_s->BUTTON1 > 1800 && rc_control_s->BUTTON1 <2200) &&				//实时帧模式
		 (rc_control_s->THROTTLE > 1470 && rc_control_s->THROTTLE <1530) &&
		 (rc_control_s->YAW > 1485 && rc_control_s->YAW <1515) &&
		 (rc_control_s->PITCH > 1485 && rc_control_s->PITCH <1515) &&
		 (rc_control_s->ROLL > 1485 && rc_control_s->ROLL <1515) 
		)
	{
		if (time > 1000)
		{
			FC_Mode = 1;		//实时帧模式
			osEventFlagsSet(ClassisEventHandle, 1<<realframe_f);
			osEventFlagsClear(ClassisEventHandle, 1<<fix_f);
			osEventFlagsClear(ClassisEventHandle, 1<<rc_mode_f);
		}
//		printf("hello, FC_Mode, time:%d, %d",FC_Mode, time);
	}
	else if(rc_control_s->BUTTON1 > 1300 && rc_control_s->BUTTON1 <1700)
	{
		FC_Mode = 2;			//定点模式
		time = 0;
		osEventFlagsClear(ClassisEventHandle, 1<<realframe_f);
		osEventFlagsSet(ClassisEventHandle, 1<<fix_f);
		osEventFlagsClear(ClassisEventHandle, 1<<rc_mode_f);
	}
	else if(rc_control_s->BUTTON1 > 800 && rc_control_s->BUTTON1 <1200)
	{
		FC_Mode = 3;		//手动控制模式
		time = 0;
		osEventFlagsClear(ClassisEventHandle, 1<<realframe_f);
		osEventFlagsClear(ClassisEventHandle, 1<<fix_f);
		osEventFlagsSet(ClassisEventHandle, 1<<rc_mode_f);
	}
	else
	{
		time = 0;
	}
	
	switch (FC_Mode)
	{
		case 0:{}break;
		case 1:		//实时帧模式
		{
			speed_set_tmp[X] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[X] + program_ctrl_s->vel_cmps_h[X] + pc_user_s.vel_cmps_set_h[X], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Y] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[Y] + program_ctrl_s->vel_cmps_h[Y] + pc_user_s.vel_cmps_set_h[Y], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Z] = LIMIT(fc_stv.vel_limit_z_p *fs_s.speed_set_h_norm_lpf[Z] + program_ctrl_s->vel_cmps_h[Z] + pc_user_s.vel_cmps_set_h[Z], fc_stv.vel_limit_z_n, fc_stv.vel_limit_z_p);			
		}break;
		case 2:		//定点模式
		{
			speed_set_tmp[X] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[X], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Y] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[Y], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Z] = LIMIT(fc_stv.vel_limit_z_p *fs_s.speed_set_h_norm_lpf[Z], fc_stv.vel_limit_z_n, fc_stv.vel_limit_z_p);			
		}break;
		case 3:		//手动控制模式
		{
			speed_set_tmp[X] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[X], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Y] = LIMIT(fc_stv.vel_limit_xy *fs_s.speed_set_h_norm_lpf[Y], -fc_stv.vel_limit_xy, fc_stv.vel_limit_xy);
			speed_set_tmp[Z] = LIMIT(fc_stv.vel_limit_z_p *fs_s.speed_set_h_norm_lpf[Z], fc_stv.vel_limit_z_n, fc_stv.vel_limit_z_p);			
		}break;
	}
		
	
//	摇杆量转换为YAW期望角速度 + 程控期望角速度
//	speed_set_tmp[Z] = (float)(0.0023f *my_deadzone((rc_control_s->YAW - 1500),0,65) *MAX_SPEED_YAW);
	length_limit(&speed_set_tmp[X],&speed_set_tmp[Y],fc_stv.vel_limit_xy,fs_s.speed_set_h_cms);		//模长限制，最长为500cms
	if(speed_set_tmp[Z] > fc_stv.vel_limit_z_p) {speed_set_tmp[Z] = fc_stv.vel_limit_z_p;}
	else if (speed_set_tmp[Z] < fc_stv.vel_limit_z_n) {speed_set_tmp[Z] = fc_stv.vel_limit_z_n;} //控制其速度在350至-250内
	
	fs_s.speed_set_h[X] = fs_s.speed_set_h_cms[X];
	fs_s.speed_set_h[Y] = fs_s.speed_set_h_cms[Y];
	fs_s.speed_set_h[Z] = speed_set_tmp[Z];
	fs_s.distance_set_h[X] = program_ctrl_s->distance_cmps_h[X];
	fs_s.distance_set_h[Y] = program_ctrl_s->distance_cmps_h[Y];
	if(FC_Mode == 2)
		fs_s.distance_set_h[Z] = of_s->Distance;
	else 
		fs_s.distance_set_h[Z] = program_ctrl_s->distance_cmps_h[Z];
	
//	printf("fs_s.speed_set_h:%f\r\n", fs_s.speed_set_h[X]);
//	printf("fs_s.speed_set_h[Y]:%f\r\n", fs_s.speed_set_h_cms[Y]);
	
	Classis_Interface_out("d", &fs_s);

}

float my_sqrt_reciprocal(float number)
{
	long i;
	float x, y;

	x = number * 0.5F;
	y = number;
	i = * ( long * ) &y;
	i = 0x5f3759df - ( i >> 1 );

	y = * ( float * ) &i;
	y = y * ( 1.5f - ( x * y * y ) );
	y = y * ( 1.5f - ( x * y * y ) );
	
	return y;
}

//快速平方根算法
float my_sqrt(float number)
{
	return number * my_sqrt_reciprocal(number);
}

void length_limit(float *in1,float *in2,float limit,float out[2])
{
	float l = my_2_norm(*in1,*in2);
	float l_lim = LIMIT(l,0,limit);
	
	if(l==0)
	{
		out[0] = out[1] = 0;
	}
	else
	{
		out[0] = l_lim/l *(*in1);
		out[1] = l_lim/l *(*in2);
	}
}

float my_deadzone(float x,float ref,float zoom)
{
	float t;
	if(x>ref)
	{
		t = x - zoom;
		if(t<ref)
		{
			t = ref;
		}
	}
	else
	{
		t = x + zoom;
		if(t>ref)
		{
			t = ref;
		}
	}
  return (t);
}


