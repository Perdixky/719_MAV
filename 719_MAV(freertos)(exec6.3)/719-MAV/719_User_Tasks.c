/******************************************************************************************
模块名：用户任务编写模块
输入量：RC遥控器通道量
输出量：program_ctrl_s
说明	：编写飞行任务
本飞行任务20ms执行一次，称之为一个时间颗粒
program_ctrl_s.vel_cmps_h[Y]：机头方向速度
program_ctrl_s.vel_cmps_h[X]
program_ctrl_s.vel_cmps_h[Z]：竖直方向速度
作者：719-程子峻
*******************************************************************************************/
#include "719_User_Tasks.h"
#include "usart.h"
#include "stm32f103xe.h"
#include <stdio.h>
#include "719_Interface.h"
#include "cmsis_os.h"

//static RC_TYPE  rc_control;
static OF of_s;
static _fly_ct_st program_ctrl;
static uint64_t count_20ms;			//时间颗粒

extern osEventFlagsId_t ClassisEventHandle;
//=调机时的测试函数
void Vertical_Move(float speed, float distance)
{
	Classis_Interface_in(NULL);
	
	program_ctrl.vel_cmps_h[Z] = speed;
	program_ctrl.distance_cmps_h[Z] = distance;
	
	Classis_Interface_out("e", &program_ctrl);	
}

void Horizontal_Move(float speed_x, float speed_y)
{
	Classis_Interface_in(NULL);

	program_ctrl.vel_cmps_h[X] = speed_x;	
	program_ctrl.vel_cmps_h[Y] = speed_y;	
	
	Classis_Interface_out("e", &program_ctrl);	
}
//=调机时的测试函数


//=飞行任务代码
void User_Tasks(void)
{
	int retval;
	int flag_s[GroupOfFlag] = {0};
	static int mission_step;

	retval = osEventFlagsWait(ClassisEventHandle, 1<<unlock_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[unlock_f] = (retval >> unlock_f) & 0x01;

	retval = osEventFlagsWait(ClassisEventHandle, 1<<imu_init_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[imu_init_f] = (retval >> imu_init_f) & 0x01;

	retval = osEventFlagsWait(ClassisEventHandle, 1<<rc_mode_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[rc_mode_f] = (retval >> rc_mode_f) & 0x01;

	retval = osEventFlagsWait(ClassisEventHandle, 1<<fix_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[fix_f] = (retval >> fix_f) & 0x01;

	retval = osEventFlagsWait(ClassisEventHandle, 1<<realframe_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[realframe_f] = (retval >> realframe_f) & 0x01;
	
//	printf("%d, %d, %d, %d, %d\r\n", flag_s[unlock_f], flag_s[imu_init_f], flag_s[rc_mode_f], flag_s[fix_f], flag_s[realframe_f]);
	Classis_Interface_in("g", &of_s);

	if(flag_s[realframe_f] == 1)
	{
		switch(mission_step)
		{
			
			case 0:
			{
				Vertical_Move(40, 1500);
				if(of_s.Distance > 1300 && of_s.Distance < 1700)
					mission_step = 1;
			}
			case 1:
				if(count_20ms++ < 100)
				{
					program_ctrl.vel_cmps_h[Y] = 30;			//单位cm/s
					program_ctrl.vel_cmps_h[X] = 30;
				}
				else
				{
					program_ctrl.vel_cmps_h[Y] = 0;			//单位cm/s
					program_ctrl.vel_cmps_h[X] = 0;
					count_20ms = 0;
					mission_step = 2;
				}
			case 2:
			{
				Vertical_Move(40, 0);
				if(of_s.Distance < 200)
					mission_step = 3;				
			}
		}
	}
	else
	{
		Vertical_Move(40, 0);
	}
	Classis_Interface_out("e", &program_ctrl);
}
