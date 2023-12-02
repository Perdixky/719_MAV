/******************************************************************************************
模块名：控制模块
输入量：fs_s，rc_control，unlock_f，imu_init_f标志位，FLOAT_ANGLE，FLOAT_XYZ, of(光流)
输出量：Moto_PWM
作者：	719-程子峻
*******************************************************************************************/
#include "719_FC.h"
#include "stdio.h"
#include "main.h"
#include "cmsis_os.h"
/*****************************************************************************************
角度环PID:
PID[ROL][ANGLE]
PID[PIT][ANGLE]
PID[YAW][ANGLE]
角速度环PID:
PID[ROL][RATE]
PID[PIT][RATE]
PID[YAW][RATE]
******************************************************************************************/
static PID_TYPE PID[3][2];
static PID_TYPE LOC_PID[2];		//位置速度环的X,Y的PID参数
static PID_TYPE ALT_PID[2];		//[2-1]外环，[1-1]内环

//static float Moto_PWM_1 = 0.0f, Moto_PWM_2 = 0.0f, Moto_PWM_3 = 0.0f, Moto_PWM_4 = 0.0f;
static RC_TYPE  rc_control;
static FLOAT_XYZ facc_s;
static FLOAT_XYZ gyro_s;
static FLOAT_ANGLE oular_angle_s;
static _flight_state_st fs_s;
static _fly_ct_st program_ctrl_s;
static OF of_s;
static _loc_ctrl_st loc_ctrl_2, loc_ctrl_1;

extern osEventFlagsId_t ClassisEventHandle;
/******************************************************************************************
*备  注：机头与电机示意图	
					 机头(Y+)
					   
				  M1    ↑    M2
					\   |   /
					 \  |  /
					  \ | /
			    ————————+————————>X+	
					  / | \
					 /  |  \
					/   |   \
				  M4    |    M3

	1. M1 M3电机逆时针旋转，M2 M4电机顺时针旋转
	2. X:是MPU6050的 X 轴，Y:是MPU6050的 Y 轴，Z轴正方向垂直 X-Y 面，竖直向上
	3. 绕 X 轴旋转为PITCH 角 俯仰
	   绕 Y 轴旋转为 ROLL 角 横滚
	   绕 Z 轴旋转为 YAW  角 偏航
*******************************************************************************************/

/*****************************************************************************
* 函  数：void PidParameter_init(void)
* 功  能：初始化PID结构体里的一些成员值
* 参  数：无
* 返回值：无 
* 备  注: 不使用flash存储PID参数 运行时所有参数全部进行初始化,默认参数参照ANO脱空者
*****************************************************************************/
void PidParameter_init(void)
{
	static int flag;
	flag = usb_rec_flag;
	if(flag == 0)
	{
			//ROLL轴
			//角度环
			PID[ROL][ANGLE].P = 7.0f;		
			PID[ROL][ANGLE].I = 0.0f;			
			PID[ROL][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = 0.00f;		//角度环采用普通PID，fb_k1=fb_k2时，相当于普通PID，即为原来的KD项				
			
			PID[ROL][ANGLE].Ilimit_flag = 0;
			PID[ROL][ANGLE].Ilimit = 35;    
			PID[ROL][ANGLE].Irang = 1000;    
	
			//角速度环
			PID[ROL][RATE].P = 1.0f;
			PID[ROL][RATE].I = 0.01f;
			//	PID[ROL][RATE].fb_k1 = 0.2f;
			//	PID[ROL][RATE].fb_k2 = 0.0;				//微分先行系数慎重修改，由于陀螺仪变化率大，略微增添一些，会导致微分项极大								
			PID[ROL][RATE].fb_k1 = PID[ROL][RATE].fb_k2 = 0.002f;	//ANO
	
			PID[ROL][RATE].Ilimit_flag = 0; 
			PID[ROL][RATE].Ilimit = 500;    
			PID[ROL][RATE].Irang = 1000;    
	

			//PITCH轴
			//角度环
			PID[PIT][ANGLE].P = 7.0f;
			PID[PIT][ANGLE].I = 0.0f;
			PID[PIT][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = 0.00f;
			
			PID[PIT][ANGLE].Ilimit_flag = 0;
			PID[PIT][ANGLE].Ilimit = 35;    
			PID[PIT][ANGLE].Irang = 1000;   
			
			//角速度环
			PID[PIT][RATE].P = 1.0f;
			PID[PIT][RATE].I =0.01f;
		//	PID[PIT][RATE].fb_k1 = 0.2f;
		//	PID[PIT][RATE].fb_k2 = 0.0;		//微分先行系数慎重修改，由于陀螺仪变化率大，略微增添一些，会导致微分项极大			
			PID[PIT][RATE].fb_k1 = PID[PIT][RATE].fb_k2 = 0.002f;	//ANO
			
			PID[PIT][RATE].Ilimit_flag = 0; 
			PID[PIT][RATE].Ilimit = 500;    
			PID[PIT][RATE].Irang = 1000;   
			
			//YAW轴
			//角度环
			PID[YAW][ANGLE].P = 0.0f;				//参照匿名上的飞控yaw轴PID参数设置
			PID[YAW][ANGLE].I = 0.0f; 
			PID[YAW][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k1 = 0.0f;

			PID[YAW][ANGLE].Ilimit_flag = 0;
			PID[YAW][ANGLE].Ilimit = 0;    
			PID[YAW][ANGLE].Irang = 0;  
			
			//角速度环
			PID[YAW][RATE].P = 0.5f;
			PID[YAW][RATE].I = 0;
			PID[YAW][RATE].fb_k1 = 0;
			PID[YAW][RATE].fb_k2 = 0;
			
			PID[YAW][RATE].Ilimit_flag = 0; 
			PID[YAW][RATE].Ilimit = 200;    
			PID[YAW][RATE].Irang = 1200;   
	
	
			//位置速度环
			//X轴速度
			LOC_PID[X].P = 0.22f;						//参照匿名初始值，需要调
			LOC_PID[X].I = 0.0f; 						//无需调
			LOC_PID[X].fb_k1 = 0.00f;				//无需调
			LOC_PID[X].fb_k2 = 0.00f;				//参照匿名初始值，需要调
			
			LOC_PID[X].Ilimit_flag = 0; 		
			LOC_PID[X].Ilimit = 50;    		
			LOC_PID[X].Irang = 10;    		
			
			//Y轴速度
			LOC_PID[Y].P = 0.22f;						//参照匿名初始值，需要调
			LOC_PID[Y].I = 0.0f; 						//无需调
			LOC_PID[Y].fb_k1 = 0.00f;				//无需调
			LOC_PID[Y].fb_k2 = 0.00f;				//参照匿名初始值，需要调
			
			LOC_PID[Y].Ilimit_flag = 0; 
			LOC_PID[Y].Ilimit = 50;    
			LOC_PID[Y].Irang = 10;
			
			//高度环
			//外环
			ALT_PID[2-1].P = 1.0f;						//参照匿名初始值，需要调
			ALT_PID[2-1].I = 0.0f; 						//无需调
			ALT_PID[2-1].feedforward_k = 0.0f;
			ALT_PID[2-1].fb_k1 = 0.00f;				//无需调
			ALT_PID[2-1].fb_k2 = 0.00f;				//参照匿名初始值，需要调
			
			ALT_PID[2-1].Ilimit_flag = 0;
			ALT_PID[2-1].Ilimit = 100;    
			ALT_PID[2-1].Irang = 0;   
			
			//高度速度环
			//内环
			ALT_PID[1-1].P = 1.0f;							//参照匿名初始值，需要调
			ALT_PID[1-1].I = 0.0f; 							//参考1.0f
			ALT_PID[1-1].feedforward_k = 0.0f;	//参考0.05f
			ALT_PID[1-1].fb_k1 = 0.00f;					//无需调
			ALT_PID[1-1].fb_k2 = 0.0f;					//前馈值
			
			ALT_PID[1-1].Ilimit_flag = 0; 
			ALT_PID[1-1].Ilimit = 50;    
			ALT_PID[1-1].Irang = 30; 
	}
	else
	{
			//ROLL轴
			//角度环
			PID[ROL][ANGLE].P = usb_PID.angleRing_roll.p;		
			PID[ROL][ANGLE].I = usb_PID.angleRing_roll.i;			
			PID[ROL][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = usb_PID.angleRing_roll.d;		//角度环采用普通PID，fb_k1=fb_k2时，相当于普通PID，即为原来的KD项				
			
			PID[ROL][ANGLE].Ilimit_flag = 0;
			PID[ROL][ANGLE].Ilimit = 35;    
			PID[ROL][ANGLE].Irang = usb_PID.angleRing_roll.limit;    
	
			//角速度环
			PID[ROL][RATE].P = usb_PID.angularVelocity_roll.p;
			PID[ROL][RATE].I = usb_PID.angularVelocity_roll.i;
			//	PID[ROL][RATE].fb_k1 = 0.2f;
			//	PID[ROL][RATE].fb_k2 = 0.0;				//微分先行系数慎重修改，由于陀螺仪变化率大，略微增添一些，会导致微分项极大								
			PID[ROL][RATE].fb_k1 = PID[ROL][RATE].fb_k2 = usb_PID.angularVelocity_roll.d;	//ANO
	
			PID[ROL][RATE].Ilimit_flag = 0; 
			PID[ROL][RATE].Ilimit = 500;    
			PID[ROL][RATE].Irang = usb_PID.angularVelocity_roll.limit;    
	

			//PITCH轴
			//角度环
			PID[PIT][ANGLE].P = usb_PID.angleRing_pitch.p;
			PID[PIT][ANGLE].I = usb_PID.angleRing_pitch.i;
			PID[PIT][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = usb_PID.angleRing_pitch.d;
			
			PID[PIT][ANGLE].Ilimit_flag = 0;
			PID[PIT][ANGLE].Ilimit = 35;    
			PID[PIT][ANGLE].Irang = usb_PID.angleRing_pitch.limit;   
			
			//角速度环
			PID[PIT][RATE].P = usb_PID.angularVelocity_pitch.p;
			PID[PIT][RATE].I =usb_PID.angularVelocity_pitch.i;
		//	PID[PIT][RATE].fb_k1 = 0.2f;
		//	PID[PIT][RATE].fb_k2 = 0.0;		//微分先行系数慎重修改，由于陀螺仪变化率大，略微增添一些，会导致微分项极大			
			PID[PIT][RATE].fb_k1 = PID[PIT][RATE].fb_k2 = usb_PID.angularVelocity_pitch.d;	//ANO
			
			PID[PIT][RATE].Ilimit_flag = 0; 
			PID[PIT][RATE].Ilimit = 500;    
			PID[PIT][RATE].Irang = usb_PID.angularVelocity_pitch.limit;   
			
			//YAW轴
			//角度环
			PID[YAW][ANGLE].P = 5.0f;				//参照匿名上的飞控yaw轴PID参数设置
			PID[YAW][ANGLE].I = 0.0f; 
			PID[YAW][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k1 = 0.5f;

			PID[YAW][ANGLE].Ilimit_flag = 0;
			PID[YAW][ANGLE].Ilimit = 35;    
			PID[YAW][ANGLE].Irang = 200;  
			
			//角速度环
			PID[YAW][RATE].P = usb_PID.angularVelocity_yaw.p;
			PID[YAW][RATE].I = usb_PID.angularVelocity_yaw.i;
//			PID[YAW][RATE].fb_k1 = 0;
//			PID[YAW][RATE].fb_k2 = 0;
			PID[YAW][RATE].fb_k1 = PID[YAW][RATE].fb_k2 = usb_PID.angularVelocity_yaw.d;
			
			PID[YAW][RATE].Ilimit_flag = 0; 
			PID[YAW][RATE].Ilimit = 200;    
			PID[YAW][RATE].Irang = usb_PID.angularVelocity_yaw.limit;   
	
	
			//位置速度环
			//X轴速度
			LOC_PID[X].P = usb_PID.positionVelocity_X.p;						//参照匿名初始值，需要调
			LOC_PID[X].I = usb_PID.positionVelocity_X.i; 					//无需调
			LOC_PID[X].fb_k1 = 0.00f;																//无需调
			LOC_PID[X].fb_k2 = usb_PID.positionVelocity_X.d;				//参照匿名初始值，需要调
			
			LOC_PID[X].Ilimit_flag = 0; 		
			LOC_PID[X].Ilimit = 50;    		
			LOC_PID[X].Irang = usb_PID.positionVelocity_X.limit;    		
			
			//Y轴速度
			LOC_PID[Y].P = usb_PID.positionVelocity_Y.p;						//参照匿名初始值，需要调
			LOC_PID[Y].I = usb_PID.positionVelocity_Y.i; 					//无需调
			LOC_PID[Y].fb_k1 = 0.00f;																//无需调
			LOC_PID[Y].fb_k2 = usb_PID.positionVelocity_Y.d;				//参照匿名初始值，需要调
			
			LOC_PID[Y].Ilimit_flag = 0; 
			LOC_PID[Y].Ilimit = 50;    
			LOC_PID[Y].Irang = usb_PID.positionVelocity_Y.limit;
			
			//高度环
			//外环
			ALT_PID[2-1].P = usb_PID.heightRing.p;						//参照匿名初始值，需要调
			ALT_PID[2-1].I = usb_PID.heightRing.i; 					//无需调
			ALT_PID[2-1].feedforward_k = 0.0f;
			ALT_PID[2-1].fb_k1 = 0.00f;												//无需调
			ALT_PID[2-1].fb_k2 = usb_PID.heightRing.d;				//参照匿名初始值，需要调
			
			ALT_PID[2-1].Ilimit_flag = 0;
			ALT_PID[2-1].Ilimit = 100;    
			ALT_PID[2-1].Irang = usb_PID.heightRing.limit;   
			
			//高度速度环
			//内环
			ALT_PID[1-1].P = usb_PID.heightVelocity.p;							//参照匿名初始值，需要调
			ALT_PID[1-1].I = usb_PID.heightVelocity.i; 						//参考1.0f
			ALT_PID[1-1].feedforward_k = 0.0f;											//参考0.05f
			ALT_PID[1-1].fb_k1 = 0.00f;															//无需调
			ALT_PID[1-1].fb_k2 = usb_PID.heightVelocity.d;					//前馈值
			
			ALT_PID[1-1].Ilimit_flag = 0; 
			ALT_PID[1-1].Ilimit = 50;    
			ALT_PID[1-1].Irang = usb_PID.heightVelocity.limit; 
	}

}

/*****************************************************************************
* 函  数：void IsUnlock(void)
* 功  能：判断是否需要积分分离
* 参  数：无
* 返回值：Integral_Flag_s 
* 备  注: 
*****************************************************************************/
int IsUnlock(void)
{
	if( rc_control.BUTTON2 > 1500 && rc_control.BUTTON2 < 2500 && rc_control.THROTTLE >= 180 )  //飞机解锁之后再加入积分,防止积分过调
	{
		return 0;
		
	}else
	{
		return 1;
	}
}

/******************************************************************************************
函数名：void LocControl(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
作用：位置速度环控制
概述：因为无GPS，UMB，光流进行修正，故直接用期望速度转为角度
来源：匿名飞控
*******************************************************************************************/
void LocControl(float dT_s, int integral_flag_s)
{
	static float vel_fb_d_lpf[2]; 			//经低通滤波后的反馈，将用于微分先行
	
	if(1)		//光流是否开启的标志位
	{
		loc_ctrl_1.exp[X] = fs_s.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs_s.speed_set_h[Y];

		//一阶惯性（低通）滤波
		LPF_1_(5.0f,dT_s,facc_s.X,vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_s,facc_s.Y,vel_fb_d_lpf[Y]);	
	
		loc_ctrl_1.fb[X] = of_s.RealSpeed_x;		//光流
		loc_ctrl_1.fb[Y] = of_s.RealSpeed_y;		
		
//		loc_ctrl_1.fb[X] = of_s.RealSpeed_x + 0.03f *vel_fb_d_lpf[X];		//0.03微分先行系数
//		loc_ctrl_1.fb[Y] = of_s.RealSpeed_y + 0.03f *vel_fb_d_lpf[Y];		//当前速度反馈=上一次比例和积分不变项（of_s.RealSpeed_y）+微分项（0.03f *vel_fb_d_lpf[Y]）

		for(int i =0;i<2;i++)
		{
			PID_Postion_Cal(dT_s,												//周期（单位：秒）
											&LOC_PID[i],					
											loc_ctrl_1.exp[i],					//期望值（设定值）
											loc_ctrl_1.fb[i], 					//反馈值（测量值）
											0,													//前馈值
											integral_flag_s
											);

			loc_ctrl_1.out[i] = LOC_PID[i].OutPut + 0.03f *vel_fb_d_lpf[i];	//0.03微分先行系数
//			loc_ctrl_1.out[i] = LOC_PID[i].OutPut;
		}
//		printf("loc_ctrl_1.out,X,Y:%f, %f\r\n", loc_ctrl_1.out[X], loc_ctrl_1.out[Y]);
	}
	else if(0)
	{
		//姿态模式，直接用期望速度转为角度（期望角度）
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs_s.speed_set_h[X];
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs_s.speed_set_h[Y];
//		printf("loc_ctrl_1.out[X]:%f\r\n", fs_s.speed_set_h[X]);
//		printf("loc_ctrl_1.out[Y]:%f\r\n", fs_s.speed_set_h[Y]);
	}
}

/******************************************************************************************
函数名：void AttControl_2(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
作用：角度环控制
概述：先通过积分微调进一步精确期望姿态，后对yaw进行误差限幅，以保证系统稳定，最终进行PID计算，得到角速度
来源：匿名飞控
*******************************************************************************************/
void AttControl_2(float dT_s, int integral_flag_s)
{
  _att_2l_ct_st att_2l_ct;
  float exp_rol_tmp, exp_pit_tmp;				//单位：角度
	
	static uint8_t RC_middle = 1, RC_middle_num;
	static float RC_ROLL_mid = 0, RC_PITCH_mid = 0;
	
	if(RC_middle == 1)
	{
	    for(RC_middle_num = 0; RC_middle_num < 10; RC_middle_num++)
		{
			RC_ROLL_mid  += (float)rc_control.ROLL;
			RC_PITCH_mid += (float)rc_control.PITCH;
		}
		RC_ROLL_mid /= 10.0;
		RC_PITCH_mid /= 10.0;
		RC_middle =0;
	}

	//积分微调
	//位置速度环输出需要保持的角度
	exp_rol_tmp = -loc_ctrl_1.out[X];						
	exp_pit_tmp = -loc_ctrl_1.out[Y];

//printf("%f, %f\r\n", loc_ctrl_1.out[Y], loc_ctrl_1.out[X]);
	
	if(ABS(exp_rol_tmp + att_2l_ct.exp_rol_adj) < 5)
	{
		att_2l_ct.exp_rol_adj += 0.2f *exp_rol_tmp *dT_s;						//调整量的累加
		att_2l_ct.exp_rol_adj = LIMIT(att_2l_ct.exp_rol_adj,-1,1);	//将调整量限幅在【-1，1】内，比较小，称之为微调
	}																															//微调可能为了让无人机更好的维持在某个期望角度，
																																//毕竟实际情景中存在抵抗力
	if(ABS(exp_pit_tmp + att_2l_ct.exp_pit_adj) < 5)
	{
		att_2l_ct.exp_pit_adj += 0.2f *exp_pit_tmp *dT_s;
		att_2l_ct.exp_pit_adj = LIMIT(att_2l_ct.exp_pit_adj,-1,1);
	}
	att_2l_ct.exp_rol = exp_rol_tmp + att_2l_ct.exp_rol_adj;	
	att_2l_ct.exp_pit = exp_pit_tmp + att_2l_ct.exp_pit_adj;

	att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol,-MAX_ANGLE,MAX_ANGLE);			//期望角度限幅
	att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit,-MAX_ANGLE,MAX_ANGLE);

	att_2l_ct.fb_rol = -oular_angle_s.rol;																	//为了保证调参系数都是正的
	att_2l_ct.fb_pit = oular_angle_s.pit;
//	printf("%f, %f\r\n", oular_angle_s.pit, oular_angle_s.rol);
	
	PID_Postion_Cal(dT_s,												//周期（单位：秒）
	  							&PID[ROL][ANGLE],					
								  att_2l_ct.exp_rol,					//期望值（设定值）
									att_2l_ct.fb_rol, 					//反馈值（测量值）
									0,      										//前馈值
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//周期（单位：秒）
  								&PID[PIT][ANGLE],					
  								att_2l_ct.exp_pit,					//期望值（设定值）
	  							att_2l_ct.fb_pit, 					//反馈值（测量值）
	  							0,														//前馈值
									integral_flag_s
								 );

								 
//	printf("PID[ANGLE].OutPut,P,R,Y: %f, %f, %f\r\n", PID[PIT][ANGLE].OutPut, PID[ROL][ANGLE].OutPut, PID[YAW][ANGLE].OutPut);

}

/******************************************************************************************
函数名：void AttControl_1(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
作用：角速度环控制
概述：
来源：匿名飞控
*******************************************************************************************/
//static float max_yaw_speed,set_yaw_av_tmp;	//单位（分别为）：速度，角速度
void AttControl_1(float dT_s, int integral_flag_s)
{
	_att_1l_ct_st att_1l_ct;
	static uint8_t RC_middle = 1, RC_middle_num;
	static float RC_YAW_mid = 0;
	
	if(RC_middle == 1)
	{
	    for(RC_middle_num = 0; RC_middle_num < 10; RC_middle_num++)
		{
			RC_YAW_mid   += (float)rc_control.YAW;
		}
		RC_YAW_mid /= 10.0;
		RC_middle =0;
	}
	//目标角速度赋值
	att_1l_ct.exp_angular_velocity[ROL] = PID[ROL][ANGLE].OutPut;
	att_1l_ct.exp_angular_velocity[PIT] = PID[PIT][ANGLE].OutPut;
	att_1l_ct.exp_angular_velocity[YAW] = (float)((RC_YAW_mid - rc_control.YAW)/10.0f) * PID[YAW][RATE].P;
	
	//目标角速度限幅
	att_1l_ct.exp_angular_velocity[ROL] = LIMIT(att_1l_ct.exp_angular_velocity[ROL],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
	att_1l_ct.exp_angular_velocity[PIT] = LIMIT(att_1l_ct.exp_angular_velocity[PIT],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);

	att_1l_ct.fb_angular_velocity[ROL] = gyro_s.Y;
	att_1l_ct.fb_angular_velocity[PIT] = -(gyro_s.X);
	att_1l_ct.fb_angular_velocity[YAW] = gyro_s.Z;
//	printf("%f, %f, %f\r\n", -(gyro_s.X), gyro_s.Y, gyro_s.Z);
	
	PID_Postion_Cal(dT_s,												//周期（单位：秒）
	  							&PID[ROL][RATE],					
								  att_1l_ct.exp_angular_velocity[ROL],					//期望值（设定值）
									att_1l_ct.fb_angular_velocity[ROL], 					//反馈值（测量值）
									0,														//前馈值
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//周期（单位：秒）
  								&PID[PIT][RATE],					
  								att_1l_ct.exp_angular_velocity[PIT],					//期望值（设定值）
	  							att_1l_ct.fb_angular_velocity[PIT], 					//反馈值（测量值）
	  							0,														//前馈值
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//周期（单位：秒）
  								&PID[YAW][RATE],					
  								att_1l_ct.exp_angular_velocity[YAW],					//期望值（设定值）
  								att_1l_ct.fb_angular_velocity[YAW], 					//反馈值（测量值）
									0,													  //前馈值
									integral_flag_s
								 );
		
//	printf("PID[RATE].OutPut,P,R,Y: %f, %f, %f\r\n", PID[PIT][RATE].OutPut, PID[ROL][RATE].OutPut, PID[YAW][RATE].OutPut);		
}


void AltControl_2(float dT_s, float target_distance, int integral_flag_s)										//target_distance单位mm
{
	loc_ctrl_2.exp[Z] += (fs_s.speed_set_h[Z] * dT_s) * 10;				//单位mm
	loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],loc_ctrl_2.fb[Z]-200,loc_ctrl_2.fb[Z]+200);		//限幅，每一时间段内升降不能超过200mm距离
	if(target_distance > -0.0001f)							//只需为非负数即可
		loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],0, target_distance);
	loc_ctrl_2.fb[Z] = of_s.Distance;
	
  PID_Postion_Cal(dT_s,												//周期（单位：秒）
  								&ALT_PID[2-1],					
  								loc_ctrl_2.exp[Z],					//期望值（设定值）
  								loc_ctrl_2.fb[Z], 					//反馈值（测量值）
									0,													//前馈值
									integral_flag_s
								 );
//	printf("loc_ctrl_2.exp[Z]: %f\r\n", loc_ctrl_2.exp[Z]);
//	printf("ALT_PID[2-1]: %f\r\n", ALT_PID[2-1].OutPut);
}

void AltControl_1(float dT_s, float target_distance, int integral_flag_s)
{
//	static float Target_Alt_Rate, Fb_Alt_Rate;
	static float Z_Acc_Lpf;
	static float fb_k2;
	
//	printf("Z_Acc:%f\r\n", Z_Acc);
	if((of_s.Distance<target_distance-200) || (of_s.Distance>target_distance+200))
		loc_ctrl_1.exp[Z] = 0.6f * fs_s.speed_set_h[Z] + ALT_PID[2-1].OutPut;						//速度补偿系数0.6f，直接给速度，为了保证有一个稳定速度飞行
	else
		loc_ctrl_1.exp[Z] = ALT_PID[2-1].OutPut;
	
	loc_ctrl_1.fb[Z] = of_s.RealSpeed_z;

  PID_Postion_Cal(dT_s,												//周期（单位：秒）
  								&ALT_PID[1-1],					
  								loc_ctrl_1.exp[Z],					//期望值（设定值）
  								loc_ctrl_1.fb[Z], 					//反馈值（测量值）
									0,													  //前馈值
									integral_flag_s
								 );
	
	Z_Acc_Lpf += 0.2f * (facc_s.Z - Z_Acc_Lpf);				//低通滤波
	ALT_PID[1-1].OutPut += fb_k2 * Z_Acc_Lpf;					//微分先行，上边PID函数微分系数为0
//	printf("ALT_PID[1-1]: %f\r\n", ALT_PID[1-1].OutPut);
}

void Motcontrol(void)
{
	static int16_t Moto_PWM_f[4];
	static int flag_s[GroupOfFlag];
	int retval;
	if(rc_control.BUTTON2 < 1500)
	{
		flag_s[unlock_f] = 0;
		retval = osEventFlagsClear(ClassisEventHandle, 1<<unlock_f);

		Moto_PWM_f[1-1] = 0;
		Moto_PWM_f[2-1] = 0;
		Moto_PWM_f[3-1] = 0;
		Moto_PWM_f[4-1] = 0;
	}
	if(rc_control.THROTTLE < 1200)//按键拨至解锁档位，油门低位，允许解锁
	{
		if(rc_control.BUTTON2 > 1500 && rc_control.BUTTON2 < 2500)
		{
			flag_s[unlock_f] = 1;
			retval = osEventFlagsSet(ClassisEventHandle, 1<<unlock_f);
		}
	}
	
	retval = osEventFlagsWait(ClassisEventHandle, 1<<imu_init_f, osFlagsNoClear, 0);
	if(retval >= 0)
		flag_s[imu_init_f] = ((retval >> imu_init_f) & 0x01);
	
//	printf("标志位解锁：%d, %d\r\n", flag_s[unlock_f], flag_s[imu_init_f]);
	if(flag_s[unlock_f] == 1 && flag_s[imu_init_f] == 1) //飞机解锁时动力分配才生效
	{
		Moto_PWM_f[1-1] = (1500+(int)fs_s.speed_set_h[Z]) + (int)ALT_PID[1-1].OutPut - (int)PID[ROL][RATE].OutPut + (int)PID[PIT][RATE].OutPut - (int)PID[YAW][RATE].OutPut;   
		Moto_PWM_f[2-1] = (1500+(int)fs_s.speed_set_h[Z]) + (int)ALT_PID[1-1].OutPut + (int)PID[ROL][RATE].OutPut + (int)PID[PIT][RATE].OutPut + (int)PID[YAW][RATE].OutPut;   
		Moto_PWM_f[3-1] = (1500+(int)fs_s.speed_set_h[Z]) + (int)ALT_PID[1-1].OutPut + (int)PID[ROL][RATE].OutPut - (int)PID[PIT][RATE].OutPut - (int)PID[YAW][RATE].OutPut;   
		Moto_PWM_f[4-1] = (1500+(int)fs_s.speed_set_h[Z]) + (int)ALT_PID[1-1].OutPut - (int)PID[ROL][RATE].OutPut -(int) PID[PIT][RATE].OutPut + (int)PID[YAW][RATE].OutPut;
	
	}
//	printf("%d, %d, %d, %d\r\n",(int)ALT_PID[1-1].OutPut, (int)PID[ROL][RATE].OutPut, (int)PID[PIT][RATE].OutPut, (int)PID[YAW][RATE].OutPut);
	
//	printf("Moto_PWM_f: %d, %d, %d, %d\r\n", Moto_PWM_f[1-1],Moto_PWM_f[2-1], Moto_PWM_f[3-1], Moto_PWM_f[4-1]);

//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, Moto_PWM_f[1-1]);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, Moto_PWM_f[2-1]);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, Moto_PWM_f[3-1]);
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Moto_PWM_f[4-1]); //将此数值分配到定时器，输出对应占空比的PWM波
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500); //将此数值分配到定时器，输出对应占空比的PWM波
}

void MotorPwm_Init(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000); //将此数值分配到定时器，输出对应占空比的PWM波
	HAL_Delay(3000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); //将此数值分配到定时器，输出对应占空比的PWM波
	HAL_Delay(3000);
	
}
void Control_Server(void)
{
	int Integral_Flag_s = 1;
	
	Classis_Interface_in("abccdeg", &rc_control, &oular_angle_s, &gyro_s, &facc_s, &fs_s, &program_ctrl_s, &of_s);
//	printf("FS->speed_set_h_cms[X]:%f\r\n", fs_s.speed_set_h[X]);
//	printf("FS->speed_set_h_cms[Y]:%f\r\n", fs_s.speed_set_h[Y]);
	
	XYZ_Velocity_Set(2e-3f, &rc_control, &of_s, &program_ctrl_s);
	Integral_Flag_s = IsUnlock();
	
	LocControl(2e-3f, Integral_Flag_s);
	
	AttControl_2(2e-3f, Integral_Flag_s); 		//周期，欧拉角，角速度，积分分离标志
	AttControl_1(2e-3f, Integral_Flag_s);
	
	AltControl_2(2e-3f, fs_s.distance_set_h[Z], Integral_Flag_s);
	AltControl_1(2e-3f, fs_s.distance_set_h[Z], Integral_Flag_s);


	Motcontrol();
	
}
