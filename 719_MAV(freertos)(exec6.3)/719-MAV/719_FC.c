/******************************************************************************************
ģ����������ģ��
��������fs_s��rc_control��unlock_f��imu_init_f��־λ��FLOAT_ANGLE��FLOAT_XYZ, of(����)
�������Moto_PWM
���ߣ�	719-���Ӿ�
*******************************************************************************************/
#include "719_FC.h"
#include "stdio.h"
#include "main.h"
#include "cmsis_os.h"
/*****************************************************************************************
�ǶȻ�PID:
PID[ROL][ANGLE]
PID[PIT][ANGLE]
PID[YAW][ANGLE]
���ٶȻ�PID:
PID[ROL][RATE]
PID[PIT][RATE]
PID[YAW][RATE]
******************************************************************************************/
static PID_TYPE PID[3][2];
static PID_TYPE LOC_PID[2];		//λ���ٶȻ���X,Y��PID����
static PID_TYPE ALT_PID[2];		//[2-1]�⻷��[1-1]�ڻ�

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
*��  ע����ͷ����ʾ��ͼ	
					 ��ͷ(Y+)
					   
				  M1    ��    M2
					\   |   /
					 \  |  /
					  \ | /
			    ����������������+����������������>X+	
					  / | \
					 /  |  \
					/   |   \
				  M4    |    M3

	1. M1 M3�����ʱ����ת��M2 M4���˳ʱ����ת
	2. X:��MPU6050�� X �ᣬY:��MPU6050�� Y �ᣬZ��������ֱ X-Y �棬��ֱ����
	3. �� X ����תΪPITCH �� ����
	   �� Y ����תΪ ROLL �� ���
	   �� Z ����תΪ YAW  �� ƫ��
*******************************************************************************************/

/*****************************************************************************
* ��  ����void PidParameter_init(void)
* ��  �ܣ���ʼ��PID�ṹ�����һЩ��Աֵ
* ��  ������
* ����ֵ���� 
* ��  ע: ��ʹ��flash�洢PID���� ����ʱ���в���ȫ�����г�ʼ��,Ĭ�ϲ�������ANO�ѿ���
*****************************************************************************/
void PidParameter_init(void)
{
	static int flag;
	flag = usb_rec_flag;
	if(flag == 0)
	{
			//ROLL��
			//�ǶȻ�
			PID[ROL][ANGLE].P = 7.0f;		
			PID[ROL][ANGLE].I = 0.0f;			
			PID[ROL][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = 0.00f;		//�ǶȻ�������ͨPID��fb_k1=fb_k2ʱ���൱����ͨPID����Ϊԭ����KD��				
			
			PID[ROL][ANGLE].Ilimit_flag = 0;
			PID[ROL][ANGLE].Ilimit = 35;    
			PID[ROL][ANGLE].Irang = 1000;    
	
			//���ٶȻ�
			PID[ROL][RATE].P = 1.0f;
			PID[ROL][RATE].I = 0.01f;
			//	PID[ROL][RATE].fb_k1 = 0.2f;
			//	PID[ROL][RATE].fb_k2 = 0.0;				//΢������ϵ�������޸ģ����������Ǳ仯�ʴ���΢����һЩ���ᵼ��΢�����								
			PID[ROL][RATE].fb_k1 = PID[ROL][RATE].fb_k2 = 0.002f;	//ANO
	
			PID[ROL][RATE].Ilimit_flag = 0; 
			PID[ROL][RATE].Ilimit = 500;    
			PID[ROL][RATE].Irang = 1000;    
	

			//PITCH��
			//�ǶȻ�
			PID[PIT][ANGLE].P = 7.0f;
			PID[PIT][ANGLE].I = 0.0f;
			PID[PIT][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = 0.00f;
			
			PID[PIT][ANGLE].Ilimit_flag = 0;
			PID[PIT][ANGLE].Ilimit = 35;    
			PID[PIT][ANGLE].Irang = 1000;   
			
			//���ٶȻ�
			PID[PIT][RATE].P = 1.0f;
			PID[PIT][RATE].I =0.01f;
		//	PID[PIT][RATE].fb_k1 = 0.2f;
		//	PID[PIT][RATE].fb_k2 = 0.0;		//΢������ϵ�������޸ģ����������Ǳ仯�ʴ���΢����һЩ���ᵼ��΢�����			
			PID[PIT][RATE].fb_k1 = PID[PIT][RATE].fb_k2 = 0.002f;	//ANO
			
			PID[PIT][RATE].Ilimit_flag = 0; 
			PID[PIT][RATE].Ilimit = 500;    
			PID[PIT][RATE].Irang = 1000;   
			
			//YAW��
			//�ǶȻ�
			PID[YAW][ANGLE].P = 0.0f;				//���������ϵķɿ�yaw��PID��������
			PID[YAW][ANGLE].I = 0.0f; 
			PID[YAW][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k1 = 0.0f;

			PID[YAW][ANGLE].Ilimit_flag = 0;
			PID[YAW][ANGLE].Ilimit = 0;    
			PID[YAW][ANGLE].Irang = 0;  
			
			//���ٶȻ�
			PID[YAW][RATE].P = 0.5f;
			PID[YAW][RATE].I = 0;
			PID[YAW][RATE].fb_k1 = 0;
			PID[YAW][RATE].fb_k2 = 0;
			
			PID[YAW][RATE].Ilimit_flag = 0; 
			PID[YAW][RATE].Ilimit = 200;    
			PID[YAW][RATE].Irang = 1200;   
	
	
			//λ���ٶȻ�
			//X���ٶ�
			LOC_PID[X].P = 0.22f;						//����������ʼֵ����Ҫ��
			LOC_PID[X].I = 0.0f; 						//�����
			LOC_PID[X].fb_k1 = 0.00f;				//�����
			LOC_PID[X].fb_k2 = 0.00f;				//����������ʼֵ����Ҫ��
			
			LOC_PID[X].Ilimit_flag = 0; 		
			LOC_PID[X].Ilimit = 50;    		
			LOC_PID[X].Irang = 10;    		
			
			//Y���ٶ�
			LOC_PID[Y].P = 0.22f;						//����������ʼֵ����Ҫ��
			LOC_PID[Y].I = 0.0f; 						//�����
			LOC_PID[Y].fb_k1 = 0.00f;				//�����
			LOC_PID[Y].fb_k2 = 0.00f;				//����������ʼֵ����Ҫ��
			
			LOC_PID[Y].Ilimit_flag = 0; 
			LOC_PID[Y].Ilimit = 50;    
			LOC_PID[Y].Irang = 10;
			
			//�߶Ȼ�
			//�⻷
			ALT_PID[2-1].P = 1.0f;						//����������ʼֵ����Ҫ��
			ALT_PID[2-1].I = 0.0f; 						//�����
			ALT_PID[2-1].feedforward_k = 0.0f;
			ALT_PID[2-1].fb_k1 = 0.00f;				//�����
			ALT_PID[2-1].fb_k2 = 0.00f;				//����������ʼֵ����Ҫ��
			
			ALT_PID[2-1].Ilimit_flag = 0;
			ALT_PID[2-1].Ilimit = 100;    
			ALT_PID[2-1].Irang = 0;   
			
			//�߶��ٶȻ�
			//�ڻ�
			ALT_PID[1-1].P = 1.0f;							//����������ʼֵ����Ҫ��
			ALT_PID[1-1].I = 0.0f; 							//�ο�1.0f
			ALT_PID[1-1].feedforward_k = 0.0f;	//�ο�0.05f
			ALT_PID[1-1].fb_k1 = 0.00f;					//�����
			ALT_PID[1-1].fb_k2 = 0.0f;					//ǰ��ֵ
			
			ALT_PID[1-1].Ilimit_flag = 0; 
			ALT_PID[1-1].Ilimit = 50;    
			ALT_PID[1-1].Irang = 30; 
	}
	else
	{
			//ROLL��
			//�ǶȻ�
			PID[ROL][ANGLE].P = usb_PID.angleRing_roll.p;		
			PID[ROL][ANGLE].I = usb_PID.angleRing_roll.i;			
			PID[ROL][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = usb_PID.angleRing_roll.d;		//�ǶȻ�������ͨPID��fb_k1=fb_k2ʱ���൱����ͨPID����Ϊԭ����KD��				
			
			PID[ROL][ANGLE].Ilimit_flag = 0;
			PID[ROL][ANGLE].Ilimit = 35;    
			PID[ROL][ANGLE].Irang = usb_PID.angleRing_roll.limit;    
	
			//���ٶȻ�
			PID[ROL][RATE].P = usb_PID.angularVelocity_roll.p;
			PID[ROL][RATE].I = usb_PID.angularVelocity_roll.i;
			//	PID[ROL][RATE].fb_k1 = 0.2f;
			//	PID[ROL][RATE].fb_k2 = 0.0;				//΢������ϵ�������޸ģ����������Ǳ仯�ʴ���΢����һЩ���ᵼ��΢�����								
			PID[ROL][RATE].fb_k1 = PID[ROL][RATE].fb_k2 = usb_PID.angularVelocity_roll.d;	//ANO
	
			PID[ROL][RATE].Ilimit_flag = 0; 
			PID[ROL][RATE].Ilimit = 500;    
			PID[ROL][RATE].Irang = usb_PID.angularVelocity_roll.limit;    
	

			//PITCH��
			//�ǶȻ�
			PID[PIT][ANGLE].P = usb_PID.angleRing_pitch.p;
			PID[PIT][ANGLE].I = usb_PID.angleRing_pitch.i;
			PID[PIT][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k2 = usb_PID.angleRing_pitch.d;
			
			PID[PIT][ANGLE].Ilimit_flag = 0;
			PID[PIT][ANGLE].Ilimit = 35;    
			PID[PIT][ANGLE].Irang = usb_PID.angleRing_pitch.limit;   
			
			//���ٶȻ�
			PID[PIT][RATE].P = usb_PID.angularVelocity_pitch.p;
			PID[PIT][RATE].I =usb_PID.angularVelocity_pitch.i;
		//	PID[PIT][RATE].fb_k1 = 0.2f;
		//	PID[PIT][RATE].fb_k2 = 0.0;		//΢������ϵ�������޸ģ����������Ǳ仯�ʴ���΢����һЩ���ᵼ��΢�����			
			PID[PIT][RATE].fb_k1 = PID[PIT][RATE].fb_k2 = usb_PID.angularVelocity_pitch.d;	//ANO
			
			PID[PIT][RATE].Ilimit_flag = 0; 
			PID[PIT][RATE].Ilimit = 500;    
			PID[PIT][RATE].Irang = usb_PID.angularVelocity_pitch.limit;   
			
			//YAW��
			//�ǶȻ�
			PID[YAW][ANGLE].P = 5.0f;				//���������ϵķɿ�yaw��PID��������
			PID[YAW][ANGLE].I = 0.0f; 
			PID[YAW][ANGLE].fb_k1 = PID[ROL][ANGLE].fb_k1 = 0.5f;

			PID[YAW][ANGLE].Ilimit_flag = 0;
			PID[YAW][ANGLE].Ilimit = 35;    
			PID[YAW][ANGLE].Irang = 200;  
			
			//���ٶȻ�
			PID[YAW][RATE].P = usb_PID.angularVelocity_yaw.p;
			PID[YAW][RATE].I = usb_PID.angularVelocity_yaw.i;
//			PID[YAW][RATE].fb_k1 = 0;
//			PID[YAW][RATE].fb_k2 = 0;
			PID[YAW][RATE].fb_k1 = PID[YAW][RATE].fb_k2 = usb_PID.angularVelocity_yaw.d;
			
			PID[YAW][RATE].Ilimit_flag = 0; 
			PID[YAW][RATE].Ilimit = 200;    
			PID[YAW][RATE].Irang = usb_PID.angularVelocity_yaw.limit;   
	
	
			//λ���ٶȻ�
			//X���ٶ�
			LOC_PID[X].P = usb_PID.positionVelocity_X.p;						//����������ʼֵ����Ҫ��
			LOC_PID[X].I = usb_PID.positionVelocity_X.i; 					//�����
			LOC_PID[X].fb_k1 = 0.00f;																//�����
			LOC_PID[X].fb_k2 = usb_PID.positionVelocity_X.d;				//����������ʼֵ����Ҫ��
			
			LOC_PID[X].Ilimit_flag = 0; 		
			LOC_PID[X].Ilimit = 50;    		
			LOC_PID[X].Irang = usb_PID.positionVelocity_X.limit;    		
			
			//Y���ٶ�
			LOC_PID[Y].P = usb_PID.positionVelocity_Y.p;						//����������ʼֵ����Ҫ��
			LOC_PID[Y].I = usb_PID.positionVelocity_Y.i; 					//�����
			LOC_PID[Y].fb_k1 = 0.00f;																//�����
			LOC_PID[Y].fb_k2 = usb_PID.positionVelocity_Y.d;				//����������ʼֵ����Ҫ��
			
			LOC_PID[Y].Ilimit_flag = 0; 
			LOC_PID[Y].Ilimit = 50;    
			LOC_PID[Y].Irang = usb_PID.positionVelocity_Y.limit;
			
			//�߶Ȼ�
			//�⻷
			ALT_PID[2-1].P = usb_PID.heightRing.p;						//����������ʼֵ����Ҫ��
			ALT_PID[2-1].I = usb_PID.heightRing.i; 					//�����
			ALT_PID[2-1].feedforward_k = 0.0f;
			ALT_PID[2-1].fb_k1 = 0.00f;												//�����
			ALT_PID[2-1].fb_k2 = usb_PID.heightRing.d;				//����������ʼֵ����Ҫ��
			
			ALT_PID[2-1].Ilimit_flag = 0;
			ALT_PID[2-1].Ilimit = 100;    
			ALT_PID[2-1].Irang = usb_PID.heightRing.limit;   
			
			//�߶��ٶȻ�
			//�ڻ�
			ALT_PID[1-1].P = usb_PID.heightVelocity.p;							//����������ʼֵ����Ҫ��
			ALT_PID[1-1].I = usb_PID.heightVelocity.i; 						//�ο�1.0f
			ALT_PID[1-1].feedforward_k = 0.0f;											//�ο�0.05f
			ALT_PID[1-1].fb_k1 = 0.00f;															//�����
			ALT_PID[1-1].fb_k2 = usb_PID.heightVelocity.d;					//ǰ��ֵ
			
			ALT_PID[1-1].Ilimit_flag = 0; 
			ALT_PID[1-1].Ilimit = 50;    
			ALT_PID[1-1].Irang = usb_PID.heightVelocity.limit; 
	}

}

/*****************************************************************************
* ��  ����void IsUnlock(void)
* ��  �ܣ��ж��Ƿ���Ҫ���ַ���
* ��  ������
* ����ֵ��Integral_Flag_s 
* ��  ע: 
*****************************************************************************/
int IsUnlock(void)
{
	if( rc_control.BUTTON2 > 1500 && rc_control.BUTTON2 < 2500 && rc_control.THROTTLE >= 180 )  //�ɻ�����֮���ټ������,��ֹ���ֹ���
	{
		return 0;
		
	}else
	{
		return 1;
	}
}

/******************************************************************************************
��������void LocControl(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
���ã�λ���ٶȻ�����
��������Ϊ��GPS��UMB������������������ֱ���������ٶ�תΪ�Ƕ�
��Դ�������ɿ�
*******************************************************************************************/
void LocControl(float dT_s, int integral_flag_s)
{
	static float vel_fb_d_lpf[2]; 			//����ͨ�˲���ķ�����������΢������
	
	if(1)		//�����Ƿ����ı�־λ
	{
		loc_ctrl_1.exp[X] = fs_s.speed_set_h[X];
		loc_ctrl_1.exp[Y] = fs_s.speed_set_h[Y];

		//һ�׹��ԣ���ͨ���˲�
		LPF_1_(5.0f,dT_s,facc_s.X,vel_fb_d_lpf[X]);
		LPF_1_(5.0f,dT_s,facc_s.Y,vel_fb_d_lpf[Y]);	
	
		loc_ctrl_1.fb[X] = of_s.RealSpeed_x;		//����
		loc_ctrl_1.fb[Y] = of_s.RealSpeed_y;		
		
//		loc_ctrl_1.fb[X] = of_s.RealSpeed_x + 0.03f *vel_fb_d_lpf[X];		//0.03΢������ϵ��
//		loc_ctrl_1.fb[Y] = of_s.RealSpeed_y + 0.03f *vel_fb_d_lpf[Y];		//��ǰ�ٶȷ���=��һ�α����ͻ��ֲ����of_s.RealSpeed_y��+΢���0.03f *vel_fb_d_lpf[Y]��

		for(int i =0;i<2;i++)
		{
			PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
											&LOC_PID[i],					
											loc_ctrl_1.exp[i],					//����ֵ���趨ֵ��
											loc_ctrl_1.fb[i], 					//����ֵ������ֵ��
											0,													//ǰ��ֵ
											integral_flag_s
											);

			loc_ctrl_1.out[i] = LOC_PID[i].OutPut + 0.03f *vel_fb_d_lpf[i];	//0.03΢������ϵ��
//			loc_ctrl_1.out[i] = LOC_PID[i].OutPut;
		}
//		printf("loc_ctrl_1.out,X,Y:%f, %f\r\n", loc_ctrl_1.out[X], loc_ctrl_1.out[Y]);
	}
	else if(0)
	{
		//��̬ģʽ��ֱ���������ٶ�תΪ�Ƕȣ������Ƕȣ�
		loc_ctrl_1.out[X] = (float)MAX_ANGLE/MAX_SPEED *fs_s.speed_set_h[X];
		loc_ctrl_1.out[Y] = (float)MAX_ANGLE/MAX_SPEED *fs_s.speed_set_h[Y];
//		printf("loc_ctrl_1.out[X]:%f\r\n", fs_s.speed_set_h[X]);
//		printf("loc_ctrl_1.out[Y]:%f\r\n", fs_s.speed_set_h[Y]);
	}
}

/******************************************************************************************
��������void AttControl_2(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
���ã��ǶȻ�����
��������ͨ������΢����һ����ȷ������̬�����yaw��������޷����Ա�֤ϵͳ�ȶ������ս���PID���㣬�õ����ٶ�
��Դ�������ɿ�
*******************************************************************************************/
void AttControl_2(float dT_s, int integral_flag_s)
{
  _att_2l_ct_st att_2l_ct;
  float exp_rol_tmp, exp_pit_tmp;				//��λ���Ƕ�
	
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

	//����΢��
	//λ���ٶȻ������Ҫ���ֵĽǶ�
	exp_rol_tmp = -loc_ctrl_1.out[X];						
	exp_pit_tmp = -loc_ctrl_1.out[Y];

//printf("%f, %f\r\n", loc_ctrl_1.out[Y], loc_ctrl_1.out[X]);
	
	if(ABS(exp_rol_tmp + att_2l_ct.exp_rol_adj) < 5)
	{
		att_2l_ct.exp_rol_adj += 0.2f *exp_rol_tmp *dT_s;						//���������ۼ�
		att_2l_ct.exp_rol_adj = LIMIT(att_2l_ct.exp_rol_adj,-1,1);	//���������޷��ڡ�-1��1���ڣ��Ƚ�С����֮Ϊ΢��
	}																															//΢������Ϊ�������˻����õ�ά����ĳ�������Ƕȣ�
																																//�Ͼ�ʵ���龰�д��ڵֿ���
	if(ABS(exp_pit_tmp + att_2l_ct.exp_pit_adj) < 5)
	{
		att_2l_ct.exp_pit_adj += 0.2f *exp_pit_tmp *dT_s;
		att_2l_ct.exp_pit_adj = LIMIT(att_2l_ct.exp_pit_adj,-1,1);
	}
	att_2l_ct.exp_rol = exp_rol_tmp + att_2l_ct.exp_rol_adj;	
	att_2l_ct.exp_pit = exp_pit_tmp + att_2l_ct.exp_pit_adj;

	att_2l_ct.exp_rol = LIMIT(att_2l_ct.exp_rol,-MAX_ANGLE,MAX_ANGLE);			//�����Ƕ��޷�
	att_2l_ct.exp_pit = LIMIT(att_2l_ct.exp_pit,-MAX_ANGLE,MAX_ANGLE);

	att_2l_ct.fb_rol = -oular_angle_s.rol;																	//Ϊ�˱�֤����ϵ����������
	att_2l_ct.fb_pit = oular_angle_s.pit;
//	printf("%f, %f\r\n", oular_angle_s.pit, oular_angle_s.rol);
	
	PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
	  							&PID[ROL][ANGLE],					
								  att_2l_ct.exp_rol,					//����ֵ���趨ֵ��
									att_2l_ct.fb_rol, 					//����ֵ������ֵ��
									0,      										//ǰ��ֵ
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
  								&PID[PIT][ANGLE],					
  								att_2l_ct.exp_pit,					//����ֵ���趨ֵ��
	  							att_2l_ct.fb_pit, 					//����ֵ������ֵ��
	  							0,														//ǰ��ֵ
									integral_flag_s
								 );

								 
//	printf("PID[ANGLE].OutPut,P,R,Y: %f, %f, %f\r\n", PID[PIT][ANGLE].OutPut, PID[ROL][ANGLE].OutPut, PID[YAW][ANGLE].OutPut);

}

/******************************************************************************************
��������void AttControl_1(float dT_s, FLOAT_ANGLE *att_in, FLOAT_XYZ *gyr_in)
���ã����ٶȻ�����
������
��Դ�������ɿ�
*******************************************************************************************/
//static float max_yaw_speed,set_yaw_av_tmp;	//��λ���ֱ�Ϊ�����ٶȣ����ٶ�
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
	//Ŀ����ٶȸ�ֵ
	att_1l_ct.exp_angular_velocity[ROL] = PID[ROL][ANGLE].OutPut;
	att_1l_ct.exp_angular_velocity[PIT] = PID[PIT][ANGLE].OutPut;
	att_1l_ct.exp_angular_velocity[YAW] = (float)((RC_YAW_mid - rc_control.YAW)/10.0f) * PID[YAW][RATE].P;
	
	//Ŀ����ٶ��޷�
	att_1l_ct.exp_angular_velocity[ROL] = LIMIT(att_1l_ct.exp_angular_velocity[ROL],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);
	att_1l_ct.exp_angular_velocity[PIT] = LIMIT(att_1l_ct.exp_angular_velocity[PIT],-MAX_ROLLING_SPEED,MAX_ROLLING_SPEED);

	att_1l_ct.fb_angular_velocity[ROL] = gyro_s.Y;
	att_1l_ct.fb_angular_velocity[PIT] = -(gyro_s.X);
	att_1l_ct.fb_angular_velocity[YAW] = gyro_s.Z;
//	printf("%f, %f, %f\r\n", -(gyro_s.X), gyro_s.Y, gyro_s.Z);
	
	PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
	  							&PID[ROL][RATE],					
								  att_1l_ct.exp_angular_velocity[ROL],					//����ֵ���趨ֵ��
									att_1l_ct.fb_angular_velocity[ROL], 					//����ֵ������ֵ��
									0,														//ǰ��ֵ
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
  								&PID[PIT][RATE],					
  								att_1l_ct.exp_angular_velocity[PIT],					//����ֵ���趨ֵ��
	  							att_1l_ct.fb_angular_velocity[PIT], 					//����ֵ������ֵ��
	  							0,														//ǰ��ֵ
									integral_flag_s
								 );
										 
  PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
  								&PID[YAW][RATE],					
  								att_1l_ct.exp_angular_velocity[YAW],					//����ֵ���趨ֵ��
  								att_1l_ct.fb_angular_velocity[YAW], 					//����ֵ������ֵ��
									0,													  //ǰ��ֵ
									integral_flag_s
								 );
		
//	printf("PID[RATE].OutPut,P,R,Y: %f, %f, %f\r\n", PID[PIT][RATE].OutPut, PID[ROL][RATE].OutPut, PID[YAW][RATE].OutPut);		
}


void AltControl_2(float dT_s, float target_distance, int integral_flag_s)										//target_distance��λmm
{
	loc_ctrl_2.exp[Z] += (fs_s.speed_set_h[Z] * dT_s) * 10;				//��λmm
	loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],loc_ctrl_2.fb[Z]-200,loc_ctrl_2.fb[Z]+200);		//�޷���ÿһʱ������������ܳ���200mm����
	if(target_distance > -0.0001f)							//ֻ��Ϊ�Ǹ�������
		loc_ctrl_2.exp[Z] = LIMIT(loc_ctrl_2.exp[Z],0, target_distance);
	loc_ctrl_2.fb[Z] = of_s.Distance;
	
  PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
  								&ALT_PID[2-1],					
  								loc_ctrl_2.exp[Z],					//����ֵ���趨ֵ��
  								loc_ctrl_2.fb[Z], 					//����ֵ������ֵ��
									0,													//ǰ��ֵ
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
		loc_ctrl_1.exp[Z] = 0.6f * fs_s.speed_set_h[Z] + ALT_PID[2-1].OutPut;						//�ٶȲ���ϵ��0.6f��ֱ�Ӹ��ٶȣ�Ϊ�˱�֤��һ���ȶ��ٶȷ���
	else
		loc_ctrl_1.exp[Z] = ALT_PID[2-1].OutPut;
	
	loc_ctrl_1.fb[Z] = of_s.RealSpeed_z;

  PID_Postion_Cal(dT_s,												//���ڣ���λ���룩
  								&ALT_PID[1-1],					
  								loc_ctrl_1.exp[Z],					//����ֵ���趨ֵ��
  								loc_ctrl_1.fb[Z], 					//����ֵ������ֵ��
									0,													  //ǰ��ֵ
									integral_flag_s
								 );
	
	Z_Acc_Lpf += 0.2f * (facc_s.Z - Z_Acc_Lpf);				//��ͨ�˲�
	ALT_PID[1-1].OutPut += fb_k2 * Z_Acc_Lpf;					//΢�����У��ϱ�PID����΢��ϵ��Ϊ0
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
	if(rc_control.THROTTLE < 1200)//��������������λ�����ŵ�λ���������
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
	
//	printf("��־λ������%d, %d\r\n", flag_s[unlock_f], flag_s[imu_init_f]);
	if(flag_s[unlock_f] == 1 && flag_s[imu_init_f] == 1) //�ɻ�����ʱ�����������Ч
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
//	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, Moto_PWM_f[4-1]); //������ֵ���䵽��ʱ���������Ӧռ�ձȵ�PWM��
	
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1500);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1500); //������ֵ���䵽��ʱ���������Ӧռ�ձȵ�PWM��
}

void MotorPwm_Init(void)
{
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 2000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 2000); //������ֵ���䵽��ʱ���������Ӧռ�ձȵ�PWM��
	HAL_Delay(3000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_1, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_2, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_3, 1000);
	__HAL_TIM_SET_COMPARE(&htim3, TIM_CHANNEL_4, 1000); //������ֵ���䵽��ʱ���������Ӧռ�ձȵ�PWM��
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
	
	AttControl_2(2e-3f, Integral_Flag_s); 		//���ڣ�ŷ���ǣ����ٶȣ����ַ����־
	AttControl_1(2e-3f, Integral_Flag_s);
	
	AltControl_2(2e-3f, fs_s.distance_set_h[Z], Integral_Flag_s);
	AltControl_1(2e-3f, fs_s.distance_set_h[Z], Integral_Flag_s);


	Motcontrol();
	
}
