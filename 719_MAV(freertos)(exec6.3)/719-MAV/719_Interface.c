/******************************************************************************************
ģ�����������ӿ�ģ��
���ã��������ж���д������
������Classis_Interface_in��Classis_Interface_out
			Sensor_Interface_in��Sensor_Interface_out
			Trifles_Interface_in��Trifles_Interface_out
���ߣ�719-κ�ο������Ӿ�
*******************************************************************************************/
#include "719_Interface.h"
#include "FreeRTOS.h"
#include "task.h"
#include "main.h"
#include "cmsis_os.h"

//=========================================================������������=============================================================//
Classis_Container classis;						//������������ʵ��
extern osMutexId ChassisMutexHandle;	//����������������
/******************************************************************************************
ģ������								������������(д��)�ӿں���
���ã�									������д��ȫ�ֱ���
******RCcontrol��				����RC������Դ���洢���յ���ͨ����
******Oular_Angle&fAcc������Oular_angle,fAcc������Դ���洢�ǶȺͽ��ٶ���Ϣ
******FS��							����ά�ؼĴ�����ȡ��������Դ���洢�Ĵ�����ȡ��
******Sreg[REGSIZE]��		����Ŀ��X,Y���ٶ���Ϣ������Դ���洢��X,Y���ٶ���Ϣ
******of��							�����������X,Y���ٶ���Ϣ������Դ���洢��X,Y���ٶ���Ϣ
******printf:						������������Ƿ�����
*******************************************************************************************/
void Classis_Interface_in(char* fmt, ...)
{
	RC_TYPE* RCcontrol = NULL;
	FLOAT_ANGLE* Oular_Angle = NULL;
	FLOAT_XYZ* Gyro = NULL;
	FLOAT_XYZ* fAcc = NULL;
	_flight_state_st* FS = NULL;
	_fly_ct_st* Program_Ctrl = NULL;
	int16_t* Sreg = NULL;
	OF* of = NULL;
	
	va_list args;
	va_start(args, fmt);
	
	while(*fmt)
	{
		switch(*fmt++)
		{
			case 'a':
				RCcontrol = va_arg(args, RC_TYPE*); break;
			case 'b':
				Oular_Angle = va_arg(args, FLOAT_ANGLE*); break;
			case 'c':
				if(Gyro == NULL)
				{Gyro = va_arg(args, FLOAT_XYZ*); break;}
				else
				{fAcc = va_arg(args, FLOAT_XYZ*); break;}
			case 'd':
				FS = va_arg(args, _flight_state_st*); break;
			case 'e':
				Program_Ctrl = va_arg(args, _fly_ct_st*); break;
			case 'f':
				Sreg = va_arg(args, int16_t*); break;
			case 'g':
				of = va_arg(args, OF*); break;
		}
	}
	if(osMutexWait(ChassisMutexHandle, 1) == osOK)
	{
		if(RCcontrol != NULL)
		{
			RCcontrol->ROLL = classis.RC_Control.ROLL;
			RCcontrol->PITCH = classis.RC_Control.PITCH;
			RCcontrol->THROTTLE = classis.RC_Control.THROTTLE;
			RCcontrol->YAW = classis.RC_Control.YAW;
			RCcontrol->BUTTON1 = classis.RC_Control.BUTTON1;
			RCcontrol->BUTTON2 = classis.RC_Control.BUTTON2;
//			printf("%d, %d, %d, %d, %d, %d\r\n",RCcontrol->ROLL, RCcontrol->PITCH, RCcontrol->THROTTLE, RCcontrol->YAW, RCcontrol->BUTTON1, RCcontrol->BUTTON2);
		}
		if(Oular_Angle != NULL)
		{
			Oular_Angle->rol = classis.oular_angle.rol;
			Oular_Angle->pit = classis.oular_angle.pit;
			Oular_Angle->yaw = classis.oular_angle.yaw;
			
//			printf("Oular_Angle->pit,rol,yaw:%f, %f, %f\r\n", Oular_Angle->pit, Oular_Angle->rol, Oular_Angle->yaw);

		}
		if(Gyro != NULL)
		{
			Gyro->X = classis.gyro.X;
			Gyro->Y = classis.gyro.Y;	
			Gyro->Z = classis.gyro.Z;
			
//			printf("Gyro->X,Y,Z:%f, %f, %f\r\n", Gyro->X, Gyro->Y, Gyro->Z);
		}
		if(fAcc != NULL)
		{
			fAcc->X = classis.facc.X;
			fAcc->Y = classis.facc.Y;	
			fAcc->Z = classis.facc.Z;
//			printf("fAcc->X, Y, Z:%f, %f, %f\r\n", fAcc->X, fAcc->Y, fAcc->Y);
			
		}
		if(Sreg != NULL)
		{
			for(uint8_t i = 0; i < 12; i++)
			{
//				fAcc[i] = sReg_s[AX+i] / 32768.0f * 16.0f;
				Sreg[AX+i] = classis.sreg[AX+i];
			}
		}
		if(FS != NULL)
		{
			FS->speed_set_h[X] = classis.fs.speed_set_h[X];
			FS->speed_set_h[Y] = classis.fs.speed_set_h[Y];
			FS->speed_set_h[Z] = classis.fs.speed_set_h[Z];
			FS->distance_set_h[X] = classis.fs.distance_set_h[X];
			FS->distance_set_h[Y] = classis.fs.distance_set_h[Y];
			FS->distance_set_h[Z] = classis.fs.distance_set_h[Z];
//			printf("%f, %f, %f, %f\r\n",FS->speed_set_h[X],FS->speed_set_h[Y], FS->speed_set_h[Z], FS->distance_set_h[Z] );
		}
		if(Program_Ctrl != NULL)
		{
			Program_Ctrl->vel_cmps_h[X] = classis.program_ctrl.vel_cmps_h[X];
			Program_Ctrl->vel_cmps_h[Y] = classis.program_ctrl.vel_cmps_h[Y];
			Program_Ctrl->vel_cmps_h[Z] = classis.program_ctrl.vel_cmps_h[Z];
			Program_Ctrl->distance_cmps_h[Z] = classis.program_ctrl.distance_cmps_h[Z];
//			printf("%f, %f, %f, %f\r\n",Program_Ctrl->vel_cmps_h[X],Program_Ctrl->vel_cmps_h[Y], Program_Ctrl->vel_cmps_h[Z], Program_Ctrl->distance_cmps_h[Z] );
		}
		if(of != NULL)
		{
	
			of->Distance = classis.Of.Distance;
			of->RealSpeed_x = classis.Of.RealSpeed_x;
			of->RealSpeed_y = classis.Of.RealSpeed_y;
//			printf("%f, %f, %f\r\n",of->Distance, of->RealSpeed_x, of->RealSpeed_y );
		}
  }
	osMutexRelease(ChassisMutexHandle);
}

/******************************************************************************************
ģ������								������������(д��)�ӿں���
���ã�									�����ݴ�ȫ�ֱ�����������ģ����ľ�ֹȫ�ֱ���
******RCcontrol��				����RC������Դ���洢���յ���ͨ����
******Oular_Angle&fAcc������Oular_angle,fAcc������Դ���洢�ǶȺͽ��ٶ���Ϣ
******FS��							����ά�ؼĴ�����ȡ��������Դ���洢�Ĵ�����ȡ��
******Sreg[REGSIZE]��		����Ŀ��X,Y���ٶ���Ϣ������Դ���洢��X,Y���ٶ���Ϣ
******of��							�����������X,Y���ٶ���Ϣ������Դ���洢��X,Y���ٶ���Ϣ
******printf:						������������Ƿ�����
*******************************************************************************************/
void Classis_Interface_out(char* fmt, ...)
{
	RC_TYPE* RCcontrol = NULL;
	FLOAT_ANGLE* Oular_Angle = NULL;
	FLOAT_XYZ* Gyro = NULL;
	FLOAT_XYZ* fAcc = NULL;
	_flight_state_st* FS = NULL;
	_fly_ct_st* Program_Ctrl = NULL;
	int16_t* Sreg = NULL;
	OF* of = NULL;
	
	va_list args;
	va_start(args, fmt);
	
	while(*fmt)
	{
		switch(*fmt++)
		{
			case 'a':
				RCcontrol = va_arg(args, RC_TYPE*); break;
			case 'b':
				Oular_Angle = va_arg(args, FLOAT_ANGLE*); break;
			case 'c':
				if(Gyro == NULL)
				{Gyro = va_arg(args, FLOAT_XYZ*); break;}
				else
				{fAcc = va_arg(args, FLOAT_XYZ*); break;}
			case 'd':
				FS = va_arg(args, _flight_state_st*); break;
			case 'e':
				Program_Ctrl = va_arg(args, _fly_ct_st*); break;
			case 'f':
				Sreg = va_arg(args, int16_t*); break;
			case 'g':
				of = va_arg(args, OF*); break;
		}
	}
	if(osMutexWait(ChassisMutexHandle, 2) == osOK)
	{
		if(RCcontrol != NULL)
		{
			classis.RC_Control.ROLL = RCcontrol->ROLL;
			classis.RC_Control.PITCH = RCcontrol->PITCH;
			classis.RC_Control.THROTTLE = RCcontrol->THROTTLE;
			classis.RC_Control.YAW = RCcontrol->YAW;
			classis.RC_Control.BUTTON1 = RCcontrol->BUTTON1;
			classis.RC_Control.BUTTON2 = RCcontrol->BUTTON2;
//			printf("%d, %d, %d, %d, %d, %d\r\n",classis.RC_Control.ROLL, classis.RC_Control.PITCH, classis.RC_Control.THROTTLE, classis.RC_Control.YAW, classis.RC_Control.BUTTON1, classis.RC_Control.BUTTON2);
		}
		if(Oular_Angle != NULL)
		{
			classis.oular_angle.rol = Oular_Angle->rol;
			classis.oular_angle.pit = Oular_Angle->pit;
			classis.oular_angle.yaw = Oular_Angle->yaw;

//			printf("Oular_Angle->pit,rol,yaw:%f, %f, %f\r\n", classis.oular_angle.pit, classis.oular_angle.rol, classis.oular_angle.yaw);
			
		}
		if(Gyro != NULL)
		{
			classis.gyro.X = Gyro->X;
			classis.gyro.Y = Gyro->Y;
			classis.gyro.Z = Gyro->Z;
			
//			printf("classis.gyro.X, Y, Z:%f, %f, %f\r\n", classis.gyro.X, classis.gyro.Y, classis.gyro.Z);
			
		}
		if(fAcc != NULL)
		{
			classis.facc.X = fAcc->X;
			classis.facc.Y = fAcc->Y;
			classis.facc.Z = fAcc->Z;
			
//			printf("classis.facc.X,Y,Z:%f, %f, %f\r\n", classis.facc.X, classis.facc.Y, classis.facc.Z);

		}
		if(Sreg != NULL)
		{
				for(uint8_t i = 0; i < 12; i++)
				{
//					fAcc[i] = sReg_s[AX+i] / 32768.0f * 16.0f;
					classis.sreg[AX+i] = Sreg[AX+i];					
				}
		}
		if(FS != NULL)
		{
			classis.fs.speed_set_h[X] = FS->speed_set_h[X];
			classis.fs.speed_set_h[Y] = FS->speed_set_h[Y];
			classis.fs.speed_set_h[Z] = FS->speed_set_h[Z];
			classis.fs.distance_set_h[X] = FS->distance_set_h[X];
			classis.fs.distance_set_h[Y] = FS->distance_set_h[Y];
			classis.fs.distance_set_h[Z] = FS->distance_set_h[Z];
//			printf("%f, %f, %f, %f\r\n",classis.fs.speed_set_h[X],classis.fs.speed_set_h[Y], classis.fs.speed_set_h[Z], classis.fs.distance_set_h[Z] );
		}
		if(Program_Ctrl != NULL)
		{
			classis.program_ctrl.vel_cmps_h[X] = Program_Ctrl->vel_cmps_h[X];
			classis.program_ctrl.vel_cmps_h[Y] = Program_Ctrl->vel_cmps_h[Y];
			classis.program_ctrl.vel_cmps_h[Z] = Program_Ctrl->vel_cmps_h[Z];
			classis.program_ctrl.distance_cmps_h[Z] = Program_Ctrl->distance_cmps_h[Z];
//			printf("%f, %f, %f, %f\r\n",classis.program_ctrl.vel_cmps_h[X],classis.program_ctrl.vel_cmps_h[Y], classis.program_ctrl.vel_cmps_h[Z], classis.program_ctrl.distance_cmps_h[Z] );
//			printf("%f\r\n", Program_Ctrl->vel_cmps_h[X]);			//����һ�η���̫��Ῠ����
		}
		if(of != NULL)
		{
			classis.Of.Distance = of->Distance;
			classis.Of.RealSpeed_x = of->RealSpeed_x;
			classis.Of.RealSpeed_y = of->RealSpeed_y;
		}
	}
	osMutexRelease(ChassisMutexHandle);
}
//=========================================================������������=============================================================//

//=========================================================��������������===========================================================//
extern osMutexId SensorMutexHandle;		//������������������

//=========================================================��������������===========================================================//

//=========================================================���´�����������=========================================================//
extern osMutexId TriflesMutexHandle;	//�������´�����������

//=========================================================���´�����������=========================================================//


/******************************************************************************************
ģ�������¼���־��ӿں���
��˼���¼���־���ڲ���������޷���װ��������ʱ�����£������ͬʱ���У�����ʱ���λ�� �ڸո�
��־������1��ʱ����һ�̣�CPU�л�����һ�����̣������ǡ�ý���Ӧ�ı�־λ����Ϊ1����Ϣȱʧ��
*******************************************************************************************/
void Flag_Interface_in(int Flag[GroupOfFlag])	//�����־λ������Դ���������¼��ı�־λ
{
	
}

void Flag_Interface_out(int Flag[GroupOfFlag])
{
	
}
