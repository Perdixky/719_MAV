#include "main.h"
#include "719_PID.h"
#include "719_FC.h"
#include "Drv_ws2812.h"
#include "Drv_Imu.h"

//extern FLOAT_ANGLE oular_angle;
//extern FLOAT_XYZ   gyro;

//uint8_t control_flag = 0;
//uint8_t led_station_flag = 0;
//int AttControl_2_flag = 0;
//int AttControl_1_flag = 0;
//int LocControl_1_flag = 0;
//int XYZVelocity_Set_flag = 0;

void Task_Schedule(void)
{
//	if(control_flag == 1)
//	{
//		Get_ImuData();
//		control_flag = 0;
//		if(XYZVelocity_Set_flag>0)		//�ɿ�ϵͳ�ٶ�Ŀ�����ۺ��趨��1���������趨һ��
//		{
//			XYZ_Velocity_Set(1e-3f);
//			XYZVelocity_Set_flag = 0;
//		}
//		if(AttControl_2_flag>4)				//�ٶȻ���5���Ŀ���һ��
//		{
//			AttControl_2(5e-3f, &oular_angle, &gyro);
//			AttControl_2_flag = 0;
//		}
//		if(AttControl_1_flag>1)				//���ٶȻ���2���Ŀ���һ��
//		{
//			AttControl_1(2e-3f, &oular_angle, &gyro);
//			AttControl_1_flag = 0;
//		}
//		if(LocControl_1_flag>19)   		//�߶��ٶȻ���20���Ŀ���һ��
//		{
//			LocControl(20e-3f, &oular_angle, &gyro);
//			LocControl_1_flag = 0;
////			printf("hello, world!\r\n");
//		}
//	}
	
//	if(led_station_flag == 1 && lock == 0)
//	{
//		led_station_flag = 0;
//        ws2812_example();
//	}	
}


