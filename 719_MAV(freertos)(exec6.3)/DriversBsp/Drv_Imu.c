/******************************************************************************************
模块名：IMU获取模块
输入量：sReg
输出量：三轴角度和三轴角速度，imu_init_f标志位
作者：	719-张木悦
*******************************************************************************************/
#include "Drv_Imu.h"
#include "cmsis_os.h"

static volatile char s_cDataUpdate = 0, s_cCmd = 0xff;
void Get_ImuData_Init(void)
{
   WitInit(WIT_PROTOCOL_I2C, 0x50);
   WitI2cFuncRegister(IICwriteBytes, IICreadBytes);
   WitRegisterCallBack(CopeSensorData);
   WitDelayMsRegister(Delayms);
   printf("\r\n********************** wit-motion IIC example  ************************\r\n");
   AutoScanSensor();
}

static void CopeSensorData(uint32_t uiReg, uint32_t uiRegNum)
{
	int i;
    for(i = 0; i < uiRegNum; i++)
    {
        switch(uiReg)
        {
            case AZ:
				s_cDataUpdate |= ACC_UPDATE;
            break;
            case GZ:
				s_cDataUpdate |= GYRO_UPDATE;
            break;
            case HZ:
				s_cDataUpdate |= MAG_UPDATE;
            break;
            case Yaw:
				s_cDataUpdate |= ANGLE_UPDATE;
            break;
            default:
				s_cDataUpdate |= READ_UPDATE;
			break;
        }
		uiReg++;
    }
}

static void Delayms(uint16_t ucMs)
{
	HAL_Delay(ucMs);
}

static void AutoScanSensor(void)
{
	int i, iRetry;
	
	for(i = 0; i < 0x7F; i++)
	{
		WitInit(WIT_PROTOCOL_I2C, i);
		iRetry = 2;
		do
		{
			s_cDataUpdate = 0;
			WitReadReg(AX, 3);
			HAL_Delay(5);
			if(s_cDataUpdate != 0)
			{
				printf("find %02X addr sensor\r\n", i);
				ShowHelp();
				return ;
			}
			iRetry--;
		}while(iRetry);		
	}
	printf("can not find sensor\r\n");
	printf("please check your connection\r\n");
}

static void ShowHelp(void)
{
	printf("\r\n************************	 WIT_SDK_DEMO	************************");
	printf("\r\n************************          HELP           ************************\r\n");
	printf("UART SEND:a\\r\\n   Acceleration calibration.\r\n");
	printf("UART SEND:m\\r\\n   Magnetic field calibration,After calibration send:   e\\r\\n   to indicate the end\r\n");
	printf("UART SEND:U\\r\\n   Bandwidth increase.\r\n");
	printf("UART SEND:u\\r\\n   Bandwidth reduction.\r\n");
	printf("UART SEND:B\\r\\n   Baud rate increased to 115200.\r\n");
	printf("UART SEND:b\\r\\n   Baud rate reduction to 9600.\r\n");
	printf("UART SEND:h\\r\\n   help.\r\n");
	printf("******************************************************************************\r\n");
}
static float fAcc[3], fGyro[3], fAngle[3];
static FLOAT_XYZ facc_s;
static FLOAT_XYZ fgyro_s;
static FLOAT_ANGLE oular_angle_s;
static int16_t sReg_s[REGSIZE];
static uint8_t FLAG1_s = 1;
static uint16_t tim_flag_s = 0;

extern osEventFlagsId_t ClassisEventHandle;
void Get_ImuData_Server(void)
{
	tim_flag_s++;
	
	Classis_Interface_in("f", sReg_s);										//数据接口
	WitReadReg(AX, 12);
	if(s_cDataUpdate)
	{
		for(uint8_t i = 0; i < 3; i++)
		{
			fAcc[i] = sReg_s[AX+i] / 32768.0f * 16.0f;
			fGyro[i] = sReg_s[GX+i] / 32768.0f * 2000.0f;
			fAngle[i] = sReg_s[Roll+i] / 32768.0f * 180.0f;
		}
		//得到三轴角度和三轴角速度
    oular_angle_s.rol = fAngle[1];
    oular_angle_s.pit = fAngle[0];
    oular_angle_s.yaw = fAngle[2];
//		printf("oular_angle_s.pit, rol, yaw:%f, %f, %f\r\n", oular_angle_s.pit, oular_angle_s.rol, oular_angle_s.yaw);
		
		facc_s.Y = fAcc[1];
		facc_s.X = fAcc[0];
		facc_s.Z = fAcc[2];
//		printf("facc_s.X,Y,Z:%f, %f, %f\r\n", facc_s.X, facc_s.Y, facc_s.Z);
		
		fgyro_s.Y = fGyro[1]; 
		fgyro_s.X = fGyro[0]; 
		fgyro_s.Z = fGyro[2]; 
//		printf("fgyro_s.X, Y, Z:%f, %f, %f\r\n", fgyro_s.X, fgyro_s.Y, fgyro_s.Z);
		
		if(oular_angle_s.rol <= 20.0 && oular_angle_s.pit <= 20.0 && tim_flag_s >= 200 && FLAG1_s == 1)
		{
			FLAG1_s = 0;
			
			osEventFlagsSet(ClassisEventHandle, 1<<imu_init_f);
		}
		else if(oular_angle_s.rol > 20.0 || oular_angle_s.pit > 20.0)
		{
			tim_flag_s = 0;
		}
//		printf("检测各个数据事件标志组：%f,%f, %d, %d\r\n", oular_angle_s.rol, oular_angle_s.pit, tim_flag_s, FLAG1_s);
		Classis_Interface_out("bcc", &oular_angle_s, &fgyro_s, &facc_s);		//数据接口
		
		if(s_cDataUpdate & ACC_UPDATE)
		{
			s_cDataUpdate &= ~ACC_UPDATE;
		}
		if(s_cDataUpdate & GYRO_UPDATE)
		{
			s_cDataUpdate &= ~GYRO_UPDATE;
		}
		if(s_cDataUpdate & ANGLE_UPDATE)
		{
			s_cDataUpdate &= ~ANGLE_UPDATE;
		}
		if(s_cDataUpdate & MAG_UPDATE)
		{
			s_cDataUpdate &= ~MAG_UPDATE;
		}
	}
	
}
