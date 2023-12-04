/******************************************************************************************
ģ���������ջ�PPM����ģ��
����������
�������rc_controlң����ͨ����
˵��  ��
PPM_Databuf[0]-------------���
PPM_Databuf[1]-------------����
PPM_Databuf[2]-------------����
PPM_Databuf[3]-------------ƫ��
PPM_Databuf[4]-------------����ģʽ
PPM_Databuf[5]-------------��
���ߣ�719-��ľ�ã����Ӿ�
*******************************************************************************************/
#include "Drv_ppm.h"
#include "719_PID.h"

#define PPM_Chn_Max 8//���ͨ����
static uint16_t PPM_Sample_Cnt = 0;//ͨ��
static uint32_t PPM_Time = 0;//��ȡͨ��ʱ��
static uint16_t PPM_Okay = 0;//��һ�ν���״̬
static int16_t PPM_Databuf[8] = {0};//����ͨ��������


void get_rc_data(void)
{
	RC_TYPE  rc_control;		
	
	Classis_Interface_in(NULL);							//���ݽӿڣ���ȫ�ֱ����л�ȡ����
	
	rc_control.ROLL = PPM_Databuf[0];
	rc_control.PITCH = PPM_Databuf[1];
	rc_control.THROTTLE = PPM_Databuf[2];
	rc_control.YAW = PPM_Databuf[3];
	rc_control.BUTTON1 = PPM_Databuf[4];
	rc_control.BUTTON2 = PPM_Databuf[5];
	
//	printf("%d, %d, %d, %d, %d, %d\r\n",rc_control.ROLL, rc_control.PITCH, rc_control.THROTTLE, rc_control.YAW, rc_control.BUTTON1, rc_control.BUTTON2);
	
	Classis_Interface_out("a", &rc_control);		//���ݽӿڣ��������ݵ�ȫ�ֱ�������
}

//PPM�����жϻص�����
void HAL_GPIO_EXTI_Callback(uint16_t GPIO_Pin)
{
    if(GPIO_Pin == GPIO_PIN_7)//�ж��Ƿ�Ϊ�������������жϣ���������ΪPIN8
    {
        PPM_Time = TIM4 ->CNT;//����ʱ��ת��
        TIM4 -> CNT = 0;//����������
        if (PPM_Okay == 1)//�ж��Ƿ����µ�һ�ֽ���
        {
            PPM_Sample_Cnt++;//ͨ����+1
            PPM_Databuf[PPM_Sample_Cnt - 1] = PPM_Time;//��ÿһ��ͨ������ֵ��������
            if (PPM_Sample_Cnt >= PPM_Chn_Max)//�ж��Ƿ񳬹��ͨ����
                PPM_Okay = 0;
        }
        if (PPM_Time >= 2050)//��ʱ�����½��ؼ���ͨ�����ݣ�������һ�ֽ���
        {
            PPM_Okay = 1;
            PPM_Sample_Cnt = 0;
        }
    }
}

