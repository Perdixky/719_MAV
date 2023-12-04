/******************************************************************************************
ģ������PID������
���ߣ�	719-���Ӿ�
*******************************************************************************************/
#include "719_PID.h"

void PID_Postion_Cal(float dT_s,						//���ڣ���λ���룩
										 PID_TYPE*PID,					
										 float target,					//����ֵ���趨ֵ��
										 float measure, 				//����ֵ������ֵ��
										 float feedforward,			//ǰ��ֵ
										 int Integral_Flag			//���ַ����־
										)
{
	float hz = safe_div(1.0f,dT_s,0);
	
	PID->target = target;
	PID->feedback = measure;
	PID->Error  = target - measure;           					    								//���
	
	PID->Pout = PID->P * PID->Error;                        								//��������
	PID->Iout = PID->Ilimit_flag * PID->I * PID->Integral;  								//���ֿ���
	PID->Dout = PID->fb_k1 * (PID->target - PID->target_pre) * hz
						- PID->fb_k2 * (PID->feedback - PID->feedback_pre) * hz;			//΢�ֿ��ƣ���ȡ����������

	if( Integral_Flag == 0 )  //�ɻ�����֮���ټ������,��ֹ���ֹ���
	{
		if(measure > (PID->Ilimit)||measure < -PID->Ilimit)   //���ַ��루�����������趨ֵƫ��ϴ�ʱ��ȡ���������ã�
		{PID->Ilimit_flag = 0;}
		else
		{
			PID->Ilimit_flag = 1;                               //������֣����ַ����־Ϊ1���������������ڣ�
			PID->Integral += PID->Error;                        //�������л���
			PID->Integral = LIMIT(PID->Integral, -PID->Irang, PID->Irang);		//�����޷�           
		}
	}else
	{
		PID->Integral = 0;
	}
	
	PID->OutPut = PID->feedforward_k *feedforward + PID->Pout + PID->Iout + PID->Dout; //ǰ�� + ���� + ���� + ΢���ܿ���
	
	PID->PreError = PID->Error ;                            //ǰһ�����ֵ
	PID->target_pre = PID->target;													//�¾�����ֵ����
	PID->feedback_pre = PID->feedback;											//�¾ɷ���ֵ����
}

