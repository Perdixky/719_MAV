#include "iic.h"
#include "gpio.h"
 
static void delay_us(uint32_t u_sec)
{
	uint16_t cnt = 0;
 
	while(u_sec--)
	{
		for(cnt=168/5; cnt>0; cnt--);
	}
}
 
void IIC_Init(void)
{
	IIC_SDA_OUT(); //PB11���ģʽ
}
 
/*
 * @brief IIC_SDA����ģʽ
 */
void IIC_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IIC_SDA_PIN;                      //IIC��SDA����
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                 //����ģʽ
	GPIO_InitStruct.Pull = GPIO_PULLUP;                     //����
	HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);   			//����SDA��
}
/*
 * @brief IIC_SDA���ģʽ
 */
void IIC_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IIC_SDA_PIN;                     //IIC��SDA����
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;            //��©���ģʽ
	GPIO_InitStruct.Pull = GPIO_PULLUP;                    //����
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;          //SDA���ٶ�
	HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct); 			 //����SDA��
}
/*
 * @brief IIC��ʼ�ź�
 */
void IIC_Start(void)
{
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_SET); //������SDA������SCL����ֹ���ִ����ź�
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2); //��ʱ��֤SCL�ߵ�ƽʱ��SDA�½���
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //ǯסI2C���ߣ�׼�����ͻ��������
}
 
/*
 * @brief IICֹͣ�ź�
 */
void IIC_Stop(void)
{
	//IIC_SCL(GPIO_PIN_RESET); //phph2045
	IIC_SDA_OUT(); //phph2045
	IIC_SDA(GPIO_PIN_RESET); //������SDA������SCL����ֹ���ִ����ź�
	delay_us(2); //��ʱ��֤SCL�ߵ�ƽʱ��SDA������ //phph2045
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2); //��ʱ��֤SCL�ߵ�ƽʱ��SDA������
	IIC_SDA(GPIO_PIN_SET);
	//����Ͳ�������SCL�ˣ���ΪIICͨѶ�Ѿ�����
	//IIC_SCL(GPIO_PIN_RESET); //ǯסI2C���ߣ�׼�����ͻ��������
}
 
/*
 * @brief IIC�����ȴ�Ӧ��
 * ����ֵ��0������Ӧ��ɹ�
 *         -1������Ӧ��ʧ��
 */
int8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
 
	IIC_SDA(GPIO_PIN_SET); //һ��Ҫ������SDA��������SCL������Ϳ��ܱ����ʼ�źźͽ����ź���
	delay_us(1);
	IIC_SDA_IN(); //SDA����Ϊ����
	delay_us(1); //phph2045
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	while(IIC_Read_SDA())
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{
			IIC_Stop();
			IIC_SDA_OUT(); //SDA�ָ�Ϊ���
			return 1;
		}
	}
	IIC_SCL(GPIO_PIN_RESET); //IICͨѶû����������SCL��
	IIC_SDA_OUT(); //SDA�ָ�Ϊ���
 
	return 0;
}
 
//����ACKӦ��
void IIC_Ack(void)
{
	//IIC_SCL(GPIO_PIN_RESET); //phph2047
	IIC_SDA_OUT(); //SDA����Ϊ���
	//IIC_SCL(GPIO_PIN_RESET); //phph2046
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //IICͨѶû����������SCL��
}
 
//����NACKӦ��
void IIC_NAck(void)
{
	IIC_SCL(GPIO_PIN_RESET); //phph2045 
	IIC_SDA_OUT(); //SDA����Ϊ���
	//IIC_SCL(GPIO_PIN_RESET); //phph2046
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //IICͨѶû����������SCL��
}
 
//��1���ֽڣ�ack=1ʱ������ACk��ack=0������NACK
uint8_t IIC_Read_Byte(uint8_t ack)
{
	uint8_t i = 0;
	uint8_t recv = 0;
 
	IIC_SDA_IN(); //SDA����Ϊ����
 
	for(i=0; i<8; i++)
	{
		IIC_SCL(GPIO_PIN_RESET); //����ʱ����
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET); //����ʱ���ߣ�����1λ����
		recv <<= 1;
		//phph2045 delay_us(2);
		if(IIC_Read_SDA())
			recv |= 0x01;
		delay_us(2);
	}
	IIC_SCL(GPIO_PIN_RESET); //phph2047
 
	if(!ack)
		IIC_NAck(); //����NACK
	else
		IIC_Ack(); //����ACK
 
	IIC_SDA_OUT(); //SDA�ָ�Ϊ���
 
	return recv;
}
 
/*
 * @brief IIC����һ���ֽ�
 * @para  ��д����ֽ�����
 * ���شӻ�����Ӧ��
 *    1����Ӧ��
 *    0����Ӧ��
 */
void IIC_Send_Byte(uint8_t data)
{
	uint8_t i = 0;
 
	IIC_SDA_OUT();
	//phph2045 IIC_SCL(GPIO_PIN_RESET); //����ʱ�ӿ�ʼ���ݴ���
 
	for(i=0; i<8; i++)
	{
		IIC_SCL(GPIO_PIN_RESET); //����ʱ����
		//׼������
		if(data & 0x80)
			IIC_SDA(GPIO_PIN_SET);
		else
			IIC_SDA(GPIO_PIN_RESET);
		data <<= 1;
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET); //����ʱ���ߣ�����1λ����
		delay_us(2);
	}
	IIC_SCL(GPIO_PIN_RESET); //ͨѶδ����������SCL
	delay_us(2); //phph2045
}

//IIC������
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length)
{
  uint32_t count = 0;

  IIC_Start();
  IIC_Send_Byte(dev);	
  if(IIC_Wait_Ack() == 1)return 0;
  IIC_Send_Byte(reg);
  if(IIC_Wait_Ack() == 1)return 0;
  IIC_Start();
  IIC_Send_Byte(dev+1); 
  if(IIC_Wait_Ack() == 1)return 0;

  for(count=0; count<length; count++)
  {
      if(count!=length-1)data[count]=IIC_Read_Byte(1);
      else  data[count]=IIC_Read_Byte(0);	 
  }
  IIC_Stop();
  return 1;
}

//IIC����д
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length)
{
  uint32_t count = 0;
  IIC_Start();
  IIC_Send_Byte(dev);	   
  if(IIC_Wait_Ack() == 1)return 0;
  IIC_Send_Byte(reg);   
  if(IIC_Wait_Ack() == 1)return 0;
  for(count=0; count<length; count++)
  {
      IIC_Send_Byte(data[count]);
      if(IIC_Wait_Ack() == 1)return 0;
  }
  IIC_Stop();

  return 1; 
}
