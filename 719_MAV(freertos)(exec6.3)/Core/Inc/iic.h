#ifndef __BSP_I2C_H
#define __BSP_I2C_H
 
#include "main.h"
 
#define DEVICE_ADDRESS	0xA0 //7λ��ַ
#define BIT_WRITE	0
#define BIT_READ	1
 
//���Ŷ���
/*
 * SCL����PB6
 * SDA����PB7
 */
//#define IIC_SDA_IN()		{GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}//PB7����ģʽ
//#define IIC_SDA_OUT()	{GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}//PB7���ģʽ
 
#define IIC_SCL_PIN	GPIO_PIN_6
#define IIC_SDA_PIN	GPIO_PIN_7
#define IIC_GPIO_PORT	GPIOB
#define IIC_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
 
//IIC��IO��������
//���κ꣬��������������һ��ʹ��
#define IIC_SCL(a)		HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, a)
#define IIC_SDA(a)		HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, a)
#define IIC_Read_SDA()	HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN)
 
void IIC_SDA_IN(void);
void IIC_SDA_OUT(void);
void IIC_Init(void);
void IIC_Start(void); //����IIC��ʼ�ź�
void IIC_Stop(void); //����IICֹͣ�ź�
void IIC_Send_Byte(uint8_t data); //IIC����һ���ֽ�
uint8_t IIC_Read_Byte(uint8_t ack); //IIC��ȡһ���ֽ�
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
int8_t IIC_Wait_Ack(void); //IIC�ȴ�ACK�ź�
void IIC_Ack(void); //IIC����ACK�ź�
void IIC_NAck(void); //IIC����NACK�ź�
 
#endif /*__BSP_I2C_H */

//------------------End of File----------------------------
