#ifndef __BSP_I2C_H
#define __BSP_I2C_H
 
#include "main.h"
 
#define DEVICE_ADDRESS	0xA0 //7位地址
#define BIT_WRITE	0
#define BIT_READ	1
 
//引脚定义
/*
 * SCL——PB6
 * SDA——PB7
 */
//#define IIC_SDA_IN()		{GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=0<<7*2;}//PB7输入模式
//#define IIC_SDA_OUT()	{GPIOB->MODER&=~(3<<(7*2));GPIOB->MODER|=1<<7*2;}//PB7输出模式
 
#define IIC_SCL_PIN	GPIO_PIN_6
#define IIC_SDA_PIN	GPIO_PIN_7
#define IIC_GPIO_PORT	GPIOB
#define IIC_GPIO_CLK_ENABLE()	__HAL_RCC_GPIOB_CLK_ENABLE()
 
//IIC的IO操作函数
//带参宏，可以像内联函数一样使用
#define IIC_SCL(a)		HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SCL_PIN, a)
#define IIC_SDA(a)		HAL_GPIO_WritePin(IIC_GPIO_PORT, IIC_SDA_PIN, a)
#define IIC_Read_SDA()	HAL_GPIO_ReadPin(IIC_GPIO_PORT, IIC_SDA_PIN)
 
void IIC_SDA_IN(void);
void IIC_SDA_OUT(void);
void IIC_Init(void);
void IIC_Start(void); //发送IIC开始信号
void IIC_Stop(void); //发送IIC停止信号
void IIC_Send_Byte(uint8_t data); //IIC发送一个字节
uint8_t IIC_Read_Byte(uint8_t ack); //IIC读取一个字节
int32_t IICreadBytes(uint8_t dev, uint8_t reg, uint8_t *data, uint32_t length);
int32_t IICwriteBytes(uint8_t dev, uint8_t reg, uint8_t* data, uint32_t length);
int8_t IIC_Wait_Ack(void); //IIC等待ACK信号
void IIC_Ack(void); //IIC发送ACK信号
void IIC_NAck(void); //IIC发送NACK信号
 
#endif /*__BSP_I2C_H */

//------------------End of File----------------------------
