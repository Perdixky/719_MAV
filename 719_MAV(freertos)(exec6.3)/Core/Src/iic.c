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
	IIC_SDA_OUT(); //PB11输出模式
}
 
/*
 * @brief IIC_SDA输入模式
 */
void IIC_SDA_IN(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IIC_SDA_PIN;                      //IIC的SDA引脚
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;                 //输入模式
	GPIO_InitStruct.Pull = GPIO_PULLUP;                     //上拉
	HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct);   			//设置SDA口
}
/*
 * @brief IIC_SDA输出模式
 */
void IIC_SDA_OUT(void)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = IIC_SDA_PIN;                     //IIC的SDA引脚
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_OD;            //开漏输出模式
	GPIO_InitStruct.Pull = GPIO_PULLUP;                    //上拉
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_HIGH;          //SDA口速度
	HAL_GPIO_Init(IIC_GPIO_PORT, &GPIO_InitStruct); 			 //设置SDA口
}
/*
 * @brief IIC起始信号
 */
void IIC_Start(void)
{
	IIC_SDA_OUT();
	IIC_SDA(GPIO_PIN_SET); //先拉高SDA再拉高SCL，防止出现错误信号
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2); //延时保证SCL高电平时，SDA下降沿
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //钳住I2C总线，准备发送或接收数据
}
 
/*
 * @brief IIC停止信号
 */
void IIC_Stop(void)
{
	//IIC_SCL(GPIO_PIN_RESET); //phph2045
	IIC_SDA_OUT(); //phph2045
	IIC_SDA(GPIO_PIN_RESET); //先拉低SDA再拉高SCL，防止出现错误信号
	delay_us(2); //延时保证SCL高电平时，SDA上升沿 //phph2045
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2); //延时保证SCL高电平时，SDA上升沿
	IIC_SDA(GPIO_PIN_SET);
	//这里就不用拉低SCL了，因为IIC通讯已经结束
	//IIC_SCL(GPIO_PIN_RESET); //钳住I2C总线，准备发送或接收数据
}
 
/*
 * @brief IIC主机等待应答
 * 返回值：0，接收应答成功
 *         -1，接收应答失败
 */
int8_t IIC_Wait_Ack(void)
{
	uint8_t ucErrTime = 0;
 
	IIC_SDA(GPIO_PIN_SET); //一定要先拉高SDA，再拉高SCL，否则就可能变成起始信号和结束信号了
	delay_us(1);
	IIC_SDA_IN(); //SDA设置为输入
	delay_us(1); //phph2045
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	while(IIC_Read_SDA())
	{
		ucErrTime++;
		if(ucErrTime > 250)
		{
			IIC_Stop();
			IIC_SDA_OUT(); //SDA恢复为输出
			return 1;
		}
	}
	IIC_SCL(GPIO_PIN_RESET); //IIC通讯没结束，拉低SCL线
	IIC_SDA_OUT(); //SDA恢复为输出
 
	return 0;
}
 
//产生ACK应答
void IIC_Ack(void)
{
	//IIC_SCL(GPIO_PIN_RESET); //phph2047
	IIC_SDA_OUT(); //SDA设置为输出
	//IIC_SCL(GPIO_PIN_RESET); //phph2046
	IIC_SDA(GPIO_PIN_RESET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //IIC通讯没结束，拉低SCL线
}
 
//产生NACK应答
void IIC_NAck(void)
{
	IIC_SCL(GPIO_PIN_RESET); //phph2045 
	IIC_SDA_OUT(); //SDA设置为输出
	//IIC_SCL(GPIO_PIN_RESET); //phph2046
	IIC_SDA(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_SET);
	delay_us(2);
	IIC_SCL(GPIO_PIN_RESET); //IIC通讯没结束，拉低SCL线
}
 
//读1个字节，ack=1时，发送ACk；ack=0，发送NACK
uint8_t IIC_Read_Byte(uint8_t ack)
{
	uint8_t i = 0;
	uint8_t recv = 0;
 
	IIC_SDA_IN(); //SDA设置为输入
 
	for(i=0; i<8; i++)
	{
		IIC_SCL(GPIO_PIN_RESET); //拉低时钟线
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET); //拉高时钟线，接收1位数据
		recv <<= 1;
		//phph2045 delay_us(2);
		if(IIC_Read_SDA())
			recv |= 0x01;
		delay_us(2);
	}
	IIC_SCL(GPIO_PIN_RESET); //phph2047
 
	if(!ack)
		IIC_NAck(); //发送NACK
	else
		IIC_Ack(); //发送ACK
 
	IIC_SDA_OUT(); //SDA恢复为输出
 
	return recv;
}
 
/*
 * @brief IIC发送一个字节
 * @para  待写入的字节数据
 * 返回从机有无应答
 *    1，有应答
 *    0，无应答
 */
void IIC_Send_Byte(uint8_t data)
{
	uint8_t i = 0;
 
	IIC_SDA_OUT();
	//phph2045 IIC_SCL(GPIO_PIN_RESET); //拉低时钟开始数据传输
 
	for(i=0; i<8; i++)
	{
		IIC_SCL(GPIO_PIN_RESET); //拉低时钟线
		//准备数据
		if(data & 0x80)
			IIC_SDA(GPIO_PIN_SET);
		else
			IIC_SDA(GPIO_PIN_RESET);
		data <<= 1;
		delay_us(2);
		IIC_SCL(GPIO_PIN_SET); //拉高时钟线，发送1位数据
		delay_us(2);
	}
	IIC_SCL(GPIO_PIN_RESET); //通讯未结束，拉低SCL
	delay_us(2); //phph2045
}

//IIC连续读
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

//IIC连续写
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
