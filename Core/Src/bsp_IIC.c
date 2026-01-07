/*
 * bsp_IIC.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */
#include "bsp_IIC.h"

#define DELAY_TIME 5

//延时
void IIC_delay(void)
{
	//uint8_t t=3;
	/*uint8_t t=10;
	while(t--);*/
	delay_us(10);
}

//起始信号
void I2C_Start(void)
{
	SDIN_Set();
	SCLK_Set();
	IIC_delay();
	SDIN_Clr();
	IIC_delay();
	SCLK_Clr();
	IIC_delay();
}


//结束信号
void I2C_Stop(void)
{
	SDIN_Clr();
	SCLK_Set();
	IIC_delay();
	SDIN_Set();
}


//写入一个字节
void Send_Byte(uint8_t dat)
{
	uint8_t i;
	for(i=0;i<8;i++)
	{
		SCLK_Clr();//将时钟信号设置为低电平
		if(dat&0x80)//将dat的8位从最高位依次写入
		{
			SDIN_Set();
    }
		else
		{
			SDIN_Clr();
    }
		IIC_delay();
		SCLK_Set();
		IIC_delay();
		SCLK_Clr();
		dat<<=1;
  }
}

//等待信号响应
void I2C_WaitAck(void) //测数据信号的电平
{
	uint8_t t=0;
	SDIN_Set();
	IIC_delay();
	SCLK_Set();
	IIC_delay();
	while(IIC_READ_SDA)
	{
		t++;
		if(t>200)
		{
			I2C_Stop();
			return;
		}
	}
	SCLK_Clr();
	IIC_delay();
}
/**
* @brief I2C接收一个字节数据
*
* @param[in] none
* @param[out] da
* @return da - 从I2C总线上接收到得数据
*/
uint8_t I2C_ReceiveByte(void)
{
	uint8_t da=0;
	uint8_t i;
//
	for(i=0;i<8;i++){
		SCLK_Set();
		SDIN_Set();
		IIC_delay();
		da <<= 1;
		if(IIC_READ_SDA)
			da |= 0x01;
		SCLK_Clr();
		IIC_delay();
	}
//
	return da;
}

/**
* @brief 发送应答
*
* @param[in] ackbit - 设定是否发送应答
* @return - none
*/
void I2C_send_ac(uint8_t What)
{
	SCLK_Clr();
	if(What)   SDIN_Set(); //0：发送应答信号；1：发送非应答信号
	else SDIN_Clr();

    IIC_delay();
    SCLK_Set();
    IIC_delay();
    SCLK_Clr();
    SDIN_Set();
    IIC_delay();
}


