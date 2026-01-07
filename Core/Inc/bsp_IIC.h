/*
 * bsp_IIC.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_IIC_H_
#define SRC_BSP_IIC_H_
#include "main.h"

//-----------------IIC端口定义----------------

#define SCLK_Clr() HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_RESET)
#define SCLK_Set() HAL_GPIO_WritePin(SCL_GPIO_Port,SCL_Pin,GPIO_PIN_SET)  //SCL

#define SDIN_Clr() HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_RESET)//DIN
#define SDIN_Set() HAL_GPIO_WritePin(SDA_GPIO_Port,SDA_Pin,GPIO_PIN_SET)

#define IIC_READ_SDA    HAL_GPIO_ReadPin(SDA_GPIO_Port, SDA_Pin)

void I2C_Start(void);
void I2C_Stop(void);
void Send_Byte(uint8_t dat);
uint8_t I2C_ReceiveByte(void);
void I2C_send_ac(uint8_t What);
void I2C_WaitAck(void) ;

#endif /* SRC_BSP_IIC_H_ */
