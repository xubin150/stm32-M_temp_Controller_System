/*
 * bsp_AT24C02.c
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */
#include "bsp_AT24C02.h"


void AT24_WriteByte(uint8_t WordAddress, uint8_t Byte)
{
    I2C_Start();
    Send_Byte(ADDR);
    I2C_WaitAck();

    Send_Byte(WordAddress);
    I2C_WaitAck();

    Send_Byte(Byte);
    I2C_WaitAck();

    I2C_Stop();
    osDelay(5);// EEPROM 写入需要 5ms
  //  delay_us(5000);
 //   HAL_Delay(5);   // EEPROM 写入需要 5ms
}
uint8_t AT24_ReadByte(uint8_t address)
{
	uint8_t data ;

    I2C_Start();
    Send_Byte(ADDR);
    I2C_WaitAck();

    Send_Byte(address);
    I2C_WaitAck();

    I2C_Start();
    Send_Byte(ADDR |0x01);
    I2C_WaitAck();
    data =I2C_ReceiveByte();

    I2C_send_ac(1);
    I2C_Stop();
    return data ;

}

