/*
 * bsp_AT24C02.h
 *
 *  Created on: Dec 2, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_AT24C02_H_
#define SRC_BSP_AT24C02_H_
#include "main.h"
#include "bsp_IIC.h"

#define ADDR 0xA0
void AT24_WriteByte(uint8_t WordAddress, uint8_t Byte);
uint8_t AT24_ReadByte(uint8_t address);

#endif /* SRC_BSP_AT24C02_H_ */
