/*
 * bsp_max31855_driver.h
 *
 *  Created on: Jul 17, 2025
 *      Author: Administrator
 */

#ifndef SRC_BSP_MAX31855_DRIVER_H_
#define SRC_BSP_MAX31855_DRIVER_H_
#include "main.h"
#include "gpio.h"

/*************************************************************************************************/
/**
 *  temperature sensing MAX31855 (K type thermocouuple)
 * software SPI
 * */

#define SPI_CS_HIGH() HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_SET) //CS high
#define SPI_CS_LOW() HAL_GPIO_WritePin(SPI_CS_GPIO_Port,SPI_CS_Pin,GPIO_PIN_RESET) //CS low

#define SPI_SCK_HIGH() HAL_GPIO_WritePin(SPI_SCK_GPIO_Port,SPI_SCK_Pin,GPIO_PIN_SET) //sck high
#define SPI_SCK_LOW() HAL_GPIO_WritePin(SPI_SCK_GPIO_Port,SPI_SCK_Pin,GPIO_PIN_RESET) //sck low

#define READ_SPI_MISO() HAL_GPIO_ReadPin(SPI_MISO_GPIO_Port,SPI_MISO_Pin) //read data
/*************************************************************************************************/


typedef struct {
	uint8_t fault;
	uint8_t ocFault;
	uint8_t scgFault;
	uint8_t scvFault;
	int32_t intTemp;
	uint8_t intTempSign;
	int32_t extTemp;
	uint8_t extTempSign;
} MAX31855_StateHandle;

extern MAX31855_StateHandle MAX31855_Handle;

uint8_t MAX31855_GetFault(MAX31855_StateHandle *MAX31855);
float MAX31855_GetTemperature(MAX31855_StateHandle *MAX31855);
float MAX31855_GeInternalTemperature(MAX31855_StateHandle *MAX31855);
void MAX31855_ReadData(MAX31855_StateHandle *MAX31855);
void MAX31855_SetNSSState(MAX31855_StateHandle *MAX31855, GPIO_PinState state);
float MAX31855_GetTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855);
float MAX31855_GeInternalTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855);
#endif /* SRC_BSP_MAX31855_DRIVER_H_ */
