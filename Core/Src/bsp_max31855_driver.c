/*
 * bsp_max31855_driver.c
 *
 *  Created on: Jul 17, 2025
 *      Author: Administrator
 */
#include "bsp_max31855_driver.h"
#define MAX31855_DATA_BIT_NUM 32 //32 bits of data in total
MAX31855_StateHandle MAX31855_Handle;


uint8_t MAX31855_GetFault(MAX31855_StateHandle *MAX31855)
{
	if(MAX31855->ocFault) // open circuit fault
	{
		return 1;
	}
	else if(MAX31855->scgFault) // short to gnd fault
	{
		return 2;
	}
	else if(MAX31855->scvFault)// short to vcc fault
	{
		return 3;
	}
	else
	{
		return 0;
	}

}
float MAX31855_GetTemperature(MAX31855_StateHandle *MAX31855)
{
	 return MAX31855->extTemp*0.25;
}
float MAX31855_GeInternalTemperature(MAX31855_StateHandle *MAX31855){
	 return MAX31855->intTemp*0.0625;
}

float MAX31855_GetTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855)
{
 	 float temp=MAX31855_GetTemperature(MAX31855);
 	 temp*=1.8;
 	 temp += 32;
 	 return temp;
}
float MAX31855_GeInternalTemperatureInFahrenheit(MAX31855_StateHandle *MAX31855){
	 float temp=MAX31855_GeInternalTemperature(MAX31855);
	 temp*=1.8;
	 temp += 32;
	 return temp;
}

void MAX31855_ReadData(MAX31855_StateHandle *MAX31855)
{
	uint32_t temp_reg=0;
	SPI_CS_LOW();
	SPI_SCK_LOW();
	for(uint8_t i=0;i<MAX31855_DATA_BIT_NUM;i++)
	{
		temp_reg<<=1;
		SPI_SCK_HIGH();
		delay_us(1);
		if(READ_SPI_MISO()==GPIO_PIN_SET) //HIGH level
		{
			temp_reg |=1;
		}
		SPI_SCK_LOW();
		delay_us(1);
	}
	SPI_CS_HIGH();

	//MAX31855_SetNSSState(MAX31855,GPIO_PIN_SET);
	MAX31855->scvFault=0;
	MAX31855->scgFault=0;
	MAX31855->ocFault=0;
	MAX31855->fault=0;
	MAX31855->extTemp=0;
	MAX31855->extTempSign=0;
	MAX31855->intTemp=0;
	MAX31855->intTempSign=0;

	int32_t extTemp=temp_reg;
	int32_t intTemp=temp_reg;

	if(temp_reg& 0b00000000000000000000000000000100)
	{
		MAX31855->scvFault=1;
	}
	if(temp_reg& 0b00000000000000000000000000000010)
	{
		MAX31855->scgFault=1;
	}
	if(temp_reg& 0b00000000000000000000000000000001)
	{
		MAX31855->ocFault=1;
	}
	if(temp_reg&0b00000000000000010000000000000000)
	{
		MAX31855->fault=1;
	}
	if(temp_reg&0b10000000000000000000000000000000)
	{
		MAX31855->extTempSign=1;
	}
	if(temp_reg&0b00000000000000001000000000000000)
	{
		MAX31855->intTempSign=1;
	}

	extTemp = (temp_reg >> 18) & 0x3FFF; // 14-bit
	if (extTemp & 0x2000) { // bit13 = sign
		extTemp |= 0xFFFFC000; // 扩展到32-bit
		//extTemp = ~(extTemp & 0b11111111111111);
	}
	MAX31855->extTemp=extTemp;

	intTemp = (temp_reg>>4) & 0x0fff; // 取内部温度原始 12bit 数据
	if (intTemp & 0x800)   // bit11 = 1
	{
		intTemp |= 0xFFFFF000;  // 将高 20bit 全部补 1
		//intTemp=~(intTemp|0b1111100000000000); //Experimental code, not tested!
	}
	MAX31855->intTemp=intTemp;
}


