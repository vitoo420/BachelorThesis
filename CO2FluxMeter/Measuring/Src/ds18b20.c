/*
 * ds18b20.c
 *
 *  Created on: Dec 5, 2021
 *      Author: vitja
 *
 *
 */

#include "ds18b20.h"

uint8_t DS18B20_Init (void);
void DS18B20_Write (uint8_t data);
uint8_t DS18B20_Read (void);

void Set_Pin_Output (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_OUTPUT_PP;
	GPIO_InitStruct.Speed = GPIO_SPEED_FREQ_LOW;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

void Set_Pin_Input (GPIO_TypeDef *GPIOx, uint16_t GPIO_Pin)
{
	GPIO_InitTypeDef GPIO_InitStruct = {0};
	GPIO_InitStruct.Pin = GPIO_Pin;
	GPIO_InitStruct.Mode = GPIO_MODE_INPUT;
	GPIO_InitStruct.Pull = GPIO_PULLUP;
	HAL_GPIO_Init(GPIOx, &GPIO_InitStruct);
}

uint8_t DS18B20_Init (void)
{
	uint8_t Response = 0;
	Set_Pin_Output(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);
	HAL_GPIO_WritePin(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin, 0);
	delayus(480);

	Set_Pin_Input(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);
	delayus (60);

	if(!HAL_GPIO_ReadPin (TEMPERATURE_GPIO_Port, TEMPERATURE_Pin)) Response = 1;
	else Response = 0;

	delayus(420);

	return Response;
}

void DS18B20_Write (uint8_t data)
{
	Set_Pin_Output(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);  // set as output

	for (int i=0; i<8; i++)
	{

		if ((data & (1<<i))!=0)  // if the bit is high
		{
			// write 1

			Set_Pin_Output(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);  // set as output
			HAL_GPIO_WritePin (TEMPERATURE_GPIO_Port, TEMPERATURE_Pin, 0);  // pull the pin LOW
			delayus(1);  // wait for 1 us

			Set_Pin_Input(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);  // set as input
			delayus(50);  // wait for 60 us
		}

		else  // if the bit is low
		{
			// write 0

			Set_Pin_Output(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);
			HAL_GPIO_WritePin (TEMPERATURE_GPIO_Port, TEMPERATURE_Pin, 0);  // pull the pin LOW
			delayus(50);  // wait for 60 us

			Set_Pin_Input(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);
		}
	}
}

uint8_t DS18B20_Read (void)
{
	uint8_t value=0;

	//Set_Pin_Input(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);

	for(int i = 0; i < 8; i++)
	{
		Set_Pin_Output(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);

		HAL_GPIO_WritePin(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin, 0);
		delayus(2);

		Set_Pin_Input(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin);
		if (HAL_GPIO_ReadPin(TEMPERATURE_GPIO_Port, TEMPERATURE_Pin))
		{
			value |= 1<<i;
		}
		delayus (58);
	}
	return value;
}

float DS18B20_ReadTemperature()
{
	uint8_t Presence;
	uint8_t Temp_byte1;
	uint8_t Temp_byte2;
	uint16_t TEMP;

	Presence = DS18B20_Init();
	HAL_Delay (1);
	DS18B20_Write (0xCC);  // skip ROM
	DS18B20_Write (0x44);  // convert t
	HAL_Delay (800);

	Presence = DS18B20_Init();
	HAL_Delay(1);
	DS18B20_Write(0xCC);  // skip ROM
	DS18B20_Write(0xBE);  // Read Scratch-pad

	Temp_byte1 = DS18B20_Read();
	Temp_byte2 = DS18B20_Read();
	TEMP = (Temp_byte2<<8)|Temp_byte1;

	return (float)TEMP/16;
}
