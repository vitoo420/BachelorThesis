/*
 * mh_z16.c
 *
 *  Created on: Nov 17, 2021
 *      Author: vitja
 */
#include "mh_z16.h"
#include "retarget.h"

char getCheckSum(uint8_t*);

extern UART_HandleTypeDef huart1;

uint16_t MHZ16_Read(void)
{
	uint8_t TxMessage[9] = {0xFF, 0x01, 0x86, 0x00, 0x00, 0x00, 0x00, 0x00, 0x79};
	uint8_t RxBuffer[9];
	
	HAL_UART_MspInit(&huart1);
	__HAL_UART_DISABLE(&huart1);
	__HAL_UART_ENABLE(&huart1);
	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_SET);
	if(HAL_UART_Transmit(&huart1, TxMessage, sizeof(TxMessage), 1000) != HAL_OK)
	{
		Error_Handler();
	}

	if(HAL_UART_Receive(&huart1, RxBuffer, sizeof(RxBuffer), 1000) != HAL_OK)
	{
		Error_Handler();
	}

	HAL_GPIO_WritePin(LED1_GPIO_Port, LED1_Pin, GPIO_PIN_RESET);
	if (getCheckSum(RxBuffer) == RxBuffer[sizeof(RxBuffer) - 1])
	{
		uint16_t ret = RxBuffer[2] * 256 + RxBuffer[3];
		ret++;
		return ret;
	}

	else return 0;
}

char getCheckSum(uint8_t *packet)
{
	int i;
	uint8_t checksum = 0;

	for( i = 1; i < 8; i++)
	{
		checksum += packet[i];
	}
	checksum = 0xff - checksum;
	checksum += 1;
	return checksum;
}


