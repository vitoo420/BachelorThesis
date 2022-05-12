/*
 * DS1307.c
 *
 *  Created on: Apr 19, 2022
 *      Author: vitja
 */

#include "stdint.h"
#include "DS1307.h"
#include "main.h"
#define DS1307_ADDRESS 0x68<<1	//adresa je 0x68, ale je na prvnich 7 bitech, proto ji musim bitove posunout na MSB, zapisu tedy 0xD0

extern I2C_HandleTypeDef hi2c3;

uint8_t decToBcd(int val)
{
	return (uint8_t)((val/10*16) + (val%10));
}

uint8_t bcdToDec(uint8_t val)
{
	return (int)((val/16*10) + (val%16));
}

void setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year)
{
	uint8_t set_time[7];
	set_time[0] = decToBcd(sec);
	set_time[1] = decToBcd(min);
	set_time[2] = decToBcd(hour);
	set_time[3] = decToBcd(dow);
	set_time[4] = decToBcd(dom);
	set_time[5] = decToBcd(month);
	set_time[6] = decToBcd(year);
	HAL_I2C_Mem_Write(&hi2c3, DS1307_ADDRESS, 0x00, 1, set_time, 7, 1000);
}

void getTime(void)
{
	uint8_t get_time[7];
	HAL_I2C_Mem_Read(&hi2c3, DS1307_ADDRESS, 0x00, 1, get_time, 7, 1000);
	s_time.seconds = bcdToDec(get_time[0]);
	s_time.minutes = bcdToDec(get_time[1]);
	s_time.hours = bcdToDec(get_time[2]);
	s_time.dayofweak = bcdToDec(get_time[3]);
	s_time.dayofmonth = bcdToDec(get_time[4]);
	s_time.month = bcdToDec(get_time[5]);
	s_time.year = bcdToDec(get_time[6]);
}




