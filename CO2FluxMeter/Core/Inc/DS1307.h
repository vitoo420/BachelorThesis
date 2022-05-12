/*
 * DS1307.h
 *
 *  Created on: Apr 19, 2022
 *      Author: vitja
 */
#include "stm32wlxx_hal.h"

#ifndef INC_DS1307_H_
#define INC_DS1307_H_

#endif /* INC_DS1307_H_ */


typedef struct
{
	uint8_t seconds;
	uint8_t minutes;
	uint8_t hours;
	uint8_t dayofweak;
	uint8_t dayofmonth;
	uint8_t month;
	uint8_t year;
} TIME;

TIME s_time;

void setTime(uint8_t sec, uint8_t min, uint8_t hour, uint8_t dow, uint8_t dom, uint8_t month, uint8_t year);
void getTime(void);
