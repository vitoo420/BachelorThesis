/*
 * mh_z16.h
 *
 *  Created on: Nov 17, 2021
 *      Author: vitja
 */

#ifndef APPLICATION_USER_CORE_MH_Z16_H_
#define APPLICATION_USER_CORE_MH_Z16_H_
#include "main.h"

uint16_t MHZ16_Read(void);
char getCheckSum(uint8_t *packet);

#endif /* APPLICATION_USER_CORE_MH_Z16_H_ */
