/*
 * switch.c
 *
 *  Created on: Nov 4, 2025
 *      Author: raito
 */
#include "switch.h"

uint16_t getSwitchStatus(uint8_t position)
{

	uint16_t ret = 0;

	if(position == 'R' && HAL_GPIO_ReadPin(GPIOC,GPIO_PIN_10) == 0){                      //sw3
		ret = 1;
	}
	else if (position == 'L' && HAL_GPIO_ReadPin(GPIOA,GPIO_PIN_15) == 0){                 //sw2
		ret = 1;
	}

	return ret;

}

