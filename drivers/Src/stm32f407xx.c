/*
 * stm32f407xx.c
 *
 *  Created on: Mar 18, 2024
 *      Author: tonyi
 */

#include "stm32f407xx.h"

/* Return the Port for Given GPIO Base Address */
uint8_t retPort(GPIO_RegDef_t* pGPIOx){
	if(pGPIOx == GPIOA){
		return 0;
	}
	else if(pGPIOx == GPIOB){
		return 1;
	}
	else if(pGPIOx == GPIOC){
		return 2;
	}
	else if(pGPIOx == GPIOD){
		return 3;
	}
	else if(pGPIOx == GPIOE){
		return 4;
	}
	else if(pGPIOx == GPIOF){
		return 5;
	}
	else if(pGPIOx == GPIOG){
		return 6;
	}
	else if(pGPIOx == GPIOH){
		return 7;
	}
	else if(pGPIOx == GPIOI){
		return 8;
	}
}
