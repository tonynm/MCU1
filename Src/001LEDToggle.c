/*
 * 001LEDToggle.c
 *
 *  Created on: Mar 5, 2024
 *      Author: tonyi
 */
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 450000; i++);
}

int main(void)
{
	GPIO_Handle_t GpioLed;

	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_GEN_PURP_OUTPUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_HS;
	GpioLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GpioLed.pGPIOx, ENABLE);

	GPIO_Init(&GpioLed);

	while(1)
	{
		GPIO_ToggleOutputPin(GpioLed.pGPIOx, GpioLed.GPIO_PinConfig.GPIO_PinNumber);
		delay();

		//GPIO_WriteToOutputPin(GPIOD, 12, ENABLE);
	}

	return 0;
}
