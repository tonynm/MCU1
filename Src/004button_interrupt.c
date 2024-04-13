/*
 * 004button_interrupt.c
 *
 *  Created on: Apr 1, 2024
 *      Author: tonyi
 */

#include <string.h>
#include "stm32f407xx.h"

void delay(void)
{
	for(uint32_t i = 0; i < 450000/2; i++);
}

int main(void)
{
	// Instantiate Handle Structs for the LED and Button
	GPIO_Handle_t GPIOLed;
	GPIO_Handle_t GPIOBtn;

	// Initialize data members in the structs to 0
	memset(&GPIOLed, 0, sizeof(GPIOLed));
	memset(&GPIOBtn, 0, sizeof(GPIOBtn));

	// Adjust the settings for the LED port
	GPIOLed.pGPIOx = GPIOD;
	GPIOLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_8;
	GPIOLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_GEN_PURP_OUTPUT;
	GPIOLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OP_TYPE_PP;
	GPIOLed.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_HS;
	GPIOLed.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_NO_PUPD;

	GPIO_PeriClockControl(GPIOLed.pGPIOx, ENABLE); // Enable the Peripheral Clock for the LED (Port D, Pin Number 12)
 	GPIO_Init(&GPIOLed); // Initialize the settings for the LED

	// Adjust the settings for the Button port
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NUM_6;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinSpeed = GPIO_SPD_HS;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPuPdControl = GPIO_PIN_PU;

	GPIO_PeriClockControl(GPIOBtn.pGPIOx, ENABLE); // Enable the Peripheral Clock for the Button (Port D, Pin Number 6)
	GPIO_Init(&GPIOBtn); // Initialize the settings for the Button

	// IRQ Configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5, NVIC_IRQ_PRIO15); // Set priority for IRQ Numbers from 5-9 to 15
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5, ENABLE); // Enable IRQ Number 5

	while(1);
}

void EXTI9_5_IRQHandler(void){ // ISR to handle the toggling of the LED when the button is pressed
	delay(); // Debouncing for the button
	GPIO_IRQHandling(GPIO_PIN_NUM_6);
	GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NUM_8);
}
