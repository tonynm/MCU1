/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: Feb 26, 2024
 *      Author: tonyi
 */

#include "stm32f407xx_gpio_driver.h"

/*
 * Init and Deinit
 */

/***********************************************************
 * @fn				- GPIO_Init
 *
 * @brief			- Configures the settings the GPIO Port of the provided base address
 *
 * @param[in]		- Base address of the given GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	uint32_t temp;

	/* 1. Configure the Pin Mode for the GPIO Pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG){ // For any non-interupt GPIO Modes
		temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
		//temp = 16777216;
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= temp;
	}
	else{ // If the GPIO Port is going be used as an interrupt
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT){
			// 1. Configure the FTSR
			EXTI->FTSR |= (0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			// Clear the corresponding RTSR bit
			EXTI->RTSR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT){
			// 1. Configure the RTSR
			EXTI->RTSR |= (0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

			// Clear the corresponding FTSR bit
			EXTI->FTSR &= ~(0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}

		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FRT){
			// 1. Configure the FTSR and RTSR
			EXTI->FTSR |= (0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			EXTI->RTSR |= (0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		}

		// 2. Configure the GPIO port selection in SYSCFG_EXTICR
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4;
		uint8_t temp3 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4;
		uint8_t port = retPort(pGPIOHandle->pGPIOx);
		SYSCFG_PCLK_EN();
		SYSCFG->EXTICR[temp2] = port << (temp3*4);

		// 3. Enable the EXTI interrupt delivery using IMR (interrupt mask register)
		EXTI->IMR |= (0x1 << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	}
	temp = 0;

	/* 2. Configure the Speed for the GPIO Pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= temp;
	temp = 0;

	/* 3. Configure the PuPd settings for the GPIO Pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinPuPdControl << (pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber * 2));
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= temp;
	temp = 0;

	/* 4. Configure the Output Type for the GPIO Pin */
	temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |= temp;
	temp = 0;

	/* 5. Configure the Alt. Functionality for the GPIO Pin */
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALT_FUNC){
		uint8_t temp2 = 0;

		temp = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/8;
		temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%8;

		pGPIOHandle->pGPIOx->AFR[temp] &= ~(0x15 << (temp2 * 4));
		pGPIOHandle->pGPIOx->AFR[temp] |= (pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFuncMode << (temp2 * 4));
	}
}

/***********************************************************
 * @fn				- GPIO_DeInit
 *
 * @brief			- Resets the GPIO port with the base address provided
 *
 * @param[in]		- Base address of the given GPIO peripheral
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){
	if(pGPIOx == GPIOA){
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB){
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC){
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD){
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE){
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF){
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG){
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH){
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI){
		GPIOI_REG_RESET();
	}
}

/*
 * Peripheral Clock Control
 */

/***********************************************************
 * @fn				- GPIO_PeriClockControl
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- Base address of the given GPIO peripheral
 * @param[in]		- Enable or Disable macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi){
	if(EnOrDi == ENABLE) // If parameter is 1, then enable Peripheral Clock for specified GPIO port
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_EN();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_EN();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_EN();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_EN();
		}
	}

	else // If parameter is 0, then disable Peripheral Clock for specified GPIO port
	{
		if(pGPIOx == GPIOA){
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx == GPIOB){
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx == GPIOC){
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx == GPIOD){
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx == GPIOE){
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx == GPIOF){
			GPIOF_PCLK_DI();
		}
		else if(pGPIOx == GPIOG){
			GPIOG_PCLK_DI();
		}
		else if(pGPIOx == GPIOH){
			GPIOH_PCLK_DI();
		}
		else if(pGPIOx == GPIOI){
			GPIOI_PCLK_DI();
		}
	}
}

/*
 * Read and Write Data
 */

/***********************************************************
 * @fn				- GPIO_ReadFromInputPin
 *
 * @brief			- Reads the pin state of the given pin number of the given GPIO port
 *
 * @param[in]		- Base address of the given GPIO peripheral
 * @param[in]		- Pin number of the given GPIO Port
 *
 * @return			- Return the 8 bit value of the pin state
 *
 * @Note			- none
 *
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum){						/* Read the pin state at the specified GPIO pin */
	uint8_t value; // Value to return
	value = (uint8_t)((pGPIOx->IDR >> pinNum) & 0x00000001); // Take bits of IDR and right shift by pinNum to get needed bit to least sig. bit.
															 // Mask every bit except least sig. bit and assign value to value variable
	return value;
}

/***********************************************************
 * @fn				- GPIO_ReadFromInputPort
 *
 * @brief			- Reads the value stored in the data register of the given GPIO Port
 *
 * @param[in]		- Base address of the given GPIO peripheral
 *
 * @return			- Return the 16 bit result stored in the data register
 *
 * @Note			- none
 *
 */

uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx){										/* Read the value at the data register of the specified GPIO port */
	uint16_t value; // Value to return
	value = (uint16_t)(pGPIOx->IDR); // Give value of entire port to value variable

	return value;
}

/***********************************************************
 * @fn				- GPIO_WriteToOutputPin
 *
 * @brief			- Writes a value to the specified GPIO Pin
 *
 * @param[in]		- Base address of the given GPIO peripheral
 * @param[in]		- Pin number of the given GPIO port
 * @param[in]		- 8 bit value to be written to the pin
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t val){				/* Write a value to the specified GPIO pin */
	if(val == GPIO_PIN_SET){ // Set bit at GPIO pin number to 1
		pGPIOx->ODR |= (1 << pinNum);
	}
	else{					 // Set bit at GPIO pin number to 0
		pGPIOx->ODR &= ~(1 << pinNum);
	}
}

/***********************************************************
 * @fn				- GPIO_WriteToOutputPort
 *
 * @brief			- Writes a value to the data register of the given GPIO Port
 *
 * @param[in]		- Base address of the given GPIO peripheral
 * @param[in]		- 16 bit value to be written to the data register
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val){							/* Write a value to the data register of the specified GPIO port */
	pGPIOx->ODR = val; // Set whole ODR register to a value
}

/***********************************************************
 * @fn				- GPIO_ToggleOutputPin
 *
 * @brief			- Toggles the bit of the given GPIO Pin of the given GPIO Port
 *
 * @param[in]		- Base address of the given GPIO peripheral
 * @param[in]		- Pin number of the given GPIO Port
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum){							/* Toggle a specified GPIO pin */
	pGPIOx->ODR ^= (1 << pinNum);
}

/*
 * IRQ Configuration and ISR Handling
 */

/***********************************************************
 * @fn				- GPIO_IRQInterruptConfig
 *
 * @brief			- Enable or disable the given IRQ Number
 *
 * @param[in]		- The IRQ number that needs to be enabled or disabled
 * @param[in]		- Enable or Disable macros
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi){
	if(EnOrDi == ENABLE){
		if(IRQNum <= 31){ // IRQ Number 0 to 31
			// program ISER0 register
			*NVIC_ISER0 |= (1 << IRQNum);
		}
		else if(IRQNum >= 32 && IRQNum <= 63){ // IRQ Number 32 to 63
			// program ISER1 register
			*NVIC_ISER1 |= (1 << (IRQNum % 32));
		}
		else if(IRQNum >= 64 && IRQNum <= 95){ // IRQ Number 64 to 95
			// program ISER2 register
			*NVIC_ISER2 |= (1 << (IRQNum % 64));
		}
	}
	else{
		if(IRQNum <= 31){ // IRQ Number 0 to 31
			// program ICER0 register
			*NVIC_ICER0 |= (1 << IRQNum);
		}
		else if(IRQNum >= 32 && IRQNum <= 63){ // IRQ Number 32 to 63
			// program ICER1 register
			*NVIC_ICER1 |= (1 << (IRQNum % 32));
		}
		else if(IRQNum >= 64 && IRQNum <= 95){ // IRQ Number 64 to 95
			// program ICER2 register
			*NVIC_ICER2 |= (1 << (IRQNum % 64));
		}
	}
}

/***********************************************************
 * @fn				- GPIO_IRQPriorityConfig
 *
 * @brief			- This function enables or disables peripheral clock for the given GPIO port
 *
 * @param[in]		- The IRQ Number
 * @param[in]		- The priority value that needs to be set to the given IRQ Number
 *
 * @return			- none
 *
 * @Note			- none
 *
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority){
	uint8_t iprx = IRQNum / 4; // 1. Find out what ipr register is needed
	uint8_t iprx_section = IRQNum % 4; // 2. Find out what section within the register is needed

	// Find out how much offset is needed within the section of the register
	uint8_t shift_amount = (8 * iprx_section) + (8 - NUM_PR_BITS_IMPLEMENTED);  /* The calculation of the shift amount
																				 * is the starting bit (LSB)
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * of the section added by the number of bits that
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 * are used within the section
	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 	 */

	*(NVIC_IPR_BASE_ADDR + iprx) |= (IRQPriority << shift_amount); // We add iprx*4 to the base addr because the addresses incremented by 4 bytes
}

/***********************************************************
 * @fn				- GPIO_IRQHandling
 *
 * @brief			- This function handles the IRQ response when the IRQ Handler is triggered in the main file
 *
 * @param[in]		- The Pin Number that is used as an input to take in the interrupt response
 *
 * @return			- none
 *
 * @Note			- none
 *
 */


void GPIO_IRQHandling(uint8_t PinNum){
	// clear the EXTI_PR (EXTI Pending Register) register corresponding to the pin number
	if(EXTI->PR & (1 << PinNum)){ // If the bit at the bit position that corresponds to the pin number is 1, then there is a pending interrupt event signal
		// Clear the register
		EXTI->PR |= (1 << PinNum);
	}
}
