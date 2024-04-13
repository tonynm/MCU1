/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: Feb 26, 2024
 *      Author: tonyi
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_

#include "stm32f407xx.h"

/*
 * This is a configuration structure for a GPIO Pin
 */

typedef struct{
	uint8_t	GPIO_PinNumber;																/* GPIO Pin Number */
	uint8_t GPIO_PinMode;																/* GPIO Pin Mode */
	uint8_t GPIO_PinSpeed;																/* GPIO Pin Speed */
	uint8_t	GPIO_PinPuPdControl; 														/* GPIO Pull-up & Pull-down Resistor Control */
	uint8_t GPIO_PinOPType; 															/* GPIO Output Type */
	uint8_t GPIO_PinAltFuncMode; 														/* GPIO Alternate Function Mode */
}GPIO_PinConfig_t;


/*
 * This is a handle structure for a GPIO pin
 * */

typedef struct{
	GPIO_RegDef_t *pGPIOx;
	GPIO_PinConfig_t GPIO_PinConfig;
}GPIO_Handle_t;

/* GPIO Pin Numbers */

#define GPIO_PIN_NUM_0																	0
#define GPIO_PIN_NUM_1																	1
#define GPIO_PIN_NUM_2																	2
#define GPIO_PIN_NUM_3																	3
#define GPIO_PIN_NUM_4																	4
#define GPIO_PIN_NUM_5																	5
#define GPIO_PIN_NUM_6																	6
#define GPIO_PIN_NUM_7																	7
#define GPIO_PIN_NUM_8																	8
#define GPIO_PIN_NUM_9																	9
#define GPIO_PIN_NUM_10																	10
#define GPIO_PIN_NUM_11																	11
#define GPIO_PIN_NUM_12																	12
#define GPIO_PIN_NUM_13																	13
#define GPIO_PIN_NUM_14																	14
#define GPIO_PIN_NUM_15																	15

/* GPIO Pin Modes */

#define GPIO_MODE_INPUT																	0x0						// Mode: Input, Value: 0
#define GPIO_MODE_GEN_PURP_OUTPUT														0x1						// Mode: Gen. Purpose Output, Value: 1
#define GPIO_MODE_ALT_FUNC																0x2						// Mode: Alt. Function, Value: 2
#define GPIO_MODE_ANALOG																0x3						// Mode: Analog, Value: 3
#define GPIO_MODE_IT_FT																	0x4						// Mode: Interrupt Falling Edge, Value: 4
#define GPIO_MODE_IT_RT																	0x5						// Mode: Interrupt Rising Edge, Value: 5
#define GPIO_MODE_IT_FRT																0x6						// Mode: Interrupt Falling Rising Edge, Value: 6

/* GPIO Pin Output Types */

#define GPIO_OP_TYPE_PP																	0x0						// Output Type: Push Pull, Value: 0
#define GPIO_OP_TYPE_OD																	0x1						// Output Type: Open Drain, Value: 1

/* GPIO Pin Output Speed */

#define GPIO_SPD_LS																		0x0						// Speed: Low, Value: 0
#define GPIO_SPD_MS																		0x1						// Speed: Medium, Value: 1
#define GPIO_SPD_HS																		0x2						// Speed: High, Value: 2
#define GPIO_SPD_VHS																	0x3						// Speed: Very High, Value: 3

/* GPIO Pin Pull Up Pull Down Config Macros */

#define GPIO_NO_PUPD																	0x0						// No Pullup nor Pulldown
#define GPIO_PIN_PU																		0x1						// Enable Pullup Resistor
#define GPIO_PIN_PD																		0x2						// Enable Pulldown Resistor

/* GPIO Pin Alt. Function Config Macros */

#define GPIO_ALT_FUNC_AF0																0x0
#define GPIO_ALT_FUNC_AF1																0x1
#define GPIO_ALT_FUNC_AF2																0x2
#define GPIO_ALT_FUNC_AF3																0x3
#define GPIO_ALT_FUNC_AF4																0x4
#define GPIO_ALT_FUNC_AF5																0x5
#define GPIO_ALT_FUNC_AF6																0x6
#define GPIO_ALT_FUNC_AF7																0x7
#define GPIO_ALT_FUNC_AF8																0x8
#define GPIO_ALT_FUNC_AF9																0x9
#define GPIO_ALT_FUNC_AF10																0x10
#define GPIO_ALT_FUNC_AF11																0x11
#define GPIO_ALT_FUNC_AF12																0x12
#define GPIO_ALT_FUNC_AF13																0x13
#define GPIO_ALT_FUNC_AF14																0x14
#define GPIO_ALT_FUNC_AF15																0x15

/***********************************
 * APIs supported by this driver
 ***********************************/

/*
 * Init and Deinit
 */

void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);

/*
 * Peripheral Clock Control
 */

void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t EnOrDi);

/*
 * Read and Write Data
 */

uint8_t GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);						/* Read the pin state at the specified GPIO pin */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);										/* Read the value at the data register of the specified GPIO port */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum, uint8_t val);				/* Write a value to the specified GPIO pin */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, uint16_t val);							/* Write a value to the data register of the specified GPIO port */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, uint8_t pinNum);							/* Toggle a specified GPIO pin */

/*
 * IRQ Configuration and ISR Handling
 */

void GPIO_IRQInterruptConfig(uint8_t IRQNum, uint8_t EnOrDi);
void GPIO_IRQPriorityConfig(uint8_t IRQNum, uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNum);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
