/*
 * stm32f407xx.h
 *
 *  Created on: Jan 23, 2024
 *      Author: tonyi
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_

#include <stdint.h>

/***************** Processor Specific Details *****************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Addresses
 */

#define NVIC_ISER0										( (volatile uint32_t*) 0xE000E100 )
#define NVIC_ISER1										( (volatile uint32_t*) 0xE000E104 )
#define NVIC_ISER2										( (volatile uint32_t*) 0xE000E108 )
#define NVIC_ISER3										( (volatile uint32_t*) 0xE000E10C )
#define NVIC_ISER4										( (volatile uint32_t*) 0xE000E110 )
#define NVIC_ISER5										( (volatile uint32_t*) 0xE000E114 )
#define NVIC_ISER6										( (volatile uint32_t*) 0xE000E118 )
#define NVIC_ISER7										( (volatile uint32_t*) 0xE000E11C )

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Addresses
 */

#define NVIC_ICER0										( (volatile uint32_t*) 0XE000E180 )
#define NVIC_ICER1										( (volatile uint32_t*) 0XE000E184 )
#define NVIC_ICER2										( (volatile uint32_t*) 0XE000E188 )
#define NVIC_ICER3										( (volatile uint32_t*) 0XE000E18C )
#define NVIC_ICER4										( (volatile uint32_t*) 0XE000E190 )
#define NVIC_ICER5										( (volatile uint32_t*) 0XE000E194 )
#define NVIC_ICER6										( (volatile uint32_t*) 0XE000E198 )
#define NVIC_ICER7										( (volatile uint32_t*) 0XE000E19C )

/*
 * NVIC IRQ Numbers
 */

#define IRQ_NO_EXTI0									6					/* EXTI0 corresponds to IRQ Number 6 */
#define IRQ_NO_EXTI1									7					/* EXTI1 corresponds to IRQ Number 7 */
#define IRQ_NO_EXTI2									8					/* EXTI2 corresponds to IRQ Number 8 */
#define IRQ_NO_EXTI3									9					/* EXTI3 corresponds to IRQ Number 9 */
#define IRQ_NO_EXTI4									10					/* EXTI4 corresponds to IRQ Number 10 */
#define IRQ_NO_EXTI9_5									23					/* EXTI5 - EXTI9 corresponds to IRQ Number 23 */
#define IRQ_NO_EXTI15_10								40					/* EXTI10 - EXTI15 corresponds to IRQ Number 40 */

/*
 * NVIC Priority Levels
 */

#define NVIC_IRQ_PRIO0									0
#define NVIC_IRQ_PRIO15									15

/*
 * ARM Cortex Mx Processor NVIC IPRx Base Address
 */
#define NVIC_IPR_BASE_ADDR								( (volatile uint32_t*) 0xE000E400 )

#define NUM_PR_BITS_IMPLEMENTED							4 // Number of bits that are used to set the priority of the IRQ Number

/***************** Peripheral Register Structures *****************/

typedef struct{
	volatile uint32_t MODER;									/* !< Description: GPIO Port Mode Register | Offset: 0x00 >*/
	volatile uint32_t OTYPER;									/* !< Description: GPIO Port Output Typer Register | Offset: 0x04 */
	volatile uint32_t OSPEEDR;									/* !< Description: GPIO Port Output Speed Register | Offset: 0x08 */
	volatile uint32_t PUPDR;									/* !< Description: GPIO Port Pull-up/Pull-down Register | Offset: 0x0C */
	volatile uint32_t IDR;										/* !< Description: GPIO Port Input Data Register | Offset: 0x10 */
	volatile uint32_t ODR;										/* !< Description: GPIO Port Output Data Register | Offset: 0x14 */
	volatile uint32_t BSRR;										/* !< Description: GPIO Port Bit Set/Reset Register | Offset: 0x18 */
	volatile uint32_t LCKR;										/* !< Description: GPIO Port Configuration Lock Register | Offset: 0x1C */
	volatile uint32_t AFR[2];									/* !< Description: GPIO Alternate Function Register (AFRL = AFR[0] & AFRH = AFR[1]) | AFRL (AFR[0]) Offset: 0x20 | AFRH (AFR[1]) Offset: 0x24 */
}GPIO_RegDef_t;

typedef struct{
	volatile uint32_t CR;										/* Description: RCC Clock Control Register | Offset: 0x00 */
	volatile uint32_t PLLCFGR;									/* Description: RCC PLL Configuration Register | Offset: 0x04 */
	volatile uint32_t CFGR;										/* Description: RCC Clock Configuration Register | Offset: 0x08 */
	volatile uint32_t CIR;										/* Description: RCC Clock Interrupt Register | Offset: 0x0C */
	volatile uint32_t AHB1RSTR;									/* Description: RCC AHB1 Peripheral Reset Register | Offset: 0x10 */
	volatile uint32_t AHB2RSTR;									/* Description: RCC AHB2 Peripheral Reset Register | Offset: 0x14 */
	volatile uint32_t AHB3RSTR;									/* Description: RCC AHB3 Peripheral Reset Register | Offset: 0x18 */
	uint32_t RESERVED0;											/* RESERVED REGISTER | Offset: 0x1C */
	volatile uint32_t APB1RSTR;									/* Description: RCC APB1 Peripheral Reset Register | Offset: 0x20 */
	volatile uint32_t APB2RSTR;									/* Description: RCC APB2 Peripheral Reset Register | Offset: 0x24 */
	uint32_t RESERVED1[2];										/* RESERVED REGISTER | Offset: 0x28 & Offset: 0x2C */
	volatile uint32_t AHB1ENR;									/* Description: RCC AHB1 Peripheral Clock Enable Register | Offset: 0x30 */
	volatile uint32_t AHB2ENR;									/* Description: RCC AHB2 Peripheral Clock Enable Register | Offset: 0x34 */
	volatile uint32_t AHB3ENR;									/* Description: RCC AHB3 Peripheral Clock Enable Register | Offset: 0x38 */
	uint32_t RESERVED2;											/* RESERVED REGISTER | Offset: 0x3C */
	volatile uint32_t APB1ENR;									/* Description: RCC APB1 Peripheral Clock Enable Register | Offset: 0x40 */
	volatile uint32_t APB2ENR;									/* Description: RCC APB2 Peripheral Clock Enable Register | Offset: 0x44 */
	uint32_t RESERVED3[2];										/* RESERVED REGISTER | Offset: 0x48 & Offset: 0x4C */
	volatile uint32_t AHB1LPENR;								/* Description: RCC AHB1 Peripheral Clock Enable In Low Power Mode Register | Offset: 0x50 */
	volatile uint32_t AHB2LPENR;								/* Description: RCC AHB2 Peripheral Clock Enable In Low Power Mode Register | Offset: 0x54 */
	volatile uint32_t AHB3LPENR;								/* Description: RCC AHB3 Peripheral Clock Enable In Low Power Mode Register | Offset: 0x58 */
	uint32_t RESERVED4;											/* RESERVED REGISTER | Offset: 0x5C */
	volatile uint32_t APB1LPENR;								/* Description: RCC APB1 Peripheral Clock Enable In Low Power Mode Register | Offset: 0x60 */
	volatile uint32_t APB2LPENR;								/* Description: RCC APB2 Peripheral Clock Enable In Low Power Mode Register | Offset: 0x64 */
	uint32_t RESERVED5[2];										/* RESERVED REGISTER | Offset: 0x68 & Offset: 0x6C */
	volatile uint32_t BDCR;										/* Description: RCC Backup Domain Control Register | Offset: 0x70 */
	volatile uint32_t CSR;										/* Description: RCC Clock Control & Status Register | Offset: 0x74 */
	uint32_t RESERVED6[2];										/* RESERVED REGISTER | Offset: 0x78 & Offset: 0x7C */
	volatile uint32_t SSCGR;									/* Description: RCC Spread Spectrum Clock Generation Register | Offset: 0x80 */
	volatile uint32_t PLLI2SCFGR;								/* Description: RCC PLLI2S Configuration Register | Offset: 0x84 */
}RCC_RegDef_t;

typedef struct{
	volatile uint32_t IMR;										/* Description: EXTI Interrupt Mask Register | Offset: 0x00 */
	volatile uint32_t EMR;										/* Description: EXTI Event Mask Register | Offset: 0x04 */
	volatile uint32_t RTSR;										/* Description: EXTI Rising Trigger Selection Register | Offset: 0x08 */
	volatile uint32_t FTSR;										/* Description: EXTI Falling Trigger Selection Register | Offset: 0x0C */
	volatile uint32_t SWIER;									/* Description: EXTI Software Interrupt Event Register | Offset: 0x10 */
	volatile uint32_t PR;										/* Description: EXTI Pending Register | Offset: 0x14 */
}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t MEMRMP;									/* Description: SYSCFG Memory Remap Register | Offset: 0x00 */
	volatile uint32_t PMC;										/* Description: SYSCFG Peripheral Mode Config. Register | Offset: 0x04 */
	volatile uint32_t EXTICR[4];								/* Description: SYSCFG Ext. Interrupt Config. Register
																 * (EXTICR1 = EXTICR[0] & EXTICR2 = EXTICR[1]
																 * & EXTICR3 = EXTICR[2] & EXTICR4 = EXTICR[3])
																 * EXTICR1 (EXTICR[0]) Offset: 0x08 | EXTICR2 (EXTICR[1]) Offset: 0x0C
																 * EXTICR3 (EXTICR[2]) Offset: 0x10 | EXTICR4 (EXTICR[3]) Offset: 0x14 */
	uint32_t RESERVED[2];										/* RESERVED REGISTER | Offset: 0x18 & Offset: 0x1C */
	volatile uint32_t CMPCR;									/* Description: SYSCFG Compensation Cell Control Register | Offset: 0x20 */
}SYSCFG_RegDef_t;

/***************** BASE ADDRESSES FOR FLASH AND SRAM MEMORIES *****************/

#define FLASH_BASEADDR									0x08000000U		/* Base Address for the main (flash) memory */
#define SRAM1_BASEADDR									0x20000000U		/* Base Address for SRAM1 */
#define SRAM2_BASEADDR									0x2001C000U
#define ROM_BASEADDR									0x1FFF0000U		/* Base Address for the system memory (ROM) */
#define SRAM 											SRAM1_BASEADDR

/***************** BASE ADDRESSES FOR AHBx and APBx peripherals *****************/

#define PERIPH_BASEADDR									0x40000000U
#define APB1PERIPH_BASEADDR								PERIPH_BASEADDR
#define APB2PERIPH_BASEADDR								0x40010000U
#define AHB1PERIPH_BASEADDR								0x40020000U
#define AHB2PERIPH_BASEADDR								0x50000000U

/***************** BASE ADDRESSES FOR PERIPHERALS HANGING ON AHB1 BUS *****************/

#define GPIOA_BASEADDR									(AHB1PERIPH_BASEADDR + 0x0000U) // Base Address for GPIO Port A
#define GPIOB_BASEADDR									(AHB1PERIPH_BASEADDR + 0x0400U) // Base Address for GPIO Port B
#define GPIOC_BASEADDR									(AHB1PERIPH_BASEADDR + 0x0800U) // Base Address for GPIO Port C
#define GPIOD_BASEADDR									(AHB1PERIPH_BASEADDR + 0x0C00U) // Base Address for GPIO Port D
#define GPIOE_BASEADDR									(AHB1PERIPH_BASEADDR + 0x1000U) // Base Address for GPIO Port E
#define GPIOF_BASEADDR									(AHB1PERIPH_BASEADDR + 0x1400U) // Base Address for GPIO Port F
#define GPIOG_BASEADDR									(AHB1PERIPH_BASEADDR + 0x1800U) // Base Address for GPIO Port G
#define GPIOH_BASEADDR									(AHB1PERIPH_BASEADDR + 0x1C00U) // Base Address for GPIO Port H
#define GPIOI_BASEADDR									(AHB1PERIPH_BASEADDR + 0x2000U) // Base Address for GPIO Port I

#define RCC_BASEADDR									(AHB1PERIPH_BASEADDR + 0x3800U) // Base Address for RCC

/***************** BASE ADDRESSES FOR PERIPHERALS HANGING ON APB1 BUS *****************/

/* BASE ADDRESSES FOR I2Cx */
#define I2C1_BASEADDR									(APB1PERIPH_BASEADDR + 0x5400U) // Base Address for I2C1
#define I2C2_BASEADDR									(APB1PERIPH_BASEADDR + 0x5800U) // Base Address for I2C2
#define I2C3_BASEADDR									(APB1PERIPH_BASEADDR + 0x5C00U) // Base Address for I2C3

/* BASE ADDRESSES FOR USARTx AND UARTx IN APB1 BUS */
#define USART2_BASEADDR									(APB1PERIPH_BASEADDR + 0x4400U) // Base Address for USART2
#define USART3_BASEADDR									(APB1PERIPH_BASEADDR + 0x4800U) // Base Address for USART3
#define UART4_BASEADDR									(APB1PERIPH_BASEADDR + 0x4C00U) // Base Address for UART4
#define UART5_BASEADDR									(APB1PERIPH_BASEADDR + 0x5000U) // Base Address for UART5

/* BASE ADDRESSES FOR SPIx IN APB1 BUS */
#define SPI2_BASEADDR									(APB1PERIPH_BASEADDR + 0x3800U) // Base Address for SPI2
#define SPI3_BASEADDR									(APB1PERIPH_BASEADDR + 0x3C00U) // Base Address for SPI3

/***************** BASE ADDRESSES FOR PERIPHERALS HANGING ON APB2 BUS *****************/

/* BASE ADDRESSES FOR USARTx in APB2 BUS */
#define USART1_BASEADDR									(APB2PERIPH_BASEADDR + 0x1000U) // Base Address for USART1
#define USART6_BASEADDR									(APB2PERIPH_BASEADDR + 0x1400U) // Base Address for USART6

/* BASE ADDRESS for SPI1 */
#define SPI1_BASEADDR									(APB2PERIPH_BASEADDR + 0x3000U) // Base Address for SPI1

/* BASE ADDRESSES FOR EXTI AND SYSCFG */
#define EXTI_BASEADDR									(APB2PERIPH_BASEADDR + 0x3C00U) // Base Address for EXTI (External Interrupt Controller)
#define SYSCFG_BASEADDR									(APB2PERIPH_BASEADDR + 0x3800U) // Base Address for SYSCFG (System Config)

/***************** Peripheral Definitions *****************/

#define GPIOA											((GPIO_RegDef_t*)GPIOA_BASEADDR)
#define GPIOB											((GPIO_RegDef_t*)GPIOB_BASEADDR)
#define GPIOC											((GPIO_RegDef_t*)GPIOC_BASEADDR)
#define GPIOD											((GPIO_RegDef_t*)GPIOD_BASEADDR)
#define GPIOE											((GPIO_RegDef_t*)GPIOE_BASEADDR)
#define GPIOF											((GPIO_RegDef_t*)GPIOF_BASEADDR)
#define GPIOG											((GPIO_RegDef_t*)GPIOG_BASEADDR)
#define GPIOH											((GPIO_RegDef_t*)GPIOH_BASEADDR)
#define GPIOI											((GPIO_RegDef_t*)GPIOI_BASEADDR)

#define RCC												((RCC_RegDef_t*)RCC_BASEADDR)

#define EXTI											((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SYSCFG											((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)

/***************** Clock Enable Macros for GPIOx Peripherals *****************/

#define GPIOA_PCLK_EN()									( RCC->AHB1ENR |= (1 << 0) )
#define GPIOB_PCLK_EN()									( RCC->AHB1ENR |= (1 << 1) )
#define GPIOC_PCLK_EN()									( RCC->AHB1ENR |= (1 << 2) )
#define GPIOD_PCLK_EN()									( RCC->AHB1ENR |= (1 << 3) )
#define GPIOE_PCLK_EN()									( RCC->AHB1ENR |= (1 << 4) )
#define GPIOF_PCLK_EN()									( RCC->AHB1ENR |= (1 << 5) )
#define GPIOG_PCLK_EN()									( RCC->AHB1ENR |= (1 << 6) )
#define GPIOH_PCLK_EN()									( RCC->AHB1ENR |= (1 << 7) )
#define GPIOI_PCLK_EN()									( RCC->AHB1ENR |= (1 << 8) )

/***************** Clock Enable Macros for I2Cx Peripherals *****************/

#define I2C1_PCLK_EN()									( RCC->APB1ENR |= (1 << 21) )
#define I2C2_PCLK_EN()									( RCC->APB1ENR |= (1 << 22) )
#define I2C3_PCLK_EN()									( RCC->APB1ENR |= (1 << 23) )

/***************** Clock Enable Macros for SPIx Peripherals *****************/

#define SPI1_PCLK_EN()									( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()									( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()									( RCC->APB1ENR |= (1<<15) )

/***************** Clock Enable Macros for USARTx Peripherals *****************/

#define USART1_PCLK_EN()								( RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN()								( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()								( RCC->APB1ENR |= (1<<18) )
#define UART4_PCLK_EN()									( RCC->APB1ENR |= (1<<19) )
#define UART5_PCLK_EN()									( RCC->APB1ENR |= (1<<20) )
#define USART6_PCLK_EN()								( RCC->APB2ENR |= (1<<5) )

/***************** Clock Enable Macros for SYSCFG Peripherals *****************/

#define SYSCFG_PCLK_EN()								( RCC->APB2ENR |= (1<<14) )

/***************** Clock Disable Macros for GPIOx Peripherals *****************/

#define GPIOA_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 0) )
#define GPIOB_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 1) )
#define GPIOC_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 2) )
#define GPIOD_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 3) )
#define GPIOE_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 4) )
#define GPIOF_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 5) )
#define GPIOG_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 6) )
#define GPIOH_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 7) )
#define GPIOI_PCLK_DI()									( RCC->AHB1ENR &= ~(1 << 8) )

/***************** Clock Disable Macros for I2Cx Peripherals *****************/

#define I2C1_PCLK_DI()									( RCC->APB1ENR &= ~(1 << 21) )
#define I2C2_PCLK_DI()									( RCC->APB1ENR &= ~(1 << 22) )
#define I2C3_PCLK_DI()									( RCC->APB1ENR &= ~(1 << 23) )

/***************** Clock Disable Macros for SPIx Peripherals *****************/

#define SPI1_PCLK_DI()									( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()									( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()									( RCC->APB1ENR &= ~(1<<15) )

/***************** Clock Disable Macros for USARTx Peripherals *****************/

#define USART1_PCLK_DI()								( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI()								( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()								( RCC->APB1ENR &= ~(1<<18) )
#define UART4_PCLK_DI()									( RCC->APB1ENR &= ~(1<<19) )
#define UART5_PCLK_DI()									( RCC->APB1ENR &= ~(1<<20) )
#define USART6_PCLK_DI()								( RCC->APB2ENR &= ~(1<<5) )

/***************** Clock Disable Macros for SYSCFG Peripherals *****************/

#define SYSCFG_PCLK_DI()								( RCC->APB2ENR &= ~(1<<14) )

/***************** GPIO Reset Macros *****************/

#define GPIOA_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<0) ); ( RCC->AHB1RSTR &= ~(1<<0) );}while(0)
#define GPIOB_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<1) ); ( RCC->AHB1RSTR &= ~(1<<1) );}while(0)
#define GPIOC_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<2) ); ( RCC->AHB1RSTR &= ~(1<<2) );}while(0)
#define GPIOD_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<3) ); ( RCC->AHB1RSTR &= ~(1<<3) );}while(0)
#define GPIOE_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<4) ); ( RCC->AHB1RSTR &= ~(1<<4) );}while(0)
#define GPIOF_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<5) ); ( RCC->AHB1RSTR &= ~(1<<5) );}while(0)
#define GPIOG_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<6) ); ( RCC->AHB1RSTR &= ~(1<<6) );}while(0)
#define GPIOH_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<7) ); ( RCC->AHB1RSTR &= ~(1<<7) );}while(0)
#define GPIOI_REG_RESET()								do{( RCC->AHB1RSTR |= (1<<8) ); ( RCC->AHB1RSTR &= ~(1<<8) );}while(0)

/* Return the Port for Given GPIO Base Address */
uint8_t retPort(GPIO_RegDef_t* pGPIOx);

/****************** Generic Macros ******************/

#define ENABLE 											1
#define DISABLE 										0
#define SET 											ENABLE
#define RESET 											DISABLE
#define GPIO_PIN_SET 									SET
#define GPIO_PIN_RESET 									RESET

#include "stm32f407xx_gpio_driver.h"

#endif /* INC_STM32F407XX_H_ */
