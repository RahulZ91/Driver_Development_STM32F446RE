/*
 * stmf446xx.h
 *
 *  Created on: Apr 8, 2020
 *      Author: Rahul
 */

// MCU Specific data
#include<stdint.h>

#ifndef INC_STMF446XX_H_
#define INC_STMF446XX_H_
#define __vo volatile

// Generic Macros
#define ENABLE 			1
#define DISABLE 		0
#define SET 			ENABLE
#define RESET 			DISABLE
#define GPIO_PinSet 	SET
#define GPIO_PinReset 	RESET

/* Define the base addresses for flash and SRAM */
#define DRV_BASEADDR_Flash    0x08000000u		// page 62 from reference manual
#define DRV_BASEADDR_SRAM1	  0x20000000u		// page 62 from reference manual
#define DRV_BASEADDR_SRAM2	  0x2001C000u 		// same as above or add HEX(112*1024) to DRV_BASEADDR_SRAM1
#define DRV_BASEADDR_ROM 	  0x1FFF0000u 	   //  ROM is also called the system memory- pg 62 from reference manual

/* Define the base address for each of the bus domain */
#define DRV_BASEADDR_APB1 0x40000000u          // found in reference manual
#define DRV_BASEADDR_APB2 0x40010000u
#define DRV_BASEADDR_AHB1 0x40020000u
#define DRV_BASEADDR_AHB2 0x50000000u
#define DRV_BASEADDR_AHB3 0xA0001000u

/* Define the base address of the required peripheral i.e base address of the bus domain it is communicating with plus the offset */
// GPIOs
#define DRV_BASEADDR_GPIOA 0x40020000u
#define DRV_BASEADDR_GPIOB 0x40020400u
#define DRV_BASEADDR_GPIOC 0x40020800u
#define DRV_BASEADDR_GPIOD 0x40020C00u
#define DRV_BASEADDR_GPIOE 0x40021000u
#define DRV_BASEADDR_GPIOF 0x40021400u
#define DRV_BASEADDR_GPIOG 0x40021800u
#define DRV_BASEADDR_GPIOH 0x40021C00u

// Another way to define them would be to take the base address of the bus domain and add the offset for the peripheral
// For APB1 bus
#define DRV_BASEADDR_I2C1 	(DRV_BASEADDR_APB1+ 0x5400)
#define DRV_BASEADDR_I2C2 	(DRV_BASEADDR_APB1+ 0x5800)
#define DRV_BASEADDR_I2C3 	(DRV_BASEADDR_APB1+ 0x5C00)
#define DRV_BASEADDR_USART2	(DRV_BASEADDR_APB1+ 0x4400)
#define DRV_BASEADDR_USART3	(DRV_BASEADDR_APB1+ 0x4800)
#define DRV_BASEADDR_UART4	(DRV_BASEADDR_APB1+ 0x4C00)
#define DRV_BASEADDR_UART5	(DRV_BASEADDR_APB1+ 0x5000)

// For APB2
#define DRV_BASEADDR_USART1	(DRV_BASEADDR_APB2+ 0x1000)
#define DRV_BASEADDR_USART6	(DRV_BASEADDR_APB2+ 0x1400)
#define DRV_BASEADDR_SPI1	(DRV_BASEADDR_APB2+ 0x3000)
#define DRV_BASEADDR_SPI4	(DRV_BASEADDR_APB2+ 0x3400)
#define DRV_BASEADDR_SYSCFG	(DRV_BASEADDR_APB2+ 0x3800)
#define DRV_BASEADDR_EXTI	(DRV_BASEADDR_APB2+ 0x3C00)

// For AHB1
#define DRV_BASEADDR_RCC    (DRV_BASEADDR_AHB1+ 0x3800)

// Create macro for typecasted address - pointer points to the base address of the first member in the structure
#define GPIOA ((GPIO_RegDef*)DRV_BASEADDR_GPIOA)
#define GPIOB ((GPIO_RegDef*)DRV_BASEADDR_GPIOB)
#define GPIOC ((GPIO_RegDef*)DRV_BASEADDR_GPIOC)
#define GPIOD ((GPIO_RegDef*)DRV_BASEADDR_GPIOD)
#define GPIOE ((GPIO_RegDef*)DRV_BASEADDR_GPIOE)
#define GPIOF ((GPIO_RegDef*)DRV_BASEADDR_GPIOF)
#define GPIOG ((GPIO_RegDef*)DRV_BASEADDR_GPIOG)
#define GPIOH ((GPIO_RegDef*)DRV_BASEADDR_GPIOH)

// Macros for typecasted address for RCC
#define RCC  ((RCC_RegDef*)DRV_BASEADDR_RCC)

// Macros for the peripheral clock enable for GPIO
#define 	GPIOA_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<0)) // Set the 0th bit in the RCC_AHB1ENR for PortA
#define 	GPIOB_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<1)) // Set the 1st bit in the RCC_AHB1ENR for PortB
#define 	GPIOC_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<2)) // Set the 2nd bit in the RCC_AHB1ENR for PortC
#define 	GPIOD_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<3)) // Set the 3rd bit in the RCC_AHB1ENR for PortD
#define 	GPIOE_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<4)) // Set the 4th bit in the RCC_AHB1ENR for PortE
#define 	GPIOF_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<5)) // Set the 5th bit in the RCC_AHB1ENR for PortF
#define 	GPIOG_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<6)) // Set the 6th bit in the RCC_AHB1ENR for PortG
#define 	GPIOH_P_CLK_EN() 	(RCC->AHB1ENR|=(1<<7)) // Set the 7th bit in the RCC_AHB1ENR for PortH

// Clock enable macros for I2C peripheral
#define I2C1_P_CLK_EN()			(RCC->APB1ENR|=(1<<21))
#define I2C2_P_CLK_EN()			(RCC->APB1ENR|=(1<<22))
#define I2C3_P_CLK_EN()			(RCC->APB1ENR|=(1<<23))

// Clock enable macros for SPI peripheral
#define SPI1_P_CLK_EN()		(RCC->APB2ENR|=(1<<12))
#define SPI2_P_CLK_EN()		(RCC->APB1ENR|=(1<<14))
#define SPI3_P_CLK_EN()		(RCC->APB1ENR|=(1<<15))
#define SPI4_P_CLK_EN()		(RCC->APB2ENR|=(1<<13))

// Clock enable macros for UART and USART
#define USART1_P_CLK_EN()			(RCC->APB2ENR|=(1<<4))
#define USART2_P_CLK_EN()			(RCC->APB1ENR|=(1<<17))
#define USART3_P_CLK_EN()			(RCC->APB1ENR|=(1<<18))
#define UART4_P_CLK_EN()			(RCC->APB1ENR|=(1<<19))
#define UART5_P_CLK_EN()			(RCC->APB1ENR|=(1<<20))
#define USART6_P_CLK_EN()			(RCC->APB2ENR|=(1<<5))

// Clock enable macro for SYSCFG
#define SYSCFG_P_CLK_EN()	(RCC->APB2ENR |=(1<<14))

// Clock disable macro for GPIO
#define 	GPIOA_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<0)) // Reset the 0th bit in the RCC_AHB1ENR for PortA
#define 	GPIOB_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<1)) // Reset the 1st bit in the RCC_AHB1ENR for PortB
#define 	GPIOC_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<2)) // Reset the 2nd bit in the RCC_AHB1ENR for PortC
#define 	GPIOD_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<3)) // Reset the 3rd bit in the RCC_AHB1ENR for PortD
#define 	GPIOE_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<4)) // Reset the 4th bit in the RCC_AHB1ENR for PortE
#define 	GPIOF_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<5)) // Reset the 5th bit in the RCC_AHB1ENR for PortF
#define 	GPIOG_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<6)) // Reset the 6th bit in the RCC_AHB1ENR for PortG
#define 	GPIOH_P_CLK_DI() 	(RCC->AHB1ENR&=~(1<<7)) // Reset the 7th bit in the RCC_AHB1ENR for PortH

// Clock disable macros for I2C
#define I2C1_P_CLK_DI()			(RCC->APB1ENR&=~(1<<21))
#define I2C2_P_CLK_DI()			(RCC->APB1ENR&=~(1<<22))
#define I2C3_P_CLK_DI()			(RCC->APB1ENR&=~(1<<23))

// Clock disable macro for SPI Peripheral
#define SPI1_P_CLK_DI()		(RCC->APB2ENR&=~(1<<12))
#define SPI2_P_CLK_DI()		(RCC->APB1ENR&=~(1<<14))
#define SPI3_P_CLK_DI()		(RCC->APB1ENR&=~(1<<15))
#define SPI4_P_CLK_DI()		(RCC->APB2ENR&=~(1<<13))

// Clock disable macros for USART and UART
#define USART1_P_CLK_DI()			(RCC->APB2ENR&=~(1<<4))
#define USART2_P_CLK_DI()			(RCC->APB1ENR&=~(1<<17))
#define USART3_P_CLK_DI()			(RCC->APB1ENR&=~(1<<18))
#define UART4_P_CLK_DI()			(RCC->APB1ENR&=~(1<<19))
#define UART5_P_CLK_DI()			(RCC->APB1ENR&=~(1<<20))
#define USART6_P_CLK_DI()			(RCC->APB2ENR&=~(1<<5))

// Clock disable macro for SYSCFG
#define SYSCFG_P_CLK_DI()	(RCC->APB2ENR &=~(1<<14))

/* Structuring the Peripheral registers i.e create a C structure for a peripheral with the peripheral registers as the members of the structure
The structure is initialized with the base address of the peripheral- use of Register Map from the reference manual
The specific register can be accessed via de-referencing -> */
// _vo indicates Volatile as is defined by the macro
// For GPIO
typedef struct
{
	// Order is based on the offset ==important to order as per offset
		__vo uint32_t MODER;// offset: 0x00    GPIO port mode register
		__vo uint32_t OTYPER;// offset: 0x04   GPIO port output type register
		__vo uint32_t OSPEEDER;// offset: 0x08 GPIO port output speed register
		__vo uint32_t PUPDR;// offset: 0x0c    GPIO port pull-up/pull-down register
		__vo uint32_t IDR;// offset: 0x10	  GPIO port input data register
		__vo uint32_t ODR;// offset: 0x14	  GPIO port output data register
		__vo uint32_t BSRR;// offset: 0x18	  GPIO port bit set/reset register
		__vo uint32_t LCKR;// offset: 0x1c     GPIO port configuration lock register
		__vo uint32_t AFR[2];// offset: 0x20   GPIO alternate function register[0:low, 1:high]
}GPIO_RegDef;
//GPIO_RegDef *pGPIOA=GPIOA;

// Structure for RCC since the clock is to be controlled via RCC
typedef struct
{
	__vo uint32_t CR; // Offset : 0x00
	__vo uint32_t PLL_CFGR; // Offset : 0x04
	__vo uint32_t CFGR; // Offset : 0x08
	__vo uint32_t CIR; // Offset : 0x0C
	__vo uint32_t AHB1RSTR; // Offset : 0x10
	__vo uint32_t AHB2RSTR; // Offset : 0x14
	__vo uint32_t AHB3RSTR; // Offset : 0x18
	 uint32_t Reserved1; // Offset : 0x1C
	__vo uint32_t APB1RSTR; // Offset : 0x20
	__vo uint32_t APB2RSTR; // Offset : 0x24
	 uint32_t Reserved2; // Offset : 0x28
	 uint32_t Reserved3; // Offset : 0x2C
	__vo uint32_t AHB1ENR; // Offset : 0x30
	__vo uint32_t AHB2ENR; // Offset : 0x34
	__vo uint32_t AHB3ENR; // Offset : 0x38
	uint32_t Reserved4; // Offset : 0x3C
	__vo uint32_t APB1ENR; // Offset : 0x40
	__vo uint32_t APB2ENR; // Offset : 0x44
	uint32_t Reserved5; // Offset : 0x48
	uint32_t Reserved6; // Offset : 0x4C
	__vo uint32_t AHB1LPENR; // Offset : 0x50
	__vo uint32_t AHB2LPENR; // Offset : 0x54
	__vo uint32_t AHB3LPENR; // Offset : 0x58
	uint32_t Reserved7; // Offset : 0x5C
	__vo uint32_t APB1LPENR; // Offset : 0x60
	__vo uint32_t APB2LPENR; // Offset : 0x64
	uint32_t Reserved8[2]; // Offset : 0x68 - 0x6C
	__vo uint32_t BDCR; // Offset : 0x70
	__vo uint32_t CSR; // Offset : 0x74
}RCC_RegDef;
//RCC_RegDef *pRCC=RCC; // RCC is a defined macro for RCC base address

#endif /* INC_STMF446XX_H_ */
