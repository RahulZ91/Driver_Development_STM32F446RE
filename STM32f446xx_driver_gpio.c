/*
 * STM32f446xx_driver_gpio.c
 *
 *  Created on: 10-Apr-2020
 *      Author: Rahul
 */

#include "STM32f446xx_driver_gpio.h"

// API definitions in the driver header file and definitions in the .c file

// Peripheral clock control
// Prototype to enable/disable Peripheral clock
/* *******************************************
 * Function name: PClockControl
 *
 * What it does:  Enables or Disables the Peripheral clock for the selected GPIO Port
 *
 * Parameters:
 * *pGPIO:        The base address for the GPIO port. Access via dereferencing the structure
 * EnorDI:		  Enable or Disable- Macro for the same has been defined
 *
 * Returns:       No return type since only the clock is enabled or disabled
 *
 */

void PClockControl(GPIO_RegDef *pGPIOx,uint8_t EnorDI)
{
	if(EnorDI==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_P_CLK_EN();
		}else if(pGPIOx==GPIOB)
		{
			GPIOB_P_CLK_EN();
		}else if(pGPIOx==GPIOC)
		{
			GPIOC_P_CLK_EN();
		}else if(pGPIOx==GPIOD)
		{
			GPIOD_P_CLK_EN();
		}else if(pGPIOx==GPIOE)
		{
			GPIOE_P_CLK_EN();
		}else if(pGPIOx==GPIOF)
		{
			GPIOF_P_CLK_EN();
		}else if(pGPIOx==GPIOG)
		{
			GPIOG_P_CLK_EN();
		}else
		{
			GPIOH_P_CLK_EN();
		}
	}else
	{
		if(pGPIOx==GPIOA)
				{
					GPIOA_P_CLK_DI();
				}else if(pGPIOx==GPIOB)
				{
					GPIOB_P_CLK_DI();
				}else if(pGPIOx==GPIOC)
				{
					GPIOC_P_CLK_DI();
				}else if(pGPIOx==GPIOD)
				{
					GPIOD_P_CLK_DI();
				}else if(pGPIOx==GPIOE)
				{
					GPIOE_P_CLK_DI();
				}else if(pGPIOx==GPIOF)
				{
					GPIOF_P_CLK_DI();
				}else if(pGPIOx==GPIOG)
				{
					GPIOG_P_CLK_DI();
				}else
				{
					GPIOH_P_CLK_DI();
				}
	}
}
// API prototypes
// Prototype for GPIO Initialisation

/**********************************************
 * Function name:GPIO_Init
 *
 * What it does: Initialises the GPIO
 *
 * Parameters:
 * *pGPIOHandle: Takes the pointer of the type GPIO_Handle that points to the base address of the port for the pin
 *
 * Returns:      Only initialises- does not return any value
 */
void GPIO_Init(GPIO_Handle *pGPIOHandle)
{
	uint32_t temp=0; // temporary register initiliased to 0
	if(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode<=GPIO_MODE_ANALOG )
	{
		temp=pGPIOHandle->GPIO_PinConfig.G
	}


}


/***********************************************
 * Function name: GPIO_DeInit
 *
 * What it does : Deinitiliases the GPIO pin
 *
 * Parameters:
 * *pGPIOHandle: Pointer of type GPIOHandle. Points to the pointer that points to the base address of the GPIO Port
 *
 * Returns     : No data return
 */
void GPIO_DeInit(GPIO_Handle *pGPIOHandle)
{}
// IO
// Prototype to read from the pin
/**********************************************
 * Function name: GPIO_ReadFromInputPin
 *
 * What it does:  Reads the data from a GPIO pin configured as input
 *
 * Parameters:
 * *pGPIOx:		  Points to the base address of the port whose pin is required to be read
 * pinNumber:	  The pin number of the pin whose input data is to be read
 *
 * Returns:		  0/1
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegDef *pGPIOx,uint8_t pinNumber)
{}
// (port choose, pin choose)
// Prototype to read from input port
/*************************************************
 *Function name: GPIO_ReadFromInputPort
 *
 *What it does:  Reads data from the specified port
 *
 *Parameters:
 **pGPIOx:		 Points to the base address of the specified port
 *
 *Returns:       Data as a word form
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef *pGPIOx)
{}
// Prototype Write to GPIO

/**************************************************
 * Function name: GPIO_WriteToOutputPin
 *
 * What it does:  Writes the specified value to the particular GPIO pin(in output mode) of the specified port
 *
 * Parameters:
 * *pGPIOx :      Points to the base address of the GPIO port, port access via the Peripheral register
 * pinNumber:     Specifies the GPIO pin for write operation
 * value:         Specifies the value to be written to the GPIO pin
 *
 * Returns:      No value returned
 */
void GPIO_WriteToOutputPin(GPIO_RegDef *pGPIOx,uint8_t pinNumber,uint8_t Value)
{}// value is 0 or 1 i.e SET or RESET
// Prototype to write to output port

/******************************************************
 * Function name: GPIO_WriteToOutputPort
 *
 * What it does:  Writes data to the specified port
 *
 * Parameters:
 * *pGPIOx:       Pointer to the base address of the GPIO ports
 * value:         Value to be written
 *
 * Returns:       No value returned
 */
void GPIO_WriteToOutputPort(GPIO_RegDef *pGPIOx,uint16_t Value)
{}
// GPIO_RegDef *pGPIOx points to the GPIOA and then required GPIO can be accessed via dereferncing i.e. from the structure
// Prototype to toggle pin

/********************************************************
 * Function name: GPIO_TogglePin
 *
 * What it does:  Toggles a GPIO pin of the specified port and pin number
 *
 * Parameters:
 * *pGPIOx:	      Points to the base address of the GPIO port
 * pinNumber:     Specifies the pin number of the pin to be toggled
 */
void GPIO_TogglePin(GPIO_RegDef *pGPIOx,uint8_t pinNumber)
{}

// Interrupt APIs
// Interrupt

/***********************************************************
 * Function name:GPIO_IRQConfig
 *
 * What it does: Configures the interrupt based on IRQ number and priority
 *
 * Parameters:
 * IRQ_Number:
 * IRQ_Priority:
 * EnorDi;      Enable or disable, defined by a macro
 *
 * Returns:     No value returned
 */
void GPIO_IRQConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority,uint8_t EnorDi)
{}// Config IRQ number
// IRQ Handle

/************************************************************
 * Function name: GPIO_IRQHandling
 *
 * What it does:
 *
 * Parameters:
 * pinNumber:     from which pin the interrupt is triggered is entered via parameter ie the pinNumber
 *
 * Returns:       No value returned
 */
void GPIO_IRQHandling(uint8_t pinNumber)
{}// from which pin the interrupt is triggered is entered via parameter ie the pinNumber

