/*
 * STM32f446xx_driver_gpio.c
 *
 *  Created on: 10-Apr-2020
 *      Author: Rahul
 */

#include "STM32F446xx_GPIO_Driver.h"

// API definitions in the driver header file and definitions in the .c file

// Peripheral clock control
// Prototype to enable/disable Peripheral clock
/* *******************************************
 * Function name: PClockControl
 *
 * What it does:  Enables or Disables the Peripheral clock for the selected GPIO Port
 *
 * Parameters:
 * *pGPIO:        The base address for the GPIO port.To check for the port and then to enable /disable(based on EnorDi) the clock for the port using the macros
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
	uint32_t temp=0;// Temporary register

	//1. Configure the pin mode : first register in RCC for a given port
	if(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode<=GPIO_MODE_ANALOG)
	{
		temp =pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode <<(2*pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber is used to select the pin number set by the user and shift it twice since one pin has 2 bits in the port mode register
		pGPIOHandle->pGPIOx->MODER &= ~(0x3 << pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // clear the bits before setting
		pGPIOHandle->pGPIOx->MODER|=temp; // store the value in MODER use of bitwise OR |= so that other bits in the register are not disturbed
	}
	else
	{
		if(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode == GPIO_MODE_IT_FT)
		{// Configure FTSR
				EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // set FTSR
				EXTI->RTSR &=~(1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // clear RTSR
		}else if(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode == GPIO_MODE_IT_RT)
			{// configure RTSR
				EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // set FTSR
				EXTI->FTSR &=~(1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // clear RTSR
			}else if(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinMode == GPIO_MODE_IT_RFT)
			{// configure both RTSR and FTSR
				EXTI->FTSR |= (1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // set FTSR
				EXTI->RTSR |= (1<< pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // set FTSR
			}
			//2. Configure GPIO port in SYSCFG_EXTI register
			SYSCFG_P_CLK_EN();
			uint8_t temp1= pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber / 4;
			uint8_t temp2= pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber % 4;
			uint8_t portCode=GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx); // portcode can be  0 to 6
			SYSCFG->EXTICR[temp1]=portCode <<(temp2 *4);
			// 3. Enable EXTI interrupt delivery using IMR
			EXTI->IMR |=(1<<pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);
	}
	

	temp=0; // Re-initiliase temp to 0
	//2. Configure pin speed
	temp=pGPIOHandle->GPIO_PinConfigSet.GPIO_PinSpeed <<(2*pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber); // Set pin speed
	pGPIOHandle->pGPIOx->OSPEEDER &= ~(0x3 << pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OSPEEDER|=temp;

	temp=0;

	//3. Configure the pullup pulldown settings
	temp=pGPIOHandle->GPIO_PinConfigSet.GPIO_PuPdControl <<(2*pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);// Set the pullup /pulldown using the PuPDR register; it is also having 2 bits for a pin
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x3 << pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->PUPDR|=temp;

	temp=0;

	//4. Configure the output type
	temp=pGPIOHandle->GPIO_PinConfigSet.GPIO_PinOPType <<(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER &=~(0x1<<pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber);
	pGPIOHandle->pGPIOx->OTYPER |=temp;

	temp=0;

	//5. Configure the alternate functionality
	uint32_t temp1,temp2;
	temp1= pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber /8; // if 0 then choose ARL if 1 choose ARH
	temp2= pGPIOHandle->GPIO_PinConfigSet.GPIO_PinNumber %8; // 4*temp2 determines start of bit set;since each pin has 4 bits to be configured in ARL or ARH
	pGPIOHandle->pGPIOx->AFR[temp1] &=~(0xF<<(4*temp2));
	pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfigSet.GPIO_PinAltFnMode <<(4*temp2));
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
void GPIO_DeInit(GPIO_Handle *pGPIOx)
{
			if(pGPIOx==GPIOA)
			{
				GPIOA_REG_RESET();
			}else if(pGPIOx==GPIOB)
			{
				GPIOB_REG_RESET();
			}else if(pGPIOx==GPIOC)
			{
				GPIOC_REG_RESET();
			}else if(pGPIOx==GPIOD)
			{
				GPIOD_REG_RESET();
			}else if(pGPIOx==GPIOE)
			{
				GPIOE_REG_RESET();
			}else if(pGPIOx==GPIOF)
			{
				GPIOF_REG_RESET();
			}else if(pGPIOx==GPIOG)
			{
				GPIOG_REG_RESET();
			}else
			{
				GPIOH_REG_RESET();
			}
}

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
{
	uint8_t value;
	value =(pGPIOx->IDR >> pinNumber ) & (0x00000001); // shift the value from the pin to LSB and mask all other bit except LSB
	return value;
}

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
{
	uint16_t value;
	value=(uint16_t *)pGPIOx->IDR;
	return value;
}


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
{
	if(Value == GPIO_PinSet)
	{
		// return 1
		pGPIOx->ODR |=(1<< pinNumber);
	}
	else
	{
		// write 0
		pGPIOx->ODR &=~(1<< pinNumber);
	}
}

// value is 0 or 1 i.e SET or RESET
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
{
	pGPIOx->ODR = Value;
}
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
{
	pGPIOx->ODR ^=(1 << pinNumber);
}

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
{
	if (EnorDi==ENABLE) // if EnorDi is ENABLE then set the priority hence configure the ISER(Interrupt Set Register)
	{
			if(IRQ_Number <=31)
			{
				// Set corresponding bit in ISER0
				*NVIC_ISER0 |=(1 << IRQ_Number);
			}
			else if(IRQ_Number>31 && IRQ_Number<64)
			{
				// Set the corresponding bit in ISER1
				*NVIC_ISER1 |= (1<<(IRQ_Number%32));
			}
			else if(IRQ_Number>64 && IRQ_Number<95)
			{
				// Set the corresponding bit in ISER2
				*NVIC_ISER2 |=(1<<(IRQ_Number%64));
			}
	}
	else
	{
			if(IRQ_Number <=31)
			{
				// Set corresponding bit in ICER0
				*NVIC_ICER0 |=(1 << IRQ_Number);
			}
			else if(IRQ_Number>31 && IRQ_Number<64)
			{
				// Set the corresponding bit in ICER1
				*NVIC_ICER1 |=(1<<(IRQ_Number%32));
			}
			else if(IRQ_Number>64 && IRQ_Number<95)
			{
				// Set the corresponding bit in ICER2
				*NVIC_ICER2 |=(1<<(IRQ_Number%64));
			}
	}
	
}
	
	// Config IRQ number
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

