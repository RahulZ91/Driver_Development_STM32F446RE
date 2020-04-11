/*
 * STM32f446xx_driver_gpio.h
 *
 *  Created on: 10-Apr-2020
 *      Author: Rahul
 */
// Contains driver specific data
#ifndef INC_STM32F446XX_DRIVER_GPIO_H_
#define INC_STM32F446XX_DRIVER_GPIO_H_

#include "stmf446xx.h"

typedef struct
{
	// Members are of 8 bits or 1 byte since the pins vary from 0-15 in a port i.e can be 0000 to 1111
	uint8_t GPIO_PinNumber; // Possible pins @pin_numbers
	uint8_t GPIO_PinMode; // Possible modes @GPIO_pin_modes
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_PuPdControl;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFnMode;
}GPIO_PinConfig;

// The Handle structure for GPIO Pin
typedef struct
{
	// Pointer to hold the base address of the GPIO peripheral
	GPIO_RegDef *pGPIOx; // Hold the base address of the port to which the pin belongs
	GPIO_PinConfig GPIO_PinConfigSet; // The structure holding the various pin configurations
}GPIO_Handle;

// GPIO pin numbers
/*****
 * @pin_numbers
 */
#define GPIO_PIN_NO_0 		0
#define GPIO_PIN_NO_1 		1
#define GPIO_PIN_NO_2 		2
#define GPIO_PIN_NO_3 		3
#define GPIO_PIN_NO_4 		4
#define GPIO_PIN_NO_5 		5
#define GPIO_PIN_NO_6 		6
#define GPIO_PIN_NO_7 		7
#define GPIO_PIN_NO_8 		8
#define GPIO_PIN_NO_9 		9
#define GPIO_PIN_NO_10 		10
#define GPIO_PIN_NO_11 		11
#define GPIO_PIN_NO_12		12
#define GPIO_PIN_NO_13 		13
#define GPIO_PIN_NO_14		14
#define GPIO_PIN_NO_15		15

// Macros for pin modes
/****
 * @GPIO_pin_modes
 */
#define GPIO_MODE_IN 		0
#define GPIO_MODE_OUT 		1
#define GPIO_MODE_ALTFN 	2
#define GPIO_MODE_ANALOG 	3
#define GPIO_MODE_IT_FT 	4       // falling edge
#define GPIO_MODE_IT_RT 	5		// rising edge
#define GPIO_MODE_IT_RFT 	6

// Macros for the GPIO pin output types
#define GPIO_OP_TYPE_PP		0	// Pushpull
#define GPIO_OP_TYPE_OD		1	// Open Drain

// Macros for possible speeds
#define GPIO_SPEED_LOW      0
#define GPIO_SPEED_MEDIUM   1
#define GPIO_SPEED_FAST		2
#define GPIO_SPEED_HIGH		3

// Macros for pullup pull down
#define GPIO_NO_PUPD        0
#define GPIO_PIN_PU   	    1
#define GPIO_PIN_PD		    2

// Prototype to enable/disable Peripheral clock
void PClockControl(GPIO_RegDef *pGPIOx,uint8_t EnorDI);

// API prototypes
// Prototype for GPIO Initialisation
void GPIO_Init(GPIO_Handle *pGPIOHandle);
// Prototype to de-initiliase a port
void GPIO_DeInit(GPIO_Handle *pGPIOHandle); //sends a register to its reset state ; use of the RCC Reset register- set a bit for the entire GPIO port to be de-initialised

// IO
// Prototype to read from the pin
uint8_t GPIO_ReadFromInoutPin(GPIO_RegDef *pGPIOx,uint8_t pinNumber); // (port choose, pin choose)
// Prototype to read from input port
uint16_t GPIO_ReadFromInputPort(GPIO_RegDef *pGPIOx);
// Prototype Write to GPIO
void GPIO_WriteToOutputPin(GPIO_RegDef *pGPIOx,uint8_t pinNumber,uint8_t Value); // value is 0 or 1 i.e SET or RESET
// Prototype to write to output port
void GPIO_WriteToOutputPort(GPIO_RegDef *pGPIOx,uint16_t Value);// GPIO_RegDef *pGPIOx points to the GPIOA and then required GPIO can be accessed via dereferncing i.e. from the structure
// Prototype to toggle pin
void GPIO_TogglePin(GPIO_RegDef *pGPIOx,uint8_t pinNumber);

// Interrupt APIs
// Interrupt
void GPIO_IRQConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority,uint8_t EnorDi); // Config IRQ number
// IRQ Handle
void GPIO_IRQHandling(uint8_t pinNumber); // from which pin the interrupt is triggered is entered via parameter ie the pinNumber


#endif /* INC_STM32F446XX_DRIVER_GPIO_H_ */
