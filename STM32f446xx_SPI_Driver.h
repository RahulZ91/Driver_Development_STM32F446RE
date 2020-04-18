/*
 * STM32f446xx_spi_driver.h
 *
 *  Created on: 18-Apr-2020
 *      Author: Rahul
 */

#ifndef INC_STM32F446XX_SPI_DRIVER_H_
#define INC_STM32F446XX_SPI_DRIVER_H_
#include "stmf446xx.h"

typedef struct
{
	// Members are of 8 bits or 1 byte since the pins vary from 0-15 in a port i.e can be 0000 to 1111
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig; // Simplex,duplex,half duplex
	uint8_t SPI_DFF;//8/16 bit data frame format
	uint8_t SPI_CPHA;// Phase i.e data sampled from leading edge or trailing edge
	uint8_t SPI_CPOL;// Polarity i.e. sclk state on idle 0 or 1
	uint8_t SPI_SSM;// Slave selection
	uint8_t SPI_Speed;// Speed of the SCLK

}SPI_RegConfig;

/* handle structure for the SPI */
typedef struct
{
	// Pointer to hold the base address of the GPIO peripheral
	SPI_RegDef *pSPIx; // Hold the base address of the port to which the pin belongs
	SPI_RegConfig SPI_RegConfigSet; // The structure holding the various pin configurations
}SPI_Handle;

/*********************************
 *  API Prototypes
 */

/*******************
 * Peripheral Clock Control
 */
void SPI_PClockControl(SPI_RegDef *pSPIx,uint8_t EnorDI);

/**********************
 *  Init and Deinit
 */
void SPI_Init(SPI_RegDef *pSPIHandle);
// Prototype to de-initiliase a port
void SPI_DeInit(SPI_RegDef *pSPIx); //sends a register to its reset state ; use of the RCC Reset resgister

/******************************
 * Data send and recieve
 */
// Blocking based- polling based

void SPI_sendData(SPI_RegDef *pSPIx,uint8_t *pTxBuffer,uint32_t Len);

void SPI_receiveData(SPI_RegDef *pSPIx,uint8_t *pRxBuffer,uint32_t Len);


/**********************
 * IRQ and ISR Handling
 */

// Interrupt
void SPI_IRQInterruptConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority,uint8_t EnorDi); // Config IRQ number
// IRQ priority set
void SPI_IRQPriorityConfig(uint8_t IRQ_Number,uint8_t IRQ_Priority);//Configure the priority
// Handle Interrupt and ISR
void SPI_IRQHandling(SPI_Handle *pHandle); // from which pin the interrupt is triggered is entered via parameter ie the pinNumber

#endif /* INC_STM32F446XX_SPI_DRIVER_H_ */
