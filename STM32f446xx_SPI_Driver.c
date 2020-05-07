/*
 * STM32f446xx_spi_driver.c
 *
 *  Created on: 19-Apr-2020
 *      Author: Rahul
 */

#include "STM32f446xx_spi_driver.h"

// SPI_GetFlagStatus
uint8_t SPI_GetFlagStatus(SPI_RegDef *pSPIx,uint32_t FlagName)
{
	if(pSPIx->SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
// API definitions
/*******************
 * Peripheral Clock Control
 */
void SPI_PClockControl(SPI_RegDef *pSPIx,uint8_t EnorDI)
{
		if(EnorDI==ENABLE)
		{
			if(pSPIx==SPI1)
			{
				SPI1_P_CLK_EN();
			}else if(pSPIx==SPI2)
			{
				SPI2_P_CLK_EN();
			}else if(pSPIx==SPI3)
			{
				SPI3_P_CLK_EN();
			}else
			{
				SPI4_P_CLK_EN();
			}
		}else
		{
			if(pSPIx==SPI1)
					{
						SPI1_P_CLK_DI();
					}else if(pSPIx==SPI2)
					{
						SPI2_P_CLK_DI();
					}else if(pSPIx==SPI3)
					{
						SPI3_P_CLK_DI();
					}else
					{
						SPI4_P_CLK_DI();
					}
		}
}

/**********************
 *  Init and Deinit
 */
void SPI_Init(SPI_Handle *pSPIHandle)
{
	//configure the CR1 register// create a 32 bit temp variable and copy it to the register
	uint32_t tempreg;

	// 1. Configure the device mode CR1 bit 2
	tempreg|= pSPIHandle->SPI_RegConfigSet.SPI_DeviceMode<<SPI_CR1_MSTR;

	// 2. Configure the bus to half duplex, full duplex or simplex
	//for full deuplex clear the bit 15 of the CR1 register
	if(pSPIHandle->SPI_RegConfigSet.SPI_BusConfig==SPI_BUS_CONFIG_FD)
	{
		// clear the 15th bit
		tempreg &=~(1<<SPI_CR1_BIDIMODE);

	}
	else if(pSPIHandle->SPI_RegConfigSet.SPI_BusConfig==SPI_BUS_CONFIG_HD)
	{
		// set the 15th bit
		tempreg |=(1<<SPI_CR1_BIDIMODE);

	}else if(pSPIHandle->SPI_RegConfigSet.SPI_BusConfig==SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		// clear the 15th bit and set the 10th bit of CR1
		tempreg &= ~(1<<SPI_CR1_BIDIMODE);
		tempreg |= (1<<SPI_CR1_RXONLY);

	}

	// 3. Configure the SCLK speed i.e the baud rate using the 3,4 and the 5th bit
	tempreg |=pSPIHandle->SPI_RegConfigSet.SPI_SclkSpeed<<SPI_CR1_BR;

	// 4. Configure the DFF
	tempreg |=pSPIHandle->SPI_RegConfigSet.SPI_DFF<<SPI_CR1_DFF;

	// 5. Configure the CPHA
	tempreg |=pSPIHandle->SPI_RegConfigSet.SPI_CPHA<<SPI_CR1_CPHA;

	// 6. Configure the CPOL
	tempreg |=pSPIHandle->SPI_RegConfigSet.SPI_CPHA<<SPI_CR1_CPOL;

	// 7. Configure the SSM
	tempreg |=pSPIHandle->SPI_RegConfigSet.SPI_SSM<<SPI_CR1_SSM;

	pSPIHandle->pSPIx->CR1=tempreg; // saving the value of tempreg to CR1 for configuring CR1


}
// Prototype to de-initiliase a port
void SPI_DeInit(SPI_RegDef *pSPIx) //sends a register to its reset state ; use of the RCC Reset resgister
{
				if(pSPIx==SPI1)
				{
					SPI1_REG_RESET();
				}else if(pSPIx==SPI2)
				{
					SPI2_REG_RESET();
				}else if(pSPIx==SPI3)
				{
					SPI3_REG_RESET();
				}else
				{
					SPI4_REG_RESET();
				}
}

/******************************
 * Data send and recieve
 */
// Blocking based- polling based -waits till all the bytes are transmitted
void SPI_sendData(SPI_RegDef *pSPIx,uint8_t *pTxBuffer,uint32_t Len)
{
	// Check the length of the data to be sent
	while(Len >0)
	{
		//1. Wait till TXE is 0 i.e. the TXBuffer is empty
		while(SPI_GetFlagStatus(pSPIx,SPI_TXE_FLAG)==FLAG_RESET); // wait till TXE is set

		//2. Check the DFF
		if(pSPIx->CR1 & (1<<SPI_CR1_DFF))  // and is 1 so 16 bits as DFF
		{
			pSPIx->DR= *((uint16_t *)pTxBuffer); // TxBuffer typecasted to 16 bits from 8 bits and data copied
			Len--;
			Len--; // twice since 2 bytes of data sent
			(uint16_t *)pTxBuffer++;
		}
		else
		{
			pSPIx->DR=*pTxBuffer;
			Len--;
			pTxBuffer++;
		}

	}
}

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

void SPI_PeripheralControl(SPI_RegDef *pSPIx,uint8_t EnorDi)
{
	if(EnorDi==ENABLE)
	{
		pSPIx->CR1 |=(1<<SPI_CR_SPE);
	}
	else
	{
		pSPIx->CR1 &= ~(1<<SPI_CR_SPE);
	}

}

