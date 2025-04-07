/*
 * stm32f411xx_spi.c
 *
 *  Created on: 19-Aug-2023
 *      Author: Neena
 */

#include "stm32f411xx_spi.h"


/*
 * SPI CLOCK CONTROL
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI==ENABLE)
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_EN();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_EN();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_EN();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_EN();
		}
		else if(pSPIx==SPI5)
		{
			SPI5_PCLK_EN();
		}
	}
	else
	{
		if(pSPIx==SPI1)
		{
			SPI1_PCLK_DI();
		}
		else if(pSPIx==SPI2)
		{
			SPI2_PCLK_DI();
		}
		else if(pSPIx==SPI3)
		{
			SPI3_PCLK_DI();
		}
		else if(pSPIx==SPI4)
		{
			SPI4_PCLK_DI();
		}
		else if(pSPIx==SPI5)
		{
			SPI5_PCLK_DI();
		}
	}
}

/*
 * SPI Initialization
 */
void SPI_Init(SPI_Handle_t *pSPIHandle){
	//PHERIPHERAL CLOCK ENABLE
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);
	uint32_t tempreg=0;
	// SETTING DEVICE MODE
	tempreg |=((pSPIHandle->SPIConfig.SPI_DeviceMode)<<2);
	// SETTING BUS CONFIG
	if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BusConfig_FD)
	{
		//bidi mode should be cleared
		tempreg &=~(1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BusConfig_HD)
	{
		//bidi mode should be set
		tempreg |= (1<<15);
	}
	else if(pSPIHandle->SPIConfig.SPI_BusConfig==SPI_BusConfig_S_RXONLY )
	{
		//bidi mode should be cleared
		tempreg &=~(1<<15);
		//rx_only bit should be set
		tempreg |= (1<<10);

	}
	// SETTING BAUDRATE PRESCALAR
	tempreg |= ((pSPIHandle->SPIConfig.SPI_SclkSpeed)<<3);
	// SETTING DATA FRAME FORMAT
	tempreg |= ((pSPIHandle->SPIConfig.SPI_DFF)<<11);
	// SETTING CPOL
	tempreg |= ((pSPIHandle->SPIConfig.SPI_CPOL)<<1);
	// SETTING CPHA
	tempreg |= ((pSPIHandle->SPIConfig.SPI_CPHA)<<0);
    //SETTING FIRSTBIT
	tempreg |= ((pSPIHandle->SPIConfig.SPI_FirstBit)<<7);
	//SETTING SSM BIT
	tempreg |= ((pSPIHandle->SPIConfig.SPI_SSM)<<9);
	pSPIHandle->pSPIx->SPI_CR1 |=tempreg;


}

/*
 * SPI DeInitialization
 */
void SPI_DeInit(SPI_RegDef_t *pSPIx){

}

uint8_t Get_SPIStatusFlag(SPI_RegDef_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName)
	{
		return FLAG_SET;
	}
	return FLAG_RESET;
}
/*
 * SPI send data (This is BLocking call)
 */

void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length){

	while(length>0)
	{
		//wait until TXE is set
		while(Get_SPIStatusFlag(pSPIx, SPI_TXE_FLAG)==FLAG_RESET);
		//check DFF
		if(pSPIx->SPI_CR1 & (1<<11))
		{
			//16 bit DFF
			//load Data in DR
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer);
			length--;
			length--;
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			//8 bit DFF

			pSPIx->SPI_DR = *pTxBuffer;
			length--;
			pTxBuffer++;
		}

	}

}

void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI==ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<6);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1<<6);
	}
}

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI){
	if(ENorDI==ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<8);
	}
	else
	{
		pSPIx->SPI_CR1 &= ~(1<<8);
	}

}
