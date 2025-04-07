/*
 * stm32f411xx_spi.h
 *
 *  Created on: 19-Aug-2023
 *      Author: Neena
 */

#ifndef INC_STM32F411XX_SPI_H_
#define INC_STM32F411XX_SPI_H_

#include "stm32f411xx.h"


/*
 * SPI Configuration Structure
 */

typedef struct{
	uint8_t SPI_DeviceMode;
	uint8_t SPI_BusConfig;
	uint8_t SPI_SclkSpeed;
	uint8_t SPI_DFF;
	uint8_t SPI_CPOL;
	uint8_t SPI_CPHA;
	uint8_t SPI_SSM;
    uint8_t SPI_FirstBit;

}SPI_Config_t;

/*
 * SPI Handle Structure
 */

typedef struct{
	SPI_RegDef_t *pSPIx;
	SPI_Config_t SPIConfig;

}SPI_Handle_t;

/*
 * API FOR SPI DRIVER
 */

/*
 * SPI CLOCK CONTROL
 */

void SPI_PeriClockControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 * SPI Init and DeInit
 */
void SPI_Init(SPI_Handle_t *pSPIHandle);
void SPI_DeInit(SPI_RegDef_t *pSPIx);

/*
 * DATA SEND AND RECEIVE
 */
void SPI_SendData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx,uint8_t *pTxBuffer,uint32_t length);


/*
 * SPI Interrupt Handling
 */
void SPI_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *pSPIHandle);


/*
 * SPI Peripheral Enable
 */
void SPI_PeriControl(SPI_RegDef_t *pSPIx, uint8_t ENorDI);

/*
 * SPI SSI CONFIG
 */
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, uint8_t ENorDI);


/*
 * CPOL_STATE
 */
#define SPI_POLARITY_LOW        0
#define SPI_POLARITY_HIGH       1

/*
 * CPHA_STATE
 */
#define SPI_PHASE_1EDGE         0
#define SPI_PHASE_2EDGE         1

/*
 * SPI_MODE
 */
#define SPI_MODE_SLAVE          0
#define SPI_MODE_MASTER         1

/*
 * SPI_BAUDRATE_PRESCALER
 */
#define SPI_BAUDRATEPRESCALER_2     0
#define SPI_BAUDRATEPRESCALER_4     1
#define SPI_BAUDRATEPRESCALER_8     2
#define SPI_BAUDRATEPRESCALER_16    3
#define SPI_BAUDRATEPRESCALER_32    4
#define SPI_BAUDRATEPRESCALER_64    5
#define SPI_BAUDRATEPRESCALER_128   6
#define SPI_BAUDRATEPRESCALER_256   7

/*
 * SPI DATA FRAME FORMAT
 */
#define SPI_DATASIZE_8BIT      0
#define SPI_DATASIZE_16BIT     1

/*
 * SPI_FIRSTBIT
 */
#define SPI_FIRSTBIT_MSB       0
#define SPI_FIRSTBIT_LSB	   1

/*
 * SPI Software Slave Management
 */
#define SPI_SSM_DISABLED       0
#define SPI_SSM_ENABLED        1

/*
 * SPI_Bus_Config
 */
#define SPI_BusConfig_FD           1
#define SPI_BusConfig_HD           2
#define SPI_BusConfig_S_RXONLY     3

/*
 * SPI Related status flag definition
 */
#define SPI_TXE_FLAG      (1<<1)
#define SPI_RXNE_FLAG     (1<<0)
#define SPI_BSY_FLAG      (1<<7)



#endif /* INC_STM32F411XX_SPI_H_ */
