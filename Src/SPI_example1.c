/*
 * 002PushBtn.c
 *
 *  Created on: Aug 5, 2023
 *      Author: Neena
 */

#include "stm32f411xx.h"
#include "stm32f411xx_gpio.h"
#include "stm32f411xx_spi.h"
#include <stdint.h>
#include <string.h>

#if !defined(__SOFT_FP__) && defined(__ARM_FP)
#warning "FPU is not initialized, but the project is compiling for an FPU. Please initialize the FPU before use."
#endif
/*
 * SPI Pins
 * PA7 -> SPI1_MOSI
 * PA6 -> SPI1_MISO
 * PA5 -> SPI1_SCK
 * PB6 -> CS
 * ALTERNATE FUNCTION MODE IS 5
 */
void delay(uint32_t vlaue);
void SPI1GpioInit(void);
void SPI1_Init(void);

int main(void)
{
	char user_data[]="Hello World";
	SPI1GpioInit();
	GPIO_Write_Output_Pin(GPIOB,GPIO_PIN_6 , GPIO_PIN_SET);
	SPI1_Init();
	SPI_SSIConfig(SPI1,ENABLE);

	while(1)
	{
		GPIO_Write_Output_Pin(GPIOB,GPIO_PIN_6 , GPIO_PIN_RESET);
		SPI_PeriControl(SPI1,ENABLE);
		SPI_SendData(SPI1,(uint8_t*)user_data,strlen(user_data));
		GPIO_Write_Output_Pin(GPIOB,GPIO_PIN_6 , GPIO_PIN_SET);
		SPI_PeriControl(SPI1,DISABLE);
	}
    return 0;
}


void delay(uint32_t value)
{
	uint32_t i;

	for(i=0;i<value;i++);
}

void SPI1GpioInit(void)
{
	GPIO_Handle_t SPIPins;

	SPIPins.pGPIOx=GPIOA;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_ALF_MODE;
	SPIPins.GPIO_PinConfig.GPIO_PinAltFunMode=5;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_Pinpupdconrol=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	//SPI_SCK
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_5;
	GPIO_Init(&SPIPins);

	//SPI_MISO
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_6;
	GPIO_Init(&SPIPins);

	//SPI_MOSI
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_7;
	GPIO_Init(&SPIPins);

//	SPI_CS
	SPIPins.pGPIOx=GPIOB;
	SPIPins.GPIO_PinConfig.GPIO_PinNumber=GPIO_PIN_6;
	SPIPins.GPIO_PinConfig.GPIO_PinMode=GPIO_OUTPUT_MODE;
	SPIPins.GPIO_PinConfig.GPIO_PinOPType=GPIO_OP_TYPE_PP;
	SPIPins.GPIO_PinConfig.GPIO_Pinpupdconrol=GPIO_NO_PUPD;
	SPIPins.GPIO_PinConfig.GPIO_PinSpeed=GPIO_SPEED_FAST;
	GPIO_Init(&SPIPins);


}

void SPI1_Init(void){

	SPI_Handle_t SPI1handle;

	SPI1handle.pSPIx=SPI1;
	SPI1handle.SPIConfig.SPI_BusConfig=SPI_BusConfig_FD;
	SPI1handle.SPIConfig.SPI_DeviceMode=SPI_MODE_MASTER;
	SPI1handle.SPIConfig.SPI_SclkSpeed=SPI_BAUDRATEPRESCALER_2;
	SPI1handle.SPIConfig.SPI_DFF=SPI_DATASIZE_8BIT;
	SPI1handle.SPIConfig.SPI_CPHA=SPI_PHASE_1EDGE;
	SPI1handle.SPIConfig.SPI_CPOL=SPI_POLARITY_LOW;
	SPI1handle.SPIConfig.SPI_FirstBit=SPI_FIRSTBIT_MSB;
	SPI1handle.SPIConfig.SPI_SSM=SPI_SSM_ENABLED;
	SPI_Init(&SPI1handle);

}


