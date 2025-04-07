/*
 * stm32f411xx_gpio.c
 *
 *  Created on: 30-Jul-2023
 *      Author: Neena
 */

#include "stm32f411xx_gpio.h"


/*
 *GPIO Peripheral clock control
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI){
	if(ENorDI==ENABLE)
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_EN();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_EN();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_EN();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_EN();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_EN();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_EN();
		}

	}
	else
	{
		if(pGPIOx==GPIOA)
		{
			GPIOA_PCLK_DI();
		}
		else if(pGPIOx==GPIOB)
		{
			GPIOB_PCLK_DI();
		}
		else if(pGPIOx==GPIOC)
		{
			GPIOC_PCLK_DI();
		}
		else if(pGPIOx==GPIOD)
		{
			GPIOD_PCLK_DI();
		}
		else if(pGPIOx==GPIOE)
		{
			GPIOE_PCLK_DI();
		}
		else if(pGPIOx==GPIOH)
		{
			GPIOH_PCLK_DI();
		}
	}

}

/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle){
	//PHERIPHERAL CLOCK ENABLE
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);
	uint32_t temp = 0;  //Temperature Register
	//Configure the mode of GPIO pin
if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE)
{
    temp= (pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
    pGPIOHandle->pGPIOx->MODE &=~(0x3 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Bit clear
    pGPIOHandle->pGPIOx->MODE |=temp;
}
else
{
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FALLING)
	{
		//1. Configure falling edge clock register
		EXTI->EXTI_FTSR  |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//CLEAR THE CORRESPONDING RTSR BIT
		EXTI->EXTI_RTSR  &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_RISING)
	{
		//1. Configure rising edge clock register
		EXTI->EXTI_RTSR  |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		//CLEAR THE CORRESPONDING FTSR BIT
		EXTI->EXTI_FTSR  &= ~(1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);

	}else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_IT_FALLRISE)
	{
		//1. Configure falling-rising edge clock register
		EXTI->EXTI_FTSR  |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
		EXTI->EXTI_RTSR  |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}
	uint8_t SYS_temp1,SYS_temp2,PortCode;
	SYS_temp1=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 4);
	SYS_temp2=(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 4);
	PortCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
	//2. Configure GPIO Pin in SysCFG_EXTICR
    SYSCFG_PCLK_EN();
	SYSCFG->SYSCFG_EXTICR[SYS_temp1]= PortCode<<(SYS_temp2*4);

	//3. ENABLE THE EXTI INTERRUPT DELIVERY USING IMR
	EXTI->EXTI_IMR |= (1<<pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
}
	//Configure the speed
temp=0;
temp=(pGPIOHandle->GPIO_PinConfig.GPIO_PinSpeed<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
pGPIOHandle->pGPIOx->SPEED &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Bit clear
pGPIOHandle->pGPIOx->SPEED|=temp;

	//COnfigure PUllup_Dn setting
temp=0;
temp = (pGPIOHandle->GPIO_PinConfig.GPIO_Pinpupdconrol<<(2 * pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
pGPIOHandle->pGPIOx->PULL_UP_DOWN &=~(0x3 <<(2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Bit clear
pGPIOHandle->pGPIOx->PULL_UP_DOWN|=temp;

	//COnfigure Output Type
//COnfigure PUllup_Dn setting
temp=0;
temp = (pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType<<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
pGPIOHandle->pGPIOx->OUT_TYPE &=~(0x1 <<(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber)); //Bit clear
pGPIOHandle->pGPIOx->OUT_TYPE|=temp;
	//Configure the alternate Functionality
temp=0;
if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode==GPIO_ALF_MODE)
{
	uint32_t temp1,temp2;
	temp1=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber / 8;
	temp2=pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber % 8;
	pGPIOHandle->pGPIOx->AFR[temp1] |=(0xF<<(4 * temp2));
	pGPIOHandle->pGPIOx->AFR[temp1] |=(pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode <<(4 * temp2));

}



}

void GPIO_DeInit(GPIO_RegDef_t *pGPIOx){

	if(pGPIOx==GPIOA)
			{
		        GPIOA_REG_RESET();
			}
			else if(pGPIOx==GPIOB)
			{
				GPIOB_REG_RESET();
			}
			else if(pGPIOx==GPIOC)
			{
				GPIOC_REG_RESET();
			}
			else if(pGPIOx==GPIOD)
			{
				GPIOD_REG_RESET();
			}
			else if(pGPIOx==GPIOE)
			{
				GPIOE_REG_RESET();
			}
			else if(pGPIOx==GPIOH)
			{
				GPIOH_REG_RESET();
			}


}


/*
 * Data Read and Write
 */
uint8_t GPIO_Read_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){
	uint8_t value;
	value=(uint8_t) ((pGPIOx->IDR >> PinNumber) && 0x00000001);
	return value;

}
uint16_t GPIO_Read_Input_Port(GPIO_RegDef_t *pGPIOx){

	uint16_t value;
	value=(uint16_t)pGPIOx->IDR;
	return value;

}
void GPIO_Write_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value){

	if(Value==GPIO_PIN_SET)
	{
		pGPIOx->ODR |= (1<<PinNumber);
	}
	else if(Value==GPIO_PIN_RESET)
	{
		pGPIOx->ODR &= ~(1<<PinNumber);
	}

}
void GPIO_Write_Output_Port(GPIO_RegDef_t *pGPIOx,uint8_t value){

	pGPIOx->ODR = value;
}
void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber){

	pGPIOx->ODR ^= (1<<PinNumber);

}

/*
 * GPIO Interrupt Configuration
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI){

	if(ENorDI==ENABLE)
	{
		if(IRQNumber<=31)
		{
			//Program ISER0 Register
			*ISER0_BASEADDR |= (1<<IRQNumber);
		}
		else if((IRQNumber>31) && (IRQNumber<64))
		{
			//Program ISER1 Register
			*ISER1_BASEADDR |= (1<<(IRQNumber % 32));
		}
		else if((IRQNumber>=64) && (IRQNumber<96))
		{
			//Program ISER2 Register
			*ISER2_BASEADDR |= (1<<(IRQNumber % 64));
		}
	}
	else
	{
		if(IRQNumber<=31)
		{
			//Program ICER0 Register
			*ICER0_BASEADDR |= (1<<IRQNumber);
		}
		else if((IRQNumber>31) && (IRQNumber<64))
		{
			//Program ICER1 Register
			*ICER1_BASEADDR |= (1<<(IRQNumber % 32));
		}
		else if((IRQNumber>=64) && (IRQNumber<96))
		{
			//Program ICER2 Register
			*ICER2_BASEADDR |= (1<<(IRQNumber % 64));
		}

	}

}

/*
 * GPIO Interrupt Priority Configuration
 */

void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority){
	uint8_t iprx = IRQNumber / 4;
	uint8_t iprx_section = IRQNumber % 4;
	uint8_t shift_amount = (8 *iprx_section) + (8 - NO_PR_BITS_IMPLEMENTED);

	*(NVIC_IPR_BASEADDR + (iprx * 4)) |= (IRQPriority<<shift_amount);
}

/*
 * GPIO Interrupt Handling
 */
void GPIO_IRQHandling(uint8_t PinNumber){
	//Clear the exti Pending Register corresponding to the pin number
	if(EXTI->EXTI_PR &(1<< PinNumber))
	{
		EXTI->EXTI_PR |=(1<< PinNumber);
	}

}


