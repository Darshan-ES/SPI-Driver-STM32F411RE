/*
 * stm32f411xx.h
 *
 *  Created on: Jul 26, 2023
 *      Author: Neena
 */

#ifndef INC_STM32F411XX_H_
#define INC_STM32F411XX_H_

#include <stdint.h>

/************************Processor specific Registers**********************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Register Address
 */
#define ISER0_BASEADDR      ((volatile uint32_t*)0xE000E100)
#define ISER1_BASEADDR      ((volatile uint32_t*)0xE000E104)
#define ISER2_BASEADDR      ((volatile uint32_t*)0xE000E108)
#define ISER3_BASEADDR      ((volatile uint32_t*)0xE000E10C)

/*
 * ARM Cortex Mx Processor NVIC ICERx Register Address
 */
#define ICER0_BASEADDR      ((volatile uint32_t*)0XE000E180)
#define ICER1_BASEADDR      ((volatile uint32_t*)0XE000E184)
#define ICER2_BASEADDR      ((volatile uint32_t*)0XE000E188)
#define ICER3_BASEADDR      ((volatile uint32_t*)0XE000E18C)

/*
 * ARM Cortex Mx Processor NVIC_IPRx Register Address
 */

#define NVIC_IPR_BASEADDR   ((volatile uint32_t*)0XE000E400)

#define NO_PR_BITS_IMPLEMENTED   4

/* base address for flash and sram */

#define FLASH_BASEADDRESS 0x08000000U     // Base address for flash memory
#define SRAM_BASEADDRESS  0x20000000U     // Base address for sram memory
#define ROM               0x1FFF0000U     // Base address for system memory


/* base address for bus peripheral bus*/
#define PERIPH_BASE       0x40000000U
#define APB1PERIPH_BASE   PERIPH_BASE
#define APB2PERIPH_BASE   0x40010000U
#define AHB1PERIPH_BASE   0x40020000U
#define AHB2PERIPH_BASE   0x50000000U


/* Define Base Address for all the peripherals on AHB1 bus */
#define GPIOA_BASEADDR    (AHB1PERIPH_BASE + 0X0000UL)
#define GPIOA             ((GPIO_RegDef_t*)GPIOA_BASEADDR)

#define GPIOB_BASEADDR    (AHB1PERIPH_BASE + 0X0400UL)
#define GPIOB             ((GPIO_RegDef_t*)GPIOB_BASEADDR)

#define GPIOC_BASEADDR    (AHB1PERIPH_BASE + 0X0800UL)
#define GPIOC             ((GPIO_RegDef_t*)GPIOC_BASEADDR)

#define GPIOD_BASEADDR    (AHB1PERIPH_BASE + 0X0C00UL)
#define GPIOD             ((GPIO_RegDef_t*)GPIOD_BASEADDR)

#define GPIOE_BASEADDR    (AHB1PERIPH_BASE + 0X1000UL)
#define GPIOE             ((GPIO_RegDef_t*)GPIOE_BASEADDR)

#define GPIOH_BASEADDR    (AHB1PERIPH_BASE + 0X1C00UL)
#define GPIOH             ((GPIO_RegDef_t*)GPIOH_BASEADDR)

#define CRC_BASEADDR      (AHB1PERIPH_BASE + 0X3000UL)

#define RCC_BASEADDR      (AHB1PERIPH_BASE + 0X3800UL)
#define RCC               ((RCC_RegDef_t*)RCC_BASEADDR)

#define FLASH_INTERFACE_BASEADDR      (AHB1PERIPH_BASE + 0X3C00UL)
#define DMA1_BASEADDR     (AHB1PERIPH_BASE + 0X6000UL)
#define DMA2_BASEADDR     (AHB1PERIPH_BASE + 0X6400UL)


/* Define Base Address for the peripherals on APB1 bus */
#define I2C1_BASEADDR     (APB1PERIPH_BASE + 0x5400UL)
#define I2C2_BASEADDR     (APB1PERIPH_BASE + 0x5800UL)
#define I2C3_BASEADDR     (APB1PERIPH_BASE + 0x5C00UL)

#define SPI2_BASEADDR     (APB1PERIPH_BASE + 0x3800UL)
#define SPI2			  ((SPI_RegDef_t*)SPI2_BASEADDR)
#define SPI3_BASEADDR     (APB1PERIPH_BASE + 0x3C00UL)
#define SPI3			  ((SPI_RegDef_t*)SPI3_BASEADDR)
#define USART2_BASEADDR   (APB1PERIPH_BASE + 0x4400UL)




/* Define Base Address for the peripherals on APB2 bus */
#define EXTI_BASEADDR     (APB2PERIPH_BASE + 0x3C00UL)
#define EXTI			  ((EXTI_RegDef_t*)EXTI_BASEADDR)

#define SPI1_BASEADDR     (APB2PERIPH_BASE + 0x3000UL)
#define SPI1			  ((SPI_RegDef_t*)SPI1_BASEADDR)
#define SPI4_BASEADDR     (APB2PERIPH_BASE + 0x3400UL)
#define SPI4			  ((SPI_RegDef_t*)SPI4_BASEADDR)
#define SPI5_BASEADDR     (APB2PERIPH_BASE + 0x5000UL)
#define SPI5			  ((SPI_RegDef_t*)SPI5_BASEADDR)

#define USART6_BASEADDR   (APB2PERIPH_BASE + 0x1400UL)
#define USART1_BASEADDR   (APB2PERIPH_BASE + 0x1000UL)

#define SYSCFG_BASEADDR   (APB2PERIPH_BASE + 0x3800UL)
#define SYSCFG            ((SYSCFG_RegDef_t*)SYSCFG_BASEADDR)



/* GPIO Register Peripheral Structure */

typedef struct{
	volatile uint32_t MODE;
	volatile uint32_t OUT_TYPE;
	volatile uint32_t SPEED;
	volatile uint32_t PULL_UP_DOWN;
	volatile uint32_t IDR;
	volatile uint32_t ODR;
	volatile uint32_t BSRR;
	volatile uint32_t LCK;
	volatile uint32_t AFR[2];

}GPIO_RegDef_t;

//GPIO_RegDef_t *pGPIOA = GPIOA;

typedef struct{
	volatile uint32_t RCC_CR;
	volatile uint32_t RCC_PLLCFGR;
	volatile uint32_t RCC_CFGR;
	volatile uint32_t RCC_CIR;
	volatile uint32_t RCC_AHB1RSTR;
	volatile uint32_t RCC_AHB2RSTR;
	uint32_t Reserved1;
	uint32_t Reserved2;
	volatile uint32_t RCC_APB1RSTR;
	volatile uint32_t RCC_APB2RSTR;
	uint32_t Reserved3;
	uint32_t Reserved4;
	volatile uint32_t RCC_AHB1ENR;
	volatile uint32_t RCC_AHB2ENR;
	uint32_t Reserved5;
	uint32_t Reserved6;
	volatile uint32_t RCC_APB1ENR;
	volatile uint32_t RCC_APB2ENR;
	uint32_t Reserved7;
	uint32_t Reserved8;
	volatile uint32_t RCC_AHB1LPENR;
	volatile uint32_t RCC_AHB2LPENR;
	uint32_t Reserved9;
	uint32_t Reserved10;
	volatile uint32_t RCC_APB1LPENR;
	volatile uint32_t RCC_APB2LPENR;
	uint32_t Reserved12;
	uint32_t Reserved13;
	volatile uint32_t RCC_BDCR;
	volatile uint32_t RCC_CSR;
	uint32_t Reserved14;
	uint32_t Reserved15;
	volatile uint32_t RCC_SSCGR;
	volatile uint32_t RCC_PLLI2SCFGR;
	uint32_t Reserved16;
	volatile uint32_t RCC_DCKCFGR;

}RCC_RegDef_t;

typedef struct{
	volatile uint32_t EXTI_IMR;
	volatile uint32_t EXTI_EMR;
	volatile uint32_t EXTI_RTSR;
	volatile uint32_t EXTI_FTSR;
	volatile uint32_t EXTI_SWIER;
	volatile uint32_t EXTI_PR;

}EXTI_RegDef_t;

typedef struct{
	volatile uint32_t SYSCFG_MEMRMP;
	volatile uint32_t SYSCFG_PMC;
	volatile uint32_t SYSCFG_EXTICR[4];
	uint32_t Reserved[2];
	volatile uint32_t SYSCFG_CMPCRC;

}SYSCFG_RegDef_t;


/*
 * SPI Register Definition
 */

typedef struct{
	volatile uint32_t SPI_CR1;
	volatile uint32_t SPI_CR2;
	volatile uint32_t SPI_SR;
	volatile uint32_t SPI_DR;
	volatile uint32_t SPI_CRCPR;
	volatile uint32_t SPI_RXCRCR;
	volatile uint32_t SPI_TXCRCR;
	volatile uint32_t SPI_I2SCFGR;
	volatile uint32_t SPI_I2SPR;

}SPI_RegDef_t;



/*
 * GPIO Peripheral Reset
 */

#define GPIOA_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<0)); (RCC->RCC_AHB1RSTR &=~(1<<0));} while(0)
#define GPIOB_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<1)); (RCC->RCC_AHB1RSTR &=~(1<<1));} while(0)
#define GPIOC_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<2)); (RCC->RCC_AHB1RSTR &=~(1<<2));} while(0)
#define GPIOD_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<3)); (RCC->RCC_AHB1RSTR &=~(1<<3));} while(0)
#define GPIOE_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<4)); (RCC->RCC_AHB1RSTR &=~(1<<4));} while(0)
#define GPIOH_REG_RESET()    do{(RCC->RCC_AHB1RSTR |= (1<<5)); (RCC->RCC_AHB1RSTR &=~(1<<5));} while(0)

/*
 * Clock enable for GPIOx peripheral
 */

#define GPIOA_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<0))
#define GPIOB_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<1))
#define GPIOC_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<2))
#define GPIOD_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<3))
#define GPIOE_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<4))
#define GPIOH_PCLK_EN()    (RCC->RCC_AHB1ENR |= (1<<7))

/*
 * Clock disable for GPIOx peripheral
 */
#define GPIOA_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<0))
#define GPIOB_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<1))
#define GPIOC_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<2))
#define GPIOD_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<3))
#define GPIOE_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<4))
#define GPIOH_PCLK_DI()    (RCC->RCC_AHB1ENR &= ~(1<<7))

/*
 * Clock enable for I2Cx peripheral
 */
#define I2C1_PCLK_EN()     (RCC->RCC_APB1ENR |= (1<<21))
#define I2C2_PCLK_EN()     (RCC->RCC_APB1ENR |= (1<<22))
#define I2C3_PCLK_EN()     (RCC->RCC_APB1ENR |= (1<<23))

/*
 * Clock enable for SPIx peripheral
 */

#define SPI1_PCLK_EN()		(RCC->RCC_APB2ENR |= (1<<12))
#define SPI2_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<14))
#define SPI3_PCLK_EN()		(RCC->RCC_APB1ENR |= (1<<15))
#define SPI4_PCLK_EN()		(RCC->RCC_APB2ENR |= (1<<13))
#define SPI5_PCLK_EN()		(RCC->RCC_APB2ENR |= (1<<20))

/*
 * Clock DISABLE for SPIx peripheral
 */

#define SPI1_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(1<<12))
#define SPI2_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<14))
#define SPI3_PCLK_DI()		(RCC->RCC_APB1ENR &= ~(1<<15))
#define SPI4_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(1<<13))
#define SPI5_PCLK_DI()		(RCC->RCC_APB2ENR &= ~(1<<20))

/*
 * SPI RESET
 */
#define SPI1_REG_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<12)); (RCC->RCC_APB2RSTR &=~(1<<12));} while(0)
#define SPI2_REG_RESET()	do{(RCC->RCC_APB1RSTR |= (1<<14)); (RCC->RCC_APB1RSTR &=~(1<<14));} while(0)
#define SPI3_REG_RESET()	do{(RCC->RCC_APB1RSTR |= (1<<15)); (RCC->RCC_APB1RSTR &=~(1<<15));} while(0)
#define SPI4_REG_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<13)); (RCC->RCC_APB2RSTR &=~(1<<13));} while(0)
#define SPI5_REG_RESET()	do{(RCC->RCC_APB2RSTR |= (1<<20)); (RCC->RCC_APB2RSTR &=~(1<<20));} while(0)

/*
 * Clock enable for SYSCFG peripheral
 */
#define SYSCFG_PCLK_EN()   (RCC->RCC_APB2ENR |=(1<<14));

/*
 *PortCode for EXTI
 */
#define GPIO_BASEADDR_TO_CODE(x)    ((x==GPIOA)?0:\
		    						(x==GPIOB)?1:\
		    						(x==GPIOC)?2:\
		    						(x==GPIOD)?3:\
		    						(x==GPIOE)?4:\
		    						(x==GPIOH)?7:0)



/*
 * Generic Macros
 */
#define ENABLE           1
#define DISABLE          0
#define SET              ENABLE
#define RESET            DISABLE
#define GPIO_PIN_SET     SET
#define GPIO_PIN_RESET   RESET
#define FLAG_RESET       RESET
#define FLAG_SET         SET


/*
 * INTERRUPT REQUEST NUMBER (IRQ)
 */

#define IRQ_NO_EXTI0      6
#define IRQ_NO_EXTI1      7
#define IRQ_NO_EXTI2      8
#define IRQ_NO_EXTI3      9
#define IRQ_NO_EXTI4      10
#define IRQ_NO_EXTI9_5    23
#define IRQ_NO_EXTI15_10  40




#endif /* INC_STM32F411XX_H_ */
