/*
 * stm32f411xx_gpio.h
 *
 *  Created on: Jul 28, 2023
 *      Author: Neena
 */

#ifndef INC_STM32F411XX_GPIO_H_
#define INC_STM32F411XX_GPIO_H_

#include "stm32f411xx.h"

/*
 * GPIO pin configuration structure
 */
typedef struct
{
	uint8_t GPIO_PinNumber;          //@GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;            //@GPIO mode macros
	uint8_t GPIO_PinSpeed;
	uint8_t GPIO_Pinpupdconrol;
	uint8_t GPIO_PinOPType;
	uint8_t GPIO_PinAltFunMode;


}GPIO_PinConfig_t;

/*
 * This is handle structure for gpio pin
 */

typedef struct
{
	GPIO_RegDef_t *pGPIOx;   //this holds the base address of the gpio port to which the pin belongs
	GPIO_PinConfig_t GPIO_PinConfig;  //this holds gpio pin configuration settings
}GPIO_Handle_t;

/*
 * API for GPIOx Driver
 */
/*
 *GPIO Peripheral clock control
 */
void GPIO_PeriClockControl(GPIO_RegDef_t *pGPIOx, uint8_t ENorDI);

/*
 * GPIO Init and DeInit
 */
void GPIO_Init(GPIO_Handle_t *pGPIOHandle);
void GPIO_DeInit(GPIO_RegDef_t *pGPIOx);


/*
 * Data Read and Write
 */
uint8_t GPIO_Read_Input_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);
uint16_t GPIO_Read_Input_Port(GPIO_RegDef_t *pGPIOx);
void GPIO_Write_Output_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber, uint8_t Value);
void GPIO_Write_Output_Port(GPIO_RegDef_t *pGPIOx,uint8_t value);
void GPIO_Toggle_Pin(GPIO_RegDef_t *pGPIOx, uint8_t PinNumber);

/*
 * GPIO Interrupt Handling
 */
void GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t ENorDI);
void GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
void GPIO_IRQHandling(uint8_t PinNumber);



/*
 * GPIO_PIN_NUMBER
 */
#define GPIO_PIN_0        0
#define GPIO_PIN_1        1
#define GPIO_PIN_2        2
#define GPIO_PIN_3        3
#define GPIO_PIN_4        4
#define GPIO_PIN_5        5
#define GPIO_PIN_6        6
#define GPIO_PIN_7        7
#define GPIO_PIN_8        8
#define GPIO_PIN_9        9
#define GPIO_PIN_10       10
#define GPIO_PIN_11       11
#define GPIO_PIN_12       12
#define GPIO_PIN_13       13
#define GPIO_PIN_14       14
#define GPIO_PIN_15


/*
 * GPIO mode macros
 */
#define GPIO_INPUT_MODE            0
#define GPIO_OUTPUT_MODE           1
#define GPIO_ALF_MODE              2
#define GPIO_ANALOG_MODE           3
#define GPIO_IT_FALLING            4
#define GPIO_IT_RISING             5
#define GPIO_IT_FALLRISE           4


/*
 * GPIO Output Types
 */
#define GPIO_OP_TYPE_PP            0
#define GPIO_OP_TYPE_OD            1

/*
 * GPIO Speed modes
 */

#define GPIO_SPEED_LOW             0
#define GPIO_SPEED_MEDIUM          1
#define GPIO_SPEED_FAST            2
#define GPIO_SPEED_HIGH            3

/*
 * GPIO pin pull up and pull down configuration macros
 */

#define GPIO_NO_PUPD                 0
#define GPIO_PIN_PU                  1
#define GPIO_PIN_PD                  2

#endif /* INC_STM32F411XX_GPIO_H_ */
