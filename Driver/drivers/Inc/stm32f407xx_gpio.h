/*
 * stm32f407xx_gpio.h
 *
 *  Created on: Mar 15, 2024
 *      Author: Huy Phan Quang
 */

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

#include "huypq_stm32f407xx.h"

/***********************************************************
 *             WRAPPER OF IP DRIVER LAYER
 ***********************************************************/
/*
 * Hold GPIO pin configuration setting
 */
typedef struct
{
	//uint8_t GPIO_PortName;      // Port Name
	uint8_t GPIO_PinNumber;     // Pin number. Refer to @GPIO_PIN_NUMBER
	uint8_t GPIO_PinMode;       // Pin mode. Refer to @GPIO_PIN_MODES
	uint8_t GPIO_PinOSpeed;     // Pin output speed. Refer to @GPIO_OUTPUT_SPEED
	uint8_t GPIO_PinPUPD;       // Pin pull up / pull down control. Refer to @GPIO_PUPDR_CONFIG
	uint8_t GPIO_PinOPType;     // Pin output Type. Refer to @GPIO_OUTPUT_TYPE
	uint8_t GPIO_PinAltFunMode; // Pin alternate function mode.
}GPIO_PinConfig_t;

/*
 * Handler structure for GPIO pin
 */
typedef struct
{
	GPIO_RegMap_t *pGPIOx; // hold the base address of GPIO port which the pin belong
	GPIO_PinConfig_t GPIO_PinConfig; // Hold GPIO pin configuration setting
}GPIO_Handle_t;

/***********************************************************
*                      GPIO API
 ***********************************************************/

/*
 * Peripheral Clock setup
 */
uint8_t GPIO_PeriClockControl(GPIO_RegMap_t *pGPIOx, uint8_t EnOrDI);

/*
 * Init and De-init
 */
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOx);
uint8_t GPIO_DeInit(GPIO_Handle_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOx);
uint8_t GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber, uint8_t value);
uint8_t GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOx, uint16_t value);
uint8_t GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and ISR handling
 */
uint8_t GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDI);
uint8_t GPIO_IRQHandling(uint8_t pinNumber);

#endif /* INC_STM32F407XX_GPIO_H_ */
