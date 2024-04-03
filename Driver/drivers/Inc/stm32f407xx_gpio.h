/*==================================================================================================
*   Project              : MCU1 Driver for STM32F4xx
*   Platform             : CORTEXM
*   Peripheral           : PORT
*
*   SW Version           : 1.0.0
*   Created on           : Mar, 2024
*   Author               : Huy Phan Quang
*
==================================================================================================*/
/**
*   @file    stm32f407xx_gpio.h
*
*   @brief   GPIO low-level driver header file.
*   @details GPIO low-level driver header file.
*
*/

#ifndef INC_STM32F407XX_GPIO_H_
#define INC_STM32F407XX_GPIO_H_

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/

#include "huypq_stm32f407xx.h"

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*
 * @GPIO_PIN_NUMBER
 */
#define GPIO_PIN_NO_0        0
#define GPIO_PIN_NO_1        1
#define GPIO_PIN_NO_2        2
#define GPIO_PIN_NO_3        3
#define GPIO_PIN_NO_4        4
#define GPIO_PIN_NO_5        5
#define GPIO_PIN_NO_6        6
#define GPIO_PIN_NO_7        7
#define GPIO_PIN_NO_8        8
#define GPIO_PIN_NO_9        9
#define GPIO_PIN_NO_10       10
#define GPIO_PIN_NO_11       11
#define GPIO_PIN_NO_12       12
#define GPIO_PIN_NO_13       13
#define GPIO_PIN_NO_14       14
#define GPIO_PIN_NO_15       15

/*
 * @GPIO_PIN_MODES
 * configure the I/O direction mode
 */
#define GPIO_MODE_IN         0
#define GPIO_MODE_OUT        1
#define GPIO_MODE_ALTFN      2
#define GPIO_MODE_ANALOG     3
#define GPIO_MODE_IT_FT      4            // interrupt falling edge
#define GPIO_MODE_IT_RT      5            // interrupt rising edge
#define GPIO_MODE_IT_RFT     6            // interrupt falling edge, rising edge trigger


/*
 * @GPIO_OUTPUT_TYPE
 * configure the output type of the I/O port
 */
#define GPIO_OTYPER_PP         0          // Output push-pull (reset state)
#define GPIO_OTYPER_OD         1          // Output open-drain

/*
 * @GPIO_OUTPUT_SPEED
 *  configure the I/O output speed.
 */
#define GPIO_OUT_SP_LOW          0          // Low speed
#define GPIO_OUT_SP_MED          1          // Medium speed
#define GPIO_OUT_SP_HIGH         2          // High speed
#define GPIO_OUT_SP_VRHIGH       3          // Very high speed

/*
 * @GPIO_PUPDR_CONFIG
 *  configure the I/O pull-up or pull-down
 */
#define GPIO_CONFIG_NO_PUPD        0          // No pull-up, pull-down
#define GPIO_CONFIG_PU             1          // Pull-up
#define GPIO_CONFIG_PD             2          // Pull-down
#define GPIO_CONFIG_REV            3          // Reserved

/*==================================================================================================
*                                            ENUMS
==================================================================================================*/

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/

/*---------------------------------------------------------------------------
*                         WRAPPER OF IP DRIVER LAYER
-----------------------------------------------------------------------------*/
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

/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
/*
 * Peripheral Clock setup
 */
GPIO_JobResultType GPIO_PeriClockControl(GPIO_RegMap_t *pGPIOx, uint8_t EnOrDI);

/*
 * Init and De-init
 */
GPIO_JobResultType GPIO_Init(GPIO_Handle_t *pGPIOHandle);
GPIO_JobResultType GPIO_DeInit(GPIO_RegMap_t *pGPIOx);

/*
 * Data read and write
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegMap_t *pGPIOx);
GPIO_JobResultType GPIO_WriteToOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber, uint8_t value);
GPIO_JobResultType GPIO_WriteToOutputPort(GPIO_RegMap_t *pGPIOx, uint16_t value);
GPIO_JobResultType GPIO_ToggleOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber);

/*
 * IRQ Configuration and ISR handling
 */
GPIO_JobResultType GPIO_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDI);
GPIO_JobResultType GPIO_IRQHandling(uint8_t pinNumber);
GPIO_JobResultType GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

#endif /* INC_STM32F407XX_GPIO_H_ */
