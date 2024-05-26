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
*   @file    stm32f407xx_gpio.c
*
*   @brief   GPIO low-level driver implementations.
*   @details GPIO low-level driver implementations.
*
*/

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "stm32f407xx_gpio.h"

/*==================================================================================================
*                          LOCAL TYPEDEFS (STRUCTURES, UNIONS, ENUMS)
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL MACROS
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
*                                       LOCAL VARIABLES
==================================================================================================*/
/*==================================================================================================
*                                      GLOBAL CONSTANTS
==================================================================================================*/
/*==================================================================================================
                                    GLOBAL VARIABLES
==================================================================================================*/
/*==================================================================================================
*                                    LOCAL FUNCTION PROTOTYPES
==================================================================================================*/
/*==================================================================================================
*                                         LOCAL FUNCTIONS
==================================================================================================*/
/*==================================================================================================
*                                        GLOBAL FUNCTIONS
==================================================================================================*/

/*---------------------------------------------------------------------------
*                         Peripheral Clock setup
-----------------------------------------------------------------------------*/
/**
 * @brief       Control peripheral clock for a GPIO port
 *
 * @details     This function enables or disables the peripheral clock for a GPIO port.
 *
 * @param[in]   pGPIOx : hold the base address of GPIOx
 * @param[in]   EnOrDI : Enable or disable operation, use ENABLE or DISABLE macros
 *
 * @return      GPIO_JobResultType
 * @retval      GPIO_JOB_OK : The job has been finished successfully
 * @retval      OTHER :  The job fail
 *
 * @Note
 */
GPIO_JobResultType GPIO_PeriClockControl(GPIO_RegMap_t *pGPIOx, uint8_t EnOrDI)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	if(ENABLE == EnOrDI)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERIF_CLK_EB();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PERIF_CLK_EB();
		}
	}
	else if(DISABLE == EnOrDI)
	{
		if(pGPIOx == GPIOA)
		{
			GPIOA_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOB)
		{
			GPIOB_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOC)
		{
			GPIOC_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOD)
		{
			GPIOD_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOE)
		{
			GPIOE_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOF)
		{
			GPIOF_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOG)
		{
			GPIOG_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOH)
		{
			GPIOH_PERIF_CLK_DI();
		}
		else if(pGPIOx == GPIOI)
		{
			GPIOI_PERIF_CLK_DI();
		}
	}
	else
	{
		eLldRetVal = GPIO_BLOCK_INVALID;
	}
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                         Init and De-init
-----------------------------------------------------------------------------*/
/**
 * @brief        GPIO init
 *
 * @details      configure GPIO mode, Speed, PUPD , Output Type and alternate functionality (if is the case)
 *
 * @param[in]    pGPIOHandle : struct hold the base address of GPIO port and PIO pin configuration setting
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note          step to Configure GPIO interrupt :
 * 				  E0. pin must be in input mode.
 * 				  E1. Configure the edge trigger (RT,FT,RFT )
 * 				  E2. Enable interrupt delivery from peripheral to Processor (peripheral side )
 * 				  E3. Identify IRQ number which processor accepts the interrupt from that pin
 * 				  E4. configure the IRQ priority for the identified IRQ number(processor side )
 * 			      E5. Enable interrupt reception on that IRQ number (processor side)
 * 			      E6. Implement IRQ handler
 *
 */
GPIO_JobResultType GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t tempReg = 0U;
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;

	//peripheral clock enable
	GPIO_PeriClockControl(pGPIOHandle->pGPIOx, ENABLE);

	// 1. configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// clear before setting
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= tempReg; // setting
	}
	else // interrupt mode
	{
		//E0. pin must be in input mode.
		//set as input mode 00
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));

		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_FT)
		{
			// E1. Configure the edge trigger (FT)
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //disable RT
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RT)
		{
			// E1. Configure the edge trigger (RT)
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_FTSR &= ~(1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); //disable FT
		}
		else if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_IT_RFT)
		{
			// E1. Configure the edge trigger (RFT)
			EXTI->EXTI_FTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
			EXTI->EXTI_RTSR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber); // enable both RT & FT
		}
		// Configure the GPIO port selection in SYSCFG
		uint8_t temp1 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber/4; //get the right SYSCFG_EXTICRx register
		uint8_t temp2 = pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber%4; //get the right position
		uint8_t portCode = GPIO_BASEADDR_TO_CODE(pGPIOHandle->pGPIOx);
		SYSCFG_PERIF_CLK_EB(); // enable clock for SYSCFG
		SYSCFG->SYSCFG_EXTICR[temp1] = portCode << (temp2*4);

		// E2. Enable interrupt delivery from peripheral to Processor (peripheral side )
		//. Enable the EXTI interrupt delivery using IMR
		EXTI->EXTI_IMR |= (1 << pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber);
	}

	// reset temp Register
	tempReg = 0;

	// 2. configure the Speed
	tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOSpeed << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// clear before setting
	pGPIOHandle->pGPIOx->OSPEEDR &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OSPEEDR |= tempReg;
	// reset temp Register
	tempReg = 0;

	// 3. configure the PUPD settings
	tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinPUPD << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// clear before setting
	pGPIOHandle->pGPIOx->PUPDR &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->PUPDR |= tempReg;
	// reset temp Register
	tempReg = 0;

	// 4. configure the OPType
	tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinOPType << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	// clear before setting
	pGPIOHandle->pGPIOx->OTYPER &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
	pGPIOHandle->pGPIOx->OTYPER |= tempReg;
	// reset temp Register
	tempReg = 0;

	//5. Configure the alternate functionality
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode == GPIO_MODE_ALTFN)
	{
		if(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber < 8U)
		{
			tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			// clear before setting
			pGPIOHandle->pGPIOx->AFRL &= ~(0x0F << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRL |= tempReg;
		}
		else
		{
			tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinAltFunMode << (4*(pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber-8)));
			// clear before setting
			pGPIOHandle->pGPIOx->AFRH &= ~(0x0F << (4*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
			pGPIOHandle->pGPIOx->AFRH |= tempReg;
		}
		// reset temp Register
		tempReg = 0;
	}

	return eLldRetVal;
}

/**
 * @brief        GPIO De-init
 *
 * @details      Reset GPIO port by RCC register
 *
 * @param[in]    pGPIOx : hold the base address of GPIOx
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note
 *
 */
GPIO_JobResultType GPIO_DeInit(GPIO_RegMap_t *pGPIOx)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	if(pGPIOx == GPIOA)
	{
		GPIOA_REG_RESET();
	}
	else if(pGPIOx == GPIOB)
	{
		GPIOB_REG_RESET();
	}
	else if(pGPIOx == GPIOC)
	{
		GPIOC_REG_RESET();
	}
	else if(pGPIOx == GPIOD)
	{
		GPIOD_REG_RESET();
	}
	else if(pGPIOx == GPIOE)
	{
		GPIOE_REG_RESET();
	}
	else if(pGPIOx == GPIOF)
	{
		GPIOF_REG_RESET();
	}
	else if(pGPIOx == GPIOG)
	{
		GPIOG_REG_RESET();
	}
	else if(pGPIOx == GPIOH)
	{
		GPIOH_REG_RESET();
	}
	else if(pGPIOx == GPIOI)
	{
		GPIOI_REG_RESET();
	}
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                         Data read and write
-----------------------------------------------------------------------------*/
/**
 * @brief        Reads the input data register (IDR) of a GPIO Pin
 *
 * @details      This function reads the current state of a input pin of a GPIO port
 *               by accessing the input data register (IDR) of the specified GPIO port
 *               and extracting the state of the pin indicated by the given pin number.
 *
 * @param[in]    pGPIOx : hold the base address of GPIOx
 *
 * @return       value
 * @retval       1 or 0
 *
 * @Note         shift the value to the LSB. Then performs a bitwise AND operation
 * 				 with 1 to extract only the least significant bit
 *
 */
uint8_t GPIO_ReadFromInputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber)
{
	uint8_t value = 0U;
	value = (uint8_t)(( pGPIOx->IDR >> pinNumber ) & (0x01));
	return value;
}

/**
 * @brief        Reads the input data register (IDR) of a GPIO port
 *
 * @details      This function reads the current state of all pins of a GPIO port
 *               by accessing the input data register (IDR) of the specified GPIO port.
 *
 * @param[in]    pGPIOx : hold the base address of GPIOx
 *
 * @return       value : 16-bit unsigned integer representing the state of all pins of the GPIO port.
 * @retval       1 or 0
 *
 * @Note
 *
 */
uint16_t GPIO_ReadFromInputPort(GPIO_RegMap_t *pGPIOx)
{
	uint16_t value = 0U;
	value = (uint16_t)(pGPIOx->IDR);
	return value;
}

/**
 * @brief        Write to output data register (ODR) of a GPIO pin
 *
 * @details      This function write the state to a pins of a GPIO port
 *               by accessing the output data register (IDR) of the specified pin in a GPIO port.
 *
 * @param[in]    pGPIOx    : hold the base address of GPIOx
 * @param[in]    pinNumber : pin number
 * @param[in]    value     : data to write : 1 or 0
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note
 *
 */
GPIO_JobResultType GPIO_WriteToOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber, uint8_t value)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	if (value == GPIO_PIN_SET)
	{
		// write 1
		pGPIOx->ODR |= (1 << pinNumber );
	}
	else
	{
		// write 0
		pGPIOx->ODR &= ~(1 << pinNumber );
	}
	return eLldRetVal;
}

/**
 * @brief        Write to output data register (ODR) of a GPIO port
 *
 * @details      This function write the state to all pins of a GPIO port
 *               by accessing the output data register (IDR) of the specified GPIO port.
 *
 * @param[in]    pGPIOx    : hold the base address of GPIOx
 * @param[in]    value     : data to write to GPIO port
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note
 *
 */
GPIO_JobResultType GPIO_WriteToOutputPort(GPIO_RegMap_t *pGPIOx, uint16_t value)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	pGPIOx->ODR = value;
	return eLldRetVal;
}

/**
 * @brief        Toggle output of a specific pin in a GPIO port
 *
 * @details      This function toggle the state of a GPIO pin in a GPIO port
 *               by using XOR operator.
 *
 * @param[in]    pGPIOx    : hold the base address of GPIOx
 * @param[in]    pinNumber : pin number
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note
 *
 */
GPIO_JobResultType GPIO_ToggleOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	pGPIOx->ODR ^= (1 << pinNumber );
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                        IRQ Configuration and ISR handling
-----------------------------------------------------------------------------*/
/**
 * @brief Configures the interrupt for a given IRQ number.
 *
 * @details Enables or disables the interrupt based on the provided parameters.
 *
 * @param[in] IRQNumber : The IRQ number to configure.
 * @param[in] EnOrDI    : Specifies whether to enable or disable the interrupt (ENABLE or DISABLE).
 *
 * @return GPIO_JobResultType
 * @retval GPIO_JOB_OK   The job has been finished successfully.
 * @retval OTHER         The job failed.
 *
 * @note This function configures interrupt settings by programming NVIC ISER/ICER registers.
 * 		  E5. Enable interrupt reception on that IRQ number (processor side)
 */
GPIO_JobResultType GPIO_IRQInterruptConfig(uint8_t IRQNumber, uint8_t EnOrDI)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	if(EnOrDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	return eLldRetVal;
}

/**
 * @brief Configures the priority for a given IRQ number.
 *
 * @details Sets the priority level for the specified IRQ.
 *
 * @param[in] IRQNumber   :    The IRQ number to configure.
 * @param[in] IRQPriority :  The priority level for the IRQ.
 *
 * @return GPIO_JobResultType
 * @retval GPIO_JOB_OK   The job has been finished successfully.
 * @retval OTHER         The job failed.
 *
 * @note This function configures the priority of the IRQ by writing to NVIC IPR registers.
 *       E4. configure the IRQ priority for the identified IRQ number(processor side )
 */
GPIO_JobResultType GPIO_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	/*1. find out the ipr register of IRQNumber*/
	uint8_t iprx = IRQNumber / 4 ;
	/*2. tim ra field(8 bit) quy dinh priority cua loai IRQ*/
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8 - NUM_PR_BITS_IMPLEMENTED);

	/*con tro kieu uint32 nen chi can + iprx de ra duoc dia chi thanh ghi*/
	*(NVIC_IPR_BASEADDR + iprx) |= ( IRQPriority << shift_amount);
	return eLldRetVal;
}

/**
 * @brief        GPIO IRQ handling
 *
 * @details      Clears the pending interrupt flag corresponding to the specified pin number.
 *
 * @param[in]    pinNumber : pin number
 *
 * @return       GPIO_JobResultType
 * @retval       GPIO_JOB_OK : The job has been finished successfully
 * @retval       OTHER :  The job fail
 *
 * @Note         E6. Implement IRQ handler
 *               IRQ handle should be implement in application code,
 *               and call to this function form main.c.
 *
 */
GPIO_JobResultType GPIO_IRQHandling(uint8_t pinNumber)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	//clear the exti PR register corresponding to the pin number
	if (EXTI->EXTI_PR & (1 << pinNumber))
	{
		//clear by write 1 to the PR register
		EXTI->EXTI_PR |= (1 << pinNumber);
	}
	return eLldRetVal;
}
