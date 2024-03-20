/*
 * stm32f407xx_gpio.c
 *
 *  Created on: Mar 15, 2024
 *      Author: Huy Phan Quang
 */

#include "stm32f407xx_gpio.h"


 /******************************************************************************************
 *								Peripheral Clock setup
 ******************************************************************************************/
uint8_t GPIO_PeriClockControl(GPIO_RegMap_t *pGPIOx, uint8_t EnOrDI)
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

/******************************************************************************************
 *								Init and De-init
 ******************************************************************************************/
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
 * @Note
 *
 */
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOHandle)
{
	uint32_t tempReg = 0U;
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;

	// 1. configure the mode of GPIO pin
	if(pGPIOHandle->GPIO_PinConfig.GPIO_PinMode <= GPIO_MODE_ANALOG)
	{
		tempReg = ( pGPIOHandle->GPIO_PinConfig.GPIO_PinMode << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		// clear before setting
		pGPIOHandle->pGPIOx->MODER &= ~(0x03 << (2*pGPIOHandle->GPIO_PinConfig.GPIO_PinNumber));
		pGPIOHandle->pGPIOx->MODER |= tempReg;
	}
	else
	{
		// Interrupt mode. handler later. Do nothing.
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
uint8_t GPIO_DeInit(GPIO_RegMap_t *pGPIOx)
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

/******************************************************************************************
 *								Data read and write
 ******************************************************************************************/
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
uint8_t GPIO_WriteToOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber, uint8_t value)
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
uint8_t GPIO_WriteToOutputPort(GPIO_RegMap_t *pGPIOx, uint16_t value)
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
uint8_t GPIO_ToggleOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber)
{
	GPIO_JobResultType eLldRetVal = GPIO_JOB_OK;
	pGPIOx->ODR ^= (1 << pinNumber );
	return eLldRetVal;
}

/******************************************************************************************
 *								IRQ Configuration and ISR handling
 ******************************************************************************************/
uint8_t GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDI);
uint8_t GPIO_IRQHandling(uint8_t pinNumber);
