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
uint8_t GPIO_ReadFromInputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_RegMap_t *pGPIOx);
uint8_t GPIO_WriteToOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber, uint8_t value);
uint8_t GPIO_WriteToOutputPort(GPIO_RegMap_t *pGPIOx, uint16_t value);
uint8_t GPIO_ToggleOutputPin(GPIO_RegMap_t *pGPIOx, uint8_t pinNumber);

/******************************************************************************************
 *								IRQ Configuration and ISR handling
 ******************************************************************************************/
uint8_t GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDI);
uint8_t GPIO_IRQHandling(uint8_t pinNumber);
