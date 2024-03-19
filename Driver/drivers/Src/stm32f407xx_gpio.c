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
uint8_t GPIO_Init(GPIO_Handle_t *pGPIOx);
uint8_t GPIO_DeInit(GPIO_Handle_t *pGPIOx);

/******************************************************************************************
 *								Data read and write
 ******************************************************************************************/
uint8_t GPIO_ReadFromInputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber);
uint16_t GPIO_ReadFromInputPort(GPIO_Handle_t *pGPIOx);
uint8_t GPIO_WriteToOutputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber, uint8_t value);
uint8_t GPIO_WriteToOutputPort(GPIO_Handle_t *pGPIOx, uint16_t value);
uint8_t GPIO_ToggleOutputPin(GPIO_Handle_t *pGPIOx, uint8_t pinNumber);

/******************************************************************************************
 *								IRQ Configuration and ISR handling
 ******************************************************************************************/
uint8_t GPIO_IRQConfig(uint8_t IRQNumber,uint8_t IRQPriority,uint8_t EnOrDI);
uint8_t GPIO_IRQHandling(uint8_t pinNumber);
