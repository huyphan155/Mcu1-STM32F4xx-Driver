/*
 * 001LedToggle.c
 *
 *  Created on: Mar 20, 2024
 *      Author: Huy Phan Quang
 */

#include <stdint.h>
#include "huypq_stm32f407xx.h"

void delay()
{
	for (uint32_t i=0U; i < 50000000; i++)
	{

	}
}

int main(void)
{
	GPIO_Handle_t GpioLedToggle;
	GpioLedToggle.pGPIOx = GPIOD;
	GpioLedToggle.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLedToggle.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLedToggle.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioLedToggle.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OUT_SP_HIGH;
	GpioLedToggle.GPIO_PinConfig.GPIO_PinPUPD = GPIO_CONFIG_NO_PUPD;

	//enable clock of GPIO D
	GPIO_PeriClockControl(GPIOD,ENABLE);
	// init GPIO Port
	GPIO_Init(&GpioLedToggle);

	while(1)
	{
		//toggle LED D12
		GPIO_ToggleOutputPin(GPIOD, GPIO_PIN_NO_12);
		delay();
	}

	return 0;
}
