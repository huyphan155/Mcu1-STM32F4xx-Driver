/*
 * 005button_interrupt.c
 *
 *  Created on: May, 2024
 *      Author: Phan Quang Huy
 */

#include <stdint.h>
#include<string.h>
#include "huypq_stm32f407xx.h"

void delay()
{
	for (uint32_t i=0U; i < 50000000; i++)
	{

	}
}

int main(void)
{

	GPIO_Handle_t GpioLed, GPIOBtn;

	memset(&GpioLed,0,sizeof(GpioLed));
	memset(&GPIOBtn,0,sizeof(GpioLed));

	//this is led gpio configuration : D12
	GpioLed.pGPIOx = GPIOD;
	GpioLed.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GpioLed.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_OUT;
	GpioLed.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OUT_SP_LOW;
	GpioLed.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioLed.GPIO_PinConfig.GPIO_PinPUPD = GPIO_CONFIG_NO_PUPD;

	GPIO_Init(&GpioLed);

	//this is button gpio configuration : D5
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OUT_SP_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPUPD = GPIO_CONFIG_PU;

	GPIO_Init(&GPIOBtn);

	GPIO_WriteToOutputPin(GPIOD,GPIO_PIN_NO_12,GPIO_PIN_RESET);
	//IRQ configurations
	GPIO_IRQPriorityConfig(IRQ_NO_EXTI9_5,15);
	GPIO_IRQInterruptConfig(IRQ_NO_EXTI9_5,ENABLE);

    while(1);

}


void EXTI9_5_IRQHandler(void)
{
   /// delay(); //200ms . wait till button de-bouncing gets over
	GPIO_IRQHandling(GPIO_PIN_NO_5); //clear the pending event from exti line
	GPIO_ToggleOutputPin(GPIOD,GPIO_PIN_NO_12);
}
