/*
 * 006Test_SPI_send.c
 *
 *  Created on: Apr, 2024
 *      Author: Huy Phan Quang
 */


/*
 * from the data sheet table 9 : Alternate function mapping
 * SPI2
 * PB12 - SPI2_NSS
 * PB13 - SPI2_SCK
 * PB14 - SPI2_MISO
 * PB15 - SPI2_MOSI
 * Alternate function mode : 5
 */

#include <stdint.h>
#include <string.h>
#include "huypq_stm32f407xx.h"

void GPIO_INIT_SPI2()
{
	GPIO_Handle_t GpioSpi2;
	GpioSpi2.pGPIOx = GPIOB;
	GpioSpi2.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_ALTFN;
	GpioSpi2.GPIO_PinConfig.GPIO_PinAltFunMode = 5;
	GpioSpi2.GPIO_PinConfig.GPIO_PinOPType = GPIO_OTYPER_PP;
	GpioSpi2.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OUT_SP_HIGH;
	GpioSpi2.GPIO_PinConfig.GPIO_PinPUPD = GPIO_CONFIG_NO_PUPD;

	//PB13 - SPI2_SCK
	GpioSpi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_13;
	// init GPIO Port
	GPIO_Init(&GpioSpi2);

	//PB15 - SPI2_MOSI
	GpioSpi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_15;
	GPIO_Init(&GpioSpi2);

	//PB14 - SPI2_MISO
	GpioSpi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_14;
	GPIO_Init(&GpioSpi2);

	//PB12 - SPI2_NSS
	GpioSpi2.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_12;
	GPIO_Init(&GpioSpi2);
}

void SPI2_Inits()
{
	SPI_Handle_t SPI2Handle;

	SPI2Handle.pSPIx = SPI2;
	SPI2Handle.SPI_Config.SPI_BusConfig = SPI_BUS_CONFIG_FD;
	SPI2Handle.SPI_Config.SPI_CPHA = SPI_CPHA_LOW;
	SPI2Handle.SPI_Config.SPI_CPOL = SPI_CPOL_LOW;
	SPI2Handle.SPI_Config.SPI_DFF = SPI_DFF_8_BIT;
	SPI2Handle.SPI_Config.SPI_DeviceMode = SPI_DEVICE_MODE_MASTER;
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_EN;
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV2; // generate sclk of 8MHZ

	SPI_Init(&SPI2Handle);
}

int main(void)
{

	char user_data[] = "Hello world";

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	GPIO_INIT_SPI2();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	//SPI_SSM is enable, so enable SSI to avoid MODF error
	SPI_SSIConfig(SPI2,ENABLE);

	//enable the SPI2 peripheral
	SPI_PeripheralControl(SPI2,ENABLE);

	//sent data
	SPI_SentData(SPI2, (uint8_t*)user_data, strlen(user_data));

	//disable the SPI2 peripheral when finish sending the data
	SPI_PeripheralControl(SPI2,ENABLE);

	while(1);

	return 0;
}



