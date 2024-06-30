/*
 * 007Test_SPI_send_to_Adruino_uno.c
 *
 *  Created on: Jun, 2024
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

void delay(void)
{
	for(uint32_t i = 0 ; i < 500000 ; i ++);
}


void GPIO_Button_Init()

{
	GPIO_Handle_t GPIOBtn;
	//this is button gpio configuration : D5
	GPIOBtn.pGPIOx = GPIOD;
	GPIOBtn.GPIO_PinConfig.GPIO_PinNumber = GPIO_PIN_NO_5;
	GPIOBtn.GPIO_PinConfig.GPIO_PinMode = GPIO_MODE_IT_FT;
	GPIOBtn.GPIO_PinConfig.GPIO_PinOSpeed = GPIO_OUT_SP_HIGH;
	GPIOBtn.GPIO_PinConfig.GPIO_PinPUPD = GPIO_CONFIG_PU;

	GPIO_Init(&GPIOBtn);
}

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
	SPI2Handle.SPI_Config.SPI_SSM = SPI_SSM_DI; // hardware slave management enable
	SPI2Handle.SPI_Config.SPI_SclkSpeed = SPI_SCLK_SPEED_DIV8; // generate sclk of 2MHZ

	SPI_Init(&SPI2Handle);
}

int main(void)
{

	char user_data[] = "sent data to arduino uno r3";

	// this function is used to initialize a GPIO button
	GPIO_Button_Init();

	//this function is used to initialize the GPIO pins to behave as SPI2 pins
	GPIO_INIT_SPI2();

	//This function is used to initialize the SPI2 peripheral parameters
	SPI2_Inits();

	/*
	* SPI_SSM is disable, so enable SSOE
	* making SSOE 1 does NSS output enable.
	* The NSS pin is automatically managed by the hardware.
	* i.e when SPE=1 , NSS will be pulled to low
	* and NSS pin will be high when SPE=0
	*/
	SPI_SSOEConfig(SPI2,ENABLE);

	while(1)
	{
		//wait till button is pressed
		while(! GPIO_ReadFromInputPin(GPIOD, GPIO_PIN_NO_5));

		//to avoid button de-bouncing related issues 200ms of delay
		delay();

		//enable the SPI2 peripheral : Enable SPI_CR1_SPE bit
		SPI_PeripheralControl(SPI2,ENABLE);

		//first send length information
		uint8_t dataLen = strlen(user_data);
		SPI_SendData(SPI2,&dataLen,1);

		//sent data
		SPI_SentData(SPI2, (uint8_t*)user_data, strlen(user_data));

		// wait for the data is finish sending : check the busy flag status
		while (	SPI_GetFlagStatus(SPI2,(1 << SPI_SR_BSY)));

		//disable the SPI2 peripheral when finish sending the data
		SPI_PeripheralControl(SPI2,ENABLE);

	}

	return 0;
}





