/*==================================================================================================
*   Project              : MCU1 Driver for STM32F4xx
*   Platform             : CORTEXM
*   Peripheral           : SPI
*
*   SW Version           : 1.0.0
*   Created on           : Mar, 2024
*   Author               : Huy Phan Quang
*
==================================================================================================*/
/**
*   @file    stm32f407xx_spi.c
*
*   @brief   SPI low-level driver implementations.
*   @details SPI low-level driver implementations.
*
*/

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/
#include "stm32f407xx_spi.h"

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
 * @brief       Controls peripheral clock for SPI module.
 *
 * @details     This function enables or disables the peripheral clock for the specified SPI module.
 *
 * @param[in]   pSPIx  : hold the base address of pSPIx
 * @param[in]   EnOrDI : Enable or disable operation, use ENABLE or DISABLE macros
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK : The job has been finished successfully.
 * @retval      OTHER : The job failed.
 *
 * @note
 */
Spi_JobResultType SPI_PeriClockControl(SPI_RegMap_t *pSPIx, uint8_t EnOrDI)
{
    Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(ENABLE == EnOrDI)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERIF_CLK_EB();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERIF_CLK_EB();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERIF_CLK_EB();
		}
	}
	else if(DISABLE == EnOrDI)
	{
		if(pSPIx == SPI1)
		{
			SPI1_PERIF_CLK_DI();
		}
		else if(pSPIx == SPI2)
		{
			SPI2_PERIF_CLK_DI();
		}
		else if(pSPIx == SPI3)
		{
			SPI3_PERIF_CLK_DI();
		}
	}
	else
	{
		eLldRetVal = SPI_JOB_FAILED;
	}
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                         Init and De-init
-----------------------------------------------------------------------------*/
/**
 * @brief       Initialize the SPI module.
 *
 * @details     This function initializes the SPI module based on the provided configuration parameters.
 * 				Such as mode, communication mode, clock speed, data frame format, clock polarity, clock phase, and software slave management
 *
 * @param[in]   pSPIHandle :  Pointer to the SPI handle structure containing configuration parameters.
 *
 * @return      Spi_JobResultTyp
 * @retval      SPI_JOB_OK: The SPI module has been initialized successfully.
 * @retval      OTHER : The job failed.
 *
 * @note
 */

Spi_JobResultType SPI_Init(SPI_Handle_t *pSPIHandle)
{
	//refere to RM0090 28.5
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;

	//peripheral clock enable
	SPI_PeriClockControl(pSPIHandle->pSPIx, ENABLE);

	// clear the SPI_CR1 register
	pSPIHandle->pSPIx->SPI_CR1 = 0;
	// 1. configure the mode of SPI
	if(pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_MASTER)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_MSTR);
	}
	else if (pSPIHandle->SPI_Config.SPI_DeviceMode == SPI_DEVICE_MODE_SLAVE)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_MSTR);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	// 2. configure the communication mode of SPI
	if(pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_HD)
	{
		//BIDIMODE set
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_FD)
	{
		//BIDIMODE clear
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_BIDIMODE);
	}
	else if (pSPIHandle->SPI_Config.SPI_BusConfig == SPI_BUS_CONFIG_SIMPLEX_RXONLY)
	{
		//BIDIMODE clear
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_BIDIMODE);
		// RXONLY set
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_RXONLY);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	// 3. configure the clock speed
	pSPIHandle->pSPIx->SPI_CR1 |= (pSPIHandle->SPI_Config.SPI_SclkSpeed<<SPI_CR1_BR);
	// 4. configure the data frame format
	if(pSPIHandle->SPI_Config.SPI_DFF == SPI_DFF_8_BIT)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_DFF);
	}
	else if (pSPIHandle->SPI_Config.SPI_DFF == SPI_DFF_16_BIT)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_DFF);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	// 5. Cpol configuration
	if(pSPIHandle->SPI_Config.SPI_CPOL == SPI_CPOL_LOW)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_CPOL);
	}
	else if (pSPIHandle->SPI_Config.SPI_CPOL == SPI_CPOL_HIGH)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_CPOL);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	// 6. Cpha configuration.
	if(pSPIHandle->SPI_Config.SPI_CPHA == SPI_CPHA_LOW)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_CPHA);
	}
	else if (pSPIHandle->SPI_Config.SPI_CPHA == SPI_CPHA_HIGH)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_CPHA);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	// 7. configure the Software slave management
	if(pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_DI)
	{
		pSPIHandle->pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SSM);
	}
	else if (pSPIHandle->SPI_Config.SPI_SSM == SPI_SSM_EN)
	{
		pSPIHandle->pSPIx->SPI_CR1 |= (1<<SPI_CR1_SSM);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	return eLldRetVal;
}


/**
 * @brief       SPI module De-init.
 *
 * @details     Reset SPI module by RCC register
 *
 * @param[in]   pSPIx  : Pointer to the SPI handle structure containing configuration parameters.
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK : The job has been finished successfully.
 * @retval      OTHER : The job failed.
 *
 * @note
 */
Spi_JobResultType SPI_DeInit(SPI_RegMap_t *pSPIx)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(pSPIx == SPI1)
	{
		SPI1_REG_RESET();
	}
	else if(pSPIx == SPI2)
	{
		SPI2_REG_RESET();
	}
	else if(pSPIx == SPI3)
	{
		SPI3_REG_RESET();
	}
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                         Data Sent and Receive
-----------------------------------------------------------------------------*/
/**
 * @brief       Send data over SPI.
 *
 * @details     This function sends data over SPI by continuously transmitting data until the specified length is reached.
 *              This function waits for the transmit buffer to be empty
 *              before sending data and handles both 8-bit and 16-bit data transmission based on the configured data frame format (DFF).
 *
 * @param[in]   pSPIx : Pointer to the SPI peripheral register map.
 * @param[in]   pTxBuffer :  Pointer to the transmit buffer containing data to be sent.
 * @param[in]   Len : Length of the data to be sent.
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note        Blocking function
 */
Spi_JobResultType SPI_SentData(SPI_RegMap_t *pSPIx, Spi_BufferSize *pTxBuffer, uint32_t Len)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	uint8_t FlagStatus = 0;
	while(Len != 0U)
	{
		FlagStatus = SPI_GetFlagStatus(pSPIx,(1 << SPI_SR_TXE));
		// wait until the Tx buffer is empty ( TXE is SET )
		while (RESET == FlagStatus);
		// check DFF to see the data is 8bit or 16bit
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF case
			// load the data in to the DR ( data register )
			pSPIx->SPI_DR = *((uint16_t*)pTxBuffer); // type cast to uint16_t and get value
			Len -= 2;
			// increase to point to the next data
			(uint16_t*)pTxBuffer++;
		}
		else
		{
			// 8 bit DFF case
			pSPIx->SPI_DR = *pTxBuffer; // get value. No need  to type cast because pointer is uint8_t by default
			Len--;
			// increase to point to the next data
			pTxBuffer++;
		}

	}
	return eLldRetVal;
}

/**
 * @brief       Receive data over SPI.
 *
 * @details     This function receive data over SPI by continuously reading the data until the specified length is reached.
 *              This function waits for the receive buffer to be empty
 *              before read data and handles both 8-bit and 16-bit data transmission based on the configured data frame format (DFF).
 *
 * @param[in]   pSPIx : Pointer to the SPI peripheral register map.
 * @param[in]   pTxBuffer :  Pointer to the transmit buffer containing data to be sent.
 * @param[in]   Len : Length of the data to be sent.
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note        Blocking function
 */
Spi_JobResultType SPI_ReceiveData(SPI_RegMap_t *pSPIx, Spi_BufferSize *pRxBuffer, uint32_t Len)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	uint8_t FlagStatus = 0;
	while(Len != 0U)
	{
		FlagStatus = SPI_GetFlagStatus(pSPIx,(1 << SPI_SR_RXNE));
		// wait until the Rx buffer is empty (RXNE is set )
		while (RESET == FlagStatus);
		// check DFF to see the data is 8bit or 16bit
		if (pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF case
			// read the data from the DR ( data register )
			*((uint16_t*)pRxBuffer) = pSPIx->SPI_DR; // type cast to uint16_t and get value
			Len -= 2;
			// increase to point to the next data
			(uint16_t*)pRxBuffer++;
		}
		else
		{
			// 8 bit DFF case
			*(pRxBuffer) = pSPIx->SPI_DR; // get value. No need  to type cast because pointer is uint8_t by default
			Len--;
			// increase to point to the next data
			pRxBuffer++;
		}
	}
	return eLldRetVal;
}

/*---------------------------------------------------------------------------
*                        IRQ Configuration and ISR handling
-----------------------------------------------------------------------------*/
Spi_JobResultType SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDI);
Spi_JobResultType SPI_IRQHandling(uint8_t pinNumber);
Spi_JobResultType SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);

/*---------------------------------------------------------------------------
*                        Peripheral Status Check APIS
-----------------------------------------------------------------------------*/
uint8_t SPI_GetFlagStatus(SPI_RegMap_t *pSPIx, uint32_t FlagName)
{
	if(pSPIx->SPI_SR & FlagName) //bit mask
	{
		return SET;
	}
	return RESET;
}
/*---------------------------------------------------------------------------
*                        Other Peripheral Control APIS
-----------------------------------------------------------------------------*/
/**
 * @brief       SPI_PeripheralControl
 *
 * @details     In order to use SPI peripheral, beside enable peripheral clock (RCC)
 *              SPE bit is also need to be set/clear to enable or disable specific SPI peripheral
 *
 * @param[in]   pSPIx : Pointer to the SPI peripheral register map.
 * @param[in]   EnOrDI : Enable or disable operation, use ENABLE or DISABLE macros
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_PeripheralControl(SPI_RegMap_t *pSPIx, uint8_t EnOrDI)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(EnOrDI == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<SPI_CR1_SPE);
	}
	else if(EnOrDI == DISABLE)
	{
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SPE);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	return eLldRetVal;
}

/**
 * @brief       SPI_SSIConfig
 *
 * @details     Enable or Disable SSI: Internal slave select.
 *              This bit has an effect only when the SSM bit is set. The value of this bit is forced onto the
 *              NSS pin and the IO value of the NSS pin is ignored.
 *              If SP_SSM (Software slave management) is enable, must enable this bit to avoid MODF error
 *
 * @param[in]   pSPIx : Pointer to the SPI peripheral register map.
 * @param[in]   EnOrDI : Enable or disable operation, use ENABLE or DISABLE macros
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_SSIConfig(SPI_RegMap_t *pSPIx, uint8_t EnOrDI)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(EnOrDI == ENABLE)
	{
		pSPIx->SPI_CR1 |= (1<<SPI_CR1_SSI);
	}
	else if(EnOrDI == DISABLE)
	{
		pSPIx->SPI_CR1 &= ~(1<<SPI_CR1_SSI);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	return eLldRetVal;
}

/**
 * @brief       SPI_SSOEConfig
 *
 * @details     Enable or Disable SSOE :SS output enable.
 * 				If SP_SSM (Software slave management) is disable - aka Hardware slave management
 * 				enable SSOE if the device operates in master mode.
 * 				The NSS signal is driven low when the master starts the communication and is kept low until the SPI is disabled.
 *
 *                  0: SS output is disabled in master mode and the cell can work in multimaster configuration
 *                  1: SS output is enabled in master mode and when the cell is enabled. The cell cannot work
 *                  in a multimaster environment
 *
 * @param[in]   pSPIx : Pointer to the SPI peripheral register map.
 * @param[in]   EnOrDI : Enable or disable operation, use ENABLE or DISABLE macros
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_SSOEConfig(SPI_RegMap_t *pSPIx, uint8_t EnOrDI)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(EnOrDI == ENABLE)
	{
		pSPIx->SPI_CR2 |= (1<<SPI_CR2_SSOE);
	}
	else if(EnOrDI == DISABLE)
	{
		pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_SSOE);
	}
	else
	{
		// should not enter here
		eLldRetVal = SPI_JOB_FAILED;
	}
	return eLldRetVal;
}
