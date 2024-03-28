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
/*
 * Peripheral Clock setup
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

/*
 * Init and De-init
 */
Spi_JobResultType SPI_Init(SPI_Handle_t *pSPIHandle);
Spi_JobResultType SPI_DeInit(SPI_RegMap_t *pSPIx);

/*
 * Data Sent and Receive
 */
Spi_JobResultType SPI_SentData(SPI_RegMap_t *pSPIx, Spi_BufferSize *pTxBuffer, uint32_t Len);
Spi_JobResultType SPI_ReceiveData(SPI_RegMap_t *pSPIx, Spi_BufferSize *pTxBuffer, uint32_t Len);

/*
 * IRQ Configuration and ISR handling
 */
Spi_JobResultType SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDI);
Spi_JobResultType SPI_IRQHandling(uint8_t pinNumber);
Spi_JobResultType SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority);
