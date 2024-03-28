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
*   @file    stm32f407xx_spi.h
*
*   @brief   SPI low-level driver header file.
*   @details SPI low-level driver header file.
*
*/

#ifndef INC_STM32F407XX_SPI_H_
#define INC_STM32F407XX_SPI_H_

/*==================================================================================================
*                                        INCLUDE FILES
* 1) system and project includes
* 2) needed interfaces from external units
* 3) internal and external interfaces from this unit
==================================================================================================*/

#include "huypq_stm32f407xx.h"

/*==================================================================================================
*                                          CONSTANTS
==================================================================================================*/

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*==================================================================================================
*                                            ENUMS
==================================================================================================*/

/*==================================================================================================
*                                STRUCTURES AND OTHER TYPEDEFS
==================================================================================================*/
/**
* @brief   Specifies the size for SPI buffer.
*
* @implements Spi_BufferSize_typedef
*/
typedef uint8_t Spi_BufferSize;
/*---------------------------------------------------------------------------
*                         WRAPPER OF IP DRIVER LAYER
-----------------------------------------------------------------------------*/

/*
 * Hold SPIx configuration setting
 */
typedef struct
{
	uint8_t SPI_DeviceMode;      // Master or Slave mode
	uint8_t SPI_BusConfig;       // Comunication mode : half/full duplex
	uint8_t SPI_SclkSpeed;       // Clock speed
	uint8_t SPI_DFF;             // data frame format (8/16 bit)
	uint8_t SPI_CPOL;            // Cpol configuration.
    uint8_t SPI_CPAL;            // Cpal configuration.
	uint8_t SPI_SSM;             // slave management (software/hardware)
}SPI_Config_t;

/*
 * Handler structure for SPIx peripheral
 */
typedef struct
{
	SPI_RegMap_t *pSPIx;     // hold the base address of SPIx  peripheral
	SPI_Config_t SPI_Config; // Hold SPIx configuration setting
}SPI_Handle_t;

/*==================================================================================================
*                                GLOBAL VARIABLE DECLARATIONS
==================================================================================================*/

/*==================================================================================================
*                                    FUNCTION PROTOTYPES
==================================================================================================*/
/*
 * Peripheral Clock setup
 */
Spi_JobResultType SPI_PeriClockControl(SPI_RegMap_t *pSPIx, uint8_t EnOrDI);

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

#endif /* INC_STM32F407XX_SPI_H_ */
