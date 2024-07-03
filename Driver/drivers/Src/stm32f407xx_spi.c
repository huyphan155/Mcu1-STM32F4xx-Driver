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
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle);
static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
/*==================================================================================================
*                                         LOCAL FUNCTIONS
==================================================================================================*/
/**
*   @func     spi_txe_interrupt_handle.c
*
*   @brief    handler for spi tx transmit data
*   @details  same as SPI_SentData, send call back func to application when finish
*
*/
static void  spi_txe_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	while(pSPIHandle->TxLen != 0U)
	{
		// check DFF to see the data is 8bit or 16bit
		if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF case
			// load the data in to the DR ( data register )
			pSPIHandle->pSPIx->SPI_DR = *((uint16_t*)pSPIHandle->pTxBuffer); // type cast to uint16_t and get value
			pSPIHandle->TxLen -= 2;
			// increase to point to the next data
			(uint16_t*)pSPIHandle->pTxBuffer++;
		}
		else
		{
			// 8 bit DFF case
			pSPIHandle->pSPIx->SPI_DR = *pSPIHandle->pTxBuffer; // get value. No need  to type cast because pointer is uint8_t by default
			pSPIHandle->TxLen--;
			// increase to point to the next data
			pSPIHandle->pTxBuffer++;;
		}
		// check if pSPIHandle->TxLen is zero
		if(! pSPIHandle->TxLen)
		{
			//TxLen is zero , so close the spi transmission and inform the application that
			//TX is over.

			//this prevents interrupts from setting up of TXE flag
			pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_TXEIE); // disable interrupt generation
			pSPIHandle->TxLen = 0;
			pSPIHandle->pTxBuffer = NULL;
			pSPIHandle->TxState = SPI_READY;
			// send call back function to the application to inform that TX is over
			//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
	}
}

/**
*   @func     spi_rxne_interrupt_handle.c
*
*   @brief    handler for spi rx transmit data
*   @details  same as SPI_ReceiveData, send call back func to application when finish
*
*/
static void  spi_rxne_interrupt_handle(SPI_Handle_t *pSPIHandle)
{
	while(pSPIHandle->RxLen != 0U)
	{
		// check DFF to see the data is 8bit or 16bit
		if (pSPIHandle->pSPIx->SPI_CR1 & (1 << SPI_CR1_DFF))
		{
			// 16 bit DFF case
			// read the data from the DR ( data register )
			*((uint16_t*)pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR; // type cast to uint16_t and get value
			pSPIHandle->RxLen -= 2;
			// increase to point to the next data
			(uint16_t*)pSPIHandle->pRxBuffer++;
		}
		else
		{
			// 8 bit DFF case
			*(pSPIHandle->pRxBuffer) = pSPIHandle->pSPIx->SPI_DR; // get value. No need  to type cast because pointer is uint8_t by default
			pSPIHandle->RxLen--;
			// increase to point to the next data
			pSPIHandle->pRxBuffer++;
		}
		// check if pSPIHandle->TxLen is zero
		if(! pSPIHandle->RxLen)
		{
			//RxLen is zero , so close the spi transmission and inform the application that
			//RX is over.

			//this prevents interrupts from setting up of RXNE flag
			pSPIHandle->pSPIx->SPI_CR2 &= ~(1<<SPI_CR2_RXNEIE); // disable interrupt generation
			pSPIHandle->TxLen = 0;
			pSPIHandle->pTxBuffer = NULL;
			pSPIHandle->TxState = SPI_READY;
			// send call back function to the application to inform that RX is over
			//SPI_ApplicationEventCallback(pSPIHandle,SPI_EVENT_TX_CMPLT);
		}
	}
}

static void  spi_ovr_err_interrupt_handle(SPI_Handle_t *pSPIHandle);
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
 * @brief       Sent data over SPI.
 *
 * @details     This function sent data over SPI by continuously sending the data until the specified length is reached.
 *              This function waits for the transmit buffer to be empty
 *              before send data and handles both 8-bit and 16-bit data transmission based on the configured data frame format (DFF).
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
*                        Data Sent and Receive in Interrupt
-----------------------------------------------------------------------------*/
/**
 * @brief       Sent data over SPI in Interrupt
 *
 * @details     Store parameter to SPI_Handle,
 *              Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
 *
 * @param[in]   pSPIHandle :  Pointer to the SPI handle structure containing configuration parameters.
 * @param[in]   pTxBuffer :  Pointer to the transmit buffer containing data to be sent.
 * @param[in]   Len : Length of the data to be sent.
 *
 * @return      state.
 * @retval      SPI_BUSY_IN_TX: Data transmission busy.
 * @retval
 *
 * @note       Not write data to the data register inside this function, interrupt handle will do that.
 */
uint8_t SPI_SentDataIT(SPI_Handle_t *pSPIHandle, Spi_BufferSize *pTxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->TxState;
	if(state != SPI_BUSY_IN_TX)
	{
		// 1. Save the Tx buffer address and Len information in some global var
		pSPIHandle->pTxBuffer = pTxBuffer;
		pSPIHandle->TxLen = Len;
		//2.  Mark the SPI state as busy in transmission so that
		//    no other code can take over same SPI peripheral until transmission is over
		pSPIHandle->TxState = SPI_BUSY_IN_TX;
		//3. Enable the TXEIE control bit to get interrupt whenever TXE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_TXEIE);
	}
	return state;

}

/**
 * @brief       Receive data over SPI in Interrupt
 *
 * @details     Store parameter to SPI_Handle,
 *              Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set
 *
 * @param[in]   pSPIHandle :  Pointer to the SPI handle structure containing configuration parameters.
 * @param[in]   pRxBuffer :  Pointer to the transmit buffer containing data to be receive.
 * @param[in]   Len : Length of the data to be receive.
 *
 * @return      state.
 * @retval      SPI_BUSY_IN_RX: Data transmission busy.
 * @retval
 *
 * @note       Not write data to the data register inside this function, interrupt handle will do that.
 */
uint8_t SPI_ReceiveDataIT(SPI_Handle_t *pSPIHandle, Spi_BufferSize *pRxBuffer, uint32_t Len)
{
	uint8_t state = pSPIHandle->RxState;
	if(state != SPI_BUSY_IN_RX)
	{
		//1 . Save the Rx buffer address and Len information in some global variables
		pSPIHandle->pRxBuffer = pRxBuffer;
		pSPIHandle->RxLen = Len;
		//2.  Mark the SPI state as busy in reception so that
		//    no other code can take over same SPI peripheral until reception is over
		pSPIHandle->RxState = SPI_BUSY_IN_RX;
		//3. Enable the RXNEIE control bit to get interrupt whenever RXNEIE flag is set in SR
		pSPIHandle->pSPIx->SPI_CR2 |= (1<<SPI_CR2_RXNEIE);
	}
	return state;
}

/*---------------------------------------------------------------------------
*                        IRQ Configuration and ISR handling
-----------------------------------------------------------------------------*/
/**
 * @brief     Configures the interrupt for a given IRQ number.
 *
 * @details   Enables or disables the interrupt based on the provided parameters.
 *
 * @param[in] IRQNumber : The IRQ number to configure.
 * @param[in] EnOrDI    : Specifies whether to enable or disable the interrupt (ENABLE or DISABLE).
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_IRQInterruptConfig(uint8_t IRQNumber,uint8_t EnOrDI)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	if(EnOrDI == ENABLE)
	{
		if(IRQNumber <= 31)
		{
			//program ISER0 register
			*NVIC_ISER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 ) //32 to 63
		{
			//program ISER1 register
			*NVIC_ISER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ISER2 register //64 to 95
			*NVIC_ISER2 |= ( 1 << (IRQNumber % 64) );
		}
	}else
	{
		if(IRQNumber <= 31)
		{
			//program ICER0 register
			*NVIC_ICER0 |= ( 1 << IRQNumber );
		}else if(IRQNumber > 31 && IRQNumber < 64 )
		{
			//program ICER1 register
			*NVIC_ICER1 |= ( 1 << (IRQNumber % 32) );
		}
		else if(IRQNumber >= 64 && IRQNumber < 96 )
		{
			//program ICER2 register
			*NVIC_ICER2 |= ( 1 << (IRQNumber % 64) );
		}
	}
	return eLldRetVal;
}

/**
 * @brief     Configures the priority for a given IRQ number.
 *
 * @details   Sets the priority level for the specified IRQ.
 *
 * @param[in] IRQNumber   :    The IRQ number to configure.
 * @param[in] IRQPriority :  The priority level for the IRQ.
 *
 * @return      Spi_JobResultType.
 * @retval      SPI_JOB_OK: Data transmission completed successfully.
 * @retval      OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_IRQPriorityConfig(uint8_t IRQNumber,uint8_t IRQPriority)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_OK;
	/*1. find out the ipr register of IRQNumber*/
	uint8_t iprx = IRQNumber / 4 ;
	/*2. tim ra field(8 bit) quy dinh priority cua loai IRQ*/
	uint8_t iprx_section = IRQNumber % 4;

	uint8_t shift_amount = (8*iprx_section) + (8 - NUM_PR_BITS_IMPLEMENTED);

	/*con tro kieu uint32 nen chi can + iprx de ra duoc dia chi thanh ghi*/
	*(NVIC_IPR_BASEADDR + iprx) |= ( IRQPriority << shift_amount);
	return eLldRetVal;
}

/**
 * @brief        SPI IRQ handling
 *
 * @details      Check what event is cause the interrupt, then direct to the corresponding function
 *
 * @param[in]    pSPIHandle :  Pointer to the SPI handle structure containing configuration parameters.
 *
 * @return       Spi_JobResultType.
 * @retval       SPI_JOB_OK: Data transmission completed successfully.
 * @retval       OTHER : The job failed
 *
 * @note
 */
Spi_JobResultType SPI_IRQHandling(SPI_Handle_t *pSPIHandle)
{
	Spi_JobResultType eLldRetVal = SPI_JOB_FAILED;
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_TXE)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_TXEIE)) )
	{
		// flag TXE is enable in Status register and TXEIE is enable in Control register ( enable interrupt )
		// interrupt occur because of TXE
		spi_txe_interrupt_handle(pSPIHandle);
		eLldRetVal = SPI_JOB_OK;
	}
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_RXNE)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_RXNEIE)) )
	{
		// interrupt occur because of RXNE
		spi_rxne_interrupt_handle(pSPIHandle);
		eLldRetVal = SPI_JOB_OK;
	}
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_MODF)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_ERRIE)) )
	{
		// interrupt occur because of MODF
		// TBD
	}
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_OVR)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_ERRIE)) )
	{
		// interrupt occur because of OVR
		spi_ovr_err_interrupt_handle(pSPIHandle);
		eLldRetVal = SPI_JOB_OK;

	}
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_CRCERR)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_ERRIE)) )
	{
		// interrupt occur because of CRCERR
		// TBD
	}
	if( ((pSPIHandle->pSPIx->SPI_SR) & ( 1 << SPI_SR_FRE)) &&((pSPIHandle->pSPIx->SPI_CR2)&(1 << SPI_CR2_ERRIE)) )
	{
		// interrupt occur because of FRE
		// TBD
	}
	return eLldRetVal;
}

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
