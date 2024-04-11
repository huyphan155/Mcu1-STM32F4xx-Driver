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
*   @file    huypq_stm32f407xx.h
*
*   @brief   Header file for STM32F4xx
*   @details Header file for STM32F4xx
*
*/

#ifndef INC_HUYPQ_STM32F407XX_H_
#define INC_HUYPQ_STM32F407XX_H_

#include <stdint.h>

/*==================================================================================================
*                             Processor Specific Details : ARM_CORTEX M4
==================================================================================================*/
/*
 * Arm Cortex Mx processor NVIC ISERx register Addresses
 * from : CortexM4 generic user guide 4.2
 * @brief    : Interrupt Set-enable Registers
 */
#define NVIC_ISER0           ( (volatile uint32_t*) 0xE000E100U ) // cast to pointer tell the compiler that these values are addresses and not numbers.
#define NVIC_ISER1           ( (volatile uint32_t*) 0xE000E104U )
#define NVIC_ISER2           ( (volatile uint32_t*) 0xE000E108U )
#define NVIC_ISER3           ( (volatile uint32_t*) 0xE000E10CU )
#define NVIC_ISER4           ( (volatile uint32_t*) 0xE000E110U )
#define NVIC_ISER5           ( (volatile uint32_t*) 0xE000E114U )
#define NVIC_ISER6           ( (volatile uint32_t*) 0xE000E118U )
#define NVIC_ISER7           ( (volatile uint32_t*) 0xE000E12CU )


/*
 * Arm Cortex Mx processor NVIC ISERx register Addresses
 * from : CortexM4 generic user guide 4.2
 * @brief    : Interrupt Clear-enable Registers
 */
#define NVIC_ICER0           ( (volatile uint32_t*) 0XE000E180U ) // cast to pointer tell the compiler that these values are addresses and not numbers.
#define NVIC_ICER1           ( (volatile uint32_t*) 0xE000E184U )
#define NVIC_ICER2           ( (volatile uint32_t*) 0xE000E188U )
#define NVIC_ICER3           ( (volatile uint32_t*) 0xE000E18CU )
#define NVIC_ICER4           ( (volatile uint32_t*) 0xE000E190U )
#define NVIC_ICER5           ( (volatile uint32_t*) 0xE000E194U )
#define NVIC_ICER6           ( (volatile uint32_t*) 0xE000E198U )
#define NVIC_ICER7           ( (volatile uint32_t*) 0xE000E19CU )

/*
 * Arm Cortex Mx processor NVIC ISERx register Addresses
 * from : CortexM4 generic user guide 4.2
 * @brief    : Interrupt Priority Registers
 */
#define NVIC_IPR_BASEADDR           ( (volatile uint32_t*) 0xE000E400U ) // cast to pointer tell the compiler that these values are addresses and not numbers.

/*
 * Arm Cortex Mx processor number of priority bits implemented in Priority register
 */
#define NUM_PR_BITS_IMPLEMENTED        4

/*==================================================================================================
*                                      DEFINES AND MACROS
==================================================================================================*/

/*
 * some generic marco
 */
#define ENABLE              1
#define DISABLE             0
#define SET                 1
#define RESET               0
#define GPIO_PIN_SET        1
#define GPIO_PIN_RESET      0


/**
* @brief     GPIO job return status
*/
typedef enum
{
    GPIO_JOB_OK = 0,               /**< @brief The job has been finished succesfully */
    GPIO_JOB_FAILED,               /**< @brief The job has not been finished succesfully */
	GPIO_JOB_PENDING,              /**< @brief The job has not yet been finished */
	GPIO_JOB_CANCELED,             /**< @brief The job has been canceled */
	GPIO_BLOCK_INCONSISTENT,       /**< @brief The requested block is inconsistent, it may contain corrupted data */
	GPIO_BLOCK_INVALID             /**< @brief The requested block has been marked as invalid, the requested operation can not be performed */
}GPIO_JobResultType;

/**
* @brief   This type defines a range of specific Jobs status for SPI Driver.
*
*/
typedef enum
{
    SPI_JOB_OK = 0,     /**< @brief The last transmission of the Job has been finished successfully. */
    SPI_JOB_PENDING,    /**< @brief The SPI Handler/Driver is performing a SPI Job. The meaning of this status is equal to SPI_BUSY.. */
    SPI_JOB_FAILED,     /**< @brief The last transmission of the Job has failed. */
    SPI_JOB_QUEUED      /**< @brief An asynchronous transmit Job has been accepted, while actual
                                     transmission for this Job has not started yet. */
} Spi_JobResultType;

/**
* @brief    : return portCode for given GPIOx base address
*/
#define GPIO_BASEADDR_TO_CODE(x) ( ( x == GPIOA ) ? 0 :\
		                           ( x == GPIOB ) ? 1 :\
		                           ( x == GPIOC ) ? 2 :\
								   ( x == GPIOD ) ? 3 :\
								   ( x == GPIOE ) ? 4 :\
								   ( x == GPIOF ) ? 5 :\
								   ( x == GPIOG ) ? 6 :\
								   ( x == GPIOH ) ? 7 :\
								   ( x == GPIOI ) ? 8 : 0 )

/*---------------------------------------------------------------------------
*                         Bit position definitions of SPI peripheral
-----------------------------------------------------------------------------*/
/*
 * Bit position definitions SPI_CR1
 */
#define SPI_CR1_CPHA     				 0
#define SPI_CR1_CPOL      				 1
#define SPI_CR1_MSTR     				 2
#define SPI_CR1_BR   					 3
#define SPI_CR1_SPE     				 6
#define SPI_CR1_LSBFIRST   			 	 7
#define SPI_CR1_SSI     				 8
#define SPI_CR1_SSM      				 9
#define SPI_CR1_RXONLY      		 	10
#define SPI_CR1_DFF     			 	11
#define SPI_CR1_CRCNEXT   			 	12
#define SPI_CR1_CRCEN   			 	13
#define SPI_CR1_BIDIOE     			 	14
#define SPI_CR1_BIDIMODE      			15

/*
 * Bit position definitions SPI_CR2
 */
#define SPI_CR2_RXDMAEN		 			0
#define SPI_CR2_TXDMAEN				 	1
#define SPI_CR2_SSOE				 	2
#define SPI_CR2_FRF						4
#define SPI_CR2_ERRIE					5
#define SPI_CR2_RXNEIE				 	6
#define SPI_CR2_TXEIE					7


/*
 * Bit position definitions SPI_SR
 */
#define SPI_SR_RXNE						0
#define SPI_SR_TXE				 		1
#define SPI_SR_CHSIDE				 	2
#define SPI_SR_UDR					 	3
#define SPI_SR_CRCERR				 	4
#define SPI_SR_MODF					 	5
#define SPI_SR_OVR					 	6
#define SPI_SR_BSY					 	7
#define SPI_SR_FRE					 	8

/*==================================================================================================
*                                      MEMORY BASE ADDRESS
==================================================================================================*/
/*
 * Base address of FLASH and SRAM memory
 */
#define FLASH_BASEADDR         0x08000000U  /*FLASH Base memory address*/
#define SRAM1_BASEADDR 		   0x20000000U  /*112KB = 112*1024 byte = 1C000 in Hex */
#define SRAM2_BASEADDR 		   0x2001C000U  /*FLASH Base memory address*/
#define ROM__BASEADDR	 	   0x1FFF0000U  /*30Kb - System memory */
#define SRAM				   SRAM1_BASEADDR

/*
 * Base address of AHBx and APBx bus
 */
#define PERIPH_BASEADDR            0x40000000U
#define APB1PERIPH_BASEADDR        PERIPH_BASE
#define APB2PERIPH_BASEADDR        0x40010000U
#define AHB1PERIPH_BASEADDR		   0x40020000U
#define AHB2PERIPH_BASEADDR		   0x50000000U
#define AHB3PERIPH_BASEADDR        0xA0000000U

/*
 * Base address of peripherals on AHB1 bus
 */
#define GPIOA_BASEADDR          AHB1PERIPH_BASEADDR
#define GPIOB_BASEADDR		    0x40020400U
#define GPIOC_BASEADDR			0x4002080U
#define GPIOD_BASEADDR			0x40020C00U
#define GPIOE_BASEADDR			0x40021000U
#define GPIOF_BASEADDR			0x40021400U
#define GPIOG_BASEADDR			0x40021800U
#define GPIOH_BASEADDR			0x40021C00U
#define GPIOI_BASEADDR			0x40022000U
#define GPIOJ_BASEADDR			0x40022400U
#define GPIOK_BASEADDR			0x40022800U
#define CRC_BASEADDR			0x40023000U
#define RCC_BASEADDR			0x40023800U

/*
 * Base address of peripherals on APB1 bus
 */
#define SPI2_BASEADDR			0x40003800U
#define SPI3_BASEADDR			0x40003C00U
#define UART2_BASEADDR			0x40004400U
#define USART3_BASEADDR			0x40004800U
#define USART4_BASEADDR			0x40004C00U
#define UART5_BASEADDR			0x40005000U
#define I2C1_BASEADDR			0x40005400U
#define I2C2_BASEADDR			0x40005800U
#define I2C3_BASEADDR			0x40005C00U
#define CAN1_BASEADDR			0x40006400U
#define CAN2_BASEADDR			0x40006800U
#define UART7_BASEADDR			0x40007800U
#define UART8_BASEADDR			0x40007C00U


/*
 * Base address of peripherals on APB2 bus
 */
#define USART1_BASEADDR			0x40011000U
#define USART6_BASEADDR			0x40011400U
#define SPI1_BASEADDR			0x40013000U
#define SPI4_BASEADDR			0x40013400U
#define SYSCFG_BASEADDR			0x40013800U
#define EXTI_BASEADDR			0x40013C00U
#define SPI5_BASEADDR			0x40015000U
#define SPI6_BASEADDR			0x40015400U

/*==================================================================================================
*                            PERIPHERAL REGISTER MAP ADDRESS STRUCTURE
==================================================================================================*/
/*---------------------------------------------------------------------------
*                         GPIO-General-purpose-I/O
-----------------------------------------------------------------------------*/

/*
 * how to use : example with GPIOA
 * //define a pointer with value is base address of GPIOA :
 * GPIO_RegMap_t *pGPIOA = (GPIO_RegMap_t*)0x40020000U // 0x40020000U is base address of GPIOA
 * // access to peripheral of GPIO with :
 * pGPIOA->MODER = 25 // store value 25 to MODER register of GPIOA
 * ( compiler will do : *(0x40020000U + 0x00) = 25 )
*/
typedef struct
{
    volatile uint32_t MODER;               // GPIO port mode register
    volatile uint32_t OTYPER;              // GPIO port output type register
    volatile uint32_t OSPEEDR;             // GPIO port output speed register
    volatile uint32_t PUPDR;               // GPIO port pull-up/pull-down register
    volatile uint32_t IDR;                 // GPIO port input data register
    volatile uint32_t ODR;                 // GPIO port output data register
    volatile uint32_t BSRR;                // GPIO port bit set/reset register
    volatile uint32_t LCKR;                // GPIO port configuration lock register
    volatile uint32_t AFRL;                // GPIO alternate function low register
    volatile uint32_t AFRH;                // GPIO alternate function high register
}GPIO_RegMap_t;

/*
 * GPIO Peripheral Base address cast to struct pointer type of GPIO
 * now instead of :  GPIO_RegMap_t *pGPIOA = (GPIO_RegMap_t*)0x40020000U
 * you can use    :  GPIO_RegMap_t *pGPIOA = GPIOA
 */
#define GPIOA        (GPIO_RegMap_t*)GPIOA_BASEADDR
#define GPIOB        (GPIO_RegMap_t*)GPIOB_BASEADDR
#define GPIOC        (GPIO_RegMap_t*)GPIOC_BASEADDR
#define GPIOD        (GPIO_RegMap_t*)GPIOD_BASEADDR
#define GPIOE        (GPIO_RegMap_t*)GPIOE_BASEADDR
#define GPIOF        (GPIO_RegMap_t*)GPIOF_BASEADDR
#define GPIOG        (GPIO_RegMap_t*)GPIOG_BASEADDR
#define GPIOH        (GPIO_RegMap_t*)GPIOH_BASEADDR
#define GPIOI        (GPIO_RegMap_t*)GPIOI_BASEADDR
#define GPIOJ        (GPIO_RegMap_t*)GPIOJ_BASEADDR
#define GPIOK        (GPIO_RegMap_t*)GPIOK_BASEADDR

/*---------------------------------------------------------------------------
*                   SPI- Serial peripheral interface
-----------------------------------------------------------------------------*/

/*
 * Pheripheral register definition structure for SPI
*/
typedef struct
{
    volatile uint32_t SPI_CR1;                  // SPI control register 1
    volatile uint32_t SPI_CR2;                  // SPI control register 2
    volatile uint32_t SPI_SR;                   // SPI status register
    volatile uint32_t SPI_DR;                   // SPI data register
    volatile uint32_t SPI_CRCPR;                // SPI CRC polynomial register
    volatile uint32_t SPI_RXCRCR;               // SPI RX CRC register
    volatile uint32_t SPI_TXCRCR;               // SPI TX CRC register
    volatile uint32_t SPI_I2SCFGR;              // SPI_I2S configuration register
    volatile uint32_t SPI_I2SPR;                // SPI_I2S prescaler register
}SPI_RegMap_t;

/*
 * SPIx Peripheral Base address cast to struct pointer type of SPIx
 * now instead of :  SPI_RegMap_t *pSPI1 = (SPI_RegMap_t*)0x4001300OU
 * you can use    :  SPI_RegMap_t *pSPI1 = SPI1
 */
#define SPI1        (SPI_RegMap_t*)SPI1_BASEADDR
#define SPI2        (SPI_RegMap_t*)SPI2_BASEADDR
#define SPI3        (SPI_RegMap_t*)SPI3_BASEADDR
#define SPI4        (SPI_RegMap_t*)SPI4_BASEADDR

/*---------------------------------------------------------------------------
*                   RCC-Reset and clock control
-----------------------------------------------------------------------------*/
typedef struct
{
    volatile uint32_t CR;                 // RCC clock control register
    volatile uint32_t PLLCFGR;            // RCC PLL configuration register
    volatile uint32_t CFGR;               // RCC clock configuration register
    volatile uint32_t CIR;                // RCC clock interrupt register
    volatile uint32_t AHB1RSTR;           // RCC AHB1 peripheral reset register
    volatile uint32_t AHB2RSTR;           // RCC AHB2 peripheral reset register
    volatile uint32_t AHB3RSTR;           // RCC AHB3 peripheral reset register
    uint32_t RESERVED1;                   // Reserved
    volatile uint32_t APB1RSTR;           // RCC APB1 peripheral reset register
    volatile uint32_t APB2RSTR;           // RCC APB2 peripheral reset register
    uint32_t RESERVED2[2];                // Reserved
    volatile uint32_t AHB1ENR;            // RCC AHB1 peripheral clock enable register
    volatile uint32_t AHB2ENR;            // RCC AHB2 peripheral clock enable register
    volatile uint32_t AHB3ENR;            // RCC AHB3 peripheral clock enable register
    uint32_t RESERVED3;                   // Reserved
    volatile uint32_t APB1ENR;            // RCC APB1 peripheral clock enable register
    volatile uint32_t APB2ENR;            // RCC APB2 peripheral clock enable register
    uint32_t RESERVED4[2];                // Reserved
    volatile uint32_t AHB1LPENR;          // RCC AHB1 peripheral clock enable in low power mode register
    volatile uint32_t AHB2LPENR;          // RCC AHB2 peripheral clock enable in low power mode register
    volatile uint32_t AHB3LPENR;          // RCC AHB3 peripheral clock enable in low power mode register
    uint32_t RESERVED5;                   // Reserved
    volatile uint32_t APB1LPEN;           // RCC APB1 peripheral clock enable in low power mode register
    volatile uint32_t APB2LPENR;          // RCC APB2 peripheral clock enabled in low power mode
    uint32_t RESERVED6[2];                // Reserved
    volatile uint32_t BDCR;               // RCC Backup domain control register
    volatile uint32_t CSR;                // RCC clock control & status register
    volatile uint32_t SSCGR;              // RCC spread spectrum clock generation register
    volatile uint32_t PLLI2SCFGR;         // RCC PLLI2S configuration register
}RCC_RegMap_t;

/*
 * RCC Peripheral Base address cast to struct pointer type of RCC
 */
#define RCC           ((RCC_RegMap_t*)(RCC_BASEADDR))

/*---------------------------------------------------------------------------
*                   CLOCK ENABLE - DISABLE - RESET
-----------------------------------------------------------------------------*/
/*
 * Clock enable Marco for GPIOx peripheral
 */
#define GPIOA_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 0 ) )
#define GPIOB_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 1 ) )
#define GPIOC_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 2 ) )
#define GPIOD_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 3 ) )
#define GPIOE_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 4 ) )
#define GPIOF_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 5 ) )
#define GPIOG_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 6 ) )
#define GPIOH_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 7 ) )
#define GPIOI_PERIF_CLK_EB()         ( RCC->AHB1ENR |= ( 1 << 8 ) )

/*
 * Clock enable Marco for I2Cx peripheral
 */
#define I2C1_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 21 ) )
#define I2C2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 22 ) )
#define I2C3_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 23 ) )

/*
 * Clock enable Marco for SPIx peripheral
 */
#define SPI1_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 12 ) )
#define SPI2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 14 ) )
#define SPI3_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 15 ) )

/*
 * Clock enable Marco for UARTx peripheral
 */
#define USART1_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PERIF_CLK_EB()          ( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PERIF_CLK_EB()          ( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 5 ) )

/*
 * Clock enable Marco for CANx peripheral
 */
#define CAN1_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 25 ) )
#define CAN2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 26 ) )

/*
 * Clock enable Marco for SYSCFG peripheral
 */
#define SYSCFG_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 14 ) )

/*
 * Clock disable Marco for GPIOx peripheral
 */
#define GPIOA_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 0 ) )
#define GPIOB_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 1 ) )
#define GPIOC_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 2 ) )
#define GPIOD_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 3 ) )
#define GPIOE_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 4 ) )
#define GPIOF_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 5 ) )
#define GPIOG_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 6 ) )
#define GPIOH_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 7 ) )
#define GPIOI_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 8 ) )

/*
 * Clock disable Marco for I2Cx peripheral
 */
#define I2C1_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 21 ) )
#define I2C2_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 22 ) )
#define I2C3_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 23 ) )

/*
 * Clock disable Marco for SPIx peripheral
 */
#define SPI1_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 12 ) )
#define SPI2_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 14 ) )
#define SPI3_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 15 ) )

/*
 * Clock disable Marco for UARTx peripheral
 */
#define USART1_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PERIF_CLK_DI()          ( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PERIF_CLK_DI()          ( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 5 ) )

/*
 * Clock disable Marco for CANx peripheral
 */
#define CAN1_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 25 ) )
#define CAN2_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 26 ) )

/*
 * Clock disable Marco for SYSCFG peripheral
 */
#define SYSCFG_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 14 ) )

/*
 * Reset GPIOx Peripherals
 * Reset by OR operator, then clear by AND to release it.
 * Do ( do 1 time ) while (0)
 */
#define GPIOA_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 0)); (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 1)); (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 2)); (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 3)); (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 4)); (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 5)); (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 6)); (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 7)); (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()         do {(RCC->AHB1RSTR |= (1 << 8)); (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Reset SPIx Peripherals
 * Reset by OR operator, then clear by AND to release it.
 * Do ( do 1 time ) while (0)
 */
#define SPI1_REG_RESET()         do {(RCC->APB2RSTR |= (1 << 12)); (RCC->AHB1RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()         do {(RCC->APB1RSTR |= (1 << 14)); (RCC->AHB1RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()         do {(RCC->APB1RSTR |= (1 << 15)); (RCC->AHB1RSTR &= ~(1 << 15)); }while(0)

/*---------------------------------------------------------------------------
*                   EXTERNAL INTERRUPT/EVENT CONTROLLER
-----------------------------------------------------------------------------*/
typedef struct
{
	volatile uint32_t EXTI_IMR;               // Interrupt mask register
	volatile uint32_t EXTI_EMR;               // Event mask register
	volatile uint32_t EXTI_RTSR;              // Rising trigger selection register
	volatile uint32_t EXTI_FTSR;              // Falling trigger selection register
	volatile uint32_t EXTI_SWIER;             // Software interrupt event register
	volatile uint32_t EXTI_PR;                // Pending register
}EXTI_RegMap_t;

/*
 * EXTI Peripheral Base address cast to struct pointer type of EXTI
 */
#define EXTI           ((EXTI_RegMap_t*)(EXTI_BASEADDR))

/*
 * IRQ ( Interrupt requests) number of STM32F407x MCU
 */
#define IRQ_NO_WWDG                 0
#define IRQ_NO_PVD                  1
#define IRQ_NO_TAMP_STAMP           2
#define IRQ_NO_WKUP                 3
#define IRQ_NO_FLASH                4
#define IRQ_NO_RCC                  5
#define IRQ_NO_EXTIO                6
#define IRQ_NO_EXTI1                7
#define IRQ_NO_EXTI2                8
#define IRQ_NO_EXTI3                9
#define IRQ_NO_EXTI4                10
#define IRQ_NO_EXTI9_5              23
#define IRQ_NO_EXTI15_10            40

/*---------------------------------------------------------------------------
*                   SYSTEM CONFIGURATION CONTROLLER (SYSCFG)
-----------------------------------------------------------------------------*/
typedef struct
{
	volatile uint32_t SYSCFG_MEMRMP;               // SYSCFG memory remap register
	volatile uint32_t SYSCFG_PMC;                  // SYSCFG peripheral mode configuration register
	volatile uint32_t SYSCFG_EXTICR[4];              // SYSCFG external interrupt configuration register
	uint32_t RESERVED[2];                         // Reserved
	volatile uint32_t SYSCFG_CMPCR;                // Compensation cell control register
}SYSCFG_RegMap_t;

/*
 * EXTI Peripheral Base address cast to struct pointer type of EXTI
 */
#define SYSCFG           ((SYSCFG_RegMap_t*)(SYSCFG_BASEADDR))


#include "stm32f407xx_gpio.h"
#include "stm32f407xx_spi.h"

#endif /* INC_HUYPQ_STM32F407XX_H_ */
