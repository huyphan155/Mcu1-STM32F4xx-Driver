/*
 * huypq_stm32f407xx.h
 *
 *  Created on: Mar 13, 2024
 *      Author: Huy Phan Quang
 */

#ifndef INC_HUYPQ_STM32F407XX_H_
#define INC_HUYPQ_STM32F407XX_H_

#include <stdint.h>

/************************************************
 *                     MARCO
 ************************************************/

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
* @brief    : GPIO job return status
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


/***********************************************************
 ***************MEMORY BASE ADDRESS*************************
 ***********************************************************/
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
#define SPI1_BASEADDR			0x4001300OU
#define SPI4_BASEADDR			0x40013400U
#define SYSCFG_BASEADDR			0x40013800U
#define EXTI_BASEADDR			0x40013C00U
#define SPI5_BASEADDR			0x40015000U
#define SPI6_BASEADDR			0x40015400U

/*********************************************************************
 *           PERIPHERAL REGISTER MAP ADDRESS STRUCTURE
 ********************************************************************/

/*********GPIO-General-purpose-I/O***************/

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

/*********RCC-Reset and clock control***************/
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


/*********CLOCK ENABLE - DISABLE - RESET***************/
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
//#define SPI4_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 13 ) ) //
//#define SPI5_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 12 ) )
//#define SPI6_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 12 ) )

/*
 * Clock enable Marco for UARTx peripheral
 */
#define USART1_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 4 ) )
#define USART2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 17 ) )
#define USART3_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 18 ) )
#define UART4_PERIF_CLK_EB()          ( RCC->APB1ENR |= ( 1 << 19 ) )
#define UART5_PERIF_CLK_EB()          ( RCC->APB1ENR |= ( 1 << 20 ) )
#define USART6_PERIF_CLK_EB()         ( RCC->APB2ENR |= ( 1 << 5 ) )
//#define UART7_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 12 ) )
//#define UART8_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 12 ) )

/*
 * Clock enable Marco for CANx peripheral
 */
#define CAN1_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 25 ) )
#define CAN2_PERIF_CLK_EB()         ( RCC->APB1ENR |= ( 1 << 26 ) )

/*
 * Clock enable Marco for SYSCFG peripheral
 */
#define SYSCFG_PERIF_CLK_EB         ( RCC->APB2ENR |= ( 1 << 14 ) )

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
//#define GPIOJ_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 9 ) )
//#define GPIOK_PERIF_CLK_DI()         ( RCC->AHB1ENR &= ~( 1 << 10 ) )

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
//#define SPI4_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 12 ) )
//#define SPI5_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 12 ) )
//#define SPI6_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 12 ) )

/*
 * Clock disable Marco for UARTx peripheral
 */
#define USART1_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 4 ) )
#define USART2_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 17 ) )
#define USART3_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 18 ) )
#define UART4_PERIF_CLK_DI()          ( RCC->APB1ENR &= ~( 1 << 19 ) )
#define UART5_PERIF_CLK_DI()          ( RCC->APB1ENR &= ~( 1 << 20 ) )
#define USART6_PERIF_CLK_DI()         ( RCC->APB2ENR &= ~( 1 << 5 ) )
//#define UART7_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 12 ) )
//#define UART8_PERIF_CLK_DI()         ( RCC->APB1ENR &= ~( 1 << 12 ) )

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

#endif /* INC_HUYPQ_STM32F407XX_H_ */
