/*
 * stm32f407xx.h
 *
 *  Created on: Oct 4, 2023
 *      Author: suraj.sd
 */

#ifndef INC_STM32F407XX_H_
#define INC_STM32F407XX_H_


#include <stdint.h>
#include <stddef.h>

#define __vo volatile

/******************************************START:Processor specific Details**************************/
/*
 * ARM Cortex Mx Processor NVIC ISERx Registers Address
 * These registers are used to enable the interrupt on a particular IRQ Number
 * 0 Has no effect and 1 is used to enable
 * NVIC - Nested Vectored Interrupt controller (Refer 375 page of reference manual)
 * 82 maskable interrupt channels for STM32F405xx/07xx
 */
#define NVIC_ISER0                    ( (__vo uint32_t *)0xE000E100 )
#define NVIC_ISER1                    ( (__vo uint32_t *)0xE000E104 )
#define NVIC_ISER2                    ( (__vo uint32_t *)0xE000E108 )
#define NVIC_ISER3                    ( (__vo uint32_t *)0xE000E10c )

/*
 * ARM Cortex Mx Processor NVIC ICERx Registers Address
 * These registers are used to disable the interrupt on a particular IRQ Number
 * 0 has no effect and 1 is used to disable
 */
#define NVIC_ICER0                    ( (__vo uint32_t *)0XE000E180 )
#define NVIC_ICER1                    ( (__vo uint32_t *)0XE000E184 )
#define NVIC_ICER2                    ( (__vo uint32_t *)0XE000E188 )
#define NVIC_ICER3                    ( (__vo uint32_t *)0XE000E18c )


/*
 * ARM Cortex Mx Processor NVIC IPR Register Address
 */
#define NVIC_PR_BADE_ADDR             ( (__vo uint32_t *)0xE000E400 )

/*
 * In priority registers lower bits are unimplemented only upper bits are implemented
 */
#define NO_PR_BITS_IMPLEMENTED        4

/*
 *  base addresess of flash memory and sram memory
 */

#define FLASH_ADDR                    0X08000000U            //we can use DRV_FLASH_ADDR as it belongs to driver layer
#define SRAM1_ADDR                    0X20000000U            //SRAM1 is of 112kb
#define SRAM                          SRAM1_ADDR
#define SRAM2_ADDR                    0X2001C000U            //SRAM2 is of 16kb followed by SRAM1
#define ROM                           0x1FFF 0000            //system memory

/*
 *  AHBx and APBx Bus Peripheral base addresses
 */

#define PERIPH_BASEADDR               0X40000000U
#define APB1PERIPH_BASEADDR           PERIPH_BASEADDR        //address from where apb1 connected peripheral's register starts
#define APB2PERIPH_BASEADDR           0X40010000U            //address from where apb2 connected peripheral's register starts
#define AHB1PERIPH_BASEADDR           0X40020000U            //address from where ahb1 connected peripheral's register starts
#define AHB2PERIPH_BASEADDR           0X50000000U            //address from where ahb2 connected peripheral's register starts

/*
 * Base addresses of peripherals which are hanging on AHB1 bus
 */

#define GPIOA_BASEADDR                (AHB1PERIPH_BASEADDR + 0X0000)           //address offset:0x0000
#define GPIOB_BASEADDR                (AHB1PERIPH_BASEADDR + 0X0400)           //address offset:0x0400
#define GPIOC_BASEADDR                (AHB1PERIPH_BASEADDR + 0X0800)           //address offset:0x0800
#define GPIOD_BASEADDR                (AHB1PERIPH_BASEADDR + 0X0C00)           //address offset:0x0C00
#define GPIOE_BASEADDR                (AHB1PERIPH_BASEADDR + 0X1000)           //address offset:0x1000
#define GPIOF_BASEADDR                (AHB1PERIPH_BASEADDR + 0X1400)           //address offset:0x1400
#define GPIOG_BASEADDR                (AHB1PERIPH_BASEADDR + 0X1800)           //address offset:0x1800
#define GPIOH_BASEADDR                (AHB1PERIPH_BASEADDR + 0X1C00)           //address offset:0x1C00
#define GPIOI_BASEADDR                (AHB1PERIPH_BASEADDR + 0X2000)           //address offset:0x2000
#define RCC_BASEADDR                  (AHB1PERIPH_BASEADDR + 0X3800)           //Address offset:0x3800

/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define I2C1_BASEADDR                 (APB1PERIPH_BASEADDR + 0X5400)          //address offset:0x5400
#define I2C2_BASEADDR                 (APB1PERIPH_BASEADDR + 0X5800)          //address offset:0x5800
#define I2C3_BASEADDR                 (APB1PERIPH_BASEADDR + 0X5C00)          //address offset:0x5C00

#define SPI2_BASEADDR                 (APB1PERIPH_BASEADDR + 0X3800)          //address offset:0x3800
#define SPI3_BASEADDR                 (APB1PERIPH_BASEADDR + 0X3C00)          //address offset:0x3C00

#define USART2_BASEADDR               (APB1PERIPH_BASEADDR + 0X4400)          //address offset:0x4400
#define USART3_BASEADDR               (APB1PERIPH_BASEADDR + 0X4800)          //address offset:0x4800
#define UART4_BASEADDR                (APB1PERIPH_BASEADDR + 0X4C00)          //address offset:0x4C00
#define UART5_BASEADDR                (APB1PERIPH_BASEADDR + 0X5000)          //address offset:0x5000


/*
 * Base addresses of peripherals which are hanging on APB1 bus
 */

#define EXTI_BASEADDR                 (APB2PERIPH_BASEADDR + 0X3C00)          //address offset:0x3C00
#define USART1_BASEADDR               (APB2PERIPH_BASEADDR + 0X1000)          //address offset:0x1000
#define USART6_BASEADDR               (APB2PERIPH_BASEADDR + 0X1400)          //address offset:0x1400
#define SPI1_BASEADDR                 (APB2PERIPH_BASEADDR + 0X3C00)          //address offset:0x3C00
#define SYSCFG_BASEADDR               (APB2PERIPH_BASEADDR + 0X3800)          //address offset:0x3800


/*************************************PERIPHERAL REGISTER DEFINATION STRUCTURES************************/

/*
 * Note : Registers of a peripheral are specific to MCU
 * Ex : No. of Registers of SPI peripheral of STM32Fx family of MCU's may be different(more or less)
 * Compared to no. of registers of SPI peripheral of SM32F0x family of MCU's
 * Please check your device specific RM
 */

/*
 * peripheral register structure defination for GPIO
 */


typedef struct
{
	__vo uint32_t MODER;                     //GPIO port mode register                 Address offset: 0x00
	__vo uint32_t OTYPER;                    //GPIO port output type register          Address offset: 0x04
	__vo uint32_t OSPEEDR;                   //GPIO port output speed register         Address offset: 0x08
	__vo uint32_t PUPDR;                     //GPIO port pull-up/pull-down register    Address offset: 0x0C
	__vo uint32_t IDR;                       //GPIO port input data register           Address offset: 0x10
	__vo uint32_t ODR;                       //GPIO port output data register          Address offset: 0x14
	__vo uint32_t BSRR;                      //GPIO port bit set/reset register        Address offset: 0x18
	__vo uint32_t LCKR;                      //GPIO port configuration lock register   Address offset: 0x1C
	__vo uint32_t AFR[2];                    //AFR[0] : GPIO alternate function low register    Address offset: 0x20
	                                         //AFR[1] : GPIO alternate function high register   Address offset: 0x24
}GPIO_RegDef_t;


/*
 * RCC Peripheral register structure definition
 */

typedef struct
{
	__vo uint32_t CR;                        //RCC clock control register                     Address offset: 0x00
	__vo uint32_t PLLCFGR;                   //RCC PLL configuration register                 Address offset: 0x04
	__vo uint32_t CFGR;                      //RCC clock configuration register               Address offset: 0x08
	__vo uint32_t CIR;                       //RCC clock interrupt register                   Address offset: 0x0C
	__vo uint32_t AHB1RSTR;                  //RCC AHB1 peripheral reset register             Address offset: 0x10
	__vo uint32_t AHB2RSTR;                  //RCC AHB2 peripheral reset register             Address offset: 0x14
	__vo uint32_t AHB3RSTR;                  //RCC AHB3 peripheral reset register             Address offset: 0x18
	uint32_t      RESERVED1;                 //
	__vo uint32_t APB1RSTR;                  //RCC APB1 peripheral reset register             Address offset: 0x20
	__vo uint32_t APB2RSTR;                  //RCC APB2 peripheral reset register             Address offset: 0x24
    uint32_t      RESERVED2[2];
	__vo uint32_t AHB1ENR;                   //RCC AHB1 peripheral clock register             Address offset: 0x30
	__vo uint32_t AHB2ENR;                   //RCC AHB2 peripheral clock register             Address offset: 0x34
	__vo uint32_t AHB3ENR;                   //RCC AHB3 peripheral clock register             Address offset: 0x38
	uint32_t      RESERVED3;
	__vo uint32_t APB1ENR;                   //RCC APB1 peripheral clock enable register      Address offset: 0x40
	__vo uint32_t APB2ENR;                   //RCC APB1 peripheral clock enable register      Address offset: 0x44
	uint32_t      RESERVED4[2];
	__vo uint32_t AHB1LPENR;                 //RCC AHB1 peripheral clock enable in low power mode register      Address offset: 0x50
	__vo uint32_t AHB2LPENR;                 //RCC AHB2 peripheral clock enable in low power mode register      Address offset: 0x54
	__vo uint32_t AHB3LPENR;                 //RCC AHB3 peripheral clock enable in low power mode register      Address offset: 0x58
	uint32_t      RESERVED5;
	__vo uint32_t APB1LPENR;                 //RCC APB1 peripheral clock enable in low power mode register      Address offset: 0x60
	__vo uint32_t APB2LPENR;                 //RCC APB2 peripheral clock enable in low power mode register      Address offset: 0x64
	uint32_t      RESERVED6[2];
	__vo uint32_t BDCR;                      //RCC Backup domain control register             Address offset: 0x70
	__vo uint32_t CSR;                       //RCC clock control & status register            Address offset: 0x74
	uint32_t      RESERVED7[2];
	__vo uint32_t SSCGR;                     //RCC spread spectrum clock generation register  Address offset: 0x80
	__vo uint32_t PLLI2SC;                   //RCC PLLI2S configuration register              Address offset: 0x84
	__vo uint32_t PLLSAICFGR;                //RCC PLL configuration register                 Address offset: 0x88
	__vo uint32_t DCKCFGR;                   //RCC Dedicated Clock Configuration Register     Address offset: 0x8C
}RCC_RegDef_t;

/*
 * EXTI Peripheral register structure definition
 * EXTI - External Interrupt/event Controller
 * Each input line can be independently configured to select the type
   (interrupt or event) and the corresponding trigger event (rising or falling or both).
 */

typedef struct
{
	__vo uint32_t IMR;
	__vo uint32_t EMR;
	__vo uint32_t RTSR;
	__vo uint32_t FTSR;
	__vo uint32_t SWIER;
	__vo uint32_t PR;
}EXTI_RegDef_t;

/*
 * SYSCFG Peripheral register structure definition
 */
typedef struct
{
	__vo uint32_t MEMRMP;                    //SYSCFG memory remap register     Address offset: 0x00
	__vo uint32_t PMC;                       //SYSCFG peripheral mode configuration register     Address offset: 0x04
	__vo uint32_t EXTICR[4];                 //Array of SYSCFG external interrupt configuration register     Address offset: 0x08 - 0x14
	                                         /*
	                                          * Pin0 of which port should take over EXTI0 is configured in this register
	                                          * i.e This registers decides which GPIO port should take over EXTI line.
	                                          */
	uint32_t      RESERVED1[2];              //Reserved                         Address offset: 0x00
	__vo uint32_t CMPCR;                     //Compensation cell control register     Address offset: 0x20
	uint32_t      RESERVED2[2];              //Reserved                          Address offset: 0x00
	__vo uint32_t CFGR;
}SYSCFG_RegDef_t;


/*
 * SPI Peripheral register structure defination
 */

typedef struct
{
	__vo uint32_t CR1;                   //SPI control register 1          Address Offset:0x00
	__vo uint32_t CR2;                   //SPI control register 2          Address Offset:0x04
	__vo uint32_t SR;                    //SPI status register             Address Offset:0x08
	__vo uint32_t DR;                    //SPI data register               Address Offset:0x0C
	__vo uint32_t CRCPR;                 //SPI CRC polynomial register     Address Offset:0x10
	__vo uint32_t RXCRCR;                //SPI RX CRC register             Address Offset:0x14 (not used in I2S mode)
	__vo uint32_t TXCRCR;                //SPI TX CRC register             Address Offset:0x18 (not used in I2S mode)
	__vo uint32_t I2SCFGR;               //SPI_I2S configuration register  Address Offset:0x1C
	__vo uint32_t I2SPR;                 //SPI_I2S prescaler register      Address Offset:0x20
}SPI_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to xxx_RegDef_t)
 */

#define GPIOA                          ((GPIO_RegDef_t *)GPIOA_BASEADDR)
#define GPIOB                          ((GPIO_RegDef_t *)GPIOB_BASEADDR)
#define GPIOC                          ((GPIO_RegDef_t *)GPIOC_BASEADDR)
#define GPIOD                          ((GPIO_RegDef_t *)GPIOD_BASEADDR)
#define GPIOE                          ((GPIO_RegDef_t *)GPIOE_BASEADDR)
#define GPIOF                          ((GPIO_RegDef_t *)GPIOF_BASEADDR)
#define GPIOG                          ((GPIO_RegDef_t *)GPIOG_BASEADDR)
#define GPIOH                          ((GPIO_RegDef_t *)GPIOH_BASEADDR)
#define GPIOI                          ((GPIO_RegDef_t *)GPIOI_BASEADDR)
#define RCC                            ((RCC_RegDef_t *)RCC_BASEADDR)

#define EXTI                           ((EXTI_RegDef_t *)EXTI_BASEADDR)

#define SYSCFG                         ((SYSCFG_RegDef_t *)SYSCFG_BASEADDR)

#define SPI1                           ((SPI_RegDef_t *)SPI1_BASEADDR)
#define SPI2                           ((SPI_RegDef_t *)SPI2_BASEADDR)
#define SPI3                           ((SPI_RegDef_t *)SPI3_BASEADDR)

/*
 * Clock enable macros for GPIO peripherals
 */
#define GPIOA_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 0) )
#define GPIOB_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 1) )
#define GPIOC_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 2) )
#define GPIOD_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 3) )
#define GPIOE_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 4) )
#define GPIOF_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 5) )
#define GPIOG_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 6) )
#define GPIOH_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 7) )
#define GPIOI_PCLK_EN()                  ( (RCC->AHB1ENR) |= (1 << 8) )

/*
 * Clock enable macros for I2C Peripherals
 */
#define I2C1_PCLK_EN()                    ( RCC->APB1ENR |= (1<<21) )
#define I2C2_PCLK_EN()                    ( RCC->APB1ENR |= (1<<22) )
#define I2C3_PCLK_EN()                    ( RCC->APB1ENR |= (1<<23) )

/*
 * Clock enable macros for SPI Peripherals
 */
#define SPI1_PCLK_EN()                   ( RCC->APB2ENR |= (1<<12) )
#define SPI2_PCLK_EN()                   ( RCC->APB1ENR |= (1<<14) )
#define SPI3_PCLK_EN()                   ( RCC->APB1ENR |= (1<<15) )
#define SPI4_PCLK_EN()                   ( RCC->APB2ENR |= (1<<13) )
#define SPI5_PCLK_EN()                   ( RCC->APB2ENR |= (1<<20) )
#define SPI6_PCLK_EN()                   ( RCC->APB2ENR |= (1<<21) )



/*
 * Clock enable macros for USART peripherals
 */
#define USART1_PCLK_EN()                 ( RCC->APB2ENR |= (1<<4) )
#define USART2_PCLK_EN()                 ( RCC->APB1ENR |= (1<<17) )
#define USART3_PCLK_EN()                 ( RCC->APB1ENR |= (1<<18) )
#define USART6_PCLK_EN()                 ( RCC->APB2ENR |= (1<<5) )


/*
 * Clock enable macros for SYSCFG Peripherals
 */
#define SYSCFG_PCLK_EN()                 ( RCC->APB2ENR |= (1<<14) )


/*
 * Clock disable macros for GPIO peripherals
 */
#define GPIOA_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 0) )
#define GPIOB_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 1) )
#define GPIOC_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 2) )
#define GPIOD_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 3) )
#define GPIOE_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 4) )
#define GPIOF_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 5) )
#define GPIOG_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 6) )
#define GPIOH_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 7) )
#define GPIOI_PCLK_DI()                  ( (RCC->AHB1ENR) &= ~(1 << 8) )

/*
 * Clock disable macros for I2C peripheral
 */
#define I2C1_PCLK_DI()                    ( RCC->APB1ENR &= ~(1<<21) )
#define I2C2_PCLK_DI()                    ( RCC->APB1ENR &= ~(1<<22) )
#define I2C3_PCLK_DI()                    ( RCC->APB1ENR &= ~(1<<23) )

/*
 * Clock disable macros for SPI peripherals
 */
#define SPI1_PCLK_DI()                   ( RCC->APB2ENR &= ~(1<<12) )
#define SPI2_PCLK_DI()                   ( RCC->APB1ENR &= ~(1<<14) )
#define SPI3_PCLK_DI()                   ( RCC->APB1ENR &= ~(1<<15) )
#define SPI4_PCLK_DI()                   ( RCC->APB2ENR &= ~(1<<13) )
#define SPI5_PCLK_DI()                   ( RCC->APB2ENR &= ~(1<<20) )
#define SPI6_PCLK_DI()                   ( RCC->APB2ENR &= ~(1<<21) )

/*
 * Clock disable macros for USART Peripherals
 */
#define USART1_PCLK_DI()                 ( RCC->APB2ENR &= ~(1<<4) )
#define USART2_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<17) )
#define USART3_PCLK_DI()                 ( RCC->APB1ENR &= ~(1<<18) )
#define USART6_PCLK_DI()                 ( RCC->APB2ENR &= ~(1<<5) )

/*
 * Clock disable macros for SYSCFG peripheral
 */
#define SYSCFG_PCLK_DI()                 ( RCC->APB2ENR &= ~(1<<14) )



/*
 * Macros to reset GPIOx peripherals
 */
#define GPIOA_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 0));  (RCC->AHB1RSTR &= ~(1 << 0)); }while(0)
#define GPIOB_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 1));  (RCC->AHB1RSTR &= ~(1 << 1)); }while(0)
#define GPIOC_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 2));  (RCC->AHB1RSTR &= ~(1 << 2)); }while(0)
#define GPIOD_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 3));  (RCC->AHB1RSTR &= ~(1 << 3)); }while(0)
#define GPIOE_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 4));  (RCC->AHB1RSTR &= ~(1 << 4)); }while(0)
#define GPIOF_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 5));  (RCC->AHB1RSTR &= ~(1 << 5)); }while(0)
#define GPIOG_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 6));  (RCC->AHB1RSTR &= ~(1 << 6)); }while(0)
#define GPIOH_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 7));  (RCC->AHB1RSTR &= ~(1 << 7)); }while(0)
#define GPIOI_REG_RESET()                do{ (RCC->AHB1RSTR |= (1 << 8));  (RCC->AHB1RSTR &= ~(1 << 8)); }while(0)

/*
 * Macros to reset SPI Peripherals
 */
#define SPI1_REG_RESET()                 do{ (RCC->APB2RSTR |= (1 << 12)); (RCC->APB2RSTR &= ~(1 << 12)); }while(0)
#define SPI2_REG_RESET()                 do{ (RCC->APB1RSTR |= (1 << 14)); (RCC->APB2RSTR &= ~(1 << 14)); }while(0)
#define SPI3_REG_RESET()                 do{ (RCC->APB1RSTR |= (1 << 15)); (RCC->APB2RSTR &= ~(1 << 15)); }while(0)

/*
 * IRQ(Interrupt Request) Numbers of GPIO Peripheral STM32F07x MCU
 */
#define IRQ_NO_EXTI0                     6
#define IRQ_NO_EXTI1                     7
#define IRQ_NO_EXTI2                     8
#define IRQ_NO_EXTI3                     9
#define IRQ_NO_EXTI4                     10
#define IRQ_NO_EXTI9_5                   23
#define IRQ_NO_EXTI15_10                 40

/*
 * IRQ Numbers on which SPI peripheral will deliver the interrupt to processor
 */
#define IRQ_NO_SPI1                      35
#define IRQ_NO_SPI2                      36
#define IRQ_NO_SPI3                      51


/*
 * Macros for all the priority levels
 */

#define NVIC_IRQ_PRI0                    0
#define NVIC_IRQ_PRI1                    1
#define NVIC_IRQ_PRI2                    2
#define NVIC_IRQ_PRI3                    3
#define NVIC_IRQ_PRI4                    4
#define NVIC_IRQ_PRI5                    5
#define NVIC_IRQ_PRI6                    6
#define NVIC_IRQ_PRI7                    7
#define NVIC_IRQ_PRI8                    8
#define NVIC_IRQ_PRI9                    9
#define NVIC_IRQ_PRI10                   10
#define NVIC_IRQ_PRI11                   11
#define NVIC_IRQ_PRI12                   12
#define NVIC_IRQ_PRI13                   13
#define NVIC_IRQ_PRI14                   14
#define NVIC_IRQ_PRI15                   15


/*
 * Some generic macros
 */
#define ENABLE            1
#define DISABLE           0
#define SET               ENABLE
#define RESET             DISABLE
#define GPIO_PIN_SET      SET
#define GPIO_PIN_RESET    RESET
#define FLAG_RESET        RESET
#define FLAG_SET          SET

/*
 * This macro returns code b/w(0 and 7) for a given GPIO base address(x)
 */

#define GPIO_BASEADDR_TO_CODE(x)         ( (x == GPIOA)?0:\
		                                   (x == GPIOB)?1:\
		                                   (x == GPIOC)?2:\
		                                   (x == GPIOD)?3:\
		                                   (x == GPIOE)?4:\
		                                   (x == GPIOF)?5:\
		                                   (x == GPIOG)?6:\
		                                   (x == GPIOH)?7:0 )


/*
 * Bit position definations of SPI Peripheral
 */

#define SPI_CR1_CPHA             0
#define SPI_CR1_CPOL             1
#define SPI_CR1_MSTR             2
#define SPI_CR1_BR               3
#define SPI_CR1_SPE              6
#define SPI_CR1_LSBFIRST         7
#define SPI_CR1_SSI              8
#define SPI_CR1_SSM              9
#define SPI_CR1_RXONLY           10
#define SPI_CR1_DFF              11
#define SPI_CR1_CRCNEXT          12
#define SPI_CR1_CRCEN            13
#define SPI_CR1_BIDIOE           14
#define SPI_CR1_BIDIMODE         15


/*
 * Bit position defination of SPI Peripheral CR2 reg
 */
#define SPI_CR2_RXDMAEN          0
#define SPI_CR2_TXDMAEN          1
#define SPI_CR2_SSOE             2
#define SPI_CR2_FRF              4
#define SPI_CR2_ERRIE            5
#define SPI_CR2_RXNEIE           6
#define SPI_CR2_TXEIE            7

/*
 * Bit position defination of SPI Peripheral Status register
 */
#define SPI_SR1_RXNE             0
#define SPI_SR1_TXE              1
#define SPI_SR1_CHSIDE           2
#define SPI_SR1_UDR              3
#define SPI_SR1_CRCERR           4
#define SPI_SR1_MODF             5
#define SPI_SR1_OVR              6
#define SPI_SR1_BSY              7
#define SPI_SR1_FRE              8



#include <stm32f407xx_gpio_driver.h>
#include <stm32f407xx_spi_driver.h>

#endif /* INC_STM32F407XX_H_ */
