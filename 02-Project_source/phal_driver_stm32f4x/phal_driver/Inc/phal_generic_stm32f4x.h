/*
 * phal_generic_stm32f4x.h
 *
 *  Created on: Jan 15, 2024
 *      Author: monurparlak@gmail.com
 *  
 *  Purpose:
 *
 */

#ifndef INC_PHAL_GENERIC_STM32F4X_H_
#define INC_PHAL_GENERIC_STM32F4X_H_

/*****************************************************************************/
/**< Includes */
/*****************************************************************************/

#include <stdint.h>
#include <stdio.h>
#include <stdbool.h>

//#include "stm32f407xx.h"

/*****************************************************************************/
/**< Macros */
/*****************************************************************************/

#define PHAL_VO     volatile

#define PHAL_ENABLE      1
#define PHAL_DISABLE     0

/** 
 * @brief Memory Addresses
 */
#define FLASH_ADDR      0x08000000U
#define SRAM1_ADDR      0x20000000U
#define SRAM2_ADDR      0x20001C00U
#define ROM_ADDR        0x1FFF0000U
#define SRAM            SRAM1_ADDR 

/** 
 * @brief AHBx and APBx Addresses
 */
#define PERIPHERAL_ADDR         0x40000000U    
#define AHB1_PERIPHERAL_ADDR    PERIPHERAL_ADDR
#define AHB2_PERIPHERAL_ADDR    0x40010000U    
#define APB1_PERIPHERAL_ADDR    0x40020000U    
#define APB2_PERIPHERAL_ADDR    0x50000000U    

/** 
 * @brief AHB1 Block Addresses
 */
#define PHAL_GPIO_PORTA_ADDR     (AHB1_PERIPHERAL_ADDR + 0x0000U)
#define PHAL_GPIO_PORTB_ADDR     (AHB1_PERIPHERAL_ADDR + 0x0400U)
#define PHAL_GPIO_PORTC_ADDR     (AHB1_PERIPHERAL_ADDR + 0x0800U)
#define PHAL_GPIO_PORTD_ADDR     (AHB1_PERIPHERAL_ADDR + 0x0C00U)
#define PHAL_GPIO_PORTE_ADDR     (AHB1_PERIPHERAL_ADDR + 0x1000U)
#define PHAL_GPIO_PORTF_ADDR     (AHB1_PERIPHERAL_ADDR + 0x1400U)
#define PHAL_GPIO_PORTG_ADDR     (AHB1_PERIPHERAL_ADDR + 0x1800U)
#define PHAL_GPIO_PORTH_ADDR     (AHB1_PERIPHERAL_ADDR + 0x1C00U)
#define PHAL_GPIO_PORTI_ADDR     (AHB1_PERIPHERAL_ADDR + 0x2000U)

#define PHAL_RCC_ADDR            (AHB1_PERIPHERAL_ADDR + 0x2000U)

/** 
 * @brief APB1 Block Addresses
 */
#define PHAL_I2C1_ADDR       (APB1_PERIPHERAL_ADDR + 0x5400U)
#define PHAL_I2C2_ADDR       (APB1_PERIPHERAL_ADDR + 0x5800U)
#define PHAL_I2C3_ADDR       (APB1_PERIPHERAL_ADDR + 0x5C00U)

#define PHAL_SPI1_ADDR       (APB1_PERIPHERAL_ADDR + 0x3800U)
#define PHAL_SPI2_ADDR       (APB1_PERIPHERAL_ADDR + 0x3C00U)

#define PHAL_USART2_ADDR     (APB1_PERIPHERAL_ADDR + 0x4400U)
#define PHAL_USART3_ADDR     (APB1_PERIPHERAL_ADDR + 0x4800U)
#define PHAL_UART4_ADDR      (APB1_PERIPHERAL_ADDR + 0x4C00U)
#define PHAL_UART5_ADDR      (APB1_PERIPHERAL_ADDR + 0x5000U)

/** 
 * @brief APB2 Block Addresses
 */
#define EXTI_ADDR       (APB2_PERIPHERAL_ADDR + 0x3C00U)
#define PHAL_SPI3_ADDR       (APB2_PERIPHERAL_ADDR + 0x3000U)
#define SYSCFG_ADDR     (APB2_PERIPHERAL_ADDR + 0x3800U)
#define PHAL_USART1_ADDR     (APB2_PERIPHERAL_ADDR + 0x1000U)
#define PHAL_USART6_ADDR     (APB2_PERIPHERAL_ADDR + 0x1400U)

/*****************************************************************************/
/**< Typedef Structures */
/*****************************************************************************/

/** 
 * @brief GPIO register definition
 */
typedef struct {
    PHAL_VO uint32_t phal_moder;         /**< GPIO Port Mode Register */
    PHAL_VO uint32_t phal_otyper;        /**< GPIO Port Output Type Register */
    PHAL_VO uint32_t phal_ospeedr;       /**< GPIO Port Output Speed Register */
    PHAL_VO uint32_t phal_pupdr;         /**< GPIO Port Pull-Up/Pull-Down Register */
    PHAL_VO uint32_t phal_idr;           /**< GPIO Port Input Data Register */
    PHAL_VO uint32_t phal_odr;           /**< GPIO Port Output Data Register */
    PHAL_VO uint32_t phal_bsrr;          /**< GPIO Port Bit Set/Reset Register */
    PHAL_VO uint32_t phal_lckr;          /**< GPIO Port Configuration Lock Register */
    PHAL_VO uint32_t phal_afr[2];        /**< GPIO Alternate Function Low/High Register */

} gpio_reg_def_t;

/** 
 * @brief PHAL_SPI register definition
 */
typedef struct {
	PHAL_VO uint32_t phal_cr1;
	PHAL_VO uint32_t phal_cr2;
	PHAL_VO uint32_t phal_sr;
	PHAL_VO uint32_t phal_dr;
	PHAL_VO uint32_t phal_crcpr;
	PHAL_VO uint32_t phal_txcrcr;
	PHAL_VO uint32_t phal_i2scfgr;
	PHAL_VO uint32_t phal_i2spr;

}spi_reg_def_t;

/**
 * @brief PHAL_RCC register definition
 */
typedef struct {
    PHAL_VO uint32_t AHB1RSTR;      /**< PHAL_RCC AHB1 Peripheral Reset Register */
    PHAL_VO uint32_t AHB2RSTR;      /**< PHAL_RCC AHB2 Peripheral Reset Register */
    PHAL_VO uint32_t AHB3RSTR;      /**< PHAL_RCC AHB3 Peripheral Reset Register */
    uint32_t RESERVED0;             /**< Reserved - Do Something */
    PHAL_VO uint32_t APB1RSTR;      /**< PHAL_RCC APB1 Peripheral Reset Register */
    PHAL_VO uint32_t APB2RSTR;      /**< PHAL_RCC APB2 Peripheral Reset Register */
    uint32_t RESERVED1[2];          /**< Reserved - Do Something */
    PHAL_VO uint32_t AHB1ENR;       /**< PHAL_RCC AHB1 Peripheral Clock Enable Register */
    PHAL_VO uint32_t AHB2ENR;       /**< PHAL_RCC AHB2 Peripheral Clock Enable Register */
    uint32_t RESERVED2;             /**< Reserved - Do Something */
    PHAL_VO uint32_t APB1ENR;       /**< PHAL_RCC APB1 Peripheral Clock Enable Register */
    PHAL_VO uint32_t APB2ENR;       /**< PHAL_RCC APB2 Peripheral Clock Enable Register */
    uint32_t RESERVED3[2];          /**< Reserved - Do Something */
    PHAL_VO uint32_t AHB1LPENR;     /**< PHAL_RCC AHB1 Peripheral Clock Enable in Low Power Mode Register */
    PHAL_VO uint32_t AHB2LPENR;     /**< PHAL_RCC AHB2 Peripheral Clock Enable in Low Power Mode Register */
    PHAL_VO uint32_t AHB3LPENR;     /**< PHAL_RCC AHB3 Peripheral Clock Enable in Low Power Mode Register */
    uint32_t RESERVED4;             /**< Reserved - Do Something */
    PHAL_VO uint32_t APB1LPENR;     /**< PHAL_RCC APB1 Peripheral Clock Enable in Low Power Mode Register */
    PHAL_VO uint32_t APB2LPENR;     /**< PHAL_RCC APB2 Peripheral Clock Enable in Low Power Mode Register */
    uint32_t RESERVED5[2];          /**< Reserved - Do Something */
    PHAL_VO uint32_t BDCR;          /**< PHAL_RCC Backup Domain Control Register */
    PHAL_VO uint32_t CSR;           /**< PHAL_RCC Clock Control & Status Register */
    uint32_t RESERVED6[2];          /**< Reserved - Do Something */
    PHAL_VO uint32_t SSCGR;         /**< PHAL_RCC Spread Spectrum Clock Generation Register */
    PHAL_VO uint32_t PLLI2SCFGR;    /**< PHAL_RCC PLLI2S Configuration Register */
    PHAL_VO uint32_t PLLSAICFGR;    /**< PHAL_RCC PLLSAI Configuration Register */
    PHAL_VO uint32_t DCKCFGR;       /**< PHAL_RCC Dedicated Clock Configuration Register */
    PHAL_VO uint32_t CKGATENR;      /**< PHAL_RCC Clocks Gated Enable Register */
    PHAL_VO uint32_t DCKCFGR2;      /**< PHAL_RCC Dedicated Clock Configuration Register 2 */

} rcc_reg_def_t;

/*****************************************************************************/
/**< Definition Structures */
/*****************************************************************************/

/** 
 * @brief GPIO peripheral definition
 */
#define PHAL_GPIO_PORTA  ((gpio_reg_def_t*) PHAL_GPIO_PORTA_ADDR)
#define PHAL_GPIO_PORTB  ((gpio_reg_def_t*) PHAL_GPIO_PORTB_ADDR)
#define PHAL_GPIO_PORTC  ((gpio_reg_def_t*) PHAL_GPIO_PORTC_ADDR)
#define PHAL_GPIO_PORTD  ((gpio_reg_def_t*) PHAL_GPIO_PORTD_ADDR)
#define PHAL_GPIO_PORTE  ((gpio_reg_def_t*) PHAL_GPIO_PORTE_ADDR)
#define PHAL_GPIO_PORTF  ((gpio_reg_def_t*) PHAL_GPIO_PORTF_ADDR)
#define PHAL_GPIO_PORTG  ((gpio_reg_def_t*) PHAL_GPIO_PORTG_ADDR)
#define PHAL_GPIO_PORTH  ((gpio_reg_def_t*) PHAL_GPIO_PORTH_ADDR)
#define PHAL_GPIO_PORTI  ((gpio_reg_def_t*) PHAL_GPIO_PORTI_ADDR)

#define PHAL_RCC         ((rcc_reg_def_t*) PHAL_RCC_ADDR)  /**< PHAL_RCC Register */

/** 
 * @brief Clock enable for GPIO peripheral definition
 */
#define PHAL_GPIO_PORTA_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 0))
#define PHAL_GPIO_PORTB_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 1))
#define PHAL_GPIO_PORTC_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 2))
#define PHAL_GPIO_PORTD_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 3))
#define PHAL_GPIO_PORTE_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 4))
#define PHAL_GPIO_PORTF_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 5))
#define PHAL_GPIO_PORTG_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 6))
#define PHAL_GPIO_PORTI_CLK_EN()     (PHAL_RCC->AHB1ENR |= (1 << 7))

/** 
 * @brief Clock enable for PHAL_I2C peripheral definition
 */
#define PHAL_I2C1_ADDR_CLK_EN()       (PHAL_RCC->APB1ENR |= (1 << 21))
#define PHAL_I2C2_ADDR_CLK_EN()       (PHAL_RCC->APB1ENR |= (1 << 22))
#define PHAL_I2C3_ADDR_CLK_EN()       (PHAL_RCC->APB1ENR |= (1 << 23))

/** 
 * @brief Clock enable for PHAL_SPI peripheral definition
 */
#define PHAL_SPI1_ADDR_CLK_EN()       (PHAL_RCC->APB2ENR |= (1 << 12))
#define PHAL_SPI2_ADDR_CLK_EN()       (PHAL_RCC->APB2ENR |= (1 << 14))
#define PHAL_SPI3_ADDR_CLK_EN()       (PHAL_RCC->APB2ENR |= (1 << 15))
#define PHAL_SPI4_ADDR_CLK_EN()       (PHAL_RCC->APB2ENR |= (1 << 13))

/** 
 * @brief Clock enable for PHAL_USART peripheral definition
 */
#define PHAL_USART1_ADDR_CLK_EN()    (PHAL_RCC->APB2ENR |= (1 << 4))
#define PHAL_USART2_ADDR_CLK_EN()    (PHAL_RCC->APB2ENR |= (1 << 17))
#define PHAL_USART3_ADDR_CLK_EN()    (PHAL_RCC->APB2ENR |= (1 << 18))
#define PHAL_UART4_ADDR_CLK_EN()     (PHAL_RCC->APB2ENR |= (1 << 19))
#define PHAL_UART5_ADDR_CLK_EN()     (PHAL_RCC->APB2ENR |= (1 << 20))
#define PHAL_USART6_ADDR_CLK_EN()    (PHAL_RCC->APB2ENR |= (1 << 5))

/** 
 * @brief Clock disable for GPIO peripheral definition
 */
#define PHAL_GPIO_PORTA_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 0))
#define PHAL_GPIO_PORTB_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 1))
#define PHAL_GPIO_PORTC_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 2))
#define PHAL_GPIO_PORTD_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 3))
#define PHAL_GPIO_PORTE_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 4))
#define PHAL_GPIO_PORTF_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 5))
#define PHAL_GPIO_PORTG_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 6))
#define PHAL_GPIO_PORTI_CLK_DIS()     (RC->AHB1ENR &= ~(1 << 7))

/** 
 * @brief Clock disable for PHAL_I2C peripheral definition
 */
#define PHAL_I2C1_ADDR_CLK_DIS()       (PHAL_RCC->APB1ENR &= ~(1 << 21))
#define PHAL_I2C2_ADDR_CLK_DIS()       (PHAL_RCC->APB1ENR &= ~(1 << 22))
#define PHAL_I2C3_ADDR_CLK_DIS()       (PHAL_RCC->APB1ENR &= ~(1 << 23))

/** 
 * @brief Clock disable for PHAL_SPI peripheral definition
 */
#define PHAL_SPI1_ADDR_CLK_DIS()       (PHAL_RCC->APB2ENR &= ~(1 << 12))
#define PHAL_SPI2_ADDR_CLK_DIS()       (PHAL_RCC->APB2ENR &= ~(1 << 14))
#define PHAL_SPI3_ADDR_CLK_DIS()       (PHAL_RCC->APB2ENR &= ~(1 << 15))
#define PHAL_SPI4_ADDR_CLK_DIS()       (PHAL_RCC->APB2ENR &= ~(1 << 13))

/** 
 * @brief Clock disable for PHAL_USART peripheral definition
 */
#define PHAL_USART1_ADDR_CLK_DIS()    (PHAL_RCC->APB2ENR &= ~(1 << 4))
#define PHAL_USART2_ADDR_CLK_DIS()    (PHAL_RCC->APB2ENR &= ~(1 << 17))
#define PHAL_USART3_ADDR_CLK_DIS()    (PHAL_RCC->APB2ENR &= ~(1 << 18))
#define PHAL_UART4_ADDR_CLK_DIS()     (PHAL_RCC->APB2ENR &= ~(1 << 19))
#define PHAL_UART5_ADDR_CLK_DIS()     (PHAL_RCC->APB2ENR &= ~(1 << 20))
#define PHAL_USART6_ADDR_CLK_DIS()    (PHAL_RCC->APB2ENR &= ~(1 << 5))

#define PHAL_GPIO_PORTA_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 0));  (PHAL_RCC->AHB1RSTR &= ~(1 << 0)); } while(0)
#define PHAL_GPIO_PORTB_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 1));  (PHAL_RCC->AHB1RSTR &= ~(1 << 1)); } while(0)
#define PHAL_GPIO_PORTC_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 2));  (PHAL_RCC->AHB1RSTR &= ~(1 << 2)); } while(0)
#define PHAL_GPIO_PORTD_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 3));  (PHAL_RCC->AHB1RSTR &= ~(1 << 3)); } while(0)
#define PHAL_GPIO_PORTE_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 4));  (PHAL_RCC->AHB1RSTR &= ~(1 << 4)); } while(0)
#define PHAL_GPIO_PORTF_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 5));  (PHAL_RCC->AHB1RSTR &= ~(1 << 5)); } while(0)
#define PHAL_GPIO_PORTG_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 6));  (PHAL_RCC->AHB1RSTR &= ~(1 << 6)); } while(0)
#define PHAL_GPIO_PORTH_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 7));  (PHAL_RCC->AHB1RSTR &= ~(1 << 7)); } while(0)
#define PHAL_GPIO_PORTI_REG_RESET()  do{(PHAL_RCC->AHB1RSTR |= (1 << 8));  (PHAL_RCC->AHB1RSTR &= ~(1 << 8)); } while(0)

/**
 * @brief EXTI IRQ (Interrupt Request)
 */
#define	PHAL_IRQ_NO_EXTI_0	6
#define	PHAL_IRQ_NO_EXTI_1	7
#define	PHAL_IRQ_NO_EXTI_2	8
#define	PHAL_IRQ_NO_EXTI_3	9
#define	PHAL_IRQ_NO_EXTI_4	10
#define	PHAL_IRQ_NO_EXTI_9	23
#define	PHAL_IRQ_NO_EXTI_15	40

/**
 * @brief EXTI IRQ (Interrupt Request) Priority
 */
#define	NVIC_IRQ_PRIORITY_0		0
#define	NVIC_IRQ_PRIORITY_15	15

#endif /* INC_PHAL_GENERIC_STM32F4X_H_ */
