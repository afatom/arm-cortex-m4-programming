


/*
 * stm32f407xx_rcc_driver.h
 *
 *  Created on: 25 Feb 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_RCC_DRIVER_H_
#define INC_STM32F407XX_RCC_DRIVER_H_

#include "hal_stm32f407xx.h"

/*
 * Peripheral registers definition structure for RCC (Reset and clock control)
 */
struct rcc_regs {
	volatile uint32_t CR;          /* RCC clock control register (RCC_CR) */
	volatile uint32_t PLLCFGR;     /* RCC PLL configuration register (RCC_PLLCFGR) */
	volatile uint32_t CFGR;        /* RCC clock configuration register (RCC_CFGR) */
	volatile uint32_t CIR;         /* RCC clock interrupt register (RCC_CIR)*/
	volatile uint32_t AHB1RSTR;    /* RCC AHB1 peripheral reset register (RCC_AHB1RSTR)*/
	volatile uint32_t AHB2RSTR;    /* RCC AHB2 peripheral reset register (RCC_AHB2RSTR)*/
	volatile uint32_t AHB3RSTR;    /* RCC AHB3 peripheral reset register (RCC_AHB3RSTR)*/
	volatile uint32_t RESERVED0;  /* reserved NC for now*/
	volatile uint32_t APB1RSTR;    /* RCC APB1 peripheral reset register (RCC_APB1RSTR) */
	volatile uint32_t APB2RSTR;    /* RCC APB2 peripheral reset register (RCC_APB2RSTR) */
	volatile uint32_t RESERVED_BUF0[2];
	volatile uint32_t AHB1ENR;     /* RCC AHB1 peripheral clock enable register (RCC_AHB1ENR) */
	volatile uint32_t AHB2ENR;     /* RCC AHB2 peripheral clock enable register (RCC_AHB2ENR) */
	volatile uint32_t AHB3ENR;     /* RCC AHB3 peripheral clock enable register (RCC_AHB3ENR) */
	volatile uint32_t RESERVED1;  /* reserved NC for now*/
	volatile uint32_t APB1ENR;     /* RCC APB1 peripheral clock enable register (RCC_APB1ENR) */
	volatile uint32_t APB2ENR;     /* RCC APB2 peripheral clock enable register (RCC_APB2ENR) */
	volatile uint32_t RESERVED_BUF1[2];
	volatile uint32_t AHB1LPENR;   /* RCC AHB1 peripheral clock enable in low power mode register (RCC_AHB1LPENR) */
	volatile uint32_t AHB2LPENR;   /* RCC AHB2 peripheral clock enable in low power mode register (RCC_AHB2LPENR) */
	volatile uint32_t AHB3LPENR;   /* RCC AHB3 peripheral clock enable in low power mode register (RCC_AHB3LPENR) */
	volatile uint32_t RESERVED2;  /* reserved NC for now*/
	volatile uint32_t APB1LPENR;   /* RCC APB1 peripheral clock enable in low power mode register (RCC_APB1LPENR) */
	volatile uint32_t APB2LPENR;   /* RCC APB2 peripheral clock enable in low power mode register (RCC_APB1LPENR) */
	volatile uint32_t RESERVED_BUF2[2];
	volatile uint32_t BDCR;        /* RCC Backup domain control register (RCC_BDCR) */
	volatile uint32_t CSR;         /* RCC clock control & status register (RCC_CSR)*/
	volatile uint32_t RESERVED_BUF3[2];
	volatile uint32_t SSCGR;       /* RCC spread spectrum clock generation register (RCC_SSCGR) */
	volatile uint32_t PLLI2SCFGR;  /* RCC PLLI2S configuration register (RCC_PLLI2SCFGR)*/
};


struct rcc_driver {
	struct rcc_regs rcc_regs;
};

#define RCC_DRIVER    ((struct rcc_driver*)HAL_RCC_BASE_ADDRESS)

#define GPIO_X_REG_RESET(gpio_port)  \
	do { \
		RCC_DRIVER->rcc_regs.AHB1RSTR |= (1 << gpio_port);  /*reset IO port X or gpio_port */ \
		RCC_DRIVER->rcc_regs.AHB1RSTR &= ~(1 << gpio_port); /*return default value to IO port X or gpio_port */ \
	} while(0)


/*
 * Clock enable macros for GPIOx peripherals
 */
#define GPIO_A_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 0))  /* IO port A (GPIOA) clock enable */
#define GPIO_B_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 1))  /* IO port B clock enable */
#define GPIO_C_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 2))  /* IO port C clock enable */
#define GPIO_D_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 3))  /* IO port D clock enable */
#define GPIO_E_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 4))  /* IO port E clock enable */
#define GPIO_F_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 5))  /* IO port F clock enable */
#define GPIO_G_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 6))  /* IO port G clock enable */
#define GPIO_H_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 7))  /* IO port H clock enable */
#define GPIO_I_PCLK_EN()        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << 8))  /* IO port I clock enable */

/* callers passes the port number/offset, according to AHB1ENR register e.g. GPIO A = 0, GPIO B = 1,...*/
#define GPIO_X_PCLK_EN(port)        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << port))  /* IO port X clock enable */

/*
 * Clock disable macros for GPIOx peripherals
 */
#define GPIO_A_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 0))  /* IO port A (GPIOA) clock disable */
#define GPIO_B_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 1))  /* IO port B clock disable */
#define GPIO_C_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 2))  /* IO port C clock disable */
#define GPIO_D_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 3))  /* IO port D clock disable */
#define GPIO_E_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 4))  /* IO port E clock disable */
#define GPIO_F_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 5))  /* IO port F clock disable */
#define GPIO_G_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 6))  /* IO port G clock disable */
#define GPIO_H_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 7))  /* IO port H clock disable */
#define GPIO_I_PCLK_DI()        (RCC_DRIVER->rcc_regs.AHB1ENR &= ~(1 << 8))  /* IO port I clock disable */

/* caller passes the port number/offset, according to AHB1ENR register e.g. GPIO A = 0, GPIO B = 1,...*/
#define GPIO_X_PCLK_DI(port)        (RCC_DRIVER->rcc_regs.AHB1ENR |= (1 << port))  /* IO port X clock disable */



/*
 * Clock enable macros for I2Cx peripherals
 */
#define I2C_1_PCLK_EN()        (RCC_DRIVER->rcc_regs.APB1ENR |= (1 << 21))  /* I2C1 clock enable */
#define I2C_2_PCLK_EN()        (RCC_DRIVER->rcc_regs.APB1ENR |= (1 << 22))  /* I2C2 clock enable */
#define I2C_3_PCLK_EN()        (RCC_DRIVER->rcc_regs.APB1ENR |= (1 << 23))  /* I2C3 clock enable */


/*
 * Clock disable macros for I2Cx peripherals
 */
#define I2C_1_PCLK_DI()        (RCC_DRIVER->rcc_regs.APB1ENR &= ~(1 << 21))  /* I2C1 clock disable */
#define I2C_2_PCLK_DI()        (RCC_DRIVER->rcc_regs.APB1ENR &= ~(1 << 22))  /* I2C2 clock disable */
#define I2C_3_PCLK_DI()        (RCC_DRIVER->rcc_regs.APB1ENR &= ~(1 << 23))  /* I2C3 clock disable */

/*
 * Clock enable/disable macros for SYSCFG peripheral
 */
/* Bit 14 SYSCFGEN: System configuration controller clock enable */
#define RCC_DRIVER_SYSCFG_PCLK_EN()       HAL_MACRO_TON_REG_BIT(&RCC_DRIVER->rcc_regs.APB2ENR,   BIT_14)
#define RCC_DRIVER_SYSCFG_PCLK_DI()       HAL_MACRO_TOFF_REG_BIT(&RCC_DRIVER->rcc_regs.APB2ENR,  BIT_14)


/*
 * Clock enable/disable macros for EXTI peripheral
 */



#endif /* INC_STM32F407XX_RCC_DRIVER_H_ */



