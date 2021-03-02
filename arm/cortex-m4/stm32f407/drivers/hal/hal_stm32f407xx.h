/*
 * stm32f407xx.h
 *
 *  Created on: Feb 14, 2021
 *      Author: fares.adham@gmail.com
 */

#ifndef INC_HAL_STM32F407XX_H_
#define INC_HAL_STM32F407XX_H_


#include <stdint.h>

typedef uint8_t   u8;
typedef uint16_t  u16;
typedef uint32_t  u32;
typedef uint64_t  u64;

typedef u64   U64;
typedef u32   U32;
typedef u16   U16;
typedef u8    U8;


#define BIT_0        (0 )
#define BIT_1        (1 )
#define BIT_2        (2 )
#define BIT_3        (3 )
#define BIT_4        (4 )
#define BIT_5        (5 )
#define BIT_6        (6 )
#define BIT_7        (7 )
#define BIT_8        (8 )
#define BIT_9        (9 )
#define BIT_10       (10)
#define BIT_11       (11)
#define BIT_12       (12)
#define BIT_13       (13)
#define BIT_14       (14)
#define BIT_15       (15)
#define BIT_16       (16)
#define BIT_17       (17)
#define BIT_18       (18)
#define BIT_19       (19)
#define BIT_20       (20)
#define BIT_21       (21)
#define BIT_22       (22)
#define BIT_23       (23)
#define BIT_24       (24)
#define BIT_25       (25)
#define BIT_26       (26)
#define BIT_27       (27)
#define BIT_28       (28)
#define BIT_29       (29)
#define BIT_30       (30)
#define BIT_31       (31)


#define HAL_MACRO_TOGGLE_REG_BIT(reg, bit)      ((*reg) ^= (1 << bit))
#define HAL_MACRO_TON_REG_BIT(reg, bit)         ((*reg) |= (1 << bit))
#define HAL_MACRO_TOFF_REG_BIT(reg, bit)        ((*reg) &= ~(1 << bit))


/*0x0 is wrong i need to change it later*/
#define HAL_INVALID_RAM_ADDRESS                 ((U32*)0x0)
#define nullptr                                 (HAL_INVALID_RAM_ADDRESS)

//#define WRITE_REGISTER(_REG_, _VAL_)

/* Memory definitions taken from Table 5. Flash module organization (STM32F40x and STM32F41x) RM75 */

#define HAL_FLASH_BASE_ADDRESS                (0x08000000U)               /* Main memory area */
#define HAL_FLASH_LENGTH                      (0x100000U)                 /* flash length in bytes, 1 MB total length */
#define HAL_FLASH_16KB_SECTOR_LENGTH          (0x4000U)                   /* small flash sector length is  16  KB*/
#define HAL_FLASH_64KB_SECTOR_LENGTH          (0x10000U)                  /* med flash sector length is    64  KB*/
#define HAL_FLASH_128KB_SECTOR_LENGTH         (0x20000U)                   /* large flash sector length is  128 KB*/

/* flashHAL_ can be separated and behaved as sectors below */
#define HAL_FLASH_SECTOR_0_BASE_ADDRESS       (HAL_FLASH_BASE_ADDRESS)                                                 /* 16KB flash Sector  */
#define HAL_FLASH_SECTOR_1_BASE_ADDRESS       (HAL_FLASH_SECTOR_0_BASE_ADDRESS +  HAL_FLASH_16KB_SECTOR_LENGTH)             /* 16KB flash Sector  */
#define HAL_FLASH_SECTOR_2_BASE_ADDRESS       (HAL_FLASH_SECTOR_1_BASE_ADDRESS +  HAL_FLASH_16KB_SECTOR_LENGTH)             /* 16KB flash Sector  */
#define HAL_FLASH_SECTOR_3_BASE_ADDRESS       (HAL_FLASH_SECTOR_2_BASE_ADDRESS +  HAL_FLASH_16KB_SECTOR_LENGTH)             /* 16KB flash Sector  */
#define HAL_FLASH_SECTOR_4_BASE_ADDRESS       (HAL_FLASH_SECTOR_3_BASE_ADDRESS +  HAL_FLASH_16KB_SECTOR_LENGTH)             /* 64KB flash Sector  */
#define HAL_FLASH_SECTOR_5_BASE_ADDRESS       (HAL_FLASH_SECTOR_4_BASE_ADDRESS +  HAL_FLASH_64KB_SECTOR_LENGTH)             /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_6_BASE_ADDRESS       (HAL_FLASH_SECTOR_5_BASE_ADDRESS +  HAL_FLASH_128KB_SECTOR_LENGTH)            /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_7_BASE_ADDRESS       (HAL_FLASH_SECTOR_6_BASE_ADDRESS +  HAL_FLASH_128KB_SECTOR_LENGTH)            /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_8_BASE_ADDRESS       (HAL_FLASH_SECTOR_7_BASE_ADDRESS +  HAL_FLASH_128KB_SECTOR_LENGTH)            /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_9_BASE_ADDRESS       (HAL_FLASH_SECTOR_8_BASE_ADDRESS +  HAL_FLASH_128KB_SECTOR_LENGTH)            /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_10_BASE_ADDRESS      (HAL_FLASH_SECTOR_9_BASE_ADDRESS +  HAL_FLASH_128KB_SECTOR_LENGTH)            /* 128KB flash Sector */
#define HAL_FLASH_SECTOR_11_BASE_ADDRESS      (HAL_FLASH_SECTOR_10_BASE_ADDRESS + HAL_FLASH_128KB_SECTOR_LENGTH)           /* 128KB flash Sector */

#define HAL_SYSTEM_MEM_BASE_ADDRESS           (0x1FFF0000U)
#define HAL_SYSTEM_MEM_LENGTH                 (0x7800U)   /* 30KB length */
#define HAL_ROM_BASE_ADDRESS                  (HAL_SYSTEM_MEM_BASE_ADDRESS)
#define HAL_ROM_LENGTH                        (HAL_SYSTEM_MEM_LENGTH)
#define HAL_OTP_BASE_ADDRESS                  (0x1FFF7800U)
#define HAL_OTP_LENGTH                        (0x210U)  /*512Bytes*/
#define HAL_OPTION_BYTES_BASE_ADDRESS         (0x1FFFC000U)
#define HAL_OPTION_BYTES_LENGTH               (0x10U) /*16bytes*/
#define HAL_SRAM1_BASE_ADDRESS                (0x20000000U)
#define HAL_SRAM1_LENGTH                      (0x1C000U)                  /* 112KB */
#define HAL_SRAM2_BASE_ADDRESS                (0x2001C000U)               /* (SRAM1_BASE_ADDRESS + SRAM1_LENGTH) */
#define HAL_SRAM2_LENGTH                      (0x4000)                   /* 16KB */
#define HAL_SRAM                              (HAL_SRAM1_BASE_ADDRESS)

/* Base addresses of various bus domains e.g. AHBx APBx
 * 2 AHB bus domain and 2 APB bus domains. AHB bus is used for those peripherals which need high speed data communication
 * e.g. camera and gpios. where APB bus is used for those peripherals which low speed communication would suffice
 * below is figure of the whole memory map of STM32F407 MCU
 * *-------------------------------------*------------------------------------------------------*
 * |                                     |                                                      |
 * |                                     |  memory mapped register and peripheral registers     |
 * *-------------------------------------*------------------------------------------------------*
 * 0x0                                   0x4000_000                                             0xFFFF_FFFF
 * */

#define HAL_PERIPHERALS_BASE_ADDRESS          (0x40000000U)
#define HAL_APB1_BUS_PERIPH_BASE_ADDRESS      (HAL_PERIPHERALS_BASE_ADDRESS)
#define HAL_APB2_BUS_PERIPH_BASE_ADDRESS      (0x40010000U)
#define HAL_AHB1_BUS_PERIPH_BASE_ADDRESS      (0x40020000U)
#define HAL_AHB2_BUS_PERIPH_BASE_ADDRESS      (0x50000000U)
/*definitions of various peripherals that hanging out on the buses above and in details:
 * AHB1 Peripherals: GPIOA...GPIOI
 * APB1 Peripherals:*/
                                          /*bus base address +             offset */
#define HAL_GPIO_A_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x0000)
#define HAL_GPIO_B_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x0400)
#define HAL_GPIO_C_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x0800)
#define HAL_GPIO_D_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x0C00)
#define HAL_GPIO_E_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x1000)
#define HAL_GPIO_F_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x1400)
#define HAL_GPIO_G_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x1800)
#define HAL_GPIO_H_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x1C00)
#define HAL_GPIO_I_BASE_ADDRESS                (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS  + 0x2000)

//APB1HAL_
#define HAL_SPI_2_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3800)
#define HAL_SPI_3_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3C00)
#define HAL_USART_2_BASE_ADDRESS               (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x4400)
#define HAL_USART_3_BASE_ADDRESS               (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x4800)
#define HAL_UART_4_BASE_ADDRESS                (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x4C00)
#define HAL_UART_5_BASE_ADDRESS                (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x5000)
#define HAL_UART_7_BASE_ADDRESS                (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x7800)
#define HAL_UART_8_BASE_ADDRESS                (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x7C00)
#define HAL_I2C_1_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x5400)
#define HAL_I2C_2_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x5800)
#define HAL_I2C_3_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS  + 0x5C00)

//APB2
#define HAL_USART_1_BASE_ADDRESS               (HAL_APB2_BUS_PERIPH_BASE_ADDRESS + 0x1000)
#define HAL_USART_6_BASE_ADDRESS               (HAL_APB2_BUS_PERIPH_BASE_ADDRESS + 0x1400)
#define HAL_SPI_1_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3000)
#define HAL_SPI_4_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3400)
#define HAL_SYSCFG_BASE_ADDRESS                (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3800)
#define HAL_EXTI_BASE_ADDRESS                  (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x3C00)
#define HAL_SPI_5_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x5000)
#define HAL_SPI_6_BASE_ADDRESS                 (HAL_APB1_BUS_PERIPH_BASE_ADDRESS + 0x5400)



/* GPIO Registers Section
 * This section gives a detailed description of the GPIO registers.
 * For a summary of register bits, register address offsets and reset values, see Table 39.
 * The GPIO registers can be accessed by byte (8 bits), half-words (16 bits) or words (32 bits).
 */

/* GPIO port mode register (GPIOx_MODER) (x = A..I/J/K)
 * OFSSET = 0x00 */
#define HAL_GPIO_A_MODER_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x00)
#define HAL_GPIO_B_MODER_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x00)
#define HAL_GPIO_C_MODER_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x00)
#define HAL_GPIO_D_MODER_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x00)
#define HAL_GPIO_E_MODER_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x00)
#define HAL_GPIO_F_MODER_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x00)
#define HAL_GPIO_G_MODER_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x00)
#define HAL_GPIO_H_MODER_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x00)
#define HAL_GPIO_I_MODER_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x00)

/* GPIO port output type register (GPIOx_OTYPER) (x = A..I/J/K)
 * OFFSET = 0x04 */
#define HAL_GPIO_A_OTYPER_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x04)
#define HAL_GPIO_B_OTYPER_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x04)
#define HAL_GPIO_C_OTYPER_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x04)
#define HAL_GPIO_D_OTYPER_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x04)
#define HAL_GPIO_E_OTYPER_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x04)
#define HAL_GPIO_F_OTYPER_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x04)
#define HAL_GPIO_G_OTYPER_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x04)
#define HAL_GPIO_H_OTYPER_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x04)
#define HAL_GPIO_I_OTYPER_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x04)

/* GPIO port output speed register (GPIOx_OSPEEDR) (x = A..I/J/K)
 * OFFSET = 0x08 */
#define HAL_GPIO_A_OSPEEDR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x08)
#define HAL_GPIO_B_OSPEEDR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x08)
#define HAL_GPIO_C_OSPEEDR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x08)
#define HAL_GPIO_D_OSPEEDR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x08)
#define HAL_GPIO_E_OSPEEDR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x08)
#define HAL_GPIO_F_OSPEEDR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x08)
#define HAL_GPIO_G_OSPEEDR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x08)
#define HAL_GPIO_H_OSPEEDR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x08)
#define HAL_GPIO_I_OSPEEDR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x08)

/* GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..I/J/K)
Address offset: 0x0C */
#define HAL_GPIO_A_PUPDR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_B_PUPDR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_C_PUPDR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_D_PUPDR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_E_PUPDR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_F_PUPDR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_G_PUPDR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_H_PUPDR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x0C)
#define HAL_GPIO_I_PUPDR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x0C)

/* GPIO port input data register (GPIOx_IDR) (x = A..I/J/K)
Address offset: 0x10 */
#define HAL_GPIO_A_IDR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x10)
#define HAL_GPIO_B_IDR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x10)
#define HAL_GPIO_C_IDR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x10)
#define HAL_GPIO_D_IDR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x10)
#define HAL_GPIO_E_IDR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x10)
#define HAL_GPIO_F_IDR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x10)
#define HAL_GPIO_G_IDR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x10)
#define HAL_GPIO_H_IDR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x10)
#define HAL_GPIO_I_IDR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x10)

/* GPIO port output data register (GPIOx_ODR) (x = A..I/J/K)
Address offset: 0x14 */
#define HAL_GPIO_A_ODR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x14)
#define HAL_GPIO_B_ODR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x14)
#define HAL_GPIO_C_ODR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x14)
#define HAL_GPIO_D_ODR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x14)
#define HAL_GPIO_E_ODR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x14)
#define HAL_GPIO_F_ODR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x14)
#define HAL_GPIO_G_ODR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x14)
#define HAL_GPIO_H_ODR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x14)
#define HAL_GPIO_I_ODR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x14)

/* GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K)
Address offset: 0x18 */
#define HAL_GPIO_A_BSRR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x18)
#define HAL_GPIO_B_BSRR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x18)
#define HAL_GPIO_C_BSRR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x18)
#define HAL_GPIO_D_BSRR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x18)
#define HAL_GPIO_E_BSRR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x18)
#define HAL_GPIO_F_BSRR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x18)
#define HAL_GPIO_G_BSRR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x18)
#define HAL_GPIO_H_BSRR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x18)
#define HAL_GPIO_I_BSRR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x18)

/* GPIO port configuration lock register (GPIOx_LCKR)
(x = A..I/J/K)
This register is used to lock the configuration of the port bits when a correct write sequence
is applied to bit 16 (LCKK). The value of bits [15:0] is used to lock the configuration of the
GPIO. During the write sequence, the value of LCKR[15:0] must not change. When the
LOCK sequence has been applied on a port bit, the value of this port bit can no longer be
modified until the next MCU or peripheral reset.
Address offset: 0x1C */
#define HAL_GPIO_A_LCKR_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_B_LCKR_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_C_LCKR_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_D_LCKR_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_E_LCKR_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_F_LCKR_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_G_LCKR_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_H_LCKR_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x1C)
#define HAL_GPIO_I_LCKR_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x1C)

/* GPIO alternate function low register (GPIOx_AFRL) (x = A..I/J/K)
Address offset: 0x20 */
#define HAL_GPIO_A_AFRL_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x20)
#define HAL_GPIO_B_AFRL_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x20)
#define HAL_GPIO_C_AFRL_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x20)
#define HAL_GPIO_D_AFRL_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x20)
#define HAL_GPIO_E_AFRL_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x20)
#define HAL_GPIO_F_AFRL_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x20)
#define HAL_GPIO_G_AFRL_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x20)
#define HAL_GPIO_H_AFRL_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x20)
#define HAL_GPIO_I_AFRL_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x20)

/* GPIO alternate function high register (GPIOx_AFRH) (x = A..I/J/K)
Address offset: 0x24 */
#define HAL_GPIO_A_AFRH_REGISTER              (HAL_GPIO_A_BASE_ADDRESS + 0x24)
#define HAL_GPIO_B_AFRH_REGISTER              (HAL_GPIO_B_BASE_ADDRESS + 0x24)
#define HAL_GPIO_C_AFRH_REGISTER              (HAL_GPIO_C_BASE_ADDRESS + 0x24)
#define HAL_GPIO_D_AFRH_REGISTER              (HAL_GPIO_D_BASE_ADDRESS + 0x24)
#define HAL_GPIO_E_AFRH_REGISTER              (HAL_GPIO_E_BASE_ADDRESS + 0x24)
#define HAL_GPIO_F_AFRH_REGISTER              (HAL_GPIO_F_BASE_ADDRESS + 0x24)
#define HAL_GPIO_G_AFRH_REGISTER              (HAL_GPIO_G_BASE_ADDRESS + 0x24)
#define HAL_GPIO_H_AFRH_REGISTER              (HAL_GPIO_H_BASE_ADDRESS + 0x24)
#define HAL_GPIO_I_AFRH_REGISTER              (HAL_GPIO_I_BASE_ADDRESS + 0x24)


/*SPI control register 1 (SPI_CR1) (not used in I 2 S mode)
Address offset: 0x00*/
#define HAL_SPI_CR1_REGISTER                  (HAL_SPI_1_BASE_ADDRESS + 0x00)
/*SPI control register 2 Address offset: 0x04*/
#define HAL_SPI_CR2_REGISTER                  (HAL_SPI_1_BASE_ADDRESS + 0x04)
/*SPI status register (SPI_SR) Address offset: 0x08*/
#define HAL_SPI_SR_REGISTER                   (HAL_SPI_1_BASE_ADDRESS + 0x08)
/*SPI data register (SPI_DR) Address offset: 0x0C*/
#define HAL_SPI_DR_REGISTER                   (HAL_SPI_1_BASE_ADDRESS + 0x0C)
/*SPI CRC polynomial register (SPI_CRCPR) (not used in I 2 S mode) Address offset: 0x10*/
#define HAL_SPI_CRCPR_REGISTER                (HAL_SPI_1_BASE_ADDRESS + 0x10)
/* SPI RX CRC register (SPI_RXCRCR) (not used in I 2 S mode) Address offset: 0x14 */
#define HAL_SPI_RXCRCR_REGISTER               (HAL_SPI_1_BASE_ADDRESS + 0x14)
/* SPI TX CRC register (SPI_TXCRCR) (not used in I 2 S mode) 0x18*/
#define HAL_SPI_TXCRCR_REGISTER               (HAL_SPI_1_BASE_ADDRESS + 0x18)
/*SPI_I 2 S configuration register (SPI_I2SCFGR) 0x1C*/
#define HAL_SPI_I2SCFGR_REGISTER              (HAL_SPI_1_BASE_ADDRESS + 0x1C)
/*SPI_I 2 S prescaler register (SPI_I2SPR) 0x20*/
#define HAL_SPI_I2SPR_REGISTER                (HAL_SPI_1_BASE_ADDRESS + 0x20)



#define HAL_RCC_BASE_ADDRESS  (HAL_AHB1_BUS_PERIPH_BASE_ADDRESS + 0x3800U)


#define GPIO_AFR_LOW         (0x0)
#define GPIO_AFR_HIGH        (0x1)





#endif /* INC_HAL_STM32F407XX_H_ */
