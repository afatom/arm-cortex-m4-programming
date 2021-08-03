/*
 * stm32f407xx_i2c_driver.h
 *
 *  Created on: 2 Aug 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_I2C_DRIVER_H_
#define INC_STM32F407XX_I2C_DRIVER_H_

#include "hal_stm32f407xx.h"


typedef struct {
	_vou32 I2C_CR1;		/* I2C Control register 1 (I2C_CR1) */
	_vou32 I2C_CR2;		/* I2C Control register 2 (I2C_CR2) */
	_vou32 I2C_OAR1;	/* I 2 C Own address register 1 (I2C_OAR1) */
	_vou32 I2C_OAR2;	/* I 2 C Own address register 2 (I2C_OAR2) */
	_vou32 I2C_DR;		/* I 2 C Data register (I2C_DR) */
	_vou32 I2C_SR1;		/* I 2 C Status register 1 (I2C_SR1) */
	_vou32 I2C_SR2;		/* I 2 C Status register 2 (I2C_SR2) */
	_vou32 I2C_CCR;		/* I 2 C Clock control register (I2C_CCR) */
	_vou32 I2C_TRISE;   /* I 2 C TRISE register (I2C_TRISE) controls the maximum duration of the SCL feedback loop in master mode.*/
	_vou32 I2C_FLTR;    /* Noise FILTER control */ /* The I2C_FLTR is available on STM32F42xxx and STM32F43xxx only.*/
}I2C_RegDef_t;



typedef struct {
	U8 MasterSlaveCfg;
	U8 AckEnable;
	U8 I2CBusSpeed;

}I2C_Config_t;

typedef struct {
	I2C_RegDef_t *pI2Cx;
	I2C_Config_t I2Cx_Config;
}I2C_Handle_t;


#endif /* INC_STM32F407XX_I2C_DRIVER_H_ */
