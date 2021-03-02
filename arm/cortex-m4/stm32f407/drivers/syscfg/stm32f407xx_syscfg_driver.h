/*
 * syscfg_driver.h
 *
 *  Created on: 24 Feb 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_SYSCFG_DRIVER_H_
#define INC_STM32F407XX_SYSCFG_DRIVER_H_

#include "hal_stm32f407xx.h"


struct syscfg_regs {
	volatile U32 MEMRMP;
	volatile U32 PMC;
	volatile U32 EXTICR1;
	volatile U32 EXTICR2;
	volatile U32 EXTICR3;
	volatile U32 EXTICR4;
	volatile U32 CMPCR;
};


struct syscfg_driver {
	struct syscfg_regs sysc;
};


#define SYSCFG ((syscfg_regs*)HAL_SYSCFG_BASE_ADDRESS)

void syscfg_driver_init(struct syscfg_driver *sys);
void syscfg_driver_deinit(struct syscfg_driver *sys);
void syscfg_driver_extint_line_connect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin);
void ssyscfg_driver_extint_line_disconnect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin);


#endif /* INC_STM32F407XX_SYSCFG_DRIVER_H_ */
