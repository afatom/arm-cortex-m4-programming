/*
 * syscfg_driver.h
 *
 *  Created on: 24 Feb 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_SYSCFG_DRIVER_H_
#define INC_STM32F407XX_SYSCFG_DRIVER_H_

#include "hal_stm32f407xx.h"

#define EXTICR1_TABLE_ENTRY             (0x0) //EXTICR1 REG
#define EXTICR2_TABLE_ENTRY             (0x1) //EXTICR1 REG
#define EXTICR3_TABLE_ENTRY             (0x2) //EXTICR1 REG
#define EXTICR4_TABLE_ENTRY             (0x3) //EXTICR1 REG




typedef struct {
	volatile U32 MEMRMP;
	volatile U32 PMC;
	volatile U32 EXTICRTABLE[4];
	U32 RESERVED1[2];
	volatile U32 CMPCR;
	U32 RESERVED2[2];
	volatile U32 CFGR;;
}SYSCFG_RegDef_t;



#define SYSCFG    ((SYSCFG_RegDef_t*)HAL_SYSCFG_BASE_ADDRESS)



/*
void syscfg_driver_init(struct syscfg_driver *sys);
void syscfg_driver_deinit(struct syscfg_driver *sys);
void syscfg_driver_extint_line_connect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin);
void ssyscfg_driver_extint_line_disconnect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin);
*/

/**
 * @function: SYSCFG_ExtiIRQConfig(): configure the GPIO port interrupt delivery
 * over EXTI lines
 *
 * @pSYSCFG: syscfg regdef base address
 * @GPIOPort: GPIOPort GPIO Port Base address
 * @GPIOPin: pin number in this gpio port
 *
 * @return: none
 */
void SYSCFG_ExtiIRQConfig(SYSCFG_RegDef_t *pSYSCFG, U8 GPIOPortId, U8 GPIOPin);

void SYSCFG_DriverReset(void);
void SYSCFG_DriverInit(void);

#endif /* INC_STM32F407XX_SYSCFG_DRIVER_H_ */
