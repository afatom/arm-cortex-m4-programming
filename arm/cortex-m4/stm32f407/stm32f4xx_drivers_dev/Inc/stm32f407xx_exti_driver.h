/*
 * irq_driver.h
 *
 *  Created on: 24 Feb 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_EXTI_DRIVER_H_
#define INC_STM32F407XX_EXTI_DRIVER_H_

#include "hal_stm32f407xx.h"

typedef struct {
	volatile U32 IMR;	/* Interrupt mask register. mask/unmask IRQ at line x */
	volatile U32 EMR;	/* Event mask register.  mask/unmask ERQ at line x*/
	volatile U32 RTSR;	/* Rising trigger selection register. 1/0 Rising trigger en/di (for Event and Interrupt) for input line */
	volatile U32 FTSR;	/* falling trigger selection register. 1/0 falling trigger en/di (for Event and Interrupt) for input line */

	volatile U32 SWIER;	/* Software interrupt event register
				   If interrupt are enabled on line x in the EXTI_IMR register, writing '1' to SWIERx bit when it is
				   set at '0' sets the corresponding pending bit in the EXTI_PR register, thus resulting in an
				   interrupt request generation.
				   This bit is cleared by clearing the corresponding bit in EXTI_PR (by writing a 1 to the bit).*/

	volatile U32 PR;	/* Pending Register.
				   0: No trigger request occurred
				   1: selected trigger request occurred
			           This bit is set when the selected edge event arrives on the external interrupt line.
				   This bit is cleared by programming it to ‘1’. */
}EXTI_RegDef_t;




#define EXTI                 ((EXTI_RegDef_t*)(HAL_EXTI_BASE_ADDRESS))
#define IRQ_DRIVER           (EXTI)
#define IRQDRV               (IRQ_DRIVER)

//struct exti_driver {
//	struct exti_regs    *extr;         /*user dont need to know about the struct exti_regs*/
//	struct syscfg_regs  *sysr;
//	volatile U32        *pNVIQIrqEnReg;
//};


void exti_driver_init(EXTI_RegDef_t *pEXTI);
void exti_driver_deinit(EXTI_RegDef_t *pEXTI);
//void exti_driver_init_PeriClkCtrl(exti_driver *handle, U8 cmd); should be static in c file
void exti_driver_gpio_irq_enable(EXTI_RegDef_t *pEXTI, U8 gpio_port, U8 gpio_pin, void(*gpio_isr)(U8));
void exti_driver_gpio_irq_disable(EXTI_RegDef_t *pEXTI, U8 gpio_port, U8 gpio_pin);


#endif /* INC_STM32F407XX_EXTI_DRIVER_H_ */
