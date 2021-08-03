/*
 * irq_driver.c
 *
 *  Created on: 24 Feb 2021
 *      Author: amach
 */
#include <stm32f407xx_exti_driver.h>

/*
 * EXTI controller connected to the NVIC via 23 line (23 IRQs can be established via those line)
 * 16/23 lines serving GPIOs. By default the EXTI lines are disabled. enable them via IMR register.
 *
 * 1) 	select which entry goes out by configuring the SYSCFG_EXTICRx register
 * 	In other words link GPIO port x pin y to EXTI line y via register SYCFG_EXTICRy
 *
 * 2)	Configure the trigger detection (falling\rising\both) for relevant EXTI line
 * 	Via EXTI_CR register
 *
 * 3)	register or subscribe the GPIO User ISR
 * */

void exti_driver_init(EXTI_RegDef_t *pEXTI)
{
//	exti->extr = EXTI;
//	exti->sysr = SYSCFG;
//	exti->pNVIQIrqEnReg = (U32*)0xE000E100; /* I need to know the source of this address in the ARM docs */
//
//	/* enable pclk to syscfg controller */
//	RCC_DRIVER_SYSCFG_PCLK_EN();
}

void exti_driver_deinit(EXTI_RegDef_t *pEXTI)
{
//	exti->extr = nullptr;
//	exti->sysr = nullptr;
//	exti->pNVIQIrqEnReg = nullptr; /* I need to know the source of this address in the ARM docs */
//
//	RCC_DRIVER_SYSCFG_PCLK_DI();
}


/*
void IRQDRV_PeriClkCtrl(IRQDRV_Handle_t *handle, U8 cmd)
{

}
*/

void exti_driver_gpio_irq_enable(EXTI_RegDef_t *pEXTI, U8 gpio_port, U8 gpio_pin, void(*gpio_isr)(U8)){}
void exti_driver_gpio_irq_disable(EXTI_RegDef_t *pEXTI, U8 gpio_port, U8 gpio_pin){}

