/*
 * stm32f4xx_syscfg_driver.c
 *
 *  Created on: 28 Jul 2021
 *      Author: amach
 */

#include "hal_stm32f407xx.h"
#include "stm32f407xx_rcc_driver.h"
#include "stm32f407xx_syscfg_driver.h"


#define M_GET_EXTI_TABLE_ENTRY(GpioPinNum)      ((GpioPinNum)/(4))

/* select the GPIO port and number to dilever its interrupt over EXTI approprate pin */
#define M_EXTI_IRQ_ENTRY_SELECT(GpioPort,GpioPinNum)\
	do {\
		SYSCFG->EXTICRTABLE[M_GET_EXTI_TABLE_ENTRY((GpioPinNum))] |= ((GpioPort) << (((GpioPinNum) % 4)*4));\
	} while (0)


#define GPIO_PORT_ID_TO_SYSCFG_GPIO_CODE(portId)\
	( (portId == GPIO_PORT_A_ID)?0:\
	  (portId == GPIO_PORT_B_ID)?1:\
	  (portId == GPIO_PORT_C_ID)?2:\
	  (portId == GPIO_PORT_D_ID)?3:\
	  (portId == GPIO_PORT_E_ID)?4:\
	  (portId == GPIO_PORT_F_ID)?5:\
	  (portId == GPIO_PORT_G_ID)?6:\
	  (portId == GPIO_PORT_H_ID)?7:\
	  (portId == GPIO_PORT_I_ID)?8:\
	  (portId == GPIO_PORT_J_ID)?9:10)



void SYSCFG_ExtiIRQConfig(SYSCFG_RegDef_t *pSYSCFG, U8 GPIOPortId, U8 GPIOPin)
{
	/* get the right table entry in the control registers table */
	U8 tableEntry = GPIOPin/4;
	/* get the right control register offset */
	U8 registerOffset =  (GPIOPin % 4) * 4;
	U8 PortCode = GPIO_PORT_ID_TO_SYSCFG_GPIO_CODE(GPIOPortId);
	pSYSCFG->EXTICRTABLE[tableEntry] |= (PortCode << registerOffset);
}


void SYSCFG_DriverInit(void)
{
	//Bit 14 SYSCFGEN: System configuration controller clock enable

	RCC_DRIVER->rcc_regs.APB2ENR |= (1 << 14);
}

void SYSCFG_DriverReset(void)
{
	RCC_DRIVER->rcc_regs.APB2ENR &= ~(1 << 14);
}
