/*
 * syscfg_driver.c
 *
 *  Created on: 25 Feb 2021
 *      Author: amach
 */


#include <stm32f407xx_syscfg_driver.h>


/* init driver and enable rcc clock feed */
void syscfg_driver_init(struct syscfg_driver *sys)
{

}

/* reset driver registers and disable rcc clock feed */
void syscfg_driver_deinit(struct syscfg_driver *sys)
{

}

/* select/enable external interrupt EXTI line associated with GPIO port & Pin */
void syscfg_driver_extint_line_connect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin)
{

}

/* deselect/disable external interrupt EXTI line associated with GPIO port & Pin */
void ssyscfg_driver_extint_line_disconnect(struct syscfg_driver *sys, U8 gpio_port, U8 gpio_pin)
{

}
