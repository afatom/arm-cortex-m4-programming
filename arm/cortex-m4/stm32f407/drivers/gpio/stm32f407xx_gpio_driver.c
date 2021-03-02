/*
 * stm32f407xx_gpio_driver.c
 *
 *  Created on: 16 Feb 2021
 *      Author: amach
 */

#include <stdint.h>

#include "hal_stm32f407xx.h"
#include "stm32f407xx_gpio_driver.h"
#include "stm32f407xx_rcc_driver.h"


static u8 get_gpio_port(GPIO_RegDef_t *pGPIOx);

/*APIs supported by GPIO driver*/

/**
 * GPIO_Init(): Initialize the GPIO relative port driver layer,
 *              configure the requested pin in that port according to structure handle
 *              user must call this function before working with GPIO
 *
 * @handle: caller passes the suitable GPIO port and pin configuration in this argument
 *
 * Return: void
 */
void GPIO_Init(GPIO_Handle_t *handle)
{
	u8 pin_number = handle->GPIO_PinConfig.GPIO_PinNumber;

	/* configure the GPIO pin mode */
	if (handle->GPIO_PinConfig.GPIO_PinMode <= GPIO_ANALOG_MODE) {
		/* non interrupt driven modes */
		/* read the user defined mode for this pin and write it to the MODER register */
		/* No error checking here, if user writes invalid values, behavior is undefined */
		handle->pGPIOx->MODER &= ~(0x3 << (2 * pin_number)); /* clear bits before write */
		handle->pGPIOx->MODER |= ( (handle->GPIO_PinConfig.GPIO_PinMode) << (2 * pin_number) );

		/* configure GPIO port output type register for this PIN */
		handle->pGPIOx->OTYPER &= ~(0x1 << pin_number);
		handle->pGPIOx->OTYPER |= ( (handle->GPIO_PinConfig.GPIO_PinOPType) << (pin_number) );

		/* configure GPIO port output speed register */
		handle->pGPIOx->OSPEEDR &= ~(0x3 << (2 * pin_number)); /* clear bits before write */
		handle->pGPIOx->OSPEEDR |= ( (handle->GPIO_PinConfig.GPIO_PinSpeed) << (2 * pin_number) );

		/* configure GPIO port pull-up/pull-down register */
		handle->pGPIOx->PUPDR &= ~(0x3 << (2 * pin_number)); /* clear bits before write */
		handle->pGPIOx->PUPDR |= ( (handle->GPIO_PinConfig.GPIO_PinPupdControl) << (2 * pin_number) );

		/* configure GPIO port alternate function register only if the mode is GPIO_ALTERNATE_FUNCTION_MODE*/
		if (handle->GPIO_PinConfig.GPIO_PinMode == GPIO_ALTERNATE_FUNCTION_MODE) {
			handle->pGPIOx->AFR[pin_number/8] &= ~(0xF << (4 * (pin_number % 8)));
			handle->pGPIOx->AFR[pin_number/8] |= ( (handle->GPIO_PinConfig.GPIO_PinAltFuncMode) << (4 * (pin_number % 8)) );
		}

	} else {
		/* to be coded later*/
	}
	/* config gpio speed */

	/*config output type*/

	/* config pupd settings */

	/*config alt function*/
}

/**
 * GPIO_Reset(): resets GPIO port to default state (via RCC reset register)
 *              user must call this function at the end
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 *
 * Return: void
 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx)
{
	GPIO_X_REG_RESET(get_gpio_port(pGPIOx));
}

/**
 * GPIO_PeriClkCtrl(): clock configuration, enables or disables the clock to GPIO port
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 * @e: enumeration for disabling or enabling clock
 *
 * Return: void
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, u8 cfg)
{
	u8 port;

	port = get_gpio_port(pGPIOx);

	if(GPIO_ENABLE == cfg) {
		GPIO_X_PCLK_EN(port);
	} else {
		GPIO_X_PCLK_DI(port);
	}
}

static u8 get_gpio_port(GPIO_RegDef_t *pGPIOx)
{
	u8 port;

	if (GPIO_A == pGPIOx) {
		port = 0;
	} else if (GPIO_B == pGPIOx) {
		port = 1;
	} else if (GPIO_C == pGPIOx) {
		port = 2;
	} else if (GPIO_D == pGPIOx) {
		port = 3;
	} else if (GPIO_E == pGPIOx) {
		port = 4;
	} else if (GPIO_F == pGPIOx) {
		port = 5;
	} else if (GPIO_G == pGPIOx) {
		port = 6;
	} else if (GPIO_H == pGPIOx) {
		port = 7;
	} else {
		port = 8;
	}

	return port;
}

/**
 * GPIO_ReadFromInputPin(): read data from GPIO pin
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 * @pin: from which pin data need to be read from
 *
 * Return: u8, "1" for high and "0" for low
 */
u8  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, u8 pin)
{
	/* They contain the input value of the corresponding I/O port.*/
	return (u8)( (pGPIOx->IDR >> pin) & 0x1 );
}

/**
 * GPIO_ReadFromInputPort(): read data from GPIO entire port
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 *
 * Return: u16, (each bit represents the data in the suitable oin in the above port)
 */
u16 GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx)
{
	return (u16)(pGPIOx->IDR);
}

/**
 * GPIO_WriteToOutputPin(): write data to GPIO pin
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 * @pin: oin number to be written to
 * @val: "1" or "0" valid values to be written
 *
 * Return: void
 */
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, u8 pin, u8 val)
{
	if (val >= GPIO_SET) {
		pGPIOx->ODR |= (1 << pin);
	} else {
		pGPIOx->ODR &= ~(1 << pin);
	}
}

/**
 * GPIO_WriteToOutputPin(): write data to GPIO entire port
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 * @val: "1" or "0" valid values to be written. val is behaved as bit representing
 * mask where each bit represents the value for the suitable pin
 *
 * Return: void
 */
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, u16 val)
{
	pGPIOx->ODR = val;
}

/**
 * GPIO_WriteToOutputPin(): write data to GPIO entire port
 *
 * @pGPIOx: caller passes the suitable GPIO port in this argument
 * @val: "1" or "0" valid values to be written. val is behaved as bit representing
 * mask where each bit represents the value for the suitable pin
 *
 * Return: void
 */
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, u8 pin)
{
	pGPIOx->ODR ^= (1 << pin);
}

/**
 * GPIO_IRQConfig(): configure priority, enable/disable, the IRQ accosiated with
 *                   GPIO
 *
 * @irq_num: IRQ number to be configured
 * @irq_p: IRQ priority (see rm)
 * @e: disable (mask) or eable (unmask) the GPIO interrupt (IRQ)
 *
 * Return: void
 */
void GPIO_IRQConfig(u8 irq_num, u8 irq_p, u8 enable)
{

}

/*the isr needs to know from which pin the irq was raised*/
/**
 * GPIO_ISR(): This is the ISR for that will be called upon each IRQ fired above
 * the isr needs to know from whic pin the IRQ was fired hence the argument pin
 *
 * @pin: the is the pin nuber that issued the IRQ
 *
 * Return: void
 */
void GPIO_ISR(u8 pin)
{

}

