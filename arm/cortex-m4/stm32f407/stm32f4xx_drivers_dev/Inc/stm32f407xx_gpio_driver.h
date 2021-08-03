/*
 * stm32f407xx_gpio_driver.h
 *
 *  Created on: 16 Feb 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_GPIO_DRIVER_H_
#define INC_STM32F407XX_GPIO_DRIVER_H_


/*
 * Peripheral registers definition structure for GPIO
 */
typedef struct {
	volatile uint32_t MODER;      /* GPIO port mode register (GPIOx_MODER) (x = A..I/J/K) */
	volatile uint32_t OTYPER;     /* GPIO port output type register (GPIOx_OTYPER) (x = A..I/J/K) */
	volatile uint32_t OSPEEDR;    /* GPIO port output speed register (GPIOx_OSPEEDR) (x = A..I/J/K) */
	volatile uint32_t PUPDR;      /* GPIO port pull-up/pull-down register (GPIOx_PUPDR) (x = A..I/J/K)*/
	volatile uint32_t IDR;        /* GPIO port input data register (GPIOx_IDR) (x = A..I/J/K) */
	volatile uint32_t ODR;        /* GPIO port output data register (GPIOx_ODR) (x = A..I/J/K) */
	volatile uint32_t BSRR;       /* GPIO port bit set/reset register (GPIOx_BSRR) (x = A..I/J/K) */
	volatile uint32_t LCKR;       /* GPIO port configuration lock register (GPIOx_LCKR)*/
	volatile uint32_t AFR[2];     /* GPIO alternate function low=AFR[0] & high=AFR[1] register*/
}GPIO_RegDef_t;

/*
 * Peripheral definitions (Peripheral base addresses type casted to ???_RegDef_t pointer type)
 */
#define GPIO_A               ((GPIO_RegDef_t*)HAL_GPIO_A_BASE_ADDRESS)
#define GPIO_B               ((GPIO_RegDef_t*)HAL_GPIO_B_BASE_ADDRESS)
#define GPIO_C               ((GPIO_RegDef_t*)HAL_GPIO_C_BASE_ADDRESS)
#define GPIO_D               ((GPIO_RegDef_t*)HAL_GPIO_D_BASE_ADDRESS)
#define GPIO_E               ((GPIO_RegDef_t*)HAL_GPIO_E_BASE_ADDRESS)
#define GPIO_F               ((GPIO_RegDef_t*)HAL_GPIO_F_BASE_ADDRESS)
#define GPIO_G               ((GPIO_RegDef_t*)HAL_GPIO_G_BASE_ADDRESS)
#define GPIO_H               ((GPIO_RegDef_t*)HAL_GPIO_H_BASE_ADDRESS)
#define GPIO_I               ((GPIO_RegDef_t*)HAL_GPIO_I_BASE_ADDRESS)
#define GPIO_J               ((GPIO_RegDef_t*)HAL_GPIO_J_BASE_ADDRESS)
#define GPIO_K               ((GPIO_RegDef_t*)HAL_GPIO_K_BASE_ADDRESS)


/******************************************************************************
 *                          User defined Public types
 ******************************************************************************/

/**
 * struct GPIO_PinConfig_t - GPIO Pin definition
 * @pGPIOx: pointer to memory mapped GPIO registers area (the memory type casted to GPIO_RegDef_t
 * 			structure.
 * @GPIO_PinConfig: GPIO pin configuration settings filled by caller //#TBD: why not pointer ?
 *
 * Longer description -
 */
typedef struct {
	u8 GPIO_PinNumber;         /* 0-15 pins per GPIO port */
	u8 GPIO_PinMode;           /* input, output or AF */
	u8 GPIO_PinSpeed;          /* Pin max speed, low, med, high and very high speeds see RM */
	u8 GPIO_PinPupdControl;    /* pin pull-up/pull-down */
	u8 GPIO_PinOPType;         /**/
	u8 GPIO_PinAltFuncMode;
}GPIO_PinConfig_t;

/**
 * struct GPIO_Handle_t - handle which contains the GPIO port,pin configurations
 * @pGPIOx: pointer to memory mapped GPIO registers area (the memory type casted to GPIO_RegDef_t
 * 			structure.
 * @GPIO_PinConfig: GPIO pin configuration settings filled by caller //#TBD: why not pointer ?
 *
 * Longer description -
 */
typedef struct {
	GPIO_RegDef_t      *pGPIOx; /* the base address of the GPIO port that which the pin belongs*/
	GPIO_PinConfig_t   GPIO_PinConfig;  /*GPIO pin config settings*/
}GPIO_Handle_t;

/******************************************************************************
 * MACROS and definitions
 ******************************************************************************/

/*
 * GPIO initial states
 */
#define GPIO_SET          (0x1)
#define GPIO_RESET        (0x0)
#define GPIO_ENABLE       (GPIO_SET)
#define GPIO_DISABLE      (GPIO_RESET)

/*
 * GPIO pin possible modes definitions
 * These bits are written by software to configure the I/O direction mode
 */
/* non interrupt driven GPIO modes */
#define GPIO_INPUT_RESET_MODE                   (0x0)      /* CAUTION: do not change this value */
#define GPIO_GENERAL_PURPOSE_OUTPUT_MODE        (0x1)
#define GPIO_ALTERNATE_FUNCTION_MODE            (0x2)
#define GPIO_ANALOG_MODE                        (0x3)
/* interrupt driven GPIO modes */
#define GPIO_INPUT_FT                           (0x4)      /* falling edge trigger input pin interrupt mode */
#define GPIO_INPUT_RT                           (0x5)      /* rising edge trigger input pin interrupt mode */
#define GPIO_INPUT_RFT                          (0x6)      /* rising and falling edge trigger input */


#define GPIO_IN_IRQ_FALLING_EDGE_TRIG_MODE      (GPIO_INPUT_FT)
#define GPIO_IN_IRQ_RISING_EDGE_TRIG_MODE      (GPIO_INPUT_RT)
#define GPIO_IN_IRQ_FALLRISE_EDGE_TRIG_MODE      (GPIO_INPUT_RFT)


/*
 * GPIO output definitions
 */
#define GPIO_OP_TYPE_PUSH_PULL                  (0x0)
#define GPIO_OP_TYPE_OPEN_DRAIN                 (0x1)

/*
 * GPIO input and output definitions
 */
#define GPIO_SPEED_LOW         (0x0)
#define GPIO_SPEED_MEDIUM      (0x1)
#define GPIO_SPEED_FAST        (0x2)
#define GPIO_SPEED_HIGH        (0x3)

#define GPIO_NO_PUPD          (0x0)
#define GPIO_PIN_PU           (0x1)
#define GPIO_PIN_PD           (0x2)


/* Macros and user definitions */
#define GPIO_PIN_NO_0        (0)
#define GPIO_PIN_NO_1        (1)
#define GPIO_PIN_NO_2        (2)
#define GPIO_PIN_NO_3        (3)
#define GPIO_PIN_NO_4        (4)
#define GPIO_PIN_NO_5        (5)
#define GPIO_PIN_NO_6        (6)
#define GPIO_PIN_NO_7        (7)
#define GPIO_PIN_NO_8        (8)
#define GPIO_PIN_NO_9        (9)
#define GPIO_PIN_NO_10       (10)
#define GPIO_PIN_NO_11       (11)
#define GPIO_PIN_NO_12       (12)
#define GPIO_PIN_NO_13       (13)
#define GPIO_PIN_NO_14       (14)
#define GPIO_PIN_NO_15       (15)




/******************************************************************************************************
 * APIs supported by GPIO driver
 * ****************************************************************************************************/

/**
 * @function: GPIO_Init(): Initialize GPIOx Port with Pinx passed by the caller. This fn should be called
 * for each GPIO pin
 * @arg1: gpio handle structure, filled by the caller which describes the pin and caller requests
 * @warning: no error handling or null pointer check.
 *
 * @retval: none
 */
void GPIO_Init(GPIO_Handle_t *handle);

/**
 * @function: GPIO_Reset(): de initialize/reset GPIOx Port with Pinx passed by the caller to default state.
 * This fn should be called for each GPIO pin user wants to reset.
 * @arg1: pGPIOx pointer to gpio port
 * @warning: no error handling or null pointer check.
 *
 * @retval: none
 */
void GPIO_Reset(GPIO_RegDef_t *pGPIOx);

/**
 * @function: GPIO_PeriClkCtrl(): clock configuration, enables or disables the clock to GPIO port
 *
 * @arg1: pGPIOx: caller passes the GPIO port pointer in this argument
 * @arg2: en, passed as C macro for setting/enabling or disabling the clock to GPIO port
 * @warning: no error handling or null pointer check.
 *
 * @retval: none
 */
void GPIO_PeriClkCtrl(GPIO_RegDef_t *pGPIOx, u8 en);

/*GPIO driver data read and write*/
u8  GPIO_ReadFromInputPin(GPIO_RegDef_t *pGPIOx, u8 pin);
u16 GPIO_ReadFromInputPort(GPIO_RegDef_t *pGPIOx);
void GPIO_WriteToOutputPin(GPIO_RegDef_t *pGPIOx, u8 pin, u8 val);
void GPIO_WriteToOutputPort(GPIO_RegDef_t *pGPIOx, u16 val);
void GPIO_ToggleOutputPin(GPIO_RegDef_t *pGPIOx, u8 pin);
/*GPIO IRQ and ISR configuration*/
void GPIO_IRQConfig(u8 IRQNumber, u8 EnableOrDisable);
/*the isr needs to know from which pin the irq was raised*/
void GPIO_ISR(u8 pin);
void GPIO_IRQPriorityConfig(u8 IRQNumber, u32 IRQPriority);

#endif /* INC_STM32F407XX_GPIO_DRIVER_H_ */
