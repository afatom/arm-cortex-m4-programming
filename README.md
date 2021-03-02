# embedded
rt embedded projects, applications, drivers development and more.
Directory structure

embedded
        |_Arm
            |_cortex_m4
                      |_stm32f407xx
                                  |_hal (hw abstraction layer for drivers & projects)
                                  |_drivers (drivers layer)
                                  |       |_GPIO (GPIO driver)
                                  |       |_SPI (SPI driver)
                                  |       |_UART (UART driver)
                                  |       |_I2C (I2C driver)
                                  |       |_RCC (reset and clk controll driver)
                                  |       |_SYSCFG (system configuration and control driver)
                                  |       |_EXTI (external interrupt controler driver)
                                  |
                                  |
                                  |
                                  |_projects (projects directory)
                                          |_blink_led
                                          |_...
               
