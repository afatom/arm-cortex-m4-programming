/*
 * stm32f407xx_spi_driver.h
 *
 *  Created on: 5 Jul 2021
 *      Author: amach
 */

#ifndef INC_STM32F407XX_SPI_DRIVER_H_
#define INC_STM32F407XX_SPI_DRIVER_H_
#include "hal_stm32f407xx.h"

typedef struct {
	_vou32 SPI_CR1;
	_vou32 SPI_CR2;
	_vou32 SPI_SR;
	_vou32 SPI_DR;
	_vou32 SPI_CRCPR;
	_vou32 SPI_RXCRCR;
	_vou32 SPI_TXCRCR;
	_vou32 SPI_I2SCFGR;
	_vou32 SPI_I2SPR;
}SPI_RegDef_t;

/* config SPIx structure;; now we colud use bitfields!! we dont need all this struct size */
typedef struct {
	u8 SPI_DeviceMode;
	u8 SPI_BusConfig;
	u8 SPI_SclkSpeed;
	u8 SPI_DFFLen;
	u8 SPI_DFFOrder;
	u8 SPI_CPOL;
	u8 SPI_CPHA;
	u8 SPI_SSM;
}SPI_Config_t;

typedef struct {
	SPI_RegDef_t  *pSPIx;
	SPI_Config_t  SPIConfig;
	const U8 *pTxBuffer;
	U8 *pRxBuffer;
	U32 TxLen;
	U32 RxLen;
	U8 TxState;
	U8 RxState;
}SPI_Handle_t;



#define SPI_READY               0
#define SPI_BUSY_IN_RX          1
#define SPI_BUSY_IN_TX          2


#define SPI1     ((SPI_RegDef_t*)HAL_SPI_1_BASE_ADDRESS)
#define SPI2     ((SPI_RegDef_t*)HAL_SPI_2_BASE_ADDRESS)
#define SPI3     ((SPI_RegDef_t*)HAL_SPI_3_BASE_ADDRESS)
#define SPI4     ((SPI_RegDef_t*)HAL_SPI_4_BASE_ADDRESS)
#define SPI5     ((SPI_RegDef_t*)HAL_SPI_5_BASE_ADDRESS)
#define SPI6     ((SPI_RegDef_t*)HAL_SPI_6_BASE_ADDRESS)

/*SPI Global Defs*/
#define SPI_LOW                 0
#define SPI_HIGH                1


/*bit positions defs for SPI peripheral CR1 register*/
#define   SPI_CR1_CPHA_BIT				(0)
#define   SPI_CR1_CPOL_BIT				(1)
#define   SPI_CR1_MSTR_BIT				(2)
#define   SPI_CR1_BR_BIT				(3)
#define   SPI_CR1_SPE_BIT               (6)
#define   SPI_CR1_LSB_FIRST_BIT			(7)
#define   SPI_CR1_SSI_BIT               (8)
#define   SPI_CR1_SSM_BIT				(9)
#define   SPI_CR1_RXONLY_BIT			(10)
#define   SPI_CR1_DFF_BIT				(11)
#define   SPI_CR1_BIDI_OUTEN_BIT		(14)
#define   SPI_CR1_BIDI_MODE_BIT			(15)

/*bit positions defs for SPI peripheral SR register*/
#define SPI_SR_RXNE_BIT			(0)
#define SPI_SR_TXE_BIT    		(1)
#define SPI_SR_CHSIDE_BIT    	(2)
#define SPI_SR_UDR_BIT    		(3)
#define SPI_SR_CRCERR_BIT    	(4)
#define SPI_SR_MODF_BIT    		(5)
#define SPI_SR_OVR_BIT    		(6)
#define SPI_SR_BSY_BIT    		(7)
#define SPI_SR_FRE_BIT    		(8)

/*bit positions defs for SPI peripheral CR2 register*/
#define SPI_CR2_RXDMAEN_BIT			(0)
#define SPI_CR2_TXDMAEN_BIT    		(1)
#define SPI_CR2_SSOE_BIT    	(2)
#define SPI_CR2_FRF_BIT    	(4)
#define SPI_CR2_ERRIE_BIT    		(5)
#define SPI_CR2_RXNEIE_BIT    		(6)
#define SPI_CR2_TXEIE_BIT    		(7)



#define TX_BUFFER_EMPTY              (1)
#define TX_BUFFER_NOT_EMPTY          (0)

#define RX_BUFFER_EMPTY              (0)
#define RX_BUFFER_NOT_EMPTY          (1)

/*SPI device modes*/
#define SPI_DEV_MODE_MASTER 1
#define SPI_DEV_MODE_SLAVE 0

/*SPI BUS Config options*/
#define SPI_HALF_DUPLEX_BUS_CFG                 (2)
#define SPI_FULL_DUPLEX_BUS_CFG                 (1)
#define SPI_SIMPLEX_RXONLY_BUS_CFG              (3)

/*Speed => Baud rate control*/
#define SPI_SCLK_SPEED_DEV_BY_2              	(0)
#define SPI_SCLK_SPEED_DEV_BY_4					(1)
#define SPI_SCLK_SPEED_DEV_BY_8					(2)
#define SPI_SCLK_SPEED_DEV_BY_16				(3)
#define SPI_SCLK_SPEED_DEV_BY_32				(4)
#define SPI_SCLK_SPEED_DEV_BY_64				(5)
#define SPI_SCLK_SPEED_DEV_BY_128				(6)
#define SPI_SCLK_SPEED_DEV_BY_256				(7)

/*data frame format options*/
#define SPI_8_BITS_DFF            	(0)
#define SPI_16_BITS_DFF             (1)

/*SPI_DFFOrder*/
#define SPI_DFF_ORDER_MSB_FIRST     (0)
#define SPI_DFF_ORDER_LSB_FIRST     (1)
#define SPI_DFF_ORDER_DFLT          (SPI_DFF_ORDER_MSB_FIRST)
/*spi clk polarity options */
#define SPI_CPOL_RESET              (SPI_LOW)
#define SPI_CPOL_SET                (SPI_HIGH)

/*spi clk phase options */
#define SPI_CPHA_SET                (SPI_HIGH)
#define SPI_CPHA_RESET              (SPI_LOW)

/*SPI SSM (SW Slave Management) */
#define SPI_SSM_SW_CTRL_DI            0
#define SPI_SSM_SW_CTRL_EN            1
#define SPI_SSM_DI                    (SPI_SSM_SW_CTRL_DI)
#define SPI_SSM_EN                    (SPI_SSM_SW_CTRL_EN)



#define SPI_NSS_SW_CTRL       (1)  //SSM=1
#define SPI_NSS_HW_CTRL       (0)  //SSM=0

//SSM=0 SSOE=1
#define SPI_NSS_OUTPUT_EN(pSPIx)\
	do {\
	} while (0)

//SSM=0 SSOE=0
#define SPI_NSS_OUTPUT_DI(pSPIx) \
	do { \
	} while (0)





//CPOL Defs
#define SPI_SCK_LOW_LEVEL_IDLE_STATE
#define SPI_SCK_HIGH_LEVEL_IDLE_STATE



//CPHA defs
#define SPI_2ND_

#define SPI_ENABLE    (1)
#define SPI_DISABLE   (0)




void SPI_Init(SPI_Handle_t *handle);
void SPI_Reset(SPI_RegDef_t *pSPIx);
void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, u8 en);
//blocking API Data Transfer/Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, const u8 *pTxBuffer, u32 buflen);
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, u8 *pRxBuffer, u32 buflen);

U8 SPI_INTBasedSendData(SPI_Handle_t *handle, const u8 *pTxBuffer, u32 buflen);
U8 SPI_INTBasedReceiveData(SPI_Handle_t *handle, u8 *pRxBuffer, u32 buflen);



void SPI_IRQInterruptConfig(u8 IRQNumber, u8 EnOrDi);
void SPI_IRQPriorityConfig(u8 IRQNumber, u32 IRQPriority);
void SPI_IRQHandling(SPI_Handle_t *handle);




void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, u8 en);
void SPI_SSIConfig(SPI_RegDef_t *pSPIx, u8 en);







#endif /* INC_STM32F407XX_SPI_DRIVER_H_ */
