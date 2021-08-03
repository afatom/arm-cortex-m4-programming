/*
 * stm32f407xx_spi_driver.c
 *
 *  Created on: 5 Jul 2021
 *      Author: amach
 */

#include "hal_stm32f407xx.h"
#include "stm32f407xx_spi_driver.h"
#include "stm32f407xx_rcc_driver.h"

void SPI_SSIConfig(SPI_RegDef_t *pSPIx, u8 en)
{

}

//call this function after spi init ALWAYS (not before)
void SPI_PeripheralControl(SPI_RegDef_t *pSPIx, u8 en)
{
	if (en == SPI_ENABLE) {
		//set the SPE bit in the ctrl reg
		pSPIx->SPI_CR1 |= (1 << SPI_CR1_SPE_BIT);
	} else {
		pSPIx->SPI_CR1 &= ~(1 << SPI_CR1_SPE_BIT);
	}
}

void SPI_Init(SPI_Handle_t *handle)
{
	u32 tmpreg = 0;
	SPI_PeriClkCtrl(handle->pSPIx, SPI_CLK_EN);

	// SPI device mode configuration (MSTR Bit)
	tmpreg |= handle->SPIConfig.SPI_DeviceMode << SPI_CR1_MSTR_BIT;
	//SPI Bus configuration
	switch (handle->SPIConfig.SPI_BusConfig) {
	case (SPI_FULL_DUPLEX_BUS_CFG):
		//BIDIMODE should be cleared
	    tmpreg &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
		break;
	case (SPI_HALF_DUPLEX_BUS_CFG):
		//BIDIMODE should be set
		tmpreg |= (1 << SPI_CR1_BIDI_MODE_BIT);
		break;
	case (SPI_SIMPLEX_RXONLY_BUS_CFG):
		//BIDIMODE should be cleared
		tmpreg &= ~(1 << SPI_CR1_BIDI_MODE_BIT);
	    tmpreg |= (1 << SPI_CR1_RXONLY_BIT);
		break;
	default:
		//fatal
		break;
	}
	//sclk speed or baud rate config
	tmpreg |= (handle->SPIConfig.SPI_SclkSpeed << SPI_CR1_BR_BIT);
	//data frame format buffer length
	tmpreg |= handle->SPIConfig.SPI_DFFLen << SPI_CR1_DFF_BIT;
	// data frame order config
	tmpreg |= handle->SPIConfig.SPI_DFFOrder << SPI_CR1_LSB_FIRST_BIT;
	tmpreg |= handle->SPIConfig.SPI_CPOL << SPI_CR1_CPOL_BIT;
	tmpreg |= handle->SPIConfig.SPI_CPHA << SPI_CR1_CPHA_BIT;
	tmpreg |= handle->SPIConfig.SPI_SSM <<SPI_CR1_SSM_BIT;

	handle->pSPIx->SPI_CR1 = tmpreg;




}
void SPI_Reset(SPI_RegDef_t *pSPIx);

void SPI_PeriClkCtrl(SPI_RegDef_t *pSPIx, u8 en)
{
	if (en == SPI_CLK_EN) {
		switch ((u32)pSPIx) {
		case (u32)SPI1:
			SPI_1_PCLK_EN();
			break;
		case (u32)SPI2:
			SPI_2_PCLK_EN();
			break;
		case (u32)SPI3:
			SPI_3_PCLK_EN();
			break;
		case (u32)SPI4:
			SPI_4_PCLK_EN();
			break;
		case (u32)SPI5:
			SPI_5_PCLK_EN();
			break;
		case (u32)SPI6:
			SPI_6_PCLK_EN();
			break;
		default:
			/* should not reach here fatal*/
			break;
		}
	} else {
		switch ((u32)pSPIx) {
		case (u32)SPI1:
			SPI_1_PCLK_DI();
			break;
		case (u32)SPI2:
			SPI_2_PCLK_DI();
			break;
		case (u32)SPI3:
			SPI_3_PCLK_DI();
			break;
		case (u32)SPI4:
			SPI_4_PCLK_DI();
			break;
		case (u32)SPI5:
			SPI_5_PCLK_DI();
			break;
		case (u32)SPI6:
			SPI_6_PCLK_DI();
			break;
		default:
			/* should not reach here fatal*/
			break;
		}
	}
}
//blocking API Data Transfer/Receive
void SPI_SendData(SPI_RegDef_t *pSPIx, const u8 *pTxBuffer, u32 buflen)
{
	for (;;) {
		/* if Transaction finished */
		if (!buflen)
			break;
		/* poll and wait for the transmit spi buffer to be empty */
		while ( ((pSPIx->SPI_SR >> 1) & 1) == TX_BUFFER_NOT_EMPTY );
		if ((pSPIx->SPI_CR1 >> SPI_CR1_DFF_BIT) & 1) { //16 bit frame
			//copy 16 bits to TX buffer
			pSPIx->SPI_DR = *((u16*)pTxBuffer);
			pTxBuffer += 2;
			buflen -=2 ;
		} else {
			//copy 8 bits to TX buffer
			pSPIx->SPI_DR = *pTxBuffer;
			pTxBuffer++;
			buflen--;
		}
	}
}

//blocking API Data Transfer/Receive
void SPI_ReceiveData(SPI_RegDef_t *pSPIx, u8 *pRxBuffer, u32 buflen)
{
	for (;;) {
		/* if Transaction finished */
		if (!buflen)
			break;
		/* poll and wait for the receive spi buffer to be full */
		while ( (pSPIx->SPI_SR & 1) == RX_BUFFER_EMPTY );
		if ((pSPIx->SPI_CR1 >> SPI_CR1_DFF_BIT) & 1) { //16 bit frame
			//copy 16 bits from RX buffer
			*((u16*)pRxBuffer) = (u16)pSPIx->SPI_DR;
			pRxBuffer += 2;
			buflen -=2 ;
		} else {
			//copy 8 bits from RX buffer
			*pRxBuffer = (u8)pSPIx->SPI_DR;
			pRxBuffer++;
			buflen--;
		}
	}
}

void SPI_IRQConfig(u8 IRQNumber, u8 EnOrDi)
{
	//all config in this function is in the processor side
	//u8 ISERIndex = IRQNumber/32;
	//u8 ISERxBitPos = IRQNumber%32;

	if (EnOrDi == HAL_ENABLE) {
		if ( IRQNumber <= 31 ) {
			*pNVIC_ISER0_REG |= (1 << IRQNumber);
		} else if ( (IRQNumber > 31) && (IRQNumber < 64) ) {
			*pNVIC_ISER1_REG |= (1 << (IRQNumber%32));
		} else if ( (IRQNumber >= 64) && (IRQNumber < 96) ) {
			*pNVIC_ISER2_REG |= (1 << (IRQNumber%64));
		}

		//NVIC->ISER_REGTABLE[ISERIndex] |= (1 << ISERxBitPos);		/* Enable interrupt num IRQNumber in the NVIC */
		//*pNVIC_ISER0_REG |= (1 << ISERxBitPos);
	} else {
		//NVIC->ICER_REGTABLE[ICERIndex] |= (1 << ISERxBitPos);		/* Disable interrupt num IRQNumber in the NVIC */
		//*pNVIC_ICER0_REG |= (1 << ISERxBitPos);
		if ( IRQNumber <= 31 ) {
			*pNVIC_ICER0_REG |= ( 1 << IRQNumber );
		} else if ( (IRQNumber > 31) && (IRQNumber < 64) ) {
			*pNVIC_ICER1_REG |= ( 1 <<  (IRQNumber % 32) );
		} else if ( (IRQNumber >= 64) && (IRQNumber < 96) ){
			*pNVIC_ICER2_REG |= ( 1 << (IRQNumber % 64) );
		}
	}
}

void SPI_IRQPriorityConfig(u8 IRQNumber, u32 IRQPriority)
{
	// find the IPR register in the table
	u8 IPRIndex = IRQNumber/4;
	u8 IPRSection = IRQNumber%4;
	u8 shiftamount = 8*IPRSection + (8 - 4);
	*(pNVIC_IPR0_REG + IPRIndex) |= ( IRQPriority << shiftamount);

}


U8 SPI_INTBasedSendData(SPI_Handle_t *handle, const u8 *pTxBuffer, u32 buflen)
{
	/*
	 * (1) Save The TX SPI buffer len and address in global variable so we can access it from the ISR
	 * (2) Mark the SPI state as BUSY in transmission so that no other code can take over same SPI
	 *     Peripheral until transmission is over
	 * (3) Enable the TXEIE control bit to get interrupt whenever TXE flag is SET in Status Register
	 * (4) data transmission will be handled by the ISR  code
	 * */
	if (handle->TxState != SPI_BUSY_IN_TX) {
		handle->TxLen = buflen;
		handle->pTxBuffer = pTxBuffer;
		handle->TxState = SPI_BUSY_IN_TX;
		handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_TXEIE_BIT);
	}
	return handle->TxState;
}

U8 SPI_INTBasedReceiveData(SPI_Handle_t *handle, u8 *pRxBuffer, u32 buflen)
{
	/*
	 * (1) Save The RX SPI buffer len and address in global variable so we can access it from the ISR
	 * (2) Mark the SPI state as BUSY in RX so that no other code can take over same SPI
	 *     Peripheral until RX transaction is over
	 * (3) Enable the RXNEIE control bit to get interrupt whenever RXE flag is SET in Status Register
	 * (4) data RX will be handled by the ISR  code
	 * */
	if (handle->TxState != SPI_BUSY_IN_RX) {
		handle->RxLen = buflen;
		handle->pRxBuffer = pRxBuffer;
		handle->RxState = SPI_BUSY_IN_RX;
		handle->pSPIx->SPI_CR2 |= (1 << SPI_CR2_RXNEIE_BIT);
	}
	return handle->RxState;
}

//SPI ISR
void SPI_IRQHandling(SPI_Handle_t *handle)
{
	//we can reach here in 3 different scenarios
	//1 due to TX buffer interrupt
	//2 due to RX buffer interrupt
	//3 due to an error on the SPI engine

	// if Tx buffer empty interrupt enable && TXE bit is set then we reach here tue to empty TX buffer IRQ
	U8 statusReg = 0, ctrl2Register = 0;

	statusReg = handle->pSPIx->SR;
	ctrl2Register = handle->pSPIx->SPI_CR2;

	if ( (statusReg & (1 << SPI_SR_TXE_BIT)) && (ctrl2Register& (1 << SPI_CR2_TXEIE_BIT)) )
	{
		// handle TX buffer empty scenario
		SPI_TXE_IRQHandler();
	}
	else if ( (statusReg & (1 << SPI_SR_RXE_BIT)) && (ctrl2Register& (1 << SPI_CR2_RXNEIE_BIT)) )
	{
		//handle RX buffer empty scenario
		SPI_RXE_IRQHandler();
	}
	else if ( (ctrl2Register& (1 << SPI_CR2_ERRIE_BIT))
			&&
			( (statusReg & (1 << SPI_SR_OVR_BIT))  ||
			  (statusReg & (1 << SPI_SR_MODF_BIT)) ||
			  (statusReg & (1 << SPI_SR_FRE_BIT))  ||
			  (statusReg & (1 << SPI_SR_CRCERR_BIT))
			)
			)
	{
		//handle error SPI scenario
		SPI_ERR_IRQHandler();
	}
}
