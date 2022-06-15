/* Copyright, DSI FCEIA UNR - Sistemas Digitales 2
 *    DSI: http://www.dsi.fceia.unr.edu.ar/
 * Copyright, Gustavo Muro
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 *
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 */

/*==================[inclusions]=============================================*/

// Standard C Included Files
#include <string.h>
#include <stdio.h>
#include <stdbool.h>

// Project Included Files
#include "SD2_board.h"
#include "fsl_lpsci.h"
#include "fsl_port.h"
#include "board.h"
#include "MKL46Z4.h"
#include "pin_mux.h"
#include "uart_ringBuffer.h"
#include "ringBuffer.h"
#include "fsl_dmamux.h"
#include "fsl_lpsci_dma.h"
#include "fsl_uart.h"
#include "fsl_uart_dma.h"

/*==================[macros and definitions]=================================*/

//#define UART0_ON

#define LPSCI_TX_DMA_CHANNEL 0U
#define UART1_TX_DMA_CHANNEL 0U
#define TX_BUFFER_DMA_SIZE  32
#define RX_BUFFER_SIZE 32U

#define BOARD_DE_GPIO GPIOA
#define BOARD_RE_GPIO GPIOA
#define BOARD_DE_PORT PORTA
#define BOARD_RE_PORT PORTA
#define BOARD_DE_PIN  17U
#define BOARD_RE_PIN  16U

/*==================[internal data declaration]==============================*/
static void* pRingBufferRx;

static uint8_t txBuffer_dma[TX_BUFFER_DMA_SIZE];
volatile bool txOnGoing = false;

static lpsci_dma_handle_t lpsciDmaHandle;
static dma_handle_t lpsciTxDmaHandle;
static uart_dma_handle_t uartDmaHandle;
static dma_handle_t uart1TxDmaHandle;

/*==================[internal functions declaration]=========================*/

/*==================[external data definition]===============================*/

/*==================[internal functions definition]==========================*/

/*==================[external functions definition]==========================*/

#ifdef UART0_ON

static void LPSCI_UserCallback(UART0_Type *base, lpsci_dma_handle_t *handle, status_t status, void *userData)
{
    if (kStatus_LPSCI_TxIdle == status)
    {
        txOnGoing = false;
    }
}

void uart_ringBuffer_init(void)
{
	lpsci_config_t config;

    pRingBufferRx = ringBuffer_init(RX_BUFFER_SIZE);

	CLOCK_SetLpsci0Clock(0x1U);

	/* PORTA1 (pin 35) is configured as UART0_RX */
	PORT_SetPinMux(PORTA, 1U, kPORT_MuxAlt2);

	/* PORTA2 (pin 36) is configured as UART0_TX */
	PORT_SetPinMux(PORTA, 2U, kPORT_MuxAlt2);

	/*
	 * config.parityMode = kLPSCI_ParityDisabled;
	 * config.stopBitCount = kLPSCI_OneStopBit;
	 * config.enableTx = false;
	 * config.enableRx = false;
	 */
	LPSCI_GetDefaultConfig(&config);
	config.baudRate_Bps = 115200;
	config.parityMode = kLPSCI_ParityDisabled;
	config.stopBitCount = kLPSCI_OneStopBit;
	config.enableTx = true;
	config.enableRx = true;

	LPSCI_Init(UART0, &config, CLOCK_GetFreq(kCLOCK_CoreSysClk));

	/* Habilita interrupciones */
	LPSCI_EnableInterrupts(UART0, kLPSCI_RxDataRegFullInterruptEnable);
	LPSCI_EnableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);
	EnableIRQ(UART0_IRQn);

	/*Configuracion DMA para Tx UART1 */
	DMAMUX_Init(DMAMUX0);

	//Vinculo DMA con UART1
	DMAMUX_SetSource(DMAMUX0, LPSCI_TX_DMA_CHANNEL, kDmaRequestMux0LPSCI0Tx);
	DMAMUX_EnableChannel(DMAMUX0, LPSCI_TX_DMA_CHANNEL);

	//Init DMA
	DMA_Init(DMA0);
	DMA_CreateHandle(&lpsciTxDmaHandle, DMA0, LPSCI_TX_DMA_CHANNEL);

	//Creacion del Handler de UART1 DMA
	LPSCI_TransferCreateHandleDMA(
			UART0,
			&lpsciDmaHandle,
			LPSCI_UserCallback,
			NULL,
			&lpsciTxDmaHandle,
			NULL);

}

/** \brief recibe datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde guardar los datos
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes recibidos
 **/
int32_t uart_ringBuffer_recDatos(uint8_t *pBuf, int32_t size)
{
    int32_t ret = 0;
	LPSCI_ClearStatusFlags(UART0, kLPSCI_RxOverrunFlag);

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART0_IRQn);

    while (!ringBuffer_isEmpty(pRingBufferRx) && ret < size)
    {
        ringBuffer_getData(pRingBufferRx, &pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART0_IRQn);

    return ret;
}

/** \brief envía datos por puerto serie accediendo al RB
 **
 ** \param[inout] pBuf buffer a donde estan los datos a enviar
 ** \param[in] size tamaño del buffer
 ** \return cantidad de bytes enviados
 **/
int32_t uart_ringBuffer_envDatos(uint8_t *pBuf, int32_t size)
{
	lpsci_transfer_t xfer;

	if (txOnGoing)
	{
		size = 0;
	}
	else
	{
		/* limita size */
		if (size > TX_BUFFER_DMA_SIZE)
			size = TX_BUFFER_DMA_SIZE;

		// Hace copia del buffer a transmitir en otro arreglo
		memcpy(txBuffer_dma, pBuf, size);

		xfer.data = txBuffer_dma;
		xfer.dataSize = size;

		txOnGoing = true;
		LPSCI_TransferSendDMA(UART0, &lpsciDmaHandle, &xfer);

		LPSCI_EnableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);

	}

	return size;
}


void UART0_IRQHandler(void)
{
	uint8_t data;

    if ( ((kLPSCI_RxDataRegFullFlag)            & LPSCI_GetStatusFlags(UART0)) &&
         ((kLPSCI_RxDataRegFullInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0)))
	{
		LPSCI_ClearStatusFlags(UART0, kLPSCI_RxDataRegFullFlag);
		LPSCI_ClearStatusFlags(UART0, kLPSCI_RxOverrunFlag);

        /* obtiene dato recibido por puerto serie */
	    data = LPSCI_ReadByte(UART0);

		/* pone dato en ring buffer */
		ringBuffer_putData(pRingBufferRx, data);
	}

    if ( (kLPSCI_TransmissionCompleteFlag)            & LPSCI_GetStatusFlags(UART0) &&
		 (kLPSCI_TransmissionCompleteInterruptEnable) & LPSCI_GetEnabledInterrupts(UART0) )
	{
		LPSCI_DisableInterrupts(UART0, kLPSCI_TransmissionCompleteInterruptEnable);
		LPSCI_ClearStatusFlags(UART0, kLPSCI_TransmissionCompleteFlag);
	}

}

#else

static void UART1_UserCallBack(UART_Type *base, uart_dma_handle_t *handle, status_t status, void *userData)
{
	if (kStatus_UART_TxIdle == status)
	{
		txOnGoing = false;
	}
}

void uart_ringBuffer_init(void)
{
	uart_config_t config_uart1;

	gpio_pin_config_t gpio_RS_485_config =
	{
		.outputLogic = 1,
		.pinDirection = kGPIO_DigitalOutput,
	};

	const port_pin_config_t port_RS_485_config = {
		/* Internal pull-up/down resistor is disabled */
		.pullSelect = kPORT_PullDisable,
		/* Slow slew rate is configured */
		.slewRate = kPORT_SlowSlewRate,
		/* Passive filter is disabled */
		.passiveFilterEnable = kPORT_PassiveFilterDisable,
		/* Low drive strength is configured */
		.driveStrength = kPORT_LowDriveStrength,
		/* Pin is configured as PTC3 */
		.mux = kPORT_MuxAsGpio,
	};

	//RE init
	PORT_SetPinConfig(BOARD_DE_PORT, BOARD_DE_PIN, &port_RS_485_config);
	GPIO_PinInit(BOARD_DE_GPIO, BOARD_DE_PIN, &gpio_RS_485_config);

	//DE init
	PORT_SetPinConfig(BOARD_RE_PORT, BOARD_RE_PIN, &port_RS_485_config);
	GPIO_PinInit(BOARD_RE_GPIO, BOARD_RE_PIN, &gpio_RS_485_config);

	//Habilito Recepcion
	GPIO_PortClear(BOARD_DE_GPIO, 1<<BOARD_DE_PIN);
	GPIO_PortClear(BOARD_RE_GPIO, 1<<BOARD_RE_PIN);


	//Inicializo Ring Buffer Recepcion
    pRingBufferRx = ringBuffer_init(RX_BUFFER_SIZE);

	CLOCK_EnableClock(kCLOCK_Uart1);

	/* PORTE1 (pin 35) is configured as UART1_RX */
	PORT_SetPinMux(PORTE, 1U, kPORT_MuxAlt3);

	/* PORTE2 (pin 36) is configured as UART1_TX */
	PORT_SetPinMux(PORTE, 0U, kPORT_MuxAlt3);

	habilitar_Recepcion();

	//Configuracion del UART1
	UART_GetDefaultConfig(&config_uart1);
	config_uart1.baudRate_Bps = 115200;
	config_uart1.parityMode = kUART_ParityDisabled;
	config_uart1.stopBitCount = kUART_OneStopBit;
	config_uart1.enableTx = true;
	config_uart1.enableRx = true;

	UART_Init(UART1, &config_uart1, CLOCK_GetFreq(kCLOCK_BusClk));

	//Habilitacion manual de Rx y Tx
	UART1->C2 |= UART_C2_TE_MASK;
	UART1->C2 |= UART_C2_RE_MASK;


	//UART_Init(UART1, &config, CLOCK_GetFreq(kCLOCK_BusClk));

	UART_EnableInterrupts(UART1, kUART_RxDataRegFullInterruptEnable);
	UART_EnableInterrupts(UART1, kUART_TransmissionCompleteInterruptEnable);
	EnableIRQ(UART1_IRQn);

	/*Configuracion DMA para Tx UART1 */
	DMAMUX_Init(DMAMUX0);

	//Vinculo DMA con UART1
	DMAMUX_SetSource(DMAMUX0, UART1_TX_DMA_CHANNEL, kDmaRequestMux0UART1Tx);
	DMAMUX_EnableChannel(DMAMUX0, UART1_TX_DMA_CHANNEL);

	//Init DMA
	DMA_Init(DMA0);
	DMA_CreateHandle(&uart1TxDmaHandle, DMA0, UART1_TX_DMA_CHANNEL);

	//Creacion del Handler de UART1 DMA
	UART_TransferCreateHandleDMA(
			UART1,
			&uartDmaHandle,
			UART1_UserCallBack,
			NULL,
			&uart1TxDmaHandle,
			NULL);

}

int32_t uart_ringBuffer_recDatos(uint8_t *pBuf, int32_t size)
{
    int32_t ret = 0;

    /* entra sección de código crítico */
    NVIC_DisableIRQ(UART1_IRQn);

    while (!ringBuffer_isEmpty(pRingBufferRx) && ret < size)
    {
        ringBuffer_getData(pRingBufferRx, &pBuf[ret]);
        ret++;
    }

    /* sale de sección de código crítico */
    NVIC_EnableIRQ(UART1_IRQn);

    return ret;
}

int32_t uart_ringBuffer_envDatos(uint8_t *pBuf, int32_t size)
{
	uart_transfer_t xfer;
	habilitar_Transmicion();

	if (txOnGoing)
	{
		size = 0;
	}
	else
	{
		/* limita size */
		if (size > TX_BUFFER_DMA_SIZE)
			size = TX_BUFFER_DMA_SIZE;

		// Hace copia del buffer a transmitir en otro arreglo
		memcpy(txBuffer_dma, pBuf, size);

		xfer.data = txBuffer_dma;
		xfer.dataSize = size;

		txOnGoing = true;
		UART_TransferSendDMA(UART1, &uartDmaHandle, &xfer);

		UART_EnableInterrupts(UART1, kUART_TransmissionCompleteInterruptEnable);

	}

	return size;
}

void habilitar_Recepcion()
{
	//DE y RE en 0 -> Recepcion Habilitada
	GPIO_PortClear(BOARD_DE_GPIO, 1<<BOARD_DE_PIN);
	GPIO_PortClear(BOARD_RE_GPIO, 1<<BOARD_RE_PIN);
}

void habilitar_Transmicion()
{
	//DE y RE en 1 -> Transmicion Habilitada
	GPIO_PortSet(BOARD_DE_GPIO, 1<<BOARD_DE_PIN);
	GPIO_PortSet(BOARD_RE_GPIO, 1<<BOARD_RE_PIN);
}

void UART1_IRQHandler(void)
{
	uint8_t data;

	if ( ((kUART_RxDataRegFullFlag)            & UART_GetStatusFlags(UART1)) &&
		 ((kUART_RxDataRegFullInterruptEnable) & UART_GetEnabledInterrupts(UART1)) )
	{
		UART_ClearStatusFlags(UART1, kUART_RxDataRegFullFlag);
		UART_ClearStatusFlags(UART1, kUART_RxOverrunFlag);

		/* obtiene dato recibido por puerto serie */
		data = UART_ReadByte(UART1);

		/* pone dato en ring buffer */
		ringBuffer_putData(pRingBufferRx, data);
	}

	if ( (kUART_TransmissionCompleteFlag)            & UART_GetStatusFlags(UART1) &&
		 (kUART_TransmissionCompleteInterruptEnable) & UART_GetEnabledInterrupts(UART1) )
	{
		UART_DisableInterrupts(UART1, kUART_TransmissionCompleteInterruptEnable);
		UART_ClearStatusFlags(UART1, kUART_TransmissionCompleteFlag);

		habilitar_Recepcion();

	}
}
#endif

/*==================[end of file]============================================*/

