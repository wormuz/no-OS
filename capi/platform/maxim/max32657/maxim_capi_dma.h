/*******************************************************************************
 *   @file   maxim_capi_dma.h
 *   @brief  Header file for DMA functions with CAPI
 *   @author Ramon Miguel Imbao (ramonmiguel.imbao@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#ifndef MAXIM_CAPI_DMA_H_
#define MAXIM_CAPI_DMA_H_

#include "dma.h"
#include "capi_dma.h"

#if !defined(CONFIG_TRUSTED_EXECUTION_SECURE) || (CONFIG_TRUSTED_EXECUTION_SECURE != 1)
#error "CONFIG_TRUSTED_EXECUTION_SECURE must be defined and set to 1."
#endif

enum max_capi_dma_request {
	MAX_CAPI_DMA_REQUEST_MEMTOMEM = MXC_DMA_REQUEST_MEMTOMEM,
	MAX_CAPI_DMA_REQUEST_SPI_RX = MXC_DMA_REQUEST_SPIRX,
	MAX_CAPI_DMA_REQUEST_SPI_TX = MXC_DMA_REQUEST_SPITX,
	MAX_CAPI_DMA_REQUEST_UART_RX = MXC_DMA_REQUEST_UARTRX,
	MAX_CAPI_DMA_REQUEST_UART_TX = MXC_DMA_REQUEST_UARTTX,
	MAX_CAPI_DMA_REQUEST_I3C_RX_CONTROLLER = MXC_DMA_REQUEST_I3CRX_CONT,
	MAX_CAPI_DMA_REQUEST_I3C_TX_CONTROLLER = MXC_DMA_REQUEST_I3CTX_CONT,
	MAX_CAPI_DMA_REQUEST_I3C_RX_TARGET = MXC_DMA_REQUEST_I3CRX_TARG,
	MAX_CAPI_DMA_REQUEST_I3C_TX_TARGET = MXC_DMA_REQUEST_I3CTX_TARG,
	MAX_CAPI_DMA_REQUEST_AES_RX = MXC_DMA_REQUEST_AESRX,
	MAX_CAPI_DMA_REQUEST_AES_TX = MXC_DMA_REQUEST_AESTX,
	MAX_CAPI_DMA_REQUEST_CRC_TX = MXC_DMA_REQUEST_CRCTX,
};

struct max_capi_dma_priv {
	/** Identifier */
	uint64_t id;
	/** Array of pointers to channels */
	struct capi_dma_chan **channels;
	/** Number of channels storage */
	uint8_t num_channels;
};

struct max_capi_dma_ch_priv {
	/* MXC_DMA_AcquireChannel returns a channel ID and is stored here */
	uint32_t hw_channel_id;
	/* DMA completion flag */
	volatile bool completed;
};

struct max_capi_dma_xfer_extra {
	/** DMA request type */
	enum max_capi_dma_request reqsel;
};

extern const struct capi_dma_ops max_capi_dma_ops;

#endif /* MAXIM_CAPI_DMA_H_ */
