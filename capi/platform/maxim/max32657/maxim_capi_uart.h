/*******************************************************************************
 *   @file   maxim_capi_uart.h
 *   @brief  Header file for UART functions with CAPI
 *   @author Ramon Miguel Imbao (ramonmiguel.imbao@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#ifndef MAXIM_CAPI_UART_H_
#define MAXIM_CAPI_UART_H_

#include "uart.h"
#include "capi_dma.h"
#include "capi_uart.h"
#include "maxim_capi_dma.h"
#include "maxim_capi_irq.h"
#include "maxim_capi_gpio.h"

#define MAX_CAPI_UART_DEFAULT_BAUD 115200

struct max_capi_uart_extra {
	/** GPIO voltage selection */
	enum max_capi_gpio_vssel vssel;
	/** OPTIONAL - DMA config */
	struct capi_dma_config *dma_config;
};

struct max_capi_uart_priv {
	/** Identifier */
	uint32_t id;
	/** UART registers */
	mxc_uart_regs_t *uart;
	/** Clock source storage */
	mxc_uart_clock_t clk_src;
	/** Line config storage */
	struct capi_uart_line_config line_config;
	/** DMA handle */
	struct capi_dma_handle *dma_handle;
	/** Async callback */
	capi_uart_callback callback;
	/** Callback arg */
	void *callback_arg;
	/** UART request storage for async */
	mxc_uart_req_t async_req;
	/** DMA transfer storage for async */
	struct capi_dma_transfer dma_xfer;
	/** DMA transfer extra struct storage for async */
	struct max_capi_dma_xfer_extra dma_xfer_extra;
};

extern struct capi_uart_ops max_capi_uart_ops;

int max_capi_uart_stdio_enable(struct capi_uart_handle *handle);

#endif /* MAXIM_CAPI_UART_H_ */
