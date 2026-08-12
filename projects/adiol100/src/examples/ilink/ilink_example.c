/***************************************************************************//**
 *   @file   ilink_example.c
 *   @brief  ADIOL100 i-link IO-Link stack example.
 *   @author Liviu Stan (liviu.stan@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#include <stdio.h>
#include "ilink_example.h"
#include "common_data.h"
#include "adiol100_ilink_pl.h"
#include "iolink_app.h"
#include "osal.h"
#include "no_os_print_log.h"
#include "no_os_gpio.h"
#include "no_os_irq.h"
#include "FreeRTOS.h"
#include "task.h"

/* i-link stack thread configuration */
#define APP_MASTER_THREAD_STACK_SIZE  (4 * 1024)
#define APP_MASTER_THREAD_PRIO        4
#define APP_DL_THREAD_STACK_SIZE      (2 * 1024)
#define APP_DL_THREAD_PRIO            5
#define APP_HANDLER_THREAD_STACK_SIZE (4 * 1024)
#define APP_HANDLER_THREAD_PRIO       3

/* IRQA GPIO pin — directly connected to ADIOL100 interrupt output. */
#define IRQA_PORT  0
#define IRQA_PIN   12

/* IO-Link master context */
static struct iolink_app_master app_master;

/* Port modes: channel A = IO-Link (SDCI), channel B = inactive. */
static iolink_pl_mode_t mode_ch[] = {
	iolink_mode_SDCI,
	iolink_mode_INACTIVE,
};

/**
 * @brief FreeRTOS task that initializes and runs the IO-Link app.
 * @param ctx - Pointer to iolink_m_cfg_t master configuration.
 */
static void app_task(void *ctx)
{
	iolink_m_cfg_t *cfg = ctx;

	if (iolink_app_init(&app_master, cfg) != 0) {
		pr_info("iolink_app_init failed\n");
		return;
	}

	iolink_app_run(&app_master);
}

/**
 * @brief Run the i-link IO-Link stack example.
 *
 * Initializes the ADIOL100 port layer, configures port A for IO-Link (SDCI
 * auto mode), and starts a FreeRTOS task that runs the IO-Link application
 * event loop. Process data from connected devices is printed to the console.
 *
 * @return 0 on success, negative error code otherwise.
 */
int ilink_example_main(void)
{
	iolink_hw_drv_t *hw;

	pr_info("ADIOL100 i-link example\n");

	/* IRQ GPIO for ADIOL100 channel A interrupt (falling edge, active-low) */
	struct no_os_gpio_init_param irq_gpio_a_ip = {
		.port = IRQA_PORT,
		.number = IRQA_PIN,
		.platform_ops = GPIO_OPS,
		.extra = GPIO_EXTRA,
	};

	struct no_os_irq_init_param irq_ip = {
		.irq_ctrl_id = 0,
		.platform_ops = GPIO_IRQ_OPS,
		.extra = NULL,
	};

	/* ADIOL100 port layer config: SPI device + IRQ pin for channel A */
	iolink_adiol100_cfg_t adiol100_cfg = {
		.adiol100_ip = &adiol100_ip,
		.irq_gpio_a = &irq_gpio_a_ip,
		.irq_gpio_b = NULL,
		.irq_ip = &irq_ip,
	};

	/* Initialize port layer — sets up SPI, ADIOL100 registers and IRQ handler */
	hw = iolink_adiol100_init(&adiol100_cfg);
	if (hw == NULL) {
		pr_info("iolink_adiol100_init failed\n");
		return -1;
	}

	pr_info("ADIOL100 port layer initialized\n");

	/* i-link port config: map each stack port to an ADIOL100 channel */
	iolink_port_cfg_t port_cfgs[] = {
		{
			.name = "/adiol100/0",
			.mode = &mode_ch[0],
			.drv = hw,
			.arg = (void *)(uintptr_t)ADIOL100_CH_A,
		},
		{
			.name = "/adiol100/1",
			.mode = &mode_ch[1],
			.drv = hw,
			.arg = (void *)(uintptr_t)ADIOL100_CH_B,
		},
	};

	/* Master stack config — callbacks are set by iolink_app_init() */
	static iolink_m_cfg_t m_cfg;

	m_cfg.port_cnt                 = NELEMENTS(port_cfgs);
	m_cfg.port_cfgs                = port_cfgs;
	m_cfg.master_thread_prio       = APP_MASTER_THREAD_PRIO;
	m_cfg.master_thread_stack_size = APP_MASTER_THREAD_STACK_SIZE;
	m_cfg.dl_thread_prio           = APP_DL_THREAD_PRIO;
	m_cfg.dl_thread_stack_size     = APP_DL_THREAD_STACK_SIZE;

	/* Start the app task — init and event loop run inside FreeRTOS */
	xTaskCreate(
		app_task,
		"iolink_app",
		APP_HANDLER_THREAD_STACK_SIZE / sizeof(StackType_t),
		&m_cfg,
		APP_HANDLER_THREAD_PRIO,
		NULL);

	pr_info("Starting FreeRTOS scheduler\n");
	vTaskStartScheduler();

	return 0;
}
