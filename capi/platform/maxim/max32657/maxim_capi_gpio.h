/***************************************************************************//**
 *   @file   maxim_capi_gpio.h
 *   @brief  Header file for GPIO functions with CAPI.
 *   @author Ramon Miguel Imbao (ramonmiguel.imbao@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#ifndef MAXIM_CAPI_GPIO_H_
#define MAXIM_CAPI_GPIO_H_

#include "gpio.h"
#include "capi_gpio.h"

enum max_capi_gpio_vssel {
	MAX_CAPI_GPIO_VSSEL_VDDIO = MXC_GPIO_VSSEL_VDDIO,
	MAX_CAPI_GPIO_VSSEL_VDDIOH = MXC_GPIO_VSSEL_VDDIOH,
};

enum max_capi_gpio_func {
	MAX_CAPI_GPIO_FUNC_IN = MXC_GPIO_FUNC_IN,
	MAX_CAPI_GPIO_FUNC_OUT = MXC_GPIO_FUNC_OUT,
	MAX_CAPI_GPIO_FUNC_ALT_1 = MXC_GPIO_FUNC_ALT1,
	MAX_CAPI_GPIO_FUNC_ALT_2 = MXC_GPIO_FUNC_ALT2,
	MAX_CAPI_GPIO_FUNC_ALT_3 = MXC_GPIO_FUNC_ALT3,
	MAX_CAPI_GPIO_FUNC_ALT_4 = MXC_GPIO_FUNC_ALT4,
};

enum max_capi_gpio_pad {
	MAX_CAPI_GPIO_PAD_NONE = MXC_GPIO_PAD_NONE,
	MAX_CAPI_GPIO_PAD_PULL_UP = MXC_GPIO_PAD_WEAK_PULL_UP,
	MAX_CAPI_GPIO_PAD_PULL_DOWN = MXC_GPIO_PAD_WEAK_PULL_DOWN,
};

enum max_capi_gpio_drvstr {
	MAX_CAPI_GPIO_DRVSTR_0 = MXC_GPIO_DRVSTR_0,
	MAX_CAPI_GPIO_DRVSTR_1 = MXC_GPIO_DRVSTR_1,
	MAX_CAPI_GPIO_DRVSTR_2 = MXC_GPIO_DRVSTR_2,
	MAX_CAPI_GPIO_DRVSTR_3 = MXC_GPIO_DRVSTR_3,
};

struct max_capi_gpio_extra_config {
	enum max_capi_gpio_vssel vssel;
	enum max_capi_gpio_func func;
	enum max_capi_gpio_pad pad;
	enum max_capi_gpio_drvstr drvstr;
};

struct max_capi_gpio_port_priv {
	uint32_t id;
	mxc_gpio_regs_t *port;
	uint8_t num_pins;
	uint32_t pin_mask;
	uint32_t direction_mask;
	struct max_capi_gpio_extra_config extra;
};

extern const struct capi_gpio_ops max_capi_gpio_ops;

#endif /* MAXIM_CAPI_GPIO_H_ */
