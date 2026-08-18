/*******************************************************************************
 *   @file   maxim_capi_init.c
 *   @brief  Platform initialization for MAX32657 CAPI
 *   @author Ramon Miguel Imbao (ramonmiguel.imbao@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#include "mxc_sys.h"
#include "mxc_delay.h"

int Board_Init(void)
{
	SysTick_Config(SystemCoreClock / 1000);

	return MXC_Delay(1);
}
