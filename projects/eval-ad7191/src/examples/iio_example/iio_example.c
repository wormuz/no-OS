/***************************************************************************//**
 *   @file   iio_example.c
 *   @brief  IIO example source file for eval-ad7191 project.
 *   @author Alisa-Dariana Roman (alisa.roman@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
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
 * 3. Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived from this
 *    software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED WARRANTIES OF
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO
 * EVENT SHALL ANALOG DEVICES, INC. BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA,
 * OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF
 * LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
 * NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
 * EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

#include "common_data.h"
#include "iio_ad7191.h"
#include "iio_app.h"
#include "no_os_print_log.h"
#include "no_os_util.h"

/**
 * Number of scans the buffer can hold. Only one channel can be active at a
 * time, and each sample occupies four bytes of storage.
 */
#define DATA_BUFFER_SIZE	400

static uint8_t iio_data_buffer[DATA_BUFFER_SIZE * sizeof(uint32_t)];

/**
 * @brief IIO example main execution.
 *
 * On the linux platform iio_app serves IIOD over TCP rather than the UART, so
 * the application is reachable at ip:<host>:30431.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously function iio_app_run and will not return.
 */
int example_main(void)
{
	struct ad7191_iio_dev *ad7191_iio_desc;
	struct ad7191_iio_dev_init_param ad7191_iio_ip = {
		.ad7191_dev_init = &ad7191_ip,
	};
	struct iio_data_buffer data_buff = {
		.buff = (void *)iio_data_buffer,
		.size = sizeof(iio_data_buffer),
	};
	struct iio_app_desc *app;
	struct iio_app_init_param app_init_param = { 0 };
	int ret;

	ret = ad7191_iio_init(&ad7191_iio_desc, &ad7191_iio_ip);
	if (ret)
		return ret;

	struct iio_app_device iio_devices[] = {
		{
			.name = "ad7191",
			.dev = ad7191_iio_desc,
			.dev_descriptor = ad7191_iio_desc->iio_dev,
			.read_buff = &data_buff,
		}
	};

	app_init_param.devices = iio_devices;
	app_init_param.nb_devices = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = ad7191_uart_ip;

	ret = iio_app_init(&app, app_init_param);
	if (ret)
		goto remove_iio_ad7191;

	ret = iio_app_run(app);

	iio_app_remove(app);

remove_iio_ad7191:
	ad7191_iio_remove(ad7191_iio_desc);

	return ret;
}
