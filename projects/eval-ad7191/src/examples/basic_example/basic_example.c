/***************************************************************************//**
 *   @file   basic_example.c
 *   @brief  Basic example source file for eval-ad7191 project.
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

#include "ad7191.h"
#include "common_data.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"

/**
 * @brief Read and print one conversion from the currently selected input.
 * @param dev - Device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
static int basic_example_read_voltage(struct ad7191_dev *dev)
{
	int64_t uvolts;
	uint32_t code;
	int ret;

	ret = ad7191_read_sample(dev, &code);
	if (ret)
		return ret;

	ret = ad7191_code_to_uvolts(dev, code, &uvolts);
	if (ret)
		return ret;

	pr_info("AIN%d/AIN%d: code 0x%06lX, %ld uV (gain %lu)\n",
		dev->chan == AD7191_CHAN_AIN1_AIN2 ? 1 : 3,
		dev->chan == AD7191_CHAN_AIN1_AIN2 ? 2 : 4,
		(unsigned long)code, (long)uvolts,
		(unsigned long)ad7191_gain_to_value(dev->gain));

	return 0;
}

/**
 * @brief Read and print one conversion from the internal temperature sensor.
 *
 * Selecting and deselecting the sensor are both configuration changes, so this
 * costs two settling times on top of the conversion itself.
 *
 * @param dev - Device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
static int basic_example_read_temp(struct ad7191_dev *dev)
{
	int32_t millidegrees;
	uint32_t code;
	int ret;

	ret = ad7191_set_temp_en(dev, true);
	if (ret)
		return ret;

	ret = ad7191_read_sample(dev, &code);
	if (ret)
		goto disable_temp;

	ret = ad7191_code_to_millidegrees(code, &millidegrees);
	if (ret)
		goto disable_temp;

	pr_info("temperature: code 0x%06lX, %ld mdegC\n",
		(unsigned long)code, (long)millidegrees);

disable_temp:
	return ad7191_set_temp_en(dev, false) ? : ret;
}

/**
 * @brief Basic example main execution.
 *
 * Reads both differential input pairs and the internal temperature sensor in a
 * loop, printing the raw code alongside the converted value.
 *
 * @return ret - Result of the example execution. If working correctly, will
 *               execute continuously the while(1) loop and will not return.
 */
int example_main(void)
{
	struct no_os_uart_desc *uart_desc;
	struct ad7191_dev *ad7191_desc;
	int ret;

	ret = no_os_uart_init(&uart_desc, &ad7191_uart_ip);
	if (ret)
		return ret;

	no_os_uart_stdio(uart_desc);

	ret = ad7191_init(&ad7191_desc, &ad7191_ip);
	if (ret)
		goto remove_uart;

	pr_info("AD7191 initialized, VREF %lu mV\n",
		(unsigned long)ad7191_desc->vref_mv);

	while (1) {
		ret = ad7191_set_channel(ad7191_desc, AD7191_CHAN_AIN1_AIN2);
		if (ret)
			goto remove_ad7191;

		ret = basic_example_read_voltage(ad7191_desc);
		if (ret)
			goto remove_ad7191;

		ret = ad7191_set_channel(ad7191_desc, AD7191_CHAN_AIN3_AIN4);
		if (ret)
			goto remove_ad7191;

		ret = basic_example_read_voltage(ad7191_desc);
		if (ret)
			goto remove_ad7191;

		ret = basic_example_read_temp(ad7191_desc);
		if (ret)
			goto remove_ad7191;

		no_os_mdelay(1000);
	}

remove_ad7191:
	pr_err("Error: %d\n", ret);
	ad7191_remove(ad7191_desc);
remove_uart:
	no_os_uart_remove(uart_desc);

	return ret;
}
