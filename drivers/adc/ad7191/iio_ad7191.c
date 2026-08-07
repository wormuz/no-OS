/***************************************************************************//**
 *   @file   iio_ad7191.c
 *   @brief  Implementation of AD7191 IIO Driver.
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
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. “AS IS” AND ANY EXPRESS OR
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

#include <errno.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "ad7191.h"
#include "iio_ad7191.h"
#include "no_os_alloc.h"
#include "no_os_error.h"
#include "no_os_util.h"

/** Number of fractional bits in the voltage scale (2^23). */
#define AD7191_BIT_SCALE	(AD7191_RESOLUTION - 1)

/**
 * Channel addresses. The temperature sensor is not a separate input pair but
 * an override of whichever pair CHAN selects, so it is given its own address
 * and switched in on demand.
 */
#define AD7191_ADDR_AIN1_AIN2	0
#define AD7191_ADDR_AIN3_AIN4	1
#define AD7191_ADDR_TEMP	2

/**
 * @brief Select the input a channel refers to, if not already selected.
 * @param dev - Device descriptor.
 * @param address - Channel address.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_iio_select_input(struct ad7191_dev *dev,
				   unsigned long address)
{
	bool want_temp = (address == AD7191_ADDR_TEMP);
	int ret;

	if (dev->temp_en != want_temp) {
		ret = ad7191_set_temp_en(dev, want_temp);
		if (ret)
			return ret;
	}

	if (want_temp)
		return 0;

	if (dev->chan != (enum ad7191_chan)address)
		return ad7191_set_channel(dev, (enum ad7191_chan)address);

	return 0;
}

/**
 * @brief Handles the read request for the raw attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_raw(void *dev, char *buf, uint32_t len,
			       const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	uint32_t code;
	int32_t val;
	int ret;

	if (!dev || !channel)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	ret = ad7191_iio_select_input(iio_ad7191->ad7191_dev, channel->address);
	if (ret)
		return ret;

	ret = ad7191_read_sample(iio_ad7191->ad7191_dev, &code);
	if (ret)
		return ret;

	val = (int32_t)code;

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Handles the read request for the scale attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_scale(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	int32_t vals[2];

	if (!dev || !channel)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	switch (channel->type) {
	case IIO_VOLTAGE:
		/*
		 * Full scale spans +/-VREF/gain over 2^24 codes, so one LSB is
		 * VREF / (gain * 2^23) millivolts. Every available gain is a
		 * power of two, so it folds into the log2 denominator and the
		 * scale stays exact instead of losing the fraction of a
		 * millivolt that VREF/gain would truncate.
		 */
		vals[0] = (int32_t)iio_ad7191->ad7191_dev->vref_mv;
		vals[1] = AD7191_BIT_SCALE +
			  no_os_find_first_set_bit(
				  ad7191_gain_to_value(iio_ad7191->ad7191_dev->gain));
		return iio_format_value(buf, len, IIO_VAL_FRACTIONAL_LOG2, 2, vals);
	case IIO_TEMP:
		/*
		 * The sensor produces 2815 codes per degree, so a code is
		 * 1000/2815 millidegrees.
		 */
		vals[0] = 1000;
		vals[1] = AD7191_TEMP_CODES_PER_DEGREE;
		return iio_format_value(buf, len, IIO_VAL_FRACTIONAL, 2, vals);
	default:
		return -EINVAL;
	}
}

/**
 * @brief Handles the read request for the offset attribute.
 *
 * Conversions are coded in offset binary, so the raw value must be shifted
 * down by mid-scale before the scale is applied. For temperature the kelvin
 * to Celsius correction folds into the same offset.
 *
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_offset(void *dev, char *buf, uint32_t len,
				  const struct iio_ch_info *channel,
				  intptr_t priv)
{
	int32_t val;

	if (!dev || !channel)
		return -EINVAL;

	switch (channel->type) {
	case IIO_VOLTAGE:
		val = -AD7191_MIDSCALE;
		break;
	case IIO_TEMP:
		val = -AD7191_MIDSCALE -
		      AD7191_TEMP_KELVIN_OFFSET * AD7191_TEMP_CODES_PER_DEGREE;
		break;
	default:
		return -EINVAL;
	}

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Handles the read request for the sampling frequency attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_odr(void *dev, char *buf, uint32_t len,
			       const struct iio_ch_info *channel, intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	int32_t val;

	if (!dev)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	val = (int32_t)ad7191_odr_to_hz(iio_ad7191->ad7191_dev->odr);

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Handles the write request for the sampling frequency attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer holding the requested value.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes read on success, negative error code otherwise.
 */
static int ad7191_iio_write_odr(void *dev, char *buf, uint32_t len,
				const struct iio_ch_info *channel,
				intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	enum ad7191_odr odr;
	int32_t val;
	int ret;

	if (!dev)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	ret = iio_parse_value(buf, IIO_VAL_INT, &val, NULL);
	if (ret)
		return ret;

	switch (val) {
	case 120:
		odr = AD7191_ODR_120HZ;
		break;
	case 60:
		odr = AD7191_ODR_60HZ;
		break;
	case 50:
		odr = AD7191_ODR_50HZ;
		break;
	case 10:
		odr = AD7191_ODR_10HZ;
		break;
	default:
		return -EINVAL;
	}

	ret = ad7191_set_odr(iio_ad7191->ad7191_dev, odr);
	if (ret)
		return ret;

	return len;
}

/**
 * @brief Handles the read request for the hardwaregain attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_gain(void *dev, char *buf, uint32_t len,
				const struct iio_ch_info *channel,
				intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	int32_t val;

	if (!dev)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	val = (int32_t)ad7191_gain_to_value(iio_ad7191->ad7191_dev->gain);

	return iio_format_value(buf, len, IIO_VAL_INT, 1, &val);
}

/**
 * @brief Handles the write request for the hardwaregain attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer holding the requested value.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes read on success, negative error code otherwise.
 */
static int ad7191_iio_write_gain(void *dev, char *buf, uint32_t len,
				 const struct iio_ch_info *channel,
				 intptr_t priv)
{
	struct ad7191_iio_dev *iio_ad7191;
	enum ad7191_gain gain;
	int32_t val;
	int ret;

	if (!dev)
		return -EINVAL;

	iio_ad7191 = dev;
	if (!iio_ad7191->ad7191_dev)
		return -EINVAL;

	ret = iio_parse_value(buf, IIO_VAL_INT, &val, NULL);
	if (ret)
		return ret;

	switch (val) {
	case 1:
		gain = AD7191_GAIN_1;
		break;
	case 8:
		gain = AD7191_GAIN_8;
		break;
	case 64:
		gain = AD7191_GAIN_64;
		break;
	case 128:
		gain = AD7191_GAIN_128;
		break;
	default:
		return -EINVAL;
	}

	ret = ad7191_set_gain(iio_ad7191->ad7191_dev, gain);
	if (ret)
		return ret;

	return len;
}

/**
 * @brief Handles the read request for the sampling_frequency_available
 *        attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_odr_avail(void *dev, char *buf, uint32_t len,
				     const struct iio_ch_info *channel,
				     intptr_t priv)
{
	return snprintf(buf, len, "10 50 60 120");
}

/**
 * @brief Handles the read request for the hardwaregain_available attribute.
 * @param dev     - The iio device structure.
 * @param buf     - Command buffer to be filled with requested data.
 * @param len     - Length of the received command buffer in bytes.
 * @param channel - Command channel info.
 * @param priv    - Command attribute id.
 * @return Number of bytes written on success, negative error code otherwise.
 */
static int ad7191_iio_read_gain_avail(void *dev, char *buf, uint32_t len,
				      const struct iio_ch_info *channel,
				      intptr_t priv)
{
	return snprintf(buf, len, "1 8 64 128");
}

/**
 * @brief Read samples into the IIO buffer.
 *
 * Only one input can be converted at a time, so a scan containing more than
 * one active channel would have to switch inputs between samples and pay the
 * settling time for each; that is rejected rather than silently returning
 * badly skewed data.
 *
 * @param iio_dev_data - The IIO device data structure.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_iio_submit_buffer(struct iio_device_data *iio_dev_data)
{
	struct ad7191_iio_dev *iio_ad7191;
	struct iio_buffer *buffer;
	uint32_t code;
	uint32_t i;
	int ret;

	if (!iio_dev_data || !iio_dev_data->dev)
		return -EINVAL;

	iio_ad7191 = iio_dev_data->dev;
	buffer = iio_dev_data->buffer;

	if (no_os_hweight32(buffer->active_mask) != 1)
		return -EINVAL;

	ret = ad7191_iio_select_input(iio_ad7191->ad7191_dev,
				      no_os_find_first_set_bit(buffer->active_mask));
	if (ret)
		return ret;

	for (i = 0; i < buffer->samples; i++) {
		ret = ad7191_read_sample(iio_ad7191->ad7191_dev, &code);
		if (ret)
			return ret;

		ret = iio_buffer_push_scan(buffer, &code);
		if (ret)
			return ret;
	}

	return 0;
}

static struct iio_attribute ad7191_iio_voltage_attrs[] = {
	{
		.name = "raw",
		.show = ad7191_iio_read_raw,
	},
	{
		.name = "scale",
		.show = ad7191_iio_read_scale,
	},
	{
		.name = "offset",
		.show = ad7191_iio_read_offset,
	},
	{
		.name = "hardwaregain",
		.show = ad7191_iio_read_gain,
		.store = ad7191_iio_write_gain,
	},
	{
		.name = "hardwaregain_available",
		.show = ad7191_iio_read_gain_avail,
	},
	END_ATTRIBUTES_ARRAY,
};

static struct iio_attribute ad7191_iio_temp_attrs[] = {
	{
		.name = "raw",
		.show = ad7191_iio_read_raw,
	},
	{
		.name = "scale",
		.show = ad7191_iio_read_scale,
	},
	{
		.name = "offset",
		.show = ad7191_iio_read_offset,
	},
	END_ATTRIBUTES_ARRAY,
};

static struct iio_attribute ad7191_iio_global_attrs[] = {
	{
		.name = "sampling_frequency",
		.show = ad7191_iio_read_odr,
		.store = ad7191_iio_write_odr,
	},
	{
		.name = "sampling_frequency_available",
		.show = ad7191_iio_read_odr_avail,
	},
	END_ATTRIBUTES_ARRAY,
};

/*
 * Conversions are offset binary, so the raw value is unsigned and the
 * mid-scale shift is expressed through the offset attribute instead.
 */
static struct scan_type ad7191_iio_scan_type = {
	.sign = 'u',
	.realbits = AD7191_RESOLUTION,
	.storagebits = 32,
	.shift = 0,
	.is_big_endian = false,
};

static struct iio_channel ad7191_channels[] = {
	{
		.name = "voltage0-voltage1",
		.ch_type = IIO_VOLTAGE,
		.channel = 0,
		.channel2 = 1,
		.address = AD7191_ADDR_AIN1_AIN2,
		.scan_index = 0,
		.scan_type = &ad7191_iio_scan_type,
		.attributes = ad7191_iio_voltage_attrs,
		.ch_out = false,
		.indexed = 1,
		.diferential = true,
	},
	{
		.name = "voltage2-voltage3",
		.ch_type = IIO_VOLTAGE,
		.channel = 2,
		.channel2 = 3,
		.address = AD7191_ADDR_AIN3_AIN4,
		.scan_index = 1,
		.scan_type = &ad7191_iio_scan_type,
		.attributes = ad7191_iio_voltage_attrs,
		.ch_out = false,
		.indexed = 1,
		.diferential = true,
	},
	{
		.name = "temp",
		.ch_type = IIO_TEMP,
		.channel = 0,
		.address = AD7191_ADDR_TEMP,
		.scan_index = 2,
		.scan_type = &ad7191_iio_scan_type,
		.attributes = ad7191_iio_temp_attrs,
		.ch_out = false,
		.indexed = 1,
	},
};

static struct iio_device ad7191_iio_dev = {
	.num_ch = NO_OS_ARRAY_SIZE(ad7191_channels),
	.channels = ad7191_channels,
	.attributes = ad7191_iio_global_attrs,
	.submit = ad7191_iio_submit_buffer,
};

/**
 * @brief Initialize the AD7191 IIO device.
 * @param iio_dev    - Pointer to store the IIO device descriptor.
 * @param init_param - Initialization parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_iio_init(struct ad7191_iio_dev **iio_dev,
		    struct ad7191_iio_dev_init_param *init_param)
{
	struct ad7191_iio_dev *desc;
	int ret;

	if (!iio_dev || !init_param || !init_param->ad7191_dev_init)
		return -EINVAL;

	desc = no_os_calloc(1, sizeof(*desc));
	if (!desc)
		return -ENOMEM;

	desc->iio_dev = &ad7191_iio_dev;

	ret = ad7191_init(&desc->ad7191_dev, init_param->ad7191_dev_init);
	if (ret)
		goto err_free;

	*iio_dev = desc;

	return 0;

err_free:
	no_os_free(desc);

	return ret;
}

/**
 * @brief Free the resources allocated by ad7191_iio_init().
 * @param desc - The IIO device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_iio_remove(struct ad7191_iio_dev *desc)
{
	int ret;

	if (!desc)
		return -EINVAL;

	ret = ad7191_remove(desc->ad7191_dev);
	if (ret)
		return ret;

	no_os_free(desc);

	return 0;
}
