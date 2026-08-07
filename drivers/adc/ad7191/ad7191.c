/***************************************************************************//**
 *   @file   ad7191.c
 *   @brief  Implementation of AD7191 Driver.
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
#include <stdlib.h>
#include <string.h>

#include "ad7191.h"
#include "no_os_alloc.h"
#include "no_os_delay.h"
#include "no_os_error.h"
#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

/**
 * Oscillator start-up time after PDOWN is released, in microseconds. The
 * datasheet quotes about 1 ms; round up for margin.
 */
#define AD7191_OSC_STARTUP_US		2000
/** PDOWN pulse width. The datasheet minimum is 100 ns. */
#define AD7191_PDOWN_PULSE_US		10
/**
 * Granularity of the DOUT/RDY poll loop. Fine enough not to add meaningful
 * jitter at the fastest 120 Hz rate (8.3 ms period), coarse enough not to spin.
 */
#define AD7191_RDY_POLL_STEP_US		100

/**
 * @brief Convert an output data rate selection to its rate in Hz.
 * @param odr - Output data rate selection.
 * @return Rate in Hz.
 */
uint32_t ad7191_odr_to_hz(enum ad7191_odr odr)
{
	switch (odr) {
	case AD7191_ODR_120HZ:
		return 120;
	case AD7191_ODR_60HZ:
		return 60;
	case AD7191_ODR_50HZ:
		return 50;
	case AD7191_ODR_10HZ:
	default:
		return 10;
	}
}

/**
 * @brief Convert a gain selection to its numeric gain.
 * @param gain - Gain selection.
 * @return Numeric gain.
 */
uint32_t ad7191_gain_to_value(enum ad7191_gain gain)
{
	switch (gain) {
	case AD7191_GAIN_1:
		return 1;
	case AD7191_GAIN_8:
		return 8;
	case AD7191_GAIN_64:
		return 64;
	case AD7191_GAIN_128:
	default:
		return 128;
	}
}

/**
 * @brief Conversion period for the current output data rate, in microseconds.
 * @param dev - Device descriptor.
 * @return Conversion period in microseconds.
 */
static uint32_t ad7191_conversion_time_us(struct ad7191_dev *dev)
{
	return 1000000UL / ad7191_odr_to_hz(dev->odr);
}

/**
 * @brief Settling time after a configuration change, in microseconds.
 *
 * Any change to the gain, output data rate, channel, temperature or clock
 * selection resets the modulator and the digital filter. The datasheet gives
 * the resulting settle time as four conversion periods.
 *
 * @param dev - Device descriptor.
 * @return Settling time in microseconds.
 */
static uint32_t ad7191_settling_time_us(struct ad7191_dev *dev)
{
	return 4 * ad7191_conversion_time_us(dev);
}

/**
 * @brief Drive an optional GPIO, ignoring pins that are strapped in hardware.
 * @param desc - GPIO descriptor, may be NULL.
 * @param value - Value to drive.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_gpio_set(struct no_os_gpio_desc *desc, uint8_t value)
{
	if (!desc)
		return 0;

	return no_os_gpio_set_value(desc, value);
}

/**
 * @brief Apply a two-bit selection to a pair of pins.
 * @param lsb - Descriptor for the low bit, may be NULL.
 * @param msb - Descriptor for the high bit, may be NULL.
 * @param value - Two-bit value to apply.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_gpio_set_pair(struct no_os_gpio_desc *lsb,
				struct no_os_gpio_desc *msb,
				uint8_t value)
{
	int ret;

	ret = ad7191_gpio_set(lsb, value & NO_OS_BIT(0) ?
			      NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	return ad7191_gpio_set(msb, value & NO_OS_BIT(1) ?
			       NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
}

/**
 * @brief Configure an optional GPIO as an output driving a given level.
 *
 * Pins are only driveable once their direction has been set: the Linux GPIO
 * character device, for one, requests lines as inputs by default, so a bare
 * set_value() on a freshly acquired line has no effect.
 *
 * @param desc - GPIO descriptor, may be NULL.
 * @param value - Initial value to drive.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_gpio_init_output(struct no_os_gpio_desc *desc, uint8_t value)
{
	if (!desc)
		return 0;

	return no_os_gpio_direction_output(desc, value);
}

/**
 * @brief Configure an optional pair of pins as outputs driving a two-bit value.
 * @param lsb - Descriptor for the low bit, may be NULL.
 * @param msb - Descriptor for the high bit, may be NULL.
 * @param value - Two-bit value to drive.
 * @return 0 in case of success, negative error code otherwise.
 */
static int ad7191_gpio_init_output_pair(struct no_os_gpio_desc *lsb,
					struct no_os_gpio_desc *msb,
					uint8_t value)
{
	int ret;

	ret = ad7191_gpio_init_output(lsb, value & NO_OS_BIT(0) ?
				      NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	return ad7191_gpio_init_output(msb, value & NO_OS_BIT(1) ?
				       NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
}

/**
 * @brief Wait for the part to settle after a configuration change.
 * @param dev - Device descriptor.
 */
static void ad7191_wait_settled(struct ad7191_dev *dev)
{
	no_os_udelay(ad7191_settling_time_us(dev));
}

/**
 * @brief Reset the device.
 *
 * PDOWN doubles as the reset input: holding it high powers the part down,
 * tristates DOUT/RDY and opens the bridge power-down switch. Releasing it
 * restarts the oscillator and begins a fresh conversion sequence.
 *
 * @param dev - Device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_reset(struct ad7191_dev *dev)
{
	int ret;

	if (!dev)
		return -EINVAL;

	if (!dev->gpio_pdown)
		return -ENOTSUP;

	ret = no_os_gpio_set_value(dev->gpio_pdown, NO_OS_GPIO_HIGH);
	if (ret)
		return ret;

	no_os_udelay(AD7191_PDOWN_PULSE_US);

	ret = no_os_gpio_set_value(dev->gpio_pdown, NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	no_os_udelay(AD7191_OSC_STARTUP_US);
	ad7191_wait_settled(dev);

	return 0;
}

/**
 * @brief Set the PGA gain.
 * @param dev - Device descriptor.
 * @param gain - Gain to select.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_set_gain(struct ad7191_dev *dev, enum ad7191_gain gain)
{
	int ret;

	if (!dev)
		return -EINVAL;

	if (gain > AD7191_GAIN_128)
		return -EINVAL;

	if (!dev->gpio_pga1 || !dev->gpio_pga2)
		return -ENOTSUP;

	ret = ad7191_gpio_set_pair(dev->gpio_pga1, dev->gpio_pga2, gain);
	if (ret)
		return ret;

	dev->gain = gain;
	ad7191_wait_settled(dev);

	return 0;
}

/**
 * @brief Set the output data rate.
 * @param dev - Device descriptor.
 * @param odr - Output data rate to select.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_set_odr(struct ad7191_dev *dev, enum ad7191_odr odr)
{
	int ret;

	if (!dev)
		return -EINVAL;

	if (odr > AD7191_ODR_10HZ)
		return -EINVAL;

	if (!dev->gpio_odr1 || !dev->gpio_odr2)
		return -ENOTSUP;

	ret = ad7191_gpio_set_pair(dev->gpio_odr1, dev->gpio_odr2, odr);
	if (ret)
		return ret;

	dev->odr = odr;
	/* Settle against the new rate, which may be slower than the old one. */
	ad7191_wait_settled(dev);

	return 0;
}

/**
 * @brief Select the analog input pair.
 *
 * Has no effect on the conversion result while the temperature sensor is
 * enabled, since TEMP overrides CHAN, but the selection is still applied and
 * takes effect once the sensor is disabled again.
 *
 * @param dev - Device descriptor.
 * @param chan - Analog input pair to select.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_set_channel(struct ad7191_dev *dev, enum ad7191_chan chan)
{
	int ret;

	if (!dev)
		return -EINVAL;

	if (chan > AD7191_CHAN_AIN3_AIN4)
		return -EINVAL;

	if (!dev->gpio_chan)
		return -ENOTSUP;

	ret = no_os_gpio_set_value(dev->gpio_chan, chan ?
				   NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	dev->chan = chan;
	ad7191_wait_settled(dev);

	return 0;
}

/**
 * @brief Enable or disable the internal temperature sensor.
 *
 * While enabled, the sensor is converted instead of whichever analog input
 * pair CHAN selects, at a fixed gain of 1.
 *
 * @param dev - Device descriptor.
 * @param enable - True to select the temperature sensor.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_set_temp_en(struct ad7191_dev *dev, bool enable)
{
	int ret;

	if (!dev)
		return -EINVAL;

	if (!dev->gpio_temp)
		return -ENOTSUP;

	ret = no_os_gpio_set_value(dev->gpio_temp, enable ?
				   NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		return ret;

	dev->temp_en = enable;
	ad7191_wait_settled(dev);

	return 0;
}

/**
 * @brief Wait for DOUT/RDY to go low, signalling a conversion is available.
 * @param dev - Device descriptor.
 * @param timeout_us - Maximum time to wait, in microseconds.
 * @return 0 in case of success, -ETIMEDOUT on timeout, negative error code
 *         otherwise.
 */
static int ad7191_wait_rdy_low(struct ad7191_dev *dev, uint32_t timeout_us)
{
	uint32_t elapsed_us = 0;
	uint8_t value;
	int ret;

	while (elapsed_us < timeout_us) {
		ret = no_os_gpio_get_value(dev->gpio_rdy, &value);
		if (ret)
			return ret;

		if (value == NO_OS_GPIO_LOW)
			return 0;

		no_os_udelay(AD7191_RDY_POLL_STEP_US);
		elapsed_us += AD7191_RDY_POLL_STEP_US;
	}

	return -ETIMEDOUT;
}

/**
 * @brief Read a single conversion result.
 *
 * The AD7191 has no register map and no command phase: clocking 24 bits out of
 * DOUT/RDY is the entire read. Each conversion can be read exactly once.
 *
 * When a DOUT/RDY sense GPIO is available the read is synchronised to the
 * falling edge that marks new data. Otherwise the driver waits out one
 * conversion period and relies on the all-ones "not ready" pattern to detect
 * that it looked too early.
 *
 * @param dev - Device descriptor.
 * @param code - Pointer to store the raw conversion result.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_read_sample(struct ad7191_dev *dev, uint32_t *code)
{
	uint8_t buf[AD7191_DATA_SIZE_BYTES];
	uint32_t timeout_us;
	uint32_t elapsed_us;
	uint32_t raw;
	int ret;

	if (!dev || !code)
		return -EINVAL;

	/*
	 * Allow for a conversion that has only just started, plus a full
	 * settling window in case a configuration change is still in flight.
	 */
	timeout_us = 2 * ad7191_conversion_time_us(dev) +
		     ad7191_settling_time_us(dev);

	if (dev->gpio_rdy) {
		ret = ad7191_wait_rdy_low(dev, timeout_us);
		if (ret)
			return ret;
	}

	elapsed_us = 0;
	do {
		if (!dev->gpio_rdy) {
			no_os_udelay(ad7191_conversion_time_us(dev));
			elapsed_us += ad7191_conversion_time_us(dev);
		}

		memset(buf, 0, sizeof(buf));
		ret = no_os_spi_write_and_read(dev->spi_desc, buf, sizeof(buf));
		if (ret)
			return ret;

		raw = no_os_get_unaligned_be24(buf);
		if (raw != AD7191_DATA_NOT_READY) {
			*code = raw;
			return 0;
		}

		/*
		 * All ones means the part is still settling. With a sense GPIO
		 * this should not happen after a successful wait, so fail fast
		 * rather than spin.
		 */
		if (dev->gpio_rdy)
			return -EAGAIN;
	} while (elapsed_us < timeout_us);

	return -ETIMEDOUT;
}

/**
 * @brief Convert a raw code to a voltage in microvolts.
 *
 * The AD7191 codes bipolar inputs in offset binary, so mid-scale corresponds
 * to a 0 V differential input:
 *
 *	AIN = (code - 2^23) * VREF / (gain * 2^23)
 *
 * @param dev - Device descriptor.
 * @param code - Raw conversion result.
 * @param uvolts - Pointer to store the voltage in microvolts.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_code_to_uvolts(struct ad7191_dev *dev, uint32_t code,
			  int64_t *uvolts)
{
	int64_t centered;

	if (!dev || !uvolts)
		return -EINVAL;

	centered = (int64_t)code - AD7191_MIDSCALE;
	*uvolts = (centered * (int64_t)dev->vref_mv * 1000) /
		  ((int64_t)ad7191_gain_to_value(dev->gain) << (AD7191_RESOLUTION - 1));

	return 0;
}

/**
 * @brief Convert a raw code to a temperature in millidegrees Celsius.
 *
 * The temperature sensor reads in kelvin at 2815 codes per degree:
 *
 *	T(K) = (code - 2^23) / 2815,  T(degC) = T(K) - 273
 *
 * @param code - Raw conversion result.
 * @param millidegrees - Pointer to store the temperature in millidegrees C.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_code_to_millidegrees(uint32_t code, int32_t *millidegrees)
{
	int64_t centered;

	if (!millidegrees)
		return -EINVAL;

	centered = (int64_t)code - AD7191_MIDSCALE;
	*millidegrees = (int32_t)((centered * 1000) / AD7191_TEMP_CODES_PER_DEGREE -
				  AD7191_TEMP_KELVIN_OFFSET * 1000);

	return 0;
}

/**
 * @brief Initialize the device.
 * @param dev - Pointer to store the device descriptor.
 * @param init_param - Initialization parameters.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_init(struct ad7191_dev **dev,
		const struct ad7191_init_param *init_param)
{
	struct ad7191_dev *descriptor;
	int ret;

	if (!dev || !init_param)
		return -EINVAL;

	if (!init_param->vref_mv)
		return -EINVAL;

	if (init_param->spi_init.max_speed_hz > AD7191_MAX_SPI_FREQ_HZ)
		return -EINVAL;

	/*
	 * Data is placed on DOUT/RDY on the SCLK falling edge and is valid on
	 * the rising edge, which is SPI mode 3.
	 */
	if (init_param->spi_init.mode != NO_OS_SPI_MODE_3)
		return -EINVAL;

	descriptor = no_os_calloc(1, sizeof(*descriptor));
	if (!descriptor)
		return -ENOMEM;

	descriptor->vref_mv = init_param->vref_mv;
	descriptor->gain = init_param->gain;
	descriptor->odr = init_param->odr;
	descriptor->chan = init_param->chan;
	descriptor->clksel = init_param->clksel;

	ret = no_os_spi_init(&descriptor->spi_desc, &init_param->spi_init);
	if (ret)
		goto err_free;

	ret = no_os_gpio_get_optional(&descriptor->gpio_pga1,
				      init_param->gpio_pga1);
	if (ret)
		goto err_spi;

	ret = no_os_gpio_get_optional(&descriptor->gpio_pga2,
				      init_param->gpio_pga2);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_odr1,
				      init_param->gpio_odr1);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_odr2,
				      init_param->gpio_odr2);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_chan,
				      init_param->gpio_chan);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_temp,
				      init_param->gpio_temp);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_clksel,
				      init_param->gpio_clksel);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_pdown,
				      init_param->gpio_pdown);
	if (ret)
		goto err_gpio;

	ret = no_os_gpio_get_optional(&descriptor->gpio_rdy,
				      init_param->gpio_rdy);
	if (ret)
		goto err_gpio;

	/*
	 * Bring every driven pin to a defined level before releasing PDOWN.
	 * PDOWN starts asserted so the part is held in reset while the rest of
	 * the configuration is applied, and DOUT/RDY stays tristated meanwhile.
	 */
	ret = ad7191_gpio_init_output(descriptor->gpio_pdown, NO_OS_GPIO_HIGH);
	if (ret)
		goto err_gpio;

	ret = ad7191_gpio_init_output_pair(descriptor->gpio_pga1,
					   descriptor->gpio_pga2,
					   descriptor->gain);
	if (ret)
		goto err_gpio;

	ret = ad7191_gpio_init_output_pair(descriptor->gpio_odr1,
					   descriptor->gpio_odr2,
					   descriptor->odr);
	if (ret)
		goto err_gpio;

	ret = ad7191_gpio_init_output(descriptor->gpio_chan, descriptor->chan ?
				      NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		goto err_gpio;

	ret = ad7191_gpio_init_output(descriptor->gpio_temp, NO_OS_GPIO_LOW);
	if (ret)
		goto err_gpio;

	ret = ad7191_gpio_init_output(descriptor->gpio_clksel,
				      descriptor->clksel ?
				      NO_OS_GPIO_HIGH : NO_OS_GPIO_LOW);
	if (ret)
		goto err_gpio;

	if (descriptor->gpio_rdy) {
		ret = no_os_gpio_direction_input(descriptor->gpio_rdy);
		if (ret)
			goto err_gpio;
	}

	if (descriptor->gpio_pdown) {
		ret = ad7191_reset(descriptor);
		if (ret)
			goto err_gpio;
	} else {
		/* No reset pin: just wait for the filter to settle. */
		ad7191_wait_settled(descriptor);
	}

	*dev = descriptor;

	return 0;

err_gpio:
	no_os_gpio_remove(descriptor->gpio_rdy);
	no_os_gpio_remove(descriptor->gpio_pdown);
	no_os_gpio_remove(descriptor->gpio_clksel);
	no_os_gpio_remove(descriptor->gpio_temp);
	no_os_gpio_remove(descriptor->gpio_chan);
	no_os_gpio_remove(descriptor->gpio_odr2);
	no_os_gpio_remove(descriptor->gpio_odr1);
	no_os_gpio_remove(descriptor->gpio_pga2);
	no_os_gpio_remove(descriptor->gpio_pga1);
err_spi:
	no_os_spi_remove(descriptor->spi_desc);
err_free:
	no_os_free(descriptor);

	return ret;
}

/**
 * @brief Free the resources allocated by ad7191_init().
 * @param dev - Device descriptor.
 * @return 0 in case of success, negative error code otherwise.
 */
int ad7191_remove(struct ad7191_dev *dev)
{
	if (!dev)
		return -EINVAL;

	/* Leave the part powered down. */
	ad7191_gpio_set(dev->gpio_pdown, NO_OS_GPIO_HIGH);

	no_os_gpio_remove(dev->gpio_rdy);
	no_os_gpio_remove(dev->gpio_pdown);
	no_os_gpio_remove(dev->gpio_clksel);
	no_os_gpio_remove(dev->gpio_temp);
	no_os_gpio_remove(dev->gpio_chan);
	no_os_gpio_remove(dev->gpio_odr2);
	no_os_gpio_remove(dev->gpio_odr1);
	no_os_gpio_remove(dev->gpio_pga2);
	no_os_gpio_remove(dev->gpio_pga1);
	no_os_spi_remove(dev->spi_desc);
	no_os_free(dev);

	return 0;
}
