/***************************************************************************//**
 *   @file   ad7191.h
 *   @brief  Header file of AD7191 Driver.
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
#ifndef __AD7191_H__
#define __AD7191_H__

#include <stdbool.h>
#include <stdint.h>

#include "no_os_gpio.h"
#include "no_os_spi.h"
#include "no_os_util.h"

/** Conversion result width. */
#define AD7191_RESOLUTION		24
/** Number of bytes in one conversion result. */
#define AD7191_DATA_SIZE_BYTES		3
/** Offset binary mid-scale: the code for a 0 V differential input. */
#define AD7191_MIDSCALE			0x800000
/**
 * All-ones is returned while the part is settling after a configuration
 * change, so it is a "not ready" marker rather than a valid conversion.
 */
#define AD7191_DATA_NOT_READY		0xFFFFFF
/**
 * Maximum SCLK. The datasheet specifies t3 (SCLK falling edge to data valid)
 * and t4 (data valid to SCLK rising edge) as 100 ns min each, so a full period
 * cannot be shorter than 200 ns.
 */
#define AD7191_MAX_SPI_FREQ_HZ		5000000
/** Temperature sensor sensitivity, in codes per degree. */
#define AD7191_TEMP_CODES_PER_DEGREE	2815
/** Offset between the sensor's kelvin reading and degrees Celsius. */
#define AD7191_TEMP_KELVIN_OFFSET	273

/**
 * @enum ad7191_gain
 * @brief PGA gain, as encoded on the PGA2 and PGA1 pins.
 *
 * The value is the two-bit field {PGA2, PGA1} from Table 7 of the datasheet.
 */
enum ad7191_gain {
	/** PGA2 = 0, PGA1 = 0 */
	AD7191_GAIN_1 = 0,
	/** PGA2 = 0, PGA1 = 1 */
	AD7191_GAIN_8 = 1,
	/** PGA2 = 1, PGA1 = 0 */
	AD7191_GAIN_64 = 2,
	/** PGA2 = 1, PGA1 = 1 */
	AD7191_GAIN_128 = 3,
};

/**
 * @enum ad7191_odr
 * @brief Output data rate, as encoded on the ODR2 and ODR1 pins.
 *
 * The value is the two-bit field {ODR2, ODR1}. Note that the encoding
 * descends: a larger field value selects a slower rate.
 */
enum ad7191_odr {
	/** ODR2 = 0, ODR1 = 0 */
	AD7191_ODR_120HZ = 0,
	/** ODR2 = 0, ODR1 = 1 */
	AD7191_ODR_60HZ = 1,
	/** ODR2 = 1, ODR1 = 0 */
	AD7191_ODR_50HZ = 2,
	/** ODR2 = 1, ODR1 = 1 */
	AD7191_ODR_10HZ = 3,
};

/**
 * @enum ad7191_chan
 * @brief Analog input pair, as selected by the CHAN pin.
 *
 * Ignored by the part while the TEMP pin is asserted.
 */
enum ad7191_chan {
	/** CHAN = 0 */
	AD7191_CHAN_AIN1_AIN2 = 0,
	/** CHAN = 1 */
	AD7191_CHAN_AIN3_AIN4 = 1,
};

/**
 * @enum ad7191_clksel
 * @brief Clock source, as selected by the CLKSEL pin.
 */
enum ad7191_clksel {
	/** External clock or crystal (2.4576 MHz to 5.12 MHz). */
	AD7191_CLK_EXT = 0,
	/** Internal 4.92 MHz +/-4% oscillator. */
	AD7191_CLK_INT = 1,
};

/**
 * @struct ad7191_init_param
 * @brief AD7191 device initialization parameters.
 *
 * Every GPIO is optional: a pin left NULL (or with number -1) is assumed to be
 * strapped in hardware, and the driver will not attempt to drive it. The
 * corresponding configuration is then fixed at whatever the strapping selects,
 * and the matching setter returns -ENOTSUP.
 */
struct ad7191_init_param {
	/** SPI initialization parameters. Must be mode 3. */
	struct no_os_spi_init_param spi_init;
	/** PGA1 pin, low bit of the gain selection. */
	struct no_os_gpio_init_param *gpio_pga1;
	/** PGA2 pin, high bit of the gain selection. */
	struct no_os_gpio_init_param *gpio_pga2;
	/** ODR1 pin, low bit of the output data rate selection. */
	struct no_os_gpio_init_param *gpio_odr1;
	/** ODR2 pin, high bit of the output data rate selection. */
	struct no_os_gpio_init_param *gpio_odr2;
	/** CHAN pin, analog input pair selection. */
	struct no_os_gpio_init_param *gpio_chan;
	/** TEMP pin, internal temperature sensor selection. */
	struct no_os_gpio_init_param *gpio_temp;
	/** CLKSEL pin, clock source selection. */
	struct no_os_gpio_init_param *gpio_clksel;
	/** PDOWN pin, power-down and reset. */
	struct no_os_gpio_init_param *gpio_pdown;
	/**
	 * Optional GPIO watching the DOUT/RDY line. The AD7191 has no separate
	 * data-ready output: DOUT/RDY is the same wire as SPI MISO, so sensing
	 * it requires that wire to also reach a GPIO. When this is NULL the
	 * driver falls back to waiting out one conversion period instead.
	 */
	struct no_os_gpio_init_param *gpio_rdy;
	/** Reference voltage in millivolts. */
	uint32_t vref_mv;
	/** Initial PGA gain. */
	enum ad7191_gain gain;
	/** Initial output data rate. */
	enum ad7191_odr odr;
	/** Initial analog input pair. */
	enum ad7191_chan chan;
	/** Clock source. */
	enum ad7191_clksel clksel;
};

/**
 * @struct ad7191_dev
 * @brief AD7191 device descriptor.
 */
struct ad7191_dev {
	/** SPI device descriptor. */
	struct no_os_spi_desc *spi_desc;
	/** PGA1 pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_pga1;
	/** PGA2 pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_pga2;
	/** ODR1 pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_odr1;
	/** ODR2 pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_odr2;
	/** CHAN pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_chan;
	/** TEMP pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_temp;
	/** CLKSEL pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_clksel;
	/** PDOWN pin descriptor, NULL if strapped. */
	struct no_os_gpio_desc *gpio_pdown;
	/** DOUT/RDY sense pin descriptor, NULL if not wired. */
	struct no_os_gpio_desc *gpio_rdy;
	/** Reference voltage in millivolts. */
	uint32_t vref_mv;
	/** Currently selected PGA gain. */
	enum ad7191_gain gain;
	/** Currently selected output data rate. */
	enum ad7191_odr odr;
	/** Currently selected analog input pair. */
	enum ad7191_chan chan;
	/** Clock source. */
	enum ad7191_clksel clksel;
	/** True while the internal temperature sensor is selected. */
	bool temp_en;
};

/** Convert an ad7191_odr to the corresponding rate in Hz. */
uint32_t ad7191_odr_to_hz(enum ad7191_odr odr);

/** Convert an ad7191_gain to the corresponding numeric gain. */
uint32_t ad7191_gain_to_value(enum ad7191_gain gain);

/** Initialize the device. */
int ad7191_init(struct ad7191_dev **dev,
		const struct ad7191_init_param *init_param);

/** Free the resources allocated by ad7191_init(). */
int ad7191_remove(struct ad7191_dev *dev);

/** Reset the device by pulsing PDOWN. */
int ad7191_reset(struct ad7191_dev *dev);

/** Set the PGA gain. */
int ad7191_set_gain(struct ad7191_dev *dev, enum ad7191_gain gain);

/** Set the output data rate. */
int ad7191_set_odr(struct ad7191_dev *dev, enum ad7191_odr odr);

/** Select the analog input pair. */
int ad7191_set_channel(struct ad7191_dev *dev, enum ad7191_chan chan);

/** Enable or disable the internal temperature sensor. */
int ad7191_set_temp_en(struct ad7191_dev *dev, bool enable);

/** Read a single conversion result. */
int ad7191_read_sample(struct ad7191_dev *dev, uint32_t *code);

/** Convert a raw code to a voltage in microvolts. */
int ad7191_code_to_uvolts(struct ad7191_dev *dev, uint32_t code,
			  int64_t *uvolts);

/** Convert a raw code to a temperature in millidegrees Celsius. */
int ad7191_code_to_millidegrees(uint32_t code, int32_t *millidegrees);

#endif /* __AD7191_H__ */
