AD7191 no-OS driver
===================

.. no-os-doxygen::

Supported Devices
-----------------

- :adi:`AD7191`

Overview
--------

The AD7191 is a pin-programmable, ultra-low noise 24-bit sigma-delta ADC
intended for bridge sensor measurements. It integrates a low noise
programmable gain amplifier with gains of 1, 8, 64 and 128, two differential
analog input pairs, an internal temperature sensor, and a bridge power-down
switch that disconnects the bridge between measurements.

Unlike most converters in its family the AD7191 has **no register map**. Every
setting - gain, output data rate, input pair, temperature sensor selection and
clock source - is strapped on a dedicated pin, and the serial interface is used
only to shift out conversion results. The interface is two-wire: SCLK plus a
single DOUT/RDY line that carries both the data and the data-ready indication.
There is no chip select and no input data line.

Conversions are 24 bits in offset binary, so a zero differential input reads as
mid-scale (0x800000), and the output data rate is selectable between 10 Hz,
50 Hz, 60 Hz and 120 Hz. The 50 Hz and 60 Hz rates place notches of the digital
filter on the corresponding mains frequency.

Applications
------------

* Weigh scales
* Strain gauges and pressure measurement
* Temperature measurement
* Bridge transducer interfaces

AD7191 Device Configuration
---------------------------

Driver Initialization
~~~~~~~~~~~~~~~~~~~~~

In order to be able to use the device, you will have to provide the support for
the communication protocol (SPI). Data is placed on DOUT/RDY on the SCLK
falling edge and is valid on the rising edge, so the SPI controller must be
configured for **mode 3**. The data setup and hold times limit SCLK to
**5 MHz**; ``ad7191_init`` rejects a higher ``max_speed_hz`` and rejects any
mode other than mode 3.

Every configuration pin is optional. A pin whose init parameter is left NULL is
assumed to be strapped in hardware: the driver will not drive it, and the
matching setter returns ``-ENOTSUP``. This makes it possible to drive only the
pins that a given board actually routes to the host.

The first API to be called is **ad7191_init**. Make sure that it returns 0,
which means that the driver was initialized correctly. It acquires the SPI
descriptor and every wired GPIO, drives each configuration pin to the requested
initial state, and then resets the device.

Reset and Power-Down
~~~~~~~~~~~~~~~~~~~~

The PDOWN pin doubles as the reset. **ad7191_reset** pulses it high, waits for
the internal oscillator to restart and then waits out the filter settling time.
PDOWN also tristates DOUT/RDY and opens the bridge power-down switch while it is
asserted, so the driver holds it low for the whole of the acquisition and pulses
it only on reset. It must not be used as a per-transfer chip select: that would
reset the modulator and filter before every sample.

Channel and Temperature Selection
~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~

**ad7191_set_channel** selects between the AIN1/AIN2 and AIN3/AIN4 differential
pairs through the CHAN pin. **ad7191_set_temp_en** routes the internal
temperature sensor to the modulator through the TEMP pin; while TEMP is asserted
it overrides CHAN, so the selected input pair is ignored.

Gain and Output Data Rate
~~~~~~~~~~~~~~~~~~~~~~~~~

**ad7191_set_gain** drives the PGA2 and PGA1 pins, and **ad7191_set_odr** drives
ODR2 and ODR1. Note that the output data rate encoding descends: the field value
0 selects the fastest rate (120 Hz) and 3 the slowest (10 Hz).
**ad7191_gain_to_value** and **ad7191_odr_to_hz** convert the enumerations to
the numeric gain and to hertz.

Any configuration change resets the modulator and the digital filter. The device
drives DOUT/RDY high and returns all ones until the new setting has settled, so
each setter waits the settling time of four conversion periods (400 ms at 10 Hz
down to 33.3 ms at 120 Hz) before returning.

Reading Conversions
~~~~~~~~~~~~~~~~~~~

**ad7191_read_sample** returns one 24-bit conversion result. Because DOUT/RDY is
also the MISO line, a host can only watch the ready indication if that wire is
additionally routed to a GPIO; when ``gpio_rdy`` is supplied the driver polls it
low, and when it is not the driver simply waits out one conversion period.
Each conversion can be read exactly once.

**ad7191_code_to_uvolts** applies the offset binary coding, the reference and the
current gain to produce microvolts. **ad7191_code_to_millidegrees** converts a
temperature reading, which the sensor produces at 2815 codes per kelvin.

AD7191 Driver Initialization Example
------------------------------------

.. code-block:: bash

	struct ad7191_dev *dev;

	struct no_os_uart_init_param uip = {
		.device_id = UART_DEVICE_ID,
		.baud_rate = UART_BAUDRATE,
		.size = NO_OS_UART_CS_8,
		.parity = NO_OS_UART_PAR_NO,
		.stop = NO_OS_UART_STOP_1_BIT,
		.platform_ops = UART_OPS,
		.extra = UART_EXTRA,
	};

	struct ad7191_init_param ad7191_ip = {
		.spi_init = {
			.device_id = SPI_DEVICE_ID,
			.max_speed_hz = 1000000,
			.chip_select = SPI_CS,
			.mode = NO_OS_SPI_MODE_3,
			.platform_ops = SPI_OPS,
			.extra = SPI_EXTRA,
		},
		.gpio_pga1 = &gpio_pga1_ip,
		.gpio_pga2 = &gpio_pga2_ip,
		.gpio_odr1 = &gpio_odr1_ip,
		.gpio_odr2 = &gpio_odr2_ip,
		.gpio_chan = &gpio_chan_ip,
		.gpio_temp = &gpio_temp_ip,
		.gpio_clksel = &gpio_clksel_ip,
		.gpio_pdown = &gpio_pdown_ip,
		.gpio_rdy = &gpio_rdy_ip,
		.vref_mv = 2500,
		.gain = AD7191_GAIN_1,
		.odr = AD7191_ODR_120HZ,
		.chan = AD7191_CHAN_AIN1_AIN2,
		.clksel = AD7191_CLK_INT,
	};

	ret = ad7191_init(&dev, &ad7191_ip);
	if (ret)
		goto error;

AD7191 no-OS IIO support
------------------------

The AD7191 IIO driver comes on top of the AD7191 driver and offers support for
interfacing IIO clients through libiio.

AD7191 IIO Device Configuration
-------------------------------

Channels
~~~~~~~~

Three channels are exposed: two differential voltage channels for the AIN1/AIN2
and AIN3/AIN4 pairs, and one temperature channel for the internal sensor. Only
one input can be converted at a time, so a buffered read must have exactly one
channel enabled; enabling more is rejected rather than returning samples skewed
by the settling time of each switch.

Channel Attributes
~~~~~~~~~~~~~~~~~~

* ``raw - the 24-bit conversion result, in offset binary``
* ``scale - millivolts (voltage channels) or millidegrees Celsius (temperature channel) per code``
* ``offset - value added to raw before scaling: the mid-scale shift of the offset binary coding, plus the kelvin to Celsius correction on the temperature channel``
* ``hardwaregain - PGA gain, on the voltage channels only``
* ``hardwaregain_available - the selectable gains: 1, 8, 64 and 128``

Global Attributes
~~~~~~~~~~~~~~~~~

* ``sampling_frequency - output data rate in Hz``
* ``sampling_frequency_available - the selectable rates: 10, 50, 60 and 120``

AD7191 IIO Driver Initialization Example
----------------------------------------

.. code-block:: bash

	int ret;

	struct ad7191_iio_dev *ad7191_iio_dev;
	struct ad7191_iio_dev_init_param ad7191_iio_ip = {
		.ad7191_dev_init = &ad7191_ip,
	};

	struct iio_app_desc *app;
	struct iio_app_init_param app_init_param = {0};

	ret = ad7191_iio_init(&ad7191_iio_dev, &ad7191_iio_ip);
	if (ret)
		goto exit;

	struct iio_app_device iio_devices[] = {
		{
			.name = "ad7191",
			.dev = ad7191_iio_dev,
			.dev_descriptor = ad7191_iio_dev->iio_dev,
		},
	};

	app_init_param.devices = iio_devices;
	app_init_param.nb_devices = NO_OS_ARRAY_SIZE(iio_devices);
	app_init_param.uart_init_params = uip;

	ret = iio_app_init(&app, app_init_param);
	if (ret)
		goto remove_iio_ad7191;

	return iio_app_run(app);
