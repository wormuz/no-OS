/***************************************************************************//**
 *   @file   parameters.h
 *   @brief  Definitions specific to Linux platform used by eval-ad7191 project.
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

#ifndef __PARAMETERS_H__
#define __PARAMETERS_H__

#include "linux_gpio.h"
#include "linux_spi.h"
#include "linux_uart.h"

extern struct linux_uart_init_param ad7191_uart_extra_ip;

/*
 * The UART is only used for the console of the basic example. On a Raspberry Pi
 * the primary UART is /dev/ttyAMA0 once "enable_uart=1" is set in config.txt and
 * the Linux console has been released from it. The IIO example does not use the
 * UART at all: iio_app serves IIOD over TCP on the linux platform.
 */
#define UART_DEVICE_ID		0
#define UART_BAUDRATE		115200
#define UART_IRQ_ID		0
#define UART_OPS		&linux_uart_ops
#define UART_EXTRA		&ad7191_uart_extra_ip

/*
 * /dev/spidev0.0. Enable it with "dtparam=spi=on" in /boot/firmware/config.txt.
 * The AD7191 has no chip select, so the CS0 line of SPI0 is simply left
 * unconnected; spidev still asserts it, harmlessly.
 *
 * 1 MHz is comfortably below the 5 MHz the AD7191 data setup and hold times
 * allow.
 */
#define SPI_DEVICE_ID		0
#define SPI_BAUDRATE		1000000
#define SPI_CS			0
#define SPI_OPS			&linux_spi_ops
#define SPI_EXTRA		NULL

/*
 * GPIO numbers are BCM numbers within a gpiochip, and *_PORT_NUM is the
 * gpiochip index. On a Raspberry Pi 5 the 40-pin header is on the RP1
 * southbridge, which is not gpiochip0 - run "gpiodetect" and pick the chip
 * described as "pinctrl-rp1"; it is commonly gpiochip4 but the numbering
 * depends on the kernel version.
 */
#define GPIO_CHIP_NUM		4

#define GPIO_PGA1_PIN_NUM	5
#define GPIO_PGA1_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_PGA2_PIN_NUM	6
#define GPIO_PGA2_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_ODR1_PIN_NUM	13
#define GPIO_ODR1_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_ODR2_PIN_NUM	19
#define GPIO_ODR2_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_CHAN_PIN_NUM	26
#define GPIO_CHAN_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_TEMP_PIN_NUM	16
#define GPIO_TEMP_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_CLKSEL_PIN_NUM	20
#define GPIO_CLKSEL_PORT_NUM	GPIO_CHIP_NUM
#define GPIO_PDOWN_PIN_NUM	21
#define GPIO_PDOWN_PORT_NUM	GPIO_CHIP_NUM

/*
 * DOUT/RDY is wired to both SPI0 MISO (GPIO9) and this pin, so the ready
 * indication can be sensed through the GPIO character device while spidev
 * clocks the conversion out.
 */
#define GPIO_RDY_PIN_NUM	25
#define GPIO_RDY_PORT_NUM	GPIO_CHIP_NUM

#define GPIO_OPS		&linux_gpio_ops
#define GPIO_EXTRA		NULL

#endif /* __PARAMETERS_H__ */
