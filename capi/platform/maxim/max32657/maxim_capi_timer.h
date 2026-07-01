/*******************************************************************************
 *   @file   maxim_capi_timer.h
 *   @brief  Header file for Timer and PWM functions
 *   @author Ramon Miguel Imbao (ramonmiguel.imbao@analog.com)
********************************************************************************
 * Copyright 2026(c) Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
*******************************************************************************/

#ifndef MAXIM_CAPI_TIMER_H_
#define MAXIM_CAPI_TIMER_H_

#include "capi_timer.h"
#include "tmr.h"
#include "maxim_capi_gpio.h"

#define MAX_CAPI_PWM_TMR_MAX_VAL	0x0000FFFF
#define MAX_CAPI_PWM_PRESCALER_VAL(n)	((n - 1U) * 16)
#define MAX_CAPI_PWM_PRESCALER_TRUE(n)	(1U << ((n) / 16))
#define MAX_CAPI_PWM_GET_PRESCALER(n)	(1U << ((n) - 1))
#define MAX_CAPI_MAX_PRESCALER_INDEX	13U	/* 2^(13 - 1) = 4096 */

enum max_capi_timer_clock_source {
	/** Peripheral clock */
	MAX_CAPI_TIMER_CLOCK_APB,
	/** External clock input */
	MAX_CAPI_TIMER_CLOCK_EXTERNAL,
	/** Internal baud rate oscillator (7.3728 MHz) */
	MAX_CAPI_TIMER_CLOCK_IBRO,
	/** External RTC oscillator (32.768 kHz) */
	MAX_CAPI_TIMER_CLOCK_ERTCO,
	/** Internal nano-ring oscillator */
	MAX_CAPI_TIMER_CLOCK_INRO,
	/** IBRO divided by 8 (921.6 kHz) */
	MAX_CAPI_TIMER_CLOCK_IBRO_DIV8,
};

enum max_capi_timer_bit_mode {
	/** 32-bit timer mode */
	MAX_CAPI_TIMER_BIT_MODE_32BIT,
	/** 16-bit timer mode */
	MAX_CAPI_TIMER_BIT_MODE_16BIT_DUAL,
};

enum max_capi_timer_channel_mode {
	MAX_CAPI_TIMER_MODE_ONESHOT = CAPI_TIMER_CHANNEL_MODE_LIMIT,
	MAX_CAPI_TIMER_MODE_CONTINUOUS,
	MAX_CAPI_TIMER_MODE_COUNTER,
	MAX_CAPI_TIMER_MODE_GATED,
	MAX_CAPI_TIMER_MODE_CAPTURE_COMPARE,
	MAX_CAPI_TIMER_MODE_DUAL_EDGE,
};

struct max_capi_timer_extra {
	/** Timer bit mode (32-bit or 16-bit dual) */
	enum max_capi_timer_bit_mode bit_mode;
};

struct max_capi_timer_channel_extra {
	/** GPIO voltage selection */
	enum max_capi_gpio_vssel vssel;
	/** GPIO initialization flag */
	bool init_pin;
	/** Alternate pin selection */
	bool use_alternate_pin;
};

struct max_capi_timer_counter_extra {
	/** GPIO voltage selection */
	enum max_capi_gpio_vssel vssel;
	/** GPIO initialization flag */
	bool init_pin;
	/** Alternate pin selection */
	bool use_alternate_pin;
};

struct max_capi_timer_channel_state {
	/** Timer config */
	mxc_tmr_cfg_t config;
	/** PWM mode parameters */
	struct {
		/** PWM period in ticks */
		uint32_t period_ticks;
		/** Duty cycle in ticks */
		uint32_t duty_cycle_ticks;
	} pwm;
	/** Compare mode parameters */
	struct {
		/** Compare match value */
		uint32_t value;
	} compare;
	/** GPIO initialization flag */
	bool init_pin;
	/** Alternate pin selection */
	bool use_alternate_pin;
	/** GPIO voltage selection */
	enum max_capi_gpio_vssel vssel;
	/** GPIO already initialized flag */
	bool gpio_initialized;
	/** Channel-specific event callback */
	capi_timer_channel_callback callback;
	/** Event callback arg */
	void *callback_arg;
	/** Channel event enable mask */
	uint32_t events_enabled;
	/** Timer enabled */
	bool enabled;
};

struct max_capi_timer_priv {
	/** Timer ID */
	uint32_t identifier;
	/** Timer frequency */
	uint32_t frequency;
	/** 32-bit or dual 16-bit mode */
	enum max_capi_timer_bit_mode bit_mode;
	/** Timer A/B state */
	struct max_capi_timer_channel_state *channel[2];
	/** Global timer event callback */
	capi_timer_event_callback global_callback;
	/** Global timer event callback arg */
	void *global_callback_arg;
	/** Bitmask for enabled global events */
	uint32_t global_events_enabled;
	/** Clock source */
	enum max_capi_timer_clock_source clock_source;
	/** Clock source frequency */
	uint32_t clock_freq_hz;
	/** MSDK clock storage */
	mxc_tmr_clock_t msdk_clock;
};

extern struct capi_timer_ops max_capi_timer_ops;

#endif /* MAXIM_CAPI_TIMER_H_ */
