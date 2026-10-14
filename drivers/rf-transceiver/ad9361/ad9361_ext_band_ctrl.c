/*
 * Copyright (c) 2025-2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#include <stdint.h>
#include <stdlib.h>
#include <string.h>
#include <errno.h>
#include <inttypes.h>
#include "ad9361.h"
#include "ad9361_util.h"
#include "ad9361_ext_band_ctrl.h"
#include "no_os_gpio.h"
#include "no_os_delay.h"
#include "no_os_alloc.h"
#include "no_os_spi.h"

#define AD9361_BAND_PHASE_PRE        0u
#define AD9361_BAND_PHASE_GPIO       1u
#define AD9361_BAND_PHASE_RX_PORT    2u
#define AD9361_BAND_PHASE_TX_PORT    3u
#define AD9361_BAND_PHASE_POST       4u
#define AD9361_BAND_PHASE_WAIT       5u

static void band_cancel_active(struct ad9361_ext_band_ctl *ctl);

/**
 * @brief Grow a settings array by one element.
 * @param old_arr  - Existing array (freed on success; may be NULL).
 * @param old_count - Number of entries in old_arr.
 * @param new_elem - New element to append.
 * @return Pointer to the new array, or NULL on allocation failure.
 */
static struct ad9361_band_setting **grow_settings_array(
	struct ad9361_band_setting **old_arr,
	uint32_t old_count,
	struct ad9361_band_setting *new_elem)
{
	struct ad9361_band_setting **new_arr;
	uint32_t i;

	new_arr = (struct ad9361_band_setting **)
		  no_os_calloc(old_count + 1, sizeof(*new_arr));
	if (!new_arr)
		return NULL;

	for (i = 0; i < old_count; i++)
		new_arr[i] = old_arr[i];

	new_arr[old_count] = new_elem;

	no_os_free(old_arr);
	return new_arr;
}

static bool band_setting_is_registered(const struct ad9361_ext_band_ctl *ctl,
		const struct ad9361_band_setting *setting)
{
	uint32_t i;

	for (i = 0; i < ctl->rx_count; i++)
		if (ctl->rx_settings[i] == setting)
			return true;
	for (i = 0; i < ctl->tx_count; i++)
		if (ctl->tx_settings[i] == setting)
			return true;
	return false;
}

/**
 * @brief Allocate a band setting descriptor.
 * @param name     - Human-readable name (copied; may be NULL).
 * @param freq_min - Inclusive lower bound of the frequency range (Hz).
 * @param freq_max - Exclusive upper bound of the frequency range (Hz).
 * @param ngpios   - Number of GPIO lines this setting may drive (0 is valid).
 * @return Pointer to the new setting, or NULL on allocation failure.
 */
struct ad9361_band_setting *
ad9361_band_setting_alloc(const char *name, uint64_t freq_min,
			  uint64_t freq_max, uint32_t ngpios)
{
	struct ad9361_band_setting *s;
	uint32_t i;

	s = (struct ad9361_band_setting *)
	    no_os_calloc(1, sizeof(*s));
	if (!s)
		return NULL;

	if (name) {
		char *name_copy = (char *)no_os_calloc(1, strlen(name) + 1);
		if (!name_copy) {
			no_os_free(s);
			return NULL;
		}
		strcpy(name_copy, name);
		s->name = name_copy;
	}

	s->freq_min         = freq_min;
	s->freq_max         = freq_max;
	s->rf_rx_input_sel  = AD9361_EXT_BAND_NO_PORT_SEL;
	s->rf_tx_output_sel = AD9361_EXT_BAND_NO_PORT_SEL;

	if (ngpios > 0) {
		s->gpio_values = (uint32_t *)
				 no_os_calloc(ngpios, sizeof(uint32_t));
		if (!s->gpio_values) {
			no_os_free((void *)s->name);
			no_os_free(s);
			return NULL;
		}
		for (i = 0; i < ngpios; i++)
			s->gpio_values[i] = AD9361_EXT_BAND_CTL_OP_NOP;
		s->ngpios = ngpios;
	}

	return s;
}

static void band_setting_release_members(
		struct ad9361_band_setting *setting,
		const struct ad9361_band_setting **ancestors, uint32_t depth)
{
	uint32_t i;

	if (!setting || depth >= AD9361_EXT_BAND_MAX_SEQ_DEPTH)
		return;
	for (i = 0; i < depth; i++)
		if (ancestors[i] == setting)
			return;
	ancestors[depth] = setting;
	no_os_free((void *)setting->name);
	setting->name = NULL;
	no_os_free(setting->gpio_values);
	setting->gpio_values = NULL;

	if (setting->pre_seq) {
		for (i = 0; i < setting->pre_seq_len; i++)
			band_setting_release_members(&setting->pre_seq[i],
						     ancestors, depth + 1U);
		no_os_free(setting->pre_seq);
		setting->pre_seq = NULL;
		setting->pre_seq_len = 0;
	}
	if (setting->post_seq) {
		for (i = 0; i < setting->post_seq_len; i++)
			band_setting_release_members(&setting->post_seq[i],
						     ancestors, depth + 1U);
		no_os_free(setting->post_seq);
		setting->post_seq = NULL;
		setting->post_seq_len = 0;
	}
}

/**
 * @brief Release a band setting and its owned nested sequences.
 * @param s - Band setting to free (may be NULL).
 */
void ad9361_band_setting_free(struct ad9361_band_setting *s)
{
	const struct ad9361_band_setting *ancestors[
		AD9361_EXT_BAND_MAX_SEQ_DEPTH] = { NULL };

	if (!s)
		return;
	band_setting_release_members(s, ancestors, 0);
	no_os_free(s);
}

/**
 * @brief Set the operation for one GPIO pin.
 * @param s        - The band setting.
 * @param gpio_idx - Index in the controller's gpios[] array (0-based).
 * @param op       - AD9361_EXT_BAND_CTL_OP_LOW/HIGH/INPUT/NOP.
 * @return 0 on success, -EINVAL if gpio_idx is out of range.
 */
int ad9361_band_setting_set_gpio(struct ad9361_band_setting *s,
				 uint32_t gpio_idx, uint32_t op)
{
	if (!s || !s->gpio_values || gpio_idx >= s->ngpios)
		return -EINVAL;
	s->gpio_values[gpio_idx] = op;
	return 0;
}

/**
 * @brief Set the RX RF port for this band.
 * @param s    - The band setting.
 * @param port - RX RF input port index.
 */
void ad9361_band_setting_set_rx_port(struct ad9361_band_setting *s,
				     uint32_t port)
{
	if (s)
		s->rf_rx_input_sel = port;
}

/**
 * @brief Set the TX RF port for this band.
 * @param s    - The band setting.
 * @param port - TX RF output port index.
 */
void ad9361_band_setting_set_tx_port(struct ad9361_band_setting *s,
				     uint32_t port)
{
	if (s)
		s->rf_tx_output_sel = port;
}

/**
 * @brief Set the post-step delay (microseconds).
 * @param s        - The band setting.
 * @param delay_us - Delay in microseconds (0 = no delay).
 */
void ad9361_band_setting_set_delay(struct ad9361_band_setting *s,
				   uint32_t delay_us)
{
	if (s)
		s->delay_us = delay_us;
}

/**
 * @brief Allocate an external band controller.
 * @return Pointer to the zeroed controller, or NULL on failure.
 */
struct ad9361_ext_band_ctl *ad9361_ext_band_ctl_alloc(void)
{
	return (struct ad9361_ext_band_ctl *)
	       no_os_calloc(1, sizeof(struct ad9361_ext_band_ctl));
}

/**
 * @brief Release a controller and all registered settings.
 *        GPIO descriptors are NOT freed -- the caller owns them.
 * @param ctl - Controller to free (may be NULL).
 */
void ad9361_ext_band_ctl_free(struct ad9361_ext_band_ctl *ctl)
{
	uint32_t i;

	if (!ctl)
		return;
	band_cancel_active(ctl);
	if (ctl->registered_phy && ctl->registered_phy->ext_band_ctl == ctl)
		ctl->registered_phy->ext_band_ctl = NULL;

	for (i = 0; i < ctl->rx_count; i++)
		ad9361_band_setting_free(ctl->rx_settings[i]);

	for (i = 0; i < ctl->tx_count; i++)
		ad9361_band_setting_free(ctl->tx_settings[i]);

	no_os_free(ctl->rx_settings);
	no_os_free(ctl->tx_settings);
	no_os_free(ctl);
}

/**
 * @brief Register a GPIO with the controller.
 *        GPIOs are referenced by add order (0 = first added).
 * @param ctl  - The controller.
 * @param gpio - A configured GPIO descriptor.
 * @return 0 on success, -EINVAL or -ENOMEM on error.
 */
int ad9361_ext_band_ctl_add_gpio(struct ad9361_ext_band_ctl *ctl,
				 struct no_os_gpio_desc *gpio)
{
	if (!ctl || !gpio || ctl->active_transaction)
		return -EINVAL;

	if (ctl->ngpios >= AD9361_EXT_BAND_MAX_GPIOS)
		return -ENOMEM;

	ctl->gpios[ctl->ngpios] = gpio;
	ctl->ngpios++;
	return 0;
}

/**
 * @brief Append an RX band setting.
 *        Settings are matched first-to-last; the first frequency match wins.
 * @param ctl - The controller.
 * @param s   - Band setting to add.
 * @return 0 on success, -EINVAL or -ENOMEM on error.
 */
int ad9361_ext_band_ctl_add_rx_setting(struct ad9361_ext_band_ctl *ctl,
				       struct ad9361_band_setting *s)
{
	struct ad9361_band_setting **new_arr;

	if (!ctl || !s || ctl->active_transaction ||
	    band_setting_is_registered(ctl, s))
		return -EINVAL;
	if (ctl->rx_count >= AD9361_EXT_BAND_MAX_SETTINGS)
		return -ENOMEM;

	new_arr = grow_settings_array(ctl->rx_settings, ctl->rx_count, s);
	if (!new_arr)
		return -ENOMEM;

	ctl->rx_settings = new_arr;
	ctl->rx_count++;
	return 0;
}

/**
 * @brief Append a TX band setting.
 *        Settings are matched first-to-last; the first frequency match wins.
 * @param ctl - The controller.
 * @param s   - Band setting to add.
 * @return 0 on success, -EINVAL or -ENOMEM on error.
 */
int ad9361_ext_band_ctl_add_tx_setting(struct ad9361_ext_band_ctl *ctl,
				       struct ad9361_band_setting *s)
{
	struct ad9361_band_setting **new_arr;

	if (!ctl || !s || ctl->active_transaction ||
	    band_setting_is_registered(ctl, s))
		return -EINVAL;
	if (ctl->tx_count >= AD9361_EXT_BAND_MAX_SETTINGS)
		return -ENOMEM;

	new_arr = grow_settings_array(ctl->tx_settings, ctl->tx_count, s);
	if (!new_arr)
		return -ENOMEM;

	ctl->tx_settings = new_arr;
	ctl->tx_count++;
	return 0;
}

/**
 * @brief Attach a controller to a PHY.
 * @param phy - AD9361 PHY state.
 * @param ctl - External band controller to register.
 * @return 0 on success, -EINVAL if either argument is NULL.
 */
int32_t ad9361_register_ext_band_control(struct ad9361_rf_phy *phy,
		struct ad9361_ext_band_ctl *ctl)
{
	if (!phy || !ctl)
		return -EINVAL;
	if (phy->ext_band_transaction || ctl->active_transaction ||
	    (phy->ext_band_ctl && phy->ext_band_ctl != ctl) ||
	    (ctl->registered_phy && ctl->registered_phy != phy))
		return -EBUSY;

	phy->ext_band_ctl = ctl;
	ctl->registered_phy = phy;

	dev_dbg(&phy->spi->dev,
		"%s: registered (%u RX / %u TX settings, %d GPIOs)",
		__func__,
		(unsigned)ctl->rx_count,
		(unsigned)ctl->tx_count,
		ctl->ngpios);

	return 0;
}

/**
 * @brief Detach the controller from a PHY. Does not free anything.
 * @param phy - AD9361 PHY state.
 */
void ad9361_unregister_ext_band_control(struct ad9361_rf_phy *phy)
{
	if (phy) {
		band_cancel_active(phy->ext_band_ctl);
		if (phy->ext_band_ctl && phy->ext_band_ctl->registered_phy == phy)
			phy->ext_band_ctl->registered_phy = NULL;
		phy->ext_band_ctl = NULL;
	}
}

static uint64_t band_time_us(void)
{
	struct no_os_time now = no_os_get_time();

	return (uint64_t)now.s * 1000000ULL + now.us;
}

static void band_cancel_active(struct ad9361_ext_band_ctl *ctl)
{
	struct ad9361_ext_band_transaction *transaction;
	struct ad9361_rf_phy *phy;

	if (!ctl || !ctl->active_transaction)
		return;
	transaction = ctl->active_transaction;
	phy = transaction->phy;
	if (transaction->mutation_started) {
		if (transaction->tx)
			ctl->current_tx_setting = NULL;
		else
			ctl->current_rx_setting = NULL;
	}
	if (phy && phy->ext_band_transaction == transaction)
		phy->ext_band_transaction = NULL;
	transaction->ctl = NULL;
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = -ECANCELED;
	ctl->active_transaction = NULL;
}

static int32_t band_validate_action(const struct ad9361_ext_band_ctl *ctl,
		const struct ad9361_band_setting *setting)
{
	uint32_t i;

	if (!setting || (setting->pre_seq_len && !setting->pre_seq) ||
	    (setting->post_seq_len && !setting->post_seq) ||
	    setting->ngpios > AD9361_EXT_BAND_MAX_GPIOS)
		return -EINVAL;
	if (setting->gpio_values && setting->ngpios < (uint32_t)ctl->ngpios)
		return -EINVAL;
	if (setting->rf_rx_input_sel != AD9361_EXT_BAND_NO_PORT_SEL &&
	    setting->rf_rx_input_sel > 11)
		return -EINVAL;
	if (setting->rf_tx_output_sel != AD9361_EXT_BAND_NO_PORT_SEL &&
	    setting->rf_tx_output_sel > 1)
		return -EINVAL;
	if (setting->gpio_values) {
		for (i = 0; i < (uint32_t)ctl->ngpios; i++) {
			uint32_t op = setting->gpio_values[i];

			if (op > AD9361_EXT_BAND_CTL_OP_NOP ||
			    (op != AD9361_EXT_BAND_CTL_OP_NOP && !ctl->gpios[i]))
				return -EINVAL;
		}
	}
	return 0;
}

/* Iterative preflight bounds both recursion depth and total work, and rejects
 * cycles before the first external GPIO or RF-port mutation. */
static int32_t band_validate_graph(const struct ad9361_ext_band_ctl *ctl,
		const struct ad9361_band_setting *root)
{
	struct ad9361_ext_band_transaction_frame stack[
		AD9361_EXT_BAND_MAX_SEQ_DEPTH];
	uint16_t depth = 1, nodes = 0, i;

	memset(stack, 0, sizeof(stack));
	stack[0].setting = root;
	while (depth) {
		struct ad9361_ext_band_transaction_frame *frame = &stack[depth - 1U];
		const struct ad9361_band_setting *child = NULL;

		if (frame->phase == 0) {
			int32_t ret;

			if (++nodes > AD9361_EXT_BAND_MAX_SEQ_NODES)
				return -E2BIG;
			ret = band_validate_action(ctl, frame->setting);
			if (ret < 0)
				return ret;
			frame->phase = 1;
			frame->index = 0;
		}
		if (frame->phase == 1) {
			if (frame->index < frame->setting->pre_seq_len)
				child = &frame->setting->pre_seq[frame->index++];
			else {
				frame->phase = 2;
				frame->index = 0;
			}
		} else if (frame->phase == 2) {
			frame->phase = 3;
			frame->index = 0;
		} else if (frame->phase == 3) {
			if (frame->index < frame->setting->post_seq_len)
				child = &frame->setting->post_seq[frame->index++];
			else
				frame->phase = 4;
		} else {
			depth--;
			continue;
		}

		if (child) {
			if (depth >= AD9361_EXT_BAND_MAX_SEQ_DEPTH)
				return -E2BIG;
			for (i = 0; i < depth; i++)
				if (stack[i].setting == child)
					return -ELOOP;
			memset(&stack[depth], 0, sizeof(stack[depth]));
			stack[depth++].setting = child;
		}
	}
	return 0;
}

static int32_t band_find_target(struct ad9361_ext_band_ctl *ctl, bool tx,
		uint64_t freq, struct ad9361_band_setting **target)
{
	struct ad9361_band_setting **list = tx ? ctl->tx_settings : ctl->rx_settings;
	uint32_t count = tx ? ctl->tx_count : ctl->rx_count;
	uint32_t i;

	*target = NULL;
	if (count > AD9361_EXT_BAND_MAX_SETTINGS || (count && !list))
		return -EINVAL;
	for (i = 0; i < count; i++) {
		if (!list[i] || list[i]->freq_min >= list[i]->freq_max)
			return -EINVAL;
		if (!*target && list[i]->freq_min <= freq && freq < list[i]->freq_max)
			*target = list[i];
	}
	return 0;
}

static int32_t band_finish(struct ad9361_ext_band_transaction *transaction,
		int32_t status)
{
	struct ad9361_rf_phy *phy = transaction->phy;
	struct ad9361_ext_band_ctl *ctl = transaction->ctl;

	if (status < 0 && transaction->mutation_started && ctl) {
		if (transaction->tx)
			ctl->current_tx_setting = NULL;
		else
			ctl->current_rx_setting = NULL;
		/* The external path is ambiguous, but this alone does not prove that
		 * AD9361 register/calibration state is corrupt. Keep the setting cache
		 * invalid and return a terminal failure; the owning RF/data-path layer
		 * must fence IQ until a complete band transaction succeeds. */
	}
	if (phy && phy->ext_band_transaction == transaction)
		phy->ext_band_transaction = NULL;
	if (ctl && ctl->active_transaction == transaction)
		ctl->active_transaction = NULL;
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = status;
	return status;
}

int32_t ad9361_ext_band_transaction_start(struct ad9361_rf_phy *phy,
		bool tx, uint64_t freq,
	struct ad9361_ext_band_transaction *transaction)
{
	struct ad9361_ext_band_ctl *ctl;
	struct ad9361_band_setting *target, *current;
	int32_t ret;

	if (!phy || !transaction)
		return -EINVAL;
	if (transaction->active)
		return -EBUSY;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	memset(transaction, 0, sizeof(*transaction));
	transaction->phy = phy;
	transaction->tx = tx;
	transaction->frequency_hz = freq;
	ctl = phy->ext_band_ctl;
	if (!ctl) {
		transaction->terminal = true;
		return 0;
	}
	if (ctl->registered_phy != phy ||
	    ctl->ngpios < 0 || ctl->ngpios > AD9361_EXT_BAND_MAX_GPIOS) {
		transaction->terminal = true;
		transaction->terminal_status = -EINVAL;
		return -EINVAL;
	}
	if (ctl->active_transaction) {
		transaction->terminal = true;
		transaction->terminal_status = -EBUSY;
		return -EBUSY;
	}
	ret = band_find_target(ctl, tx, freq, &target);
	if (ret < 0) {
		transaction->terminal = true;
		transaction->terminal_status = ret;
		return ret;
	}
	transaction->ctl = ctl;
	transaction->target_setting = target;
	current = tx ? ctl->current_tx_setting : ctl->current_rx_setting;
	if (!target || target == current) {
		transaction->terminal = true;
		return 0;
	}
	if (current) {
		ret = band_validate_action(ctl, current);
		if (ret < 0) {
			transaction->terminal = true;
			transaction->terminal_status = ret;
			return ret;
		}
	}
	ret = band_validate_graph(ctl, target);
	if (ret < 0) {
		transaction->terminal = true;
		transaction->terminal_status = ret;
		return ret;
	}
	transaction->stack[0].setting = target;
	transaction->stack[0].current_setting = current;
	transaction->stack[0].phase = AD9361_BAND_PHASE_PRE;
	transaction->depth = 1;
	transaction->active = true;
	ctl->active_transaction = transaction;
	phy->ext_band_transaction = transaction;
	return 0;
}

int32_t ad9361_ext_band_transaction_poll(struct ad9361_rf_phy *phy,
		struct ad9361_ext_band_transaction *transaction)
{
	struct ad9361_ext_band_transaction_frame *frame;
	struct ad9361_band_setting *setting;
	int32_t ret;

	if (!phy || !transaction)
		return -EINVAL;
	if (transaction->terminal)
		return transaction->phy == phy ? transaction->terminal_status : -EINVAL;
	if (!transaction->active || transaction->phy != phy || !transaction->ctl ||
	    phy->ext_band_transaction != transaction ||
	    transaction->ctl->active_transaction != transaction) {
		return band_finish(transaction, -EIO);
	}
	if (ad9361_calibration_state_unknown(phy))
		return band_finish(transaction, -EIO);

	for (;;) {
		if (!transaction->depth)
			return band_finish(transaction, -EIO);
		frame = &transaction->stack[transaction->depth - 1U];
		setting = (struct ad9361_band_setting *)frame->setting;
		switch (frame->phase) {
		case AD9361_BAND_PHASE_PRE:
			if (frame->index < setting->pre_seq_len) {
				const struct ad9361_band_setting *child =
					&setting->pre_seq[frame->index++];

				if (transaction->depth >= AD9361_EXT_BAND_MAX_SEQ_DEPTH)
					return band_finish(transaction, -E2BIG);
				frame->resume_phase = AD9361_BAND_PHASE_PRE;
				frame->phase = AD9361_BAND_PHASE_WAIT;
				frame->delay_deadline_us = child->delay_us;
				frame->gpio_index = UINT32_MAX;
				frame = &transaction->stack[transaction->depth++];
				memset(frame, 0, sizeof(*frame));
				frame->setting = child;
				frame->phase = AD9361_BAND_PHASE_PRE;
				continue;
			}
			frame->phase = AD9361_BAND_PHASE_GPIO;
			frame->gpio_index = 0;
			continue;
		case AD9361_BAND_PHASE_GPIO:
			if (setting->gpio_values &&
			    frame->gpio_index < (uint32_t)transaction->ctl->ngpios) {
				uint32_t index = frame->gpio_index++;
				uint32_t new_op = setting->gpio_values[index];
				uint32_t old_op = frame->current_setting &&
					frame->current_setting->gpio_values ?
					frame->current_setting->gpio_values[index] :
					AD9361_EXT_BAND_CTL_OP_NOP;

				if (new_op == AD9361_EXT_BAND_CTL_OP_NOP ||
				    (frame->current_setting && new_op == old_op))
					continue;
				transaction->mutation_started = true;
				switch (new_op) {
				case AD9361_EXT_BAND_CTL_OP_LOW:
					ret = no_os_gpio_direction_output(
						transaction->ctl->gpios[index], 0);
					break;
				case AD9361_EXT_BAND_CTL_OP_HIGH:
					ret = no_os_gpio_direction_output(
						transaction->ctl->gpios[index], 1);
					break;
				case AD9361_EXT_BAND_CTL_OP_INPUT:
					ret = no_os_gpio_direction_input(
						transaction->ctl->gpios[index]);
					break;
				default:
					return band_finish(transaction, -EINVAL);
				}
				return ret < 0 ? band_finish(transaction, ret) : 1;
			}
			frame->phase = AD9361_BAND_PHASE_RX_PORT;
			continue;
		case AD9361_BAND_PHASE_RX_PORT:
			if (setting->rf_rx_input_sel != AD9361_EXT_BAND_NO_PORT_SEL &&
			    (!frame->current_setting ||
			     frame->current_setting->rf_rx_input_sel !=
				setting->rf_rx_input_sel)) {
				transaction->mutation_started = true;
				phy->ext_band_transaction = NULL;
				ret = ad9361_rf_port_setup(phy, false,
					setting->rf_rx_input_sel, 0);
				phy->ext_band_transaction = transaction;
				if (ret < 0)
					return band_finish(transaction, ret);
				return 1;
			}
			frame->phase = AD9361_BAND_PHASE_TX_PORT;
			continue;
		case AD9361_BAND_PHASE_TX_PORT:
			if (setting->rf_tx_output_sel != AD9361_EXT_BAND_NO_PORT_SEL &&
			    (!frame->current_setting ||
			     frame->current_setting->rf_tx_output_sel !=
				setting->rf_tx_output_sel)) {
				transaction->mutation_started = true;
				phy->ext_band_transaction = NULL;
				ret = ad9361_rf_port_setup(phy, true, 0,
					setting->rf_tx_output_sel);
				phy->ext_band_transaction = transaction;
				if (ret < 0)
					return band_finish(transaction, ret);
				return 1;
			}
			frame->phase = AD9361_BAND_PHASE_POST;
			frame->index = 0;
			continue;
		case AD9361_BAND_PHASE_POST:
			if (frame->index < setting->post_seq_len) {
				const struct ad9361_band_setting *child =
					&setting->post_seq[frame->index++];

				if (transaction->depth >= AD9361_EXT_BAND_MAX_SEQ_DEPTH)
					return band_finish(transaction, -E2BIG);
				frame->resume_phase = AD9361_BAND_PHASE_POST;
				frame->phase = AD9361_BAND_PHASE_WAIT;
				frame->delay_deadline_us = child->delay_us;
				frame = &transaction->stack[transaction->depth++];
				memset(frame, 0, sizeof(*frame));
				frame->setting = child;
				frame->phase = AD9361_BAND_PHASE_PRE;
				continue;
			}
			transaction->depth--;
			if (!transaction->depth) {
				if (transaction->tx)
					transaction->ctl->current_tx_setting =
						transaction->target_setting;
				else
					transaction->ctl->current_rx_setting =
						transaction->target_setting;
				return band_finish(transaction, 0);
			}
			frame = &transaction->stack[transaction->depth - 1U];
			if (frame->delay_deadline_us) {
				uint32_t delay_us = (uint32_t)frame->delay_deadline_us;

				frame->delay_deadline_us = band_time_us() + delay_us;
				frame->phase = AD9361_BAND_PHASE_WAIT;
				return 1;
			}
			frame->phase = frame->resume_phase;
			continue;
		case AD9361_BAND_PHASE_WAIT:
			if (band_time_us() < frame->delay_deadline_us)
				return 1;
			frame->delay_deadline_us = 0;
			frame->phase = frame->resume_phase;
			return 1;
		default:
			return band_finish(transaction, -EIO);
		}
	}
}

uint32_t ad9361_ext_band_transaction_poll_delay_us(
		const struct ad9361_ext_band_transaction *transaction)
{
	uint64_t now, remaining;
	const struct ad9361_ext_band_transaction_frame *frame;

	if (!transaction || !transaction->active || !transaction->depth)
		return 0;
	frame = &transaction->stack[transaction->depth - 1U];
	if (frame->phase != AD9361_BAND_PHASE_WAIT)
		return 0;
	now = band_time_us();
	remaining = frame->delay_deadline_us > now ?
		frame->delay_deadline_us - now : 0;
	return remaining > UINT32_MAX ? UINT32_MAX : (uint32_t)remaining;
}

static int32_t ad9361_ext_band_transaction_run_sync(
		struct ad9361_rf_phy *phy, bool tx, uint64_t freq)
{
	struct ad9361_ext_band_transaction transaction = {0};
	int32_t ret;

	ret = ad9361_ext_band_transaction_start(phy, tx, freq, &transaction);
	if (ret < 0)
		return ret;
	for (;;) {
		uint32_t delay_us;

		ret = ad9361_ext_band_transaction_poll(phy, &transaction);
		if (ret != 1)
			return ret;
		delay_us = ad9361_ext_band_transaction_poll_delay_us(&transaction);
		if (delay_us)
			no_os_udelay(delay_us);
	}
}

/** Apply an RX band change through the retained sequence engine. */
int32_t ad9361_adjust_rx_ext_band_settings(struct ad9361_rf_phy *phy,
		uint64_t freq)
{
	if (!phy)
		return -EINVAL;
	return ad9361_ext_band_transaction_run_sync(phy, false, freq);
}

/** Apply a TX band change through the retained sequence engine. */
int32_t ad9361_adjust_tx_ext_band_settings(struct ad9361_rf_phy *phy,
		uint64_t freq)
{
	if (!phy)
		return -EINVAL;
	return ad9361_ext_band_transaction_run_sync(phy, true, freq);
}
