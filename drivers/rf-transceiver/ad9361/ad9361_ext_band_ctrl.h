/*
 * Copyright (c) 2025-2026 Analog Devices, Inc.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

#ifndef AD9361_EXT_BAND_CTRL_H_
#define AD9361_EXT_BAND_CTRL_H_

#include <stdint.h>
#include <stdbool.h>
#include "no_os_gpio.h"

/* Forward declaration — full definition in ad9361.h */
struct ad9361_rf_phy;
struct ad9361_ext_band_transaction;
struct ad9361_ext_band_ctl;

/* Constants */

/** Maximum number of GPIO lines managed by one controller instance. */
#define AD9361_EXT_BAND_MAX_GPIOS   32

/** Maximum number of RX or TX band settings per controller. */
#define AD9361_EXT_BAND_MAX_SETTINGS 64

/** Maximum recursion depth accepted in a nested band-setting graph. */
#define AD9361_EXT_BAND_MAX_SEQ_DEPTH 8

/** Maximum expanded actions accepted during preflight validation. */
#define AD9361_EXT_BAND_MAX_SEQ_NODES 1024

/**
 * Sentinel value for rf_rx_input_sel / rf_tx_output_sel meaning
 * "do not change the current port selection".
 */
#define AD9361_EXT_BAND_NO_PORT_SEL  ((uint32_t)-1)

/* GPIO operation codes */

/**
 * Per-GPIO operation to perform when a band setting becomes active.
 * Use these as values in ad9361_band_setting.gpio_values[].
 */
enum ad9361_ext_band_ctl_op {
	/** Drive the pin low (output 0). */
	AD9361_EXT_BAND_CTL_OP_LOW   = 0,
	/** Drive the pin high (output 1). */
	AD9361_EXT_BAND_CTL_OP_HIGH  = 1,
	/** Switch the pin to input (high-Z). */
	AD9361_EXT_BAND_CTL_OP_INPUT = 2,
	/** Leave this pin unchanged. */
	AD9361_EXT_BAND_CTL_OP_NOP   = 3,
};

/* Band setting descriptor */

/**
 * struct ad9361_band_setting - Describes a single frequency-band configuration.
 *
 * When the LO frequency falls within [freq_min, freq_max), the driver will:
 *   1. Execute any pre_seq[] steps in order.
 *   2. Drive GPIOs according to gpio_values[].
 *   3. Switch the RX/TX port if rf_rx_input_sel / rf_tx_output_sel differ
 *      from the current selection.
 *   4. Execute any post_seq[] steps in order.
 *
 * pre_seq/post_seq arrays and their GPIO/name allocations are owned by this
 * setting and must form an exclusive tree (no cycles or shared subtrees).
 * ad9361_band_setting_free() releases that tree recursively.
 */
struct ad9361_band_setting {
	/** Human-readable identifier (optional, used for debug messages). */
	const char *name;

	/** Inclusive lower bound of the frequency range (Hz). */
	uint64_t freq_min;
	/** Exclusive upper bound of the frequency range (Hz). */
	uint64_t freq_max;

	/**
	 * GPIO operations for each pin, indexed by GPIO index in the
	 * controller's gpios[] array.  Must have ngpios entries when non-NULL.
	 * Use AD9361_EXT_BAND_CTL_OP_NOP for pins that should not change.
	 */
	uint32_t *gpio_values;
	/** Number of valid entries in gpio_values[]. */
	uint32_t  ngpios;

	/**
	 * RX RF input port to select.
	 * Set to AD9361_EXT_BAND_NO_PORT_SEL to leave port unchanged.
	 */
	uint32_t rf_rx_input_sel;

	/**
	 * TX RF output port to select.
	 * Set to AD9361_EXT_BAND_NO_PORT_SEL to leave port unchanged.
	 */
	uint32_t rf_tx_output_sel;

	/**
	 * Microsecond delay applied after this step when used inside a
	 * pre_seq[] or post_seq[] sequence.  0 = no delay.
	 */
	uint32_t delay_us;

	/** Optional sequenced steps applied before the main setting (may be NULL). */
	struct ad9361_band_setting *pre_seq;
	uint32_t pre_seq_len;

	/** Optional sequenced steps applied after the main setting (may be NULL). */
	struct ad9361_band_setting *post_seq;
	uint32_t post_seq_len;
};

struct ad9361_ext_band_transaction_frame {
	const struct ad9361_band_setting *setting;
	const struct ad9361_band_setting *current_setting;
	uint64_t delay_deadline_us;
	uint32_t index;
	uint32_t gpio_index;
	uint8_t phase;
	uint8_t resume_phase;
};

/* Retained depth-first band switch. Zero-initialize the context before its
 * first start. The setting tree, controller, GPIO descriptors, and any data
 * they reference must remain alive and immutable until terminal poll. Each
 * poll performs at most one GPIO operation or one RF-port operation; delays
 * use a monotonic deadline. */
struct ad9361_ext_band_transaction {
	struct ad9361_rf_phy *phy;
	struct ad9361_ext_band_ctl *ctl;
	struct ad9361_band_setting *target_setting;
	struct ad9361_ext_band_transaction_frame
		stack[AD9361_EXT_BAND_MAX_SEQ_DEPTH];
	uint64_t frequency_hz;
	uint16_t depth;
	uint16_t expanded_nodes;
	uint8_t tx;
	bool active;
	bool terminal;
	bool mutation_started;
	int32_t terminal_status;
};

/* Controller descriptor */

/**
 * struct ad9361_ext_band_ctl - External band controller state.
 *
 * Holds the GPIO pool and the RX/TX band-setting tables.
 * Attach to a PHY with ad9361_register_ext_band_control().
 */
struct ad9361_ext_band_ctl {
	/** GPIO descriptors managed by this controller. */
	struct no_os_gpio_desc *gpios[AD9361_EXT_BAND_MAX_GPIOS];
	/** Number of valid entries in gpios[]. */
	int ngpios;

	/** Array of pointers to RX band settings (searched first-match). */
	struct ad9361_band_setting **rx_settings;
	uint32_t rx_count;

	/** Array of pointers to TX band settings. */
	struct ad9361_band_setting **tx_settings;
	uint32_t tx_count;

	/** Currently active RX setting (NULL if none applied yet). */
	struct ad9361_band_setting *current_rx_setting;
	/** Currently active TX setting (NULL if none applied yet). */
	struct ad9361_band_setting *current_tx_setting;
	struct ad9361_rf_phy *registered_phy;
	struct ad9361_ext_band_transaction *active_transaction;
};


struct ad9361_band_setting *
ad9361_band_setting_alloc(const char *name, uint64_t freq_min,
			  uint64_t freq_max, uint32_t ngpios);

void ad9361_band_setting_free(struct ad9361_band_setting *s);

int  ad9361_band_setting_set_gpio(struct ad9361_band_setting *s,
				  uint32_t gpio_idx, uint32_t op);

void ad9361_band_setting_set_rx_port(struct ad9361_band_setting *s,
				     uint32_t port);

void ad9361_band_setting_set_tx_port(struct ad9361_band_setting *s,
				     uint32_t port);

void ad9361_band_setting_set_delay(struct ad9361_band_setting *s,
				   uint32_t delay_us);

struct ad9361_ext_band_ctl *ad9361_ext_band_ctl_alloc(void);
void ad9361_ext_band_ctl_free(struct ad9361_ext_band_ctl *ctl);

/* A successfully added setting transfers ownership to the controller; the
 * same descriptor pointer may be registered only once across RX and TX. */
int ad9361_ext_band_ctl_add_gpio(struct ad9361_ext_band_ctl *ctl,
				 struct no_os_gpio_desc *gpio);

int ad9361_ext_band_ctl_add_rx_setting(struct ad9361_ext_band_ctl *ctl,
				       struct ad9361_band_setting *s);

int ad9361_ext_band_ctl_add_tx_setting(struct ad9361_ext_band_ctl *ctl,
				       struct ad9361_band_setting *s);


int32_t ad9361_register_ext_band_control(struct ad9361_rf_phy *phy,
		struct ad9361_ext_band_ctl *ctl);

void ad9361_unregister_ext_band_control(struct ad9361_rf_phy *phy);

int32_t ad9361_adjust_rx_ext_band_settings(struct ad9361_rf_phy *phy,
		uint64_t freq);

int32_t ad9361_adjust_tx_ext_band_settings(struct ad9361_rf_phy *phy,
		uint64_t freq);

int32_t ad9361_ext_band_transaction_start(struct ad9361_rf_phy *phy,
		bool tx, uint64_t freq,
	struct ad9361_ext_band_transaction *transaction);
int32_t ad9361_ext_band_transaction_poll(
		struct ad9361_rf_phy *phy,
	struct ad9361_ext_band_transaction *transaction);
uint32_t ad9361_ext_band_transaction_poll_delay_us(
		const struct ad9361_ext_band_transaction *transaction);

#endif /* AD9361_EXT_BAND_CTRL_H_ */
