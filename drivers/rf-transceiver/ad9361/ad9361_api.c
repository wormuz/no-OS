/***************************************************************************//**
 *   @file   ad9361_api.c
 *   @brief  Implementation of AD9361 API Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2013(c) Analog Devices, Inc.
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

#include "ad9361.h"
#include "ad9361_api.h"
#include "no_os_delay.h"
#include "no_os_spi.h"
#include "no_os_util.h"
#include "no_os_alloc.h"
#include "app_config.h"
#include <inttypes.h>
#include <stdio.h>
#include <string.h>

/* NIOS runs the RFIC command path synchronously on its sole control worker.
 * Its reduced JTAG-UART driver spins forever when the 64-byte TX FIFO is
 * full, so diagnostic stdio must never be part of an RF operation's
 * completion path. The NIOS command protocol returns operation errors
 * structurally; keep console diagnostics for host builds only. */
#ifdef BLADERF_NIOS_BUILD
#define AD9361_DIAG_PRINTF(...) ((void)0)
#else
#define AD9361_DIAG_PRINTF(...) printf(__VA_ARGS__)
#endif
#ifndef AXI_ADC_NOT_PRESENT
#include "axi_adc_core.h"
#include "axi_dac_core.h"

static struct axiadc_chip_info axiadc_chip_info_tbl[] = {
	{
		"4_CH_DEV",
		4
	},
	{
		"2_CH_DEV",
		2
	},
};

#define ADI_REG_VERSION			0x0000
#endif

/*
 * The RFPLL lock-status bit is consumed by the bladeRF NIOS fastlock path as
 * the hardware completion event. Configure it explicitly instead of relying
 * on the AD9361 reset value or a bootloader's prior register state. UG-570
 * recommends continuous lock detection and at least 1024 reference-clock
 * cycles; count=2 selects 1024 cycles and mode=2 selects continuous.
 */
/**
 * Initialize the AD9361 part.
 * @param ad9361_phy The AD9361 device structure.
 * @param init_param The structure that contains the AD9361 initial parameters.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
static int32_t ad9361_init_transaction_finish(
		struct ad9361_init_transaction *transaction, int32_t status)
{
	if (status >= 0 && transaction->cleanup_status < 0)
		status = transaction->cleanup_status;
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = status;
	return status;
}

static int32_t ad9361_init_transaction_fail(
		struct ad9361_init_transaction *transaction, int32_t status)
{
	bool consumed = false;
	int32_t remove_status = 0;

	if (status >= 0)
		status = -ENODEV;
	transaction->status = status;
	if (transaction->candidate) {
		remove_status = ad9361_remove_ex(
			transaction->candidate, false, &consumed);
		if (transaction->cleanup_status >= 0 && remove_status < 0)
			transaction->cleanup_status = remove_status;
		if (consumed) {
			transaction->candidate = NULL;
		} else if (transaction->phy_out) {
			/* A quarantined partial PHY remains owned by the board caller until
			 * it asserts RESETB and retries removal. */
			*transaction->phy_out = transaction->candidate;
		}
	}
	if (transaction->cleanup_status < 0)
		AD9361_DIAG_PRINTF("%s : cleanup failed (%d) after init error (%d)\n",
		       __func__, transaction->cleanup_status, status);
	AD9361_DIAG_PRINTF("%s : AD936x initialization error\n", __func__);
	return ad9361_init_transaction_finish(transaction, status);
}

static uint64_t ad9361_init_time_us(void)
{
	struct no_os_time now = no_os_get_time();

	return (uint64_t)now.s * 1000000ULL + now.us;
}

int32_t ad9361_init_transaction_start(
		struct ad9361_init_transaction *transaction,
		struct ad9361_rf_phy **ad9361_phy,
		AD9361_InitParam *init_param)
{
	struct ad9361_rf_phy *phy;
	int32_t ret = 0;
	int32_t i   = 0;

	if (!transaction || !ad9361_phy || !init_param)
		return -EINVAL;
	if (transaction->active || transaction->setup_job.active ||
	    transaction->setup_job.calibration_job.active)
		return -EBUSY;

	memset(transaction, 0, sizeof(*transaction));
	transaction->phy_out = ad9361_phy;
	transaction->init_param = init_param;
	*ad9361_phy = NULL;

	phy = (struct ad9361_rf_phy *)no_os_calloc(1, sizeof(*phy));
	if (!phy) {
		transaction->terminal = true;
		transaction->terminal_status = -ENOMEM;
		return -ENOMEM;
	}

	phy->clk_refin = (struct no_os_clk *)no_os_calloc(1, sizeof(*phy->clk_refin));
	if (!phy->clk_refin) {
		no_os_free(phy);
		transaction->terminal = true;
		transaction->terminal_status = -ENOMEM;
		return -ENOMEM;
	}

	phy->pdata = (struct ad9361_phy_platform_data *)no_os_calloc(1,
			sizeof(*phy->pdata));
	if (!phy->pdata) {
		no_os_free(phy->clk_refin);
		no_os_free(phy);
		transaction->terminal = true;
		transaction->terminal_status = -ENOMEM;
		return -ENOMEM;
	}
#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	phy->adc_conv = (struct axiadc_converter *)no_os_calloc(1,
			sizeof(*phy->adc_conv));
	if (!phy->adc_conv) {
		no_os_free(phy->pdata);
		no_os_free(phy->clk_refin);
		no_os_free(phy);
		transaction->terminal = true;
		transaction->terminal_status = -ENOMEM;
		return -ENOMEM;
	}

	phy->adc_state = (struct axiadc_state *)no_os_calloc(1,
			 sizeof(*phy->adc_state));
	if (!phy->adc_state) {
		no_os_free(phy->adc_conv);
		no_os_free(phy->pdata);
		no_os_free(phy->clk_refin);
		no_os_free(phy);
		transaction->terminal = true;
		transaction->terminal_status = -ENOMEM;
		return -ENOMEM;
	}
	phy->adc_state->phy = phy;
#endif

	transaction->candidate = phy;

	/* Device selection */
	phy->dev_sel = init_param->dev_sel;

	/* Reference Clock */
	phy->clk_refin->rate = init_param->reference_clk_rate;

	/* Base Configuration */
	phy->pdata->fdd = init_param->frequency_division_duplex_mode_enable;
	phy->pdata->fdd_independent_mode =
		init_param->frequency_division_duplex_independent_mode_enable;
	phy->pdata->rx2tx2 = init_param->two_rx_two_tx_mode_enable;
	phy->pdata->rx1tx1_mode_use_rx_num = init_param->one_rx_one_tx_mode_use_rx_num;
	phy->pdata->rx1tx1_mode_use_tx_num = init_param->one_rx_one_tx_mode_use_tx_num;
	phy->pdata->tdd_use_dual_synth = init_param->tdd_use_dual_synth_mode_enable;
	phy->pdata->tdd_skip_vco_cal = init_param->tdd_skip_vco_cal_enable;
	phy->pdata->rx_fastlock_delay_ns = init_param->rx_fastlock_delay_ns;
	phy->pdata->tx_fastlock_delay_ns = init_param->tx_fastlock_delay_ns;
	phy->pdata->trx_fastlock_pinctrl_en[0] =
		init_param->rx_fastlock_pincontrol_enable;
	phy->pdata->trx_fastlock_pinctrl_en[1] =
		init_param->tx_fastlock_pincontrol_enable;
	if (phy->dev_sel == ID_AD9363A) {
		phy->pdata->use_ext_rx_lo = false;
		phy->pdata->use_ext_tx_lo = false;
	} else {
		phy->pdata->use_ext_rx_lo = init_param->external_rx_lo_enable;
		phy->pdata->use_ext_tx_lo = init_param->external_tx_lo_enable;
	}
	phy->pdata->dc_offset_update_events =
		init_param->dc_offset_tracking_update_event_mask;
	phy->pdata->dc_offset_attenuation_high =
		init_param->dc_offset_attenuation_high_range;
	phy->pdata->dc_offset_attenuation_low =
		init_param->dc_offset_attenuation_low_range;
	phy->pdata->rf_dc_offset_count_high = init_param->dc_offset_count_high_range;
	phy->pdata->rf_dc_offset_count_low = init_param->dc_offset_count_low_range;
	phy->pdata->split_gt = init_param->split_gain_table_mode_enable;
	/* RX GM/LNA gain-step calibration is a factory RSSI calibration that
	 * assumes a controlled input. Keep it out of normal device startup unless
	 * the application explicitly opts in; it is also available as a direct
	 * calibration API for factory workflows. */
	phy->pdata->rssi_skip_calib =
		!init_param->rssi_gain_step_calibration_enable;
	phy->pdata->trx_synth_max_fref =
		init_param->trx_synthesizer_target_fref_overwrite_hz;
	phy->pdata->qec_tracking_slow_mode_en =
		init_param->qec_tracking_slow_mode_enable;

	/* ENSM Control */
	phy->pdata->ensm_pin_pulse_mode = init_param->ensm_enable_pin_pulse_mode_enable;
	phy->pdata->ensm_pin_ctrl = init_param->ensm_enable_txnrx_control_enable;

	/* LO Control */
	phy->pdata->rx_synth_freq = init_param->rx_synthesizer_frequency_hz;
	phy->pdata->tx_synth_freq = init_param->tx_synthesizer_frequency_hz;
	phy->pdata->lo_powerdown_managed_en =
		init_param->tx_lo_powerdown_managed_enable;

	/* Rate & BW Control */
	for (i = 0; i < 6; i++) {
		phy->pdata->rx_path_clks[i] = init_param->rx_path_clock_frequencies[i];
	}
	for (i = 0; i < 6; i++) {
		phy->pdata->tx_path_clks[i] = init_param->tx_path_clock_frequencies[i];
	}
	phy->pdata->rf_rx_bandwidth_Hz = init_param->rf_rx_bandwidth_hz;
	phy->pdata->rf_tx_bandwidth_Hz = init_param->rf_tx_bandwidth_hz;

	/* RF Port Control */
	phy->pdata->rf_rx_input_sel = init_param->rx_rf_port_input_select;
	phy->pdata->rf_tx_output_sel = init_param->tx_rf_port_input_select;

	/* TX Attenuation Control */
	phy->pdata->tx_atten = init_param->tx_attenuation_mdB;
	phy->pdata->update_tx_gain_via_alert =
		init_param->update_tx_gain_in_alert_enable;

	/* Reference Clock Control */
	switch (phy->dev_sel) {
	case ID_AD9363A:
		phy->pdata->use_extclk = true;
		break;
	default:
		phy->pdata->use_extclk = init_param->xo_disable_use_ext_refclk_enable;
	}
	phy->pdata->dcxo_coarse = init_param->dcxo_coarse_and_fine_tune[0];
	phy->pdata->dcxo_fine = init_param->dcxo_coarse_and_fine_tune[1];
	phy->pdata->ad9361_clkout_mode = (enum ad9361_clkout)
					 init_param->clk_output_mode_select;

	/* Gain Control */
	phy->pdata->gain_ctrl.rx1_mode = (enum rf_gain_ctrl_mode)
					 init_param->gc_rx1_mode;
	phy->pdata->gain_ctrl.rx2_mode = (enum rf_gain_ctrl_mode)
					 init_param->gc_rx2_mode;
	phy->pdata->gain_ctrl.adc_large_overload_thresh =
		init_param->gc_adc_large_overload_thresh;
	phy->pdata->gain_ctrl.adc_ovr_sample_size = init_param->gc_adc_ovr_sample_size;
	phy->pdata->gain_ctrl.adc_small_overload_thresh =
		init_param->gc_adc_small_overload_thresh;
	phy->pdata->gain_ctrl.dec_pow_measuremnt_duration =
		init_param->gc_dec_pow_measurement_duration;
	phy->pdata->gain_ctrl.dig_gain_en = init_param->gc_dig_gain_enable;
	phy->pdata->gain_ctrl.lmt_overload_high_thresh =
		init_param->gc_lmt_overload_high_thresh;
	phy->pdata->gain_ctrl.lmt_overload_low_thresh =
		init_param->gc_lmt_overload_low_thresh;
	phy->pdata->gain_ctrl.low_power_thresh = init_param->gc_low_power_thresh;
	phy->pdata->gain_ctrl.max_dig_gain = init_param->gc_max_dig_gain;
	phy->pdata->gain_ctrl.use_rx_fir_out_for_dec_pwr_meas =
		init_param->gc_use_rx_fir_out_for_dec_pwr_meas_enable;

	/* Gain MGC Control */
	phy->pdata->gain_ctrl.mgc_dec_gain_step = init_param->mgc_dec_gain_step;
	phy->pdata->gain_ctrl.mgc_inc_gain_step = init_param->mgc_inc_gain_step;
	phy->pdata->gain_ctrl.mgc_rx1_ctrl_inp_en = init_param->mgc_rx1_ctrl_inp_enable;
	phy->pdata->gain_ctrl.mgc_rx2_ctrl_inp_en = init_param->mgc_rx2_ctrl_inp_enable;
	phy->pdata->gain_ctrl.mgc_split_table_ctrl_inp_gain_mode =
		init_param->mgc_split_table_ctrl_inp_gain_mode;

	/* Gain AGC Control */
	phy->pdata->gain_ctrl.adc_large_overload_exceed_counter =
		init_param->agc_adc_large_overload_exceed_counter;
	phy->pdata->gain_ctrl.adc_large_overload_inc_steps =
		init_param->agc_adc_large_overload_inc_steps;
	phy->pdata->gain_ctrl.adc_lmt_small_overload_prevent_gain_inc =
		init_param->agc_adc_lmt_small_overload_prevent_gain_inc_enable;
	phy->pdata->gain_ctrl.adc_small_overload_exceed_counter =
		init_param->agc_adc_small_overload_exceed_counter;
	phy->pdata->gain_ctrl.dig_gain_step_size = init_param->agc_dig_gain_step_size;
	phy->pdata->gain_ctrl.dig_saturation_exceed_counter =
		init_param->agc_dig_saturation_exceed_counter;
	phy->pdata->gain_ctrl.gain_update_interval_us =
		init_param->agc_gain_update_interval_us;
	phy->pdata->gain_ctrl.immed_gain_change_if_large_adc_overload =
		init_param->agc_immed_gain_change_if_large_adc_overload_enable;
	phy->pdata->gain_ctrl.immed_gain_change_if_large_lmt_overload =
		init_param->agc_immed_gain_change_if_large_lmt_overload_enable;
	phy->pdata->gain_ctrl.agc_inner_thresh_high = init_param->agc_inner_thresh_high;
	phy->pdata->gain_ctrl.agc_inner_thresh_high_dec_steps =
		init_param->agc_inner_thresh_high_dec_steps;
	phy->pdata->gain_ctrl.agc_inner_thresh_low = init_param->agc_inner_thresh_low;
	phy->pdata->gain_ctrl.agc_inner_thresh_low_inc_steps =
		init_param->agc_inner_thresh_low_inc_steps;
	phy->pdata->gain_ctrl.lmt_overload_large_exceed_counter =
		init_param->agc_lmt_overload_large_exceed_counter;
	phy->pdata->gain_ctrl.lmt_overload_large_inc_steps =
		init_param->agc_lmt_overload_large_inc_steps;
	phy->pdata->gain_ctrl.lmt_overload_small_exceed_counter =
		init_param->agc_lmt_overload_small_exceed_counter;
	phy->pdata->gain_ctrl.agc_outer_thresh_high = init_param->agc_outer_thresh_high;
	phy->pdata->gain_ctrl.agc_outer_thresh_high_dec_steps =
		init_param->agc_outer_thresh_high_dec_steps;
	phy->pdata->gain_ctrl.agc_outer_thresh_low = init_param->agc_outer_thresh_low;
	phy->pdata->gain_ctrl.agc_outer_thresh_low_inc_steps =
		init_param->agc_outer_thresh_low_inc_steps;
	phy->pdata->gain_ctrl.agc_attack_delay_extra_margin_us =
		init_param->agc_attack_delay_extra_margin_us;
	phy->pdata->gain_ctrl.sync_for_gain_counter_en =
		init_param->agc_sync_for_gain_counter_enable;

	/* Fast AGC */
	phy->pdata->gain_ctrl.f_agc_dec_pow_measuremnt_duration =
		init_param->fagc_dec_pow_measuremnt_duration;
	phy->pdata->gain_ctrl.f_agc_state_wait_time_ns =
		init_param->fagc_state_wait_time_ns;
	/* Fast AGC - Low Power */
	phy->pdata->gain_ctrl.f_agc_allow_agc_gain_increase =
		init_param->fagc_allow_agc_gain_increase;
	phy->pdata->gain_ctrl.f_agc_lp_thresh_increment_time =
		init_param->fagc_lp_thresh_increment_time;
	phy->pdata->gain_ctrl.f_agc_lp_thresh_increment_steps =
		init_param->fagc_lp_thresh_increment_steps;
	/* Fast AGC - Lock Level (Lock Level is set via slow AGC inner high threshold) */
	phy->pdata->gain_ctrl.f_agc_lock_level_lmt_gain_increase_en =
		init_param->fagc_lock_level_lmt_gain_increase_en;
	phy->pdata->gain_ctrl.f_agc_lock_level_gain_increase_upper_limit =
		init_param->fagc_lock_level_gain_increase_upper_limit;
	/* Fast AGC - Peak Detectors and Final Settling */
	phy->pdata->gain_ctrl.f_agc_lpf_final_settling_steps =
		init_param->fagc_lpf_final_settling_steps;
	phy->pdata->gain_ctrl.f_agc_lmt_final_settling_steps =
		init_param->fagc_lmt_final_settling_steps;
	phy->pdata->gain_ctrl.f_agc_final_overrange_count =
		init_param->fagc_final_overrange_count;
	/* Fast AGC - Final Power Test */
	phy->pdata->gain_ctrl.f_agc_gain_increase_after_gain_lock_en =
		init_param->fagc_gain_increase_after_gain_lock_en;
	/* Fast AGC - Unlocking the Gain */
	phy->pdata->gain_ctrl.f_agc_gain_index_type_after_exit_rx_mode =
		(enum f_agc_target_gain_index_type)
		init_param->fagc_gain_index_type_after_exit_rx_mode;
	phy->pdata->gain_ctrl.f_agc_use_last_lock_level_for_set_gain_en =
		init_param->fagc_use_last_lock_level_for_set_gain_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_stronger_sig_thresh_exceeded_en =
		init_param->fagc_rst_gla_stronger_sig_thresh_exceeded_en;
	phy->pdata->gain_ctrl.f_agc_optimized_gain_offset =
		init_param->fagc_optimized_gain_offset;
	phy->pdata->gain_ctrl.f_agc_rst_gla_stronger_sig_thresh_above_ll =
		init_param->fagc_rst_gla_stronger_sig_thresh_above_ll;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_sig_thresh_exceeded_en =
		init_param->fagc_rst_gla_engergy_lost_sig_thresh_exceeded_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_goto_optim_gain_en =
		init_param->fagc_rst_gla_engergy_lost_goto_optim_gain_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_engergy_lost_sig_thresh_below_ll =
		init_param->fagc_rst_gla_engergy_lost_sig_thresh_below_ll;
	phy->pdata->gain_ctrl.f_agc_energy_lost_stronger_sig_gain_lock_exit_cnt =
		init_param->fagc_energy_lost_stronger_sig_gain_lock_exit_cnt;
	phy->pdata->gain_ctrl.f_agc_rst_gla_large_adc_overload_en =
		init_param->fagc_rst_gla_large_adc_overload_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_large_lmt_overload_en =
		init_param->fagc_rst_gla_large_lmt_overload_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_en_agc_pulled_high_en =
		init_param->fagc_rst_gla_en_agc_pulled_high_en;
	phy->pdata->gain_ctrl.f_agc_rst_gla_if_en_agc_pulled_high_mode =
		(enum f_agc_target_gain_index_type)
		init_param->fagc_rst_gla_if_en_agc_pulled_high_mode;
	phy->pdata->gain_ctrl.f_agc_power_measurement_duration_in_state5 =
		init_param->fagc_power_measurement_duration_in_state5;
	phy->pdata->gain_ctrl.f_agc_large_overload_inc_steps =
		init_param->fagc_large_overload_inc_steps;

	/* RSSI Control */
	phy->pdata->rssi_ctrl.rssi_delay = init_param->rssi_delay;
	phy->pdata->rssi_ctrl.rssi_duration = init_param->rssi_duration;
	phy->pdata->rssi_ctrl.restart_mode = (enum rssi_restart_mode)
					     init_param->rssi_restart_mode;
	phy->pdata->rssi_ctrl.rssi_unit_is_rx_samples =
		init_param->rssi_unit_is_rx_samples_enable;
	phy->pdata->rssi_ctrl.rssi_wait = init_param->rssi_wait;

	/* Aux ADC Control */
	phy->pdata->auxadc_ctrl.auxadc_decimation = init_param->aux_adc_decimation;
	phy->pdata->auxadc_ctrl.auxadc_clock_rate = init_param->aux_adc_rate;

	/* AuxDAC Control */
	phy->pdata->auxdac_ctrl.auxdac_manual_mode_en =
		init_param->aux_dac_manual_mode_enable;
	phy->pdata->auxdac_ctrl.dac1_default_value =
		init_param->aux_dac1_default_value_mV;
	phy->pdata->auxdac_ctrl.dac1_in_rx_en =
		init_param->aux_dac1_active_in_rx_enable;
	phy->pdata->auxdac_ctrl.dac1_in_tx_en =
		init_param->aux_dac1_active_in_tx_enable;
	phy->pdata->auxdac_ctrl.dac1_in_alert_en =
		init_param->aux_dac1_active_in_alert_enable;
	phy->pdata->auxdac_ctrl.dac1_rx_delay_us = init_param->aux_dac1_rx_delay_us;
	phy->pdata->auxdac_ctrl.dac1_tx_delay_us = init_param->aux_dac1_tx_delay_us;
	phy->pdata->auxdac_ctrl.dac2_default_value =
		init_param->aux_dac2_default_value_mV;
	phy->pdata->auxdac_ctrl.dac2_in_rx_en =
		init_param->aux_dac2_active_in_rx_enable;
	phy->pdata->auxdac_ctrl.dac2_in_tx_en =
		init_param->aux_dac2_active_in_tx_enable;
	phy->pdata->auxdac_ctrl.dac2_in_alert_en =
		init_param->aux_dac2_active_in_alert_enable;
	phy->pdata->auxdac_ctrl.dac2_rx_delay_us = init_param->aux_dac2_rx_delay_us;
	phy->pdata->auxdac_ctrl.dac2_tx_delay_us = init_param->aux_dac2_tx_delay_us;

	/* Temperature Sensor Control */
	phy->pdata->auxadc_ctrl.temp_sensor_decimation =
		init_param->temp_sense_decimation;
	phy->pdata->auxadc_ctrl.temp_time_inteval_ms =
		init_param->temp_sense_measurement_interval_ms;
	phy->pdata->auxadc_ctrl.offset = init_param->temp_sense_offset_signed;
	phy->pdata->auxadc_ctrl.periodic_temp_measuremnt =
		init_param->temp_sense_periodic_measurement_enable;

	/* Control Out Setup */
	phy->pdata->ctrl_outs_ctrl.en_mask = init_param->ctrl_outs_enable_mask;
	phy->pdata->ctrl_outs_ctrl.index = init_param->ctrl_outs_index;

	/* External LNA Control */
	phy->pdata->elna_ctrl.settling_delay_ns = init_param->elna_settling_delay_ns;
	phy->pdata->elna_ctrl.gain_mdB = init_param->elna_gain_mdB;
	phy->pdata->elna_ctrl.bypass_loss_mdB = init_param->elna_bypass_loss_mdB;
	phy->pdata->elna_ctrl.elna_1_control_en =
		init_param->elna_rx1_gpo0_control_enable;
	phy->pdata->elna_ctrl.elna_2_control_en =
		init_param->elna_rx2_gpo1_control_enable;
	phy->pdata->elna_ctrl.elna_in_gaintable_all_index_en =
		init_param->elna_gaintable_all_index_enable;

	/* Digital Interface Control */
	phy->pdata->dig_interface_tune_skipmode =
		(init_param->digital_interface_tune_skip_mode);
	phy->pdata->dig_interface_tune_fir_disable =
		(init_param->digital_interface_tune_fir_disable);
	phy->pdata->port_ctrl.pp_conf[0] = (init_param->pp_tx_swap_enable << 7);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->pp_rx_swap_enable << 6);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->tx_channel_swap_enable << 5);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->rx_channel_swap_enable << 4);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->rx_frame_pulse_mode_enable <<
					     3);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->two_t_two_r_timing_enable <<
					     2);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->invert_data_bus_enable << 1);
	phy->pdata->port_ctrl.pp_conf[0] |= (init_param->invert_data_clk_enable << 0);
	phy->pdata->port_ctrl.pp_conf[1] = (init_param->fdd_alt_word_order_enable << 7);
	phy->pdata->port_ctrl.pp_conf[1] |= (init_param->invert_rx_frame_enable << 2);
	phy->pdata->port_ctrl.pp_conf[2] = (init_param->fdd_rx_rate_2tx_enable << 7);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->swap_ports_enable << 6);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->single_data_rate_enable << 5);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->lvds_mode_enable << 4);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->half_duplex_mode_enable << 3);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->single_port_mode_enable << 2);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->full_port_enable << 1);
	phy->pdata->port_ctrl.pp_conf[2] |= (init_param->full_duplex_swap_bits_enable <<
					     0);
	phy->pdata->port_ctrl.pp_conf[1] |= (init_param->delay_rx_data & 0x3);
	phy->pdata->port_ctrl.rx_clk_data_delay = DATA_CLK_DELAY(
				init_param->rx_data_clock_delay);
	phy->pdata->port_ctrl.rx_clk_data_delay |= RX_DATA_DELAY(
				init_param->rx_data_delay);
	phy->pdata->port_ctrl.tx_clk_data_delay = FB_CLK_DELAY(
				init_param->tx_fb_clock_delay);
	phy->pdata->port_ctrl.tx_clk_data_delay |= TX_DATA_DELAY(
				init_param->tx_data_delay);
	phy->pdata->port_ctrl.lvds_bias_ctrl = ((init_param->lvds_bias_mV - 75) / 75) &
					       0x7;
	phy->pdata->port_ctrl.lvds_bias_ctrl |=
		(init_param->lvds_rx_onchip_termination_enable << 5);
	phy->pdata->rx1rx2_phase_inversion_en = init_param->rx1rx2_phase_inversion_en;

	/* GPO Control */
	phy->pdata->gpo_ctrl.gpo_manual_mode_en =
		init_param->gpo_manual_mode_enable;
	phy->pdata->gpo_ctrl.gpo_manual_mode_enable_mask =
		init_param->gpo_manual_mode_enable_mask;
	phy->pdata->gpo_ctrl.gpo0_inactive_state_high_en =
		init_param->gpo0_inactive_state_high_enable;
	phy->pdata->gpo_ctrl.gpo1_inactive_state_high_en =
		init_param->gpo1_inactive_state_high_enable;
	phy->pdata->gpo_ctrl.gpo2_inactive_state_high_en =
		init_param->gpo2_inactive_state_high_enable;
	phy->pdata->gpo_ctrl.gpo3_inactive_state_high_en =
		init_param->gpo3_inactive_state_high_enable;

	phy->pdata->gpo_ctrl.gpo0_slave_rx_en = init_param->gpo0_slave_rx_enable;
	phy->pdata->gpo_ctrl.gpo0_slave_tx_en = init_param->gpo0_slave_tx_enable;
	phy->pdata->gpo_ctrl.gpo1_slave_rx_en = init_param->gpo1_slave_rx_enable;
	phy->pdata->gpo_ctrl.gpo1_slave_tx_en = init_param->gpo1_slave_tx_enable;
	phy->pdata->gpo_ctrl.gpo2_slave_rx_en = init_param->gpo2_slave_rx_enable;
	phy->pdata->gpo_ctrl.gpo2_slave_tx_en = init_param->gpo2_slave_tx_enable;
	phy->pdata->gpo_ctrl.gpo3_slave_rx_en = init_param->gpo3_slave_rx_enable;
	phy->pdata->gpo_ctrl.gpo3_slave_tx_en = init_param->gpo3_slave_tx_enable;

	phy->pdata->gpo_ctrl.gpo0_rx_delay_us = init_param->gpo0_rx_delay_us;
	phy->pdata->gpo_ctrl.gpo0_tx_delay_us = init_param->gpo0_tx_delay_us;
	phy->pdata->gpo_ctrl.gpo1_rx_delay_us = init_param->gpo1_rx_delay_us;
	phy->pdata->gpo_ctrl.gpo1_tx_delay_us = init_param->gpo1_tx_delay_us;
	phy->pdata->gpo_ctrl.gpo2_rx_delay_us = init_param->gpo2_rx_delay_us;
	phy->pdata->gpo_ctrl.gpo2_tx_delay_us = init_param->gpo2_tx_delay_us;
	phy->pdata->gpo_ctrl.gpo3_rx_delay_us = init_param->gpo3_rx_delay_us;
	phy->pdata->gpo_ctrl.gpo3_tx_delay_us = init_param->gpo3_tx_delay_us;

	/* Tx Monitor Control */
	phy->pdata->txmon_ctrl.low_high_gain_threshold_mdB =
		init_param->low_high_gain_threshold_mdB;
	phy->pdata->txmon_ctrl.low_gain_dB = init_param->low_gain_dB;
	phy->pdata->txmon_ctrl.high_gain_dB = init_param->high_gain_dB;
	phy->pdata->txmon_ctrl.tx_mon_track_en = init_param->tx_mon_track_en;
	phy->pdata->txmon_ctrl.one_shot_mode_en = init_param->one_shot_mode_en;
	phy->pdata->txmon_ctrl.tx_mon_delay = init_param->tx_mon_delay;
	phy->pdata->txmon_ctrl.tx_mon_duration = init_param->tx_mon_duration;
	phy->pdata->txmon_ctrl.tx1_mon_front_end_gain =
		init_param->tx1_mon_front_end_gain;
	phy->pdata->txmon_ctrl.tx2_mon_front_end_gain =
		init_param->tx2_mon_front_end_gain;
	phy->pdata->txmon_ctrl.tx1_mon_lo_cm = init_param->tx1_mon_lo_cm;
	phy->pdata->txmon_ctrl.tx2_mon_lo_cm = init_param->tx2_mon_lo_cm;

	phy->pdata->debug_mode = true;

	ret = no_os_gpio_get(&phy->gpio_desc_resetb, &init_param->gpio_resetb);
	if (ret < 0)
		goto out;
	/* Optional: next three GPIOs are used for MCS synchronization */
	ret = no_os_gpio_get_optional(&phy->gpio_desc_sync, &init_param->gpio_sync);
	if (ret < 0)
		goto out;
	ret = no_os_gpio_get_optional(&phy->gpio_desc_cal_sw1, &init_param->gpio_cal_sw1);
	if (ret < 0)
		goto out;
	ret = no_os_gpio_get_optional(&phy->gpio_desc_cal_sw2, &init_param->gpio_cal_sw2);
	if (ret < 0)
		goto out;

	ret = no_os_gpio_direction_output(phy->gpio_desc_resetb, 0);
	if (ret < 0)
		goto out;

	if (phy->gpio_desc_sync) {
		ret = no_os_gpio_direction_output(phy->gpio_desc_sync, 0);
		if (ret < 0)
			goto out;
	}

	ret = no_os_spi_init(&phy->spi, &init_param->spi_param);
	if (ret < 0)
		goto out;

	phy->pdata->port_ctrl.digital_io_ctrl = 0;
	phy->pdata->port_ctrl.lvds_invert[0] = init_param->lvds_invert1_control;
	phy->pdata->port_ctrl.lvds_invert[1] = init_param->lvds_invert2_control;
	phy->pdata->port_ctrl.clk_out_drive = init_param->clk_out_drive;
	phy->pdata->port_ctrl.dataclk_drive = init_param->dataclk_drive;
	phy->pdata->port_ctrl.data_port_drive = init_param->data_port_drive;
	phy->pdata->port_ctrl.clk_out_slew = init_param->clk_out_slew;
	phy->pdata->port_ctrl.dataclk_slew = init_param->dataclk_slew;
	phy->pdata->port_ctrl.data_port_slew = init_param->data_port_slew;

	if (AD9364_DEVICE) {
		phy->pdata->rx2tx2 = false;
		phy->pdata->rx1tx1_mode_use_rx_num = 1;
		phy->pdata->rx1tx1_mode_use_tx_num = 1;
	}

#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	phy->adc_conv->chip_info = &axiadc_chip_info_tbl[phy->pdata->rx2tx2 ?
						      ID_AD9361 : ID_AD9364];
#endif

	phy->rx_eq_2tx = false;

	ret = ad9361_init_gain_tables(phy);
	if (ret < 0)
		goto out;
	phy->bypass_tx_fir = true;
	phy->bypass_rx_fir = true;
	phy->rate_governor = 1;
	phy->rfdc_track_en = true;
	phy->bbdc_track_en = true;
	phy->quad_track_en = true;

	phy->bist_loopback_mode = 0;
	phy->bist_loopback_valid = true;
	phy->bist_config = 0;
	phy->bist_config_valid = true;
	phy->bist_prbs_mode = BIST_DISABLE;
	phy->bist_tone_mode = BIST_DISABLE;
	phy->bist_tone_freq_Hz = 0;
	phy->bist_tone_level_dB = 0;
	phy->bist_tone_mask = 0;

	phy->ad9361_rfpll_ext_recalc_rate = init_param->ad9361_rfpll_ext_recalc_rate;
	phy->ad9361_rfpll_ext_round_rate = init_param->ad9361_rfpll_ext_round_rate;
	phy->ad9361_rfpll_ext_set_rate = init_param->ad9361_rfpll_ext_set_rate;

	/* All blocking reset timing and subsequent RFIC setup are advanced by
	 * poll(); do not enter the setup path from start(). */
	transaction->stage = AD9361_INIT_TXN_RESET_ASSERT;
	transaction->active = true;
	return 0;

out:
	return ad9361_init_transaction_fail(transaction, ret);
}

int32_t ad9361_init_transaction_poll(
		struct ad9361_init_transaction *transaction)
{
	static const uint16_t lock_config_regs[] = {
		REG_RX_LOCK_DETECT_CONFIG,
		REG_TX_LOCK_DETECT_CONFIG,
	};
	static const uint8_t lock_config_mask = 0x0f;
	static const uint8_t lock_config_required = 0x0a;
	struct ad9361_rf_phy *phy;
	AD9361_InitParam *init_param;
	int32_t value, post_setup_cleanup_status;
	int32_t ret;

	if (!transaction)
		return -EINVAL;
	if (transaction->terminal)
		return transaction->terminal_status;
	if (!transaction->active || !transaction->candidate ||
	    !transaction->phy_out || !transaction->init_param)
		return ad9361_init_transaction_fail(transaction, -EIO);
	if (transaction->stage > AD9361_INIT_TXN_PLATFORM_POST_SETUP ||
	    ((transaction->stage == AD9361_INIT_TXN_LOCK_READ ||
	      transaction->stage == AD9361_INIT_TXN_LOCK_WRITE ||
	      transaction->stage == AD9361_INIT_TXN_LOCK_VERIFY) &&
	     transaction->lock_config_index >= NO_OS_ARRAY_SIZE(lock_config_regs)))
		return ad9361_init_transaction_fail(transaction, -EIO);
	phy = transaction->candidate;
	init_param = transaction->init_param;

	switch (transaction->stage) {
	case AD9361_INIT_TXN_RESET_ASSERT:
		if (!phy->gpio_desc_resetb) {
			/* Preserve the legacy soft-reset failure semantics on boards
			 * without RESETB; UG-570 requires a hard reset for init cals. */
			ret = ad9361_reset(phy);
			goto fail;
		}
		ret = no_os_gpio_set_value(phy->gpio_desc_resetb, 0);
		if (ret < 0)
			goto fail;
		transaction->phase_deadline_us = ad9361_init_time_us() + 1000U;
		transaction->stage = AD9361_INIT_TXN_RESET_LOW_WAIT;
		return 1;

	case AD9361_INIT_TXN_RESET_LOW_WAIT:
		if (ad9361_init_time_us() < transaction->phase_deadline_us)
			return 1;
		transaction->stage = AD9361_INIT_TXN_RESET_RELEASE;
		return 1;

	case AD9361_INIT_TXN_RESET_RELEASE:
		ret = no_os_gpio_set_value(phy->gpio_desc_resetb, 1);
		if (ret < 0)
			goto fail;
		transaction->phase_deadline_us = ad9361_init_time_us() + 1000U;
		transaction->stage = AD9361_INIT_TXN_RESET_HIGH_WAIT;
		return 1;

	case AD9361_INIT_TXN_RESET_HIGH_WAIT:
		if (ad9361_init_time_us() < transaction->phase_deadline_us)
			return 1;
		transaction->stage = AD9361_INIT_TXN_PRODUCT_ID;
		return 1;

	case AD9361_INIT_TXN_PRODUCT_ID:
		value = ad9361_spi_read(phy->spi, REG_PRODUCT_ID);
		if (value < 0) {
			ret = value;
			goto fail;
		}
		if ((value & PRODUCT_ID_MASK) != PRODUCT_ID_9361) {
			AD9361_DIAG_PRINTF("%s : Unsupported PRODUCT_ID 0x%X", __func__,
			       (unsigned int)value);
			ret = -ENODEV;
			goto fail;
		}
		transaction->revision = value & REV_MASK;
		transaction->lock_config_index = 0;
		transaction->stage = AD9361_INIT_TXN_LOCK_READ;
		return 1;

	case AD9361_INIT_TXN_LOCK_READ:
		value = ad9361_spi_read(phy->spi,
				lock_config_regs[transaction->lock_config_index]);
		if (value < 0) {
			ret = value;
			goto fail;
		}
		transaction->lock_config_value =
			(value & ~lock_config_mask) | lock_config_required;
		transaction->stage = AD9361_INIT_TXN_LOCK_WRITE;
		return 1;

	case AD9361_INIT_TXN_LOCK_WRITE:
		ret = ad9361_spi_write(phy->spi,
				lock_config_regs[transaction->lock_config_index],
				transaction->lock_config_value);
		if (ret < 0)
			goto fail;
		transaction->stage = AD9361_INIT_TXN_LOCK_VERIFY;
		return 1;

	case AD9361_INIT_TXN_LOCK_VERIFY:
		value = ad9361_spi_read(phy->spi,
				lock_config_regs[transaction->lock_config_index]);
		if (value < 0) {
			ret = value;
			goto fail;
		}
		if ((value & lock_config_mask) != lock_config_required) {
			ret = -EIO;
			goto fail;
		}
		transaction->lock_config_index++;
		transaction->stage =
			transaction->lock_config_index < NO_OS_ARRAY_SIZE(lock_config_regs)
			? AD9361_INIT_TXN_LOCK_READ : AD9361_INIT_TXN_CLOCKS;
		return 1;

	case AD9361_INIT_TXN_CLOCKS:
		phy->ad9361_rfpll_ext_recalc_rate =
			init_param->ad9361_rfpll_ext_recalc_rate;
		phy->ad9361_rfpll_ext_round_rate =
			init_param->ad9361_rfpll_ext_round_rate;
		phy->ad9361_rfpll_ext_set_rate =
			init_param->ad9361_rfpll_ext_set_rate;
		ret = ad9361_register_clocks(phy);
		if (ret < 0)
			goto fail;
		transaction->stage = AD9361_INIT_TXN_SETUP_START;
		return 1;

	case AD9361_INIT_TXN_SETUP_START:
		/* This call still has a large synchronous setup prefix. Keep it a
		 * visible phase until that prefix is decomposed into retained jobs. */
		ret = ad9361_setup_transaction_start(phy, &transaction->setup_job);
		if (ret < 0)
			goto fail;
		transaction->stage = AD9361_INIT_TXN_SETUP_POLL;
		return 1;

	case AD9361_INIT_TXN_SETUP_POLL:
		ret = ad9361_setup_transaction_poll(phy, &transaction->setup_job);
		if (ret == 1)
			return 1;
		if (transaction->cleanup_status >= 0 &&
		    transaction->setup_job.cleanup_status < 0)
			transaction->cleanup_status =
				transaction->setup_job.cleanup_status;
		if (ret < 0)
			goto fail;
		transaction->stage = AD9361_INIT_TXN_PLATFORM_POST_SETUP;
		return 1;

	case AD9361_INIT_TXN_PLATFORM_POST_SETUP:
		break;

	default:
		ret = -EIO;
		goto fail;
	}

#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	ret = axi_adc_init(&phy->rx_adc, init_param->rx_adc_init);
	if (ret < 0)
		goto fail;
	ret = axi_adc_read(phy->rx_adc, ADI_REG_VERSION,
			   &phy->adc_state->pcore_version);
	if (ret < 0)
		goto fail;

	/* The TX half of the same core. init_param carries tx_dac_init and
	 * nothing here ever used it, so the DAC side stayed in whatever state
	 * power-up left it in - data source DDS, not DMA. Everything upstream
	 * of it works: control plane, sync_tx(), USB completions, TX LO. The
	 * submitted samples simply are not what the DAC plays.
	 *
	 * Measured on a bladeRF 2.0 micro before this call was added: a 960 kHz
	 * tone transmitted into a cabled loopback produced nothing in its own
	 * bin or its image, while the TX LO leakage sat at +38 dB throughout.
	 * Swapping I and Q - which must move the tone to the mirror bin if the
	 * payload plays at all - changed nothing either. */
	/* CNTRL_2 (R1_MODE) and RATECNTRL of this half are not set here on
	 * purpose: ad9361_post_setup() below writes both, reaching the DAC
	 * half through the ADC handle at 0x4048 and 0x404c. Setting them here
	 * as the old in-tree dac_init() did only duplicates that and hides
	 * which write is the effective one - verified by reading the core
	 * back after init: cntrl_2 = 0, ratecntrl = 1 on this 2R2T board. */
	if (init_param->tx_dac_init)
		if ((ret = axi_dac_init(&phy->tx_dac, init_param->tx_dac_init)) < 0)
			goto fail;

	/* platform specific wrapper to call ad9361_post_setup() */
	post_setup_cleanup_status = 0;
	ret = ad9361_post_setup_with_cleanup(
		phy, &post_setup_cleanup_status);
	if (transaction->cleanup_status >= 0 &&
	    post_setup_cleanup_status < 0)
		transaction->cleanup_status = post_setup_cleanup_status;
	if (ret < 0)
		goto fail;
#endif
	/* BLADERF_NIOS_BUILD excludes the AXI ADC/DAC initialization block above,
	 * so this PHY has no rx_adc handle. The NIOS sample-rate command must skip
	 * the BIST tuner when preset-delay mode is active; an unconditional call
	 * fails validation before touching AXI. The host's separate PHY and AXI
	 * handles are not shared with this firmware instance. */

	AD9361_DIAG_PRINTF("%s : AD936x Rev %d successfully initialized\n",
	       __func__, (int)transaction->revision);
	*transaction->phy_out = phy;
	transaction->candidate = NULL;
	transaction->status = 0;
	return ad9361_init_transaction_finish(transaction, 0);

fail:
	return ad9361_init_transaction_fail(transaction, ret);
}

uint32_t ad9361_init_transaction_poll_delay_us(
		const struct ad9361_init_transaction *transaction)
{
	if (!transaction || !transaction->active)
		return 0;
	if (transaction->stage == AD9361_INIT_TXN_RESET_LOW_WAIT ||
	    transaction->stage == AD9361_INIT_TXN_RESET_HIGH_WAIT) {
		uint64_t now_us = ad9361_init_time_us();
		uint64_t remaining_us = transaction->phase_deadline_us > now_us
			? transaction->phase_deadline_us - now_us : 0;

		return remaining_us > UINT32_MAX ? UINT32_MAX :
		       (uint32_t)remaining_us;
	}
	if (transaction->stage != AD9361_INIT_TXN_SETUP_POLL)
		return 0;
	return ad9361_setup_transaction_poll_delay_us(
		&transaction->setup_job);
}

int32_t ad9361_init_with_cleanup(struct ad9361_rf_phy **ad9361_phy,
		AD9361_InitParam *init_param, int32_t *cleanup_status)
{
	struct ad9361_init_transaction transaction = {0};
	int32_t ret;

	if (cleanup_status)
		*cleanup_status = 0;

	ret = ad9361_init_transaction_start(
		&transaction, ad9361_phy, init_param);
	if (ret < 0) {
		if (cleanup_status)
			*cleanup_status = transaction.cleanup_status;
		return ret;
	}
	for (;;) {
		uint32_t delay_us;

		ret = ad9361_init_transaction_poll(&transaction);
		if (ret != 1) {
			if (cleanup_status)
				*cleanup_status = transaction.cleanup_status;
			return ret;
		}
		delay_us = ad9361_init_transaction_poll_delay_us(&transaction);
		if (delay_us)
			no_os_udelay(delay_us);
	}
}

int32_t ad9361_init(struct ad9361_rf_phy **ad9361_phy,
		    AD9361_InitParam *init_param)
{
	return ad9361_init_with_cleanup(ad9361_phy, init_param, NULL);
}

/**
 * Free the allocated resources.
 * @param phy The AD9361 current state structure.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_remove_ex(struct ad9361_rf_phy *phy,
		bool board_reset_asserted, bool *consumed)
{
	int32_t ret = 0;
	int32_t cleanup_ret;
	bool saved_unknown = false;
	bool saved_quarantine = false;
	struct ad9361_calibration_wait *saved_calibration_wait = NULL;

	if (!consumed)
		return -EINVAL;
	*consumed = false;
	if (!phy)
		return -EINVAL;

	/* A caller may release a calibration quarantine only after it has
	 * physically asserted board RESETB. Clear just the quarantine fence while
	 * checking for unrelated live continuations; restore it if any remain. */
	if (board_reset_asserted && phy->calibration_wait_quarantined) {
		saved_unknown = phy->calibration_state_unknown;
		saved_quarantine = phy->calibration_wait_quarantined;
		saved_calibration_wait = phy->calibration_wait;
		phy->calibration_state_unknown = false;
		phy->calibration_wait_quarantined = false;
		phy->calibration_wait = NULL;
		if (ad9361_operation_busy(phy)) {
			phy->calibration_state_unknown = saved_unknown;
			phy->calibration_wait_quarantined = saved_quarantine;
			phy->calibration_wait = saved_calibration_wait;
			return -EBUSY;
		}
	} else if (ad9361_operation_busy(phy)) {
		return -EBUSY;
	}

	/* The controller is caller-owned, but its PHY back-pointer must not
	 * survive consumption of the PHY object. Active band work was rejected by
	 * the busy check above, so this only detaches an idle controller. */
	ad9361_unregister_ext_band_control(phy);

	/* From this point on every path frees `phy`, even if a lower-level remove
	 * reports an error. Publish consumption separately from status so callers
	 * never infer object lifetime from an errno that may also come from a
	 * platform cleanup callback. */
	*consumed = true;

	cleanup_ret = ad9361_unregister_clocks(phy);
	if (cleanup_ret < 0 && ret == 0)
		ret = cleanup_ret;
	if (phy->spi) {
		cleanup_ret = no_os_spi_remove(phy->spi);
		if (cleanup_ret < 0 && ret == 0)
			ret = cleanup_ret;
	}
	cleanup_ret = no_os_gpio_remove(phy->gpio_desc_resetb);
	if (cleanup_ret < 0 && ret == 0)
		ret = cleanup_ret;
	cleanup_ret = no_os_gpio_remove(phy->gpio_desc_sync);
	if (cleanup_ret < 0 && ret == 0)
		ret = cleanup_ret;
	cleanup_ret = no_os_gpio_remove(phy->gpio_desc_cal_sw1);
	if (cleanup_ret < 0 && ret == 0)
		ret = cleanup_ret;
	cleanup_ret = no_os_gpio_remove(phy->gpio_desc_cal_sw2);
	if (cleanup_ret < 0 && ret == 0)
		ret = cleanup_ret;
#ifndef AXI_ADC_NOT_PRESENT
	if (phy->rx_adc) {
		cleanup_ret = axi_adc_remove(phy->rx_adc);
		if (cleanup_ret < 0 && ret == 0)
			ret = cleanup_ret;
	}
	if (phy->tx_dac) {
		cleanup_ret = axi_dac_remove(phy->tx_dac);
		if (cleanup_ret < 0 && ret == 0)
			ret = cleanup_ret;
	}
	no_os_free(phy->adc_conv);
	no_os_free(phy->adc_state);
#endif
	no_os_free(phy->clk_refin);
	no_os_free(phy->pdata);
	no_os_free(phy);

	return ret;
}

int32_t ad9361_remove(struct ad9361_rf_phy *phy)
{
	bool consumed;

	return ad9361_remove_ex(phy, false, &consumed);
}

/**
 * Set the Enable State Machine (ENSM) mode.
 * @param phy The AD9361 current state structure.
 * @param mode The ENSM mode.
 * 			   Accepted values:
 * 				ENSM_MODE_TX
 * 				ENSM_MODE_RX
 * 				ENSM_MODE_ALERT
 * 				ENSM_MODE_FDD
 * 				ENSM_MODE_WAIT
 * 				ENSM_MODE_SLEEP
 * 				ENSM_MODE_PINCTRL
 * 				ENSM_MODE_PINCTRL_FDD_INDEP
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_en_state_machine_mode(struct ad9361_rf_phy *phy,
		uint32_t mode)
{
	int32_t ret;
	uint8_t ensm_state;
	bool pinctrl = false;
	bool fdd_independent = false;
	bool old_fdd_independent;

	if (!phy || !phy->pdata || !phy->spi)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	switch (mode) {
	case ENSM_MODE_TX:
		ensm_state = ENSM_STATE_TX;
		break;
	case ENSM_MODE_RX:
		ensm_state = ENSM_STATE_RX;
		break;
	case ENSM_MODE_ALERT:
		ensm_state = ENSM_STATE_ALERT;
		break;
	case ENSM_MODE_FDD:
		ensm_state = ENSM_STATE_FDD;
		break;
	case ENSM_MODE_WAIT:
		ensm_state = ENSM_STATE_SLEEP_WAIT;
		break;
	case ENSM_MODE_SLEEP:
		ensm_state = ENSM_STATE_SLEEP;
		break;
	case ENSM_MODE_PINCTRL:
		ensm_state = ENSM_STATE_SLEEP_WAIT;
		pinctrl = true;
		break;
	case ENSM_MODE_PINCTRL_FDD_INDEP:
		ensm_state = ENSM_STATE_FDD;
		fdd_independent = true;
		break;
	default:
		return -EINVAL;
	}

	old_fdd_independent = phy->pdata->fdd_independent_mode;
	/* ad9361_set_ensm_mode() programs FDD_EXTERNAL_CTRL_ENABLE from this
	 * platform-data field, so stage the requested mode before writing it. */
	phy->pdata->fdd_independent_mode = fdd_independent;
	ret = ad9361_set_ensm_mode(phy, phy->pdata->fdd, pinctrl);
	if (ret < 0) {
		phy->pdata->fdd_independent_mode = old_fdd_independent;
		ad9361_mark_calibration_state_unknown(phy);
		return ret;
	}
	ret = ad9361_ensm_set_state(phy, ensm_state, pinctrl);
	if (ret < 0) {
		/* The FDD external-control bit was committed by the successful mode
		 * write above; retain its cache so retries do not misreport hardware. */
		ad9361_mark_calibration_state_unknown(phy);
		return ret;
	}

	return ret;
}

/**
 * Get the Enable State Machine (ENSM) mode.
 * @param phy The AD9361 current state structure.
 * @param mode A variable to store the selected ENSM mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_en_state_machine_mode(struct ad9361_rf_phy *phy,
		uint32_t *mode)
{
	uint8_t ensm_state;
	bool pinctrl = false;
	int32_t ret;

	if (!phy || !phy->pdata || !phy->spi || !mode)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	ret = ad9361_spi_read(phy->spi, REG_STATE);
	if (ret < 0)
		return ret;
	ensm_state = ret;
	ensm_state &= ENSM_STATE(~0);
	ret = ad9361_spi_read(phy->spi, REG_ENSM_CONFIG_1);
	if (ret < 0)
		return ret;
	if ((ret & ENABLE_ENSM_PIN_CTRL) == ENABLE_ENSM_PIN_CTRL)
		pinctrl = true;

	switch (ensm_state) {
	case ENSM_STATE_TX:
		*mode = ENSM_MODE_TX;
		break;
	case ENSM_STATE_RX:
		*mode = ENSM_MODE_RX;
		break;
	case ENSM_STATE_ALERT:
		*mode = ENSM_MODE_ALERT;
		break;
	case ENSM_STATE_FDD:
		if (phy->pdata->fdd_independent_mode)
			*mode = ENSM_MODE_PINCTRL_FDD_INDEP;
		else
			*mode = ENSM_MODE_FDD;
		break;
	case ENSM_STATE_SLEEP_WAIT:
		if (pinctrl)
			*mode = ENSM_MODE_PINCTRL;
		else
			*mode = ENSM_MODE_WAIT;
		break;
	case ENSM_STATE_SLEEP:
		*mode = ENSM_MODE_SLEEP;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

/**
 * Set the receive RF gain for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (RX1, RX2).
 * 			 Accepted values in 2x2 mode:
 * 			  RX1 (0)
 * 			  RX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  RX1 (0)
 * @param gain_db The RF gain (dB).
 * 				  Example:
 * 				   10 (10 dB)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rf_gain(struct ad9361_rf_phy *phy,
			      uint8_t ch, int32_t gain_db)
{
	struct rf_rx_gain rx_gain = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int32_t ret = 0;

	if (!phy || !phy->spi || !phy->pdata || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;

	rx_gain.gain_db = gain_db;
	ret = ad9361_set_rx_gain(phy,
				 ad9361_1rx1tx_channel_map(phy, false,
						 ch + 1), &rx_gain);

	return ret;
}

/**
 * Get current receive RF gain for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (RX1, RX2).
 * 			 Accepted values in 2x2 mode:
 * 			  RX1 (0)
 * 			  RX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  RX1 (0)
 * @param gain_db A variable to store the RF gain (dB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rf_gain(struct ad9361_rf_phy *phy,
			      uint8_t ch, int32_t *gain_db)
{
	struct rf_rx_gain rx_gain = {0, 0, 0, 0, 0, 0, 0, 0, 0};
	int32_t ret = 0;

	if (!phy || !phy->spi || !phy->pdata || !gain_db || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;

	ret = ad9361_get_rx_gain(phy, ad9361_1rx1tx_channel_map(phy,
				 false, ch + 1), &rx_gain);
	if (ret < 0)
		return ret;
	*gain_db = rx_gain.gain_db;

	return 0;
}

/**
 * Set the RX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz The desired bandwidth (Hz).
 * 					   Example:
 * 					    18000000 (18 MHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_rx_rf_bandwidth_with_cleanup(
	struct ad9361_rf_phy *phy, uint32_t bandwidth_hz,
	int32_t *cleanup_status)
{
	int32_t ret = 0;
	if (cleanup_status)
		*cleanup_status = 0;
	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;

	bandwidth_hz = ad9361_validate_rf_bw(phy, bandwidth_hz);

	if (!phy->current_bw_valid || phy->current_rx_bw_Hz != bandwidth_hz)
		ret = ad9361_update_rf_bandwidth_with_cleanup(
			phy, bandwidth_hz, phy->current_tx_bw_Hz, cleanup_status);
	else
		ret = 0;

	return ret;
}

int32_t ad9361_set_rx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t bandwidth_hz)
{
	return ad9361_set_rx_rf_bandwidth_with_cleanup(phy, bandwidth_hz, NULL);
}

/**
 * Get the RX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz A variable to store the bandwidth value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t *bandwidth_hz)
{
	if (!phy || !bandwidth_hz)
		return -EINVAL;
	if (!phy->current_bw_valid)
		return -EIO;

	*bandwidth_hz = phy->current_rx_bw_Hz;

	return 0;
}

/* A clock-plan update changes the shared RX/TX chain before its RF bandwidth
 * and calibration work completes. Restore the previous chain only after the
 * new chain was applied successfully and a later step failed. If applying the
 * new chain itself fails, it either rejected during side-effect-free
 * preflight or latched poison after mutation; neither case permits rollback
 * writes here. */
static int32_t ad9361_restore_clock_chain_after_update_failure(
	struct ad9361_rf_phy *phy, const uint32_t old_rx[6],
	const uint32_t old_tx[6], uint32_t old_rx_bw, uint32_t old_tx_bw,
	int32_t *cleanup_status_out)
{
	uint32_t restore_rx[6], restore_tx[6], check_rx[6], check_tx[6];
	int32_t chain_ret, read_ret, bandwidth_ret;
	int32_t cleanup_status = 0, child_cleanup_status = 0;
	bool clock_chain_restored;

	if (cleanup_status_out)
		*cleanup_status_out = 0;

	if (ad9361_calibration_state_unknown(phy)) {
		/* A clock rollback is another AD9361 mutation. When calibration may
		 * still be active, retain fail-closed state for RESETB recovery. */
		phy->current_bw_valid = false;
		dev_err(&phy->spi->dev,
			"Skipping clock-plan rollback: RFIC state is unknown");
		return -EIO;
	}

	memcpy(restore_rx, old_rx, sizeof(restore_rx));
	memcpy(restore_tx, old_tx, sizeof(restore_tx));
	chain_ret = ad9361_set_trx_clock_chain_with_cleanup(
		phy, restore_rx, restore_tx, &child_cleanup_status);
	if (child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	read_ret = ad9361_get_trx_clock_chain(phy, check_rx, check_tx);
	clock_chain_restored = read_ret == 0 &&
		!memcmp(old_rx, check_rx, sizeof(check_rx)) &&
		!memcmp(old_tx, check_tx, sizeof(check_tx));

	/* Even if the clock-change hook reported an error, the actual dividers may
	 * have been restored. Verify them before recalibrating the old bandwidth. */
	child_cleanup_status = 0;
	bandwidth_ret = clock_chain_restored
		? ad9361_update_rf_bandwidth_with_cleanup(
			phy, old_rx_bw, old_tx_bw, &child_cleanup_status)
		: -EIO;
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (cleanup_status_out)
		*cleanup_status_out = cleanup_status;
	if (!clock_chain_restored || bandwidth_ret < 0) {
		phy->current_bw_valid = false;
		dev_err(&phy->spi->dev,
			"Clock-plan rollback failed (clock=%"PRId32
			", readback=%"PRId32", bandwidth=%"PRId32")",
			chain_ret, read_ret, bandwidth_ret);
		return -EIO;
	}

	return 0;
}

static int32_t ad9361_sample_rate_transaction_finish(
		struct ad9361_sample_rate_transaction *transaction, int32_t status)
{
	struct ad9361_rf_phy *phy = transaction->phy;

	/* Do not strand a nested BBPLL/clock-chain owner if the outer context
	 * reaches an error path unexpectedly. Poison first, then let the child
	 * observe that state and retire without further hardware writes. */
	if (phy && transaction->clock_chain_job.active) {
		ad9361_mark_calibration_state_unknown(phy);
		(void)ad9361_clock_chain_transaction_poll(
			phy, &transaction->clock_chain_job);
		if (transaction->clock_chain_job.active)
			status = -EIO;
	}
	if (transaction->cleanup_status >= 0 &&
	    transaction->clock_chain_job.cleanup_status < 0)
		transaction->cleanup_status =
			transaction->clock_chain_job.cleanup_status;
	if (transaction->cleanup_status >= 0 &&
	    transaction->bandwidth_job.cleanup_status < 0)
		transaction->cleanup_status =
			transaction->bandwidth_job.cleanup_status;
	if (transaction->cleanup_status >= 0 &&
	    transaction->bandwidth_job.tracking_restore_status < 0)
		transaction->cleanup_status =
			transaction->bandwidth_job.tracking_restore_status;
	if (status >= 0 && transaction->cleanup_status < 0)
		status = transaction->cleanup_status;
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = status;
	if (phy && phy->sample_rate_transaction == transaction)
		phy->sample_rate_transaction = NULL;
	return status;
}

int32_t ad9361_sample_rate_transaction_start(
		struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz, bool tx,
		struct ad9361_sample_rate_transaction *transaction)
{
	uint32_t current_rate;
	int32_t ret;

	if (!transaction)
		return -EINVAL;
	if (transaction->active || transaction->clock_chain_job.active ||
	    transaction->clock_chain_job.bbpll_job.active ||
	    transaction->clock_chain_job.bbpll_job.rate_transaction.active ||
	    transaction->bandwidth_job.active ||
	    transaction->bandwidth_job.filter_job.active ||
	    transaction->bandwidth_job.tx_quad_job.active ||
	    transaction->bandwidth_job.tx_quad_job.attempt.active ||
	    transaction->bandwidth_job.tx_quad_job.phase_search.active ||
	    transaction->bandwidth_job.tx_quad_job.bandwidth_job.active ||
	    transaction->clock_chain_job.bbpll_job.rate_transaction.wait.active ||
	    transaction->bandwidth_job.filter_job.rx_bbf_job.wait.active ||
	    transaction->bandwidth_job.filter_job.tx_bbf_job.wait.active ||
	    transaction->bandwidth_job.tx_quad_job.attempt.wait.active ||
	    transaction->bandwidth_job.tx_quad_job.phase_search.attempt.wait.active)
		return -EBUSY;
	memset(transaction, 0, sizeof(*transaction));
	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	transaction->phy = phy;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	if (!phy->current_bw_valid) {
		dev_err(&phy->spi->dev,
			"Cannot change sample rate with unknown RF bandwidth state");
		return -EIO;
	}

	transaction->requested_rate = sampling_freq_hz;
	transaction->tx = tx;
	transaction->old_rx_bw = phy->current_rx_bw_Hz;
	transaction->old_tx_bw = phy->current_tx_bw_Hz;

	ret = clk_get_rate_checked(phy,
				   phy->ref_clk_scale[tx ? TX_SAMPL_CLK : RX_SAMPL_CLK],
				   &current_rate);
	if (ret < 0)
		return ret;
	if (current_rate == sampling_freq_hz)
		return ad9361_sample_rate_transaction_finish(transaction, 0);

	ret = ad9361_get_trx_clock_chain(phy, transaction->old_rx_path_clks,
					 transaction->old_tx_path_clks);
	if (ret < 0)
		return ret;
	ret = ad9361_calculate_rf_clock_chain(
		phy, sampling_freq_hz, phy->rate_governor,
		transaction->new_rx_path_clks,
		transaction->new_tx_path_clks);
	if (ret < 0)
		return ret;

	/* Keep BBPLL lock and every downstream divider update in the retained
	 * child. Its start performs validation only; hardware changes advance in
	 * poll(), allowing NIOS to service the queue between phases. */
	ret = ad9361_clock_chain_transaction_start(
		phy, transaction->new_rx_path_clks,
		transaction->new_tx_path_clks,
		&transaction->clock_chain_job);
	if (ret < 0)
		return ret;
	transaction->active = true;
	transaction->stage = AD9361_SAMPLE_RATE_TRANSACTION_NEW_CLOCK;
	phy->sample_rate_transaction = transaction;
	return 0;
}

int32_t ad9361_sample_rate_transaction_poll(
		struct ad9361_rf_phy *phy,
		struct ad9361_sample_rate_transaction *transaction)
{
	uint32_t rx[NUM_RX_CLOCKS], tx_path[NUM_TX_CLOCKS];
	int32_t ret;

	if (!phy || !phy->spi || !transaction)
		return -EINVAL;
	if (transaction->terminal) {
		if (transaction->clock_chain_job.active) {
			ad9361_mark_calibration_state_unknown(phy);
			(void)ad9361_clock_chain_transaction_poll(
				phy, &transaction->clock_chain_job);
			transaction->terminal_status = -EIO;
		}
		if (transaction->bandwidth_job.tx_quad_started &&
		    !transaction->bandwidth_job.tx_quad_job.terminal) {
			/* A terminal outer transaction cannot release the nested TXQ
			 * context while its hardware wait lacks a terminal observation. */
			ad9361_mark_calibration_state_unknown(phy);
			transaction->terminal_status = -EIO;
		}
		return transaction->terminal_status;
	}
	if (!transaction->active ||
	    transaction->stage > AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_BW) {
		ad9361_mark_calibration_state_unknown(phy);
		return ad9361_sample_rate_transaction_finish(transaction, -EIO);
	}
	if (transaction->phy != phy) {
		if (transaction->phy)
			ad9361_mark_calibration_state_unknown(transaction->phy);
		return ad9361_sample_rate_transaction_finish(transaction, -EIO);
	}

	switch (transaction->stage) {
	case AD9361_SAMPLE_RATE_TRANSACTION_NEW_CLOCK:
		if (phy->sample_rate_transaction != transaction) {
			ad9361_mark_calibration_state_unknown(phy);
			return ad9361_sample_rate_transaction_finish(transaction, -EIO);
		}
		/* Transfer the visible owner to the clock-chain child for this
		 * indivisible poll; restore the sample-rate owner before yielding. */
		phy->sample_rate_transaction = NULL;
		ret = ad9361_clock_chain_transaction_poll(
			phy, &transaction->clock_chain_job);
		phy->sample_rate_transaction = transaction;
		if (ret == 1)
			return 1;
		if (ret < 0) {
			ad9361_mark_calibration_state_unknown(phy);
			return ad9361_sample_rate_transaction_finish(transaction, ret);
		}

		/* The new clock plan is committed. Start bandwidth/TXQ only now; if
		 * this side-effect-free preflight fails, retain ownership and roll the
		 * clock chain back through another retained child. */
		phy->sample_rate_transaction = NULL;
		ret = ad9361_rf_bandwidth_transaction_start(
			phy, transaction->old_rx_bw, transaction->old_tx_bw,
			&transaction->bandwidth_job);
		phy->sample_rate_transaction = transaction;
		if (ret < 0) {
			if (ad9361_calibration_state_unknown(phy))
				return ad9361_sample_rate_transaction_finish(transaction, ret);
			transaction->status = ret;
			transaction->rollback_clock_started = false;
			transaction->stage =
				AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_CLOCK;
			return 1;
		}
		transaction->stage = AD9361_SAMPLE_RATE_TRANSACTION_NEW_BANDWIDTH;
		/* The bandwidth child owns subsequent polls; retain the sample-rate
		 * marker only if a failure creates a rollback scheduling gap. */
		phy->sample_rate_transaction = NULL;
		return 1;

	case AD9361_SAMPLE_RATE_TRANSACTION_NEW_BANDWIDTH:
	case AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_BW:
		ret = ad9361_rf_bandwidth_transaction_poll(
			phy, &transaction->bandwidth_job);
		if (ret == 1)
			return 1;
		if (transaction->stage ==
		    AD9361_SAMPLE_RATE_TRANSACTION_NEW_BANDWIDTH) {
			if (ret == 0)
				return ad9361_sample_rate_transaction_finish(
					transaction, 0);
			transaction->status = ret;
			if (ad9361_calibration_state_unknown(phy))
				return ad9361_sample_rate_transaction_finish(
					transaction, ret);
			transaction->stage =
				AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_CLOCK;
			transaction->rollback_clock_started = false;
			/* The child bandwidth job has released its owner at terminal
			 * failure. Retain the parent across the rollback scheduling gap. */
			phy->sample_rate_transaction = transaction;
			return 1;
		}

		if (ret < 0)
			dev_err(&phy->spi->dev,
				"Sample-rate rollback bandwidth failed (%"PRId32")",
				ret);
		else
			transaction->rollback_complete = true;
		return ad9361_sample_rate_transaction_finish(
			transaction, transaction->status);

	case AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_CLOCK:
		if (phy->sample_rate_transaction != transaction) {
			ad9361_mark_calibration_state_unknown(phy);
			return ad9361_sample_rate_transaction_finish(transaction, -EIO);
		}
		if (ad9361_calibration_state_unknown(phy))
			return ad9361_sample_rate_transaction_finish(
				transaction, transaction->status);
		if (!transaction->rollback_clock_started) {
			phy->sample_rate_transaction = NULL;
			ret = ad9361_clock_chain_transaction_start(
				phy, transaction->old_rx_path_clks,
				transaction->old_tx_path_clks,
				&transaction->clock_chain_job);
			phy->sample_rate_transaction = transaction;
			if (ret < 0) {
				dev_err(&phy->spi->dev,
					"Sample-rate rollback clock start failed (%"PRId32")",
					ret);
				return ad9361_sample_rate_transaction_finish(
					transaction, transaction->status);
			}
			transaction->rollback_clock_started = true;
			return 1;
		}

		phy->sample_rate_transaction = NULL;
		ret = ad9361_clock_chain_transaction_poll(
			phy, &transaction->clock_chain_job);
		phy->sample_rate_transaction = transaction;
		if (ret == 1)
			return 1;
		if (ret < 0) {
			dev_err(&phy->spi->dev,
				"Sample-rate rollback clock plan failed (%"PRId32")",
				ret);
			return ad9361_sample_rate_transaction_finish(
				transaction, transaction->status);
		}

		/* A nested clock transaction is terminal now. Temporarily drop the
		 * parent marker for checked SPI readback and bandwidth-job admission. */
		phy->sample_rate_transaction = NULL;
		ret = ad9361_get_trx_clock_chain(phy, rx, tx_path);
		phy->sample_rate_transaction = transaction;
		if (ret < 0 ||
		    memcmp(rx, transaction->old_rx_path_clks, sizeof(rx)) != 0 ||
		    memcmp(tx_path, transaction->old_tx_path_clks, sizeof(tx_path)) != 0) {
			dev_err(&phy->spi->dev,
				"Sample-rate rollback clock readback mismatch (%"PRId32")",
				ret);
			ad9361_mark_calibration_state_unknown(phy);
			return ad9361_sample_rate_transaction_finish(
				transaction, transaction->status);
		}
		phy->sample_rate_transaction = NULL;
		ret = ad9361_rf_bandwidth_transaction_start(
			phy, transaction->old_rx_bw, transaction->old_tx_bw,
			&transaction->bandwidth_job);
		phy->sample_rate_transaction = transaction;
		if (ret < 0) {
			dev_err(&phy->spi->dev,
				"Sample-rate rollback calibration could not start (%"PRId32")",
				ret);
			return ad9361_sample_rate_transaction_finish(
				transaction, transaction->status);
		}
		transaction->stage = AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_BW;
		phy->sample_rate_transaction = NULL;
		return 1;
	default:
		ad9361_mark_calibration_state_unknown(phy);
		return ad9361_sample_rate_transaction_finish(transaction, -EIO);
	}
}

/* Keep the legacy synchronous API on the same transaction implementation
 * used by cooperative NIOS callers. This prevents host and firmware setters
 * from drifting in cache, rollback, and failure semantics. */
static int32_t ad9361_sample_rate_transaction_run_sync(
	struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz, bool tx,
	int32_t *cleanup_status)
{
	struct ad9361_sample_rate_transaction transaction = {0};
	int32_t ret;

	if (cleanup_status)
		*cleanup_status = 0;
	ret = ad9361_sample_rate_transaction_start(
		phy, sampling_freq_hz, tx, &transaction);
	if (ret < 0) {
		if (cleanup_status)
			*cleanup_status = transaction.cleanup_status;
		return ret;
	}

	for (;;) {
		ret = ad9361_sample_rate_transaction_poll(phy, &transaction);
		if (ret != 1) {
			if (cleanup_status)
				*cleanup_status = transaction.cleanup_status;
			return ret;
		}

		if ((transaction.stage == AD9361_SAMPLE_RATE_TRANSACTION_NEW_CLOCK ||
		     transaction.stage == AD9361_SAMPLE_RATE_TRANSACTION_ROLLBACK_CLOCK) &&
		    transaction.clock_chain_job.stage ==
		    AD9361_CLOCK_CHAIN_TRANSACTION_BBPLL &&
		    transaction.clock_chain_job.bbpll_job.rate_transaction.wait_started) {
			no_os_udelay(transaction.clock_chain_job.bbpll_job.rate_transaction.
				     wait.poll_delay_us);
		} else if (transaction.bandwidth_job.stage ==
		    AD9361_RF_BANDWIDTH_TRANSACTION_FILTER) {
			if (transaction.bandwidth_job.filter_job.stage ==
			    AD9361_RF_BANDWIDTH_FILTER_RX_BBF_WAIT)
				no_os_udelay(transaction.bandwidth_job.filter_job.
					    rx_bbf_job.wait.poll_delay_us);
			else if (transaction.bandwidth_job.filter_job.stage ==
				 AD9361_RF_BANDWIDTH_FILTER_TX_BBF_WAIT)
				no_os_udelay(transaction.bandwidth_job.filter_job.
					    tx_bbf_job.wait.poll_delay_us);
		} else if (transaction.bandwidth_job.stage ==
		    AD9361_RF_BANDWIDTH_TRANSACTION_TX_QUAD) {
			struct ad9361_tx_quad_calibration_job *job =
				&transaction.bandwidth_job.tx_quad_job;

			if (job->stage == AD9361_TX_QUAD_JOB_BANDWIDTH_SETUP ||
			    job->stage == AD9361_TX_QUAD_JOB_BANDWIDTH_RESTORE) {
				if (job->bandwidth_job.stage ==
				    AD9361_RF_BANDWIDTH_FILTER_RX_BBF_WAIT)
					no_os_udelay(job->bandwidth_job.rx_bbf_job.
						     wait.poll_delay_us);
				else if (job->bandwidth_job.stage ==
					 AD9361_RF_BANDWIDTH_FILTER_TX_BBF_WAIT)
					no_os_udelay(job->bandwidth_job.tx_bbf_job.
						     wait.poll_delay_us);
			} else if (job->stage == AD9361_TX_QUAD_JOB_SEARCH) {
				no_os_udelay(job->phase_search.attempt.wait.poll_delay_us);
			} else if (job->stage != AD9361_TX_QUAD_JOB_CLEANUP) {
				no_os_udelay(job->attempt.wait.poll_delay_us);
			}
		}
	}
}

/**
 * Set the RX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz The desired frequency (Hz).
 * 						   Example:
 * 						    30720000 (30.72 MHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_rx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t sampling_freq_hz)
{
	return ad9361_sample_rate_transaction_run_sync(
		phy, sampling_freq_hz, false, NULL);
}

int32_t ad9361_set_rx_sampling_freq_with_cleanup(
	struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz,
	int32_t *cleanup_status)
{
	return ad9361_sample_rate_transaction_run_sync(
		phy, sampling_freq_hz, false, cleanup_status);
}

/**
 * Get current RX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t *sampling_freq_hz)
{
	uint32_t freq;
	int32_t ret;

	if (!phy || !sampling_freq_hz)
		return -EINVAL;

	ret = clk_get_rate_checked(
		phy, phy->ref_clk_scale[RX_SAMPL_CLK], &freq);
	if (ret < 0)
		return ret;
	if (freq == 0)
		return -ENODEV;

	*sampling_freq_hz = freq;

	return 0;
}

/**
 * Set the RX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz The desired frequency (Hz).
 * 					 Example:
 * 					  2400000000 (2.4 GHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_rx_lo_freq(struct ad9361_rf_phy *phy,
			      uint64_t lo_freq_hz)
{
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (lo_freq_hz < MIN_RX_CARRIER_FREQ_HZ ||
	    lo_freq_hz > MAX_CARRIER_FREQ_HZ)
		return -EINVAL;
	if (!phy->rfpll_source_valid[0])
		return -EIO;

	ret = clk_set_rate(phy, phy->ref_clk_scale[RX_RFPLL],
			   ad9361_to_clk(lo_freq_hz));

	return ret;
}

/**
 * Get current RX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_lo_freq(struct ad9361_rf_phy *phy,
			      uint64_t *lo_freq_hz)
{
	uint32_t rate;
	int32_t ret;
	struct refclk_scale *pll_scale;
	uint32_t parent_source;

	if (!phy || !phy->spi || !phy->pdata || !lo_freq_hz)
		return -EINVAL;
	if (!phy->rfpll_source_valid[0])
		return -EIO;

	if (phy->pdata->use_ext_rx_lo) {
		if (phy->ad9361_rfpll_ext_recalc_rate) {
			if (!phy->ref_clk_scale[RX_RFPLL])
				return -EINVAL;
			rate = phy->ad9361_rfpll_ext_recalc_rate(
				phy->ref_clk_scale[RX_RFPLL]);
		} else {
			if (!phy->ref_clk_scale[RX_RFPLL_DUMMY])
				return -EINVAL;
			rate = ad9361_rfpll_dummy_recalc_rate(
				phy->ref_clk_scale[RX_RFPLL_DUMMY]);
		}
	} else {
		pll_scale = phy->ref_clk_scale[RX_RFPLL_INT];
		if (!pll_scale || pll_scale->parent_source >= NUM_AD9361_CLKS ||
		    !phy->clks[pll_scale->parent_source])
			return -EINVAL;
		parent_source = pll_scale->parent_source;
		ret = ad9361_rfpll_int_recalc_rate_checked(
			pll_scale, phy->clks[parent_source]->rate,
			&rate);
		if (ret < 0)
			return ret;
	}

	*lo_freq_hz = ad9361_from_clk(rate);

	return 0;
}

/**
 * Switch between internal and external LO.
 * @param phy The AD9361 state structure.
 * @param int_ext The selected lo (INT_LO, EXT_LO).
 * 			  Accepted values:
 * 			   INT_LO
 * 			   EXT_LO
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_lo_int_ext(struct ad9361_rf_phy *phy, uint8_t int_ext)
{
	uint8_t old_use_ext;
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata || !phy->ref_clk_scale[RX_RFPLL] ||
	    (int_ext != INT_LO && int_ext != EXT_LO))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_use_ext = phy->pdata->use_ext_rx_lo;
	if ((phy->dev_sel == ID_AD9363A) && (int_ext == EXT_LO)) {
		AD9361_DIAG_PRINTF("%s : EXT_LO is not supported by AD9363!\n", __func__);
		return -1;
	}

	phy->pdata->use_ext_rx_lo = int_ext;
	ret = ad9361_clk_mux_set_parent(phy->ref_clk_scale[RX_RFPLL], int_ext);
	/* clk_mux_set_parent commits mult as soon as the source switch succeeds;
	 * it can still report an ENSM-restore or internal-PLL relock error after
	 * that point. Keep pdata aligned with the committed mux source. */
	if (ret < 0 && phy->ref_clk_scale[RX_RFPLL]->mult != int_ext)
		phy->pdata->use_ext_rx_lo = old_use_ext;
	return ret;
}

/**
 * Get the RSSI for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (RX1, RX2).
 * 			 Accepted values in 2x2 mode:
 * 			  RX1 (0)
 * 			  RX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  RX1 (0)
 * @param rssi A variable to store the RSSI.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rssi(struct ad9361_rf_phy *phy,
			   uint8_t ch, struct rf_rssi *rssi)
{
	int32_t ret;
	struct rf_rssi result = {0};

	if (!phy || !phy->spi || !phy->pdata || !rssi || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	result.ant = ad9361_1rx1tx_channel_map(phy, false, ch + 1);
	result.duration = 1;
	ret = ad9361_read_rssi(phy, &result);
	if (ret < 0)
		return ret;
	*rssi = result;
	return ret;
}

/**
 * Set the gain control mode for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (RX1, RX2).
 * 			 Accepted values in 2x2 mode:
 * 			  RX1 (0)
 * 			  RX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  RX1 (0)
 * @param gc_mode The gain control mode (manual, fast_attack, slow_attack,
 * 				  hybrid).
 *                Accepted values:
 *				   RF_GAIN_MGC (manual)
 *				   RF_GAIN_FASTATTACK_AGC (fast_attack)
 *				   RF_GAIN_SLOWATTACK_AGC (slow_attack)
 *				   RF_GAIN_HYBRID_AGC (hybrid)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_gain_control_mode(struct ad9361_rf_phy *phy,
					uint8_t ch, uint8_t gc_mode)
{
	struct rf_gain_ctrl gc = {0, 0};

	if (!phy || !phy->spi || !phy->pdata || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	gc.ant = ad9361_1rx1tx_channel_map(phy, false, ch + 1);
	gc.mode = gc_mode;
	if (gc_mode > RF_GAIN_HYBRID_AGC)
		return -EINVAL;
	{
		int32_t ret = ad9361_set_gain_ctrl_mode(phy, &gc);
		if (ret < 0)
			return ret;
	}
	phy->agc_mode[ch] = gc_mode;
	return 0;
}

/**
 * Get the gain control mode for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (RX1, RX2).
 * 			 Accepted values:
 * 			  RX1 (0)
 * 			  RX2 (1)
 * @param gc_mode A variable to store the gain control mode.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_gain_control_mode(struct ad9361_rf_phy *phy,
					uint8_t ch, uint8_t *gc_mode)
{
	if (!phy || !phy->pdata || !gc_mode || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*gc_mode = phy->agc_mode[ch];

	return 0;
}

/**
 * Set the RX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param fir_cfg FIR filter configuration.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_rx_fir_config_with_cleanup(
	struct ad9361_rf_phy *phy, AD9361_RXFIRConfig fir_cfg,
	int32_t *cleanup_status)
{
	int32_t ret;
	uint8_t old_rx_fir_dec;
	if (cleanup_status)
		*cleanup_status = 0;

	if (!phy || !phy->spi)
		return -EINVAL;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_rx_fir_dec = phy->rx_fir_dec;

	phy->fir_config_valid = false;
	phy->rx_fir_dec = fir_cfg.rx_dec;
	ret = ad9361_load_fir_filter_coef_with_cleanup(
		phy, (enum fir_dest)(fir_cfg.rx | FIR_IS_RX), fir_cfg.rx_gain,
		fir_cfg.rx_coef_size, fir_cfg.rx_coef, cleanup_status);
	if (ret < 0) {
		phy->rx_fir_dec = old_rx_fir_dec;
	}
	return ret;
}

int32_t ad9361_set_rx_fir_config(struct ad9361_rf_phy *phy,
				 AD9361_RXFIRConfig fir_cfg)
{
	return ad9361_set_rx_fir_config_with_cleanup(phy, fir_cfg, NULL);
}

/**
 * Get the RX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param rx_ch The selected RX channel (RX1, RX2).
 * 				Accepted values:
 * 				 RX1 (0)
 * 				 RX2 (1)
 * @param fir_cfg FIR filter configuration output file.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_fir_config(struct ad9361_rf_phy *phy, uint8_t rx_ch,
				 AD9361_RXFIRConfig *fir_cfg)
{
	int32_t ret;
	uint32_t fir_conf;
	uint32_t original_fir_conf;
	uint8_t index;
	AD9361_RXFIRConfig result = {0};
	if (!phy || !phy->spi || !fir_cfg || rx_ch > 1)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	rx_ch += 1;

	ret = ad9361_spi_read(phy->spi, REG_RX_FILTER_CONFIG);
	if (ret < 0)
		return ret;
	original_fir_conf = ret;
	fir_conf = original_fir_conf;

	result.rx_coef_size = (((fir_conf & FIR_NUM_TAPS(7)) >> 5) + 1) * 16;

	ret = ad9361_spi_read(phy->spi, REG_RX_FILTER_GAIN);
	if (ret < 0)
		return ret;
	result.rx_gain = -6 * (ret & FILTER_GAIN(3)) + 6;
	result.rx = rx_ch;

	fir_conf &= ~FIR_SELECT(3);
	fir_conf |= FIR_SELECT(rx_ch) | FIR_START_CLK;
	ret = ad9361_spi_write(phy->spi, REG_RX_FILTER_CONFIG, fir_conf);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy);
		goto rx_fir_restore;
	}

	for (index = 0; index < 128; index++) {
		ret = ad9361_spi_write(phy->spi, REG_RX_FILTER_COEF_ADDR, index);
		if (ret < 0) {
			ad9361_mark_calibration_state_unknown(phy);
			goto rx_fir_restore;
		}
		ret = ad9361_spi_read(phy->spi, REG_RX_FILTER_COEF_READ_DATA_1);
		if (ret < 0)
			goto rx_fir_restore;
		result.rx_coef[index] = ret;
		ret = ad9361_spi_read(phy->spi, REG_RX_FILTER_COEF_READ_DATA_2);
		if (ret < 0)
			goto rx_fir_restore;
		result.rx_coef[index] |= (ret << 8);
	}

	rx_fir_restore:
	if (ad9361_calibration_state_unknown(phy))
		return ret < 0 ? ret : -EIO;
	{
		int32_t restore_ret = ad9361_spi_write(phy->spi,
						       REG_RX_FILTER_CONFIG,
						       original_fir_conf & ~FIR_START_CLK);
		if (restore_ret < 0) {
			ad9361_mark_calibration_state_unknown(phy);
			if (ret >= 0)
				ret = restore_ret;
		}
	}
	if (ret < 0)
		return ret;

	result.rx_dec = phy->rx_fir_dec;
	*fir_cfg = result;

	return 0;
}

/**
 * Enable/disable the RX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_rx_fir_en_dis_with_cleanup(
	struct ad9361_rf_phy *phy, uint8_t en_dis, int32_t *cleanup_status)
{
	int32_t ret = 0;
	bool old_bypass;
	if (cleanup_status)
		*cleanup_status = 0;
	if (!phy || !phy->spi)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_bypass = phy->bypass_rx_fir;
	if (en_dis > 1)
		return -EINVAL;

	if (phy->fir_config_valid && phy->bypass_rx_fir == !en_dis)
		return ret;

	phy->bypass_rx_fir = !en_dis;
	ret = ad9361_validate_enable_fir_with_cleanup(phy, cleanup_status);
	if (ret < 0)
		phy->bypass_rx_fir = old_bypass;

	return ret;
}

int32_t ad9361_set_rx_fir_en_dis(struct ad9361_rf_phy *phy,
				 uint8_t en_dis)
{
	return ad9361_set_rx_fir_en_dis_with_cleanup(phy, en_dis, NULL);
}

/**
 * Get the status of the RX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_fir_en_dis(struct ad9361_rf_phy *phy,
				 uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*en_dis = !phy->bypass_rx_fir;

	return 0;
}

/**
 * Enable/disable the RX RFDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rfdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis)
{
	int32_t ret;
	uint8_t old_en;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_en = phy->rfdc_track_en;

	if (en_dis > 1)
		return -EINVAL;

	/* Reapply the complete tracking register set even when the requested
	 * value matches the cache. A previous multi-register SPI write may have
	 * failed partway through, leaving hardware different from that cache. */
	phy->rfdc_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				      phy->rfdc_track_en, phy->quad_track_en);
	if (ret < 0) {
		int32_t restore_ret;

		phy->rfdc_track_en = old_en;
		restore_ret = ad9361_tracking_control(
			phy, phy->bbdc_track_en, phy->rfdc_track_en,
			phy->quad_track_en);
		if (restore_ret < 0)
			dev_err(&phy->spi->dev,
				"Unable to restore RX tracking after RFDC update failure: %"PRId32,
				restore_ret);
	}

	return ret;
}

/**
 * Get the status of the RX RFDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rfdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (!phy->tracking_config_valid)
		return -EIO;

	*en_dis = phy->rfdc_track_en;

	return 0;
}

/**
 * Enable/disable the RX BasebandDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_bbdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis)
{
	int32_t ret;
	uint8_t old_en;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_en = phy->bbdc_track_en;

	if (en_dis > 1)
		return -EINVAL;

	/* Reapply all tracking registers: a prior partial SPI failure can leave
	 * hardware inconsistent with the cached enable bits. */
	phy->bbdc_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				      phy->rfdc_track_en, phy->quad_track_en);
	if (ret < 0) {
		int32_t restore_ret;

		phy->bbdc_track_en = old_en;
		restore_ret = ad9361_tracking_control(
			phy, phy->bbdc_track_en, phy->rfdc_track_en,
			phy->quad_track_en);
		if (restore_ret < 0)
			dev_err(&phy->spi->dev,
				"Unable to restore RX tracking after BB DC update failure: %"PRId32,
				restore_ret);
	}

	return ret;
}

/**
 * Get the status of the RX BasebandDC Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_bbdc_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (!phy->tracking_config_valid)
		return -EIO;

	*en_dis = phy->bbdc_track_en;

	return 0;
}

/**
 * Enable/disable the RX Quadrature Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_quad_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t en_dis)
{
	int32_t ret;
	uint8_t old_en;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_en = phy->quad_track_en;

	if (en_dis > 1)
		return -EINVAL;

	/* Reapply all tracking registers: a prior partial SPI failure can leave
	 * hardware inconsistent with the cached enable bits. */
	phy->quad_track_en = en_dis;
	ret = ad9361_tracking_control(phy, phy->bbdc_track_en,
				      phy->rfdc_track_en, phy->quad_track_en);
	if (ret < 0) {
		int32_t restore_ret;

		phy->quad_track_en = old_en;
		restore_ret = ad9361_tracking_control(
			phy, phy->bbdc_track_en, phy->rfdc_track_en,
			phy->quad_track_en);
		if (restore_ret < 0)
			dev_err(&phy->spi->dev,
				"Unable to restore RX tracking after quadrature update failure: %"PRId32,
				restore_ret);
	}

	return ret;
}

/**
 * Get the status of the RX Quadrature Tracking.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_quad_track_en_dis(struct ad9361_rf_phy *phy,
					uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (!phy->tracking_config_valid)
		return -EIO;

	*en_dis = phy->quad_track_en;

	return 0;
}

/**
 * Set the RX RF input port.
 * @param phy The AD9361 current state structure.
 * @param mode The RF port.
 * 			   Accepted values:
 *				A_BALANCED (0 - (RX1A_N &  RX1A_P) and (RX2A_N & RX2A_P) enabled; balanced)
 *				B_BALANCED (1 - (RX1B_N &  RX1B_P) and (RX2B_N & RX2B_P) enabled; balanced)
 *				C_BALANCED (2 - (RX1C_N &  RX1C_P) and (RX2C_N & RX2C_P) enabled; balanced)
 *				A_N		   (3 - RX1A_N and RX2A_N enabled; unbalanced)
 *				A_P		   (4 - RX1A_P and RX2A_P enabled; unbalanced)
 *				B_N		   (5 - RX1B_N and RX2B_N enabled; unbalanced)
 *				B_P		   (6 - RX1B_P and RX2B_P enabled; unbalanced)
 *				C_N		   (7 - RX1C_N and RX2C_N enabled; unbalanced)
 *				C_P		   (8 - RX1C_P and RX2C_P enabled; unbalanced)
 *				TX_MON1	   (9 - TX_MONITOR1)
 *				TX_MON2	   (10 - TX_MONITOR2)
 *				TX_MON1_2  (11 - TX_MONITOR1 & TX_MONITOR2)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_rx_rf_port_input(struct ad9361_rf_phy *phy,
				    uint32_t mode)
{
	int32_t ret;
	uint32_t old_mode;

	if (!phy || !phy->spi || !phy->pdata || mode > 11)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_mode = phy->pdata->rf_rx_input_sel;

	phy->pdata->rf_rx_input_sel = mode;

	ret = ad9361_rf_port_setup(phy, false,
				   phy->pdata->rf_rx_input_sel,
				   phy->pdata->rf_tx_output_sel);
	if (ret < 0)
		phy->pdata->rf_rx_input_sel = old_mode;

	return ret;
}

/**
 * Get the selected RX RF input port.
 * @param phy The AD9361 current state structure.
 * @param mode The RF port.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_rf_port_input(struct ad9361_rf_phy *phy,
				    uint32_t *mode)
{
	if (!phy || !phy->pdata || !mode)
		return -EINVAL;
	if (!phy->rf_port_select_valid)
		return -EIO;

	*mode = phy->pdata->rf_rx_input_sel;

	return 0;
}

/**
 * Store RX fastlock profile.
 * To create a profile tune the synthesizer (ad9361_set_rx_lo_freq()) and then
 * call this function specifying the target profile number.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_rx_fastlock_store(struct ad9361_rf_phy *phy, uint32_t profile)
{
	return ad9361_fastlock_store(phy, 0, profile);
}

/**
 * Recall specified RX fastlock profile.
 * When in fastlock pin select mode (init_param->rx_fastlock_pincontrol_enable),
 * the function needs to be called before then the pin-control can be used.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_rx_fastlock_recall(struct ad9361_rf_phy *phy, uint32_t profile)
{
	return ad9361_fastlock_recall(phy, 0, profile);
}

/**
 * Load RX fastlock profile. A previously saved profile can be loaded in any
 * of the 8 available slots.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @param values Fastlock profile program data.
 * 				 Example:
 * 				  val0,val1,val2,,val15
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_rx_fastlock_load(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values)
{
	return ad9361_fastlock_load(phy, 0, profile, values);
}

/**
 * Save RX fastlock profile. In order to use more than 8 Profiles, an existing
 * profile can be read back and stored by the user application.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @param values Fastlock profile program data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_rx_fastlock_save(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values)
{
	return ad9361_fastlock_save(phy, 0, profile, values);
}

/**
 * Power down the RX Local Oscillator.
 * @param phy The AD9361 state structure.
 * @param option The option (ON, OFF).
 * 				 Accepted values:
 * 				  ON (0)
 * 				  OFF (1)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_rx_lo_powerdown(struct ad9361_rf_phy *phy, uint8_t option)
{
	if (option > 1)
		return -EINVAL;

	return ad9361_synth_lo_powerdown(phy, option ? LO_OFF : LO_ON, LO_DONTCARE);
}

/**
 * Get the RX Local Oscillator power status.
 * @param phy The AD9361 state structure.
 * @param option Store the option (ON, OFF).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_rx_lo_power(struct ad9361_rf_phy *phy, uint8_t *option)
{
	if (!phy || !option)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*option = !(phy->cached_synth_pd[1] & RX_LO_POWER_DOWN) ? OFF : ON;

	return 0;
}

/**
 * Set the transmit attenuation for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (TX1, TX2).
 * 			 Accepted values in 2x2 mode:
 * 			  TX1 (0)
 * 			  TX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  TX1 (0)
 * @param attenuation_mdb The attenuation (mdB).
 * 						  Example:
 * 						   10000 (10 dB)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_attenuation(struct ad9361_rf_phy *phy,
				  uint8_t ch, uint32_t attenuation_mdb)
{
	int32_t ret;
	int32_t channel;

	if (!phy || !phy->spi || !phy->pdata || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;

	channel = ad9361_1rx1tx_channel_map(phy, true, ch);
	if (channel != 0 && channel != 1)
		return -EINVAL;
	ret = ad9361_set_tx_atten(phy, attenuation_mdb,
				  channel == 0, channel == 1,
				  !phy->pdata->update_tx_gain_via_alert);

	return ret;
}

/**
 * Get current transmit attenuation for the selected channel.
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel number (TX1, TX2).
 * 			 Accepted values in 2x2 mode:
 * 			  TX1 (0)
 * 			  TX2 (1)
 * 			 Accepted values in 1x1 mode:
 * 			  TX1 (0)
 * @param attenuation_mdb A variable to store the attenuation value (mdB).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_attenuation(struct ad9361_rf_phy *phy,
				  uint8_t ch, uint32_t *attenuation_mdb)
{
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata || !attenuation_mdb || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;

	ret = ad9361_get_tx_atten(phy,
				  ad9361_1rx1tx_channel_map(phy, true,
						  ch + 1));

	if (ret < 0)
		return ret;
	*attenuation_mdb = ret;

	return 0;
}

/**
 * Set the TX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz The desired bandwidth (Hz).
 * 					   Example:
 * 					    18000000 (18 MHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_tx_rf_bandwidth_with_cleanup(
	struct ad9361_rf_phy *phy, uint32_t bandwidth_hz,
	int32_t *cleanup_status)
{
	int32_t ret = 0;
	if (cleanup_status)
		*cleanup_status = 0;
	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;

	bandwidth_hz = ad9361_validate_rf_bw(phy, bandwidth_hz);

	if (!phy->current_bw_valid || phy->current_tx_bw_Hz != bandwidth_hz)
		ret = ad9361_update_rf_bandwidth_with_cleanup(
			phy, phy->current_rx_bw_Hz, bandwidth_hz, cleanup_status);
	else
		ret = 0;

	return ret;
}

int32_t ad9361_set_tx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t bandwidth_hz)
{
	return ad9361_set_tx_rf_bandwidth_with_cleanup(phy, bandwidth_hz, NULL);
}

/**
 * Get the TX RF bandwidth.
 * @param phy The AD9361 current state structure.
 * @param bandwidth_hz A variable to store the bandwidth value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_rf_bandwidth(struct ad9361_rf_phy *phy,
				   uint32_t *bandwidth_hz)
{
	if (!phy || !bandwidth_hz)
		return -EINVAL;
	if (!phy->current_bw_valid)
		return -EIO;

	*bandwidth_hz = phy->current_tx_bw_Hz;

	return 0;
}

/**
 * Set the TX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz The desired frequency (Hz).
 * 						   Example:
 * 						    30720000 (30.72 MHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_tx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t sampling_freq_hz)
{
	return ad9361_sample_rate_transaction_run_sync(
		phy, sampling_freq_hz, true, NULL);
}

int32_t ad9361_set_tx_sampling_freq_with_cleanup(
	struct ad9361_rf_phy *phy, uint32_t sampling_freq_hz,
	int32_t *cleanup_status)
{
	return ad9361_sample_rate_transaction_run_sync(
		phy, sampling_freq_hz, true, cleanup_status);
}

/**
 * Get current TX sampling frequency.
 * @param phy The AD9361 current state structure.
 * @param sampling_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_sampling_freq(struct ad9361_rf_phy *phy,
				    uint32_t *sampling_freq_hz)
{
	uint32_t freq;
	int32_t ret;

	if (!phy || !sampling_freq_hz)
		return -EINVAL;

	ret = clk_get_rate_checked(
		phy, phy->ref_clk_scale[TX_SAMPL_CLK], &freq);
	if (ret < 0)
		return ret;
	if (freq == 0)
		return -ENODEV;

	*sampling_freq_hz = freq;

	return 0;
}

/**
 * Set the TX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz The desired frequency (Hz).
 * 					 Example:
 * 					  2400000000 (2.4 GHz)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_tx_lo_freq(struct ad9361_rf_phy *phy,
			      uint64_t lo_freq_hz)
{
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (lo_freq_hz < MIN_TX_CARRIER_FREQ_HZ ||
	    lo_freq_hz > MAX_CARRIER_FREQ_HZ)
		return -EINVAL;
	if (!phy->rfpll_source_valid[1])
		return -EIO;

	ret = clk_set_rate(phy, phy->ref_clk_scale[TX_RFPLL],
			   ad9361_to_clk(lo_freq_hz));
	if (ret < 0)
		ad9361_invalidate_tx_quad_calib(phy);

	return ret;
}

/**
 * Get current TX LO frequency.
 * @param phy The AD9361 current state structure.
 * @param lo_freq_hz A variable to store the frequency value (Hz).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_lo_freq(struct ad9361_rf_phy *phy,
			      uint64_t *lo_freq_hz)
{
	uint32_t rate;
	int32_t ret;
	struct refclk_scale *pll_scale;
	uint32_t parent_source;

	if (!phy || !phy->spi || !phy->pdata || !lo_freq_hz)
		return -EINVAL;
	if (!phy->rfpll_source_valid[1])
		return -EIO;

	if (phy->pdata->use_ext_tx_lo) {
		if (phy->ad9361_rfpll_ext_recalc_rate) {
			if (!phy->ref_clk_scale[TX_RFPLL])
				return -EINVAL;
			rate = phy->ad9361_rfpll_ext_recalc_rate(
				phy->ref_clk_scale[TX_RFPLL]);
		} else {
			if (!phy->ref_clk_scale[TX_RFPLL_DUMMY])
				return -EINVAL;
			rate = ad9361_rfpll_dummy_recalc_rate(
				phy->ref_clk_scale[TX_RFPLL_DUMMY]);
		}
	} else {
		pll_scale = phy->ref_clk_scale[TX_RFPLL_INT];
		if (!pll_scale || pll_scale->parent_source >= NUM_AD9361_CLKS ||
		    !phy->clks[pll_scale->parent_source])
			return -EINVAL;
		parent_source = pll_scale->parent_source;
		ret = ad9361_rfpll_int_recalc_rate_checked(
			pll_scale, phy->clks[parent_source]->rate,
			&rate);
		if (ret < 0)
			return ret;
	}

	*lo_freq_hz = ad9361_from_clk(rate);

	return 0;
}

/**
 * Switch between internal and external LO.
 * @param phy The AD9361 state structure.
 * @param int_ext The selected lo (INT_LO, EXT_LO).
 * 			  Accepted values:
 * 			   INT_LO
 * 			   EXT_LO
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_lo_int_ext(struct ad9361_rf_phy *phy, uint8_t int_ext)
{
	uint8_t old_use_ext;
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata || !phy->ref_clk_scale[TX_RFPLL] ||
	    (int_ext != INT_LO && int_ext != EXT_LO))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_use_ext = phy->pdata->use_ext_tx_lo;
	if ((phy->dev_sel == ID_AD9363A) && (int_ext == EXT_LO)) {
		AD9361_DIAG_PRINTF("%s : EXT_LO is not supported by AD9363!\n", __func__);
		return -1;
	}

	phy->pdata->use_ext_tx_lo = int_ext;
	ret = ad9361_clk_mux_set_parent(phy->ref_clk_scale[TX_RFPLL], int_ext);
	/* See RX: a later cleanup error does not undo an already switched mux. */
	if (ret < 0 && phy->ref_clk_scale[TX_RFPLL]->mult != int_ext)
		phy->pdata->use_ext_tx_lo = old_use_ext;
	return ret;
}

/**
 * Set the TX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param fir_cfg FIR filter configuration.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_tx_fir_config_with_cleanup(
	struct ad9361_rf_phy *phy, AD9361_TXFIRConfig fir_cfg,
	int32_t *cleanup_status)
{
	int32_t ret;
	uint8_t old_tx_fir_int;
	if (cleanup_status)
		*cleanup_status = 0;

	if (!phy || !phy->spi)
		return -EINVAL;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	ad9361_invalidate_tx_quad_calib(phy);
	old_tx_fir_int = phy->tx_fir_int;

	phy->fir_config_valid = false;
	phy->tx_fir_int = fir_cfg.tx_int;
	ret = ad9361_load_fir_filter_coef_with_cleanup(
		phy, (enum fir_dest)fir_cfg.tx, fir_cfg.tx_gain,
		fir_cfg.tx_coef_size, fir_cfg.tx_coef, cleanup_status);
	if (ret < 0) {
		phy->tx_fir_int = old_tx_fir_int;
	}
	return ret;
}

int32_t ad9361_set_tx_fir_config(struct ad9361_rf_phy *phy,
				 AD9361_TXFIRConfig fir_cfg)
{
	return ad9361_set_tx_fir_config_with_cleanup(phy, fir_cfg, NULL);
}

/**
 * Get the TX FIR filter configuration.
 * @param phy The AD9361 current state structure.
 * @param tx_ch The selected TX channel (TX1, TX2).
 * 				Accepted values:
 * 				 TX1 (0)
 * 				 TX2 (1)
 * @param fir_cfg FIR filter configuration output file.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_fir_config(struct ad9361_rf_phy *phy, uint8_t tx_ch,
				 AD9361_TXFIRConfig *fir_cfg)
{
	int32_t ret;
	uint32_t fir_conf;
	uint32_t original_fir_conf;
	uint8_t index;
	AD9361_TXFIRConfig result = {0};
	if (!phy || !phy->spi || !fir_cfg || tx_ch > 1)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	tx_ch += 1;

	ret = ad9361_spi_read(phy->spi, REG_TX_FILTER_CONF);
	if (ret < 0)
		return ret;
	original_fir_conf = ret;
	fir_conf = original_fir_conf;
	result.tx_coef_size = (((fir_conf & FIR_NUM_TAPS(7)) >> 5) + 1) * 16;
	result.tx_gain = -6 * (fir_conf & TX_FIR_GAIN_6DB);
	result.tx = tx_ch;

	fir_conf &= ~FIR_SELECT(3);
	fir_conf |= FIR_SELECT(tx_ch) | FIR_START_CLK;
	ret = ad9361_spi_write(phy->spi, REG_TX_FILTER_CONF, fir_conf);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy);
		goto tx_fir_restore;
	}

	for (index = 0; index < 128; index++) {
		ret = ad9361_spi_write(phy->spi, REG_TX_FILTER_COEF_ADDR, index);
		if (ret < 0) {
			ad9361_mark_calibration_state_unknown(phy);
			goto tx_fir_restore;
		}
		ret = ad9361_spi_read(phy->spi, REG_TX_FILTER_COEF_READ_DATA_1);
		if (ret < 0)
			goto tx_fir_restore;
		result.tx_coef[index] = ret;
		ret = ad9361_spi_read(phy->spi, REG_TX_FILTER_COEF_READ_DATA_2);
		if (ret < 0)
			goto tx_fir_restore;
		result.tx_coef[index] |= (ret << 8);
	}

	tx_fir_restore:
	if (ad9361_calibration_state_unknown(phy))
		return ret < 0 ? ret : -EIO;
	{
		int32_t restore_ret = ad9361_spi_write(phy->spi,
						       REG_TX_FILTER_CONF,
						       original_fir_conf & ~FIR_START_CLK);
		if (restore_ret < 0) {
			ad9361_mark_calibration_state_unknown(phy);
			if (ret >= 0)
				ret = restore_ret;
		}
	}
	if (ret < 0)
		return ret;

	result.tx_int = phy->tx_fir_int;
	*fir_cfg = result;

	return 0;
}

/**
 * Enable/disable the TX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_tx_fir_en_dis_with_cleanup(
	struct ad9361_rf_phy *phy, uint8_t en_dis, int32_t *cleanup_status)
{
	int32_t ret = 0;
	bool old_bypass;
	if (cleanup_status)
		*cleanup_status = 0;
	if (!phy || !phy->spi)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_bypass = phy->bypass_tx_fir;
	if (en_dis > 1)
		return -EINVAL;

	if (phy->fir_config_valid && phy->bypass_tx_fir == !en_dis)
		return ret;

	ad9361_invalidate_tx_quad_calib(phy);
	phy->bypass_tx_fir = !en_dis;
	ret = ad9361_validate_enable_fir_with_cleanup(phy, cleanup_status);
	if (ret < 0)
		phy->bypass_tx_fir = old_bypass;

	return ret;
}

int32_t ad9361_set_tx_fir_en_dis(struct ad9361_rf_phy *phy,
				 uint8_t en_dis)
{
	return ad9361_set_tx_fir_en_dis_with_cleanup(phy, en_dis, NULL);
}

/**
 * Get the status of the TX FIR filter.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_fir_en_dis(struct ad9361_rf_phy *phy,
				 uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*en_dis = !phy->bypass_tx_fir;

	return 0;
}

/**
 * Enable or disable the TX power monitor.
 *
 * The monitor powers the measurement block behind REG_TX_RSSI. Until it
 * is enabled those registers read zero, so ad9361_get_tx_rssi() reports
 * 0 dBm at every gain setting - indistinguishable from a transmitter
 * that is producing no output.
 *
 * ad9361_txmon_control() already existed but was static and reachable
 * only through ad9361_rf_port_setup(), that is, only when an RX input is
 * switched to the TX monitor port. Reading TX RSSI during ordinary
 * transmit had no way to turn the block on.
 *
 * @param phy The AD9361 current state structure.
 * @param en_mask Bitmask of TX_1 and TX_2; 0 disables both.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_txmon_enable(struct ad9361_rf_phy *phy, int32_t en_mask)
{
	if (!phy || !phy->spi)
		return -EINVAL;

	return ad9361_txmon_control(phy, en_mask);
}

/**
 * Get the TX RSSI for the selected channel (TX_MON should be enabled).
 * @param phy The AD9361 current state structure.
 * @param ch The desired channel (TX1, TX2).
 * 			 Accepted values:
 * 			  TX1 (0)
 * 			  TX2 (1)
 * @param rssi_db_x_1000 A variable to store the RSSI.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_rssi(struct ad9361_rf_phy *phy,
			   uint8_t ch,
			   uint32_t *rssi_db_x_1000)
{
	uint8_t reg_val_buf[3];
	uint32_t val;
	int32_t tx_channel;
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata || !rssi_db_x_1000 || ch > 1 ||
	    (!phy->pdata->rx2tx2 && ch != 0))
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	ret = ad9361_spi_readm(phy->spi, REG_TX_RSSI_LSB,
			       reg_val_buf, NO_OS_ARRAY_SIZE(reg_val_buf));
	if (ret < 0) {
		return ret;
	}

	tx_channel = ad9361_1rx1tx_channel_map(phy, true, ch + 1);
	switch (tx_channel) {
	case 1:
		val = (reg_val_buf[2] << 1) | (reg_val_buf[0] & TX_RSSI_1);
		break;
	case 2:
		val = (reg_val_buf[1] << 1) | ((reg_val_buf[0] & TX_RSSI_2) >> 1);
		break;
	default:
		return -EINVAL;
	}

	val *= RSSI_RESOLUTION;

	*rssi_db_x_1000 = ((val / RSSI_MULTIPLIER) * 1000) +
			  (val % RSSI_MULTIPLIER);

	return 0;
}

/**
 * Set the TX RF output port.
 * @param phy The AD9361 current state structure.
 * @param mode The RF port.
 * 			   Accepted values:
 *				TXA	(0)
 *				TXB	(1)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_rf_port_output(struct ad9361_rf_phy *phy,
				     uint32_t mode)
{
	int32_t ret;
	uint32_t old_mode;

	if (!phy || !phy->spi || !phy->pdata || mode > 1)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	old_mode = phy->pdata->rf_tx_output_sel;

	phy->pdata->rf_tx_output_sel = mode;

	ret = ad9361_rf_port_setup(phy, true,
				   phy->pdata->rf_rx_input_sel,
				   phy->pdata->rf_tx_output_sel);
	if (ret < 0) {
		phy->pdata->rf_tx_output_sel = old_mode;
		ad9361_invalidate_tx_quad_calib(phy);
	}

	return ret;
}

/**
 * Get the selected TX RF output port.
 * @param phy The AD9361 current state structure.
 * @param mode The RF port.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_rf_port_output(struct ad9361_rf_phy *phy,
				     uint32_t *mode)
{
	if (!phy || !phy->pdata || !mode)
		return -EINVAL;
	if (!phy->rf_port_select_valid)
		return -EIO;

	*mode = phy->pdata->rf_tx_output_sel;

	return 0;
}

/**
 * Enable/disable the auto calibration.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_tx_auto_cal_en_dis(struct ad9361_rf_phy *phy,
				      uint8_t en_dis)
{
	if (!phy || en_dis > 1)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	if (en_dis == 0)
		phy->auto_cal_en = 0;
	else
		phy->auto_cal_en = 1;

	return 0;
}

/**
 * Get the status of the auto calibration flag.
 * @param phy The AD9361 current state structure.
 * @param en_dis The enable/disable status buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_auto_cal_en_dis(struct ad9361_rf_phy *phy,
				      uint8_t *en_dis)
{
	if (!phy || !en_dis)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*en_dis = phy->auto_cal_en;

	return 0;
}

/**
 * Store TX fastlock profile.
 * To create a profile tune the synthesizer (ad9361_set_tx_lo_freq()) and then
 * call this function specifying the target profile number.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_tx_fastlock_store(struct ad9361_rf_phy *phy, uint32_t profile)
{
	return ad9361_fastlock_store(phy, 1, profile);
}

/**
 * Recall specified TX fastlock profile.
 * When in fastlock pin select mode (init_param->tx_fastlock_pincontrol_enable),
 * the function needs to be called before then the pin-control can be used.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_tx_fastlock_recall(struct ad9361_rf_phy *phy, uint32_t profile)
{
	return ad9361_fastlock_recall(phy, 1, profile);
}

/**
 * Load TX fastlock profile. A previously saved profile can be loaded in any
 * of the 8 available slots.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @param values Fastlock profile program data.
 * 				 Example:
 * 				  val0,val1,val2,,val15
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_tx_fastlock_load(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values)
{
	return ad9361_fastlock_load(phy, 1, profile, values);
}

/**
 * Save TX fastlock profile. In order to use more than 8 Profiles, an existing
 * profile can be read back and stored by the user application.
 * @param phy The AD9361 state structure.
 * @param profile The profile number (0 - 7).
 * 				  Accepted values:
 * 				   0 - 7
 * @param values Fastlock profile program data.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_tx_fastlock_save(struct ad9361_rf_phy *phy, uint32_t profile,
				uint8_t *values)
{
	return ad9361_fastlock_save(phy, 1, profile, values);
}

/**
 * Power down the TX Local Oscillator.
 * @param phy The AD9361 state structure.
 * @param option The option (ON, OFF).
 * 				 Accepted values:
 * 				  ON (0)
 * 				  OFF (1)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_tx_lo_powerdown(struct ad9361_rf_phy *phy, uint8_t option)
{
	if (option > 1)
		return -EINVAL;

	return ad9361_synth_lo_powerdown(phy, LO_DONTCARE, option ? LO_OFF : LO_ON);
}

/**
 * Get the TX Local Oscillator power status.
 * @param phy The AD9361 state structure.
 * @param option Store the option (ON, OFF).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_tx_lo_power(struct ad9361_rf_phy *phy, uint8_t *option)
{
	if (!phy || !option)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*option = !(phy->cached_synth_pd[0] & TX_LO_POWER_DOWN) ? OFF : ON;

	return 0;
}

/**
 * Set the RX and TX path rates.
 * @param phy The AD9361 state structure.
 * @param rx_path_clks RX path rates buffer.
 * @param tx_path_clks TX path rates buffer.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_trx_path_clks_with_cleanup(
	struct ad9361_rf_phy *phy, uint32_t *rx_path_clks,
	uint32_t *tx_path_clks, int32_t *cleanup_status_out)
{
	uint32_t old_rx[6], old_tx[6];
	uint32_t old_rx_bw, old_tx_bw;
	int32_t ret, rollback_ret, cleanup_status = 0, child_cleanup_status;

	if (cleanup_status_out)
		*cleanup_status_out = 0;

	if (!phy || !phy->spi || !phy->pdata || !rx_path_clks || !tx_path_clks)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	if (!phy->current_bw_valid) {
		dev_err(&phy->spi->dev,
			"Cannot change clock plan with unknown RF bandwidth state");
		return -EIO;
	}

	ret = ad9361_get_trx_clock_chain(phy, old_rx, old_tx);
	if (ret < 0)
		return ret;
	old_rx_bw = phy->current_rx_bw_Hz;
	old_tx_bw = phy->current_tx_bw_Hz;

	child_cleanup_status = 0;
	ret = ad9361_set_trx_clock_chain_with_cleanup(
		phy, rx_path_clks, tx_path_clks, &child_cleanup_status);
	if (child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto out;

	child_cleanup_status = 0;
	ret = ad9361_update_rf_bandwidth_with_cleanup(
		phy, phy->current_rx_bw_Hz, phy->current_tx_bw_Hz,
		&child_cleanup_status);
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0) {
		rollback_ret = ad9361_restore_clock_chain_after_update_failure(
			phy, old_rx, old_tx, old_rx_bw, old_tx_bw,
			&child_cleanup_status);
		if (cleanup_status == 0 && child_cleanup_status < 0)
			cleanup_status = child_cleanup_status;
		if (rollback_ret < 0 && cleanup_status == 0)
			cleanup_status = rollback_ret;
	}

out:
	if (cleanup_status < 0)
		ad9361_mark_calibration_state_unknown(phy);
	if (cleanup_status_out)
		*cleanup_status_out = cleanup_status;
	if (ret >= 0 && cleanup_status < 0)
		ret = cleanup_status;
	return ret;
}

int32_t ad9361_set_trx_path_clks(struct ad9361_rf_phy *phy,
				 uint32_t *rx_path_clks,
				 uint32_t *tx_path_clks)
{
	return ad9361_set_trx_path_clks_with_cleanup(
		phy, rx_path_clks, tx_path_clks, NULL);
}

/**
 * Get the RX and TX path rates.
 * @param phy The AD9361 state structure.
 * @param rx_path_clks RX path rates buffer.
 * @param tx_path_clks TX path rates buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_trx_path_clks(struct ad9361_rf_phy *phy,
				 uint32_t *rx_path_clks,
				 uint32_t *tx_path_clks)
{
	if (!phy || !rx_path_clks || !tx_path_clks)
		return -EINVAL;

	return ad9361_get_trx_clock_chain(phy, rx_path_clks, tx_path_clks);
}

/**
 * Set the number of channels mode.
 * @param phy The AD9361 state structure.
 * Note: This function also resets the device, some additional
 *       configurations might be necessary
 * @param no_ch_mode Number of channels mode (MODE_1x1, MODE_2x2).
 * 				  Accepted values:
 * 				   MODE_1x1 (1)
 * 				   MODE_2x2 (2)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_no_ch_mode_with_cleanup(
	struct ad9361_rf_phy *phy, uint8_t no_ch_mode,
	int32_t *cleanup_status_out)
{
	int32_t ret, cleanup_status = 0, child_cleanup_status;
	uint32_t rates[NUM_AD9361_CLKS];
	size_t i;
	bool new_rx2tx2, old_rx2tx2;

	if (cleanup_status_out)
		*cleanup_status_out = 0;

	if (!phy || !phy->pdata || !phy->spi || !phy->clk_refin ||
	    !phy->clk_refin->rate)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	switch (no_ch_mode) {
	case 1:
		new_rx2tx2 = false;
		break;
	case 2:
		new_rx2tx2 = true;
		break;
	default:
		return -EINVAL;
	}

#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	if (!phy->adc_conv)
		return -EINVAL;
#endif

	/* Channel-mode reset is destructive. Validate every clock object used by
	 * the subsequent rebuild before changing pdata or touching the RFIC. */
	for (i = 0; i < NUM_AD9361_CLKS; ++i) {
		if (!phy->ref_clk_scale[i] || !phy->clks[i])
			return -EINVAL;
	}

	old_rx2tx2 = phy->pdata->rx2tx2;
	phy->pdata->rx2tx2 = new_rx2tx2;

#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	phy->adc_conv->chip_info = &axiadc_chip_info_tbl[phy->pdata->rx2tx2 ?
						      ID_AD9361 : ID_AD9364];
#endif
	ret = ad9361_reset(phy);
	if (ret < 0)
		goto fail_closed;
	ret = ad9361_spi_write(phy->spi, REG_SPI_CONF,
				       SOFT_RESET | _SOFT_RESET);
	if (ret < 0)
		goto fail_closed;
	ret = ad9361_spi_write(phy->spi, REG_SPI_CONF, 0x0);
	if (ret < 0)
		goto fail_closed;

	ad9361_clear_state(phy);

	/* Rebuild into a temporary clock tree. Legacy recalc wrappers convert a
	 * failed SPI read into rate zero, which made channel-mode changes report
	 * success with a corrupt cached tree. */
#define NO_CH_RECALC(index, call) do { \
	ret = (call); \
	if (ret < 0) \
		goto fail_closed; \
} while (0)
	NO_CH_RECALC(TX_REFCLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[TX_REFCLK], phy->clk_refin->rate,
		&rates[TX_REFCLK]));
	NO_CH_RECALC(RX_REFCLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[RX_REFCLK], phy->clk_refin->rate,
		&rates[RX_REFCLK]));
	NO_CH_RECALC(BB_REFCLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[BB_REFCLK], phy->clk_refin->rate,
		&rates[BB_REFCLK]));
	NO_CH_RECALC(BBPLL_CLK, ad9361_bbpll_recalc_rate_checked(
		phy->ref_clk_scale[BBPLL_CLK], rates[BB_REFCLK],
		&rates[BBPLL_CLK]));
	NO_CH_RECALC(ADC_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[ADC_CLK], rates[BBPLL_CLK], &rates[ADC_CLK]));
	NO_CH_RECALC(R2_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[R2_CLK], rates[ADC_CLK], &rates[R2_CLK]));
	NO_CH_RECALC(R1_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[R1_CLK], rates[R2_CLK], &rates[R1_CLK]));
	NO_CH_RECALC(CLKRF_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[CLKRF_CLK], rates[R1_CLK], &rates[CLKRF_CLK]));
	NO_CH_RECALC(RX_SAMPL_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[RX_SAMPL_CLK], rates[CLKRF_CLK],
		&rates[RX_SAMPL_CLK]));
	NO_CH_RECALC(DAC_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[DAC_CLK], rates[ADC_CLK], &rates[DAC_CLK]));
	NO_CH_RECALC(T2_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[T2_CLK], rates[DAC_CLK], &rates[T2_CLK]));
	NO_CH_RECALC(T1_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[T1_CLK], rates[T2_CLK], &rates[T1_CLK]));
	NO_CH_RECALC(CLKTF_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[CLKTF_CLK], rates[T1_CLK], &rates[CLKTF_CLK]));
	NO_CH_RECALC(TX_SAMPL_CLK, ad9361_clk_factor_recalc_rate_checked(
		phy->ref_clk_scale[TX_SAMPL_CLK], rates[CLKTF_CLK],
		&rates[TX_SAMPL_CLK]));
	NO_CH_RECALC(RX_RFPLL_INT, ad9361_rfpll_int_recalc_rate_checked(
		phy->ref_clk_scale[RX_RFPLL_INT], rates[RX_REFCLK],
		&rates[RX_RFPLL_INT]));
	NO_CH_RECALC(TX_RFPLL_INT, ad9361_rfpll_int_recalc_rate_checked(
		phy->ref_clk_scale[TX_RFPLL_INT], rates[TX_REFCLK],
		&rates[TX_RFPLL_INT]));
	rates[RX_RFPLL_DUMMY] = ad9361_rfpll_dummy_recalc_rate(
		phy->ref_clk_scale[RX_RFPLL_DUMMY]);
	rates[TX_RFPLL_DUMMY] = ad9361_rfpll_dummy_recalc_rate(
		phy->ref_clk_scale[TX_RFPLL_DUMMY]);
	NO_CH_RECALC(RX_RFPLL, ad9361_rfpll_recalc_rate_checked(
		phy->ref_clk_scale[RX_RFPLL], &rates[RX_RFPLL]));
	NO_CH_RECALC(TX_RFPLL, ad9361_rfpll_recalc_rate_checked(
		phy->ref_clk_scale[TX_RFPLL], &rates[TX_RFPLL]));
#undef NO_CH_RECALC
	{
		size_t i;
		for (i = 0; i < NUM_AD9361_CLKS; ++i)
			phy->clks[i]->rate = rates[i];
	}

	child_cleanup_status = 0;
	ret = ad9361_setup_with_cleanup(phy, &child_cleanup_status);
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto fail_closed;
#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	/* platform specific wrapper to call ad9361_post_setup() */
	child_cleanup_status = 0;
	ret = ad9361_post_setup_with_cleanup(phy, &child_cleanup_status);
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto fail_closed;
#endif

	if (cleanup_status < 0) {
		ret = cleanup_status;
		goto fail_closed;
	}
	if (cleanup_status_out)
		*cleanup_status_out = cleanup_status;
	return 0;

fail_closed:
	/* Channel-mode reset is a multi-register transaction. Do not leave the
	 * caller's software mode claiming a configuration that did not complete. */
	phy->pdata->rx2tx2 = old_rx2tx2;
#if !defined(AXI_ADC_NOT_PRESENT) && !defined(BLADERF_NIOS_BUILD)
	phy->adc_conv->chip_info = &axiadc_chip_info_tbl[old_rx2tx2 ?
						      ID_AD9361 : ID_AD9364];
#endif
	ad9361_mark_calibration_state_unknown(phy);
	if (cleanup_status_out)
		*cleanup_status_out = cleanup_status;
	return ret;
}

int32_t ad9361_set_no_ch_mode(struct ad9361_rf_phy *phy, uint8_t no_ch_mode)
{
	return ad9361_set_no_ch_mode_with_cleanup(phy, no_ch_mode, NULL);
}

/**
 * Do multi chip synchronization.
 * @param phy_master The AD9361 Master state structure.
 * @param phy_slave The AD9361 Slave state structure.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_do_mcs(struct ad9361_rf_phy *phy_master,
		      struct ad9361_rf_phy *phy_slave)
{
	uint32_t ensm_mode;
	int32_t step;
	int32_t reg;
	int32_t ret;

	if (!phy_master || !phy_slave || !phy_master->spi || !phy_slave->spi)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy_master) ||
	    ad9361_calibration_state_unknown(phy_slave))
		return -EIO;
	if (ad9361_operation_busy(phy_master) ||
	    ad9361_operation_busy(phy_slave))
		return -EBUSY;
	if (ad9361_operation_busy(phy_master) ||
	    ad9361_operation_busy(phy_slave))
		return -EBUSY;

	if ((phy_master->dev_sel == ID_AD9363A) ||
	    (phy_slave->dev_sel == ID_AD9363A)) {
		AD9361_DIAG_PRINTF("%s : MCS is not supported by AD9363!\n", __func__);
		return -1;
	}

	reg = ad9361_spi_read(phy_master->spi, REG_RX_CLOCK_DATA_DELAY);
	if (reg < 0)
		return reg;
	ret = ad9361_spi_write(phy_slave->spi, REG_RX_CLOCK_DATA_DELAY, reg);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy_slave);
		return ret;
	}
	reg = ad9361_spi_read(phy_master->spi, REG_TX_CLOCK_DATA_DELAY);
	if (reg < 0)
		return reg;
	ret = ad9361_spi_write(phy_slave->spi, REG_TX_CLOCK_DATA_DELAY, reg);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy_slave);
		return ret;
	}

	ret = ad9361_get_en_state_machine_mode(phy_master, &ensm_mode);
	if (ret < 0)
		return ret;

	ret = ad9361_set_en_state_machine_mode(phy_master, ENSM_MODE_ALERT);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy_master);
		ad9361_mark_calibration_state_unknown(phy_slave);
		return ret;
	}
	ret = ad9361_set_en_state_machine_mode(phy_slave, ENSM_MODE_ALERT);
	if (ret < 0) {
		ad9361_mark_calibration_state_unknown(phy_master);
		ad9361_mark_calibration_state_unknown(phy_slave);
		return ret;
	}

	for (step = 0; step <= 5; step++) {
		ret = ad9361_mcs(phy_slave, step);
		if (ret < 0) {
			ad9361_mark_calibration_state_unknown(phy_master);
			ad9361_mark_calibration_state_unknown(phy_slave);
			goto mcs_restore_ensm;
		}
		ret = ad9361_mcs(phy_master, step);
		if (ret < 0) {
			ad9361_mark_calibration_state_unknown(phy_master);
			ad9361_mark_calibration_state_unknown(phy_slave);
			goto mcs_restore_ensm;
		}
		no_os_mdelay(100);
	}

	ret = 0;
mcs_restore_ensm:
	if (!ad9361_calibration_state_unknown(phy_master) &&
	    !ad9361_calibration_state_unknown(phy_slave)) {
		int32_t restore_master = ad9361_set_en_state_machine_mode(
				phy_master, ensm_mode);
		int32_t restore_slave = ad9361_set_en_state_machine_mode(
				phy_slave, ensm_mode);
		if (ret >= 0 && restore_master < 0)
			ret = restore_master;
		if (ret >= 0 && restore_slave < 0)
			ret = restore_slave;
	}

	return ret;
}

/**
 * Enable/disable the TRX FIR filters.
 * @param phy The AD9361 current state structure.
 * @param en_dis The option (ENABLE, DISABLE).
 * 				 Accepted values:
 * 				  ENABLE (1)
 * 				  DISABLE (0)
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_set_trx_fir_en_dis_with_cleanup(
	struct ad9361_rf_phy *phy, uint8_t en_dis, int32_t *cleanup_status)
{
	int32_t ret = 0;
	bool old_rx_bypass;
	bool old_tx_bypass;
	if (cleanup_status)
		*cleanup_status = 0;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	old_rx_bypass = phy->bypass_rx_fir;
	old_tx_bypass = phy->bypass_tx_fir;

	if (en_dis > 1)
		return -EINVAL;

	if (phy->fir_config_valid &&
	    (phy->bypass_rx_fir == phy->bypass_tx_fir) &&
	    (phy->bypass_rx_fir == !en_dis))
		return ret;

	phy->bypass_rx_fir = !en_dis;
	phy->bypass_tx_fir = !en_dis;
	ret = ad9361_validate_enable_fir_with_cleanup(phy, cleanup_status);
	if (ret < 0) {
		phy->bypass_rx_fir = old_rx_bypass;
		phy->bypass_tx_fir = old_tx_bypass;
	}

	return ret;
}

int32_t ad9361_set_trx_fir_en_dis(struct ad9361_rf_phy *phy,
				  uint8_t en_dis)
{
	return ad9361_set_trx_fir_en_dis_with_cleanup(phy, en_dis, NULL);
}

/**
 * Set the OSR rate governor.
 * @param phy The AD9361 current state structure.
 * @param rate_gov OSR rate governor (highest, nominal).
 * 				   Accepted values:
 * 					HIGHEST_OSR (0 - highest OSR)
 * 					NOMINAL_OSR (1 - nominal)
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_set_trx_rate_gov(struct ad9361_rf_phy *phy, uint32_t rate_gov)
{
	if (!phy || rate_gov > 1)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	if (rate_gov == 0)
		phy->rate_governor = 0;
	else
		phy->rate_governor = 1;

	return 0;
}

/**
 * Get the OSR rate governor.
 * @param phy The AD9361 current state structure.
 * @param rate_gov Option buffer.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_trx_rate_gov(struct ad9361_rf_phy *phy, uint32_t *rate_gov)
{
	if (!phy || !rate_gov)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	*rate_gov = phy->rate_governor;

	return 0;
}

/**
 * Perform the selected calibration.
 * @param phy The AD9361 state structure.
 * @param cal The selected calibration (TX_QUAD_CAL, RFDC_CAL).
 * 			  Accepted values:
 * 			   TX_QUAD_CAL
 * 			   RFDC_CAL
 * @param arg For TX_QUAD_CAL - the optional RX phase value overwrite (set to zero).
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_do_calib_with_cleanup(struct ad9361_rf_phy *phy,
				     uint32_t cal, int32_t arg,
				     int32_t *cleanup_status)
{
	if (cleanup_status)
		*cleanup_status = 0;
	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	return ad9361_do_calib_run_with_cleanup(
		phy, cal, arg, cleanup_status);
}

int32_t ad9361_do_calib(struct ad9361_rf_phy *phy, uint32_t cal, int32_t arg)
{
	return ad9361_do_calib_with_cleanup(phy, cal, arg, NULL);
}

/**
 * Load and enable TRX FIR filters configurations.
 * @param phy The AD9361 current state structure.
 * @param rx_fir_cfg RX FIR filter configuration.
 * @param tx_fir_cfg TX FIR filter configuration.
 * @return 0 in case of success, negative error code otherwise.
 *
 * Note: This function will/may affect the data path.
 */
int32_t ad9361_trx_load_enable_fir_with_cleanup(
	struct ad9361_rf_phy *phy, AD9361_RXFIRConfig rx_fir_cfg,
	AD9361_TXFIRConfig tx_fir_cfg, int32_t *cleanup_status_out)
{
	int32_t rtx = -1, rrx = -1, ret;
	int32_t cleanup_status = 0, child_cleanup_status;

	if (cleanup_status_out)
		*cleanup_status_out = 0;

	if (!phy || !phy->spi)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	phy->filt_rx_bw_Hz = 0;
	phy->filt_tx_bw_Hz = 0;
	phy->filt_valid = false;

	if (tx_fir_cfg.tx_path_clks[TX_SAMPL_FREQ]) {
		memcpy(phy->filt_tx_path_clks, tx_fir_cfg.tx_path_clks,
		       sizeof(phy->filt_tx_path_clks));
		rtx = 0;
	}

	if (rx_fir_cfg.rx_path_clks[RX_SAMPL_FREQ]) {
		memcpy(phy->filt_rx_path_clks, rx_fir_cfg.rx_path_clks,
		       sizeof(phy->filt_rx_path_clks));
		rrx = 0;
	}

	if (tx_fir_cfg.tx_bandwidth) {
		phy->filt_tx_bw_Hz = tx_fir_cfg.tx_bandwidth;
	}

	if (rx_fir_cfg.rx_bandwidth) {
		phy->filt_rx_bw_Hz = rx_fir_cfg.rx_bandwidth;
	}

	child_cleanup_status = 0;
	ret = ad9361_set_tx_fir_config_with_cleanup(
		phy, tx_fir_cfg, &child_cleanup_status);
	if (child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto out;

	child_cleanup_status = 0;
	ret = ad9361_set_rx_fir_config_with_cleanup(
		phy, rx_fir_cfg, &child_cleanup_status);
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto out;

	if (!(rrx | rtx))
		phy->filt_valid = true;

	child_cleanup_status = 0;
	ret = ad9361_set_trx_fir_en_dis_with_cleanup(
		phy, 1, &child_cleanup_status);
	if (cleanup_status == 0 && child_cleanup_status < 0)
		cleanup_status = child_cleanup_status;
	if (ret < 0)
		goto out;

out:
	if (cleanup_status_out)
		*cleanup_status_out = cleanup_status;
	if (ret >= 0 && cleanup_status < 0)
		ret = cleanup_status;
	return ret;
}

int32_t ad9361_trx_load_enable_fir(struct ad9361_rf_phy *phy,
				   AD9361_RXFIRConfig rx_fir_cfg,
				   AD9361_TXFIRConfig tx_fir_cfg)
{
	return ad9361_trx_load_enable_fir_with_cleanup(
		phy, rx_fir_cfg, tx_fir_cfg, NULL);
}

/**
 * Do DCXO coarse tuning.
 * @param phy The AD9361 current state structure.
 * @param coarse The DCXO coarse tuning value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_do_dcxo_tune_coarse(struct ad9361_rf_phy *phy,
				   uint32_t coarse)
{
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;

	ret = ad9361_set_dcxo_tune(phy, coarse, phy->pdata->dcxo_fine);
	if (ret >= 0)
		phy->pdata->dcxo_coarse = coarse;
	return ret;
}

/**
 * Do DCXO fine tuning.
 * @param phy The AD9361 current state structure.
 * @param fine The DCXO fine tuning value.
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_do_dcxo_tune_fine(struct ad9361_rf_phy *phy,
				 uint32_t fine)
{
	int32_t ret;

	if (!phy || !phy->spi || !phy->pdata)
		return -EINVAL;

	ret = ad9361_set_dcxo_tune(phy, phy->pdata->dcxo_coarse, fine);
	if (ret >= 0)
		phy->pdata->dcxo_fine = fine;
	return ret;
}

/**
 * Get the temperature.
 * @param phy The AD9361 current state structure.
 * @param temp The temperature (degrees C * 1000).
 * @return 0 in case of success, negative error code otherwise.
 */
int32_t ad9361_get_temperature(struct ad9361_rf_phy *phy,
			       int32_t *temp)
{
	int32_t ret;

	if (!phy || !phy->spi || !temp)
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	ret = ad9361_get_temp(phy);
	if (ret < 0)
		return ret;
	*temp = ret;
	return 0;
}
