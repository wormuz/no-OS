/***************************************************************************//**
 *   @file   ad9361_util.c
 *   @brief  AD9361 Implementation of Util Driver.
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

#include "ad9361_util.h"
#include "string.h"
#include "no_os_delay.h"

#define NO_OS_BITS_PER_LONG		32

/***************************************************************************//**
 * @brief clk_prepare_enable
*******************************************************************************/
int32_t clk_prepare_enable(struct no_os_clk *clk)
{
	if (clk) {
		// Unused variable - fix compiler warning
	}

	return 0;
}

/***************************************************************************//**
 * @brief clk_get_rate
*******************************************************************************/
int32_t clk_get_rate_checked(struct ad9361_rf_phy *phy,
			     struct refclk_scale *clk_priv,
			     uint32_t *rate)
{
	if (!phy || !clk_priv || clk_priv->phy != phy || !rate ||
	    (uint32_t)clk_priv->source >= NUM_AD9361_CLKS)
		return -EINVAL;
	/* A checked clock read can fall through to live divider-register reads.
	 * No rate is authoritative while any RFIC wait/job owns the device: an
	 * RFPLL may be between divider writes, or a calibration may still be
	 * changing the clock-dependent state. */
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;

	switch (clk_priv->source) {
	case TX_REFCLK:
	case RX_REFCLK:
	case BB_REFCLK:
		if (!phy->clk_refin)
			return -EINVAL;
		return ad9361_clk_factor_recalc_rate_checked(
			clk_priv, phy->clk_refin->rate, rate);
	case TX_RFPLL_INT:
	case RX_RFPLL_INT:
		if ((uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
		    !phy->clks[clk_priv->parent_source])
			return -EINVAL;
		return ad9361_rfpll_int_recalc_rate_checked(
			clk_priv, phy->clks[clk_priv->parent_source]->rate, rate);
	case RX_RFPLL_DUMMY:
	case TX_RFPLL_DUMMY:
		*rate = ad9361_rfpll_dummy_recalc_rate(clk_priv);
		return 0;
	case TX_RFPLL:
	case RX_RFPLL:
		return ad9361_rfpll_recalc_rate_checked(clk_priv, rate);
	case BBPLL_CLK:
		if ((uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
		    !phy->clks[clk_priv->parent_source])
			return -EINVAL;
		return ad9361_bbpll_recalc_rate_checked(
			clk_priv, phy->clks[clk_priv->parent_source]->rate, rate);
	case ADC_CLK:
	case R2_CLK:
	case R1_CLK:
	case CLKRF_CLK:
	case RX_SAMPL_CLK:
	case DAC_CLK:
	case T2_CLK:
	case T1_CLK:
	case CLKTF_CLK:
	case TX_SAMPL_CLK:
		if ((uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
		    !phy->clks[clk_priv->parent_source])
			return -EINVAL;
		return ad9361_clk_factor_recalc_rate_checked(
			clk_priv, phy->clks[clk_priv->parent_source]->rate, rate);
	default:
		return -EINVAL;
	}
}

uint32_t clk_get_rate(struct ad9361_rf_phy *phy,
		      struct refclk_scale *clk_priv)
{
	uint32_t rate = 0;

	(void)clk_get_rate_checked(phy, clk_priv, &rate);
	return rate;
}

/* A failed clock write or divider readback leaves that clock's hardware
 * state uncertain. Invalidate its derived clocks too: otherwise a later
 * request matching an old child rate can skip programming even though its
 * parent was partially changed. */
static void ad9361_invalidate_clock_rate_subtree(
	struct ad9361_rf_phy *phy, uint32_t root)
{
	bool invalid[NUM_AD9361_CLKS] = { false };
	bool changed;
	int32_t i;

	if (!phy || root >= NUM_AD9361_CLKS)
		return;

	invalid[root] = true;
	do {
		changed = false;
		for (i = 0; i < NUM_AD9361_CLKS; i++) {
			uint32_t parent;

			if (invalid[i] || !phy->ref_clk_scale[i])
				continue;
			parent = phy->ref_clk_scale[i]->parent_source;
			if (parent < NUM_AD9361_CLKS && invalid[parent]) {
				invalid[i] = true;
				changed = true;
			}
		}
	} while (changed);

	for (i = 0; i < NUM_AD9361_CLKS; i++) {
		if (!invalid[i])
			continue;
		if (phy->clks[i])
			phy->clks[i]->rate = 0;
		if (i == BBPLL_CLK)
			phy->bbpll_initialized = false;
		if (i == RX_RFPLL)
			phy->rfpll_requested_valid[0] = false;
		if (i == TX_RFPLL)
			phy->rfpll_requested_valid[1] = false;
	}
}

static int32_t ad9361_clock_rate_update_failed(
	struct ad9361_rf_phy *phy, uint32_t source, int32_t status)
{
	ad9361_invalidate_clock_rate_subtree(phy, source);
	return status;
}

void ad9361_clock_rate_invalidate(struct ad9361_rf_phy *phy,
				 uint32_t source)
{
	ad9361_invalidate_clock_rate_subtree(phy, source);
}

int32_t ad9361_bbpll_clock_transaction_start(
	struct ad9361_rf_phy *phy, struct refclk_scale *clk_priv,
	uint32_t rate, struct ad9361_bbpll_clock_transaction *transaction)
{
	uint32_t parent_rate, rounded_rate;
	int32_t ret;

	if (!phy || !clk_priv || !transaction || !rate ||
	    clk_priv->phy != phy || clk_priv->source != BBPLL_CLK ||
	    !phy->clk_refin || !phy->clks[BBPLL_CLK] ||
	    (uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
	    !phy->clks[clk_priv->parent_source])
		return -EINVAL;
	if (transaction->active || transaction->rate_transaction.active ||
	    transaction->rate_transaction.wait.active)
		return -EBUSY;
	memset(transaction, 0, sizeof(*transaction));
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	parent_rate = phy->clks[clk_priv->parent_source]->rate;
	if (parent_rate == 0)
		return -EINVAL;
	rounded_rate = ad9361_bbpll_round_rate(clk_priv, rate, &parent_rate);
	if ((int32_t)rounded_rate < 0)
		return (int32_t)rounded_rate;

	transaction->phy = phy;
	transaction->clk_priv = clk_priv;
	transaction->rounded_rate = rounded_rate;

	ret = ad9361_bbpll_rate_transaction_start(
		clk_priv, rounded_rate, parent_rate,
		&transaction->rate_transaction);
	if (ret < 0) {
		transaction->phy = NULL;
		transaction->clk_priv = NULL;
		return ret;
	}
	transaction->rate_transaction.retain_owner_on_terminal = true;

	/* The old cached rate ceases to describe the RFIC as soon as this
	 * transaction is accepted. Publish it again only after lock and readback. */
	ad9361_invalidate_clock_rate_subtree(phy, BBPLL_CLK);
	transaction->active = true;
	transaction->terminal_status = -EINPROGRESS;
	return 0;
}

static void ad9361_bbpll_clock_transaction_release_owner(
	struct ad9361_bbpll_clock_transaction *transaction)
{
	if (transaction->phy &&
	    transaction->phy->bbpll_rate_transaction ==
	    &transaction->rate_transaction)
		transaction->phy->bbpll_rate_transaction = NULL;
}

int32_t ad9361_bbpll_clock_transaction_poll(
	struct ad9361_rf_phy *phy,
	struct ad9361_bbpll_clock_transaction *transaction)
{
	uint32_t readback_rate;
	int32_t ret;

	if (!phy || !transaction)
		return -EINVAL;
	if (transaction->terminal)
		return transaction->phy == phy ? transaction->terminal_status :
		       -EINVAL;
	if (!transaction->active || transaction->phy != phy ||
	    !transaction->clk_priv || transaction->clk_priv->phy != phy ||
	    transaction->clk_priv->source != BBPLL_CLK) {
		if (transaction->rate_transaction.phy)
			(void)ad9361_bbpll_rate_transaction_poll(
				phy, &transaction->rate_transaction);
		if (transaction->phy)
			ad9361_clock_rate_update_failed(
				transaction->phy, BBPLL_CLK, -EIO);
		ad9361_bbpll_clock_transaction_release_owner(transaction);
		transaction->active = false;
		transaction->terminal = true;
		transaction->terminal_status = -EIO;
		return -EIO;
	}

	if (!transaction->verify_pending) {
		ret = ad9361_bbpll_rate_transaction_poll(
			phy, &transaction->rate_transaction);
		if (ret == 1)
			return 1;
		if (ret < 0) {
			transaction->active = false;
			transaction->terminal = true;
			transaction->terminal_status =
				ad9361_clock_rate_update_failed(phy, BBPLL_CLK, ret);
			ad9361_bbpll_clock_transaction_release_owner(transaction);
			return transaction->terminal_status;
		}
		/* Keep the low-level owner while divider readback runs in its own
		 * cooperative poll, so no other AD9361 user can interleave here. */
		transaction->verify_pending = true;
		return 1;
	}

	ret = ad9361_bbpll_recalc_rate_checked(
		transaction->clk_priv,
		phy->clks[transaction->clk_priv->parent_source]->rate,
		&readback_rate);
	if (ret < 0) {
		transaction->active = false;
		transaction->terminal = true;
		transaction->terminal_status =
			ad9361_clock_rate_update_failed(phy, BBPLL_CLK, ret);
		ad9361_bbpll_clock_transaction_release_owner(transaction);
		return transaction->terminal_status;
	}

	phy->clks[BBPLL_CLK]->rate = readback_rate;
	phy->bbpll_initialized = true;
	ad9361_bbpll_clock_transaction_release_owner(transaction);
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = 0;
	return 0;
}

int32_t ad9361_bbpll_clock_set_rate(struct ad9361_rf_phy *phy,
				   struct refclk_scale *clk_priv,
				   uint32_t rate)
{
	struct ad9361_bbpll_clock_transaction transaction = {0};
	int32_t ret;

	ret = ad9361_bbpll_clock_transaction_start(
		phy, clk_priv, rate, &transaction);
	if (ret < 0)
		return ret;

	for (;;) {
		struct no_os_time now;
		uint64_t now_us, remaining_us;

		ret = ad9361_bbpll_clock_transaction_poll(phy, &transaction);
		if (ret != 1)
			return ret;
		if (!transaction.rate_transaction.wait_started)
			continue;
		now = no_os_get_time();
		now_us = (uint64_t)now.s * 1000000ULL + now.us;
		remaining_us =
			transaction.rate_transaction.wait.poll_not_before_us > now_us ?
			transaction.rate_transaction.wait.poll_not_before_us - now_us :
			0;
		if (remaining_us != 0)
			no_os_udelay(remaining_us >
				     transaction.rate_transaction.wait.poll_delay_us ?
				     transaction.rate_transaction.wait.poll_delay_us :
				     (uint32_t)remaining_us);
	}
}

int32_t ad9361_rfpll_clock_transaction_start(
	struct ad9361_rf_phy *phy, struct refclk_scale *clk_priv,
	uint32_t rate, struct ad9361_rfpll_clock_transaction *transaction)
{
	struct refclk_scale *int_clk;
	uint32_t parent_rate;
	uint64_t requested_hz;
	uint64_t min_hz;
	uint8_t tx;
	int32_t rounded_rate;
	int32_t ret;

	if (!phy || !phy->pdata || !clk_priv || !transaction || !rate ||
	    clk_priv->phy != phy ||
	    (clk_priv->source != RX_RFPLL && clk_priv->source != TX_RFPLL) ||
	    !phy->clks[clk_priv->source])
		return -EINVAL;
	if (transaction->active || transaction->rate_transaction.active ||
	    transaction->rate_transaction.wait.active)
		return -EBUSY;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;

	tx = clk_priv->source == TX_RFPLL;
	if (tx ? phy->pdata->use_ext_tx_lo : phy->pdata->use_ext_rx_lo)
		return -ENOTSUP;
	requested_hz = ad9361_from_clk(rate);
	min_hz = tx ? MIN_TX_CARRIER_FREQ_HZ : MIN_RX_CARRIER_FREQ_HZ;
	if (requested_hz < min_hz || requested_hz > MAX_CARRIER_FREQ_HZ)
		return -EINVAL;
	int_clk = phy->ref_clk_scale[tx ? TX_RFPLL_INT : RX_RFPLL_INT];
	if (!int_clk || int_clk->phy != phy ||
	    int_clk->source != (tx ? TX_RFPLL_INT : RX_RFPLL_INT) ||
	    (uint32_t)int_clk->parent_source >= NUM_AD9361_CLKS ||
	    !phy->clks[int_clk->parent_source])
		return -EINVAL;
	parent_rate = phy->clks[int_clk->parent_source]->rate;
	if (!parent_rate)
		return -EINVAL;

	/* The internal RFPLL round-rate path range-checks and preserves the
	 * requested shifted rate, matching clk_set_rate()'s existing behavior. */
	rounded_rate = ad9361_rfpll_int_round_rate(int_clk, rate,
							 &parent_rate);
	if (rounded_rate < 0 && rate <= INT32_MAX)
		return rounded_rate;

	memset(transaction, 0, sizeof(*transaction));
	transaction->phy = phy;
	transaction->clk_priv = clk_priv;
	transaction->source = (uint8_t)clk_priv->source;
	transaction->requested_rate = rate;
	transaction->rounded_rate = (uint32_t)rounded_rate;
	if (phy->rfpll_requested_valid[tx] &&
	    phy->rfpll_requested[tx] == transaction->rounded_rate) {
		transaction->terminal = true;
		transaction->terminal_status = 0;
		return 0;
	}

	phy->rfpll_requested_valid[tx] = false;
	ret = ad9361_rfpll_rate_transaction_start(
		int_clk, transaction->rounded_rate, parent_rate,
		&transaction->rate_transaction);
	if (ret < 0) {
		transaction->terminal = true;
		transaction->terminal_status =
			ad9361_clock_rate_update_failed(phy, clk_priv->source, ret);
		return transaction->terminal_status;
	}
	transaction->active = true;
	return 0;
}

int32_t ad9361_rfpll_clock_transaction_poll(
	struct ad9361_rf_phy *phy,
	struct ad9361_rfpll_clock_transaction *transaction)
{
	uint8_t tx;
	int32_t ret;

	if (!phy || !transaction)
		return -EINVAL;
	if (transaction->terminal)
		return transaction->phy == phy ? transaction->terminal_status :
		       -EINVAL;
	if (!transaction->active || transaction->phy != phy ||
	    !transaction->clk_priv || transaction->clk_priv->phy != phy ||
	    transaction->source != transaction->clk_priv->source ||
	    (transaction->source != RX_RFPLL &&
	     transaction->source != TX_RFPLL)) {
		if (transaction->phy && transaction->source < NUM_AD9361_CLKS)
			ad9361_clock_rate_update_failed(
				transaction->phy, transaction->source, -EIO);
		transaction->active = false;
		transaction->terminal = true;
		transaction->terminal_status = -EIO;
		return -EIO;
	}

	ret = ad9361_rfpll_rate_transaction_poll(
		phy, &transaction->rate_transaction);
	if (ret == 1)
		return 1;
	if (ret < 0)
		goto fail;

	ret = ad9361_rfpll_recalc_rate_checked(
		transaction->clk_priv,
		&phy->clks[transaction->source]->rate);
	if (ret < 0)
		goto fail;
	tx = transaction->source == TX_RFPLL;
	phy->rfpll_requested[tx] = transaction->rounded_rate;
	phy->rfpll_requested_valid[tx] = true;
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = 0;
	return 0;

fail:
	transaction->active = false;
	transaction->terminal = true;
	transaction->terminal_status = ad9361_clock_rate_update_failed(
		phy, transaction->source, ret);
	return transaction->terminal_status;
}

/***************************************************************************//**
 * @brief clk_set_rate
*******************************************************************************/
int32_t clk_set_rate(struct ad9361_rf_phy *phy,
		     struct refclk_scale *clk_priv,
		     uint32_t rate)
{
	uint32_t source;
	int32_t i, ret;
	uint32_t round_rate;

	if (!phy || !clk_priv || clk_priv->phy != phy || !rate ||
	    (uint32_t)clk_priv->source >= NUM_AD9361_CLKS ||
	    !phy->clks[clk_priv->source])
		return -EINVAL;
	if (ad9361_calibration_state_unknown(phy))
		return -EIO;
	if (ad9361_operation_busy(phy))
		return -EBUSY;
	if (!phy->clk_refin)
		return -EINVAL;
	for (i = BB_REFCLK; i < NUM_AD9361_CLKS; i++) {
		if (!phy->clks[i] || !phy->ref_clk_scale[i])
			return -EINVAL;
	}
	source = clk_priv->source;
	/* RFPLL rates have a separate requested-rate cache below. Always let
	 * those requests reach it: the reported rate is a divider readback, and
	 * after an LO mux change it may still describe the previous source. */
	if (phy->clks[source]->rate != rate || source == RX_RFPLL ||
	    source == TX_RFPLL) {
		switch (source) {
		case TX_REFCLK:
		case RX_REFCLK:
		case BB_REFCLK:
			if (!phy->clk_refin)
				return -EINVAL;
			round_rate = ad9361_clk_factor_round_rate(clk_priv, rate,
					&phy->clk_refin->rate);
			if ((int32_t)round_rate < 0)
				return (int32_t)round_rate;
			ret = ad9361_clk_factor_set_rate(clk_priv, round_rate,
							 phy->clk_refin->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			ret = ad9361_clk_factor_recalc_rate_checked(
				clk_priv, phy->clk_refin->rate,
				&phy->clks[source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			break;
		case TX_RFPLL_INT:
		case RX_RFPLL_INT:
			if ((uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
			    !phy->clks[clk_priv->parent_source])
				return -EINVAL;
			/* This round-rate helper also returns a shifted RFPLL rate
			 * through int32_t. Validate the request in its unsigned
			 * frequency domain before retaining that legacy bit pattern. */
			{
				uint64_t requested_hz = ad9361_from_clk(rate);
				uint64_t min_hz = source == RX_RFPLL_INT ?
					MIN_RX_CARRIER_FREQ_HZ : MIN_TX_CARRIER_FREQ_HZ;

				if (requested_hz < min_hz ||
				    requested_hz > MAX_CARRIER_FREQ_HZ)
					return ad9361_clock_rate_update_failed(
						phy, source, -EINVAL);
			}
			round_rate = ad9361_rfpll_int_round_rate(clk_priv, rate,
					&phy->clks[clk_priv->parent_source]->rate);
			ret = ad9361_rfpll_int_set_rate(clk_priv, round_rate,
							phy->clks[clk_priv->parent_source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			ret = ad9361_rfpll_int_recalc_rate_checked(
				clk_priv, phy->clks[clk_priv->parent_source]->rate,
				&phy->clks[source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			break;
		case RX_RFPLL_DUMMY:
		case TX_RFPLL_DUMMY:
			ad9361_rfpll_dummy_set_rate(clk_priv, rate);
			break;
		case TX_RFPLL:
		case RX_RFPLL:
			/* The historical round-rate API returns the encoded RFPLL
			 * rate through int32_t, so valid rates above INT32_MAX appear
			 * negative. Range-check the internal request before preserving
			 * that bit pattern as uint32_t; otherwise an out-of-range input
			 * can turn a negative error into an enormous hardware rate. */
			if (!(source == RX_RFPLL ? phy->pdata->use_ext_rx_lo :
			      phy->pdata->use_ext_tx_lo)) {
				uint64_t requested_hz = ad9361_from_clk(rate);
				uint64_t min_hz = source == RX_RFPLL ?
					MIN_RX_CARRIER_FREQ_HZ : MIN_TX_CARRIER_FREQ_HZ;

				if (requested_hz < min_hz ||
				    requested_hz > MAX_CARRIER_FREQ_HZ)
					return ad9361_clock_rate_update_failed(
						phy, source, -EINVAL);
			}
			round_rate = (uint32_t)ad9361_rfpll_round_rate(clk_priv, rate);
			/* The guard above compares the request against
			 * phy->clks[source]->rate, which for these two sources is
			 * read back from the synthesiser dividers rather than
			 * remembered from the request. Whenever the dividers cannot
			 * hit the request exactly the two never match, so the RF PLL
			 * is reprogrammed on every call - including calls that ask
			 * for the frequency that is already tuned.
			 *
			 * Comparing against round_rate does not help: for the
			 * internal LO ad9361_rfpll_round_rate() forwards to
			 * ad9361_rfpll_int_round_rate(), which only range-checks and
			 * returns the request unchanged. Remember what was actually
			 * programmed instead and leave the read-back cache alone, so
			 * clk_get_rate() keeps reporting the achievable rate. */
			if (phy->rfpll_requested_valid[source == TX_RFPLL] &&
			    phy->rfpll_requested[source == TX_RFPLL] ==
			    (uint32_t)round_rate)
				break;
			/* The RFIC can be partially reprogrammed even when set_rate()
			 * or the following divider readback fails. Do not let a later
			 * request match the old cache and skip recovery programming. */
			phy->rfpll_requested_valid[source == TX_RFPLL] = false;
			ret = ad9361_rfpll_set_rate(clk_priv, round_rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			ret = ad9361_rfpll_recalc_rate_checked(clk_priv,
							&phy->clks[source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			phy->rfpll_requested[source == TX_RFPLL] =
				(uint32_t)round_rate;
			phy->rfpll_requested_valid[source == TX_RFPLL] = true;
			break;
		case BBPLL_CLK:
			ret = ad9361_bbpll_clock_set_rate(phy, clk_priv, rate);
			if (ret < 0)
				return ret;
			break;
		case ADC_CLK:
		case R2_CLK:
		case R1_CLK:
		case CLKRF_CLK:
		case RX_SAMPL_CLK:
		case DAC_CLK:
		case T2_CLK:
		case T1_CLK:
		case CLKTF_CLK:
		case TX_SAMPL_CLK:
			if ((uint32_t)clk_priv->parent_source >= NUM_AD9361_CLKS ||
			    !phy->clks[clk_priv->parent_source])
				return -EINVAL;
			round_rate = ad9361_clk_factor_round_rate(clk_priv, rate,
					&phy->clks[clk_priv->parent_source]->rate);
			if ((int32_t)round_rate < 0)
				return (int32_t)round_rate;
			ret = ad9361_clk_factor_set_rate(clk_priv, round_rate,
							 phy->clks[clk_priv->parent_source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			ret = ad9361_clk_factor_recalc_rate_checked(
				clk_priv, phy->clks[clk_priv->parent_source]->rate,
				&phy->clks[source]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, source, ret);
			break;
		default:
			return -EINVAL;
		}
		for (i = BB_REFCLK; i < BBPLL_CLK; i++) {
			ret = ad9361_clk_factor_recalc_rate_checked(
				phy->ref_clk_scale[i], phy->clk_refin->rate,
				&phy->clks[i]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, i, ret);
		}
		ret = ad9361_bbpll_recalc_rate_checked(
			phy->ref_clk_scale[BBPLL_CLK],
			phy->clks[phy->ref_clk_scale[BBPLL_CLK]->parent_source]->rate,
			&phy->clks[BBPLL_CLK]->rate);
		if (ret < 0)
			return ad9361_clock_rate_update_failed(phy, BBPLL_CLK, ret);
		for (i = ADC_CLK; i < RX_RFPLL_INT; i++) {
			ret = ad9361_clk_factor_recalc_rate_checked(
				phy->ref_clk_scale[i],
				phy->clks[phy->ref_clk_scale[i]->parent_source]->rate,
				&phy->clks[i]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, i, ret);
		}
		for (i = RX_RFPLL_INT; i < RX_RFPLL_DUMMY; i++) {
			ret = ad9361_rfpll_int_recalc_rate_checked(
				phy->ref_clk_scale[i],
				phy->clks[phy->ref_clk_scale[i]->parent_source]->rate,
				&phy->clks[i]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, i, ret);
		}
		for (i = RX_RFPLL_DUMMY; i < RX_RFPLL; i++) {
			phy->clks[i]->rate = ad9361_rfpll_dummy_recalc_rate(phy->ref_clk_scale[i]);
		}
		for (i = RX_RFPLL; i < NUM_AD9361_CLKS; i++) {
			ret = ad9361_rfpll_recalc_rate_checked(
				phy->ref_clk_scale[i], &phy->clks[i]->rate);
			if (ret < 0)
				return ad9361_clock_rate_update_failed(phy, i, ret);
		}
	} else {
		if ((source == BBPLL_CLK) && !phy->bbpll_initialized) {
			ret = ad9361_bbpll_clock_set_rate(phy, clk_priv, rate);
			if (ret < 0)
				return ret;
		}
	}

	return 0;
}

/***************************************************************************//**
 * @brief int_sqrt
*******************************************************************************/
uint32_t int_sqrt(uint32_t x)
{
	uint32_t b, m, y = 0;

	if (x <= 1)
		return x;

	m = 1UL << (NO_OS_BITS_PER_LONG - 2);
	while (m != 0) {
		b = y + m;
		y >>= 1;

		if (x >= b) {
			x -= b;
			y += m;
		}
		m >>= 2;
	}

	return y;
}

/***************************************************************************//**
 * @brief ilog2
*******************************************************************************/
int32_t ilog2(int32_t x)
{
	int32_t A = !(!(x >> 16));
	int32_t count = 0;
	int32_t x_copy = x;

	count = count + (A << 4);

	x_copy = (((~A + 1) & (x >> 16)) + (~(~A + 1) & x));

	A = !(!(x_copy >> 8));
	count = count + (A << 3);
	x_copy = (((~A + 1) & (x_copy >> 8)) + (~(~A + 1) & x_copy));

	A = !(!(x_copy >> 4));
	count = count + (A << 2);
	x_copy = (((~A + 1) & (x_copy >> 4)) + (~(~A + 1) & x_copy));

	A = !(!(x_copy >> 2));
	count = count + (A << 1);
	x_copy = (((~A + 1) & (x_copy >> 2)) + (~(~A + 1) & x_copy));

	A = !(!(x_copy >> 1));
	count = count + A;

	return count;
}

/***************************************************************************//**
 * @brief find_first_bit
*******************************************************************************/
uint32_t find_first_bit(uint32_t word)
{
	int32_t num = 0;

	if ((word & 0xffff) == 0) {
		num += 16;
		word >>= 16;
	}
	if ((word & 0xff) == 0) {
		num += 8;
		word >>= 8;
	}
	if ((word & 0xf) == 0) {
		num += 4;
		word >>= 4;
	}
	if ((word & 0x3) == 0) {
		num += 2;
		word >>= 2;
	}
	if ((word & 0x1) == 0)
		num += 1;
	return num;
}

/***************************************************************************//**
 * @brief ERR_PTR
*******************************************************************************/
void * ERR_PTR(long error)
{
	return (void *) error;
}
