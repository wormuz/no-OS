/***************************************************************************//**
 *   @file   dma_example.c
 *   @brief  DMA capture example for the ad9088 project
 *   @author CHegbeli (ciprian.hegbeli@analog.com)
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

/*
 * What this example does, in the order dma_example_main() does it:
 *
 *   1-10.  Bring up the board -- three clock chips, both transceivers, both
 *          AXI JESD204 cores and the AD9088 -- then run the JESD204 FSM until
 *          the link carries data.
 *   11-13. Derive the capture geometry and the sample rates from the profile
 *          the FSM populated rather than hardcoding them, and put every NCO on
 *          a common default so a tone arrives where it was sent.
 *   14.    Three measurements, weakest claim first:
 *
 *          a) Noise floor, datapath idle. First on purpose: coherence here
 *             should collapse to roughly NO_OS_TONE_SCALE/N, which is what
 *             shows the estimator rejects noise instead of scoring whatever it
 *             is handed. Without (a), a pass in (b) proves nothing.
 *          b) RX FNCO test tone. Test mode injects a constant ahead of the
 *             FDDC mixer, so this covers the receive datapath alone -- the
 *             tone never reaches the DAC or the cables.
 *          c) Cabled DAC -> ADC loopback. Drives the DAC from DMA and captures
 *             what comes back, the first measurement covering the whole chain,
 *             scored on every channel both links can carry.
 *
 *   15.    Park in a busy loop.
 *
 * HARDWARE: (c) needs a physical cable from a DAC output to an ADC input. With
 * no cable it fails, and that failure is not a bug.
 *
 * The example deliberately never returns on success. It parks at step 15 so
 * tools/scripts/platform/xilinx/capture.tcl can read adc_buffer_dma back over
 * JTAG; returning would let the startup code re-zero .bss and wipe the capture.
 */

#include "dma_example.h"
#include "common_data.h"
#include "no_os_delay.h"
#include "no_os_print_log.h"
#include "no_os_tone.h"
#include "no_os_util.h"
#include "ad9088.h"
#include "adi_apollo_cddc.h"
#include "adi_apollo_fddc.h"
#include "adi_apollo_cduc.h"
#include "adi_apollo_fduc.h"
#include "jesd204.h"
#include "axi_adxcvr.h"
#include "axi_adc_core.h"
#include "axi_dac_core.h"
#include "axi_dmac.h"
#include "no_os_axi_io.h"
#include "jesd204_clk.h"
#include "parameters.h"
#include "xil_cache.h"

/* Side and datapath the test tone is injected on */
#define TEST_TONE_SIDE		0
#define TEST_TONE_FDDC		0
/* Mid-scale test tone amplitude, within the RX FNCO 0x1FFF limit */
#define TEST_TONE_OFFSET	0x1000

/*
 * Test tone frequency as a divisor of the capture rate rather than an absolute
 * value, so it stays inside the FDDC passband whatever the profile decimates by.
 * A power of two keeps the tone an integer number of Hz whenever the divisor
 * divides the rate, and keeps the tone module's phase step exact: the step is
 * (freq << 32) / rate, so a divisor D gives 2^32 / D, which for any power of
 * two up to the sine table length is an exact multiple of the accumulator's
 * fractional field. The phase then lands on a table entry every sample rather
 * than between two of them.
 */
#define TEST_TONE_RATE_DIV	256

/*
 * Default coarse NCO, as a divisor of the DAC rate. The fractional part of the
 * frequency tuning word is discarded unless dual modulus mode is on (see
 * adi_ad9088_calc_nco_ftw()), so only frequencies that divide the DAC rate
 * exactly are tuned without residual error -- a power of two guarantees it.
 */
#define DEFAULT_CNCO_RATE_DIV	8
#define DEFAULT_FNCO_HZ		0

/*
 * Converter pair the FDDC under test lands on. The FDDC emits a complex tone,
 * so its I and Q arrive on two adjacent converters of the JESD204 link.
 */
#define TONE_CONV_I		0
#define TONE_CONV_Q		1

/*
 * Half scale for the transmitted tone. Cable and converter loss leave the
 * received tone well inside range at this level, and driving any lower only
 * costs signal-to-noise -- which the envelope spread, a min/max statistic, is
 * the first threshold to feel.
 */
#define LOOPBACK_TX_AMPLITUDE	16384

/*
 * The TX data offload replays its whole BRAM regardless of how much was written
 * into it, so anything left unwritten comes back as noise. This bounds the
 * static buffer; the depth the offload actually has is read back from its
 * memory size register at runtime and the transfer clamped to the smaller.
 */
#define TX_OFFLOAD_BRAM_BYTES		(512 * 1024)
#define AXI_DO_REG_MEMORY_SIZE_LSB	0x0014

/*
 * Width of the TX DMAC source AXI data path in bytes, which has to match the
 * HDL build. The driver rejects a source address that is not a multiple of it;
 * the transfer size is floored to the same boundary so the last beat is whole.
 */
#define DMA_SRC_WIDTH_BYTES	128

/* Largest M the JESD204 link can report (ADI_APOLLO_CONV_PER_LINK_16) */
#define MAX_LINK_CONVERTERS	16

#define DMA_BUFFER_ALIGN	1024

/*
 * Peak of a signed converter sample, to report a level in context. Presumes the
 * 16-bit sample width the link reports as NP, which the capture geometry line
 * prints; a narrower NP would need scaling.
 */
#define SAMPLE_FULL_SCALE	32767

/*
 * Fixed-capacity static capture buffer. Samples-per-converter is derived from
 * the link's M at runtime and clamped to what fits here, so a profile with more
 * converters shortens the capture instead of overrunning the buffer.
 */
static uint16_t adc_buffer_dma[ADC_BUFFER_SAMPLES * 8]
__attribute__((aligned(DMA_BUFFER_ALIGN)));

/* Sized to the whole TX offload BRAM, see TX_OFFLOAD_BRAM_BYTES. */
static uint16_t dac_buffer_dma[TX_OFFLOAD_BRAM_BYTES / sizeof(uint16_t)]
__attribute__((aligned(DMA_BUFFER_ALIGN)));

/*
 * Shared by all three measurements: only the fields that distinguish them are
 * set at the call site, so a threshold cannot drift between tests.
 */
static const struct no_os_tone_limits dma_example_limits = {
	.coherence_min	= NO_OS_TONE_COHERENCE_PASS,
	.spread_max	= NO_OS_TONE_SPREAD_MAX,
};

/**
 * @struct dma_example_meas
 * @brief What the measurements need, gathered once after bring-up.
 *
 * Inputs only. dma_example_main() keeps ownership of every handle in here and
 * stays responsible for removing them; nothing in this struct is allocated or
 * freed, and no measurement writes to a field another one reads. It exists so
 * each measurement takes a short argument list instead of the nine or more
 * parameters the values would otherwise have to travel as.
 */
struct dma_example_meas {
	/** AD9088 device. */
	struct ad9088_phy		*phy;
	/** RX DMA controller, source of every capture. */
	struct axi_dmac			*rx_dmac;
	/** TX DMA controller, replays the loopback tone. */
	struct axi_dmac			*tx_dmac;
	/** TX TPL core, switched between zero and DMA. */
	struct axi_dac			*tx_dac;

	/* Receive geometry, derived from the link the FSM brought up. */
	/** Converter pair the captured tone lands on. */
	struct no_os_tone_layout	rx_layout;
	/** Converters the receive link carries. */
	uint8_t				num_conv;
	/** Sample width the receive link reports. */
	uint8_t				np;
	/** Samples captured per converter. */
	uint32_t			samples;
	/** Capture transfer size in bytes. */
	uint32_t			transfer_size;
	/** Rate the captured samples arrive at, in Hz. */
	uint64_t			capture_rate;

	/* Transmit geometry -- the transmit link carries its own M. */
	/** Converter pair the transmitted tone is written to. */
	struct no_os_tone_layout	tx_layout;
	/** Converters the transmit link carries. */
	uint8_t				tx_num_conv;
	/** Rate the transmitted samples leave at, in Hz. */
	uint64_t			tx_rate;
	/** Samples per converter in the replay buffer. */
	uint32_t			tx_samples;
	/** Replay transfer size in bytes. */
	uint32_t			tx_size;

	/** Complex channels both links can carry end to end. */
	uint8_t				num_ch;
	/** Frequency every measurement is scored at, in Hz. */
	int64_t				tone_hz;
};

/**
 * @brief Capture one buffer from the RX DMAC into DDR.
 * @param rx_dmac - RX DMA controller.
 * @param size - Transfer size in bytes.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_capture(struct axi_dmac *rx_dmac, uint32_t size)
{
	struct axi_dma_transfer read_transfer = {
		.size = size,
		.transfer_done = 0,
		.cyclic = NO,
		.src_addr = 0,
		.dest_addr = (uintptr_t)adc_buffer_dma,
	};
	int ret;

	ret = axi_dmac_transfer_start(rx_dmac, &read_transfer);
	if (ret) {
		pr_err("RX DMA transfer start failed (%d)\n", ret);
		return ret;
	}

	ret = axi_dmac_transfer_wait_completion(rx_dmac, 1000);
	if (ret) {
		pr_err("RX DMA transfer timed out (%d)\n", ret);
		return ret;
	}

	/*
	 * main() enables the data cache unconditionally, so the CPU would
	 * otherwise read stale lines instead of what the DMA just wrote.
	 */
	Xil_DCacheInvalidateRange((uintptr_t)adc_buffer_dma, size);

	return 0;
}

/**
 * @brief Print which datapaths the profile put on the links.
 *
 * The channel-to-datapath mapping is a property of the profile binary and can
 * not be inferred from the outside, so it gets dumped rather than assumed. What
 * to read from it: an FDDC carrying `link 0` with its clocks on is on the
 * capture, the crossbar lines say which converter of the link its I and Q land
 * on, and the CDDC/CDUC a channel belongs to is its FDDC/FDUC index halved.
 *
 * @param phy - AD9088 device.
 * @param num_conv - Converters the receive link carries.
 * @param tx_num_conv - Converters the transmit link carries.
 */
static void dma_example_dump_datapath(struct ad9088_phy *phy, uint8_t num_conv,
				      uint8_t tx_num_conv)
{
	adi_apollo_jesd_tx_link_cfg_t *frm =
		&phy->profile.jtx[TEST_TONE_SIDE].tx_link_cfg[0];
	adi_apollo_jesd_rx_link_cfg_t *dfrm =
		&phy->profile.jrx[TEST_TONE_SIDE].rx_link_cfg[0];
	adi_apollo_rxpath_t *rx_path = &phy->profile.rx_path[TEST_TONE_SIDE];
	adi_apollo_txpath_t *tx_path = &phy->profile.tx_path[TEST_TONE_SIDE];
	uint32_t val;
	uint8_t i;

	pr_info("Datapath map, side %u:\n", TEST_TONE_SIDE);

	for (i = 0; i < ADI_APOLLO_FDDCS_PER_SIDE; i++) {
		val = 0;
		adi_apollo_fddc_dcm_bf_to_val(&phy->ad9088,
					      rx_path->rx_fddc[i].drc_ratio,
					      &val);
		pr_info("  FDDC%u: link %u dcm %lu clks 0x%02x\n", i,
			rx_path->rx_fddc[i].link_num, (unsigned long)val,
			rx_path->rx_fddc[i].debug_fddc_clkoff_n);
	}

	for (i = 0; i < ADI_APOLLO_CDDCS_PER_SIDE; i++) {
		val = 0;
		adi_apollo_cddc_dcm_bf_to_val(&phy->ad9088,
					      rx_path->rx_cddc[i].drc_ratio,
					      &val);
		pr_info("  CDDC%u: dcm %lu\n", i, (unsigned long)val);
	}

	for (i = 0; i < ADI_APOLLO_FDUCS_PER_SIDE; i++) {
		val = 0;
		adi_apollo_fduc_interp_bf_to_val(&phy->ad9088,
						 tx_path->tx_fduc[i].drc_ratio,
						 &val);
		pr_info("  FDUC%u: int %lu\n", i, (unsigned long)val);
	}

	for (i = 0; i < num_conv; i++)
		pr_info("  framer conv %u <- vconv %u\n", i,
			(unsigned)frm->conv_xbar_sel[i]);

	for (i = 0; i < tx_num_conv; i++)
		pr_info("  deframer conv %u -> sample %u\n", i,
			(unsigned)dfrm->sample_xbar_sel[i]);
}

/**
 * @brief Derive the rate the captured samples arrive at.
 *
 * The FNCO mixes at the CDDC output rate and the FDDC decimates after it, so the
 * captured rate carries both ratios. The decimations are read back from the
 * profile the JESD204 FSM populated rather than assuming a value, so a profile
 * change cannot silently shift every predicted frequency.
 *
 * @param phy - AD9088 device.
 * @param rx_adc - RX TPL core, used only to cross-check the derived rate.
 * @param capture_rate - Returns the FDDC output rate in Hz.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_get_capture_rate(struct ad9088_phy *phy,
					struct axi_adc *rx_adc,
					uint64_t *capture_rate)
{
	adi_apollo_rxpath_t *rx_path = &phy->profile.rx_path[TEST_TONE_SIDE];
	uint32_t cddc_dcm;
	uint32_t fddc_dcm;
	uint8_t cddc_pi;
	uint8_t fddc_pi;
	uint64_t adc_rate;
	uint64_t delta;
	int ret;

	cddc_pi = (TEST_TONE_FDDC / 2) % ADI_APOLLO_CDUCS_PER_SIDE;
	ret = adi_apollo_cddc_dcm_bf_to_val(&phy->ad9088,
					    rx_path->rx_cddc[cddc_pi].drc_ratio,
					    &cddc_dcm);
	if (ret) {
		pr_err("Reading the CDDC decimation failed (%d)\n", ret);
		return ret;
	}

	fddc_pi = TEST_TONE_FDDC % ADI_APOLLO_FDUCS_PER_SIDE;
	ret = adi_apollo_fddc_dcm_bf_to_val(&phy->ad9088,
					    rx_path->rx_fddc[fddc_pi].drc_ratio,
					    &fddc_dcm);
	if (ret) {
		pr_err("Reading the FDDC decimation failed (%d)\n", ret);
		return ret;
	}

	if (!cddc_dcm || !fddc_dcm) {
		pr_err("Invalid decimation CDDC=%lu FDDC=%lu\n",
		       (unsigned long)cddc_dcm, (unsigned long)fddc_dcm);
		return -EINVAL;
	}

	adc_rate = phy->profile.adc_cfg[TEST_TONE_SIDE].adc_sampling_rate_Hz;
	*capture_rate = no_os_div_u64(adc_rate, cddc_dcm * fddc_dcm);

	if (!*capture_rate) {
		pr_err("Invalid capture rate\n");
		return -EINVAL;
	}

	pr_info("Profile Rates: ADC %lu kHz / CDDC %lu / FDDC %lu = capture %lu kHz\n",
		(unsigned long)no_os_div_u64(adc_rate, 1000),
		(unsigned long)cddc_dcm, (unsigned long)fddc_dcm,
		(unsigned long)no_os_div_u64(*capture_rate, 1000));

	/*
	 * The TPL core derives the same rate from its own clock registers, so a
	 * mismatch means the decimation model above is wrong and every predicted
	 * frequency would be off by that factor.
	 */
	if (rx_adc->clock_hz) {
		delta = rx_adc->clock_hz > *capture_rate ?
			rx_adc->clock_hz - *capture_rate :
			*capture_rate - rx_adc->clock_hz;

		pr_info("TPL core reports %lu kHz\n",
			(unsigned long)no_os_div_u64(rx_adc->clock_hz, 1000));

		if (no_os_div_u64(delta * 100, *capture_rate) > 5)
			pr_info("Warning: TPL rate disagrees with the profile "
				"by more than 5%%\n");
	}

	pr_info("\n");
	return 0;
}

/**
 * @brief Derive the rate the transmitted samples leave the DMA at.
 *
 * The mirror of dma_example_get_capture_rate() for the transmit direction. The
 * interpolation enums are encoded differently from the decimation ones, so the
 * CDUC/FDUC converters must be used rather than the CDDC/FDDC pair, otherwise
 * every transmitted frequency lands off by that ratio.
 *
 * @param phy - AD9088 device.
 * @param tx_dac - TX TPL core, used only to cross-check the derived rate.
 * @param tx_rate - Returns the FDUC input rate in Hz.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_get_tx_rate(struct ad9088_phy *phy,
				   struct axi_dac *tx_dac, uint64_t *tx_rate)
{
	adi_apollo_txpath_t *tx_path = &phy->profile.tx_path[TEST_TONE_SIDE];
	uint32_t cduc_int;
	uint32_t fduc_int;
	uint8_t cduc_pi;
	uint8_t fduc_pi;
	uint64_t dac_rate;
	uint64_t delta;
	int ret;

	cduc_pi = (TEST_TONE_FDDC / 2) % ADI_APOLLO_CDUCS_PER_SIDE;
	ret = adi_apollo_cduc_interp_bf_to_val(&phy->ad9088,
					       tx_path->tx_cduc[cduc_pi].drc_ratio,
					       &cduc_int);
	if (ret) {
		pr_err("Reading the CDUC interpolation failed (%d)\n", ret);
		return ret;
	}

	fduc_pi = TEST_TONE_FDDC % ADI_APOLLO_FDUCS_PER_SIDE;
	ret = adi_apollo_fduc_interp_bf_to_val(&phy->ad9088,
					       tx_path->tx_fduc[fduc_pi].drc_ratio,
					       &fduc_int);
	if (ret) {
		pr_err("Reading the FDUC interpolation failed (%d)\n", ret);
		return ret;
	}

	if (!cduc_int || !fduc_int) {
		pr_err("Invalid interpolation CDUC=%lu FDUC=%lu\n",
		       (unsigned long)cduc_int, (unsigned long)fduc_int);
		return -EINVAL;
	}

	dac_rate = phy->profile.dac_cfg[TEST_TONE_SIDE].dac_sampling_rate_Hz;
	*tx_rate = no_os_div_u64(dac_rate, cduc_int * fduc_int);

	if (!*tx_rate) {
		pr_err("Invalid transmit rate\n");
		return -EINVAL;
	}

	pr_info("Profile Rates: DAC %lu kHz / CDUC %lu / FDUC %lu = "
		"transmit %lu kHz\n",
		(unsigned long)no_os_div_u64(dac_rate, 1000),
		(unsigned long)cduc_int, (unsigned long)fduc_int,
		(unsigned long)no_os_div_u64(*tx_rate, 1000));

	if (tx_dac->clock_hz) {
		delta = tx_dac->clock_hz > *tx_rate ?
			tx_dac->clock_hz - *tx_rate :
			*tx_rate - tx_dac->clock_hz;

		pr_info("TPL core reports %lu kHz\n",
			(unsigned long)no_os_div_u64(tx_dac->clock_hz, 1000));

		if (no_os_div_u64(delta * 100, *tx_rate) > 5)
			pr_info("Warning: TPL rate disagrees with the profile "
				"by more than 5%%\n");
	}

	return 0;
}

/**
 * @brief Work out the buffer layout from the link the FSM brought up.
 *
 * Everything here is derived from the profile rather than hardcoded, so a
 * profile change cannot silently corrupt the buffer layout -- it fails the
 * range checks instead. The capture depth is clamped to what the static buffer
 * holds, so more converters shorten the capture rather than overrun it.
 *
 * @param m - Measurement inputs, with phy already set. Fills in the geometry.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_derive_geometry(struct dma_example_meas *m)
{
	adi_apollo_jesd_tx_link_cfg_t *frm =
		&m->phy->profile.jtx[TEST_TONE_SIDE].tx_link_cfg[0];
	adi_apollo_jesd_rx_link_cfg_t *dfrm =
		&m->phy->profile.jrx[TEST_TONE_SIDE].rx_link_cfg[0];

	m->num_conv = frm->m_minus1 + 1;
	m->np = frm->np_minus1 + 1;

	if (!m->num_conv || m->num_conv > MAX_LINK_CONVERTERS) {
		pr_err("Unexpected converter count M=%u\n", m->num_conv);
		return -EINVAL;
	}

	m->tx_num_conv = dfrm->m_minus1 + 1;

	if (!m->tx_num_conv || m->tx_num_conv > MAX_LINK_CONVERTERS) {
		pr_err("Unexpected transmit converter count M=%u\n",
		       m->tx_num_conv);
		return -EINVAL;
	}

	/*
	 * Complex channels the loopback can carry end to end: an I/Q pair per
	 * channel on each link, and a channel is only usable if both links have
	 * a pair for it. Every one of them is transmitted on and captured, so
	 * the whole buffer holds signal rather than just its first pair.
	 */
	m->num_ch = (m->num_conv < m->tx_num_conv ?
		     m->num_conv : m->tx_num_conv) / 2;

	if (!m->num_ch) {
		pr_err("Links carry no complex channel: M rx=%u tx=%u\n",
		       m->num_conv, m->tx_num_conv);
		return -EINVAL;
	}

	/* Clamp the capture depth to what the static buffer can hold. */
	m->samples = ADC_BUFFER_SAMPLES;
	if (m->samples * m->num_conv > NO_OS_ARRAY_SIZE(adc_buffer_dma))
		m->samples = NO_OS_ARRAY_SIZE(adc_buffer_dma) / m->num_conv;

	m->transfer_size = m->samples * m->num_conv * sizeof(adc_buffer_dma[0]);

	m->rx_layout.num_conv = m->num_conv;
	m->rx_layout.conv_i = TONE_CONV_I;
	m->rx_layout.conv_q = TONE_CONV_Q;

	/* The transmit link carries its own M, so it needs its own layout. */
	m->tx_layout.num_conv = m->tx_num_conv;
	m->tx_layout.conv_i = TONE_CONV_I;
	m->tx_layout.conv_q = TONE_CONV_Q;

	pr_info("Capture geometry: M=%u NP=%u samples/conv=%lu bytes=%lu "
		"channels=%u\n", m->num_conv, m->np,
		(unsigned long)m->samples, (unsigned long)m->transfer_size,
		m->num_ch);

	return 0;
}

/**
 * @brief Put both sides' NCOs on a known default frequency.
 *
 * A CDUC/FDUC upconverts and a CDDC/FDDC downconverts, so a tone written at
 * f_lut comes back at f_lut + (tx shifts) - (rx shifts). A profile is free to
 * leave the transmit and receive NCOs on different frequencies, and any offset
 * between them translates the tone by that much -- generally far enough to put
 * it outside the FDDC passband whatever it was transmitted at. Tuning all to
 * the same frequency cancels the translation, so a tone arrives where it was
 * sent.
 *
 * @param phy - AD9088 device.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_set_default_nco(struct ad9088_phy *phy)
{
	uint64_t dac_rate = phy->profile.dac_cfg[TEST_TONE_SIDE]
			    .dac_sampling_rate_Hz;
	int64_t cnco_hz;
	int64_t tx_cnco = 0;
	int64_t tx_fnco = 0;
	int64_t rx_cnco = 0;
	int64_t rx_fnco = 0;
	uint8_t cddc;
	uint8_t fddc;
	int ret;

	cnco_hz = (int64_t)no_os_div_u64(dac_rate, DEFAULT_CNCO_RATE_DIV);

	for (cddc = 0; cddc < ADI_APOLLO_CDDCS_PER_SIDE; cddc++) {
		ret = ad9088_set_cnco_freq(phy, ADI_APOLLO_TX, TEST_TONE_SIDE,
					   cddc, cnco_hz);
		if (!ret)
			ret = ad9088_set_cnco_freq(phy, ADI_APOLLO_RX,
						   TEST_TONE_SIDE, cddc,
						   cnco_hz);
		if (ret) {
			pr_err("Tuning CDDC/CDUC %u failed (%d)\n", cddc, ret);
			return ret;
		}
	}

	for (fddc = 0; fddc < ADI_APOLLO_FDDCS_PER_SIDE; fddc++) {
		ret = ad9088_set_fnco_freq(phy, ADI_APOLLO_TX, TEST_TONE_SIDE,
					   fddc, DEFAULT_FNCO_HZ);
		if (!ret)
			ret = ad9088_set_fnco_freq(phy, ADI_APOLLO_RX,
						   TEST_TONE_SIDE, fddc,
						   DEFAULT_FNCO_HZ);
		if (ret) {
			pr_err("Tuning FDDC/FDUC %u failed (%d)\n", fddc, ret);
			return ret;
		}
	}

	/*
	 * Read every one of them back rather than trusting the writes: a tone is
	 * only where it was sent if the whole side agrees, so one datapath that
	 * did not take the tuning is enough to invalidate a measurement.
	 */
	for (fddc = 0; fddc < ADI_APOLLO_FDDCS_PER_SIDE; fddc++) {
		cddc = (fddc / 2) % ADI_APOLLO_CDDCS_PER_SIDE;

		ret = ad9088_get_cnco_freq(phy, ADI_APOLLO_TX, TEST_TONE_SIDE,
					   cddc, &tx_cnco);
		if (!ret)
			ret = ad9088_get_fnco_freq(phy, ADI_APOLLO_TX,
						   TEST_TONE_SIDE, fddc,
						   &tx_fnco);
		if (!ret)
			ret = ad9088_get_cnco_freq(phy, ADI_APOLLO_RX,
						   TEST_TONE_SIDE, cddc,
						   &rx_cnco);
		if (!ret)
			ret = ad9088_get_fnco_freq(phy, ADI_APOLLO_RX,
						   TEST_TONE_SIDE, fddc,
						   &rx_fnco);
		if (ret) {
			pr_err("Reading back the NCOs failed (%d)\n", ret);
			return ret;
		}

		/*
		 * The tuning word drops its fractional part, so a rate that the
		 * divisor does not divide exactly lands a few Hz off. Far too
		 * little to move the tone off its bin, but it should not pass
		 * unremarked.
		 */
		if (tx_cnco != cnco_hz || rx_cnco != cnco_hz)
			pr_info("  Warning: CDDC%u asked %ld Hz, tuned tx %ld "
				"rx %ld\n", cddc, (long)cnco_hz, (long)tx_cnco,
				(long)rx_cnco);

		if ((tx_cnco + tx_fnco) != (rx_cnco + rx_fnco)) {
			pr_err("CDDC%u/FDDC%u did not take the default tuning: "
			       "tx c/f %ld/%ld kHz  rx c/f %ld/%ld kHz\n", cddc,
			       fddc, (long)no_os_div_s64(tx_cnco, 1000),
			       (long)no_os_div_s64(tx_fnco, 1000),
			       (long)no_os_div_s64(rx_cnco, 1000),
			       (long)no_os_div_s64(rx_fnco, 1000));
			return -EIO;
		}
	}

	pr_info("  NCOs: %u coarse at %ld kHz, %u fine at %ld Hz, tx and rx "
		"matched\n", (unsigned)ADI_APOLLO_CDDCS_PER_SIDE,
		(long)no_os_div_s64(cnco_hz, 1000),
		(unsigned)ADI_APOLLO_FDDCS_PER_SIDE, (long)DEFAULT_FNCO_HZ);

	return 0;
}

/**
 * @brief Measurement (a): score an idle datapath and expect nothing.
 *
 * The baseline the other two are read against. Coherence here should collapse
 * to roughly NO_OS_TONE_SCALE/N, which is what shows the estimator rejects
 * noise instead of scoring anything handed to it -- without that, a high
 * coherence in the tone test would not mean much.
 *
 * Test mode is already off after bring-up, but it is disabled explicitly so the
 * floor measures a state this function guarantees rather than one it assumes.
 *
 * @param m - Measurement inputs.
 * @param pass - Returns the verdict.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_measure_noise_floor(const struct dma_example_meas *m,
		bool *pass)
{
	struct no_os_tone_result measured;
	struct no_os_tone_test test = {
		.name = "Noise floor, datapath idle",
		.sense = NO_OS_TONE_SENSE_QUIET,
		.conjugate = NULL,
		.tx_amplitude = 0,
		.full_scale = 0,
		.limits = dma_example_limits,
	};
	int ret;

	ret = ad9088_set_fnco_test_tone(m->phy, ADI_APOLLO_RX, TEST_TONE_SIDE,
					TEST_TONE_FDDC, false, 0);
	if (ret) {
		pr_err("Disabling the RX FNCO test tone failed (%d)\n", ret);
		return ret;
	}

	no_os_mdelay(10);

	ret = dma_example_capture(m->rx_dmac, m->transfer_size);
	if (ret)
		return ret;

	ret = no_os_tone_coherence(adc_buffer_dma, m->samples, &m->rx_layout,
				   -m->tone_hz, m->capture_rate, &measured);
	if (ret) {
		pr_err("Scoring the idle capture failed (%d)\n", ret);
		return ret;
	}

	test.freq_hz = -m->tone_hz;
	test.samples = m->samples;

	*pass = no_os_tone_report(&test, &measured);

	return 0;
}

/**
 * @brief Measurement (b): inject a tone inside the RX datapath and find it.
 *
 * Test mode replaces the FDDC mixer input with a constant which the NCO then
 * rotates, so the FDDC emits a complex tone at the FNCO frequency. Nothing
 * leaves the chip, so this validates the receive datapath only -- the tone
 * never passes through the DAC, the cables or the ADC.
 *
 * Scored at -tone_hz: an RX FDDC downconverts, multiplying by exp(-jwn) so that
 * a signal at +f_nco lands at DC. The test tone is a constant injected ahead of
 * that mixer, so it comes out rotating at -f_nco. Verified on hardware -- a
 * positive programmed frequency captures as a rotation of the same magnitude in
 * the negative direction. A TX FDUC upconverts and needs the opposite sign.
 *
 * Both perturbations are undone before returning, on every path. Test mode
 * discards the FDDC mixer input and would swallow the loopback signal exactly
 * as it discards everything else upstream, and the retuned FNCO would translate
 * it.
 *
 * @param m - Measurement inputs.
 * @param pass - Returns the verdict.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_measure_rx_tone(const struct dma_example_meas *m,
				       bool *pass)
{
	struct no_os_tone_result measured;
	struct no_os_tone_test test = {
		.name = "RX FNCO test tone",
		.sense = NO_OS_TONE_SENSE_TONE,
		.conjugate = NULL,
		/*
		 * A constant of magnitude TEST_TONE_OFFSET rotated by the NCO
		 * arrives as a complex tone of sqrt(2) times it, so that is the
		 * level to reference the measurement against. The integer
		 * sqrt(2) is why the reported ratio lands near rather than
		 * exactly at its predicted value.
		 */
		.tx_amplitude = TEST_TONE_OFFSET * 2828 / 1000,
		.full_scale = SAMPLE_FULL_SCALE,
		.limits = dma_example_limits,
	};
	int restore;
	int disable;
	int ret;

	ret = ad9088_set_fnco_test_tone(m->phy, ADI_APOLLO_RX, TEST_TONE_SIDE,
					TEST_TONE_FDDC, true, TEST_TONE_OFFSET);
	if (ret) {
		pr_err("Enabling the RX FNCO test tone failed (%d)\n", ret);
		return ret;
	}

	ret = ad9088_set_fnco_freq(m->phy, ADI_APOLLO_RX, TEST_TONE_SIDE,
				   TEST_TONE_FDDC, m->tone_hz);
	if (ret) {
		pr_err("Setting the RX FNCO frequency failed (%d)\n", ret);
		goto undo;
	}

	no_os_mdelay(10);

	ret = dma_example_capture(m->rx_dmac, m->transfer_size);
	if (ret)
		goto undo;

	ret = no_os_tone_coherence(adc_buffer_dma, m->samples, &m->rx_layout,
				   -m->tone_hz, m->capture_rate, &measured);
	if (ret) {
		pr_err("Scoring the tone capture failed (%d)\n", ret);
		goto undo;
	}

	test.freq_hz = -m->tone_hz;
	test.samples = m->samples;

	*pass = no_os_tone_report(&test, &measured);
	ret = 0;

undo:
	/* Keep the first error: the undo must not mask what went wrong. */
	disable = ad9088_set_fnco_test_tone(m->phy, ADI_APOLLO_RX,
					    TEST_TONE_SIDE, TEST_TONE_FDDC,
					    false, 0);
	if (disable)
		pr_err("Disabling the RX FNCO test tone failed (%d)\n", disable);
	if (!ret)
		ret = disable;

	restore = dma_example_set_default_nco(m->phy);
	if (!ret)
		ret = restore;

	return ret;
}

/**
 * @brief Start replaying a tone out of the DAC, for measurement (c).
 *
 * The offload replays its whole BRAM whatever was written into it, so all of it
 * is filled rather than leaving the tail to come back as noise. Every channel
 * is filled too, not just the first pair: the fill zeroes what it does not
 * write, so a channel left out would have its DAC driven with silence and come
 * back empty in the capture.
 *
 * The tone is at the same frequency the tone test uses. With both sides' NCOs
 * on the same default it comes back where it was sent, on a capture bin, and a
 * whole number of cycles fits the transmit buffer, so the cyclic wrap leaves no
 * phase discontinuity for a capture to straddle -- which matters because the
 * capture is shorter than the replay and can start anywhere in it.
 *
 * @param m - Measurement inputs. Fills in the replay size.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_start_tx_tone(struct dma_example_meas *m)
{
	struct axi_dma_transfer tx_transfer;
	uint32_t tx_bram_size = 0;
	int ret;

	no_os_axi_io_read(TX_DATA_OFFLOAD_BASEADDR, AXI_DO_REG_MEMORY_SIZE_LSB,
			  &tx_bram_size);

	m->tx_size = sizeof(dac_buffer_dma);
	if (tx_bram_size && tx_bram_size < m->tx_size)
		m->tx_size = tx_bram_size;

	/* The DMAC rejects a source address off its data path width. */
	m->tx_size &= ~(uint32_t)(DMA_SRC_WIDTH_BYTES - 1);
	m->tx_samples = m->tx_size /
			(m->tx_num_conv * sizeof(dac_buffer_dma[0]));
	m->tx_size = m->tx_samples * m->tx_num_conv * sizeof(dac_buffer_dma[0]);

	pr_info("Transmitting: M=%u, %u channels, %lu samples/conv, %lu bytes "
		"cyclic\n", m->tx_num_conv, m->num_ch,
		(unsigned long)m->tx_samples, (unsigned long)m->tx_size);

	ret = no_os_tone_fill_iq(dac_buffer_dma, m->tx_samples, &m->tx_layout,
				 m->tone_hz, m->tx_rate, LOOPBACK_TX_AMPLITUDE,
				 m->num_ch);
	if (ret) {
		pr_err("Filling the transmit buffer failed (%d)\n", ret);
		return ret;
	}

	/* MEM_TO_DEV reads DDR, so dirty lines have to land there first. */
	Xil_DCacheFlushRange((uintptr_t)dac_buffer_dma, m->tx_size);

	ret = axi_dac_set_datasel(m->tx_dac, -1, AXI_DAC_DATA_SEL_DMA);
	if (ret) {
		pr_err("Selecting the DMA data source failed (%d)\n", ret);
		return ret;
	}

	tx_transfer.size = m->tx_size;
	tx_transfer.transfer_done = 0;
	tx_transfer.cyclic = CYCLIC;
	tx_transfer.src_addr = (uintptr_t)dac_buffer_dma;
	tx_transfer.dest_addr = 0;

	/*
	 * Cyclic keeps the tone running for the whole capture. It is a build
	 * option of the DMAC rather than a guarantee, so fall back to a single
	 * pass if the core rejects it. Never wait for completion either way: a
	 * cyclic transfer raises no end-of-transfer and would only time out.
	 */
	ret = axi_dmac_transfer_start(m->tx_dmac, &tx_transfer);
	if (ret) {
		pr_info("  cyclic transfer unavailable, using a single pass\n");
		tx_transfer.cyclic = NO;
		ret = axi_dmac_transfer_start(m->tx_dmac, &tx_transfer);
	}

	if (ret) {
		pr_err("TX DMA transfer start failed (%d)\n", ret);
		/* Same unwind main's error_tx_stream does, next to its cause. */
		axi_dmac_transfer_stop(m->tx_dmac);
		axi_dac_set_datasel(m->tx_dac, -1, AXI_DAC_DATA_SEL_ZERO);
		return ret;
	}

	return 0;
}

/**
 * @brief Measurement (c): capture what came back through the cable.
 *
 * The only measurement covering the whole chain. Scored at +tone_hz rather than
 * the tone test's -tone_hz: that one injects a constant ahead of a
 * downconverting mixer, whereas this transmits a real +tone_hz and the aligned
 * mixers cancel, so it arrives where it was sent.
 *
 * The second probe covers the transmitted I/Q ordering, the one convention in
 * this path no other measurement confirms: a swapped pair sends -f. Either sign
 * passes and the report names a swap rather than failing on it.
 *
 * Scored once per channel over the one capture. Every channel is transmitted on
 * and cabled, so a flat one is a real fault rather than an idle datapath.
 *
 * @param m - Measurement inputs.
 * @param pass - Returns the verdict, false if any channel fails.
 * @return 0 on success, negative error code otherwise.
 */
static int dma_example_measure_loopback(const struct dma_example_meas *m,
					bool *pass)
{
	/* Reused per channel by the loopback measurement. */
	struct no_os_tone_layout ch_layout;
	struct no_os_tone_result conjugate;
	struct no_os_tone_result measured;
	struct no_os_tone_test test = {
		.name = "Cabled DAC -> ADC loopback",
		.sense = NO_OS_TONE_SENSE_EITHER,
		.conjugate = &conjugate,
		.tx_amplitude = LOOPBACK_TX_AMPLITUDE,
		.full_scale = SAMPLE_FULL_SCALE,
		.limits = dma_example_limits,
	};
	uint8_t ch;
	int ret;

	no_os_mdelay(10);

	ret = dma_example_capture(m->rx_dmac, m->transfer_size);
	if (ret)
		return ret;

	test.freq_hz = m->tone_hz;
	test.samples = m->samples;

	*pass = true;

	for (ch = 0; ch < m->num_ch; ch++) {
		ch_layout.num_conv = m->num_conv;
		ch_layout.conv_i = TONE_CONV_I + 2 * ch;
		ch_layout.conv_q = TONE_CONV_Q + 2 * ch;

		ret = no_os_tone_coherence(adc_buffer_dma, m->samples,
					   &ch_layout, m->tone_hz,
					   m->capture_rate, &measured);
		if (ret) {
			pr_err("Scoring the loopback capture failed (%d)\n",
			       ret);
			return ret;
		}

		ret = no_os_tone_coherence(adc_buffer_dma, m->samples,
					   &ch_layout, -m->tone_hz,
					   m->capture_rate, &conjugate);
		if (ret) {
			pr_err("Probing the loopback conjugate failed (%d)\n",
			       ret);
			return ret;
		}

		pr_info("Channel %u, conv %u/%u\n", ch, ch_layout.conv_i,
			ch_layout.conv_q);

		if (!no_os_tone_report(&test, &measured))
			*pass = false;
	}

	return 0;
}

/**
 * @brief Bring up the board, then run the three measurements.
 *
 * See the roadmap at the top of this file for what each step is for. Does not
 * return on success -- step 15 parks so the capture buffer stays readable.
 *
 * @return Negative error code on failure, never returns on success.
 */
int dma_example_main(void)
{
	struct adf4382_dev *adf4382_dev;
	struct hmc7044_dev *hmc7044_dev;
	struct adf4030_dev *adf4030_dev;
	struct axi_jesd204_rx *rx_jesd;
	struct axi_jesd204_tx *tx_jesd;
	struct adxcvr *rx_adxcvr;
	struct adxcvr *tx_adxcvr;
	struct jesd204_clk rx_jesd_clk = {0};
	struct jesd204_clk tx_jesd_clk = {0};
	struct no_os_clk_desc rx_lane_clk = {0};
	struct no_os_clk_desc tx_lane_clk = {0};
	struct jesd204_topology *topology;
	struct axi_dmac *rx_dmac;
	struct axi_dmac *tx_dmac;
	struct ad9088_phy *ad9088_phy;
	struct axi_adc *rx_adc;
	struct axi_dac *tx_dac;
	struct dma_example_meas meas = {0};
	/*
	 * Held in their own flags rather than ret, since the measurements issue
	 * device calls that overwrite ret and the verdicts are combined only
	 * once every one of them has run.
	 */
	bool loopback_pass;
	bool floor_pass;
	bool tone_pass;

	int ret = 0;

	pr_info("Enter DMA example\n");

	/* Step 1: reference synthesizer. */
	ret = adf4382_init(&adf4382_dev, &adf4382_ip);
	if (ret) {
		pr_info("ADF4382 initialization failed\n");
		goto error;
	}

	/* Step 2: clock distribution. */
	ret = hmc7044_init(&hmc7044_dev, &hmc7044_ip);
	if (ret) {
		pr_info("HMC7044 initialization failed\n");
		goto error_adf4382;
	}

	/*
	 * Step 3: SYSREF provider. After the HMC7044: the ADF4030's reference
	 * comes from HMC7044 ch1.
	 */
	ret = adf4030_init(&adf4030_dev, &adf4030_ip);
	if (ret) {
		pr_info("ADF4030 initialization failed\n");
		goto error_hmc7044;
	}

	/*
	 * Step 4: enables MCS calibration, which trims the AD9088's internal
	 * SYSREF onto the external edge. Needs both clock chips probed, so it
	 * goes here.
	 */
	ret = ad9088_mcs_ops_bind(adf4030_dev, adf4382_dev);
	if (ret) {
		pr_info("MCS ops bind failed\n");
		goto error_adf4030;
	}

	/* Step 5: DMA controllers. */
	ret = axi_dmac_init(&rx_dmac, &rx_dmac_ip);
	if (ret) {
		pr_info("RX DMAC initialization failed\n");
		goto error_adf4030;
	}

	ret = axi_dmac_init(&tx_dmac, &tx_dmac_ip);
	if (ret) {
		pr_info("TX DMAC initialization failed\n");
		goto error_rx_dmac;
	}

	/* Step 6: serial transceivers. */
	ret = adxcvr_init(&tx_adxcvr, &tx_adxcvr_ip);
	if (ret) {
		pr_info("TX ADXCVR initialization failed\n");
		goto error_tx_dmac;
	}
	tx_jesd_clk.xcvr = tx_adxcvr;

	ret = adxcvr_init(&rx_adxcvr, &rx_adxcvr_ip);
	if (ret) {
		pr_info("RX ADXCVR initialization failed\n");
		goto error_tx_adxcvr;
	}
	rx_jesd_clk.xcvr = rx_adxcvr;

	/* Step 7: lane clocks, which the JESD204 cores drive through. */
	rx_lane_clk.platform_ops = &jesd204_clk_ops;
	rx_lane_clk.dev_desc = &rx_jesd_clk;
	rx_jesd204_ip.lane_clk = &rx_lane_clk;

	tx_lane_clk.platform_ops = &jesd204_clk_ops;
	tx_lane_clk.dev_desc = &tx_jesd_clk;
	tx_jesd204_ip.lane_clk = &tx_lane_clk;

	/* Step 8: AXI JESD204 link layer cores. */
	ret = axi_jesd204_rx_init(&rx_jesd, &rx_jesd204_ip);
	if (ret) {
		pr_info("JESD RX initialization failed\n");
		goto error_rx_adxcvr;
	}
	rx_jesd_clk.jesd_rx = rx_jesd;

	ret = axi_jesd204_tx_init(&tx_jesd, &tx_jesd204_ip);
	if (ret) {
		pr_info("JESD TX initialization failed\n");
		goto error_rx_jesd;
	}
	tx_jesd_clk.jesd_tx = tx_jesd;

	/* Step 9: the transceiver itself, profile and firmware included. */
	ret = ad9088_init(&ad9088_phy, &ad9088_ip);
	if (ret) {
		pr_info("AD9088 initialization failed\n");
		goto error_tx_jesd;
	}

	/*
	 * Step 10: the ADF4030 is the SYSREF provider: it drives both the
	 * Apollo's SYSREF pin and the FPGA's sysref_in. It must be listed
	 * before the top device - jesd204_topology_init() reads
	 * is_sysref_provider from this array but takes the jdev pointer from
	 * the top-device-filtered copy, so the two indices only agree while the
	 * provider precedes the top device.
	 */
	struct jesd204_topology_dev devs[] = {
		{
			.jdev = adf4030_dev->jdev,
			.link_ids = {
				FRAMER_LINK_A0_RX,
				DEFRAMER_LINK_A0_TX
			},
			.links_number = 2,
			.is_sysref_provider = true,
		},
		{
			.jdev = hmc7044_dev->jdev,
			.link_ids = {
				FRAMER_LINK_A0_RX,
				DEFRAMER_LINK_A0_TX
			},
			.links_number = 2,
		},
		{
			.jdev = rx_jesd->jdev,
			.link_ids = {FRAMER_LINK_A0_RX},
			.links_number = 1,
		},
		{
			.jdev = tx_jesd->jdev,
			.link_ids = {DEFRAMER_LINK_A0_TX},
			.links_number = 1,
		},
		{
			.jdev = ad9088_phy->jdev,
			.link_ids = {
				FRAMER_LINK_A0_RX,
				DEFRAMER_LINK_A0_TX
			},
			.links_number = 2,
			.is_top_device = true,
		},
	};

	ret = jesd204_topology_init(&topology, devs,
				    NO_OS_ARRAY_SIZE(devs));
	if (ret) {
		pr_info("JESD204 topology init failed\n");
		goto error_ad9088;
	}

	ret = jesd204_fsm_start(topology, JESD204_LINKS_ALL);
	if (ret) {
		pr_info("JESD204 FSM start failed\n");
		goto error_topology;
	}

	axi_jesd204_tx_status_read(tx_jesd);
	axi_jesd204_rx_status_read(rx_jesd);

	/* Step 11: everything the measurements need, from here on. */
	meas.phy = ad9088_phy;
	meas.rx_dmac = rx_dmac;
	meas.tx_dmac = tx_dmac;

	ret = dma_example_derive_geometry(&meas);
	if (ret)
		goto error_topology;

	dma_example_dump_datapath(meas.phy, meas.num_conv, meas.tx_num_conv);

	/* Step 12: transport layer cores, sized to the link. */
	struct axi_adc_init rx_adc_init = {
		.name = "rx_adc",
		.base = RX_CORE_BASEADDR,
		.num_channels = meas.num_conv,
	};

	struct axi_dac_init tx_dac_init = {
		.name = "tx_dac",
		.base = TX_CORE_BASEADDR,
		.num_channels = meas.num_conv,
	};

	ret = axi_adc_init(&rx_adc, &rx_adc_init);
	if (ret) {
		pr_err("RX TPL core init failed (%d)\n", ret);
		goto error_topology;
	}

	ret = axi_dac_init(&tx_dac, &tx_dac_init);
	if (ret) {
		pr_err("TX TPL core init failed (%d)\n", ret);
		goto error_rx_adc;
	}
	meas.tx_dac = tx_dac;

	/* Leave the TX datapath idle: the FNCO tone is injected inside RX. */
	axi_dac_set_datasel(tx_dac, -1, AXI_DAC_DATA_SEL_ZERO);
	pr_info("Project configured\n\n");

	/* Step 13: the operating point every measurement runs from. */
	ret = dma_example_get_capture_rate(meas.phy, rx_adc,
					   &meas.capture_rate);
	if (ret)
		goto error_tx_dac;

	meas.tone_hz = (int64_t)no_os_div_u64(meas.capture_rate,
					      TEST_TONE_RATE_DIV);

	ret = dma_example_set_default_nco(meas.phy);
	if (ret)
		goto error_tx_dac;

	/* Step 14: the three measurements, weakest claim first. */
	ret = dma_example_measure_noise_floor(&meas, &floor_pass);
	if (ret)
		goto error_tx_dac;

	ret = dma_example_measure_rx_tone(&meas, &tone_pass);
	if (ret)
		goto error_tx_dac;

	ret = dma_example_get_tx_rate(meas.phy, tx_dac, &meas.tx_rate);
	if (ret)
		goto error_tx_dac;

	ret = dma_example_start_tx_tone(&meas);
	if (ret)
		goto error_tx_dac;

	ret = dma_example_measure_loopback(&meas, &loopback_pass);
	if (ret)
		goto error_tx_stream;

	/*
	 * Sample count is the total across converters, not per converter, which
	 * is what the capture script's de-interleave expects: every field here
	 * transfers to its command line unchanged.
	 */
	pr_info("RX buffer address: 0x%08lx samples=%lu channels=%u bits=%u\n",
		(unsigned long)adc_buffer_dma,
		(unsigned long)meas.samples * meas.num_conv,
		meas.num_conv, meas.np);

	ret = (floor_pass && tone_pass && loopback_pass) ? 0 : -EIO;
	if (ret)
		goto error_tx_stream;

	/* Step 15: park, so the capture survives for capture.tcl to read. */
	pr_info("Parked for JTAG capture; reset the board to continue\n");
	while (1)
		no_os_mdelay(1000);

error_tx_stream:
	axi_dmac_transfer_stop(tx_dmac);
	axi_dac_set_datasel(tx_dac, -1, AXI_DAC_DATA_SEL_ZERO);
error_tx_dac:
	axi_dac_remove(tx_dac);
error_rx_adc:
	axi_adc_remove(rx_adc);
error_topology:
	jesd204_topology_remove(topology);
error_ad9088:
	ad9088_remove(ad9088_phy);
error_tx_jesd:
	axi_jesd204_tx_remove(tx_jesd);
error_rx_jesd:
	axi_jesd204_rx_remove(rx_jesd);
error_rx_adxcvr:
	adxcvr_remove(rx_adxcvr);
error_tx_adxcvr:
	adxcvr_remove(tx_adxcvr);
error_tx_dmac:
	axi_dmac_remove(tx_dmac);
error_rx_dmac:
	axi_dmac_remove(rx_dmac);
error_adf4030:
	adf4030_remove(adf4030_dev);
error_hmc7044:
	hmc7044_remove(hmc7044_dev);
error_adf4382:
	adf4382_remove(adf4382_dev);
error:
	if (ret)
		pr_info("Error!\n");

	return ret;
}
