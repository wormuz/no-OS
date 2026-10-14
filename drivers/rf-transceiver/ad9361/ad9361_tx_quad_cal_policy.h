#ifndef AD9361_TX_QUAD_CAL_POLICY_H_
#define AD9361_TX_QUAD_CAL_POLICY_H_

#include <stdbool.h>
#include <stdint.h>

#define AD9361_TX_QUAD_PORT_COUNT 2u
#define AD9361_TX_QUAD_MODE_COUNT 3u
#define AD9361_TX_QUAD_MODE_TX1 0u
#define AD9361_TX_QUAD_MODE_TX2 1u
#define AD9361_TX_QUAD_MODE_2X2 2u

/* Resolve and validate the RF bandwidth pair that must be restored after a
 * temporary TXQ observation filter. The caller supplies the transaction's
 * intended pair; it may differ from the committed cache until outer commit. */
static inline bool ad9361_tx_quad_restore_bandwidth_target(
		uint32_t restore_rx_bw, uint32_t restore_tx_bw,
		uint32_t *rx_bw, uint32_t *tx_bw)
{
	if (!rx_bw || !tx_bw || !restore_rx_bw || !restore_tx_bw)
		return false;
	*rx_bw = restore_rx_bw;
	*tx_bw = restore_tx_bw;
	return true;
}

/* The driver historically refreshes only after a change greater than the
 * configured threshold. Validity and port/mode selection are handled by the
 * caller so independent RF paths never share a frequency entry. */
static inline bool ad9361_tx_quad_calibration_required(
    bool valid, uint64_t last_frequency_hz, uint64_t requested_frequency_hz,
    uint32_t threshold_hz)
{
    uint64_t delta_hz = requested_frequency_hz > last_frequency_hz
                            ? requested_frequency_hz - last_frequency_hz
                            : last_frequency_hz - requested_frequency_hz;

    return !valid || delta_hz > threshold_hz;
}

/* `selected_tx` uses the AD9361 TX_1/TX_2 numeric values (1/2). */
static inline bool ad9361_tx_quad_mode_index(bool rx2tx2,
                                             uint32_t selected_tx,
                                             uint32_t *mode_index)
{
    if (mode_index == 0)
        return false;
    if (rx2tx2) {
        *mode_index = AD9361_TX_QUAD_MODE_2X2;
        return true;
    }
    if (selected_tx == 1) {
        *mode_index = AD9361_TX_QUAD_MODE_TX1;
        return true;
    }
    if (selected_tx == 2) {
        *mode_index = AD9361_TX_QUAD_MODE_TX2;
        return true;
    }
    return false;
}

#endif
