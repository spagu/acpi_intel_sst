/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST DMA Controller Implementation
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include "acpi_intel_sst.h"
#include "sst_dma.h"

/*
 * DMA Register Access
 */
static inline uint32_t
dma_read(struct sst_softc *sc, uint32_t reg)
{
	return (bus_read_4(sc->mem_res, SST_DMA_OFFSET + reg));
}

static inline void
dma_write(struct sst_softc *sc, uint32_t reg, uint32_t val)
{
	bus_write_4(sc->mem_res, SST_DMA_OFFSET + reg, val);
}

/*
 * DMA Channel Register Access
 */
#define DMA_CH_OFFSET(ch)	(0x58 * (ch))

static inline uint32_t
dma_ch_read(struct sst_softc *sc, int ch, uint32_t reg)
{
	return (bus_read_4(sc->mem_res,
			   SST_DMA_OFFSET + DMA_CH_OFFSET(ch) + reg));
}

static inline void
dma_ch_write(struct sst_softc *sc, int ch, uint32_t reg, uint32_t val)
{
	bus_write_4(sc->mem_res, SST_DMA_OFFSET + DMA_CH_OFFSET(ch) + reg, val);
}

/*
 * Enable/Disable DMA channel
 */
static void
dma_channel_enable(struct sst_softc *sc, int ch, bool enable)
{
	uint32_t val;

	val = (1 << ch);		/* Channel bit */
	val |= (1 << (ch + 8));		/* Write enable bit */

	if (enable)
		val |= (1 << ch);

	dma_write(sc, DMA_CHEN, val);
}

/*
 * Initialize DMA controller
 */
int
sst_dma_init(struct sst_softc *sc)
{
	int i;

	/* Enable DMA controller */
	dma_write(sc, DMA_CFG, 1);

	/* Initialize all channels */
	for (i = 0; i < SST_DMA_CHANNELS; i++) {
		sc->dma.ch[i].id = i;
		sc->dma.ch[i].state = SST_DMA_STATE_IDLE;
		sc->dma.ch[i].callback = NULL;
		sc->dma.ch[i].callback_arg = NULL;

		/* Disable channel */
		dma_channel_enable(sc, i, false);
	}

	/* Mask all interrupts initially */
	dma_write(sc, DMA_MASK_TFR, 0xFF00);
	dma_write(sc, DMA_MASK_BLOCK, 0xFF00);
	dma_write(sc, DMA_MASK_ERR, 0xFF00);

	/* Clear any pending interrupts */
	dma_write(sc, DMA_CLEAR_TFR, 0xFF);
	dma_write(sc, DMA_CLEAR_BLOCK, 0xFF);
	dma_write(sc, DMA_CLEAR_ERR, 0xFF);

	sc->dma.initialized = true;
	sc->dma.active_mask = 0;

	device_printf(sc->dev, "DMA initialized: %d channels\n",
		      SST_DMA_CHANNELS);

	return (0);
}

/*
 * Cleanup DMA controller
 */
void
sst_dma_fini(struct sst_softc *sc)
{
	int i;

	/* Stop all channels */
	for (i = 0; i < SST_DMA_CHANNELS; i++) {
		if (sc->dma.ch[i].state == SST_DMA_STATE_RUNNING)
			sst_dma_stop(sc, i);
	}

	/* Disable DMA controller */
	dma_write(sc, DMA_CFG, 0);

	sc->dma.initialized = false;
}

/*
 * Allocate a DMA channel
 */
int
sst_dma_alloc(struct sst_softc *sc)
{
	int i;

	for (i = 0; i < SST_DMA_CHANNELS; i++) {
		if (sc->dma.ch[i].state == SST_DMA_STATE_IDLE) {
			sc->dma.ch[i].state = SST_DMA_STATE_CONFIGURED;
			sc->dma.active_mask |= (1 << i);
			device_printf(sc->dev, "DMA: Allocated channel %d\n", i);
			return (i);
		}
	}

	device_printf(sc->dev, "DMA: No free channels\n");
	return (-1);
}

/*
 * Free a DMA channel
 */
void
sst_dma_free(struct sst_softc *sc, int ch)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return;

	if (sc->dma.ch[ch].state == SST_DMA_STATE_RUNNING)
		sst_dma_stop(sc, ch);

	sc->dma.ch[ch].state = SST_DMA_STATE_IDLE;
	sc->dma.ch[ch].callback = NULL;
	sc->dma.ch[ch].callback_arg = NULL;
	sc->dma.active_mask &= ~(1 << ch);

	device_printf(sc->dev, "DMA: Freed channel %d\n", ch);
}

/*
 * Configure DMA channel
 */
int
sst_dma_configure(struct sst_softc *sc, int ch, struct sst_dma_config *config)
{
	uint32_t ctl_lo, ctl_hi;
	uint32_t cfg_lo, cfg_hi;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	if (sc->dma.ch[ch].state == SST_DMA_STATE_RUNNING) {
		device_printf(sc->dev, "DMA%d: Cannot configure while running\n",
			      ch);
		return (EBUSY);
	}

	/* Save configuration */
	sc->dma.ch[ch].config = *config;

	/* Configure source address */
	dma_ch_write(sc, ch, DMA_SAR, config->src & 0xFFFFFFFF);

	/* Configure destination address */
	dma_ch_write(sc, ch, DMA_DAR, config->dst & 0xFFFFFFFF);

	/* Configure control register (low) */
	ctl_lo = DMA_CTL_INT_EN;
	ctl_lo |= DMA_CTL_DST_WIDTH(config->dst_width);
	ctl_lo |= DMA_CTL_SRC_WIDTH(config->src_width);
	ctl_lo |= DMA_CTL_DST_MSIZE(config->dst_burst);
	ctl_lo |= DMA_CTL_SRC_MSIZE(config->src_burst);

	switch (config->direction) {
	case DMA_TT_M2M:
		ctl_lo |= DMA_CTL_DINC(DMA_INC_INCREMENT);
		ctl_lo |= DMA_CTL_SINC(DMA_INC_INCREMENT);
		ctl_lo |= DMA_CTL_TT_FC(DMA_TT_M2M);
		break;
	case DMA_TT_M2P:
		ctl_lo |= DMA_CTL_DINC(DMA_INC_NO_CHANGE);
		ctl_lo |= DMA_CTL_SINC(DMA_INC_INCREMENT);
		ctl_lo |= DMA_CTL_TT_FC(DMA_TT_M2P);
		break;
	case DMA_TT_P2M:
		ctl_lo |= DMA_CTL_DINC(DMA_INC_INCREMENT);
		ctl_lo |= DMA_CTL_SINC(DMA_INC_NO_CHANGE);
		ctl_lo |= DMA_CTL_TT_FC(DMA_TT_P2M);
		break;
	default:
		return (EINVAL);
	}

	dma_ch_write(sc, ch, DMA_CTL_LO, ctl_lo);

	/* Configure control register (high) - block size */
	ctl_hi = (config->size / 4) & DMA_CTL_BLOCK_TS_MASK;
	dma_ch_write(sc, ch, DMA_CTL_HI, ctl_hi);

	/* Configure channel configuration register (low) */
	cfg_lo = DMA_CFG_CH_PRIOR(ch);
	if (config->direction == DMA_TT_M2P ||
	    config->direction == DMA_TT_P2M) {
		/* Hardware handshaking */
		if (config->direction == DMA_TT_M2P)
			cfg_lo &= ~DMA_CFG_HS_SEL_DST;
		else
			cfg_lo &= ~DMA_CFG_HS_SEL_SRC;
	}
	dma_ch_write(sc, ch, DMA_CFG_LO, cfg_lo);

	/* Configure channel configuration register (high) */
	cfg_hi = DMA_CFG_FIFO_MODE;
	dma_ch_write(sc, ch, DMA_CFG_HI, cfg_hi);

	/* Clear linked list pointer (single block transfer) */
	dma_ch_write(sc, ch, DMA_LLP, 0);

	device_printf(sc->dev, "DMA%d: Configured src=0x%lx dst=0x%lx size=%zu\n",
		      ch, (unsigned long)config->src, (unsigned long)config->dst,
		      config->size);

	return (0);
}

/*
 * Start DMA transfer
 */
int
sst_dma_start(struct sst_softc *sc, int ch)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	if (sc->dma.ch[ch].state == SST_DMA_STATE_RUNNING)
		return (0);

	if (sc->dma.ch[ch].state == SST_DMA_STATE_IDLE) {
		device_printf(sc->dev, "DMA%d: Not configured\n", ch);
		return (EINVAL);
	}

	/* Unmask transfer complete interrupt */
	dma_write(sc, DMA_MASK_TFR, (1 << ch) | (1 << (ch + 8)));
	dma_write(sc, DMA_MASK_ERR, (1 << ch) | (1 << (ch + 8)));

	/* Enable channel */
	dma_channel_enable(sc, ch, true);

	sc->dma.ch[ch].state = SST_DMA_STATE_RUNNING;

	device_printf(sc->dev, "DMA%d: Started\n", ch);

	return (0);
}

/*
 * Stop DMA transfer
 */
int
sst_dma_stop(struct sst_softc *sc, int ch)
{
	int timeout;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	if (sc->dma.ch[ch].state != SST_DMA_STATE_RUNNING &&
	    sc->dma.ch[ch].state != SST_DMA_STATE_PAUSED)
		return (0);

	/* Suspend channel first */
	dma_ch_write(sc, ch, DMA_CFG_LO,
		     dma_ch_read(sc, ch, DMA_CFG_LO) | DMA_CFG_CH_SUSP);

	/* Wait for FIFO empty */
	timeout = 1000;
	while (timeout-- > 0) {
		if (dma_ch_read(sc, ch, DMA_CFG_LO) & DMA_CFG_FIFO_EMPTY)
			break;
		DELAY(10);
	}

	/* Disable channel */
	dma_channel_enable(sc, ch, false);

	/* Mask interrupts */
	dma_write(sc, DMA_MASK_TFR, (1 << (ch + 8)));
	dma_write(sc, DMA_MASK_ERR, (1 << (ch + 8)));

	/* Clear any pending interrupts */
	dma_write(sc, DMA_CLEAR_TFR, (1 << ch));
	dma_write(sc, DMA_CLEAR_ERR, (1 << ch));

	sc->dma.ch[ch].state = SST_DMA_STATE_CONFIGURED;

	device_printf(sc->dev, "DMA%d: Stopped\n", ch);

	return (0);
}

/*
 * Pause DMA transfer
 */
int
sst_dma_pause(struct sst_softc *sc, int ch)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	if (sc->dma.ch[ch].state != SST_DMA_STATE_RUNNING)
		return (EINVAL);

	/* Suspend channel */
	dma_ch_write(sc, ch, DMA_CFG_LO,
		     dma_ch_read(sc, ch, DMA_CFG_LO) | DMA_CFG_CH_SUSP);

	sc->dma.ch[ch].state = SST_DMA_STATE_PAUSED;

	return (0);
}

/*
 * Resume DMA transfer
 */
int
sst_dma_resume(struct sst_softc *sc, int ch)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	if (sc->dma.ch[ch].state != SST_DMA_STATE_PAUSED)
		return (EINVAL);

	/* Resume channel */
	dma_ch_write(sc, ch, DMA_CFG_LO,
		     dma_ch_read(sc, ch, DMA_CFG_LO) & ~DMA_CFG_CH_SUSP);

	sc->dma.ch[ch].state = SST_DMA_STATE_RUNNING;

	return (0);
}

/*
 * Set completion callback
 */
void
sst_dma_set_callback(struct sst_softc *sc, int ch,
		     void (*callback)(void *), void *arg)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return;

	sc->dma.ch[ch].callback = callback;
	sc->dma.ch[ch].callback_arg = arg;
}

/*
 * DMA interrupt handler
 */
void
sst_dma_intr(struct sst_softc *sc)
{
	uint32_t status_tfr, status_err;
	int i;

	status_tfr = dma_read(sc, DMA_STATUS_TFR);
	status_err = dma_read(sc, DMA_STATUS_ERR);

	for (i = 0; i < SST_DMA_CHANNELS; i++) {
		if (status_tfr & (1 << i)) {
			/* Clear interrupt */
			dma_write(sc, DMA_CLEAR_TFR, (1 << i));

			/* Call completion callback */
			if (sc->dma.ch[i].callback)
				sc->dma.ch[i].callback(sc->dma.ch[i].callback_arg);

			/* Mark as configured (not running) */
			sc->dma.ch[i].state = SST_DMA_STATE_CONFIGURED;
		}

		if (status_err & (1 << i)) {
			/* Clear error */
			dma_write(sc, DMA_CLEAR_ERR, (1 << i));

			device_printf(sc->dev, "DMA%d: Error interrupt\n", i);
			sc->dma.ch[i].state = SST_DMA_STATE_CONFIGURED;
		}
	}
}

/*
 * Check if DMA is running
 */
bool
sst_dma_is_running(struct sst_softc *sc, int ch)
{
	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (false);

	return (sc->dma.ch[ch].state == SST_DMA_STATE_RUNNING);
}

/*
 * Get current DMA position
 */
size_t
sst_dma_get_position(struct sst_softc *sc, int ch)
{
	uint32_t dar;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (0);

	dar = dma_ch_read(sc, ch, DMA_DAR);

	return (dar - sc->dma.ch[ch].config.dst);
}
