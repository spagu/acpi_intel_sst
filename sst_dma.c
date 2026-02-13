/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST DMA Controller Implementation
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Uses DesignWare DMA Controller (DW-DMAC) in linked-list mode
 * for circular audio buffer transfers between memory and SSP.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>
#include <sys/malloc.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

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
	val |= (1 << (ch + 8));	/* Write enable bit */

	if (enable)
		val |= (1 << ch);

	dma_write(sc, DMA_CHEN, val);
}

/*
 * Free LLI descriptors for a channel
 */
static void
dma_free_lli(struct sst_softc *sc, int ch)
{
	struct sst_dma_channel *dch = &sc->dma.ch[ch];

	if (dch->lli != NULL) {
		if (dch->lli_map != NULL) {
			bus_dmamap_unload(dch->lli_tag, dch->lli_map);
			bus_dmamem_free(dch->lli_tag, dch->lli, dch->lli_map);
			dch->lli_map = NULL;
		}
		dch->lli = NULL;
	}
	if (dch->lli_tag != NULL) {
		bus_dma_tag_destroy(dch->lli_tag);
		dch->lli_tag = NULL;
	}
	dch->lli_count = 0;
	dch->lli_addr = 0;
}

/*
 * DMA callback for LLI allocation
 */
static void
dma_lli_cb(void *arg, bus_dma_segment_t *segs, int nseg, int error)
{
	bus_addr_t *addr = arg;

	if (error || nseg != 1)
		return;
	*addr = segs[0].ds_addr;
}

/*
 * Allocate and build circular LLI chain for a DMA channel
 *
 * For M2P (playback): each LLI transfers blk_size bytes from
 *   successive positions in the ring buffer to the fixed SSP SSDR.
 *   Last LLI points back to first, forming a circle.
 *
 * For P2M (capture): each LLI transfers blk_size bytes from
 *   the SSP SSDR to successive positions in the ring buffer.
 */
static int
dma_build_lli(struct sst_softc *sc, int ch, struct sst_dma_config *config)
{
	struct sst_dma_channel *dch = &sc->dma.ch[ch];
	uint32_t ctl_lo, block_ts;
	size_t lli_size;
	int error, i;

	if (config->blk_count == 0 || config->blk_count > SST_DMA_MAX_LLI)
		return (EINVAL);

	/* Build CTL_LO value shared by all descriptors */
	ctl_lo = DMA_CTL_INT_EN;
	ctl_lo |= DMA_CTL_DST_WIDTH(config->dst_width);
	ctl_lo |= DMA_CTL_SRC_WIDTH(config->src_width);
	ctl_lo |= DMA_CTL_DST_MSIZE(config->dst_burst);
	ctl_lo |= DMA_CTL_SRC_MSIZE(config->src_burst);
	ctl_lo |= DMA_CTL_LLP_DST_EN;
	ctl_lo |= DMA_CTL_LLP_SRC_EN;

	switch (config->direction) {
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

	/* Block transfer size in src_width-sized units */
	block_ts = config->blk_size / (1 << config->src_width);
	if (block_ts > DMA_CTL_BLOCK_TS_MASK) {
		device_printf(sc->dev,
		    "DMA%d: block_ts %u exceeds max %u\n",
		    ch, block_ts, DMA_CTL_BLOCK_TS_MASK);
		return (EINVAL);
	}

	/* Allocate LLI array (DMA-accessible memory) */
	lli_size = config->blk_count * sizeof(struct sst_dma_desc);

	error = bus_dma_tag_create(
	    bus_get_dma_tag(sc->dev),
	    32, 0,			/* 32-byte alignment for DW-DMAC LLI */
	    BUS_SPACE_MAXADDR_32BIT,
	    BUS_SPACE_MAXADDR,
	    NULL, NULL,
	    lli_size, 1, lli_size,
	    0, NULL, NULL,
	    &dch->lli_tag);
	if (error) {
		device_printf(sc->dev,
		    "DMA%d: LLI tag create failed: %d\n", ch, error);
		return (error);
	}

	error = bus_dmamem_alloc(dch->lli_tag, (void **)&dch->lli,
	    BUS_DMA_NOWAIT | BUS_DMA_ZERO, &dch->lli_map);
	if (error) {
		device_printf(sc->dev,
		    "DMA%d: LLI alloc failed: %d\n", ch, error);
		bus_dma_tag_destroy(dch->lli_tag);
		dch->lli_tag = NULL;
		return (error);
	}

	error = bus_dmamap_load(dch->lli_tag, dch->lli_map, dch->lli,
	    lli_size, dma_lli_cb, &dch->lli_addr, BUS_DMA_NOWAIT);
	if (error) {
		device_printf(sc->dev,
		    "DMA%d: LLI map failed: %d\n", ch, error);
		bus_dmamem_free(dch->lli_tag, dch->lli, dch->lli_map);
		bus_dma_tag_destroy(dch->lli_tag);
		dch->lli_tag = NULL;
		dch->lli = NULL;
		return (error);
	}

	dch->lli_count = config->blk_count;

	/* Build circular descriptor chain */
	for (i = 0; i < (int)config->blk_count; i++) {
		struct sst_dma_desc *d = &dch->lli[i];
		uint32_t next_idx = (i + 1) % config->blk_count;
		bus_addr_t next_addr = dch->lli_addr +
		    next_idx * sizeof(struct sst_dma_desc);

		if (config->direction == DMA_TT_M2P) {
			d->sar = (config->src + i * config->blk_size) &
			    0xFFFFFFFF;
			d->sar_hi = 0;
			d->dar = config->dst & 0xFFFFFFFF;
			d->dar_hi = 0;
		} else {
			d->sar = config->src & 0xFFFFFFFF;
			d->sar_hi = 0;
			d->dar = (config->dst + i * config->blk_size) &
			    0xFFFFFFFF;
			d->dar_hi = 0;
		}

		d->llp = next_addr & 0xFFFFFFFF;
		d->llp_hi = 0;
		d->ctl_lo = ctl_lo;
		d->ctl_hi = block_ts;
	}

	/* Sync LLI to device */
	bus_dmamap_sync(dch->lli_tag, dch->lli_map,
	    BUS_DMASYNC_PREWRITE);

	device_printf(sc->dev,
	    "DMA%d: Built %u LLI descriptors (blk=%zu, ts=%u)\n",
	    ch, config->blk_count, config->blk_size, block_ts);

	return (0);
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
		sc->dma.ch[i].lli = NULL;
		sc->dma.ch[i].lli_tag = NULL;
		sc->dma.ch[i].lli_map = NULL;
		sc->dma.ch[i].lli_count = 0;

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
		dma_free_lli(sc, i);
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

	dma_free_lli(sc, ch);

	sc->dma.ch[ch].state = SST_DMA_STATE_IDLE;
	sc->dma.ch[ch].callback = NULL;
	sc->dma.ch[ch].callback_arg = NULL;
	sc->dma.active_mask &= ~(1 << ch);

	device_printf(sc->dev, "DMA: Freed channel %d\n", ch);
}

/*
 * Configure DMA channel
 *
 * For circular mode (audio), builds a linked-list of descriptors.
 * For single-shot mode, programs registers directly (block size
 * must fit in 12-bit BLOCK_TS field: max 4095 transfers).
 */
int
sst_dma_configure(struct sst_softc *sc, int ch, struct sst_dma_config *config)
{
	struct sst_dma_channel *dch;
	uint32_t ctl_lo, ctl_hi;
	uint32_t cfg_lo, cfg_hi;
	int error;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	dch = &sc->dma.ch[ch];

	if (dch->state == SST_DMA_STATE_RUNNING) {
		device_printf(sc->dev, "DMA%d: Cannot configure while running\n",
			      ch);
		return (EBUSY);
	}

	/* Free any previous LLI */
	dma_free_lli(sc, ch);

	/* Save configuration */
	dch->config = *config;

	/*
	 * Configure channel configuration registers (shared for both modes)
	 */
	cfg_lo = DMA_CFG_CH_PRIOR(ch);
	if (config->direction == DMA_TT_M2P ||
	    config->direction == DMA_TT_P2M) {
		/* Hardware handshaking for peripheral transfers */
		if (config->direction == DMA_TT_M2P)
			cfg_lo &= ~DMA_CFG_HS_SEL_DST;
		else
			cfg_lo &= ~DMA_CFG_HS_SEL_SRC;
	}
	dma_ch_write(sc, ch, DMA_CFG_LO, cfg_lo);

	cfg_hi = DMA_CFG_FIFO_MODE;
	dma_ch_write(sc, ch, DMA_CFG_HI, cfg_hi);

	if (config->circular && config->blk_count > 0 &&
	    config->blk_size > 0) {
		/*
		 * Circular (linked-list) mode for audio streaming.
		 * Build LLI chain, set LLP register to first descriptor.
		 * DMA hardware automatically follows the chain.
		 */
		error = dma_build_lli(sc, ch, config);
		if (error)
			return (error);

		/* Point SAR/DAR to first block (also in LLI[0]) */
		dma_ch_write(sc, ch, DMA_SAR, dch->lli[0].sar);
		dma_ch_write(sc, ch, DMA_DAR, dch->lli[0].dar);

		/* Set LLP to first descriptor */
		dma_ch_write(sc, ch, DMA_LLP, dch->lli_addr & 0xFFFFFFFF);

		/* Set initial CTL from first descriptor */
		dma_ch_write(sc, ch, DMA_CTL_LO, dch->lli[0].ctl_lo);
		dma_ch_write(sc, ch, DMA_CTL_HI, dch->lli[0].ctl_hi);

		device_printf(sc->dev,
		    "DMA%d: Configured LLP circular src=0x%lx dst=0x%lx "
		    "size=%zu blk=%zu x %u\n",
		    ch, (unsigned long)config->src,
		    (unsigned long)config->dst,
		    config->size, config->blk_size, config->blk_count);
	} else {
		/*
		 * Single-block mode (non-circular).
		 */
		dma_ch_write(sc, ch, DMA_SAR, config->src & 0xFFFFFFFF);
		dma_ch_write(sc, ch, DMA_DAR, config->dst & 0xFFFFFFFF);

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

		ctl_hi = (config->size / (1 << config->src_width)) &
		    DMA_CTL_BLOCK_TS_MASK;
		dma_ch_write(sc, ch, DMA_CTL_HI, ctl_hi);

		/* No linked list */
		dma_ch_write(sc, ch, DMA_LLP, 0);

		device_printf(sc->dev,
		    "DMA%d: Configured single src=0x%lx dst=0x%lx size=%zu\n",
		    ch, (unsigned long)config->src,
		    (unsigned long)config->dst, config->size);
	}

	return (0);
}

/*
 * Start DMA transfer
 */
int
sst_dma_start(struct sst_softc *sc, int ch)
{
	struct sst_dma_channel *dch;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (EINVAL);

	dch = &sc->dma.ch[ch];

	if (dch->state == SST_DMA_STATE_RUNNING)
		return (0);

	if (dch->state == SST_DMA_STATE_IDLE) {
		device_printf(sc->dev, "DMA%d: Not configured\n", ch);
		return (EINVAL);
	}

	if (dch->lli_count > 0) {
		/*
		 * LLP circular mode: use BLOCK interrupts.
		 * Each LLI block completion fires a BLOCK interrupt,
		 * and the DMA hardware automatically fetches the next LLI.
		 * The circular chain means it never stops.
		 */
		dma_write(sc, DMA_MASK_BLOCK,
		    (1 << ch) | (1 << (ch + 8)));
	} else {
		/* Single-block mode: use TFR interrupt */
		dma_write(sc, DMA_MASK_TFR,
		    (1 << ch) | (1 << (ch + 8)));
	}
	dma_write(sc, DMA_MASK_ERR, (1 << ch) | (1 << (ch + 8)));

	/* Enable channel */
	dma_channel_enable(sc, ch, true);

	{
		uint32_t chen = dma_read(sc, DMA_CHEN);
		uint32_t cfg_reg = dma_read(sc, DMA_CFG);
		uint32_t ctl_lo = dma_ch_read(sc, ch, DMA_CTL_LO);
		uint32_t llp = dma_ch_read(sc, ch, DMA_LLP);
		device_printf(sc->dev,
		    "DMA%d: post-enable CHEN=0x%08x CFG=0x%08x "
		    "CTL_LO=0x%08x LLP=0x%08x\n",
		    ch, chen, cfg_reg, ctl_lo, llp);
	}

	dch->state = SST_DMA_STATE_RUNNING;

	device_printf(sc->dev, "DMA%d: Started (%s mode)\n", ch,
	    dch->lli_count > 0 ? "LLP circular" : "single-block");

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

	/* Mask all interrupts for this channel */
	dma_write(sc, DMA_MASK_TFR, (1 << (ch + 8)));
	dma_write(sc, DMA_MASK_BLOCK, (1 << (ch + 8)));
	dma_write(sc, DMA_MASK_ERR, (1 << (ch + 8)));

	/* Clear any pending interrupts */
	dma_write(sc, DMA_CLEAR_TFR, (1 << ch));
	dma_write(sc, DMA_CLEAR_BLOCK, (1 << ch));
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
 *
 * Handles both BLOCK and TFR interrupts:
 * - BLOCK: fired per-LLI in circular mode, channel keeps running
 * - TFR:   fired on single-block completion, channel stops
 */
void
sst_dma_intr(struct sst_softc *sc)
{
	uint32_t status_block, status_tfr, status_err;
	int i;

	status_block = dma_read(sc, DMA_STATUS_BLOCK);
	status_tfr = dma_read(sc, DMA_STATUS_TFR);
	status_err = dma_read(sc, DMA_STATUS_ERR);

	for (i = 0; i < SST_DMA_CHANNELS; i++) {
		if (status_block & (1 << i)) {
			/* Clear block interrupt */
			dma_write(sc, DMA_CLEAR_BLOCK, (1 << i));

			/* Callback - channel keeps running (circular) */
			if (sc->dma.ch[i].callback)
				sc->dma.ch[i].callback(
				    sc->dma.ch[i].callback_arg);
		}

		if (status_tfr & (1 << i)) {
			/* Clear transfer complete interrupt */
			dma_write(sc, DMA_CLEAR_TFR, (1 << i));

			/* Callback */
			if (sc->dma.ch[i].callback)
				sc->dma.ch[i].callback(
				    sc->dma.ch[i].callback_arg);

			/* Single-block mode: transfer is done */
			if (sc->dma.ch[i].lli_count == 0)
				sc->dma.ch[i].state =
				    SST_DMA_STATE_CONFIGURED;
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
 * Get current DMA position within the ring buffer
 *
 * For M2P (playback): SAR holds current source address in ring buffer
 * For P2M (capture):  DAR holds current dest address in ring buffer
 */
size_t
sst_dma_get_position(struct sst_softc *sc, int ch)
{
	struct sst_dma_channel *dch;
	uint32_t addr;

	if (ch < 0 || ch >= SST_DMA_CHANNELS)
		return (0);

	dch = &sc->dma.ch[ch];

	if (dch->config.direction == DMA_TT_M2P) {
		/* Playback: position = SAR - buffer_base */
		addr = dma_ch_read(sc, ch, DMA_SAR);
		return ((addr - (uint32_t)dch->config.src) % dch->config.size);
	} else {
		/* Capture: position = DAR - buffer_base */
		addr = dma_ch_read(sc, ch, DMA_DAR);
		return ((addr - (uint32_t)dch->config.dst) % dch->config.size);
	}
}
