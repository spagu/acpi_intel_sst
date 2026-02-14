/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST SSP (I2S) Controller Implementation
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
#include "sst_ssp.h"

/*
 * SSP Register Access
 */
static inline uint32_t
ssp_read(struct sst_softc *sc, int port, uint32_t reg)
{
	bus_addr_t base;

	base = (port == 0) ? SST_SSP0_OFFSET : SST_SSP1_OFFSET;
	return (bus_read_4(sc->mem_res, base + reg));
}

static inline void
ssp_write(struct sst_softc *sc, int port, uint32_t reg, uint32_t val)
{
	bus_addr_t base;

	base = (port == 0) ? SST_SSP0_OFFSET : SST_SSP1_OFFSET;
	bus_write_4(sc->mem_res, base + reg, val);
}

static inline void
ssp_update_bits(struct sst_softc *sc, int port, uint32_t reg,
		uint32_t mask, uint32_t val)
{
	uint32_t old, new;

	old = ssp_read(sc, port, reg);
	new = (old & ~mask) | (val & mask);
	ssp_write(sc, port, reg, new);
}

/*
 * Calculate clock dividers for sample rate
 */
static int
ssp_calc_dividers(struct sst_ssp_config *config, uint32_t *scr, uint32_t *acds)
{
	uint32_t bclk, mclk_div;
	uint32_t frame_bits;

	/* Calculate BCLK = sample_rate * channels * bits_per_sample */
	frame_bits = config->channels * config->sample_bits;
	bclk = config->sample_rate * frame_bits;

	/* MCLK is typically 256 * sample_rate or 384 * sample_rate */
	if (config->mclk_rate == 0)
		config->mclk_rate = config->sample_rate * 256;

	/* Calculate divider: BCLK = MCLK / (SCR + 1) */
	if (config->mclk_rate < bclk)
		return (EINVAL);

	mclk_div = config->mclk_rate / bclk;
	if (mclk_div > 0)
		mclk_div--;

	if (mclk_div > 0xFFF)
		return (EINVAL);

	*scr = mclk_div;
	*acds = 0; /* Use default audio clock */

	return (0);
}

/*
 * Configure SSP for I2S mode
 */
static int
ssp_configure_i2s(struct sst_softc *sc, int port, struct sst_ssp_config *config)
{
	uint32_t sscr0, sscr1, sspsp;
	uint32_t scr, acds;
	int error;

	error = ssp_calc_dividers(config, &scr, &acds);
	if (error) {
		device_printf(sc->dev, "SSP%d: Invalid clock configuration\n",
			      port);
		return (error);
	}

	/*
	 * SSCR0 Configuration:
	 * - PSP frame format (I2S compatible)
	 * - Data size (sample_bits - 1)
	 * - Serial clock rate
	 * - Network mode for I2S
	 */
	sscr0 = SSCR0_FRF_PSP;
	sscr0 |= SSCR0_DSS(config->sample_bits);
	if (config->sample_bits > 16)
		sscr0 |= SSCR0_EDSS;
	sscr0 |= SSCR0_SCR(scr);
	sscr0 |= SSCR0_MOD;	/* Network mode */
	sscr0 |= SSCR0_FRDC(config->channels);
	sscr0 |= SSCR0_TIM | SSCR0_RIM; /* Mask FIFO interrupts initially */

	/*
	 * SSCR1 Configuration:
	 * - TX/RX FIFO thresholds
	 * - DMA service enable
	 * - Clock/frame direction (master/slave)
	 */
	sscr1 = SSCR1_TFT(8) | SSCR1_RFT(8);
	sscr1 |= SSCR1_TSRE | SSCR1_RSRE;	/* Enable DMA */
	sscr1 |= SSCR1_TRAIL;			/* Trailing byte handling */

	if (config->master) {
		/* Master mode: generate clocks */
		sscr1 &= ~(SSCR1_SCLKDIR | SSCR1_SFRMDIR);
	} else {
		/* Slave mode: receive clocks */
		sscr1 |= SSCR1_SCLKDIR | SSCR1_SFRMDIR;
	}

	/*
	 * SSPSP Configuration (I2S format):
	 * - Frame sync at start of first data bit
	 * - Frame width = sample_bits
	 */
	sspsp = SSPSP_SCMODE(0);		/* Data on rising, change on falling */
	sspsp |= SSPSP_SFRMP;			/* Frame active low */
	sspsp |= SSPSP_FSRT;			/* Frame sync relative timing */
	sspsp |= SSPSP_SFRMWDTH(config->sample_bits);
	sspsp |= SSPSP_STRTDLY(1);		/* 1 BCLK start delay (I2S) */
	sspsp |= SSPSP_DMYSTRT(0);
	sspsp |= SSPSP_DMYSTOP(0);

	/* Write registers */
	ssp_write(sc, port, SSP_SSCR0, sscr0);
	ssp_write(sc, port, SSP_SSCR1, sscr1);
	ssp_write(sc, port, SSP_SSPSP, sspsp);
	ssp_write(sc, port, SSP_SSACD, SSACD_ACDS(acds));

	/* Configure time slots */
	ssp_write(sc, port, SSP_SSTSA, (1 << config->channels) - 1);
	ssp_write(sc, port, SSP_SSRSA, (1 << config->channels) - 1);

	device_printf(sc->dev, "SSP%d: Configured I2S %uHz/%ubit/%uch %s\n",
		      port, config->sample_rate, config->sample_bits,
		      config->channels, config->master ? "master" : "slave");

	return (0);
}

/*
 * Initialize SSP subsystem
 */
int
sst_ssp_init(struct sst_softc *sc)
{
	int i;

	for (i = 0; i < SST_SSP_PORTS; i++) {
		sc->ssp.port[i].id = i;
		sc->ssp.port[i].base = (i == 0) ? SST_SSP0_OFFSET :
						  SST_SSP1_OFFSET;
		sc->ssp.port[i].state = SST_SSP_STATE_IDLE;
		sc->ssp.port[i].tx_count = 0;
		sc->ssp.port[i].rx_count = 0;
		sc->ssp.port[i].errors = 0;

		/* Default configuration */
		sc->ssp.port[i].config.sample_rate = SST_RATE_48000;
		sc->ssp.port[i].config.sample_bits = 16;
		sc->ssp.port[i].config.channels = 2;
		sc->ssp.port[i].config.format = SST_FMT_I2S;
		sc->ssp.port[i].config.mclk_rate = 0;
		sc->ssp.port[i].config.master = true;
	}

	sc->ssp.initialized = true;

	device_printf(sc->dev, "SSP initialized: %d ports\n", SST_SSP_PORTS);

	return (0);
}

/*
 * Cleanup SSP subsystem
 */
void
sst_ssp_fini(struct sst_softc *sc)
{
	int i;

	for (i = 0; i < SST_SSP_PORTS; i++) {
		if (sc->ssp.port[i].state == SST_SSP_STATE_RUNNING)
			sst_ssp_stop(sc, i);
	}

	sc->ssp.initialized = false;
}

/*
 * Configure SSP port
 */
int
sst_ssp_configure(struct sst_softc *sc, int port, struct sst_ssp_config *config)
{
	int error;

	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	if (sc->ssp.port[port].state == SST_SSP_STATE_RUNNING) {
		device_printf(sc->dev, "SSP%d: Cannot configure while running\n",
			      port);
		return (EBUSY);
	}

	/* Disable SSP before configuration */
	ssp_update_bits(sc, port, SSP_SSCR0, SSCR0_SSE, 0);

	/* Apply configuration based on format */
	switch (config->format) {
	case SST_FMT_I2S:
	case SST_FMT_LEFT_J:
	case SST_FMT_DSP_A:
		error = ssp_configure_i2s(sc, port, config);
		break;
	default:
		device_printf(sc->dev, "SSP%d: Unsupported format %u\n",
			      port, config->format);
		return (EINVAL);
	}

	if (error == 0) {
		sc->ssp.port[port].config = *config;
		sc->ssp.port[port].state = SST_SSP_STATE_CONFIGURED;
	}

	return (error);
}

/*
 * Start SSP port
 */
int
sst_ssp_start(struct sst_softc *sc, int port)
{
	uint32_t sssr;

	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	if (sc->ssp.port[port].state == SST_SSP_STATE_RUNNING)
		return (0);

	if (sc->ssp.port[port].state == SST_SSP_STATE_IDLE) {
		/*
		 * The DSP firmware configures SSP registers via
		 * SET_DEVICE_FORMATS IPC without updating our state.
		 * Check if SSCR0 looks programmed (has MOD bit set).
		 */
		uint32_t sscr0 = ssp_read(sc, port, SSP_SSCR0);
		if (sscr0 & SSCR0_MOD) {
			sc->ssp.port[port].state = SST_SSP_STATE_CONFIGURED;
		} else {
			device_printf(sc->dev, "SSP%d: Not configured\n",
			    port);
			return (EINVAL);
		}
	}

	/* Clear status */
	sssr = ssp_read(sc, port, SSP_SSSR);
	ssp_write(sc, port, SSP_SSSR, sssr);

	/* Enable SSP */
	ssp_update_bits(sc, port, SSP_SSCR0, SSCR0_SSE, SSCR0_SSE);

	sc->ssp.port[port].state = SST_SSP_STATE_RUNNING;

	device_printf(sc->dev, "SSP%d: Started\n", port);

	return (0);
}

/*
 * Stop SSP port
 */
int
sst_ssp_stop(struct sst_softc *sc, int port)
{
	int timeout;

	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	if (sc->ssp.port[port].state != SST_SSP_STATE_RUNNING &&
	    sc->ssp.port[port].state != SST_SSP_STATE_PAUSED)
		return (0);

	/* Disable SSP */
	ssp_update_bits(sc, port, SSP_SSCR0, SSCR0_SSE, 0);

	/* Wait for FIFO to drain */
	timeout = 1000;
	while (timeout-- > 0) {
		if (!(ssp_read(sc, port, SSP_SSSR) & SSSR_BSY))
			break;
		DELAY(10);
	}

	sc->ssp.port[port].state = SST_SSP_STATE_CONFIGURED;

	device_printf(sc->dev, "SSP%d: Stopped\n", port);

	return (0);
}

/*
 * Pause SSP port
 */
int
sst_ssp_pause(struct sst_softc *sc, int port)
{
	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	if (sc->ssp.port[port].state != SST_SSP_STATE_RUNNING)
		return (EINVAL);

	/* Disable TX/RX DMA requests */
	ssp_update_bits(sc, port, SSP_SSCR1,
			SSCR1_TSRE | SSCR1_RSRE, 0);

	sc->ssp.port[port].state = SST_SSP_STATE_PAUSED;

	return (0);
}

/*
 * Resume SSP port
 */
int
sst_ssp_resume(struct sst_softc *sc, int port)
{
	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	if (sc->ssp.port[port].state != SST_SSP_STATE_PAUSED)
		return (EINVAL);

	/* Re-enable TX/RX DMA requests */
	ssp_update_bits(sc, port, SSP_SSCR1,
			SSCR1_TSRE | SSCR1_RSRE,
			SSCR1_TSRE | SSCR1_RSRE);

	sc->ssp.port[port].state = SST_SSP_STATE_RUNNING;

	return (0);
}

/*
 * Write data to SSP (non-DMA mode)
 */
int
sst_ssp_write(struct sst_softc *sc, int port, const void *data, size_t len)
{
	const uint32_t *ptr = data;
	size_t words = len / 4;
	int timeout;

	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	while (words-- > 0) {
		/* Wait for TX FIFO not full */
		timeout = 1000;
		while (timeout-- > 0) {
			if (ssp_read(sc, port, SSP_SSSR) & SSSR_TNF)
				break;
			DELAY(1);
		}

		if (timeout <= 0) {
			sc->ssp.port[port].errors++;
			return (ETIMEDOUT);
		}

		ssp_write(sc, port, SSP_SSDR, *ptr++);
		sc->ssp.port[port].tx_count++;
	}

	return (0);
}

/*
 * Read data from SSP (non-DMA mode)
 */
int
sst_ssp_read(struct sst_softc *sc, int port, void *data, size_t len)
{
	uint32_t *ptr = data;
	size_t words = len / 4;
	int timeout;

	if (port < 0 || port >= SST_SSP_PORTS)
		return (EINVAL);

	while (words-- > 0) {
		/* Wait for RX FIFO not empty */
		timeout = 1000;
		while (timeout-- > 0) {
			if (ssp_read(sc, port, SSP_SSSR) & SSSR_RNE)
				break;
			DELAY(1);
		}

		if (timeout <= 0) {
			sc->ssp.port[port].errors++;
			return (ETIMEDOUT);
		}

		*ptr++ = ssp_read(sc, port, SSP_SSDR);
		sc->ssp.port[port].rx_count++;
	}

	return (0);
}

/*
 * Check if SSP is running
 */
bool
sst_ssp_is_running(struct sst_softc *sc, int port)
{
	if (port < 0 || port >= SST_SSP_PORTS)
		return (false);

	return (sc->ssp.port[port].state == SST_SSP_STATE_RUNNING);
}

/*
 * Dump SSP registers for debugging
 */
void
sst_ssp_dump_regs(struct sst_softc *sc, int port)
{
	if (port < 0 || port >= SST_SSP_PORTS)
		return;

	device_printf(sc->dev, "SSP%d Register Dump:\n", port);
	device_printf(sc->dev, "  SSCR0: 0x%08x\n", ssp_read(sc, port, SSP_SSCR0));
	device_printf(sc->dev, "  SSCR1: 0x%08x\n", ssp_read(sc, port, SSP_SSCR1));
	device_printf(sc->dev, "  SSSR:  0x%08x\n", ssp_read(sc, port, SSP_SSSR));
	device_printf(sc->dev, "  SSPSP: 0x%08x\n", ssp_read(sc, port, SSP_SSPSP));
	device_printf(sc->dev, "  SSACD: 0x%08x\n", ssp_read(sc, port, SSP_SSACD));
	device_printf(sc->dev, "  SSTSA: 0x%08x\n", ssp_read(sc, port, SSP_SSTSA));
	device_printf(sc->dev, "  SSRSA: 0x%08x\n", ssp_read(sc, port, SSP_SSRSA));
}
