/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * RT286 (ALC3263) Codec Initialization over I2C
 * Target: Intel Broadwell-U with Realtek RT286 on I2C0 at 0x1C
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 * The RT286 codec is connected via I2C and uses HDA verb commands encoded
 * into 4-byte I2C transactions per the rl6347a protocol used by Linux.
 *
 * References:
 *   - Linux sound/soc/codecs/rt286.c
 *   - Linux sound/soc/codecs/rl6347a.c
 *   - Realtek RT286 datasheet
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <x86/bus.h>

#include "acpi_intel_sst.h"
#include "sst_regs.h"

/* ================================================================
 * I2C Low-Level Helpers
 *
 * These operate on the DesignWare I2C controller at SST_I2C0_BASE,
 * extracted and refined from the original sst_probe_i2c_codec().
 * ================================================================ */

/*
 * sst_i2c_init - Map and configure the DesignWare I2C0 controller
 *
 * Configures: master mode, fast speed (400kHz), 7-bit addressing,
 * target address 0x1C (RT286).
 */
static int
sst_i2c_init(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;
	bus_space_handle_t priv_handle;
	uint32_t comp_type, pmcsr;
	int timeout;

	codec->mem_tag = X86_BUS_SPACE_MEM;

	/*
	 * Wake I2C0 from D3 via its LPSS private config PMCSR register.
	 * The I2C controller is at 0xFE103000 but its power management
	 * registers live at 0xFE104000+0x84 (PMCSR). We must transition
	 * it to D0 before the controller registers become accessible.
	 */
	if (bus_space_map(codec->mem_tag, SST_I2C0_PRIV_BASE,
	    0x100, 0, &priv_handle) == 0) {
		pmcsr = bus_space_read_4(codec->mem_tag, priv_handle, 0x84);
		device_printf(sc->dev,
		    "codec: I2C0 PMCSR=0x%08x (D%d)\n", pmcsr, pmcsr & 3);
		if ((pmcsr & 3) != 0) {
			bus_space_write_4(codec->mem_tag, priv_handle, 0x84,
			    pmcsr & ~3);
			DELAY(50000);
			pmcsr = bus_space_read_4(codec->mem_tag, priv_handle,
			    0x84);
			device_printf(sc->dev,
			    "codec: I2C0 PMCSR after D0: 0x%08x\n", pmcsr);
		}
		bus_space_unmap(codec->mem_tag, priv_handle, 0x100);
	} else {
		device_printf(sc->dev,
		    "codec: warning - could not map I2C0 LPSS private config\n");
	}

	if (bus_space_map(codec->mem_tag, SST_I2C0_BASE, SST_I2C0_SIZE,
	    0, &codec->i2c_handle) != 0) {
		device_printf(sc->dev, "codec: failed to map I2C0\n");
		return (ENXIO);
	}
	codec->i2c_mapped = true;

	/* Verify DesignWare I2C controller */
	comp_type = bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_COMP_TYPE);
	if (comp_type != DW_IC_COMP_TYPE_VALUE) {
		device_printf(sc->dev,
		    "codec: bad I2C COMP_TYPE 0x%08x (expected 0x%08x)\n",
		    comp_type, DW_IC_COMP_TYPE_VALUE);
		return (ENXIO);
	}

	/* Disable controller before configuration */
	bus_space_write_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_ENABLE, 0);
	for (timeout = 100; timeout > 0; timeout--) {
		if ((bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_ENABLE_STATUS) & 1) == 0)
			break;
		DELAY(1000);
	}
	if (timeout <= 0) {
		device_printf(sc->dev, "codec: I2C disable timeout\n");
		return (ETIMEDOUT);
	}

	/* Configure: master, fast 400kHz, restart enable, slave disable */
	bus_space_write_4(codec->mem_tag, codec->i2c_handle, DW_IC_CON,
	    DW_IC_CON_MASTER | DW_IC_CON_SPEED_FS |
	    DW_IC_CON_RESTART_EN | DW_IC_CON_SLAVE_DIS);

	/* Set target address to RT286 (0x1C) */
	bus_space_write_4(codec->mem_tag, codec->i2c_handle, DW_IC_TAR,
	    SST_I2C0_CODEC_ADDR);

	/* Disable all interrupts (we poll) */
	bus_space_write_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_INTR_MASK, 0);

	/* Clear any pending interrupts */
	bus_space_read_4(codec->mem_tag, codec->i2c_handle, DW_IC_CLR_INTR);

	/* Enable controller */
	bus_space_write_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_ENABLE, 1);
	for (timeout = 100; timeout > 0; timeout--) {
		if (bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_ENABLE_STATUS) & 1)
			break;
		DELAY(1000);
	}
	if (timeout <= 0) {
		device_printf(sc->dev, "codec: I2C enable timeout\n");
		return (ETIMEDOUT);
	}

	device_printf(sc->dev, "codec: I2C0 controller initialized\n");
	return (0);
}

/*
 * sst_i2c_fini - Disable and unmap I2C0
 */
static void
sst_i2c_fini(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;

	if (codec->i2c_mapped) {
		bus_space_write_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_ENABLE, 0);
		DELAY(10000);
		bus_space_unmap(codec->mem_tag, codec->i2c_handle,
		    SST_I2C0_SIZE);
		codec->i2c_mapped = false;
	}
}

/*
 * sst_i2c_write - Write bytes to I2C with STOP
 *
 * Writes data[] bytes to the I2C bus, with STOP on the last byte.
 * Waits for TX FIFO empty and checks for abort.
 */
static int
sst_i2c_write(struct sst_softc *sc, const uint8_t *data, int len)
{
	struct sst_codec *codec = &sc->codec;
	uint32_t abort;
	int i, timeout;

	/* Clear any previous abort */
	bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_CLR_TX_ABRT);

	/* Drain stale RX data from previous transactions */
	while (bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_RXFLR) > 0)
		bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_DATA_CMD);

	for (i = 0; i < len; i++) {
		uint32_t cmd = data[i];
		if (i == len - 1)
			cmd |= DW_IC_DATA_CMD_STOP;
		bus_space_write_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_DATA_CMD, cmd);
	}

	/* Wait for TX FIFO empty */
	for (timeout = 2000; timeout > 0; timeout--) {
		if (bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_STATUS) & DW_IC_STATUS_TFE)
			break;
		DELAY(100);
	}

	/* Check for abort */
	abort = bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_TX_ABRT_SOURCE);
	if (abort) {
		bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_CLR_TX_ABRT);
		device_printf(sc->dev,
		    "codec: I2C write abort: 0x%08x\n", abort);
		return (EIO);
	}

	/* Small delay to let the bus settle */
	DELAY(500);
	return (0);
}

/*
 * sst_i2c_recv - Read-only I2C transaction (i2c_master_recv equivalent)
 *
 * Issues read commands to receive buf_len bytes from the slave.
 * Generates: START → slave_addr(R) → read bytes → STOP
 *
 * The DesignWare controller auto-generates START when the first
 * command is written after the bus is idle (previous STOP completed).
 */
static int
sst_i2c_recv(struct sst_softc *sc, uint8_t *buf, int buf_len)
{
	struct sst_codec *codec = &sc->codec;
	uint32_t abort, rxflr;
	int i, timeout;

	/* Clear any previous abort */
	bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_CLR_TX_ABRT);

	/* Drain any stale RX data */
	while (bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_RXFLR) > 0)
		bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_DATA_CMD);

	/* Issue read commands */
	for (i = 0; i < buf_len; i++) {
		uint32_t cmd = DW_IC_DATA_CMD_READ;
		if (i == buf_len - 1)
			cmd |= DW_IC_DATA_CMD_STOP;
		bus_space_write_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_DATA_CMD, cmd);
	}

	/* Wait for RX data */
	for (timeout = 2000; timeout > 0; timeout--) {
		rxflr = bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_RXFLR);
		if (rxflr >= (uint32_t)buf_len)
			break;
		DELAY(100);
	}

	/* Check for abort */
	abort = bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_TX_ABRT_SOURCE);
	if (abort) {
		bus_space_read_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_CLR_TX_ABRT);
		device_printf(sc->dev,
		    "codec: I2C recv abort: 0x%08x\n", abort);
		return (EIO);
	}

	if (timeout <= 0) {
		device_printf(sc->dev, "codec: I2C recv timeout (rxflr=%d)\n",
		    (int)rxflr);
		return (ETIMEDOUT);
	}

	/* Read data from RX FIFO */
	for (i = 0; i < buf_len; i++) {
		uint32_t data = bus_space_read_4(codec->mem_tag,
		    codec->i2c_handle, DW_IC_DATA_CMD);
		buf[i] = data & 0xFF;
	}

	DELAY(500);
	return (0);
}

/* ================================================================
 * RT286 HDA Verb over I2C (rl6347a protocol)
 *
 * The codec uses 4-byte I2C transactions to encode HDA verbs.
 * Write: data[0..3] = verb bytes
 * Read:  set bit 19 of reg, write 4 bytes, read 4 bytes back
 * ================================================================ */

/*
 * sst_codec_write - Write a verb/value to the codec
 *
 * Encodes per rl6347a_hw_write():
 *   data[0] = (reg >> 24) & 0xff
 *   data[1] = (reg >> 16) & 0xff
 *   data[2] = ((reg >> 8) & 0xff) | ((val >> 8) & 0xff)
 *   data[3] = val & 0xff
 */
static int
sst_codec_write(struct sst_softc *sc, uint32_t reg, uint32_t val)
{
	uint8_t data[4];

	data[0] = (reg >> 24) & 0xFF;
	data[1] = (reg >> 16) & 0xFF;
	data[2] = ((reg >> 8) & 0xFF) | ((val >> 8) & 0xFF);
	data[3] = val & 0xFF;

	return (sst_i2c_write(sc, data, 4));
}

/*
 * sst_codec_read - Read a value from the codec
 *
 * Per Linux rl6347a_hw_read(): set bit 19 of reg to make it a GET
 * verb, send via i2c_master_send (write with STOP), then receive
 * the 4-byte response via i2c_master_recv (read with STOP).
 *
 * These are TWO SEPARATE I2C transactions, not a combined one.
 * The STOP after the write lets the codec process the command
 * before we read the response.
 */
static int
sst_codec_read(struct sst_softc *sc, uint32_t reg, uint32_t *val)
{
	uint8_t wdata[4], rdata[4];
	uint32_t r;
	int error;

	r = reg | (1 << 19);
	wdata[0] = (r >> 24) & 0xFF;
	wdata[1] = (r >> 16) & 0xFF;
	wdata[2] = (r >> 8) & 0xFF;
	wdata[3] = r & 0xFF;

	/* Step 1: Write GET verb (with STOP) - like i2c_master_send */
	error = sst_i2c_write(sc, wdata, 4);
	if (error) {
		device_printf(sc->dev,
		    "codec: read write-phase failed reg=0x%08x: %d\n",
		    reg, error);
		return (error);
	}

	/* Step 2: Read 4-byte response (with STOP) - like i2c_master_recv */
	error = sst_i2c_recv(sc, rdata, 4);
	if (error) {
		device_printf(sc->dev,
		    "codec: read recv-phase failed reg=0x%08x: %d\n",
		    reg, error);
		return (error);
	}

	/*
	 * Reassemble 32-bit response from big-endian I2C bytes.
	 * Over I2C (rl6347a), the 4-byte response IS the raw 32-bit
	 * HDA verb response without codec address prefix.
	 */
	*val = ((uint32_t)rdata[0] << 24) | ((uint32_t)rdata[1] << 16) |
	    ((uint32_t)rdata[2] << 8) | rdata[3];

	return (0);
}

/*
 * sst_codec_update_bits - Read-modify-write a codec register
 */
static int __unused
sst_codec_update_bits(struct sst_softc *sc, uint32_t reg,
    uint32_t mask, uint32_t val)
{
	uint32_t old, new;
	int error;

	error = sst_codec_read(sc, reg, &old);
	if (error)
		return (error);

	new = (old & ~mask) | (val & mask);
	if (new == old)
		return (0);

	return (sst_codec_write(sc, reg, new));
}

/*
 * sst_codec_index_write - Write to an indexed register
 *
 * First writes the index to COEF_INDEX, then the value to PROC_COEF.
 */
static int
sst_codec_index_write(struct sst_softc *sc, uint32_t index, uint32_t val)
{
	int error;

	error = sst_codec_write(sc, RT286_COEF_INDEX, index);
	if (error)
		return (error);
	return (sst_codec_write(sc, RT286_PROC_COEF_SET, val));
}

/*
 * sst_codec_index_read - Read an indexed register
 */
static int
sst_codec_index_read(struct sst_softc *sc, uint32_t index, uint32_t *val)
{
	int error;

	error = sst_codec_write(sc, RT286_COEF_INDEX, index);
	if (error)
		return (error);
	return (sst_codec_read(sc, RT286_PROC_COEF_GET, val));
}

/*
 * sst_codec_index_update_bits - Read-modify-write an indexed register
 */
static int
sst_codec_index_update_bits(struct sst_softc *sc, uint32_t index,
    uint32_t mask, uint32_t val)
{
	uint32_t old, new;
	int error;

	error = sst_codec_index_read(sc, index, &old);
	if (error)
		return (error);

	new = (old & ~mask) | (val & mask);
	if (new == old)
		return (0);

	return (sst_codec_index_write(sc, index, new));
}

/* ================================================================
 * RT286 Initialization Sequence
 *
 * Based on Linux rt286_init() default register writes and
 * Dell-specific quirks (subvendor 0x1028/0x0665).
 * ================================================================ */

/*
 * Index register defaults written during init.
 * From Linux rt286_reg_defaults[] init sequence.
 */
/*
 * Index register defaults from Linux rt286_index_def[].
 * These are written during probe via COEF_INDEX/PROC_COEF.
 */
static const struct {
	uint32_t	index;
	uint32_t	value;
} rt286_index_defaults[] = {
	{ 0x01, 0xaaaa },	/* Power control */
	{ 0x02, 0x8aaa },	/* Power control 2 */
	{ 0x03, 0x0002 },	/* Power control 3 */
	{ 0x04, 0xaf01 },	/* Vendor-specific */
	{ 0x08, 0x000d },	/* Vendor-specific */
	{ 0x09, 0xd810 },	/* I2S control 1 (default, set_dai_fmt patches later) */
	{ 0x0a, 0x0120 },	/* I2S control 2 */
	{ 0x0b, 0x0000 },	/* Clock divider */
	{ 0x0d, 0x2800 },	/* DC gain calibration */
	{ 0x0f, 0x0000 },	/* Vendor-specific */
	{ 0x10, 0x0000 },	/* Analog bias 1 */
	{ 0x11, 0x0000 },	/* Analog bias 2 */
	{ 0x12, 0x0000 },	/* Analog bias 3 */
	{ 0x13, 0x0000 },	/* Analog bias 4 */
	{ 0x19, 0x0a17 },	/* Vendor-specific */
	{ 0x20, 0x0020 },	/* Misc control */
	{ 0x33, 0x0208 },	/* Vendor-specific */
	{ 0x49, 0x0004 },	/* PLL control 1 */
	{ 0x4f, 0x50e9 },	/* Combo jack config */
	{ 0x50, 0x2000 },	/* Combo jack config 2 */
	{ 0x63, 0x2902 },	/* PLL control (PLL disabled by default) */
	{ 0x67, 0x1111 },	/* Depop control 1 */
	{ 0x68, 0x1016 },	/* Depop control 2 */
	{ 0x69, 0x273f },	/* Depop control 3 */
};

int
sst_codec_init(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;
	uint32_t vendor_id;
	int error, i;

	memset(codec, 0, sizeof(*codec));

	device_printf(sc->dev, "codec: initializing RT286...\n");

	/* Step 1: Set up I2C */
	error = sst_i2c_init(sc);
	if (error) {
		device_printf(sc->dev, "codec: I2C init failed: %d\n", error);
		return (error);
	}

	/* Step 2: Verify vendor ID */
	error = sst_codec_read(sc, RT286_GET_PARAM(RT286_NID_ROOT, 0x00),
	    &vendor_id);
	if (error) {
		device_printf(sc->dev,
		    "codec: failed to read vendor ID: %d\n", error);
		goto fail;
	}

	codec->vendor_id = vendor_id;
	device_printf(sc->dev, "codec: vendor ID = 0x%08x\n", vendor_id);

	if (vendor_id != RT286_VENDOR_ID && vendor_id != RT288_VENDOR_ID) {
		device_printf(sc->dev,
		    "codec: unexpected vendor ID (expected 0x%08x or 0x%08x)\n",
		    RT286_VENDOR_ID, RT288_VENDOR_ID);
		error = ENXIO;
		goto fail;
	}

	device_printf(sc->dev, "codec: %s confirmed\n",
	    vendor_id == RT288_VENDOR_ID ? "RT288" : "RT286");

	/* Step 3: Write index register defaults */
	device_printf(sc->dev, "codec: writing %d index register defaults\n",
	    (int)nitems(rt286_index_defaults));
	for (i = 0; i < (int)nitems(rt286_index_defaults); i++) {
		error = sst_codec_index_write(sc,
		    rt286_index_defaults[i].index,
		    rt286_index_defaults[i].value);
		if (error) {
			device_printf(sc->dev,
			    "codec: index write [0x%02x]=0x%04x failed: %d\n",
			    rt286_index_defaults[i].index,
			    rt286_index_defaults[i].value, error);
			goto fail;
		}
	}

	/* Step 4: Audio Function Group to D3 initially */
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_AFG), RT286_PWR_D3);

	/* Step 5: Power up widgets to D1 */
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_DAC0), RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_DAC1), RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_ADC0), RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_ADC1), RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_MIC),  RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_SPK),  RT286_PWR_D1);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_HP),   RT286_PWR_D1);

	/* Step 6: Dell combo jack - already configured in defaults (0x4f=0x50e9) */

	/* Step 7: Dell GPIO6 setup - enable speaker amplifier control */
	sst_codec_write(sc, RT286_SET_GPIO_MASK, RT286_DELL_GPIO6);
	sst_codec_write(sc, RT286_SET_GPIO_DIR,  RT286_DELL_GPIO6);
	sst_codec_write(sc, RT286_SET_GPIO_DATA, RT286_DELL_GPIO6);

	/* GPIO control index: enable GPIO6 output (bit 3) */
	sst_codec_index_write(sc, RT286_IDX_GPIO_CTRL, 0x08);

	/* Step 8: Depop - already configured in defaults (0x67-0x69) */

	codec->initialized = true;
	device_printf(sc->dev, "codec: RT286 initialization complete\n");
	return (0);

fail:
	sst_i2c_fini(sc);
	return (error);
}

/* ================================================================
 * Speaker Output Enable
 *
 * Routes: DAC0 (0x02) → Mixer (0x0C) → SPK Pin (0x14) → speaker amp
 * ================================================================ */

int
sst_codec_enable_speaker(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;
	int error;

	if (!codec->initialized) {
		device_printf(sc->dev, "codec: not initialized\n");
		return (ENXIO);
	}

	device_printf(sc->dev, "codec: enabling speaker output...\n");

	/*
	 * Audio Function Group → D0, PLL clock mode.
	 *
	 * Linux rt286_set_dai_sysclk(RT286_SCLK_S_PLL, 24MHz):
	 *   - bit 6 CLEAR = PLL mode (codec PLL locks to BCLK)
	 *   - bit 6 SET   = direct MCLK mode (only 12.288/24.576MHz)
	 *
	 * The PCH provides 24MHz MCLK, which is NOT a valid frequency
	 * for direct MCLK mode.  PLL mode uses BCLK as reference
	 * instead, which works with any MCLK frequency.
	 */
	error = sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_AFG), RT286_PWR_D0);
	if (error)
		return (error);
	DELAY(50000);	/* 50ms for AFG power up */

	/*
	 * Configure I2S interface for consumer (slave) I2S mode.
	 *
	 * Linux rt286_set_dai_fmt(CBS_CFS, I2S):
	 *   Default 0xd810, clear bit11 (slave), clear bits[9:8] (I2S)
	 *   → 0xd010.  d_len_code for 16-bit: bits[4:3] = 0, no change.
	 */
	sst_codec_index_write(sc, RT286_IDX_I2S_CTRL1, 0xd010);

	/*
	 * Configure codec PLL for BCLK reference at 24MHz.
	 *
	 * Linux rt286_set_dai_sysclk(RT286_SCLK_S_PLL, 24MHz):
	 *   I2S_CTRL2 (0x0a): default=0x0120, set bit8 (BCLK ref)
	 *                      → 0x0120 (already set in default)
	 *   PLL_CTRL  (0x63): default=0x2902, set bit2 (PLL enable)
	 *                      → 0x2906
	 *   PLL_CTRL1 (0x49): default=0x0004, clear bit5 (use PLL)
	 *                      → 0x0004 (already clear)
	 */
	sst_codec_index_write(sc, RT286_IDX_I2S_CTRL2, 0x0120);
	sst_codec_index_write(sc, RT286_IDX_PLL_CTRL, 0x2906);
	sst_codec_index_write(sc, RT286_IDX_PLL_CTRL1, 0x0004);

	/*
	 * Enable power supplies via index registers.
	 * POWER_CTRL (0x01): default=0xaaaa, clear bits[15:10]
	 *   0xaaaa & ~0xFC00 = 0x02aa
	 * POWER_CTRL2 (0x02): default=0x8aaa, clear bit15
	 *   0x8aaa & ~0x8000 = 0x0aaa
	 */
	sst_codec_index_write(sc, RT286_IDX_POWER_CTRL, 0x02aa);
	sst_codec_index_write(sc, RT286_IDX_POWER_CTRL2, 0x0aaa);
	DELAY(50000);	/* 50ms for power supplies */

	/* Power up DAC (0x02), Mixer (0x0C), SPK (0x14) → D0 */
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_DAC0), RT286_PWR_D0);
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_MIX), RT286_PWR_D0);
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_SPK), RT286_PWR_D0);
	DELAY(20000);	/* 20ms for widget power */

	/*
	 * Set DAC format: 48kHz / 16-bit / 2ch.
	 * Bit 15 = 0 selects I2S input (not HDA link).
	 * Linux rt286_set_dai_fmt() explicitly clears bit 15.
	 */
	error = sst_codec_write(sc,
	    RT286_SET_FORMAT(RT286_NID_DAC0), RT286_FMT_48K_16B_2CH);
	if (error) {
		device_printf(sc->dev,
		    "codec: failed to set DAC format: %d\n", error);
		return (error);
	}

	/* Front mixer: unmute DAC input (index 0) */
	sst_codec_write(sc, RT286_SET_MIX_DAC, 0x00);

	/* Speaker mux: select "Front" (connection index 0) */
	sst_codec_write(sc, RT286_SET_CONNECT(RT286_NID_SPK), 0x00);

	/* Unmute speaker: left and right amplifiers, gain 0dB */
	sst_codec_write(sc, RT286_SET_AMP_OUT_L(RT286_NID_SPK), 0x00);
	sst_codec_write(sc, RT286_SET_AMP_OUT_R(RT286_NID_SPK), 0x00);

	/* Enable speaker pin output */
	sst_codec_write(sc,
	    RT286_SET_PIN_CTRL(RT286_NID_SPK), RT286_PIN_OUT);

	/* Enable EAPD (External Amplifier Power Down - active low) */
	sst_codec_write(sc, RT286_SET_EAPD(RT286_NID_SPK), 0x02);

	codec->speaker_active = true;
	device_printf(sc->dev, "codec: speaker output enabled\n");

	return (0);
}

/* ================================================================
 * Headphone Output Enable
 *
 * Routes: DAC1 (0x03) → HP Pin (0x21) → headphone jack
 * ================================================================ */

int
sst_codec_enable_headphone(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;
	int error;

	if (!codec->initialized) {
		device_printf(sc->dev, "codec: not initialized\n");
		return (ENXIO);
	}

	device_printf(sc->dev, "codec: enabling headphone output...\n");

	/* Audio Function Group → D0 (PLL mode, bit 6 clear) */
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_AFG), RT286_PWR_D0);
	DELAY(50000);

	/* Power up DAC1 and HP pin → D0 */
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_DAC1), RT286_PWR_D0);
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_HP), RT286_PWR_D0);
	DELAY(20000);

	/* Set DAC1 format: 48kHz / 16-bit / 2ch */
	error = sst_codec_write(sc,
	    RT286_SET_FORMAT(RT286_NID_DAC1), RT286_FMT_48K_16B_2CH);
	if (error) {
		device_printf(sc->dev,
		    "codec: failed to set HP DAC format: %d\n", error);
		return (error);
	}

	/* Unmute headphone: left and right, gain 0dB */
	sst_codec_write(sc, RT286_SET_AMP_OUT_L(RT286_NID_HP), 0x00);
	sst_codec_write(sc, RT286_SET_AMP_OUT_R(RT286_NID_HP), 0x00);

	/* Enable HP pin: output + headphone amp */
	sst_codec_write(sc,
	    RT286_SET_PIN_CTRL(RT286_NID_HP),
	    RT286_PIN_OUT | RT286_PIN_HP);

	/* Enable EAPD on HP */
	sst_codec_write(sc, RT286_SET_EAPD(RT286_NID_HP), 0x02);

	codec->hp_active = true;
	device_printf(sc->dev, "codec: headphone output enabled\n");
	return (0);
}

/* ================================================================
 * PLL Re-arm (for trigger-time codec refresh)
 *
 * Called during playback trigger after DSP resumes and SSP is
 * clocking BCLK.  Re-pokes the PLL enable and DAC format so the
 * codec PLL locks to the now-active BCLK.
 * ================================================================ */

int
sst_codec_pll_rearm(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;

	if (!codec->initialized)
		return (ENXIO);

	/*
	 * Toggle PLL: disable then re-enable to force a clean lock.
	 * The PLL was enabled during init (before BCLK was present),
	 * so it may be in a failed-lock state.  Toggling resets it.
	 */
	sst_codec_index_write(sc, RT286_IDX_PLL_CTRL, 0x2902); /* disable */
	DELAY(1000);
	sst_codec_index_write(sc, RT286_IDX_PLL_CTRL, 0x2906); /* enable */

	/* Re-set DAC formats (I2S input, 48kHz/16bit/2ch) */
	sst_codec_write(sc,
	    RT286_SET_FORMAT(RT286_NID_DAC0), RT286_FMT_48K_16B_2CH);
	sst_codec_write(sc,
	    RT286_SET_FORMAT(RT286_NID_DAC1), RT286_FMT_48K_16B_2CH);

	DELAY(50000);	/* 50ms for PLL to lock to BCLK */
	return (0);
}

/* ================================================================
 * Codec Shutdown
 * ================================================================ */

void
sst_codec_fini(struct sst_softc *sc)
{
	struct sst_codec *codec = &sc->codec;

	if (!codec->initialized)
		return;

	device_printf(sc->dev, "codec: shutting down RT286...\n");

	/* Mute speaker outputs */
	if (codec->speaker_active) {
		/* Mute by setting bit 7 (mute) in amp gain */
		sst_codec_write(sc,
		    RT286_SET_AMP_OUT_LR(RT286_NID_SPK), 0x80);
		/* Disable pin */
		sst_codec_write(sc,
		    RT286_SET_PIN_CTRL(RT286_NID_SPK), 0x00);
		/* Disable EAPD */
		sst_codec_write(sc,
		    RT286_SET_EAPD(RT286_NID_SPK), 0x00);
		/* Power down */
		sst_codec_write(sc,
		    RT286_SET_POWER(RT286_NID_SPK), RT286_PWR_D3);
		sst_codec_write(sc,
		    RT286_SET_POWER(RT286_NID_MIX), RT286_PWR_D3);
		sst_codec_write(sc,
		    RT286_SET_POWER(RT286_NID_DAC0), RT286_PWR_D3);
		codec->speaker_active = false;
	}

	/* Mute headphone outputs */
	if (codec->hp_active) {
		sst_codec_write(sc,
		    RT286_SET_AMP_OUT_LR(RT286_NID_HP), 0x80);
		sst_codec_write(sc,
		    RT286_SET_PIN_CTRL(RT286_NID_HP), 0x00);
		sst_codec_write(sc,
		    RT286_SET_EAPD(RT286_NID_HP), 0x00);
		sst_codec_write(sc,
		    RT286_SET_POWER(RT286_NID_DAC1), RT286_PWR_D3);
		sst_codec_write(sc,
		    RT286_SET_POWER(RT286_NID_HP), RT286_PWR_D3);
		codec->hp_active = false;
	}

	/* Power down remaining widgets */
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_ADC0), RT286_PWR_D3);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_ADC1), RT286_PWR_D3);
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_MIC),  RT286_PWR_D3);

	/* Audio Function Group → D3 */
	sst_codec_write(sc, RT286_SET_POWER(RT286_NID_AFG), RT286_PWR_D3);

	/* Turn off Dell GPIO6 (speaker amp) */
	sst_codec_write(sc, RT286_SET_GPIO_DATA, 0x00);

	codec->initialized = false;

	/* Tear down I2C */
	sst_i2c_fini(sc);

	device_printf(sc->dev, "codec: RT286 shutdown complete\n");
}
