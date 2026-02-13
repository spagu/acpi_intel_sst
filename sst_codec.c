/*-
 * SPDX-License-Identifier: BSD-2-Clause
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
 * sst_i2c_read - Combined write+read I2C transaction
 *
 * Writes addr[] (with RESTART, no STOP), then reads buf_len bytes
 * (with STOP on the last read).
 */
static int
sst_i2c_read(struct sst_softc *sc, const uint8_t *addr, int addr_len,
    uint8_t *buf, int buf_len)
{
	struct sst_codec *codec = &sc->codec;
	uint32_t abort, rxflr;
	int i, timeout;

	/* Clear any previous abort */
	bus_space_read_4(codec->mem_tag, codec->i2c_handle,
	    DW_IC_CLR_TX_ABRT);

	/* Write address bytes (no STOP, RESTART on first byte) */
	for (i = 0; i < addr_len; i++) {
		uint32_t cmd = addr[i];
		if (i == 0)
			cmd |= DW_IC_DATA_CMD_RESTART;
		bus_space_write_4(codec->mem_tag, codec->i2c_handle,
		    DW_IC_DATA_CMD, cmd);
	}

	/* Issue read commands */
	for (i = 0; i < buf_len; i++) {
		uint32_t cmd = DW_IC_DATA_CMD_READ;
		if (i == 0)
			cmd |= DW_IC_DATA_CMD_RESTART;
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
		    "codec: I2C read abort: 0x%08x\n", abort);
		return (EIO);
	}

	if (timeout <= 0) {
		device_printf(sc->dev, "codec: I2C read timeout (rxflr=%d)\n",
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
 * Per rl6347a_hw_read(): set bit 19 of reg to make it a GET verb,
 * write 4 bytes, then read 4 bytes back.
 */
static int
sst_codec_read(struct sst_softc *sc, uint32_t reg, uint32_t *val)
{
	uint8_t wdata[4], rdata[4];
	uint32_t r;
	int error;

	/* Set bit 19 to indicate read (GET) */
	r = reg | (1 << 19);
	wdata[0] = (r >> 24) & 0xFF;
	wdata[1] = (r >> 16) & 0xFF;
	wdata[2] = (r >> 8) & 0xFF;
	wdata[3] = r & 0xFF;

	error = sst_i2c_write(sc, wdata, 4);
	if (error)
		return (error);

	error = sst_i2c_read(sc, wdata, 4, rdata, 4);
	if (error)
		return (error);

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
static const struct {
	uint32_t	index;
	uint32_t	value;
} rt286_index_defaults[] = {
	{ 0x01, 0xACAF },	/* Power control */
	{ 0x02, 0x8000 },	/* Power control 2 */
	{ 0x03, 0x0002 },	/* Power control 3 */
	{ 0x04, 0xAF01 },	/* Vendor-specific */
	{ 0x08, 0x2004 },	/* Vendor-specific */
	{ 0x09, 0x0400 },	/* I2S control - 24-bit I2S mode */
	{ 0x0A, 0x0000 },	/* I2S control 2 */
	{ 0x0D, 0x0000 },	/* DC gain calibration */
	{ 0x0E, 0x2000 },	/* Vendor-specific */
	{ 0x10, 0x0000 },	/* Analog bias 1 */
	{ 0x11, 0x0000 },	/* Analog bias 2 */
	{ 0x12, 0x0000 },	/* Analog bias 3 */
	{ 0x13, 0x0000 },	/* Analog bias 4 */
	{ 0x16, 0x0000 },	/* PLL control */
	{ 0x19, 0x000B },	/* Vendor-specific */
	{ 0x1B, 0x0000 },	/* Vendor-specific */
	{ 0x1F, 0x0000 },	/* Vendor-specific */
	{ 0x20, 0x0000 },	/* Misc control */
	{ 0x33, 0x0400 },	/* Vendor-specific */
	{ 0x49, 0x4004 },	/* Vendor-specific */
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

	/* Step 6: Dell combo jack setup (CBJ index 0x4F) */
	sst_codec_index_update_bits(sc, RT286_IDX_CBJ, 0xF0, 0x50);

	/* Step 7: Dell GPIO6 setup - enable speaker amplifier control */
	sst_codec_write(sc, RT286_SET_GPIO_MASK, RT286_DELL_GPIO6);
	sst_codec_write(sc, RT286_SET_GPIO_DIR,  RT286_DELL_GPIO6);
	sst_codec_write(sc, RT286_SET_GPIO_DATA, RT286_DELL_GPIO6);

	/* GPIO control index: enable GPIO6 output (bit 3) */
	sst_codec_index_update_bits(sc, RT286_IDX_GPIO_CTRL, 0x08, 0x08);

	/* Step 8: Depop parameters */
	sst_codec_index_write(sc, RT286_IDX_DEPOP_CTRL1, 0x001E);
	sst_codec_index_write(sc, RT286_IDX_DEPOP_CTRL2, 0x000A);
	sst_codec_index_write(sc, RT286_IDX_DEPOP_CTRL3, 0x0005);

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

	/* Audio Function Group → D0 */
	error = sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_AFG), RT286_PWR_D0);
	if (error)
		return (error);
	DELAY(50000);	/* 50ms for AFG power up */

	/* DC gain calibration: set bit 9, wait, clear */
	sst_codec_index_update_bits(sc, RT286_IDX_DC_GAIN, 0x0200, 0x0200);
	DELAY(10000);
	sst_codec_index_update_bits(sc, RT286_IDX_DC_GAIN, 0x0200, 0x0000);

	/* Enable power supplies via index registers */
	/* Power control: enable HV, VREF, LDO1, LDO2 */
	sst_codec_index_update_bits(sc, RT286_IDX_POWER_CTRL,
	    0xFC00, 0x0000);	/* Clear power-down bits */
	sst_codec_index_update_bits(sc, RT286_IDX_POWER_CTRL2,
	    0x8000, 0x0000);	/* Clear LDO2 power-down */
	DELAY(50000);	/* 50ms for power supplies */

	/* Power up DAC (0x02), Mixer (0x0C), SPK (0x14) → D0 */
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_DAC0), RT286_PWR_D0);
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_MIX), RT286_PWR_D0);
	sst_codec_write(sc,
	    RT286_SET_POWER(RT286_NID_SPK), RT286_PWR_D0);
	DELAY(20000);	/* 20ms for widget power */

	/* Set DAC format: 48kHz / 16-bit / 2ch */
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

	/* Audio Function Group → D0 (may already be) */
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
