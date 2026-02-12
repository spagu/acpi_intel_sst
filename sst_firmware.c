/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel SST Firmware Loader Implementation
 * Target: Intel Haswell/Broadwell-U
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/firmware.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include "acpi_intel_sst.h"
#include "sst_regs.h"
#include "sst_firmware.h"
#include "sst_ipc.h"

/*
 * Validate firmware header
 */
static int
sst_fw_validate_header(struct sst_softc *sc, const struct sst_fw_header *hdr,
		       size_t size)
{
	if (size < sizeof(struct sst_fw_header)) {
		device_printf(sc->dev, "Firmware too small: %zu bytes\n", size);
		return (EINVAL);
	}

	if (memcmp(hdr->signature, SST_FW_SIGNATURE, SST_FW_SIGN_SIZE) != 0) {
		device_printf(sc->dev, "Invalid firmware signature: %.4s\n",
			      hdr->signature);
		return (EINVAL);
	}

	if (hdr->file_size > size) {
		device_printf(sc->dev, "Firmware size mismatch: %u > %zu\n",
			      hdr->file_size, size);
		return (EINVAL);
	}

	if (hdr->modules == 0) {
		device_printf(sc->dev, "No modules in firmware\n");
		return (EINVAL);
	}

	device_printf(sc->dev, "Firmware: size=%u, modules=%u, format=%u\n",
		      hdr->file_size, hdr->modules, hdr->file_format);

	return (0);
}

/*
 * Load a single block to DSP memory
 */
static int
sst_fw_load_block(struct sst_softc *sc, const struct sst_block_header *blk,
		  const uint8_t *data)
{
	bus_addr_t offset;
	size_t max_size;

	switch (blk->type) {
	case SST_BLK_TYPE_IRAM:
		offset = SST_IRAM_OFFSET + blk->ram_offset;
		max_size = SST_IRAM_SIZE;
		break;
	case SST_BLK_TYPE_DRAM:
		offset = SST_DRAM_OFFSET + blk->ram_offset;
		max_size = SST_DRAM_SIZE;
		break;
	default:
		device_printf(sc->dev, "Unknown block type: %u\n", blk->type);
		return (EINVAL);
	}

	if (blk->ram_offset + blk->size > max_size) {
		device_printf(sc->dev, "Block exceeds memory: off=%u, size=%u\n",
			      blk->ram_offset, blk->size);
		return (EINVAL);
	}

	/* Write block data to DSP memory */
	bus_write_region_1(sc->mem_res, offset, data, blk->size);

	return (0);
}

/*
 * Parse and load a module
 */
static int
sst_fw_load_module(struct sst_softc *sc, const uint8_t *data, size_t size,
		   size_t *consumed)
{
	const struct sst_module_header *mod;
	const struct sst_block_header *blk;
	const uint8_t *ptr;
	uint32_t i;
	int error;

	if (size < sizeof(struct sst_module_header)) {
		device_printf(sc->dev, "Module data too small\n");
		return (EINVAL);
	}

	mod = (const struct sst_module_header *)data;

	if (memcmp(mod->signature, SST_MOD_SIGNATURE, 4) != 0) {
		device_printf(sc->dev, "Invalid module signature: %.4s\n",
			      mod->signature);
		return (EINVAL);
	}

	if (mod->mod_size > size) {
		device_printf(sc->dev, "Module size exceeds data: %u > %zu\n",
			      mod->mod_size, size);
		return (EINVAL);
	}

	device_printf(sc->dev, "Module: size=%u, blocks=%u, entry=0x%08x\n",
		      mod->mod_size, mod->blocks, mod->entry_point);

	/* Store entry point from first module */
	if (sc->fw.entry_point == 0)
		sc->fw.entry_point = mod->entry_point;

	/* Load each block */
	ptr = data + sizeof(struct sst_module_header);
	for (i = 0; i < mod->blocks; i++) {
		if ((size_t)(ptr - data) + sizeof(struct sst_block_header) >
		    mod->mod_size) {
			device_printf(sc->dev, "Block header exceeds module\n");
			return (EINVAL);
		}

		blk = (const struct sst_block_header *)ptr;
		ptr += sizeof(struct sst_block_header);

		if ((size_t)(ptr - data) + blk->size > mod->mod_size) {
			device_printf(sc->dev, "Block data exceeds module\n");
			return (EINVAL);
		}

		error = sst_fw_load_block(sc, blk, ptr);
		if (error)
			return (error);

		ptr += blk->size;
	}

	*consumed = mod->mod_size;
	return (0);
}

/*
 * Load raw binary firmware (format >= 256)
 * These firmwares have no $MOD headers - data is directly IRAM/DRAM blocks
 */
static int
sst_fw_load_raw(struct sst_softc *sc, const uint8_t *data, size_t size)
{
	size_t iram_size, dram_size;

	/*
	 * For Haswell/Broadwell SST firmware (format 256):
	 * The binary is split: first half to IRAM, second half to DRAM
	 * Default entry point is IRAM base (0x0)
	 */

	/* Calculate split - use half for each, capped at memory size */
	iram_size = MIN(size / 2, SST_IRAM_SIZE);
	dram_size = MIN(size - iram_size, SST_DRAM_SIZE);

	device_printf(sc->dev, "Loading raw firmware: IRAM=%zu, DRAM=%zu\n",
		      iram_size, dram_size);

	/* Write to IRAM */
	if (iram_size > 0) {
		bus_write_region_1(sc->mem_res, SST_IRAM_OFFSET,
				   data, iram_size);
	}

	/* Write to DRAM */
	if (dram_size > 0) {
		bus_write_region_1(sc->mem_res, SST_DRAM_OFFSET,
				   data + iram_size, dram_size);
	}

	/* Entry point at IRAM base */
	sc->fw.entry_point = 0;

	return (0);
}

/*
 * Parse firmware and load to DSP
 */
static int
sst_fw_parse(struct sst_softc *sc)
{
	const struct sst_fw_header *hdr;
	const uint8_t *ptr;
	size_t remaining, consumed;
	uint32_t i;
	int error;

	hdr = (const struct sst_fw_header *)sc->fw.data;

	error = sst_fw_validate_header(sc, hdr, sc->fw.size);
	if (error)
		return (error);

	sc->fw.modules = hdr->modules;

	ptr = sc->fw.data + sizeof(struct sst_fw_header);
	remaining = sc->fw.size - sizeof(struct sst_fw_header);

	/*
	 * Check firmware format:
	 * - Format < 256: Uses $MOD module headers
	 * - Format >= 256: Raw binary (Haswell/Broadwell Windows driver FW)
	 *
	 * Some firmwares (like IntcSST2.bin) have format < 256 but use $SST
	 * signatures for modules instead of $MOD. Fall back to raw loading.
	 */
	if (hdr->file_format >= 256) {
		device_printf(sc->dev, "Using raw binary format (v%u)\n",
			      hdr->file_format);
		return sst_fw_load_raw(sc, ptr, remaining);
	}

	/* Check if first module has $MOD or $SST signature */
	if (remaining >= 4) {
		if (memcmp(ptr, SST_MOD_SIGNATURE, 4) != 0) {
			/* Not $MOD - check if it's $SST (nested header) */
			if (memcmp(ptr, SST_FW_SIGNATURE, 4) == 0) {
				device_printf(sc->dev,
				    "Firmware has nested $SST headers, using raw format\n");
				return sst_fw_load_raw(sc, ptr, remaining);
			}
			/* Unknown signature - try raw anyway */
			device_printf(sc->dev,
			    "Unknown module signature: %.4s, trying raw format\n",
			    (const char *)ptr);
			return sst_fw_load_raw(sc, ptr, remaining);
		}
	}

	/* Standard format with $MOD modules */
	for (i = 0; i < hdr->modules; i++) {
		error = sst_fw_load_module(sc, ptr, remaining, &consumed);
		if (error)
			return (error);

		ptr += consumed;
		remaining -= consumed;
	}

	return (0);
}

/*
 * Initialize firmware subsystem
 */
int
sst_fw_init(struct sst_softc *sc)
{
	sc->fw.fw = NULL;
	sc->fw.data = NULL;
	sc->fw.size = 0;
	sc->fw.entry_point = 0;
	sc->fw.modules = 0;
	sc->fw.state = SST_FW_STATE_NONE;

	return (0);
}

/*
 * Cleanup firmware subsystem
 */
void
sst_fw_fini(struct sst_softc *sc)
{
	sst_fw_unload(sc);
}

/*
 * Load firmware from filesystem
 */
int
sst_fw_load(struct sst_softc *sc)
{
	const char *fw_path;
	int error;

	if (sc->fw.state == SST_FW_STATE_LOADED ||
	    sc->fw.state == SST_FW_STATE_RUNNING) {
		device_printf(sc->dev, "Firmware already loaded\n");
		return (EBUSY);
	}

	sc->fw.state = SST_FW_STATE_LOADING;

	/* Select firmware path based on platform */
	fw_path = SST_FW_PATH_BDW;

	device_printf(sc->dev, "Loading firmware: %s\n", fw_path);

	/* Request firmware */
	sc->fw.fw = firmware_get(fw_path);
	if (sc->fw.fw == NULL) {
		device_printf(sc->dev, "Failed to load firmware: %s\n",
			      fw_path);
		device_printf(sc->dev, "Place firmware in /boot/firmware/%s\n",
			      fw_path);
		sc->fw.state = SST_FW_STATE_ERROR;
		return (ENOENT);
	}

	sc->fw.data = sc->fw.fw->data;
	sc->fw.size = sc->fw.fw->datasize;

	device_printf(sc->dev, "Firmware loaded: %zu bytes\n", sc->fw.size);

	/* Parse and load to DSP */
	error = sst_fw_parse(sc);
	if (error) {
		sst_fw_unload(sc);
		sc->fw.state = SST_FW_STATE_ERROR;
		return (error);
	}

	sc->fw.state = SST_FW_STATE_LOADED;
	device_printf(sc->dev, "Firmware parsed: entry=0x%08x\n",
		      sc->fw.entry_point);

	return (0);
}

/*
 * Unload firmware
 */
void
sst_fw_unload(struct sst_softc *sc)
{
	if (sc->fw.fw != NULL) {
		firmware_put(sc->fw.fw, FIRMWARE_UNLOAD);
		sc->fw.fw = NULL;
	}

	sc->fw.data = NULL;
	sc->fw.size = 0;
	sc->fw.entry_point = 0;
	sc->fw.modules = 0;
	sc->fw.state = SST_FW_STATE_NONE;
}

/*
 * Boot DSP with loaded firmware
 */
int
sst_fw_boot(struct sst_softc *sc)
{
	uint32_t csr;
	int error;

	if (sc->fw.state != SST_FW_STATE_LOADED) {
		device_printf(sc->dev, "Firmware not loaded\n");
		return (EINVAL);
	}

	device_printf(sc->dev, "Booting DSP...\n");

	/*
	 * For Broadwell-U (WPT/catpt), SHIM is at BAR0+0xE7000.
	 * CSR (Control/Status Register) is at SHIM offset 0x00.
	 */

	/* Read initial CSR state */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  Initial CSR: 0x%08x\n", csr);

	/* 1. Ensure DSP is in reset and stalled */
	csr |= (SST_CSR_RST | SST_CSR_STALL);
	sst_shim_write(sc, SST_SHIM_CSR, csr);
	DELAY(SST_RESET_DELAY_US);
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  After RST+STALL: 0x%08x\n", csr);

	/* 2. Clear reset but keep stall (allows memory access) */
	csr &= ~SST_CSR_RST;
	sst_shim_write(sc, SST_SHIM_CSR, csr);
	DELAY(SST_RESET_DELAY_US);
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  After clear RST: 0x%08x\n", csr);

	/* 3. Unmask IPC interrupts */
	sst_shim_write(sc, SST_SHIM_IMRX, 0);
	sst_shim_write(sc, SST_SHIM_IMRD, 0);
	device_printf(sc->dev, "  IMRX/IMRD unmasked\n");

	/* 4. Clear stall - DSP starts running */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	csr &= ~SST_CSR_STALL;
	sst_shim_write(sc, SST_SHIM_CSR, csr);
	DELAY(1000); /* Small delay before reading back */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  After clear STALL: 0x%08x (DSP should run)\n", csr);

	device_printf(sc->dev, "DSP running, waiting for ready...\n");

	/* 5. Wait for firmware to signal ready */
	error = sst_ipc_wait_ready(sc, SST_BOOT_TIMEOUT_MS);
	if (error) {
		device_printf(sc->dev, "DSP boot timeout\n");
		/* Reset DSP on failure */
		csr = sst_shim_read(sc, SST_SHIM_CSR);
		csr |= (SST_CSR_RST | SST_CSR_STALL);
		sst_shim_write(sc, SST_SHIM_CSR, csr);
		return (error);
	}

	sc->fw.state = SST_FW_STATE_RUNNING;
	device_printf(sc->dev, "DSP firmware running\n");

	return (0);
}
