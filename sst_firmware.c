/*-
 * SPDX-License-Identifier: BSD-3-Clause
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
 * Load a single block to DSP memory via MMIO
 *
 * Based on Linux catpt_load_block():
 *   IRAM (type 1) -> IRAM region
 *   DRAM (type 2) -> DRAM region
 *   INSTANCE (type 3) -> DRAM region (module initial state)
 *
 * Linux catpt uses DMA for this, but MMIO writes should also work
 * (the older sst-dsp.c driver uses memcpy_toio for SRAM access).
 */
static int
sst_fw_load_block(struct sst_softc *sc, const struct sst_block_header *blk,
		  const uint8_t *data)
{
	bus_addr_t offset;
	size_t max_size;
	const char *type_name;
	uint32_t readback;

	switch (blk->ram_type) {
	case SST_BLK_TYPE_IRAM:
		offset = SST_IRAM_OFFSET + blk->ram_offset;
		max_size = SST_IRAM_SIZE;
		type_name = "IRAM";
		break;
	case SST_BLK_TYPE_DRAM:
	case SST_BLK_TYPE_INSTANCE:
		/* Both DRAM and INSTANCE blocks go to DRAM (catpt: default case) */
		offset = SST_DRAM_OFFSET + blk->ram_offset;
		max_size = SST_DRAM_SIZE;
		type_name = (blk->ram_type == SST_BLK_TYPE_DRAM) ?
		    "DRAM" : "INSTANCE";
		break;
	default:
		device_printf(sc->dev, "Unknown block type: %u\n",
		    blk->ram_type);
		return (EINVAL);
	}

	if (blk->ram_offset + blk->size > max_size) {
		device_printf(sc->dev,
		    "Block exceeds %s: off=0x%x, size=0x%x, max=0x%zx\n",
		    type_name, blk->ram_offset, blk->size, max_size);
		return (EINVAL);
	}

	device_printf(sc->dev,
	    "  Block: type=%s(%u) offset=0x%x size=0x%x -> BAR0+0x%lx\n",
	    type_name, blk->ram_type, blk->ram_offset, blk->size,
	    (unsigned long)offset);

	/*
	 * Write block data to DSP SRAM.
	 *
	 * IMPORTANT: Use 32-bit (DWORD) writes, not byte writes!
	 * PCH SRAM controllers on Intel Broadwell may not support
	 * sub-DWORD writes via MMIO. bus_write_region_1 uses byte
	 * writes that don't persist in SRAM.
	 *
	 * Write aligned 32-bit words first, then handle any remainder.
	 */
	{
		uint32_t dwords = blk->size / 4;
		uint32_t remainder = blk->size % 4;
		uint32_t k;
		const uint8_t *src = data;

		/* Write 32-bit words */
		for (k = 0; k < dwords; k++) {
			uint32_t val;
			memcpy(&val, src + (k * 4), 4);
			bus_write_4(sc->mem_res, offset + (k * 4), val);
		}

		/* Handle remainder bytes (pack into a 32-bit write) */
		if (remainder > 0) {
			uint32_t val = 0;
			memcpy(&val, src + (dwords * 4), remainder);
			bus_write_4(sc->mem_res, offset + (dwords * 4), val);
		}
	}

	/* Readback verification - check first and last 4 bytes */
	if (blk->size >= 4) {
		readback = bus_read_4(sc->mem_res, offset);
		if (readback == SST_INVALID_REG_VALUE) {
			device_printf(sc->dev,
			    "  WARNING: Readback 0xFFFFFFFF at 0x%lx"
			    " - SRAM may not be accessible!\n",
			    (unsigned long)offset);
		} else {
			uint32_t expected;
			memcpy(&expected, data, 4);
			if (readback != expected) {
				device_printf(sc->dev,
				    "  WARNING: Readback mismatch at 0x%lx:"
				    " wrote=0x%08x read=0x%08x\n",
				    (unsigned long)offset, expected, readback);
			}
		}
	}

	return (0);
}

/*
 * Parse and load a module
 *
 * Based on Linux catpt_load_module().
 *
 * IMPORTANT: mod_size is the size of the module DATA (all block headers +
 * block data), NOT including the module header itself. To advance to the
 * next module: offset += sizeof(mod_header) + mod_size.
 *
 * Linux catpt also subtracts 4 from the entry point:
 *   "DSP expects address from module header subtracted by 4"
 */
static int
sst_fw_load_module(struct sst_softc *sc, const uint8_t *data, size_t size,
		   size_t *consumed)
{
	const struct sst_module_header *mod;
	const struct sst_block_header *blk;
	size_t total_size;
	uint32_t offset;
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

	/* Total size in file = module header + mod_size (data portion) */
	total_size = sizeof(struct sst_module_header) + mod->mod_size;
	if (total_size > size) {
		device_printf(sc->dev,
		    "Module exceeds data: hdr(%zu)+data(%u) = %zu > %zu\n",
		    sizeof(struct sst_module_header), mod->mod_size,
		    total_size, size);
		return (EINVAL);
	}

	device_printf(sc->dev,
	    "Module[%u]: slot=%u, size=%u, blocks=%u, entry=0x%08x,"
	    " persist=%u, scratch=%u\n",
	    mod->module_id, mod->slot, mod->mod_size, mod->blocks,
	    mod->entry_point, mod->persistent_size, mod->scratch_size);

	/* Store module info for later IPC use */
	if (mod->module_id < SST_MAX_MODULES) {
		sc->fw.mod[mod->module_id].entry_point = mod->entry_point;
		sc->fw.mod[mod->module_id].persistent_size =
		    mod->persistent_size;
		sc->fw.mod[mod->module_id].scratch_size = mod->scratch_size;
		sc->fw.mod[mod->module_id].present = true;
	}

	/*
	 * Store entry point from first module.
	 * Linux catpt: "DSP expects address from module header subtracted by 4"
	 * Only subtract if entry point is non-zero (zero means IRAM base).
	 */
	if (sc->fw.entry_point == 0 && mod->entry_point != 0)
		sc->fw.entry_point = mod->entry_point - 4;

	/* Load each block - starting from after module header */
	offset = sizeof(struct sst_module_header);
	for (i = 0; i < mod->blocks; i++) {
		if (offset + sizeof(struct sst_block_header) > total_size) {
			device_printf(sc->dev,
			    "Block %u header exceeds module\n", i);
			return (EINVAL);
		}

		blk = (const struct sst_block_header *)(data + offset);
		offset += sizeof(struct sst_block_header);

		if (offset + blk->size > total_size) {
			device_printf(sc->dev,
			    "Block %u data exceeds module: offset=%u"
			    " blk_size=%u total=%zu\n",
			    i, offset, blk->size, total_size);
			return (EINVAL);
		}

		error = sst_fw_load_block(sc, blk, data + offset);
		if (error)
			return (error);

		offset += blk->size;
	}

	*consumed = total_size;
	return (0);
}

/*
 * Parse firmware and load modules to DSP
 *
 * Based on Linux catpt_load_firmware().
 *
 * The Intel SST firmware (IntcSST2.bin) has this layout:
 *   [sst_fw_header]  - signature "$SST", file_size, modules count
 *   [sst_module_header 0] - signature "$SST", mod_size, blocks, entry_point
 *     [sst_block_header 0] [block_data 0]
 *     [sst_block_header 1] [block_data 1]
 *     ...
 *   [sst_module_header 1]
 *     [sst_block_header 0] [block_data 0]
 *     ...
 *
 * Linux catpt validates: fw_hdr.signature == mod_hdr.signature == "$SST"
 * Both SST_FW_SIGNATURE and SST_MOD_SIGNATURE are "$SST".
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

	/* Dump first bytes after header for debugging */
	ptr = sc->fw.data + sizeof(struct sst_fw_header);
	remaining = sc->fw.size - sizeof(struct sst_fw_header);

	if (remaining >= 4) {
		device_printf(sc->dev,
		    "First module bytes: %02x %02x %02x %02x (\"%.4s\")\n",
		    ptr[0], ptr[1], ptr[2], ptr[3], (const char *)ptr);
	}

	/*
	 * Linux catpt_load_firmware() validates that each module signature
	 * matches the firmware header signature (both "$SST").
	 * Parse modules sequentially.
	 */
	for (i = 0; i < hdr->modules; i++) {
		if (remaining < sizeof(struct sst_module_header)) {
			device_printf(sc->dev,
			    "Insufficient data for module %u: %zu bytes left\n",
			    i, remaining);
			return (EINVAL);
		}

		/* Validate module signature matches firmware signature */
		if (memcmp(ptr, hdr->signature, SST_FW_SIGN_SIZE) != 0) {
			device_printf(sc->dev,
			    "Module %u signature mismatch: \"%.4s\" != \"%.4s\"\n",
			    i, (const char *)ptr, hdr->signature);
			return (EINVAL);
		}

		error = sst_fw_load_module(sc, ptr, remaining, &consumed);
		if (error)
			return (error);

		ptr += consumed;
		remaining -= consumed;
	}

	device_printf(sc->dev,
	    "All %u modules loaded, %zu bytes remaining in firmware\n",
	    hdr->modules, remaining);

	return (0);
}

/*
 * Allocate DRAM regions for module persistent/scratch memory.
 *
 * The catpt firmware expects the host to pre-allocate DRAM space
 * for each module's persistent and scratch state.  These offsets
 * are passed in the ALLOC_STREAM IPC request.
 *
 * We use a simple bump allocator starting after the firmware's
 * mailbox region to avoid conflicts.
 */
void
sst_fw_alloc_module_regions(struct sst_softc *sc)
{
	uint32_t alloc_start;
	uint32_t i;

	/*
	 * Allocate from the beginning of DRAM.
	 *
	 * The catpt firmware on WPT places its mailbox near the END
	 * of DRAM (~0x8DC98).  The firmware's IRAM-only code doesn't
	 * use early DRAM, so we can safely allocate module regions
	 * starting from offset 0.  The firmware runtime heap/mailbox
	 * lives at the top of DRAM and won't conflict.
	 */
	alloc_start = 0;

	sc->fw.dram_alloc_next = alloc_start;

	device_printf(sc->dev,
	    "DRAM allocator: start=0x%x (mbox ends ~0x%x)\n",
	    alloc_start,
	    (uint32_t)(sc->ipc.mbox_out + SST_MBOX_SIZE_OUT));

	for (i = 0; i < SST_MAX_MODULES; i++) {
		if (!sc->fw.mod[i].present)
			continue;

		/* Allocate persistent memory */
		if (sc->fw.mod[i].persistent_size > 0) {
			sc->fw.mod[i].persistent_offset =
			    sc->fw.dram_alloc_next;
			sc->fw.dram_alloc_next +=
			    roundup2(sc->fw.mod[i].persistent_size, 64);
		}

		/* Allocate scratch memory */
		if (sc->fw.mod[i].scratch_size > 0) {
			sc->fw.mod[i].scratch_offset =
			    sc->fw.dram_alloc_next;
			sc->fw.dram_alloc_next +=
			    roundup2(sc->fw.mod[i].scratch_size, 64);
		}

		if (sc->fw.mod[i].persistent_size > 0 ||
		    sc->fw.mod[i].scratch_size > 0) {
			device_printf(sc->dev,
			    "  Module %u: persist @0x%x (%u bytes) "
			    "scratch @0x%x (%u bytes)\n",
			    i,
			    sc->fw.mod[i].persistent_offset,
			    sc->fw.mod[i].persistent_size,
			    sc->fw.mod[i].scratch_offset,
			    sc->fw.mod[i].scratch_size);
		}
	}

	/* Warn if allocations reach the mailbox area */
	if (sc->fw.dram_alloc_next > sc->ipc.mbox_in) {
		device_printf(sc->dev,
		    "WARNING: DRAM alloc 0x%x overlaps mailbox 0x%lx!\n",
		    sc->fw.dram_alloc_next,
		    (unsigned long)sc->ipc.mbox_in);
	}

	device_printf(sc->dev,
	    "DRAM allocator: used 0x%x - 0x%x (%u bytes, mbox@0x%lx)\n",
	    alloc_start, sc->fw.dram_alloc_next,
	    sc->fw.dram_alloc_next - alloc_start,
	    (unsigned long)sc->ipc.mbox_in);
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
	sc->fw.dram_alloc_next = 0;

	/* Assume all stages supported until probed otherwise */
	sc->fw.has_volume  = true;
	sc->fw.has_biquad  = true;
	sc->fw.has_limiter = true;
	sc->fw.caps_probed = false;

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

	/* Reset caps to optimistic defaults for next load */
	sc->fw.has_volume  = true;
	sc->fw.has_biquad  = true;
	sc->fw.has_limiter = true;
	sc->fw.caps_probed = false;
}

/*
 * Reload firmware to SRAM from cached host memory.
 *
 * After suspend (D3), SRAM contents are lost. The firmware(9) handle
 * and sc->fw.data remain valid in host memory. This function re-writes
 * all modules to IRAM/DRAM without re-fetching from disk.
 */
int
sst_fw_reload(struct sst_softc *sc)
{
	int error;

	if (sc->fw.data == NULL || sc->fw.size == 0) {
		device_printf(sc->dev, "No cached firmware for reload\n");
		return (EINVAL);
	}

	device_printf(sc->dev, "Reloading firmware to SRAM (%zu bytes)...\n",
	    sc->fw.size);

	sst_dsp_stall(sc, true);

	/* sst_fw_parse() re-writes all modules to IRAM/DRAM */
	error = sst_fw_parse(sc);
	if (error) {
		device_printf(sc->dev, "Firmware reload failed: %d\n", error);
		return (error);
	}

	sc->fw.state = SST_FW_STATE_LOADED;
	return (0);
}

/*
 * Boot DSP with loaded firmware
 *
 * Based on Linux catpt catpt_boot_firmware():
 * 1. Stall DSP (STALL=BIT(10), not BIT(0)!)
 * 2. Firmware was already loaded to IRAM/DRAM
 * 3. Unstall DSP - DSP begins execution
 * 4. Wait for FW_READY IPC message (250ms timeout in Linux)
 *
 * The DSP should already be in stalled+unreset state from
 * the SHIM configuration done during attach.
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

	device_printf(sc->dev, "Booting DSP (catpt sequence)...\n");

	/* Read initial CSR state */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  Initial CSR: 0x%08x (STALL=%d RST=%d)\n",
	    csr, !!(csr & SST_CSR_STALL), !!(csr & SST_CSR_RST));

	/*
	 * Step 1: Ensure DSP is stalled (set STALL at BIT(10))
	 * DSP should already be stalled from register defaults,
	 * but ensure it explicitly.
	 */
	error = sst_dsp_stall(sc, true);
	if (error) {
		device_printf(sc->dev, "  Failed to stall DSP\n");
	}
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	device_printf(sc->dev, "  After stall: CSR=0x%08x (STALL=%d)\n",
	    csr, !!(csr & SST_CSR_STALL));

	/*
	 * Step 2: Firmware data is already loaded to IRAM/DRAM
	 * (done by sst_fw_load/sst_fw_parse before this function)
	 *
	 * Verify SRAM is accessible by reading back some data.
	 */
	{
		uint32_t iram0, iram4, dram0, dram4;

		iram0 = bus_read_4(sc->mem_res, SST_IRAM_OFFSET);
		iram4 = bus_read_4(sc->mem_res, SST_IRAM_OFFSET + 4);
		dram0 = bus_read_4(sc->mem_res, SST_DRAM_OFFSET);
		dram4 = bus_read_4(sc->mem_res, SST_DRAM_OFFSET + 4);
		device_printf(sc->dev,
		    "  SRAM check: IRAM[0]=0x%08x [4]=0x%08x"
		    " DRAM[0]=0x%08x [4]=0x%08x\n",
		    iram0, iram4, dram0, dram4);
		if (iram0 == 0xFFFFFFFF && dram0 == 0xFFFFFFFF) {
			device_printf(sc->dev,
			    "  WARNING: SRAM reads 0xFFFFFFFF!"
			    " Firmware may not have been written.\n");
		}
	}

	/*
	 * Step 3: Reset IPC state and prepare for FW_READY detection
	 *
	 * CRITICAL: IPCD may contain stale data from a previous boot
	 * (Windows/BIOS). We cannot reliably clear IPCD - the BUSY/DONE
	 * bits are hardware-controlled. Instead, record the current value
	 * and only accept FW_READY if IPCD CHANGES to a new value with
	 * the fw_ready bit (bit 29) set.
	 *
	 * The correct way to clear the DSP doorbell is to write
	 * ISC.IPCDB (not write 0 to IPCD).
	 */
	sc->ipc.ready = false;
	sst_shim_write(sc, SST_SHIM_IPCX, 0);
	/* Clear ISC doorbell bits (proper way to ACK stale IPC) */
	sst_shim_update_bits(sc, SST_SHIM_ISRX,
	    SST_IMC_IPCDB | SST_IMC_IPCCD,
	    SST_IMC_IPCDB | SST_IMC_IPCCD);

	/* Unmask IPC doorbell and completion interrupts */
	sst_shim_update_bits(sc, SST_SHIM_IMRX,
	    SST_IMC_IPCDB | SST_IMC_IPCCD, 0);

	/* Record stale IPCD value BEFORE unstall */
	{
		uint32_t stale_ipcd = sst_shim_read(sc, SST_SHIM_IPCD);
		uint32_t stale_isrx = sst_shim_read(sc, SST_SHIM_ISRX);
		device_printf(sc->dev,
		    "  Pre-boot: IPCD=0x%08x ISRX=0x%08x IMRX=0x%08x\n",
		    stale_ipcd, stale_isrx,
		    sst_shim_read(sc, SST_SHIM_IMRX));

		/*
		 * Step 4: Clear STALL - DSP starts executing firmware.
		 * RST is already cleared (done in SHIM config during attach).
		 * Linux catpt: only toggles STALL, RST stays cleared.
		 */
		device_printf(sc->dev,
		    "  Clearing STALL (BIT 10) to start DSP...\n");
		error = sst_dsp_stall(sc, false);
		if (error) {
			device_printf(sc->dev, "  Failed to unstall DSP!\n");
			return (error);
		}
		csr = sst_shim_read(sc, SST_SHIM_CSR);
		device_printf(sc->dev,
		    "  After unstall: CSR=0x%08x (STALL=%d RST=%d)\n",
		    csr, !!(csr & SST_CSR_STALL), !!(csr & SST_CSR_RST));

		device_printf(sc->dev,
		    "DSP running, waiting for FW ready...\n");

		/*
		 * Step 5: Wait for firmware to signal ready via IPC
		 *
		 * Detection strategy:
		 * 1. Check ISC.IPCDB (doorbell) - indicates NEW IPC from DSP
		 * 2. If doorbell set, read IPCD and check fw_ready bit (29)
		 * 3. Also check if IPCD changed from stale value
		 * 4. Only accept if BOTH doorbell and fw_ready are set
		 *
		 * Fallback: if IPCD changes to a value with fw_ready=1,
		 * accept even without doorbell (some HW configs).
		 */
		{
			int elapsed = 0;
			int poll_ms = 10;
			uint32_t ipcd, isrx;

			while (elapsed < SST_BOOT_TIMEOUT_MS) {
				isrx = sst_shim_read(sc, SST_SHIM_ISRX);
				ipcd = sst_shim_read(sc, SST_SHIM_IPCD);

				/*
				 * Check for NEW IPC from DSP:
				 * - ISC doorbell (IPCDB) is set, OR
				 * - IPCD value changed from stale
				 */
				if ((isrx & SST_IMC_IPCDB) ||
				    (ipcd != stale_ipcd)) {
					/*
					 * Validate: must have fw_ready
					 * bit (29) AND busy bit (31)
					 */
					if ((ipcd & SST_IPC_FW_READY) &&
					    (ipcd & SST_IPC_BUSY)) {
						device_printf(sc->dev,
						    "DSP FW_READY! "
						    "IPCD=0x%08x "
						    "(was 0x%08x) "
						    "ISRX=0x%08x "
						    "(%dms)\n",
						    ipcd, stale_ipcd,
						    isrx, elapsed);
						/* ACK doorbell */
						sst_shim_update_bits(sc,
						    SST_SHIM_ISRX,
						    SST_IMC_IPCDB,
						    SST_IMC_IPCDB);
						sc->ipc.ready = true;
						break;
					}
					/*
					 * IPCD changed but no fw_ready -
					 * log it but keep waiting
					 */
					if (ipcd != stale_ipcd) {
						device_printf(sc->dev,
						    "  IPCD changed: "
						    "0x%08x -> 0x%08x "
						    "(fw_ready=%d) "
						    "(%dms)\n",
						    stale_ipcd, ipcd,
						    !!(ipcd &
						    SST_IPC_FW_READY),
						    elapsed);
						stale_ipcd = ipcd;
					}
				}

				/* Periodic status dump */
				if (elapsed > 0 && elapsed % 1000 == 0) {
					device_printf(sc->dev,
					    "  Boot poll %ds: "
					    "CSR=0x%08x "
					    "IPCD=0x%08x "
					    "ISRX=0x%08x\n",
					    elapsed / 1000,
					    sst_shim_read(sc,
					    SST_SHIM_CSR),
					    ipcd, isrx);
				}

				DELAY(poll_ms * 1000);
				elapsed += poll_ms;
			}

			if (!sc->ipc.ready)
				error = ETIMEDOUT;
			else
				error = 0;
		}
	}

	if (error) {
		/* Dump diagnostic info */
		csr = sst_shim_read(sc, SST_SHIM_CSR);
		device_printf(sc->dev,
		    "DSP boot timeout! CSR=0x%08x IPCD=0x%08x IPCX=0x%08x\n",
		    csr,
		    sst_shim_read(sc, SST_SHIM_IPCD),
		    sst_shim_read(sc, SST_SHIM_IPCX));
		device_printf(sc->dev,
		    "  ISRX=0x%08x IMRX=0x%08x ISRD=0x%08x IMRD=0x%08x\n",
		    sst_shim_read(sc, SST_SHIM_ISRX),
		    sst_shim_read(sc, SST_SHIM_IMRX),
		    sst_shim_read(sc, SST_SHIM_ISRD),
		    sst_shim_read(sc, SST_SHIM_IMRD));

		/* Reset DSP on failure */
		sst_dsp_stall(sc, true);
		sst_dsp_reset(sc, true);
		return (error);
	}

	sc->fw.state = SST_FW_STATE_RUNNING;
	device_printf(sc->dev, "DSP firmware running!\n");

	/*
	 * Parse FW_READY mailbox from DSP
	 *
	 * Per Linux catpt protocol (ipc.c catpt_dsp_process_response):
	 * 1. Extract mailbox_address from IPCD bits[28:0]
	 * 2. Shift left by 3 to recover actual byte offset in BAR0
	 * 3. Read catpt_fw_ready struct from that offset
	 * 4. Use inbox_offset/outbox_offset from struct for future IPC
	 *
	 * The firmware originally right-shifted the address by 3 to fit
	 * it into 29 bits of the IPCD register.
	 */
	{
		struct sst_fw_ready fw_ready;
		uint32_t ipcd_val, mbox_offset;
		uint32_t k, *dst;

		/* Re-read IPCD to get the mailbox address */
		ipcd_val = sst_shim_read(sc, SST_SHIM_IPCD);

		/*
		 * Extract mailbox offset from IPCD header.
		 * catpt format: bits[28:0] = mailbox_address >> 3
		 * Actual byte offset = mailbox_address << 3
		 */
		mbox_offset = SST_IPC_MBOX_OFFSET(ipcd_val);

		device_printf(sc->dev,
		    "FW_READY: IPCD=0x%08x mbox_addr_raw=0x%x "
		    "mbox_offset=0x%x\n",
		    ipcd_val, ipcd_val & SST_IPC_MBOX_ADDR_MASK,
		    mbox_offset);

		/*
		 * Sanity check: offset + read size must fit within BAR0.
		 * The read loop below reads sizeof(fw_ready) bytes starting
		 * at mbox_offset, so the entire range must be valid.
		 */
		if (mbox_offset > SST_DRAM_OFFSET + SST_DRAM_SIZE -
		    sizeof(fw_ready)) {
			device_printf(sc->dev,
			    "WARNING: mbox_offset 0x%x out of DRAM range "
			    "(need %zu bytes), using DRAM base\n",
			    mbox_offset, sizeof(fw_ready));
			mbox_offset = SST_DRAM_OFFSET;
		}

		/* Also dump what's at DRAM[0] for comparison */
		device_printf(sc->dev,
		    "  DRAM[0x000000]=0x%08x DRAM[0x%06x]=0x%08x\n",
		    bus_read_4(sc->mem_res, SST_DRAM_OFFSET),
		    mbox_offset,
		    bus_read_4(sc->mem_res, mbox_offset));

		/* Read fw_ready struct from correct mailbox offset */
		dst = (uint32_t *)&fw_ready;
		for (k = 0; k < sizeof(fw_ready) / 4; k++) {
			dst[k] = bus_read_4(sc->mem_res,
			    mbox_offset + k * 4);
		}

		device_printf(sc->dev,
		    "FW_READY mailbox @0x%x: inbox=0x%x outbox=0x%x "
		    "in_sz=0x%x out_sz=0x%x info_sz=%u\n",
		    mbox_offset,
		    fw_ready.inbox_offset, fw_ready.outbox_offset,
		    fw_ready.inbox_size, fw_ready.outbox_size,
		    fw_ready.fw_info_size);

		/* Update IPC mailbox configuration if offsets look valid */
		if (fw_ready.inbox_offset < SST_DRAM_SIZE &&
		    fw_ready.outbox_offset < SST_DRAM_SIZE &&
		    fw_ready.inbox_size > 0 &&
		    fw_ready.outbox_size > 0) {
			/*
			 * catpt fw_ready naming is from HOST perspective:
			 *   inbox_offset  = where host receives (DSP→Host)
			 *   outbox_offset = where host sends    (Host→DSP)
			 * Our mbox_in = Host→DSP = fw outbox_offset
			 * Our mbox_out = DSP→Host = fw inbox_offset
			 */
			sc->ipc.mbox_in = fw_ready.outbox_offset;
			sc->ipc.mbox_out = fw_ready.inbox_offset;
			device_printf(sc->dev,
			    "IPC mailbox configured: "
			    "in=BAR0+0x%lx (%lu bytes) "
			    "out=BAR0+0x%lx (%lu bytes)\n",
			    (unsigned long)sc->ipc.mbox_in,
			    (unsigned long)fw_ready.inbox_size,
			    (unsigned long)sc->ipc.mbox_out,
			    (unsigned long)fw_ready.outbox_size);
		} else {
			device_printf(sc->dev,
			    "WARNING: fw_ready offsets invalid, "
			    "keeping defaults\n");
		}

		/* Acknowledge FW_READY: clear BUSY, set DONE in IPCD */
		sst_shim_update_bits(sc, SST_SHIM_IPCD,
		    SST_IPC_BUSY | SST_IPC_DONE, SST_IPC_DONE);

		/* Print firmware info string if available */
		if (fw_ready.fw_info_size > 0 &&
		    fw_ready.fw_info_size <= SST_FW_INFO_SIZE_MAX) {
			fw_ready.fw_info[SST_FW_INFO_SIZE_MAX - 1] = '\0';
			device_printf(sc->dev, "FW info: %s\n",
			    fw_ready.fw_info);
		}
	}

	/*
	 * Re-enable DCLCGE (step 11 of catpt_dsp_power_up).
	 *
	 * DCLCGE was left disabled during power_up so that MMIO writes
	 * to SRAM would not be clock-gated. Now that firmware is loaded
	 * and the DSP is running, we can safely re-enable it.
	 */
	if (sc->shim_res != NULL) {
		uint32_t vdrtctl2;

		vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
		vdrtctl2 |= SST_VDRTCTL2_DCLCGE;
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
		device_printf(sc->dev, "DCLCGE re-enabled after FW boot\n");
	}

	return (0);
}
