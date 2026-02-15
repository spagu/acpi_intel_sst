/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel SST SRAM Control
 * SRAM power control, BAR0 testing, memory sanitization.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <vm/vm.h>
#include <vm/pmap.h>

#include "acpi_intel_sst.h"

#define SST_SRAM_CTRL_OFFSET	0xFB000
#define SST_SRAM_CTRL_ENABLE	0x1F	/* Bits 0-4 enable SRAM */
#define SST_PCI_BAR0_PHYS	0xDF800000	/* PCI-allocated BAR0 physical address */

/*
 * sst_sram_sanitize - Dummy readback after SRAM power-up
 *
 * From Linux catpt dsp.c catpt_dsp_set_srampge():
 *   "Dummy read as the very first access after block enable
 *    to prevent byte loss in future operations."
 *
 * After SRAM blocks are ungated (PGE bits cleared in VDRTCTL0),
 * the first access to each newly-powered block must be a dummy read.
 * Without this, subsequent writes may be silently lost.
 *
 * Must be called AFTER BAR0 is allocated (sc->mem_res != NULL).
 */
void
sst_sram_sanitize(struct sst_softc *sc)
{
	int i;
	uint32_t dummy;
	int dram_ok = 0, dram_fail = 0;
	int iram_ok = 0, iram_fail = 0;

	if (sc->mem_res == NULL)
		return;

	sst_dbg(sc, SST_DBG_TRACE, "=== SRAM Sanitize (dummy reads) ===\n");

	/* Dummy read from each DRAM block (20 blocks x 32KB) */
	for (i = 0; i < 20; i++) {
		dummy = bus_read_4(sc->mem_res,
		    SST_DRAM_OFFSET + i * SST_MEMBLOCK_SIZE);
		if (dummy == SST_INVALID_REG_VALUE)
			dram_fail++;
		else
			dram_ok++;
	}
	sst_dbg(sc, SST_DBG_TRACE,
	    "  DRAM: %d/%d blocks accessible (%d dead)\n",
	    dram_ok, 20, dram_fail);

	/* Dummy read from each IRAM block (10 blocks x 32KB) */
	for (i = 0; i < 10; i++) {
		dummy = bus_read_4(sc->mem_res,
		    SST_IRAM_OFFSET + i * SST_MEMBLOCK_SIZE);
		if (dummy == SST_INVALID_REG_VALUE)
			iram_fail++;
		else
			iram_ok++;
	}
	sst_dbg(sc, SST_DBG_TRACE,
	    "  IRAM: %d/%d blocks accessible (%d dead)\n",
	    iram_ok, 10, iram_fail);

	/*
	 * Now verify DRAM write-readback at critical offsets.
	 * Test block 16 (offset 0x80000) which is where firmware
	 * DRAM block at 0x84000 falls.
	 */
	{
		uint32_t test_offsets[] = {
		    0x000000,	/* DRAM block 0 */
		    0x008000,	/* DRAM block 1 */
		    0x080000,	/* DRAM block 16 (firmware DRAM target) */
		    0x084000,	/* Exact firmware DRAM offset */
		    0x098000,	/* DRAM block 19 */
		};
		int n;

		sst_dbg(sc, SST_DBG_TRACE,
		    "  DRAM write-readback test:\n");
		for (n = 0; n < 5; n++) {
			uint32_t off = test_offsets[n];
			uint32_t orig, test_val, readback;

			orig = bus_read_4(sc->mem_res, off);
			test_val = 0xDEADBEEF;
			bus_write_4(sc->mem_res, off, test_val);
			readback = bus_read_4(sc->mem_res, off);
			/* Restore original */
			bus_write_4(sc->mem_res, off, orig);

			sst_dbg(sc, SST_DBG_TRACE,
			    "    [0x%06x] orig=0x%08x "
			    "wrote=0x%08x read=0x%08x %s\n",
			    off, orig, test_val, readback,
			    (readback == test_val) ? "OK" : "FAIL");
		}
	}
}

/* ================================================================
 * Test BAR0 Accessibility
 * Returns true if BAR0 (DSP MMIO) responds with non-0xFFFFFFFF
 * ================================================================ */

bool
sst_test_bar0(struct sst_softc *sc)
{
	uint32_t val;

	if (sc->mem_res == NULL)
		return (false);

	/* Check multiple offsets - IRAM may be all-FF when uninitialized */
	val = bus_read_4(sc->mem_res, SST_SHIM_OFFSET);
	if (val != SST_INVALID_REG_VALUE)
		return (true);
	val = bus_read_4(sc->mem_res, SST_IRAM_OFFSET);
	if (val != SST_INVALID_REG_VALUE)
		return (true);
	val = bus_read_4(sc->mem_res, SST_DRAM_OFFSET);
	if (val != SST_INVALID_REG_VALUE)
		return (true);
	return (false);
}

/* BAR0 region scanner - identify which memory regions are alive */
void
sst_scan_bar0(struct sst_softc *sc)
{
	static const struct { uint32_t off; const char *name; } regs[] = {
		{ 0x000000, "DRAM base" },
		{ 0x0A0000, "IRAM base" },
		{ 0x0FB000, "SHIM (host)" },
		{ 0x0FB004, "SHIM+4" },
		{ 0x0FB008, "SHIM PISR" },
		{ 0x0FB010, "SHIM PIMR" },
		{ 0x0FB014, "SHIM ISRX" },
		{ 0x0FB018, "SHIM ISRD" },
		{ 0x0FB028, "SHIM IMRX" },
		{ 0x0FB038, "SHIM IPCX" },
		{ 0x0FB040, "SHIM IPCD" },
		{ 0x0FB044, "SHIM+0x44" },
		{ 0x0FB078, "SHIM CLKCTL" },
		{ 0x0FB05C, "SHIM CS1" },
		{ 0x0FC000, "SSP0" },
		{ 0x0FD000, "SSP1" },
		{ 0x0FE000, "DMA0" },
		{ 0x0FF000, "DMA1" },
		{ 0x0E7000, "old SHIM (LPT)" },
	};
	uint32_t val;
	int i, alive = 0;

	if (sc->mem_res == NULL)
		return;

	sst_dbg(sc, SST_DBG_TRACE, "=== BAR0 Region Scan ===\n");
	for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++) {
		if (regs[i].off >= rman_get_size(sc->mem_res))
			continue;
		val = bus_read_4(sc->mem_res, regs[i].off);
		if (val != SST_INVALID_REG_VALUE) {
			sst_dbg(sc, SST_DBG_TRACE, "  0x%06x %-16s = 0x%08x ALIVE\n",
			    regs[i].off, regs[i].name, val);
			alive++;
		}
	}
	sst_dbg(sc, SST_DBG_TRACE, "  %d regions alive\n", alive);
}

/* ================================================================
 * Enable SRAM Power via BAR0 Control Register
 * Note: 0xFB000 is now known to be the SHIM base (host offset).
 * The "SRAM_CTRL" was actually the SHIM CSR register.
 * ================================================================ */

/*
 * Immediate SRAM status check - can be called without any device setup.
 * This is for diagnosing WHEN the SRAM gets reset during driver load.
 * Returns: 1 if SRAM is alive, 0 if dead
 */
static int __unused
sst_check_sram_immediate(const char *checkpoint)
{
	void *bar0_va;
	volatile uint32_t *sram_base;
	volatile uint32_t *ctrl_reg;
	uint32_t sram_val, ctrl_val;
	int alive;

	bar0_va = pmap_mapdev_attr(SST_PCI_BAR0_PHYS, 0x100000, VM_MEMATTR_UNCACHEABLE);
	if (bar0_va == NULL) {
		printf("sst: [%s] Failed to map BAR0\n", checkpoint);
		return 0;
	}

	sram_base = (volatile uint32_t *)bar0_va;
	ctrl_reg = (volatile uint32_t *)((char *)bar0_va + SST_SRAM_CTRL_OFFSET);

	__asm __volatile("mfence" ::: "memory");
	sram_val = *sram_base;
	ctrl_val = *ctrl_reg;

	alive = (sram_val != 0xFFFFFFFF);

	printf("sst: [%s] SRAM[0]=0x%08x CTRL=0x%08x => %s\n",
	    checkpoint, sram_val, ctrl_val,
	    alive ? "ALIVE" : "DEAD");

	pmap_unmapdev(bar0_va, 0x100000);
	return alive;
}

/*
 * Check if SRAM is active and try to enable it.
 *
 * IMPORTANT: Kernel driver writes cannot trigger the SRAM hardware.
 * Manual activation via /dev/mem is required BEFORE loading driver.
 *
 * This function:
 * 1. Checks if SRAM was pre-activated (via dd before driver load)
 * 2. If active, returns success
 * 3. If not, tries atomic 32-bit writes (which write correctly but hardware ignores)
 * 4. Prints instructions for manual activation
 */
int
sst_enable_sram_direct(device_t dev)
{
	void *bar0_va;
	volatile uint32_t *ctrl_reg;
	volatile uint32_t *sram_base;
	uint32_t ctrl, test_val;
	const uint32_t clear_val = 0x84800400;
	const uint32_t set_val = 0x8480041f;

	device_printf(dev, "=== SRAM Check & Enable ===\n");

	/* Map with UNCACHED attribute - atomic 32-bit writes work correctly */
	bar0_va = pmap_mapdev_attr(SST_PCI_BAR0_PHYS, 0x100000, VM_MEMATTR_UNCACHEABLE);
	if (bar0_va == NULL) {
		device_printf(dev, "  Failed to map BAR0 at 0x%x\n", SST_PCI_BAR0_PHYS);
		return (ENOMEM);
	}

	sram_base = (volatile uint32_t *)bar0_va;
	ctrl_reg = (volatile uint32_t *)((char *)bar0_va + SST_SRAM_CTRL_OFFSET);

	__asm __volatile("mfence" ::: "memory");
	test_val = *sram_base;
	ctrl = *ctrl_reg;
	device_printf(dev, "  SRAM[0]=0x%08x, CTRL=0x%08x\n", test_val, ctrl);

	/* Check if SRAM was pre-activated via dd */
	if (test_val != 0xFFFFFFFF) {
		device_printf(dev, "  SRAM is ACTIVE! (pre-activated via /dev/mem)\n");
		pmap_unmapdev(bar0_va, 0x100000);
		return (0);
	}

	/* SRAM is not active - try atomic 32-bit writes */
	device_printf(dev, "  SRAM is DEAD. Attempting atomic 32-bit enable...\n");

	/* Clear bits 0-4 */
	__asm __volatile("mfence" ::: "memory");
	*ctrl_reg = clear_val;
	__asm __volatile("mfence" ::: "memory");
	ctrl = *ctrl_reg;
	device_printf(dev, "  Wrote 0x%08x, read back 0x%08x\n", clear_val, ctrl);

	DELAY(100000); /* 100ms */

	/* Set bits 0-4 */
	__asm __volatile("mfence" ::: "memory");
	*ctrl_reg = set_val;
	__asm __volatile("mfence" ::: "memory");

	DELAY(200000); /* 200ms */

	test_val = *sram_base;
	ctrl = *ctrl_reg;
	device_printf(dev, "  After enable: SRAM[0]=0x%08x, CTRL=0x%08x\n", test_val, ctrl);

	if (test_val != 0xFFFFFFFF) {
		device_printf(dev, "  SUCCESS! SRAM is now ACTIVE.\n");
		pmap_unmapdev(bar0_va, 0x100000);
		return (0);
	}

	/* Hardware didn't process our writes (kernel limitation) */
	device_printf(dev, "\n");
	device_printf(dev, "  *** SRAM REQUIRES MANUAL ACTIVATION ***\n");
	device_printf(dev, "  Kernel writes reach hardware but don't trigger the SRAM state machine.\n");
	device_printf(dev, "  Manual dd via /dev/mem works. Run BEFORE loading driver:\n");
	device_printf(dev, "\n");
	device_printf(dev, "  # Activate SRAM:\n");
	device_printf(dev, "  printf '\\x00\\x04\\x80\\x84' | dd of=/dev/mem bs=1 seek=$((0xdf8fb000)) conv=notrunc 2>/dev/null\n");
	device_printf(dev, "  sleep 0.1\n");
	device_printf(dev, "  printf '\\x1f\\x04\\x80\\x84' | dd of=/dev/mem bs=1 seek=$((0xdf8fb000)) conv=notrunc 2>/dev/null\n");
	device_printf(dev, "  sleep 0.1\n");
	device_printf(dev, "  # Then load driver:\n");
	device_printf(dev, "  kldload acpi_intel_sst\n");
	device_printf(dev, "\n");

	pmap_unmapdev(bar0_va, 0x100000);
	return (EIO);
}

int
sst_enable_sram(struct sst_softc *sc)
{
	uint32_t ctrl, ctrl_after, test_val;
	int retries, write_attempts;

	if (sc->mem_res == NULL) {
		device_printf(sc->dev, "SRAM Enable: BAR0 not allocated\n");
		return (ENXIO);
	}

	sst_dbg(sc, SST_DBG_OPS, "=== SRAM Power Enable Sequence ===\n");
	sst_dbg(sc, SST_DBG_OPS, "  Using BAR0 at: 0x%lx\n", rman_get_start(sc->mem_res));

	/* Check if SRAM is already accessible */
	test_val = bus_read_4(sc->mem_res, 0);
	if (test_val != SST_INVALID_REG_VALUE) {
		sst_dbg(sc, SST_DBG_LIFE, "  SRAM already accessible: 0x%08x\n", test_val);
		return (0);
	}

	/* Read current control register value */
	ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	sst_dbg(sc, SST_DBG_OPS, "  SRAM_CTRL (0x%x) before: 0x%08x\n",
	    SST_SRAM_CTRL_OFFSET, ctrl);

	/*
	 * Two-step SRAM activation (discovered via manual /dev/mem testing):
	 * 1. CLEAR enable bits (bits 0-4) → hardware enters reset/idle
	 * 2. Wait 100ms
	 * 3. SET enable bits → hardware transitions to active
	 * When working: SRAM_CTRL changes from 0x8480041f to 0x78663178
	 */

	/* Phase A: Clear enable bits first */
	ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	ctrl &= ~SST_SRAM_CTRL_ENABLE;
	bus_write_4(sc->mem_res, SST_SRAM_CTRL_OFFSET, ctrl);
	DELAY(100000);  /* 100ms */

	ctrl_after = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	sst_dbg(sc, SST_DBG_TRACE, "  After clear: SRAM_CTRL = 0x%08x\n", ctrl_after);

	/* Phase B: Set enable bits */
	ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	ctrl |= SST_SRAM_CTRL_ENABLE;
	bus_write_4(sc->mem_res, SST_SRAM_CTRL_OFFSET, ctrl);
	DELAY(100000);  /* 100ms */

	ctrl_after = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	sst_dbg(sc, SST_DBG_TRACE, "  After set: SRAM_CTRL = 0x%08x\n", ctrl_after);

	/* Check if SRAM is now accessible */
	for (retries = 0; retries < 20; retries++) {
		test_val = bus_read_4(sc->mem_res, 0);
		if (test_val != SST_INVALID_REG_VALUE) {
			sst_dbg(sc, SST_DBG_LIFE, "  SRAM enabled! IRAM[0]=0x%08x CTRL=0x%08x\n",
			    test_val, bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET));
			return (0);
		}
		DELAY(10000);
	}

	/* Retry: try multiple toggle cycles */
	for (write_attempts = 0; write_attempts < 3; write_attempts++) {
		sst_dbg(sc, SST_DBG_TRACE, "  Toggle attempt %d...\n", write_attempts + 1);

		/* Clear */
		ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
		ctrl &= ~SST_SRAM_CTRL_ENABLE;
		bus_write_4(sc->mem_res, SST_SRAM_CTRL_OFFSET, ctrl);
		DELAY(200000);

		/* Set */
		ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
		ctrl |= SST_SRAM_CTRL_ENABLE;
		bus_write_4(sc->mem_res, SST_SRAM_CTRL_OFFSET, ctrl);
		DELAY(200000);

		ctrl_after = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
		test_val = bus_read_4(sc->mem_res, 0);
		sst_dbg(sc, SST_DBG_TRACE, "  Attempt %d: CTRL=0x%08x IRAM[0]=0x%08x\n",
		    write_attempts + 1, ctrl_after, test_val);

		if (test_val != SST_INVALID_REG_VALUE) {
			sst_dbg(sc, SST_DBG_LIFE, "  SRAM enabled after toggle!\n");
			return (0);
		}
	}

	device_printf(sc->dev, "  SRAM enable FAILED\n");
	sst_dbg(sc, SST_DBG_TRACE, "  Final SRAM_CTRL: 0x%08x\n",
	    bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET));

	return (EIO);
}
