/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel Smart Sound Technology (SST) ACPI Driver for FreeBSD
 * Target: Intel Broadwell-U (INT3438)
 *
 * This driver attempts to enable audio on Dell XPS 13 9343 and similar
 * Broadwell-U platforms. It implements two strategies:
 *
 * 1. Primary: SST DSP path (BAR0 MMIO for IRAM/DRAM/SHIM access)
 * 2. Fallback: Enable PCH HDA controller by modifying RCBA Function
 *    Disable register, allowing the standard hdac(4) driver to handle audio
 *
 * Additionally, it probes the I2C0 controller to verify communication
 * with the RT286/ALC3263 audio codec at address 0x1C.
 */

#include <sys/param.h>
#include <sys/module.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/malloc.h>
#include <sys/systm.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <machine/bus.h>
#include <machine/resource.h>
#include <machine/cpufunc.h>
#include <x86/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <x86/pci_cfgreg.h>

#include "acpi_intel_sst.h"

/* Intel SST PCI IDs */
#define PCI_VENDOR_INTEL	0x8086
#define PCI_DEVICE_SST_BDW	0x9CB6
#define PCI_DEVICE_SST_HSW	0x9C76 /* Haswell pending testing */

/* SST_DRV_VERSION and SST_DRV_VERSION_NUM provided by Makefile from VERSION file */

/* Forward declarations */
static int sst_acpi_probe(device_t dev);
static int sst_acpi_attach(device_t dev);
static int sst_acpi_detach(device_t dev);
static int sst_acpi_suspend(device_t dev);
static int sst_acpi_resume(device_t dev);
static int sst_pci_probe(device_t dev);
static int sst_pci_attach(device_t dev);
static int sst_pci_detach(device_t dev);
static void sst_intr(void *arg);
/* Supported ACPI IDs */
static char *sst_ids[] = {
	SST_ACPI_ID_BDW,
	SST_ACPI_ID_HSW,
	NULL
};

/* ================================================================
 * Interrupt Handler
 * ================================================================ */

static void
sst_intr(void *arg)
{
	struct sst_softc *sc = arg;
	uint32_t isr;

	/*
	 * Check ISRX (ISC in catpt terms) directly - NOT PISR.
	 * Linux catpt checks ISC register (offset 0x18) for:
	 *   bit 0 (IPCCD) = reply to our command (DONE in IPCX)
	 *   bit 1 (IPCDB) = notification from DSP (BUSY in IPCD)
	 * PISR may not reflect all IPC events on WPT/Broadwell.
	 */
	isr = sst_shim_read(sc, SST_SHIM_ISRX);
	if (isr == 0 || isr == SST_INVALID_REG_VALUE)
		return;

	if (isr & (SST_IMC_IPCCD | SST_IMC_IPCDB))
		sst_ipc_intr(sc);

	sst_dma_intr(sc);
}

/* ================================================================
 * DSP Reset and Init
 * ================================================================ */

/*
 * sst_dsp_stall - Stall or unstall the DSP core
 * Based on Linux catpt catpt_dsp_stall()
 * STALL is at BIT(10) in CSR, NOT BIT(0)!
 */
int
sst_dsp_stall(struct sst_softc *sc, bool stall)
{
	uint32_t csr, val, timeout;

	val = stall ? SST_CSR_STALL : 0;
	sst_shim_update_bits(sc, SST_SHIM_CSR, SST_CSR_STALL, val);

	/* Poll for stall state to take effect */
	for (timeout = 0; timeout < 20; timeout++) {
		csr = sst_shim_read(sc, SST_SHIM_CSR);
		if ((csr & SST_CSR_STALL) == val)
			return (0);
		DELAY(500);
	}

	device_printf(sc->dev, "DSP %s timeout: CSR=0x%08x\n",
	    stall ? "stall" : "unstall", csr);
	return (ETIMEDOUT);
}

/*
 * sst_dsp_reset - Assert or deassert DSP reset
 * Based on Linux catpt catpt_dsp_reset()
 */
int
sst_dsp_reset(struct sst_softc *sc, bool reset)
{
	uint32_t csr, val, timeout;

	val = reset ? SST_CSR_RST : 0;
	sst_shim_update_bits(sc, SST_SHIM_CSR, SST_CSR_RST, val);

	/* Poll for reset state to take effect */
	for (timeout = 0; timeout < 20; timeout++) {
		csr = sst_shim_read(sc, SST_SHIM_CSR);
		if ((csr & SST_CSR_RST) == val)
			return (0);
		DELAY(500);
	}

	device_printf(sc->dev, "DSP %s timeout: CSR=0x%08x\n",
	    reset ? "reset" : "unreset", csr);
	return (ETIMEDOUT);
}

/*
 * sst_dsp_set_regs_defaults - Reset SHIM registers to hardware defaults
 * Based on Linux catpt catpt_dsp_set_regs_defaults()
 *
 * Hardware won't reset these registers itself, so we must do it
 * after each power cycle.
 */
void
sst_dsp_set_regs_defaults(struct sst_softc *sc)
{
	int i;

	sst_dbg(sc, SST_DBG_OPS, "  Setting SHIM register defaults...\n");

	/* SHIM registers */
	sst_shim_write(sc, SST_SHIM_CSR, SST_CS_DEFAULT);
	sst_shim_write(sc, SST_SHIM_ISRX, SST_ISC_DEFAULT);
	sst_shim_write(sc, SST_SHIM_ISRD, SST_ISD_DEFAULT);
	sst_shim_write(sc, SST_SHIM_IMRX, SST_IMC_DEFAULT);
	sst_shim_write(sc, SST_SHIM_IMRD, SST_IMD_DEFAULT);
	sst_shim_write(sc, SST_SHIM_IPCX, SST_IPCC_DEFAULT);
	sst_shim_write(sc, SST_SHIM_IPCD, SST_IPCD_DEFAULT);
	sst_shim_write(sc, SST_SHIM_CLKCTL, SST_CLKCTL_DEFAULT);
	sst_shim_write(sc, SST_SHIM_CSR2, SST_CS2_DEFAULT);
	sst_shim_write(sc, SST_SHIM_LTRC, SST_LTRC_DEFAULT);
	sst_shim_write(sc, SST_SHIM_HMDC, SST_HMDC_DEFAULT);

	/* SSP registers for both ports */
	for (i = 0; i < 2; i++) {
		uint32_t ssp_base = (i == 0) ? SST_SSP0_OFFSET : SST_SSP1_OFFSET;

		bus_write_4(sc->mem_res, ssp_base + 0x00, SST_SSC0_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x04, SST_SSC1_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x08, SST_SSS_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x0C, SST_SSIT_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x10, SST_SSD_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x28, SST_SSTO_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x2C, SST_SSPSP_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x30, SST_SSTSA_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x34, SST_SSRSA_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x38, SST_SSTSS_DEFAULT);
		bus_write_4(sc->mem_res, ssp_base + 0x40, SST_SSCR2_DEFAULT);
	}

	sst_dbg(sc, SST_DBG_OPS, "  SHIM defaults set (CSR=0x%08x)\n",
	    sst_shim_read(sc, SST_SHIM_CSR));
}

/* ================================================================
 * ACPI Probe
 * ================================================================ */

static int
sst_acpi_probe(device_t dev)
{
	int rv;

	if (acpi_disabled("sst"))
		return (ENXIO);

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, sst_ids, NULL);

	if (rv <= 0)
		device_set_desc(dev, "Intel Broadwell-U Audio DSP (SST)");

	return (rv);
}

static int
sst_pci_probe(device_t dev)
{
	if (pci_get_vendor(dev) == PCI_VENDOR_INTEL &&
	    pci_get_device(dev) == PCI_DEVICE_SST_BDW) {
		device_set_desc(dev, "Intel Broadwell-U Audio DSP (PCI Mode)");
		return (BUS_PROBE_DEFAULT);
	}
	return (ENXIO);
}

/* ================================================================
 * ACPI Attach - Main Driver Entry Point
 * ================================================================ */

static int
sst_acpi_attach(device_t dev)
{
	struct sst_softc *sc;
	uint32_t csr;
	int error = 0;
	bool bar0_ok;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);
	sc->mem_res = NULL;
	sc->shim_res = NULL;
	sc->irq_res = NULL;
	sc->irq_cookie = NULL;
	sc->attached = false;
	sc->state = SST_STATE_NONE;
	sc->debug_level = SST_DBG_LIFE;

	/* Allow boot-time override via device.hints */
	resource_int_value(device_get_name(dev), device_get_unit(dev),
	    "debug", &sc->debug_level);

	mtx_init(&sc->sc_mtx, "sst_sc", NULL, MTX_DEF);

	device_printf(dev, "Intel SST Driver v%s loading\n", SST_DRV_VERSION);

	/* ---- Phase 0: PCI BAR Fixup (DISABLED for PCI Mode) ----
	 * The ACPI driver should NOT interfere with BAR addresses if we want
	 * the kernel PCI subsystem to handle resource allocation naturally.
	 * The ACPI-hardcoded 0xFE000000 often conflicts or is simply wrong
	 * compared to what the OS allocates (e.g. 0xDF800000).
	 */
#if 0
	{
		uint32_t pci_bar0, pci_bar1, pci_cmd;

		pci_bar0 = pci_cfgregread(0, 0, 0x13, 0, 0x10, 4);
		pci_bar1 = pci_cfgregread(0, 0, 0x13, 0, 0x14, 4);
		pci_cmd  = pci_cfgregread(0, 0, 0x13, 0, 0x04, 4);

		device_printf(dev, "PCI BAR fixup: BAR0=0x%08x BAR1=0x%08x CMD=0x%04x\n",
		    pci_bar0, pci_bar1, pci_cmd & 0xFFFF);

		/* Expected ACPI addresses */
		if (pci_bar0 != 0xFE000000 || pci_bar1 != 0xFE100000) {
			device_printf(dev,
			    "  BARs relocated! Restoring ACPI defaults...\n");

			/* Disable memory decode while reprogramming */
			pci_cfgregwrite(0, 0, 0x13, 0, 0x04,
			    (pci_cmd & 0xFFFF) & ~0x06, 2);
			DELAY(1000);

			pci_cfgregwrite(0, 0, 0x13, 0, 0x10, 0xFE000000, 4);
			pci_cfgregwrite(0, 0, 0x13, 0, 0x14, 0xFE100000, 4);
			DELAY(1000);

			/* Re-enable memory + bus master */
			pci_cfgregwrite(0, 0, 0x13, 0, 0x04,
			    (pci_cmd & 0xFFFF) | 0x06, 2);
			DELAY(10000);

			pci_bar0 = pci_cfgregread(0, 0, 0x13, 0, 0x10, 4);
			pci_bar1 = pci_cfgregread(0, 0, 0x13, 0, 0x14, 4);
			device_printf(dev,
			    "  After fixup: BAR0=0x%08x BAR1=0x%08x\n",
			    pci_bar0, pci_bar1);
		}
	}
#endif

	/* ---- Phase 1: ACPI Power-Up ---- */
	sst_acpi_power_up(sc);

	/* ---- Phase 2: Allocate BAR1 (PCI config mirror) ---- */
	sc->shim_rid = 1;
	sc->shim_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->shim_rid, RF_ACTIVE);
	if (sc->shim_res == NULL) {
		device_printf(dev, "Failed to allocate BAR1 resource\n");
		error = ENXIO;
		goto fail;
	}
	sst_dbg(sc, SST_DBG_LIFE, "BAR1: 0x%lx (size: 0x%lx)\n",
	    rman_get_start(sc->shim_res), rman_get_size(sc->shim_res));

	/* Dump PCI config via BAR1 */
	sst_dump_pci_config(sc);

	/* Force enable Memory Space + Bus Master via BAR1 */
	{
		uint16_t cmd16 = bus_read_2(sc->shim_res, 0x04);
		if ((cmd16 & 0x06) != 0x06) {
			cmd16 |= 0x06;
			bus_write_2(sc->shim_res, 0x04, cmd16);
			DELAY(10000);
		}
	}

	/* ---- Phase 2.5: IOBP Configuration (BEFORE power-up!) ---- */
	/*
	 * Coreboot programs these IOBP registers during ADSP init.
	 * PMCTL=0x3f may be required for the power domain to work.
	 * Must be done before the power-up sequence.
	 */
	sst_iobp_probe(sc);

	/* ---- Phase 2.7: Enable HDA (shared clock domain) ---- */
	/*
	 * The ADSP and HDA share the audio clock domain in the PCH.
	 * If HDA is disabled (HDAD=1 in FD register), the clock source
	 * is gated and CPA will never assert during power-up.
	 * Enable HDA before attempting DSP power-up.
	 */
	(void)sst_try_enable_hda(sc);

	/* ---- Phase 3: WPT Power-Up Sequence ---- */
	sst_wpt_power_up(sc);

	/* ---- Phase 4: Allocate and Test BAR0 ---- */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate BAR0 resource\n");
		error = ENXIO;
		goto fail;
	}
	sst_dbg(sc, SST_DBG_LIFE, "BAR0: 0x%lx (size: 0x%lx)\n",
	    rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* Immediate SRAM write test - before any SHIM manipulation */
	{
		uint32_t t0, t1, t2, t3;

		t0 = bus_read_4(sc->mem_res, SST_DRAM_OFFSET);
		t1 = bus_read_4(sc->mem_res, SST_IRAM_OFFSET);
		sst_dbg(sc, SST_DBG_TRACE,
		    "SRAM pre-test: DRAM[0]=0x%08x IRAM[0]=0x%08x\n",
		    t0, t1);

		bus_write_4(sc->mem_res, SST_DRAM_OFFSET, 0xCAFEBABE);
		bus_barrier(sc->mem_res, SST_DRAM_OFFSET, 4,
		    BUS_SPACE_BARRIER_WRITE);
		t2 = bus_read_4(sc->mem_res, SST_DRAM_OFFSET);

		bus_write_4(sc->mem_res, SST_IRAM_OFFSET, 0xDEADBEEF);
		bus_barrier(sc->mem_res, SST_IRAM_OFFSET, 4,
		    BUS_SPACE_BARRIER_WRITE);
		t3 = bus_read_4(sc->mem_res, SST_IRAM_OFFSET);

		sst_dbg(sc, SST_DBG_TRACE,
		    "SRAM write test: DRAM[0]=0x%08x (want 0xCAFEBABE) "
		    "IRAM[0]=0x%08x (want 0xDEADBEEF)\n", t2, t3);

		/* Restore */
		bus_write_4(sc->mem_res, SST_DRAM_OFFSET, t0);
		bus_write_4(sc->mem_res, SST_IRAM_OFFSET, t1);
	}

	/* Enable SRAM power via control register */
	error = sst_enable_sram(sc);
	if (error != 0) {
		sst_dbg(sc, SST_DBG_LIFE, "SRAM enable returned %d, continuing anyway\n", error);
	}

	/*
	 * SRAM Sanitize: dummy reads from all SRAM blocks.
	 * Linux catpt_dsp_set_srampge() does this immediately after ungating.
	 * We do it here because BAR0 wasn't available during sst_wpt_power_up().
	 * This prevents "byte loss" on first write to newly-powered SRAM blocks.
	 */
	sst_sram_sanitize(sc);

	/* Test BAR0 */
	bar0_ok = sst_test_bar0(sc);
	sst_dbg(sc, SST_DBG_LIFE, "BAR0 test: %s\n",
	    bar0_ok ? "ACCESSIBLE" : "DEAD (0xFFFFFFFF)");

	if (bar0_ok) {
		/* Scan all BAR0 regions */
		sst_scan_bar0(sc);

		/* Probe DSP memory regions (WPT offsets) */
		sst_dbg(sc, SST_DBG_LIFE, "DSP Memory Layout (WPT):\n");
		sst_dbg(sc, SST_DBG_LIFE, "  DRAM  (0x%06x): 0x%08x\n",
		    SST_DRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_DRAM_OFFSET));
		sst_dbg(sc, SST_DBG_LIFE, "  IRAM  (0x%06x): 0x%08x\n",
		    SST_IRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_IRAM_OFFSET));
		sst_dbg(sc, SST_DBG_LIFE, "  SHIM  (0x%06x): 0x%08x\n",
		    SST_SHIM_OFFSET,
		    bus_read_4(sc->mem_res, SST_SHIM_OFFSET));

		csr = sst_shim_read(sc, SST_SHIM_CSR);
		sst_dbg(sc, SST_DBG_LIFE, "  SHIM CSR: 0x%08x\n", csr);

		/* Continue with full DSP initialization... */
		goto dsp_init;
	}

	/* ---- Phase 5: BAR0 Failed - Diagnose ---- */
	sst_dbg(sc, SST_DBG_LIFE, "=== BAR0 inaccessible - diagnosing ===\n");

	/* Dump PCH state for diagnostics */
	sst_dump_pch_state(sc);

	/* Read GNVS (BIOS NVS Area) variables to understand LPSS config.
	 * The LPSS fabric is configured by DSDT based on OSYS value.
	 * If OSYS < 0x07DC (Windows 2012), the DSDT puts LPSS devices
	 * to D3 sleep and doesn't initialize the memory fabric. */
	{
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t gnvs_handle;
		int map_err;

		sst_dbg(sc, SST_DBG_TRACE, "=== BIOS NVS Variables ===\n");

		map_err = bus_space_map(mt, PCH_GNVS_BASE, PCH_GNVS_SIZE,
		    0, &gnvs_handle);
		if (map_err == 0) {
			uint16_t osys_val;
			uint8_t s0id_val, ancs_val;

			/* OSYS at offset 0x00 (16-bit) */
			osys_val = bus_space_read_2(mt, gnvs_handle, 0x00);
			sst_dbg(sc, SST_DBG_TRACE, "  OSYS: 0x%04x (%s)\n",
			    osys_val,
			    osys_val >= 0x07DC ? "Windows 8+" :
			    osys_val >= 0x07D9 ? "Windows 7" :
			    "Pre-Win7");

			/* Read all bytes at known approximate offsets
			 * and dump relevant range for S0ID, ANCS, SMD0 */
			{
				int i;
				sst_dbg(sc, SST_DBG_TRACE,
				    "  GNVS raw [0x1C0-0x1D0]:");
				for (i = 0x1C0; i < 0x1D0; i++) {
					if ((i & 0xF) == 0)
						sst_dbg(sc, SST_DBG_TRACE,
						    "\n    [0x%03x]:", i);
					printf(" %02x",
					    bus_space_read_1(mt,
					    gnvs_handle, i));
				}
				printf("\n");
			}

			/* Try to read S0ID, ANCS by evaluating ACPI
			 * variables directly */
			{
				ACPI_STATUS ast;
				UINT32 aval;

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\S0ID", &aval);
				if (ACPI_SUCCESS(ast)) {
					s0id_val = (uint8_t)aval;
					sst_dbg(sc, SST_DBG_TRACE,
					    "  S0ID: %d (%s)\n",
					    s0id_val,
					    s0id_val ?
					    "Connected Standby" :
					    "No CS");
				} else {
					sst_dbg(sc, SST_DBG_TRACE,
					    "  S0ID: (cannot read)\n");
					s0id_val = 0;
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\ANCS", &aval);
				if (ACPI_SUCCESS(ast)) {
					ancs_val = (uint8_t)aval;
					sst_dbg(sc, SST_DBG_TRACE,
					    "  ANCS: %d (%s)\n",
					    ancs_val,
					    ancs_val ?
					    "Audio Not CS" : "Normal");
				} else {
					sst_dbg(sc, SST_DBG_TRACE,
					    "  ANCS: (cannot read)\n");
					ancs_val = 0;
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\SMD0", &aval);
				if (ACPI_SUCCESS(ast)) {
					sst_dbg(sc, SST_DBG_TRACE,
					    "  SMD0: %d (%s)\n",
					    (int)aval,
					    aval == 0 ? "Disabled" :
					    aval == 1 ? "PCI mode" :
					    aval == 2 ? "ACPI mode" :
					    "Unknown");
				} else {
					sst_dbg(sc, SST_DBG_TRACE,
					    "  SMD0: (cannot read)\n");
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\SB10", &aval);
				if (ACPI_SUCCESS(ast))
					sst_dbg(sc, SST_DBG_TRACE,
					    "  SB10: 0x%08x (SDMA BAR)\n",
					    aval);

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\ADB0", &aval);
				if (ACPI_SUCCESS(ast))
					sst_dbg(sc, SST_DBG_TRACE,
					    "  ADB0: 0x%08x (ADSP BAR0)\n",
					    aval);
			}

			bus_space_unmap(mt, gnvs_handle, PCH_GNVS_SIZE);

			/* Provide clear diagnosis */
			if (osys_val < 0x07DC) {
				sst_dbg(sc, SST_DBG_TRACE, "\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "*** ROOT CAUSE: OSYS=0x%04x < 0x07DC ***\n",
				    osys_val);
				sst_dbg(sc, SST_DBG_TRACE,
				    "FreeBSD reports Windows 7 to ACPI.\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "The DSDT requires OSYS >= 0x07DC (Win8+) to\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "initialize the LPSS fabric and enable BAR0.\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "\nFIX: Add to /boot/loader.conf:\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "  hw.acpi.install_interface=\"Windows 2012\"\n");
				sst_dbg(sc, SST_DBG_TRACE,
				    "Then reboot.\n");
			}
		} else {
			device_printf(dev,
			    "  Cannot map GNVS at 0x%08x: %d\n",
			    PCH_GNVS_BASE, map_err);
		}
	}

	/* Restore FD to BIOS default (0x00368011) in case previous
	 * module loads corrupted it with wrong bit offsets */
	{
		uint32_t rcba_reg, rcba_base, fd;
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t rh;

		rcba_reg = pci_cfgregread(0, PCH_LPC_BUS, PCH_LPC_DEV,
		    PCH_LPC_FUNC, PCH_LPC_RCBA_REG, 4);
		rcba_base = rcba_reg & PCH_RCBA_MASK;

		if (rcba_base != 0 && (rcba_reg & PCH_RCBA_ENABLE) &&
		    bus_space_map(mt, rcba_base, PCH_RCBA_SIZE, 0, &rh) == 0) {
			fd = bus_space_read_4(mt, rh, PCH_RCBA_FD);
			sst_dbg(sc, SST_DBG_TRACE, "=== FD Register ===\n");
			sst_dbg(sc, SST_DBG_TRACE, "  FD current: 0x%08x\n", fd);
			sst_dbg(sc, SST_DBG_TRACE,
			    "    ADSD (bit 1): %s\n",
			    (fd & PCH_FD_ADSD) ? "ADSP Disabled" :
			    "ADSP Enabled");
			sst_dbg(sc, SST_DBG_TRACE,
			    "    SMBD (bit 3): %s\n",
			    (fd & PCH_FD_SMBD) ? "SMBus Disabled" :
			    "SMBus Enabled");
			sst_dbg(sc, SST_DBG_TRACE,
			    "    HDAD (bit 4): %s\n",
			    (fd & PCH_FD_HDAD) ? "HDA Disabled" :
			    "HDA Enabled");

			/* Restore BIOS default if corrupted by previous
			 * wrong-bit-offset writes */
			if (fd != 0x00368011) {
				sst_dbg(sc, SST_DBG_TRACE,
				    "  Restoring FD to BIOS default 0x00368011\n");
				bus_space_write_4(mt, rh, PCH_RCBA_FD,
				    0x00368011);
				DELAY(10000);
				fd = bus_space_read_4(mt, rh, PCH_RCBA_FD);
				sst_dbg(sc, SST_DBG_TRACE,
				    "  FD after restore: 0x%08x\n", fd);
			}

			bus_space_unmap(mt, rh, PCH_RCBA_SIZE);
		}
	}

	/* Dump full LPSS private config space (BAR1)
	 * Standard PCI: 0x00-0xFF
	 * LPSS private: 0x100-0x1FF
	 * LPSS extended: 0x200-0xFFF (PMCTRL, LTR, clocks, remap)
	 * Key Linux intel-lpss registers:
	 *   0x800: LTR value
	 *   0x808: LPSS general
	 *   0x810: REMAP_ADDR (BAR0 remap!)
	 *   0x900: CLOCK_PARAMS */
	{
		int i;
		uint32_t v;

		sst_dbg(sc, SST_DBG_TRACE, "=== LPSS Private Config (BAR1) ===\n");
		/* Dump interesting non-zero regions */
		for (i = 0x00; i < 0x1000; i += 4) {
			v = bus_read_4(sc->shim_res, i);
			/* Print if non-zero or at known-interesting offset */
			if (v != 0 || i == 0x10 || i == 0x84 ||
			    i == 0xA0 || i == 0xA8 || i == 0x200 ||
			    i == 0x800 || i == 0x808 || i == 0x810 ||
			    i == 0x900)
				sst_dbg(sc, SST_DBG_TRACE,
				    "  [0x%03x]: 0x%08x\n", i, v);
		}
	}

	/* Try LPSS devices private config - read from SDMA and I2C
	 * to understand their power state */
	{
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t h;
		int map_err;
		struct {
			const char *name;
			uint32_t addr;
		} lpss_devs[] = {
			{ "SDMA",  0xFE102000 },
			{ "I2C0c", 0xFE104000 },
			{ "I2C1c", 0xFE106000 },
		};
		int d;

		sst_dbg(sc, SST_DBG_TRACE,
		    "=== LPSS Device Private Configs ===\n");
		for (d = 0; d < 3; d++) {
			map_err = bus_space_map(mt, lpss_devs[d].addr,
			    0x300, 0, &h);
			if (map_err != 0) {
				sst_dbg(sc, SST_DBG_TRACE, "  %s: map failed\n",
				    lpss_devs[d].name);
				continue;
			}

			sst_dbg(sc, SST_DBG_TRACE,
			    "  %s (0x%08x):\n", lpss_devs[d].name,
			    lpss_devs[d].addr);
			sst_dbg(sc, SST_DBG_TRACE,
			    "    VID/DID: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x00));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    CMD/STS: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x04));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    BAR0:    0x%08x\n",
			    bus_space_read_4(mt, h, 0x10));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    PMCSR:   0x%08x\n",
			    bus_space_read_4(mt, h, 0x84));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    [0x200]: 0x%08x (PMCTRL)\n",
			    bus_space_read_4(mt, h, 0x200));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    [0x204]: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x204));
			sst_dbg(sc, SST_DBG_TRACE,
			    "    [0x208]: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x208));

			bus_space_unmap(mt, h, 0x300);
		}
	}

	/* Try various LPSS BAR0 recovery strategies */
	{
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t h;
		int map_err;
		uint32_t v;

		sst_dbg(sc, SST_DBG_TRACE,
		    "=== BAR0 Memory Decode Test ===\n");

		/* Test 1: Is SDMA BAR0 (0xFE101000) accessible? */
		map_err = bus_space_map(mt, 0xFE101000, 0x1000, 0, &h);
		if (map_err == 0) {
			v = bus_space_read_4(mt, h, 0);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  SDMA BAR0 (0xFE101000)[0]: 0x%08x %s\n",
			    v, v != 0xFFFFFFFF ? "ALIVE" : "DEAD");
			bus_space_unmap(mt, h, 0x1000);
		}

		/* Test 2: Probe the 0xFE100000-0xFE10F000 range
		 * to find what's decoded */
		sst_dbg(sc, SST_DBG_TRACE,
		    "  LPSS address space probe (4KB blocks):\n");
		{
			int blk;
			for (blk = 0; blk < 16; blk++) {
				uint32_t addr = 0xFE100000 + blk * 0x1000;
				map_err = bus_space_map(mt, addr,
				    0x10, 0, &h);
				if (map_err == 0) {
					uint32_t v0, v4;
					v0 = bus_space_read_4(mt, h, 0);
					v4 = bus_space_read_4(mt, h, 4);
					sst_dbg(sc, SST_DBG_TRACE,
					    "    0x%08x: [0]=0x%08x "
					    "[4]=0x%08x%s\n",
					    addr, v0, v4,
					    v0 == 0xFFFFFFFF ?
					    " DEAD" : "");
					bus_space_unmap(mt, h, 0x10);
				}
			}
		}

		/* Test 3: Try changing ADSP BAR0 in private config
		 * to an address within the LPSS-decoded range */
		sst_dbg(sc, SST_DBG_TRACE,
		    "=== BAR0 Remap Experiment ===\n");
		{
			uint32_t old_bar0, new_bar0;

			old_bar0 = bus_read_4(sc->shim_res, 0x10);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Current BAR0 in privconfig: 0x%08x\n",
			    old_bar0);

			/* Try remapping to unused LPSS space */
			new_bar0 = 0xFE108000;
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Writing BAR0 = 0x%08x...\n", new_bar0);
			bus_write_4(sc->shim_res, 0x10, new_bar0);
			DELAY(10000);

			v = bus_read_4(sc->shim_res, 0x10);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  BAR0 readback: 0x%08x\n", v);

			if (v == new_bar0) {
				/* Try reading the new address */
				map_err = bus_space_map(mt, new_bar0,
				    0x1000, 0, &h);
				if (map_err == 0) {
					uint32_t dsp_v;
					dsp_v = bus_space_read_4(mt, h, 0);
					sst_dbg(sc, SST_DBG_TRACE,
					    "  New BAR0[0]: 0x%08x%s\n",
					    dsp_v,
					    dsp_v != 0xFFFFFFFF ?
					    " *** ALIVE! ***" : " dead");

					if (dsp_v != 0xFFFFFFFF) {
						sst_dbg(sc, SST_DBG_TRACE,
						    "  [0x00]: 0x%08x\n",
						    dsp_v);
						sst_dbg(sc, SST_DBG_TRACE,
						    "  [0x04]: 0x%08x\n",
						    bus_space_read_4(mt,
						    h, 0x04));
						sst_dbg(sc, SST_DBG_TRACE,
						    "  [0x10]: 0x%08x\n",
						    bus_space_read_4(mt,
						    h, 0x10));
					}
					bus_space_unmap(mt, h, 0x1000);
				}
			}

			/* Restore original BAR0 */
			bus_write_4(sc->shim_res, 0x10, old_bar0);
			DELAY(10000);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Restored BAR0: 0x%08x\n",
			    bus_read_4(sc->shim_res, 0x10));
		}

		/* Test 4: Try I2C0 D0 transition and BAR0 check */
		sst_dbg(sc, SST_DBG_TRACE,
		    "=== I2C0 D0 Transition Test ===\n");
		map_err = bus_space_map(mt, 0xFE104000, 0x300, 0, &h);
		if (map_err == 0) {
			uint32_t pmcsr;

			pmcsr = bus_space_read_4(mt, h, 0x84);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  I2C0 PMCSR: 0x%08x (D%d)\n",
			    pmcsr, pmcsr & 3);

			/* Force D0 */
			if ((pmcsr & 3) != 0) {
				bus_space_write_4(mt, h, 0x84,
				    pmcsr & ~3);
				DELAY(50000);
				pmcsr = bus_space_read_4(mt, h, 0x84);
				sst_dbg(sc, SST_DBG_TRACE,
				    "  I2C0 PMCSR after D0: 0x%08x\n",
				    pmcsr);
			}
			bus_space_unmap(mt, h, 0x300);

			/* Check I2C0 BAR0 */
			map_err = bus_space_map(mt, 0xFE103000,
			    0x100, 0, &h);
			if (map_err == 0) {
				v = bus_space_read_4(mt, h, 0xFC);
				sst_dbg(sc, SST_DBG_TRACE,
				    "  I2C0 BAR0 IC_COMP_TYPE: 0x%08x%s\n",
				    v,
				    v == 0x44570140 ?
				    " DesignWare!" :
				    v == 0xFFFFFFFF ?
				    " still dead" : " unknown");
				bus_space_unmap(mt, h, 0x100);
			}
		}

		/* Final BAR0 test */
		bar0_ok = sst_test_bar0(sc);
		sst_dbg(sc, SST_DBG_TRACE,
		    "BAR0 final: %s\n",
		    bar0_ok ? "*** ACCESSIBLE! ***" : "still dead");

		if (bar0_ok)
			goto dsp_init;
	}

	/* IOBP already called in Phase 2.5. If BAR0 is still dead,
	 * try re-doing power-up now that IOBP has been configured. */
	if (!bar0_ok) {
		sst_dbg(sc, SST_DBG_LIFE, "BAR0 dead after IOBP+power-up, trying power cycle again...\n");
		sst_wpt_power_up(sc);
		bar0_ok = sst_test_bar0(sc);
		sst_dbg(sc, SST_DBG_LIFE, "BAR0 after 2nd power-up: %s\n",
		    bar0_ok ? "*** ACCESSIBLE! ***" : "still dead");
	}

	/* ---- Phase 8: I2C0 Codec Probe ---- */
	/* Transition I2C0 to D0 first, then probe codec */
	{
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t h;
		int map_err;

		sst_dbg(sc, SST_DBG_TRACE,
		    "=== Phase 8: I2C0 Codec Probe ===\n");
		map_err = bus_space_map(mt, SST_I2C0_PRIV_BASE,
		    0x300, 0, &h);
		if (map_err == 0) {
			uint32_t pmcsr;

			pmcsr = bus_space_read_4(mt, h, 0x84);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  I2C0 PMCSR: 0x%08x (D%d)\n",
			    pmcsr, pmcsr & 3);

			/* Force D0 */
			if ((pmcsr & 3) != 0) {
				bus_space_write_4(mt, h, 0x84,
				    pmcsr & ~3);
				DELAY(50000);
				pmcsr = bus_space_read_4(mt, h, 0x84);
				sst_dbg(sc, SST_DBG_TRACE,
				    "  I2C0 PMCSR after D0: 0x%08x\n",
				    pmcsr);
			}
			bus_space_unmap(mt, h, 0x300);

			/* Now probe the codec on I2C0 */
			sst_probe_i2c_codec(sc);
		} else {
			device_printf(dev,
			    "  I2C0 private config map failed: %d\n",
			    map_err);
		}
	}

	/* ---- Phase 9: Summary ---- */
	{
		uint32_t hda_vid = pci_cfgregread(0, PCH_HDA_BUS,
		    PCH_HDA_DEV, PCH_HDA_FUNC, 0x00, 4);

		sst_dbg(sc, SST_DBG_LIFE, "\n=== SUMMARY ===\n");
		sst_dbg(sc, SST_DBG_LIFE, "BAR0 (DSP MMIO): %s\n",
		    sst_test_bar0(sc) ? "ACCESSIBLE" : "DEAD");
		sst_dbg(sc, SST_DBG_LIFE, "HDA controller: %s\n",
		    hda_vid != 0xFFFFFFFF ? "ENABLED" : "ABSENT");
		if (hda_vid != 0xFFFFFFFF) {
			sst_dbg(sc, SST_DBG_LIFE,
			    "  VID/DID: 0x%08x at PCI 0:%x.%d\n",
			    hda_vid, PCH_HDA_DEV, PCH_HDA_FUNC);
		}
		sst_dbg(sc, SST_DBG_LIFE,
		    "Audio path: requires reboot or MCHBAR fix.\n");
	}

	/* Stay attached for debugging */
	sc->attached = true;
	sc->state = SST_STATE_ATTACHED;
	return (0);

	/* ---- Full DSP Initialization Path ---- */
dsp_init:
	/*
	 * Initialize IPC subsystem BEFORE registering IRQ handler.
	 * The ISR calls sst_ipc_intr() which takes sc->ipc.lock.
	 * With shared IRQs, the handler can fire immediately after
	 * bus_setup_intr(), so the mutex must be initialized first.
	 */
	error = sst_ipc_init(sc);
	if (error) {
		device_printf(dev, "IPC init failed\n");
		goto fail;
	}

	/* Allocate Interrupt (after IPC init - ISR needs ipc.lock) */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res != NULL) {
		error = bus_setup_intr(dev, sc->irq_res,
		    INTR_TYPE_AV | INTR_MPSAFE,
		    NULL, sst_intr, sc, &sc->irq_cookie);
		if (error) {
			device_printf(dev, "Failed to setup IRQ: %d\n", error);
			sc->irq_cookie = NULL;
		}
	}

	error = sst_fw_init(sc);
	if (error) {
		device_printf(dev, "Firmware init failed\n");
		goto fail;
	}

	sst_init(sc);

	/*
	 * Configure SHIM for Broadwell-U (catpt) boot sequence.
	 *
	 * Based on Linux catpt catpt_dsp_power_up():
	 * 1. Set register defaults (with STALL+RST in CSR)
	 * 2. Restore MCLK
	 * 3. Select high clock (not LP clock)
	 * 4. Set 24MHz SSP bank clocks via SHIM CSR SBCS bits
	 * 5. Clear RST (leave STALL set)
	 * 6. Enable DCLCGE
	 * 7. Clear IPC interrupt deassert (unmask IPC interrupts in IMC)
	 */
	{
		uint32_t csr;

		/* Step 1: Set SHIM register defaults (puts DSP in STALL+RST) */
		sst_dsp_set_regs_defaults(sc);

		/* Step 2: Restore MCLK via CLKCTL SMOS bits */
		sst_shim_update_bits(sc, SST_SHIM_CLKCTL,
		    SST_CLKCTL_SMOS_MASK, SST_CLKCTL_SMOS_MASK);

		/* Step 3: Select high clock (clear LPCS in CSR) */
		sst_shim_update_bits(sc, SST_SHIM_CSR, SST_CSR_LPCS, 0);

		/* Step 4: Set 24MHz SSP bank clocks (SBCS0 + SBCS1) */
		sst_shim_update_bits(sc, SST_SHIM_CSR,
		    SST_CSR_SBCS0 | SST_CSR_SBCS1,
		    SST_CSR_SBCS0 | SST_CSR_SBCS1);

		/* Step 5: Clear RST (leave STALL set - DSP stalled but not reset) */
		sst_dsp_reset(sc, false);

		csr = sst_shim_read(sc, SST_SHIM_CSR);
		sst_dbg(sc, SST_DBG_OPS, "  After de-assert RST: CSR=0x%08x "
		    "(STALL=%d RST=%d)\n",
		    csr, !!(csr & SST_CSR_STALL), !!(csr & SST_CSR_RST));

		/*
		 * Step 6: Do NOT re-enable DCLCGE yet!
		 * DCLCGE blocks MMIO writes to SRAM. Since we load firmware
		 * via MMIO (not DMA like Linux catpt), DCLCGE must stay
		 * disabled until after firmware is loaded.
		 * Re-enabled in sst_fw_boot() after successful load.
		 */

		/* Step 7: Clear IPC interrupt deassert (unmask IPC in IMC) */
		sst_shim_update_bits(sc, SST_SHIM_IMRX,
		    SST_IMC_IPCDB | SST_IMC_IPCCD, 0);

		sst_dbg(sc, SST_DBG_OPS, "SHIM configured (catpt boot sequence)\n");
	}

	error = sst_dma_init(sc);
	if (error) {
		device_printf(dev, "DMA init failed\n");
		goto fail;
	}

	error = sst_ssp_init(sc);
	if (error) {
		device_printf(dev, "SSP init failed\n");
		goto fail;
	}

	error = sst_pcm_init(sc);
	if (error) {
		device_printf(dev, "PCM init failed\n");
		goto fail;
	}

	error = sst_topology_init(sc);
	if (error) {
		device_printf(dev, "Topology init failed\n");
		goto fail;
	}

	/*
	 * Load firmware with DSP stalled but NOT in reset.
	 * Linux catpt_boot_firmware(): stall only, no RST assertion.
	 * RST was already cleared in SHIM config step 5.
	 * DCLCGE must stay disabled for MMIO writes (Linux uses DMA instead).
	 */
	sst_dsp_stall(sc, true);   /* Ensure STALL is set */
	{
		uint32_t pre_csr = sst_shim_read(sc, SST_SHIM_CSR);
		sst_dbg(sc, SST_DBG_OPS, "Pre-FW load: CSR=0x%08x (STALL=%d RST=%d)\n",
		    pre_csr, !!(pre_csr & SST_CSR_STALL),
		    !!(pre_csr & SST_CSR_RST));
	}
	error = sst_fw_load(sc);
	if (error) {
		device_printf(dev, "Firmware load failed: %d\n", error);
		error = 0;
	} else {
		error = sst_fw_boot(sc);
		if (error) {
			device_printf(dev, "DSP boot failed: %d\n", error);
			error = 0;
		} else {
			sst_ipc_get_fw_version(sc, NULL);
			sst_fw_alloc_module_regions(sc);
			sst_ipc_probe_stage_caps(sc);
			sst_topology_load_default(sc);
		}
	}

	/*
	 * Configure SSP device format (once, before any stream).
	 * Linux catpt does this in catpt_dai_pcm_new() at card probe.
	 * Must happen BEFORE alloc_stream per catpt protocol.
	 */
	if (sc->fw.state == SST_FW_STATE_RUNNING) {
		struct sst_device_format devfmt;
		memset(&devfmt, 0, sizeof(devfmt));
		devfmt.iface = SST_SSP_IFACE_0;
		devfmt.mclk = SST_MCLK_FREQ_24_MHZ;
		devfmt.mode = SST_SSP_MODE_I2S_PROVIDER;
		devfmt.clock_divider = 9;
		devfmt.channels = 2;
		error = sst_ipc_set_device_formats(sc, &devfmt);
		if (error)
			device_printf(dev,
			    "SET_DEVICE_FORMATS failed: %d (non-fatal)\n",
			    error);
	}

	/* Register PCM device */
	if (sc->fw.state == SST_FW_STATE_RUNNING) {
		sst_pcm_register(sc);
	}

	/* Jack detection */
	error = sst_jack_init(sc);
	if (error == 0) {
		sst_jack_sysctl_init(sc);
		sst_jack_enable(sc);
	}
	error = 0;

	/* EQ preset sysctl */
	sst_topology_sysctl_init(sc);

	/* RT286 codec initialization and output enable */
	if (sst_codec_init(sc) == 0) {
		sst_codec_enable_speaker(sc);
		sst_codec_enable_headphone(sc);
	}

	sc->attached = true;
	sc->state = SST_STATE_ATTACHED;
	device_printf(dev, "Intel SST DSP attached successfully\n");
	return (0);

fail:
	sst_acpi_detach(dev);
	return (error);
}

/* ================================================================
 * PCI Attach - Standard PCI Driver Entry Point
 * Used when the device is visible on PCI bus (unhidden)
 * ================================================================ */

static int
sst_pci_attach(device_t dev)
{
	struct sst_softc *sc;

	int error = 0, bar0_ok = 0;

	device_printf(dev, "Intel SST Driver v%s (PCI Attach)\n", SST_DRV_VERSION);

	sc = device_get_softc(dev);

	sc->dev = dev;
	sc->handle = NULL; /* Not using ACPI handle directly in PCI mode */
	sc->mem_res = NULL;
	sc->shim_res = NULL;
	sc->irq_res = NULL;
	sc->irq_cookie = NULL;
	sc->attached = false;
	sc->state = SST_STATE_NONE;

	mtx_init(&sc->sc_mtx, "sst_sc", NULL, MTX_DEF);

	/* Enable PCI Memory Space + Bus Master */
	{
		uint16_t cmd;

		cmd = pci_read_config(dev, PCIR_COMMAND, 2);
		sst_dbg(sc, SST_DBG_OPS, "PCI Command register: 0x%04x\n", cmd);

		cmd |= (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);
		pci_write_config(dev, PCIR_COMMAND, cmd, 2);
		cmd = pci_read_config(dev, PCIR_COMMAND, 2);
		sst_dbg(sc, SST_DBG_OPS, "PCI Command after setup: 0x%04x\n", cmd);
	}

	/* Allocate BAR0 (DSP Memory) */
	sc->mem_rid = PCIR_BAR(0);
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate BAR0 resource via PCI\n");
		error = ENXIO;
		goto fail;
	}
	sst_dbg(sc, SST_DBG_LIFE, "BAR0: 0x%lx (size: 0x%lx)\n",
	    rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* Allocate BAR1 (PCI Config Mirror / Private) */
	sc->shim_rid = PCIR_BAR(1);
	sc->shim_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
	    &sc->shim_rid, RF_ACTIVE);
	if (sc->shim_res == NULL) {
		device_printf(dev, "Failed to allocate BAR1 resource via PCI\n");
		error = ENXIO;
		goto fail;
	}
	sst_dbg(sc, SST_DBG_LIFE, "BAR1: 0x%lx (size: 0x%lx)\n",
	    rman_get_start(sc->shim_res), rman_get_size(sc->shim_res));

	/* ---- WPT Power-Up (MUST happen before SRAM enable) ----
	 * This does D3 cycling + catpt power-up via PCI config space.
	 * Previous versions tried SRAM enable first, but the DSP clocks
	 * need to be running before SRAM can be powered on. */
	sst_wpt_power_up(sc);

	/* Re-verify PCI CMD survived the D3 cycle */
	{
		uint16_t cmd = pci_read_config(dev, PCIR_COMMAND, 2);
		if ((cmd & (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN)) !=
		    (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN)) {
			sst_dbg(sc, SST_DBG_LIFE,
			    "PCI CMD lost after D3 cycle (0x%04x), restoring\n",
			    cmd);
			cmd |= (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);
			pci_write_config(dev, PCIR_COMMAND, cmd, 2);
		}
	}

	/* Now try SRAM enable after DSP power-up */
	error = sst_enable_sram(sc);
	if (error != 0) {
		sst_dbg(sc, SST_DBG_LIFE, "SRAM enable returned %d, trying direct...\n",
		    error);
		sst_enable_sram_direct(dev);
	}

	/* SRAM Sanitize: dummy reads to prevent byte loss */
	sst_sram_sanitize(sc);

	/* Test BAR0 */
	bar0_ok = sst_test_bar0(sc);
	sst_dbg(sc, SST_DBG_LIFE, "BAR0 test: %s\n",
	    bar0_ok ? "ACCESSIBLE" : "DEAD (0xFFFFFFFF)");

	/* Allocate IRQ */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Failed to allocate IRQ\n");
		/* Don't fail yet, maybe we can run without IRQ for init test */
	} else {
		sst_dbg(sc, SST_DBG_LIFE, "IRQ assigned\n");
	}

	if (bar0_ok) {
		/* Probe DSP memory regions */
		sst_dbg(sc, SST_DBG_LIFE, "DSP Memory Layout (PCI):\n");
		sst_dbg(sc, SST_DBG_LIFE, "  IRAM  (0x%05x): 0x%08x\n",
		    SST_IRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_IRAM_OFFSET));
		sst_dbg(sc, SST_DBG_LIFE, "  DRAM  (0x%05x): 0x%08x\n",
		    SST_DRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_DRAM_OFFSET));
		sst_dbg(sc, SST_DBG_LIFE, "  SHIM  (0x%05x): 0x%08x\n",
		    SST_SHIM_OFFSET,
		    bus_read_4(sc->mem_res, SST_SHIM_OFFSET));

		/* Initialize IPC subsystem */
		error = sst_ipc_init(sc);
		if (error) {
			device_printf(dev, "IPC init failed\n");
			goto fail;
		}

		/* Initialize firmware subsystem */
		error = sst_fw_init(sc);
		if (error) {
			device_printf(dev, "Firmware init failed\n");
			goto fail;
		}

		/* Basic DSP init (mask interrupts, reset) */
		sst_init(sc);

		/*
		 * Configure SHIM for catpt boot (PCI attach path)
		 * Same sequence as ACPI attach.
		 */
		{
			uint32_t csr_val;

			sst_dsp_set_regs_defaults(sc);

			/* Restore MCLK */
			sst_shim_update_bits(sc, SST_SHIM_CLKCTL,
			    SST_CLKCTL_SMOS_MASK, SST_CLKCTL_SMOS_MASK);

			/* Select high clock */
			sst_shim_update_bits(sc, SST_SHIM_CSR,
			    SST_CSR_LPCS, 0);

			/* Set 24MHz SSP bank clocks */
			sst_shim_update_bits(sc, SST_SHIM_CSR,
			    SST_CSR_SBCS0 | SST_CSR_SBCS1,
			    SST_CSR_SBCS0 | SST_CSR_SBCS1);

			/* Clear RST */
			sst_dsp_reset(sc, false);

			/* Re-enable DCLCGE */
			{
				uint32_t vdrtctl2 = bus_read_4(sc->shim_res,
				    SST_PCI_VDRTCTL2);
				vdrtctl2 |= SST_VDRTCTL2_DCLCGE;
				bus_write_4(sc->shim_res,
				    SST_PCI_VDRTCTL2, vdrtctl2);
			}

			/* Clear IPC interrupt deassert */
			sst_shim_update_bits(sc, SST_SHIM_IMRX,
			    SST_IMC_IPCDB | SST_IMC_IPCCD, 0);

			csr_val = sst_shim_read(sc, SST_SHIM_CSR);
			sst_dbg(sc, SST_DBG_OPS,
			    "SHIM configured (catpt): CSR=0x%08x\n",
			    csr_val);
		}

		/* Initialize DMA subsystem */
		error = sst_dma_init(sc);
		if (error) {
			device_printf(dev, "DMA init failed\n");
			goto fail;
		}

		/* Initialize SSP (I2S) subsystem */
		error = sst_ssp_init(sc);
		if (error) {
			device_printf(dev, "SSP init failed\n");
			goto fail;
		}

		/* Initialize PCM subsystem */
		error = sst_pcm_init(sc);
		if (error) {
			device_printf(dev, "PCM init failed\n");
			goto fail;
		}

		/* Initialize topology (audio pipeline) */
		error = sst_topology_init(sc);
		if (error) {
			device_printf(dev, "Topology init failed\n");
			goto fail;
		}

		/* Load firmware */
		error = sst_fw_load(sc);
		if (error) {
			device_printf(dev, "Firmware load failed: %d\n", error);
			error = 0; /* Continue without firmware for debugging */
		} else {
			/* Boot DSP with loaded firmware */
			error = sst_fw_boot(sc);
			if (error) {
				device_printf(dev, "DSP boot failed: %d\n", error);
				error = 0; /* Continue for debugging */
			} else {
				/* Get firmware version */
				sst_ipc_get_fw_version(sc, NULL);
				sst_fw_alloc_module_regions(sc);
				sst_ipc_probe_stage_caps(sc);
				/* Load default audio topology */
				sst_topology_load_default(sc);
			}
		}

		/* Register PCM device if firmware is running */
		if (sc->fw.state == SST_FW_STATE_RUNNING) {
			sst_pcm_register(sc);
		}

		/* Jack detection */
		error = sst_jack_init(sc);
		if (error == 0) {
			sst_jack_sysctl_init(sc);
			sst_jack_enable(sc);
		}
		error = 0;

		/* EQ preset sysctl */
		sst_topology_sysctl_init(sc);

		/* RT286 codec initialization and output enable */
		if (sst_codec_init(sc) == 0) {
			sst_codec_enable_speaker(sc);
			sst_codec_enable_headphone(sc);
		}

		sc->attached = true;
		sc->state = SST_STATE_ATTACHED;
		device_printf(dev, "Intel SST DSP attached successfully\n");
	} else {
		sst_dbg(sc, SST_DBG_LIFE, "PCI Attach: BAR0 still dead. Hardware is tough.\n");
		/* Don't fail, keep attached to allow inspection */
		sc->attached = true;
	}

	return (0);

fail:
	if (sc->irq_res)
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
	if (sc->shim_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->shim_rid, sc->shim_res);
	if (sc->mem_res)
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
	mtx_destroy(&sc->sc_mtx);
	return (error);
}

static int
sst_pci_detach(device_t dev)
{
	struct sst_softc *sc = device_get_softc(dev);

	/* Cleanup subsystems in reverse order of initialization */
	sst_codec_fini(sc);
	sst_jack_fini(sc);
	sst_topology_fini(sc);
	sst_pcm_fini(sc);
	sst_ssp_fini(sc);
	sst_dma_fini(sc);
	sst_fw_fini(sc);
	sst_ipc_fini(sc);

	if (sc->irq_cookie) {
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
		sc->irq_cookie = NULL;
	}
	if (sc->irq_res) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
		sc->irq_res = NULL;
	}
	if (sc->shim_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->shim_rid, sc->shim_res);
		sc->shim_res = NULL;
	}
	if (sc->mem_res) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
		sc->mem_res = NULL;
	}

	if (mtx_initialized(&sc->sc_mtx))
		mtx_destroy(&sc->sc_mtx);

	return (0);
}

/* ================================================================
 * ACPI Detach
 * ================================================================ */

static int
sst_acpi_detach(device_t dev)
{
	struct sst_softc *sc;

	sc = device_get_softc(dev);

	sst_codec_fini(sc);
	sst_jack_fini(sc);
	sst_topology_fini(sc);
	sst_pcm_fini(sc);
	sst_ssp_fini(sc);
	sst_dma_fini(sc);
	sst_fw_fini(sc);
	sst_ipc_fini(sc);

	if (sc->irq_cookie != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
		sc->irq_cookie = NULL;
	}

	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid,
		    sc->irq_res);
		sc->irq_res = NULL;
	}

	if (sc->shim_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->shim_rid,
		    sc->shim_res);
		sc->shim_res = NULL;
	}

	if (sc->mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
		    sc->mem_res);
		sc->mem_res = NULL;
	}

	if (mtx_initialized(&sc->sc_mtx))
		mtx_destroy(&sc->sc_mtx);

	if (sc->handle != NULL)
		acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);

	return (0);
}

/* ================================================================
 * Suspend / Resume
 * ================================================================ */

static int
sst_acpi_suspend(device_t dev)
{
	struct sst_softc *sc = device_get_softc(dev);

	sst_dbg(sc, SST_DBG_LIFE, "Suspending...\n");

	/* 1. Stop jack detection polling */
	sst_jack_disable(sc);

	/* 2. Tear down active PCM streams */
	sst_pcm_suspend(sc);

	/* 3. Stop SSP ports */
	sst_ssp_stop(sc, 0);
	sst_ssp_stop(sc, 1);

	/* 4. Shutdown codec (mute, power down, release I2C) */
	sst_codec_fini(sc);

	/* 5. Clear topology state so it can be rebuilt on resume */
	sc->topology.loaded = false;
	sc->topology.pipeline_count = 0;
	sc->topology.widget_count = 0;
	sc->topology.route_count = 0;

	/* 6. Reset DSP and power off */
	sst_reset(sc);
	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);
	sc->state = SST_STATE_SUSPENDED;

	sst_dbg(sc, SST_DBG_LIFE, "Suspended\n");
	return (0);
}

static int
sst_acpi_resume(device_t dev)
{
	struct sst_softc *sc = device_get_softc(dev);
	int error;

	sst_dbg(sc, SST_DBG_LIFE, "Resuming...\n");

	/* 1. Power to D0 */
	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0);
	DELAY(10000);

	/* 2. Init SHIM (mask interrupts, reset DSP) */
	sst_init(sc);
	sc->ipc.ready = false;
	sc->ipc.state = SST_IPC_STATE_IDLE;

	/* 3. Re-write firmware to SRAM and boot DSP */
	error = sst_fw_reload(sc);
	if (error) {
		device_printf(dev, "Resume: firmware reload failed: %d\n",
		    error);
		sc->state = SST_STATE_ERROR;
		return (error);
	}

	error = sst_fw_boot(sc);
	if (error) {
		device_printf(dev, "Resume: DSP boot failed: %d\n", error);
		sc->state = SST_STATE_ERROR;
		return (error);
	}

	/* 4. Post-boot setup (same as initial attach) */
	sst_ipc_get_fw_version(sc, NULL);
	sst_fw_alloc_module_regions(sc);
	sst_ipc_probe_stage_caps(sc);

	/* 5. Rebuild audio pipeline */
	sst_topology_load_default(sc);

	/* 6. SET_DEVICE_FORMATS for SSP0 */
	{
		struct sst_device_format devfmt;

		memset(&devfmt, 0, sizeof(devfmt));
		devfmt.iface = SST_SSP_IFACE_0;
		devfmt.mclk = SST_MCLK_FREQ_24_MHZ;
		devfmt.mode = SST_SSP_MODE_I2S_PROVIDER;
		devfmt.clock_divider = 9;
		devfmt.channels = 2;
		error = sst_ipc_set_device_formats(sc, &devfmt);
		if (error)
			device_printf(dev,
			    "Resume: SET_DEVICE_FORMATS failed: %d"
			    " (non-fatal)\n", error);
	}

	/* 7. Re-init codec and enable outputs */
	if (sst_codec_init(sc) == 0) {
		sst_codec_enable_speaker(sc);
		sst_codec_enable_headphone(sc);
	}

	/* 8. Restore mixer state to topology widgets */
	sst_pcm_resume(sc);

	/* 9. Re-enable jack detection */
	sst_jack_enable(sc);

	/* 10. Mark resume ramp pending (anti-pop) */
	sc->pcm.resume_ramp = true;

	sc->state = SST_STATE_RUNNING;
	sst_dbg(sc, SST_DBG_LIFE, "Resumed\n");
	return (0);
}

/* ================================================================
 * Driver Registration
 * ================================================================ */


/* SRAM PGE bug fixed (was |= now &=~), re-enabling ACPI driver */
static device_method_t sst_methods[] = {
	DEVMETHOD(device_probe,		sst_acpi_probe),
	DEVMETHOD(device_attach,	sst_acpi_attach),
	DEVMETHOD(device_detach,	sst_acpi_detach),
	DEVMETHOD(device_suspend,	sst_acpi_suspend),
	DEVMETHOD(device_resume,	sst_acpi_resume),
	/* Bus interface for child PCM device */
	DEVMETHOD(bus_print_child,	bus_generic_print_child),
	DEVMETHOD_END
};

static driver_t sst_driver = {
	"acpi_intel_sst",
	sst_methods,
	sizeof(struct sst_softc),
};

static device_method_t sst_pci_methods[] = {
	/* PCI probe/attach */
	DEVMETHOD(device_probe,		sst_pci_probe),
	DEVMETHOD(device_attach,	sst_pci_attach),
	DEVMETHOD(device_detach,	sst_pci_detach),
	DEVMETHOD(device_suspend,	sst_acpi_suspend), /* Re-use */
	DEVMETHOD(device_resume,	sst_acpi_resume),
	DEVMETHOD_END
};

static driver_t sst_pci_driver = {
	"acpi_intel_sst", /* Same name to share code */
	sst_pci_methods,
	sizeof(struct sst_softc),
};

/*
 * Both ACPI and PCI drivers registered.
 * ACPI driver for INT3438 device (primary path).
 * PCI driver for 0x9CB6 if device is unhidden on PCI bus (fallback).
 */
DRIVER_MODULE(acpi_intel_sst, acpi, sst_driver, 0, 0);
DRIVER_MODULE(sst_pci, pci, sst_pci_driver, 0, 0);
MODULE_DEPEND(acpi_intel_sst, acpi, 1, 1, 1);
MODULE_DEPEND(acpi_intel_sst, pci, 1, 1, 1);
MODULE_DEPEND(acpi_intel_sst, firmware, 1, 1, 1);
MODULE_VERSION(acpi_intel_sst, SST_DRV_VERSION_NUM);

