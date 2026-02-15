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

#include <vm/vm.h>
#include <vm/pmap.h>

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
static void sst_reset(struct sst_softc *sc);
static void sst_init(struct sst_softc *sc);

/* New functions */
static void sst_acpi_power_up(struct sst_softc *sc);
static int sst_wpt_power_up(struct sst_softc *sc);
static void sst_sram_sanitize(struct sst_softc *sc);
static bool sst_test_bar0(struct sst_softc *sc);
static int sst_enable_sram(struct sst_softc *sc);
static int sst_enable_sram_direct(device_t dev);
static int sst_check_sram_immediate(const char *checkpoint);
static int sst_try_enable_hda(struct sst_softc *sc);
static int sst_try_enable_adsp(struct sst_softc *sc) __unused;
static void sst_probe_i2c_codec(struct sst_softc *sc);
static void sst_dump_pci_config(struct sst_softc *sc);
static void sst_dump_pch_state(struct sst_softc *sc);
static void sst_iobp_probe(struct sst_softc *sc);

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

	device_printf(sc->dev, "  Setting SHIM register defaults...\n");

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

	device_printf(sc->dev, "  SHIM defaults set (CSR=0x%08x)\n",
	    sst_shim_read(sc, SST_SHIM_CSR));
}

static void
sst_reset(struct sst_softc *sc)
{
	device_printf(sc->dev, "Resetting DSP...\n");
	sst_dsp_stall(sc, true);
	sst_dsp_reset(sc, true);
	DELAY(SST_RESET_DELAY_US);
}

static void
sst_init(struct sst_softc *sc)
{
	/* Mask all IPC interrupts initially */
	sst_shim_write(sc, SST_SHIM_IMRX, SST_IMC_DEFAULT);
	sst_shim_write(sc, SST_SHIM_IMRD, SST_IMD_DEFAULT);
	sst_reset(sc);
}

/* ================================================================
 * ACPI Power-Up Sequence
 * Calls all ACPI methods to transition device to D0
 * ================================================================ */

static void
sst_acpi_power_up(struct sst_softc *sc)
{
	ACPI_STATUS status;
	UINT32 sta;

	/* Check _STA */
	status = acpi_GetInteger(sc->handle, "_STA", &sta);
	if (ACPI_SUCCESS(status)) {
		device_printf(sc->dev, "ACPI _STA: 0x%x (%s%s%s%s)\n", sta,
		    (sta & 0x01) ? "Present " : "",
		    (sta & 0x02) ? "Enabled " : "",
		    (sta & 0x04) ? "Shown " : "",
		    (sta & 0x08) ? "Functional" : "");
	}

	/* _PS0 (D0 power state) */
	status = AcpiEvaluateObject(sc->handle, "_PS0", NULL, NULL);
	if (ACPI_SUCCESS(status)) {
		device_printf(sc->dev, "Called _PS0 successfully\n");
		DELAY(100000);
	}

	/* acpi_pwr_switch_consumer */
	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0);

	/* _DSM (Device Specific Method) */
	{
		ACPI_OBJECT_LIST args;
		ACPI_OBJECT arg[4];
		ACPI_BUFFER result = { ACPI_ALLOCATE_BUFFER, NULL };

		static uint8_t intel_dsm_uuid[] = {
			0x6e, 0x88, 0x9f, 0xa6, 0xeb, 0x6c, 0x94, 0x45,
			0xa4, 0x1f, 0x7b, 0x5d, 0xce, 0x24, 0xc5, 0x53
		};

		arg[0].Type = ACPI_TYPE_BUFFER;
		arg[0].Buffer.Length = 16;
		arg[0].Buffer.Pointer = intel_dsm_uuid;
		arg[1].Type = ACPI_TYPE_INTEGER;
		arg[1].Integer.Value = 1;
		arg[2].Type = ACPI_TYPE_INTEGER;
		arg[2].Integer.Value = 0;
		arg[3].Type = ACPI_TYPE_PACKAGE;
		arg[3].Package.Count = 0;
		arg[3].Package.Elements = NULL;
		args.Count = 4;
		args.Pointer = arg;

		status = AcpiEvaluateObject(sc->handle, "_DSM", &args, &result);
		if (ACPI_SUCCESS(status)) {
			device_printf(sc->dev, "_DSM query successful\n");
			if (result.Pointer)
				AcpiOsFree(result.Pointer);

			/* Function 1: enable */
			arg[2].Integer.Value = 1;
			result.Length = ACPI_ALLOCATE_BUFFER;
			result.Pointer = NULL;
			status = AcpiEvaluateObject(sc->handle, "_DSM",
			    &args, &result);
			if (ACPI_SUCCESS(status)) {
				device_printf(sc->dev, "_DSM enable called\n");
				if (result.Pointer)
					AcpiOsFree(result.Pointer);
			}
		}
	}

	/* _PR0 (Power Resources) */
	{
		ACPI_BUFFER result = { ACPI_ALLOCATE_BUFFER, NULL };

		status = AcpiEvaluateObject(sc->handle, "_PR0", NULL, &result);
		if (ACPI_SUCCESS(status)) {
			ACPI_OBJECT *obj = result.Pointer;
			if (obj && obj->Type == ACPI_TYPE_PACKAGE) {
				for (UINT32 i = 0; i < obj->Package.Count; i++) {
					ACPI_OBJECT *ref = &obj->Package.Elements[i];
					if (ref->Type == ACPI_TYPE_LOCAL_REFERENCE) {
						AcpiEvaluateObject(
						    ref->Reference.Handle,
						    "_ON", NULL, NULL);
						device_printf(sc->dev,
						    "Turned on power resource %d\n", i);
					}
				}
			}
			if (result.Pointer)
				AcpiOsFree(result.Pointer);
		}
	}

	/* PAUD power resource */
	{
		ACPI_HANDLE paud;

		status = AcpiGetHandle(NULL, "\\_SB.PCI0.PAUD", &paud);
		if (ACPI_SUCCESS(status)) {
			AcpiEvaluateObject(paud, "_ON", NULL, NULL);
			device_printf(sc->dev, "PAUD._ON called\n");
		}
	}

	DELAY(200000);
}

/* ================================================================
 * WPT (Wildcat Point / Broadwell-U) Power-Up
 *
 * Uses BAR1 MMIO for register access. BAR1 mirrors PCI config space
 * and is writable (confirmed on Dell XPS 13 9343).
 *
 * In ACPI mode, PCI config space reads to 0:13.0 return 0xFFFFFFFF
 * because the IOBP PCICFGCTL has PCICD set. BAR1 is the only way
 * to access the power control registers.
 *
 * The device starts in D3 in ACPI mode, which is exactly what the
 * Linux catpt driver expects.
 *
 * Based on Linux catpt driver dsp.c catpt_dsp_power_up()
 * ================================================================ */

/*
 * sst_wpt_power_down - catpt_dsp_power_down() via BAR1
 *
 * Full Linux catpt sequence:
 * 1. Disable DCLCGE
 * 2. Assert reset
 * 3. Set SSP bank clocks
 * 4. Select LP clock + disable MCLK
 * 5. Set register defaults
 * 6. VDRTCTL2 clock gating (CGEALL pattern + DTCGE)
 * 7. Gate DRAM (set DSRAMPGE + udelay 60)
 * 8. Gate IRAM (set ISRAMPGE + udelay 60)
 * 9. Set D3PGD, clear D3SRAMPGD
 * 10. D3hot + udelay 50
 * 11. Re-enable DCLCGE + udelay 50
 */
static void
sst_wpt_power_down(struct sst_softc *sc)
{
	uint32_t vdrtctl0, vdrtctl2, pmcs, mask, val;

	device_printf(sc->dev, "  --- Power-Down (catpt-exact) ---\n");

	/* 1. Disable DCLCGE */
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 &= ~SST_VDRTCTL2_DCLCGE;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);

	/* 2. Assert reset (if SHIM is accessible) */
	if (sc->mem_res != NULL)
		sst_dsp_reset(sc, true);

	/* 3-5. Set SSP bank clocks, LP clock, register defaults
	 * (only if SHIM accessible - during initial power cycle it may not be) */
	if (sc->mem_res != NULL) {
		sst_shim_update_bits(sc, SST_SHIM_CSR,
		    SST_CSR_SBCS0 | SST_CSR_SBCS1,
		    SST_CSR_SBCS0 | SST_CSR_SBCS1);
		sst_shim_update_bits(sc, SST_SHIM_CSR, SST_CSR_LPCS,
		    SST_CSR_LPCS);
		sst_shim_update_bits(sc, SST_SHIM_CLKCTL,
		    SST_CLKCTL_SMOS_MASK, 0);
		sst_dsp_set_regs_defaults(sc);
	}

	/* 6. VDRTCTL2 clock gating: set CGEALL & ~DCLCGE, clear DTCGE first */
	mask = SST_VDRTCTL2_CGEALL & ~SST_VDRTCTL2_DCLCGE;
	val = mask & ~SST_VDRTCTL2_DTCGE;
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 = (vdrtctl2 & ~mask) | val;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
	/* Then enable DTCGE */
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 |= SST_VDRTCTL2_DTCGE;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);

	/* 7. Gate DRAM: SET DSRAMPGE bits (1 = gate enabled = OFF) + 60us */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 |= SST_WPT_VDRTCTL0_DSRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);

	/* 8. Gate IRAM: SET ISRAMPGE bits (1 = gate enabled = OFF) + 60us */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 |= SST_WPT_VDRTCTL0_ISRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);

	/* 9. Set D3PGD=1, clear D3SRAMPGD (per Linux catpt) */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 |= SST_WPT_VDRTCTL0_D3PGD;
	vdrtctl0 &= ~SST_WPT_VDRTCTL0_D3SRAMPGD;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);

	/* 10. Enter D3hot + 50us */
	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	pmcs = (pmcs & ~SST_PMCS_PS_MASK) | SST_PMCS_PS_D3;
	bus_write_4(sc->shim_res, SST_PCI_PMCS, pmcs);
	DELAY(50);

	/* 11. Re-enable DCLCGE + 50us */
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 |= SST_VDRTCTL2_DCLCGE;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
	DELAY(50);

	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	device_printf(sc->dev,
	    "  After down: VDRTCTL0=0x%08x PMCS=0x%08x(D%d)\n",
	    vdrtctl0, pmcs, pmcs & SST_PMCS_PS_MASK);
}

/*
 * sst_wpt_power_up - catpt_dsp_power_up() via BAR1
 *
 * Full Linux catpt sequence from dsp.c:
 * 1. Disable DCLCGE
 * 2. VDRTCTL2 clock gating: set CGEALL & ~DCLCGE, clear DTCGE
 * 3. Transition to D0
 * 4. Set D3PGD + D3SRAMPGD (disable power gating)
 * 5a. Ungate DRAM: clear DSRAMPGE + udelay(60)
 * 5b. Ungate IRAM: clear ISRAMPGE + udelay(60)
 * (dummy reads happen later in sst_sram_sanitize after BAR0 is allocated)
 * 6-10. Register defaults, MCLK, clock, reset - done in dsp_init section
 * 11. Re-enable DCLCGE
 */
static int
sst_wpt_power_up(struct sst_softc *sc)
{
	uint32_t vdrtctl0, vdrtctl2, pmcs, mask, val;

	if (sc->shim_res == NULL) {
		device_printf(sc->dev, "WPT power-up: no BAR1 resource\n");
		return (ENXIO);
	}

	/* Check for dead BAR1 */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	if (vdrtctl0 == SST_INVALID_REG_VALUE) {
		device_printf(sc->dev,
		    "WPT power-up: BAR1 reads 0xFFFFFFFF - bus dead\n");
		return (ENXIO);
	}

	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	device_printf(sc->dev, "=== WPT Power Cycle (catpt-exact) ===\n");
	device_printf(sc->dev,
	    "  Initial: VDRTCTL0=0x%08x VDRTCTL2=0x%08x PMCS=0x%08x(D%d)\n",
	    vdrtctl0, vdrtctl2, pmcs, pmcs & SST_PMCS_PS_MASK);

	/* Power-down first (puts device in known state) */
	sst_wpt_power_down(sc);

	/* === catpt_dsp_power_up === */

	/* 1. Disable DCLCGE */
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 &= ~SST_VDRTCTL2_DCLCGE;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);

	/* 2. VDRTCTL2 clock gating: enable CGEALL except DCLCGE and DTCGE */
	mask = SST_VDRTCTL2_CGEALL & ~SST_VDRTCTL2_DCLCGE;
	val = mask & ~SST_VDRTCTL2_DTCGE;
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	vdrtctl2 = (vdrtctl2 & ~mask) | val;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);

	/* 3. Transition to D0 */
	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	pmcs = (pmcs & ~SST_PMCS_PS_MASK) | SST_PMCS_PS_D0;
	bus_write_4(sc->shim_res, SST_PCI_PMCS, pmcs);
	DELAY(100);	/* Let D0 settle */

	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	device_printf(sc->dev, "  After D0: PMCS=0x%08x(D%d) V0=0x%08x\n",
	    pmcs, pmcs & SST_PMCS_PS_MASK, vdrtctl0);

	/* 4. Set D3PGD + D3SRAMPGD (disable power gating) */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 |= (SST_WPT_VDRTCTL0_D3PGD |
	    SST_WPT_VDRTCTL0_D3SRAMPGD);
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	device_printf(sc->dev, "  After D3PGD: V0=0x%08x\n", vdrtctl0);

	/*
	 * 5a. Ungate DRAM: CLEAR DSRAMPGE bits (0 = gate disabled = ON)
	 *
	 * Linux catpt catpt_dsp_set_srampge() confirms:
	 *   power_up passes new=0 (CLEAR bits = SRAM powered ON)
	 *   power_down passes new=mask (SET bits = SRAM gated OFF)
	 * "Power Gate Enable": 1=gate enabled=off, 0=gate disabled=on
	 */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	device_printf(sc->dev, "  Pre-DRAM PGE: V0=0x%08x\n", vdrtctl0);
	vdrtctl0 &= ~SST_WPT_VDRTCTL0_DSRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	device_printf(sc->dev, "  Post-DRAM PGE: V0=0x%08x\n", vdrtctl0);

	/* 5b. Ungate IRAM: CLEAR ISRAMPGE bits (0 = gate disabled = ON) */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 &= ~SST_WPT_VDRTCTL0_ISRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	device_printf(sc->dev, "  Post-IRAM PGE: V0=0x%08x\n", vdrtctl0);

	/*
	 * NOTE: Linux catpt does register defaults, MCLK restore,
	 * LP clock select, SSP bank clocks, and reset release HERE
	 * (steps 6-10), but we need BAR0/SHIM access for those.
	 * They are done in the dsp_init section after BAR0 allocation.
	 *
	 * IMPORTANT: Do NOT re-enable DCLCGE here (step 11)!
	 * DCLCGE must remain disabled until firmware is loaded to SRAM.
	 * With DCLCGE enabled, MMIO writes to SRAM are silently lost
	 * (Linux catpt avoids this by using DMA for firmware loading).
	 * We re-enable DCLCGE after firmware boot in sst_fw_boot().
	 */

	/* Final state */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
	pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
	device_printf(sc->dev,
	    "  Final: VDRTCTL0=0x%08x VDRTCTL2=0x%08x PMCS=0x%08x(D%d)\n",
	    vdrtctl0, vdrtctl2, pmcs, pmcs & SST_PMCS_PS_MASK);
	device_printf(sc->dev,
	    "  D3PGD=%d D3SRAMPGD=%d ISRAMPGE=0x%x DSRAMPGE=0x%x\n",
	    !!(vdrtctl0 & SST_WPT_VDRTCTL0_D3PGD),
	    !!(vdrtctl0 & SST_WPT_VDRTCTL0_D3SRAMPGD),
	    (vdrtctl0 & SST_WPT_VDRTCTL0_ISRAMPGE_MASK) >>
		SST_WPT_VDRTCTL0_ISRAMPGE_SHIFT,
	    (vdrtctl0 & SST_WPT_VDRTCTL0_DSRAMPGE_MASK) >>
		SST_WPT_VDRTCTL0_DSRAMPGE_SHIFT);

	/* Quick BAR0 test - check multiple offsets */
	if (sc->mem_res != NULL) {
		uint32_t iram0 = bus_read_4(sc->mem_res, 0);
		uint32_t shim0 = bus_read_4(sc->mem_res, SST_SHIM_OFFSET);
		uint32_t sram_ctrl = bus_read_4(sc->mem_res, 0xFB000);
		device_printf(sc->dev,
		    "  BAR0 test: IRAM[0]=0x%08x SHIM[0]=0x%08x SRAM_CTRL=0x%08x\n",
		    iram0, shim0, sram_ctrl);
		if (iram0 != SST_INVALID_REG_VALUE ||
		    shim0 != SST_INVALID_REG_VALUE ||
		    sram_ctrl != SST_INVALID_REG_VALUE)
			device_printf(sc->dev, "  *** BAR0 ALIVE! ***\n");
	}

	return (0);
}

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
static void
sst_sram_sanitize(struct sst_softc *sc)
{
	int i;
	uint32_t dummy;
	int dram_ok = 0, dram_fail = 0;
	int iram_ok = 0, iram_fail = 0;

	if (sc->mem_res == NULL)
		return;

	device_printf(sc->dev, "=== SRAM Sanitize (dummy reads) ===\n");

	/* Dummy read from each DRAM block (20 blocks x 32KB) */
	for (i = 0; i < 20; i++) {
		dummy = bus_read_4(sc->mem_res,
		    SST_DRAM_OFFSET + i * SST_MEMBLOCK_SIZE);
		if (dummy == SST_INVALID_REG_VALUE)
			dram_fail++;
		else
			dram_ok++;
	}
	device_printf(sc->dev,
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
	device_printf(sc->dev,
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

		device_printf(sc->dev,
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

			device_printf(sc->dev,
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

static bool
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
static void
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

	device_printf(sc->dev, "=== BAR0 Region Scan ===\n");
	for (i = 0; i < (int)(sizeof(regs) / sizeof(regs[0])); i++) {
		if (regs[i].off >= rman_get_size(sc->mem_res))
			continue;
		val = bus_read_4(sc->mem_res, regs[i].off);
		if (val != SST_INVALID_REG_VALUE) {
			device_printf(sc->dev, "  0x%06x %-16s = 0x%08x ALIVE\n",
			    regs[i].off, regs[i].name, val);
			alive++;
		}
	}
	device_printf(sc->dev, "  %d regions alive\n", alive);
}

/* ================================================================
 * Enable SRAM Power via BAR0 Control Register
 * Note: 0xFB000 is now known to be the SHIM base (host offset).
 * The "SRAM_CTRL" was actually the SHIM CSR register.
 * ================================================================ */

#define SST_SRAM_CTRL_OFFSET	0xFB000
#define SST_SRAM_CTRL_ENABLE	0x1F	/* Bits 0-4 enable SRAM */
#define SST_PCI_BAR0_PHYS	0xDF800000	/* PCI-allocated BAR0 physical address */

/*
 * Immediate SRAM status check - can be called without any device setup.
 * This is for diagnosing WHEN the SRAM gets reset during driver load.
 * Returns: 1 if SRAM is alive, 0 if dead
 */
static int
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
static int
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

static int
sst_enable_sram(struct sst_softc *sc)
{
	uint32_t ctrl, ctrl_after, test_val;
	int retries, write_attempts;

	if (sc->mem_res == NULL) {
		device_printf(sc->dev, "SRAM Enable: BAR0 not allocated\n");
		return (ENXIO);
	}

	device_printf(sc->dev, "=== SRAM Power Enable Sequence ===\n");
	device_printf(sc->dev, "  Using BAR0 at: 0x%lx\n", rman_get_start(sc->mem_res));

	/* Check if SRAM is already accessible */
	test_val = bus_read_4(sc->mem_res, 0);
	if (test_val != SST_INVALID_REG_VALUE) {
		device_printf(sc->dev, "  SRAM already accessible: 0x%08x\n", test_val);
		return (0);
	}

	/* Read current control register value */
	ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	device_printf(sc->dev, "  SRAM_CTRL (0x%x) before: 0x%08x\n",
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
	device_printf(sc->dev, "  After clear: SRAM_CTRL = 0x%08x\n", ctrl_after);

	/* Phase B: Set enable bits */
	ctrl = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	ctrl |= SST_SRAM_CTRL_ENABLE;
	bus_write_4(sc->mem_res, SST_SRAM_CTRL_OFFSET, ctrl);
	DELAY(100000);  /* 100ms */

	ctrl_after = bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET);
	device_printf(sc->dev, "  After set: SRAM_CTRL = 0x%08x\n", ctrl_after);

	/* Check if SRAM is now accessible */
	for (retries = 0; retries < 20; retries++) {
		test_val = bus_read_4(sc->mem_res, 0);
		if (test_val != SST_INVALID_REG_VALUE) {
			device_printf(sc->dev, "  SRAM enabled! IRAM[0]=0x%08x CTRL=0x%08x\n",
			    test_val, bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET));
			return (0);
		}
		DELAY(10000);
	}

	/* Retry: try multiple toggle cycles */
	for (write_attempts = 0; write_attempts < 3; write_attempts++) {
		device_printf(sc->dev, "  Toggle attempt %d...\n", write_attempts + 1);

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
		device_printf(sc->dev, "  Attempt %d: CTRL=0x%08x IRAM[0]=0x%08x\n",
		    write_attempts + 1, ctrl_after, test_val);

		if (test_val != SST_INVALID_REG_VALUE) {
			device_printf(sc->dev, "  SRAM enabled after toggle!\n");
			return (0);
		}
	}

	device_printf(sc->dev, "  SRAM enable FAILED\n");
	device_printf(sc->dev, "  Final SRAM_CTRL: 0x%08x\n",
	    bus_read_4(sc->mem_res, SST_SRAM_CTRL_OFFSET));

	return (EIO);
}

/* ================================================================
 * Dump PCI Config via BAR1
 * ================================================================ */

static void
sst_dump_pci_config(struct sst_softc *sc)
{
	if (sc->shim_res == NULL)
		return;

	device_printf(sc->dev, "PCI Config (BAR1 mirror):\n");
	device_printf(sc->dev, "  VID/DID:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x00));
	device_printf(sc->dev, "  STS/CMD:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x04));
	device_printf(sc->dev, "  CLS/REV:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x08));
	device_printf(sc->dev, "  BAR0:     0x%08x\n",
	    bus_read_4(sc->shim_res, 0x10));
	device_printf(sc->dev, "  BAR1:     0x%08x\n",
	    bus_read_4(sc->shim_res, 0x14));
	device_printf(sc->dev, "  PMCSR:    0x%08x\n",
	    bus_read_4(sc->shim_res, SST_PCI_PMCS));
	device_printf(sc->dev, "  VDRTCTL0: 0x%08x\n",
	    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0));
	device_printf(sc->dev, "  VDRTCTL2: 0x%08x\n",
	    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2));
}

/* ================================================================
 * HDA Controller Enablement via RCBA
 *
 * On Broadwell-U, the BIOS disables the PCH HDA controller (0:1B.0)
 * when ADSP/SST mode is selected. We can re-enable it by clearing
 * the HDAD bit in the PCH Function Disable register.
 *
 * RCBA + 0x3418 (FD register) - from DSDT analysis:
 *   Bit 1: ADSD - Audio DSP Disable
 *   Bit 4: HDAD - HD Audio Disable
 * ================================================================ */

static int
sst_try_enable_hda(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base, fd, fd_new;
	uint32_t hda_vid;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	int error;

	device_printf(sc->dev, "=== HDA Controller Enablement ===\n");

	/* Read RCBA from LPC bridge */
	rcba_reg = pci_cfgregread(0, PCH_LPC_BUS, PCH_LPC_DEV,
	    PCH_LPC_FUNC, PCH_LPC_RCBA_REG, 4);
	rcba_base = rcba_reg & PCH_RCBA_MASK;

	device_printf(sc->dev, "  RCBA reg: 0x%08x, base: 0x%08x, %s\n",
	    rcba_reg, rcba_base,
	    (rcba_reg & PCH_RCBA_ENABLE) ? "enabled" : "DISABLED");

	if (rcba_base == 0 || rcba_base == PCH_RCBA_MASK ||
	    !(rcba_reg & PCH_RCBA_ENABLE)) {
		device_printf(sc->dev, "  RCBA not available\n");
		return (ENXIO);
	}

	/* Map RCBA region */
	mem_tag = X86_BUS_SPACE_MEM;
	error = bus_space_map(mem_tag, rcba_base, PCH_RCBA_SIZE, 0,
	    &rcba_handle);
	if (error != 0) {
		device_printf(sc->dev, "  Failed to map RCBA: %d\n", error);
		return (error);
	}

	/* Read Function Disable register */
	fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
	device_printf(sc->dev, "  FD [0x3418]: 0x%08x\n", fd);
	device_printf(sc->dev, "    ADSD (bit 1): %s\n",
	    (fd & PCH_FD_ADSD) ? "ADSP Disabled" : "ADSP Enabled");
	device_printf(sc->dev, "    HDAD (bit 4): %s\n",
	    (fd & PCH_FD_HDAD) ? "HDA Disabled" : "HDA Enabled");

	/* Check current HDA state on PCI bus */
	hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
	    PCH_HDA_FUNC, 0x00, 4);
	device_printf(sc->dev, "  HDA PCI (0:%x.%d): 0x%08x%s\n",
	    PCH_HDA_DEV, PCH_HDA_FUNC, hda_vid,
	    hda_vid != 0xFFFFFFFF ? " (PRESENT)" : " (absent)");

	if (hda_vid != 0xFFFFFFFF) {
		device_printf(sc->dev, "  HDA already present on PCI bus!\n");
		bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);
		return (0);
	}

	/* HDA is absent - try to enable it */
	if (fd & PCH_FD_HDAD) {
		device_printf(sc->dev, "  Enabling HDA controller...\n");

		fd_new = fd & ~PCH_FD_HDAD;	/* Clear HDAD to enable HDA */

		device_printf(sc->dev, "  Writing FD: 0x%08x -> 0x%08x\n",
		    fd, fd_new);
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);	/* 100ms settle */

		/* Verify */
		fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		device_printf(sc->dev, "  FD after write: 0x%08x\n", fd);
		device_printf(sc->dev, "    HDAD now: %s\n",
		    (fd & PCH_FD_HDAD) ? "still disabled (write rejected)" :
		    "ENABLED!");

		/* Check if HDA appeared on PCI */
		DELAY(100000);	/* Additional settle time */
		hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
		    PCH_HDA_FUNC, 0x00, 4);
		device_printf(sc->dev, "  HDA PCI after enable: 0x%08x\n",
		    hda_vid);

		if (hda_vid != 0xFFFFFFFF) {
			uint32_t hda_class = pci_cfgregread(0, PCH_HDA_BUS,
			    PCH_HDA_DEV, PCH_HDA_FUNC, 0x08, 4);
			device_printf(sc->dev,
			    "  *** HDA CONTROLLER APPEARED! ***\n");
			device_printf(sc->dev,
			    "  VID/DID: 0x%08x, Class: 0x%06x\n",
			    hda_vid, hda_class >> 8);

			/* Manually init HDA before hdac(4) attaches.
			 * The hdac driver fails with "stuck in reset"
			 * when the controller was just dynamically
			 * enabled.  We perform the HDA reset sequence
			 * here so hdac finds a clean controller. */
			{
				uint32_t hda_bar, hda_cmd;
				bus_space_tag_t ht = X86_BUS_SPACE_MEM;
				bus_space_handle_t hh;
				int herr, htimeout;

				/* Read and program HDA BAR */
				hda_bar = pci_cfgregread(0, PCH_HDA_BUS,
				    PCH_HDA_DEV, PCH_HDA_FUNC, 0x10, 4);
				if (hda_bar == 0 ||
				    hda_bar == 0xFFFFFFFF) {
					hda_bar = 0xDF900000;
					pci_cfgregwrite(0, PCH_HDA_BUS,
					    PCH_HDA_DEV, PCH_HDA_FUNC,
					    0x10, hda_bar, 4);
					DELAY(10000);
				}
				hda_bar &= ~0xF; /* mask type bits */

				/* Enable MEM + Bus Master */
				hda_cmd = pci_cfgregread(0, PCH_HDA_BUS,
				    PCH_HDA_DEV, PCH_HDA_FUNC, 0x04, 2);
				if ((hda_cmd & 0x06) != 0x06) {
					pci_cfgregwrite(0, PCH_HDA_BUS,
					    PCH_HDA_DEV, PCH_HDA_FUNC,
					    0x04, hda_cmd | 0x06, 2);
					DELAY(10000);
				}

				device_printf(sc->dev,
				    "  HDA BAR: 0x%08x, CMD: 0x%04x\n",
				    hda_bar, hda_cmd | 0x06);

				/* Map HDA MMIO */
				herr = bus_space_map(ht, hda_bar,
				    0x4000, 0, &hh);
				if (herr == 0) {
					uint32_t gcap, gctl, statests;

					gcap = bus_space_read_4(ht, hh,
					    0x00);
					gctl = bus_space_read_4(ht, hh,
					    0x08);
					device_printf(sc->dev,
					    "  HDA GCAP: 0x%08x,"
					    " GCTL: 0x%08x\n",
					    gcap, gctl);

					/* HDA Reset: enter reset */
					bus_space_write_4(ht, hh, 0x08,
					    gctl & ~1);
					for (htimeout = 100;
					    htimeout > 0; htimeout--) {
						DELAY(1000);
						gctl = bus_space_read_4(ht,
						    hh, 0x08);
						if ((gctl & 1) == 0)
							break;
					}
					device_printf(sc->dev,
					    "  HDA in reset: GCTL=0x%08x"
					    " (%s)\n", gctl,
					    (gctl & 1) == 0 ? "OK" :
					    "FAILED");

					/* Exit reset */
					DELAY(1000);
					bus_space_write_4(ht, hh, 0x08,
					    gctl | 1);
					for (htimeout = 100;
					    htimeout > 0; htimeout--) {
						DELAY(1000);
						gctl = bus_space_read_4(ht,
						    hh, 0x08);
						if (gctl & 1)
							break;
					}
					device_printf(sc->dev,
					    "  HDA out of reset:"
					    " GCTL=0x%08x (%s)\n", gctl,
					    (gctl & 1) ? "OK" : "FAILED");

					/* Wait for codec discovery */
					DELAY(500000); /* 500ms */

					/* Check STATESTS for codecs */
					statests = bus_space_read_2(ht,
					    hh, 0x0E);
					device_printf(sc->dev,
					    "  HDA STATESTS: 0x%04x"
					    " (codecs: %s%s%s%s)\n",
					    statests,
					    (statests & 1) ? "SDI0 " : "",
					    (statests & 2) ? "SDI1 " : "",
					    (statests & 4) ? "SDI2 " : "",
					    statests == 0 ? "NONE" : "");

					gcap = bus_space_read_4(ht, hh,
					    0x00);
					device_printf(sc->dev,
					    "  HDA GCAP after: 0x%08x\n",
					    gcap);

					bus_space_unmap(ht, hh, 0x4000);
				} else {
					device_printf(sc->dev,
					    "  HDA map failed: %d\n",
					    herr);
				}
			}

			/* Trigger PCI bus rescan so hdac(4)
			 * discovers the new device */
			{
				device_t acpi, pcib, pci;

				acpi = device_get_parent(sc->dev);
				pcib = device_find_child(acpi,
				    "pcib", 0);
				if (pcib != NULL) {
					pci = device_find_child(pcib,
					    "pci", 0);
					if (pci != NULL) {
						device_printf(sc->dev,
						    "  Triggering PCI"
						    " rescan...\n");
						bus_topo_lock();
						BUS_RESCAN(pci);
						bus_topo_unlock();
					}
				}
			}
		} else {
			device_printf(sc->dev,
			    "  HDA did not appear."
			    " FD write may need reboot.\n");
		}
	} else {
		device_printf(sc->dev, "  HDAD bit already clear (HDA enabled)\n");
		device_printf(sc->dev,
		    "  But HDA not on PCI bus - trying toggle...\n");

		/* Try toggling: set HDAD, wait, clear it again */
		fd_new = fd | PCH_FD_HDAD;
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);
		device_printf(sc->dev, "  FD after set HDAD: 0x%08x\n",
		    bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD));

		fd_new = fd & ~PCH_FD_HDAD;
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(200000);

		fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		device_printf(sc->dev, "  FD after clear HDAD: 0x%08x\n", fd);

		hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
		    PCH_HDA_FUNC, 0x00, 4);
		device_printf(sc->dev,
		    "  HDA PCI after toggle: 0x%08x%s\n",
		    hda_vid,
		    hda_vid != 0xFFFFFFFF ? " (APPEARED!)" :
		    " (still absent)");

		if (hda_vid != 0xFFFFFFFF) {
			device_t acpi, pcib, pci;

			acpi = device_get_parent(sc->dev);
			pcib = device_find_child(acpi,
			    "pcib", 0);
			if (pcib != NULL) {
				pci = device_find_child(pcib,
				    "pci", 0);
				if (pci != NULL) {
					device_printf(sc->dev,
					    "  Triggering PCI"
					    " rescan...\n");
					bus_topo_lock();
					BUS_RESCAN(pci);
					bus_topo_unlock();
				}
			}
		}
	}

	/* Also dump FD2 for reference */
	{
		uint32_t fd2 = bus_space_read_4(mem_tag, rcba_handle,
		    PCH_RCBA_FD2);
		device_printf(sc->dev, "  FD2 [0x3428]: 0x%08x\n", fd2);
	}

	/* Scan relevant RCBA registers for debugging */
	{
		uint32_t v;

		v = bus_space_read_4(mem_tag, rcba_handle, 0x2030);
		device_printf(sc->dev, "  RCBA [0x2030] LPSS: 0x%08x\n", v);

		v = bus_space_read_4(mem_tag, rcba_handle, 0x3410);
		device_printf(sc->dev, "  RCBA [0x3410] CG:   0x%08x\n", v);
	}

	bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);

	return (hda_vid != 0xFFFFFFFF ? 0 : ENXIO);
}

/* ================================================================
 * ADSP Enablement via RCBA
 *
 * When ADSD (bit 0 of FD register) is set, the ADSP function is
 * disabled at the PCH level. This means BAR0 (DSP MMIO) is gated
 * off and returns 0xFFFFFFFF, even though BAR1 (PCI config mirror)
 * may still be accessible via the LPSS private config space.
 *
 * This function clears ADSD to re-enable the ADSP hardware,
 * which should make BAR0 accessible.
 * ================================================================ */

static int
sst_try_enable_adsp(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base, fd, fd_new, fd_verify;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	int error;

	device_printf(sc->dev, "=== ADSP Enablement ===\n");

	/* Read RCBA from LPC bridge */
	rcba_reg = pci_cfgregread(0, PCH_LPC_BUS, PCH_LPC_DEV,
	    PCH_LPC_FUNC, PCH_LPC_RCBA_REG, 4);
	rcba_base = rcba_reg & PCH_RCBA_MASK;

	if (rcba_base == 0 || rcba_base == PCH_RCBA_MASK ||
	    !(rcba_reg & PCH_RCBA_ENABLE)) {
		device_printf(sc->dev, "  RCBA not available\n");
		return (ENXIO);
	}

	/* Map RCBA region */
	mem_tag = X86_BUS_SPACE_MEM;
	error = bus_space_map(mem_tag, rcba_base, PCH_RCBA_SIZE, 0,
	    &rcba_handle);
	if (error != 0) {
		device_printf(sc->dev, "  Failed to map RCBA: %d\n", error);
		return (error);
	}

	/* Read FD register */
	fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
	device_printf(sc->dev, "  FD before: 0x%08x (ADSD=%d, HDAD=%d)\n",
	    fd, (fd & PCH_FD_ADSD) ? 1 : 0, (fd & PCH_FD_HDAD) ? 1 : 0);

	if (!(fd & PCH_FD_ADSD)) {
		device_printf(sc->dev, "  ADSP already enabled\n");
		bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);
		return (0);
	}

	/* Clear ADSD to enable ADSP, also set HDAD to disable HDA
	 * (only one of HDA/ADSP should be active) */
	fd_new = fd & ~PCH_FD_ADSD;	/* Enable ADSP */
	fd_new |= PCH_FD_HDAD;		/* Disable HDA (mutual exclusion) */

	device_printf(sc->dev, "  Writing FD: 0x%08x -> 0x%08x\n",
	    fd, fd_new);
	device_printf(sc->dev, "  (Clearing ADSD to enable ADSP, setting HDAD)\n");

	bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
	DELAY(100000);	/* 100ms settle */

	/* Verify write took effect */
	fd_verify = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
	device_printf(sc->dev, "  FD after:  0x%08x (ADSD=%d, HDAD=%d)\n",
	    fd_verify,
	    (fd_verify & PCH_FD_ADSD) ? 1 : 0,
	    (fd_verify & PCH_FD_HDAD) ? 1 : 0);

	if (fd_verify & PCH_FD_ADSD) {
		device_printf(sc->dev,
		    "  ADSD write REJECTED by hardware\n");
		/* Try just clearing ADSD without touching HDAD */
		fd_new = fd & ~PCH_FD_ADSD;
		device_printf(sc->dev,
		    "  Retry: writing FD 0x%08x (only clear ADSD)\n", fd_new);
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);
		fd_verify = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		device_printf(sc->dev, "  FD retry:  0x%08x (ADSD=%d)\n",
		    fd_verify, (fd_verify & PCH_FD_ADSD) ? 1 : 0);
	}

	if (!(fd_verify & PCH_FD_ADSD)) {
		device_printf(sc->dev, "  *** ADSP ENABLED! ***\n");
		device_printf(sc->dev, "  Waiting for hardware to stabilize...\n");
		DELAY(200000);	/* 200ms for hardware to come up */
	} else {
		device_printf(sc->dev, "  ADSD is write-locked, cannot enable ADSP via FD\n");
	}

	bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);

	return ((fd_verify & PCH_FD_ADSD) ? EPERM : 0);
}

/* ================================================================
 * I2C Codec Probe
 *
 * The I2C1 controller at 0xFE105000 is a DesignWare I2C that talks
 * to the RT286/ALC3263 audio codec at address 0x1C on I2C0.
 * We probe it to verify the codec is alive.
 * ================================================================ */

static void
sst_probe_i2c_codec(struct sst_softc *sc)
{

	/*
	 * Legacy I2C codec probe - functionality moved to sst_codec.c.
	 * Full codec init is now done by sst_codec_init() / sst_codec_enable_speaker().
	 */
	device_printf(sc->dev,
	    "I2C codec probe: handled by sst_codec module\n");
}

/* ================================================================
 * IOBP (I/O Bridge Port) Sideband Access
 *
 * The PCH uses an IOBP sideband interface to control internal device
 * configuration. This is the mechanism that controls PCI/ACPI mode
 * switching, BAR decode enables, and power management for LPSS devices.
 *
 * Source: coreboot src/southbridge/intel/lynxpoint/iobp.c
 * ================================================================ */

static int
sst_iobp_poll(bus_space_tag_t mt, bus_space_handle_t rh)
{
	int timeout;

	for (timeout = 5000; timeout > 0; timeout--) {
		if ((bus_space_read_2(mt, rh, PCH_IOBPS) &
		    PCH_IOBPS_READY) == 0)
			return (0);
		DELAY(10);
	}
	return (ETIMEDOUT);
}

static int
sst_iobp_read(bus_space_tag_t mt, bus_space_handle_t rh,
    uint32_t addr, uint32_t *val)
{
	int error;

	/* Wait for not busy */
	error = sst_iobp_poll(mt, rh);
	if (error)
		return (error);

	/* Write target address */
	bus_space_write_4(mt, rh, PCH_IOBPIRI, addr);

	/* Set READ opcode and trigger */
	bus_space_write_2(mt, rh, PCH_IOBPS,
	    PCH_IOBPS_READ | PCH_IOBPS_READY);

	/* Wait for completion */
	error = sst_iobp_poll(mt, rh);
	if (error)
		return (error);

	/* Check transaction status */
	if (bus_space_read_2(mt, rh, PCH_IOBPS) & PCH_IOBPS_TX_MASK)
		return (EIO);

	/* Read data */
	*val = bus_space_read_4(mt, rh, PCH_IOBPD);
	return (0);
}

static int
sst_iobp_write(bus_space_tag_t mt, bus_space_handle_t rh,
    uint32_t addr, uint32_t val)
{
	int error;

	/* Wait for not busy */
	error = sst_iobp_poll(mt, rh);
	if (error)
		return (error);

	/* Write target address */
	bus_space_write_4(mt, rh, PCH_IOBPIRI, addr);

	/* Set WRITE opcode */
	bus_space_write_2(mt, rh, PCH_IOBPS, PCH_IOBPS_WRITE);

	/* Write data */
	bus_space_write_4(mt, rh, PCH_IOBPD, val);

	/* Write magic value and trigger */
	bus_space_write_2(mt, rh, PCH_IOBPU, PCH_IOBPU_MAGIC);
	bus_space_write_2(mt, rh, PCH_IOBPS,
	    PCH_IOBPS_WRITE | PCH_IOBPS_READY);

	/* Wait for completion */
	error = sst_iobp_poll(mt, rh);
	if (error)
		return (error);

	/* Check transaction status */
	if (bus_space_read_2(mt, rh, PCH_IOBPS) & PCH_IOBPS_TX_MASK)
		return (EIO);

	return (0);
}

/*
 * Read and dump all ADSP IOBP registers, then try to enable BAR0.
 */
static void
sst_iobp_probe(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base;
	uint32_t pcicfgctl, pmctl, vdldat1, vdldat2;
	bus_space_tag_t mt;
	bus_space_handle_t rh;
	int error;

	device_printf(sc->dev, "=== IOBP Sideband Probe ===\n");

	rcba_reg = pci_cfgregread(0, PCH_LPC_BUS, PCH_LPC_DEV,
	    PCH_LPC_FUNC, PCH_LPC_RCBA_REG, 4);
	rcba_base = rcba_reg & PCH_RCBA_MASK;

	if (rcba_base == 0 || !(rcba_reg & PCH_RCBA_ENABLE)) {
		device_printf(sc->dev, "  RCBA not available\n");
		return;
	}

	mt = X86_BUS_SPACE_MEM;
	error = bus_space_map(mt, rcba_base, PCH_RCBA_SIZE, 0, &rh);
	if (error != 0) {
		device_printf(sc->dev, "  Failed to map RCBA: %d\n", error);
		return;
	}

	/* Read IOBP registers */
	error = sst_iobp_read(mt, rh, ADSP_IOBP_PCICFGCTL, &pcicfgctl);
	if (error != 0) {
		device_printf(sc->dev, "  IOBP read failed: %d\n", error);
		goto out;
	}
	device_printf(sc->dev, "  PCICFGCTL: 0x%08x (PCICD=%d ACPIIE=%d)\n",
	    pcicfgctl,
	    !!(pcicfgctl & ADSP_PCICFGCTL_PCICD),
	    !!(pcicfgctl & ADSP_PCICFGCTL_ACPIIE));

	error = sst_iobp_read(mt, rh, ADSP_IOBP_PMCTL, &pmctl);
	if (error == 0)
		device_printf(sc->dev, "  PMCTL: 0x%08x %s\n",
		    pmctl, pmctl == 0x3f ? "(OK)" : "(NEED 0x3f!)");
	error = sst_iobp_read(mt, rh, ADSP_IOBP_VDLDAT1, &vdldat1);
	if (error == 0)
		device_printf(sc->dev, "  VDLDAT1: 0x%08x\n", vdldat1);
	error = sst_iobp_read(mt, rh, ADSP_IOBP_VDLDAT2, &vdldat2);
	if (error == 0)
		device_printf(sc->dev, "  VDLDAT2: 0x%08x\n", vdldat2);

	/* Set PMCTL if needed */
	if (pmctl != ADSP_PMCTL_VALUE) {
		device_printf(sc->dev, "  Setting PMCTL=0x3f\n");
		sst_iobp_write(mt, rh, ADSP_IOBP_PMCTL, ADSP_PMCTL_VALUE);
		DELAY(10000);
	}

	/* Clear PCICD to enable PCI config access */
	if (pcicfgctl & ADSP_PCICFGCTL_PCICD) {
		uint32_t new_cfg = pcicfgctl & ~(ADSP_PCICFGCTL_PCICD |
		    ADSP_PCICFGCTL_ACPIIE | 0x100);
		device_printf(sc->dev, "  Clearing PCICD: 0x%08x -> 0x%08x\n",
		    pcicfgctl, new_cfg);
		sst_iobp_write(mt, rh, ADSP_IOBP_PCICFGCTL, new_cfg);
		DELAY(100000);

		/* Verify PCI is visible */
		{
			uint32_t vid = pci_cfgregread(0, 0, 0x13, 0, 0x00, 4);
			device_printf(sc->dev, "  PCI VID: 0x%08x %s\n",
			    vid, vid != 0xFFFFFFFF ? "(VISIBLE)" : "(absent)");
			if (vid == 0xFFFFFFFF)
				goto restore;
		}
	}

	/*
	 * Full WPT power cycle via REAL PCI config space.
	 * No PCI rescan - we just use the config space directly.
	 */
	{
		uint32_t v0, v2, pmcs;

		device_printf(sc->dev, "=== PCI Config Power Cycle ===\n");

		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		v2 = pci_cfgregread(0, 0, 0x13, 0, 0xA8, 4);
		pmcs = pci_cfgregread(0, 0, 0x13, 0, 0x84, 4);
		device_printf(sc->dev,
		    "  Initial: V0=0x%08x V2=0x%08x PMCS=0x%08x\n",
		    v0, v2, pmcs);

		/*
		 * catpt_dsp_power_down via PCI config:
		 * Gate SRAM, set D3PGD=1+D3SRAMPGD=0, enter D3hot
		 */
		/* 1. Disable DCLCGE */
		v2 &= ~SST_VDRTCTL2_DCLCGE;
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA8, v2, 4);

		/* 2. Gate all SRAM */
		v0 |= (SST_WPT_VDRTCTL0_ISRAMPGE_MASK |
		    SST_WPT_VDRTCTL0_DSRAMPGE_MASK);
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA0, v0, 4);

		/* 3. D3PGD=1, D3SRAMPGD=0 (per Linux catpt) */
		v0 |= SST_WPT_VDRTCTL0_D3PGD;
		v0 &= ~SST_WPT_VDRTCTL0_D3SRAMPGD;
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA0, v0, 4);

		/* 4. Enter D3hot */
		pmcs = (pci_cfgregread(0, 0, 0x13, 0, 0x84, 4) & ~3) | 3;
		pci_cfgregwrite(0, 0, 0x13, 0, 0x84, pmcs, 4);
		DELAY(50);

		/* 5. Re-enable DCLCGE */
		v2 |= SST_VDRTCTL2_DCLCGE;
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA8, v2, 4);
		DELAY(50);

		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		pmcs = pci_cfgregread(0, 0, 0x13, 0, 0x84, 4);
		device_printf(sc->dev,
		    "  After DOWN: V0=0x%08x PMCS=0x%08x(D%d)\n",
		    v0, pmcs, pmcs & 3);

		/*
		 * catpt_dsp_power_up via PCI config:
		 * D0, D3PGD+D3SRAMPGD, ungate SRAM
		 */
		/* 1. Disable DCLCGE */
		v2 = pci_cfgregread(0, 0, 0x13, 0, 0xA8, 4);
		v2 &= ~SST_VDRTCTL2_DCLCGE;
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA8, v2, 4);

		/* 2. Transition to D0 (BEFORE setting D3PGD per catpt) */
		pmcs = pci_cfgregread(0, 0, 0x13, 0, 0x84, 4);
		pmcs = (pmcs & ~3) | 0;
		pci_cfgregwrite(0, 0, 0x13, 0, 0x84, pmcs, 4);

		/* 3. Set D3PGD + D3SRAMPGD */
		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		v0 |= (SST_WPT_VDRTCTL0_D3PGD |
		    SST_WPT_VDRTCTL0_D3SRAMPGD);
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA0, v0, 4);

		/* 4. Ungate all SRAM */
		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		v0 &= ~(SST_WPT_VDRTCTL0_ISRAMPGE_MASK |
		    SST_WPT_VDRTCTL0_DSRAMPGE_MASK);
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA0, v0, 4);

		/* 5. Re-enable DCLCGE */
		v2 |= SST_VDRTCTL2_DCLCGE;
		pci_cfgregwrite(0, 0, 0x13, 0, 0xA8, v2, 4);
		DELAY(100000);

		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		v2 = pci_cfgregread(0, 0, 0x13, 0, 0xA8, 4);
		pmcs = pci_cfgregread(0, 0, 0x13, 0, 0x84, 4);
		device_printf(sc->dev,
		    "  Final: V0=0x%08x V2=0x%08x PMCS=0x%08x(D%d)\n",
		    v0, v2, pmcs, pmcs & 3);
		device_printf(sc->dev,
		    "  D3PGD=%d D3SRAMPGD=%d ISRAMPGE=0x%x DSRAMPGE=0x%x\n",
		    !!(v0 & SST_WPT_VDRTCTL0_D3PGD),
		    !!(v0 & SST_WPT_VDRTCTL0_D3SRAMPGD),
		    (v0 & SST_WPT_VDRTCTL0_ISRAMPGE_MASK) >>
			SST_WPT_VDRTCTL0_ISRAMPGE_SHIFT,
		    (v0 & SST_WPT_VDRTCTL0_DSRAMPGE_MASK) >>
			SST_WPT_VDRTCTL0_DSRAMPGE_SHIFT);

		/* Test BAR0 directly */
		{
			bus_space_tag_t bmt = X86_BUS_SPACE_MEM;
			bus_space_handle_t bh;
			if (bus_space_map(bmt, 0xFE000000, 0x1000, 0,
			    &bh) == 0) {
				uint32_t bv = bus_space_read_4(bmt, bh, 0);
				device_printf(sc->dev,
				    "  BAR0[0]: 0x%08x %s\n",
				    bv,
				    bv != 0xFFFFFFFF ? "*** ALIVE! ***" :
				    "dead");
				bus_space_unmap(bmt, bh, 0x1000);
			}
		}

		/* Ensure MEM+BM enabled */
		{
			uint32_t cmd = pci_cfgregread(0, 0, 0x13, 0, 0x04, 4);
			if ((cmd & 0x06) != 0x06) {
				pci_cfgregwrite(0, 0, 0x13, 0, 0x04,
				    (cmd & 0xFFFF) | 0x06, 2);
				DELAY(10000);
			}
		}
	}

restore:
	/* Restore PCICD to avoid PCI driver interference */
	if (pcicfgctl & ADSP_PCICFGCTL_PCICD) {
		device_printf(sc->dev,
		    "  Restoring PCICD (hiding PCI device)\n");
		sst_iobp_write(mt, rh, ADSP_IOBP_PCICFGCTL, pcicfgctl);
		DELAY(50000);
	}

out:
	bus_space_unmap(mt, rh, PCH_RCBA_SIZE);
}

/* ================================================================
 * PCH Register Dump
 *
 * Read LPC bridge config and PCH registers to understand the LPSS
 * power/routing state. The LPSS fabric won't work if the PCH hasn't
 * enabled the memory decode for these regions.
 * ================================================================ */

static void
sst_dump_pch_state(struct sst_softc *sc)
{
	uint32_t v, pmbase, gpiobase;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	uint32_t rcba_reg, rcba_base;
	int error;

	device_printf(sc->dev, "=== PCH State Dump ===\n");

	/* LPC bridge PCI config registers */
	device_printf(sc->dev, "  LPC (0:1F.0) VID/DID: 0x%08x\n",
	    pci_cfgregread(0, 0, 0x1F, 0, 0x00, 4));

	pmbase = pci_cfgregread(0, 0, 0x1F, 0, 0x40, 4);
	device_printf(sc->dev, "  LPC PMBASE [0x40]: 0x%08x (I/O: 0x%04x)\n",
	    pmbase, pmbase & 0xFF80);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x44, 4);
	device_printf(sc->dev, "  LPC ACPI_CNTL [0x44]: 0x%08x%s\n",
	    v, (v & 0x80) ? " (ACPI enabled)" : " (ACPI disabled)");

	gpiobase = pci_cfgregread(0, 0, 0x1F, 0, 0x48, 4);
	device_printf(sc->dev, "  LPC GPIOBASE [0x48]: 0x%08x\n", gpiobase);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x4C, 4);
	device_printf(sc->dev, "  LPC GPIO_CNTL [0x4C]: 0x%08x\n", v);

	/* GCS (General Control and Status) */
	v = pci_cfgregread(0, 0, 0x1F, 0, 0xA0, 4);
	device_printf(sc->dev, "  LPC GCS [0xA0]: 0x%08x\n", v);

	/* LPC Decode ranges */
	v = pci_cfgregread(0, 0, 0x1F, 0, 0x80, 4);
	device_printf(sc->dev, "  LPC ILB_BASE [0x80]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x84, 4);
	device_printf(sc->dev, "  LPC IOAPIC_BASE [0x84]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x88, 4);
	device_printf(sc->dev, "  LPC SPI_BASE [0x88]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x8C, 4);
	device_printf(sc->dev, "  LPC MPHY_BASE [0x8C]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0xDC, 4);
	device_printf(sc->dev, "  LPC BDE [0xDC]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0xF0, 4);
	device_printf(sc->dev, "  LPC RCBA [0xF0]: 0x%08x\n", v);

	/* Map RCBA and read more registers */
	rcba_reg = pci_cfgregread(0, 0, 0x1F, 0, 0xF0, 4);
	rcba_base = rcba_reg & PCH_RCBA_MASK;
	mem_tag = X86_BUS_SPACE_MEM;

	if (rcba_base != 0 && (rcba_reg & PCH_RCBA_ENABLE)) {
		error = bus_space_map(mem_tag, rcba_base, PCH_RCBA_SIZE,
		    0, &rcba_handle);
		if (error == 0) {
			/* SOFT_RESET_CTRL / SOFT_RESET_DATA */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x0038);
			device_printf(sc->dev,
			    "  RCBA [0x0038] SOFTRSTCTRL: 0x%08x\n", v);

			/* D31IP - Device 31 Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3100);
			device_printf(sc->dev,
			    "  RCBA [0x3100] D31IP: 0x%08x\n", v);

			/* D27IP - Device 27 (HDA) Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3110);
			device_printf(sc->dev,
			    "  RCBA [0x3110] D27IP: 0x%08x\n", v);

			/* D19IP - Device 19 (LPSS) Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3130);
			device_printf(sc->dev,
			    "  RCBA [0x3130] D19IP: 0x%08x\n", v);

			/* LPSS I/O fabric configuration */
			/* Check known LPSS-related RCBA offsets */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x2330);
			device_printf(sc->dev,
			    "  RCBA [0x2330] SCC: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x2334);
			device_printf(sc->dev,
			    "  RCBA [0x2334] SCC2: 0x%08x\n", v);

			/* Function Disable 2 */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3428);
			device_printf(sc->dev,
			    "  RCBA [0x3428] FD2: 0x%08x\n", v);

			/* Check DISPBDF and other SBI registers */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3420);
			device_printf(sc->dev,
			    "  RCBA [0x3420] DISPBDF: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3424);
			device_printf(sc->dev,
			    "  RCBA [0x3424] FD_LOCK: 0x%08x\n", v);

			/* Broadwell LPSS fabric specific registers */
			/* PMC function bar decode */
			int i;
			for (i = 0x2000; i <= 0x2050; i += 4) {
				v = bus_space_read_4(mem_tag, rcba_handle, i);
				if (v != 0)
					device_printf(sc->dev,
					    "  RCBA [0x%04x]: 0x%08x\n",
					    i, v);
			}

			/* IOSFBCTL / IO sideband fabric */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3400);
			device_printf(sc->dev,
			    "  RCBA [0x3400] IOSFCTL: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3404);
			device_printf(sc->dev,
			    "  RCBA [0x3404] IOSFDAT: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3408);
			device_printf(sc->dev,
			    "  RCBA [0x3408] IOSFST: 0x%08x\n", v);

			/* CG - Clock Gating */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3410);
			device_printf(sc->dev,
			    "  RCBA [0x3410] CG: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3414);
			device_printf(sc->dev,
			    "  RCBA [0x3414] CG2: 0x%08x\n", v);

			/* FD */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3418);
			device_printf(sc->dev,
			    "  RCBA [0x3418] FD: 0x%08x\n", v);

			/* Check if there's an LPSS memory base register */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x340C);
			device_printf(sc->dev,
			    "  RCBA [0x340C]: 0x%08x\n", v);

			bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);
		}
	}

	/* Try reading ACPI PM registers via I/O space */
	{
		uint16_t pm_base = pmbase & 0xFF80;
		if (pm_base != 0) {
			uint32_t pm1_sts, pm1_cnt;

			pm1_sts = inw(pm_base);
			pm1_cnt = inl(pm_base + 0x04);
			device_printf(sc->dev,
			    "  ACPI PM1_STS: 0x%04x, PM1_CNT: 0x%08x\n",
			    pm1_sts, pm1_cnt);

			/* GPE0 status */
			v = inl(pm_base + 0x20);
			device_printf(sc->dev,
			    "  ACPI GPE0_STS: 0x%08x\n", v);
		}
	}

	/* Direct-map BAR0 and try reading (bypass resource allocation) */
	{
		bus_space_handle_t bar0_handle;

		device_printf(sc->dev,
		    "  Direct BAR0 map test (0xFE000000)...\n");
		error = bus_space_map(mem_tag, 0xFE000000, 0x1000, 0,
		    &bar0_handle);
		if (error == 0) {
			uint32_t v0 = bus_space_read_4(mem_tag, bar0_handle, 0);
			uint32_t v4 = bus_space_read_4(mem_tag, bar0_handle, 4);
			device_printf(sc->dev,
			    "  Direct BAR0[0]=0x%08x [4]=0x%08x\n", v0, v4);

			/* Try SHIM offset */
			if (0xC0000 < 0x1000) {
				/* Can't reach SHIM with 4K map */
			}
			bus_space_unmap(mem_tag, bar0_handle, 0x1000);
		} else {
			device_printf(sc->dev,
			    "  Direct BAR0 map failed: %d\n", error);
		}

		/* Also try mapping just the SHIM region */
		device_printf(sc->dev,
		    "  Direct SHIM map test (0xFE0C0000)...\n");
		error = bus_space_map(mem_tag, 0xFE0C0000, 0x1000, 0,
		    &bar0_handle);
		if (error == 0) {
			uint32_t v0 = bus_space_read_4(mem_tag, bar0_handle, 0);
			device_printf(sc->dev,
			    "  Direct SHIM[0]=0x%08x\n", v0);
			bus_space_unmap(mem_tag, bar0_handle, 0x1000);
		} else {
			device_printf(sc->dev,
			    "  Direct SHIM map failed: %d\n", error);
		}
	}
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
	device_printf(dev, "BAR1: 0x%lx (size: 0x%lx)\n",
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
	device_printf(dev, "BAR0: 0x%lx (size: 0x%lx)\n",
	    rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* Immediate SRAM write test - before any SHIM manipulation */
	{
		uint32_t t0, t1, t2, t3;

		t0 = bus_read_4(sc->mem_res, SST_DRAM_OFFSET);
		t1 = bus_read_4(sc->mem_res, SST_IRAM_OFFSET);
		device_printf(dev,
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

		device_printf(dev,
		    "SRAM write test: DRAM[0]=0x%08x (want 0xCAFEBABE) "
		    "IRAM[0]=0x%08x (want 0xDEADBEEF)\n", t2, t3);

		/* Restore */
		bus_write_4(sc->mem_res, SST_DRAM_OFFSET, t0);
		bus_write_4(sc->mem_res, SST_IRAM_OFFSET, t1);
	}

	/* Enable SRAM power via control register */
	error = sst_enable_sram(sc);
	if (error != 0) {
		device_printf(dev, "SRAM enable returned %d, continuing anyway\n", error);
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
	device_printf(dev, "BAR0 test: %s\n",
	    bar0_ok ? "ACCESSIBLE" : "DEAD (0xFFFFFFFF)");

	if (bar0_ok) {
		/* Scan all BAR0 regions */
		sst_scan_bar0(sc);

		/* Probe DSP memory regions (WPT offsets) */
		device_printf(dev, "DSP Memory Layout (WPT):\n");
		device_printf(dev, "  DRAM  (0x%06x): 0x%08x\n",
		    SST_DRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_DRAM_OFFSET));
		device_printf(dev, "  IRAM  (0x%06x): 0x%08x\n",
		    SST_IRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_IRAM_OFFSET));
		device_printf(dev, "  SHIM  (0x%06x): 0x%08x\n",
		    SST_SHIM_OFFSET,
		    bus_read_4(sc->mem_res, SST_SHIM_OFFSET));

		csr = sst_shim_read(sc, SST_SHIM_CSR);
		device_printf(dev, "  SHIM CSR: 0x%08x\n", csr);

		/* Continue with full DSP initialization... */
		goto dsp_init;
	}

	/* ---- Phase 5: BAR0 Failed - Diagnose ---- */
	device_printf(dev, "=== BAR0 inaccessible - diagnosing ===\n");

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

		device_printf(dev, "=== BIOS NVS Variables ===\n");

		map_err = bus_space_map(mt, PCH_GNVS_BASE, PCH_GNVS_SIZE,
		    0, &gnvs_handle);
		if (map_err == 0) {
			uint16_t osys_val;
			uint8_t s0id_val, ancs_val;

			/* OSYS at offset 0x00 (16-bit) */
			osys_val = bus_space_read_2(mt, gnvs_handle, 0x00);
			device_printf(dev, "  OSYS: 0x%04x (%s)\n",
			    osys_val,
			    osys_val >= 0x07DC ? "Windows 8+" :
			    osys_val >= 0x07D9 ? "Windows 7" :
			    "Pre-Win7");

			/* Read all bytes at known approximate offsets
			 * and dump relevant range for S0ID, ANCS, SMD0 */
			{
				int i;
				device_printf(dev,
				    "  GNVS raw [0x1C0-0x1D0]:");
				for (i = 0x1C0; i < 0x1D0; i++) {
					if ((i & 0xF) == 0)
						device_printf(dev,
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
					device_printf(dev,
					    "  S0ID: %d (%s)\n",
					    s0id_val,
					    s0id_val ?
					    "Connected Standby" :
					    "No CS");
				} else {
					device_printf(dev,
					    "  S0ID: (cannot read)\n");
					s0id_val = 0;
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\ANCS", &aval);
				if (ACPI_SUCCESS(ast)) {
					ancs_val = (uint8_t)aval;
					device_printf(dev,
					    "  ANCS: %d (%s)\n",
					    ancs_val,
					    ancs_val ?
					    "Audio Not CS" : "Normal");
				} else {
					device_printf(dev,
					    "  ANCS: (cannot read)\n");
					ancs_val = 0;
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\SMD0", &aval);
				if (ACPI_SUCCESS(ast)) {
					device_printf(dev,
					    "  SMD0: %d (%s)\n",
					    (int)aval,
					    aval == 0 ? "Disabled" :
					    aval == 1 ? "PCI mode" :
					    aval == 2 ? "ACPI mode" :
					    "Unknown");
				} else {
					device_printf(dev,
					    "  SMD0: (cannot read)\n");
				}

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\SB10", &aval);
				if (ACPI_SUCCESS(ast))
					device_printf(dev,
					    "  SB10: 0x%08x (SDMA BAR)\n",
					    aval);

				ast = acpi_GetInteger(ACPI_ROOT_OBJECT,
				    "\\ADB0", &aval);
				if (ACPI_SUCCESS(ast))
					device_printf(dev,
					    "  ADB0: 0x%08x (ADSP BAR0)\n",
					    aval);
			}

			bus_space_unmap(mt, gnvs_handle, PCH_GNVS_SIZE);

			/* Provide clear diagnosis */
			if (osys_val < 0x07DC) {
				device_printf(dev, "\n");
				device_printf(dev,
				    "*** ROOT CAUSE: OSYS=0x%04x < 0x07DC ***\n",
				    osys_val);
				device_printf(dev,
				    "FreeBSD reports Windows 7 to ACPI.\n");
				device_printf(dev,
				    "The DSDT requires OSYS >= 0x07DC (Win8+) to\n");
				device_printf(dev,
				    "initialize the LPSS fabric and enable BAR0.\n");
				device_printf(dev,
				    "\nFIX: Add to /boot/loader.conf:\n");
				device_printf(dev,
				    "  hw.acpi.install_interface=\"Windows 2012\"\n");
				device_printf(dev,
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
			device_printf(dev, "=== FD Register ===\n");
			device_printf(dev, "  FD current: 0x%08x\n", fd);
			device_printf(dev,
			    "    ADSD (bit 1): %s\n",
			    (fd & PCH_FD_ADSD) ? "ADSP Disabled" :
			    "ADSP Enabled");
			device_printf(dev,
			    "    SMBD (bit 3): %s\n",
			    (fd & PCH_FD_SMBD) ? "SMBus Disabled" :
			    "SMBus Enabled");
			device_printf(dev,
			    "    HDAD (bit 4): %s\n",
			    (fd & PCH_FD_HDAD) ? "HDA Disabled" :
			    "HDA Enabled");

			/* Restore BIOS default if corrupted by previous
			 * wrong-bit-offset writes */
			if (fd != 0x00368011) {
				device_printf(dev,
				    "  Restoring FD to BIOS default 0x00368011\n");
				bus_space_write_4(mt, rh, PCH_RCBA_FD,
				    0x00368011);
				DELAY(10000);
				fd = bus_space_read_4(mt, rh, PCH_RCBA_FD);
				device_printf(dev,
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

		device_printf(dev, "=== LPSS Private Config (BAR1) ===\n");
		/* Dump interesting non-zero regions */
		for (i = 0x00; i < 0x1000; i += 4) {
			v = bus_read_4(sc->shim_res, i);
			/* Print if non-zero or at known-interesting offset */
			if (v != 0 || i == 0x10 || i == 0x84 ||
			    i == 0xA0 || i == 0xA8 || i == 0x200 ||
			    i == 0x800 || i == 0x808 || i == 0x810 ||
			    i == 0x900)
				device_printf(dev,
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

		device_printf(dev,
		    "=== LPSS Device Private Configs ===\n");
		for (d = 0; d < 3; d++) {
			map_err = bus_space_map(mt, lpss_devs[d].addr,
			    0x300, 0, &h);
			if (map_err != 0) {
				device_printf(dev, "  %s: map failed\n",
				    lpss_devs[d].name);
				continue;
			}

			device_printf(dev,
			    "  %s (0x%08x):\n", lpss_devs[d].name,
			    lpss_devs[d].addr);
			device_printf(dev,
			    "    VID/DID: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x00));
			device_printf(dev,
			    "    CMD/STS: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x04));
			device_printf(dev,
			    "    BAR0:    0x%08x\n",
			    bus_space_read_4(mt, h, 0x10));
			device_printf(dev,
			    "    PMCSR:   0x%08x\n",
			    bus_space_read_4(mt, h, 0x84));
			device_printf(dev,
			    "    [0x200]: 0x%08x (PMCTRL)\n",
			    bus_space_read_4(mt, h, 0x200));
			device_printf(dev,
			    "    [0x204]: 0x%08x\n",
			    bus_space_read_4(mt, h, 0x204));
			device_printf(dev,
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

		device_printf(dev,
		    "=== BAR0 Memory Decode Test ===\n");

		/* Test 1: Is SDMA BAR0 (0xFE101000) accessible? */
		map_err = bus_space_map(mt, 0xFE101000, 0x1000, 0, &h);
		if (map_err == 0) {
			v = bus_space_read_4(mt, h, 0);
			device_printf(dev,
			    "  SDMA BAR0 (0xFE101000)[0]: 0x%08x %s\n",
			    v, v != 0xFFFFFFFF ? "ALIVE" : "DEAD");
			bus_space_unmap(mt, h, 0x1000);
		}

		/* Test 2: Probe the 0xFE100000-0xFE10F000 range
		 * to find what's decoded */
		device_printf(dev,
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
					device_printf(dev,
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
		device_printf(dev,
		    "=== BAR0 Remap Experiment ===\n");
		{
			uint32_t old_bar0, new_bar0;

			old_bar0 = bus_read_4(sc->shim_res, 0x10);
			device_printf(dev,
			    "  Current BAR0 in privconfig: 0x%08x\n",
			    old_bar0);

			/* Try remapping to unused LPSS space */
			new_bar0 = 0xFE108000;
			device_printf(dev,
			    "  Writing BAR0 = 0x%08x...\n", new_bar0);
			bus_write_4(sc->shim_res, 0x10, new_bar0);
			DELAY(10000);

			v = bus_read_4(sc->shim_res, 0x10);
			device_printf(dev,
			    "  BAR0 readback: 0x%08x\n", v);

			if (v == new_bar0) {
				/* Try reading the new address */
				map_err = bus_space_map(mt, new_bar0,
				    0x1000, 0, &h);
				if (map_err == 0) {
					uint32_t dsp_v;
					dsp_v = bus_space_read_4(mt, h, 0);
					device_printf(dev,
					    "  New BAR0[0]: 0x%08x%s\n",
					    dsp_v,
					    dsp_v != 0xFFFFFFFF ?
					    " *** ALIVE! ***" : " dead");

					if (dsp_v != 0xFFFFFFFF) {
						device_printf(dev,
						    "  [0x00]: 0x%08x\n",
						    dsp_v);
						device_printf(dev,
						    "  [0x04]: 0x%08x\n",
						    bus_space_read_4(mt,
						    h, 0x04));
						device_printf(dev,
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
			device_printf(dev,
			    "  Restored BAR0: 0x%08x\n",
			    bus_read_4(sc->shim_res, 0x10));
		}

		/* Test 4: Try I2C0 D0 transition and BAR0 check */
		device_printf(dev,
		    "=== I2C0 D0 Transition Test ===\n");
		map_err = bus_space_map(mt, 0xFE104000, 0x300, 0, &h);
		if (map_err == 0) {
			uint32_t pmcsr;

			pmcsr = bus_space_read_4(mt, h, 0x84);
			device_printf(dev,
			    "  I2C0 PMCSR: 0x%08x (D%d)\n",
			    pmcsr, pmcsr & 3);

			/* Force D0 */
			if ((pmcsr & 3) != 0) {
				bus_space_write_4(mt, h, 0x84,
				    pmcsr & ~3);
				DELAY(50000);
				pmcsr = bus_space_read_4(mt, h, 0x84);
				device_printf(dev,
				    "  I2C0 PMCSR after D0: 0x%08x\n",
				    pmcsr);
			}
			bus_space_unmap(mt, h, 0x300);

			/* Check I2C0 BAR0 */
			map_err = bus_space_map(mt, 0xFE103000,
			    0x100, 0, &h);
			if (map_err == 0) {
				v = bus_space_read_4(mt, h, 0xFC);
				device_printf(dev,
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
		device_printf(dev,
		    "BAR0 final: %s\n",
		    bar0_ok ? "*** ACCESSIBLE! ***" : "still dead");

		if (bar0_ok)
			goto dsp_init;
	}

	/* IOBP already called in Phase 2.5. If BAR0 is still dead,
	 * try re-doing power-up now that IOBP has been configured. */
	if (!bar0_ok) {
		device_printf(dev, "BAR0 dead after IOBP+power-up, trying power cycle again...\n");
		sst_wpt_power_up(sc);
		bar0_ok = sst_test_bar0(sc);
		device_printf(dev, "BAR0 after 2nd power-up: %s\n",
		    bar0_ok ? "*** ACCESSIBLE! ***" : "still dead");
	}

	/* ---- Phase 8: I2C0 Codec Probe ---- */
	/* Transition I2C0 to D0 first, then probe codec */
	{
		bus_space_tag_t mt = X86_BUS_SPACE_MEM;
		bus_space_handle_t h;
		int map_err;

		device_printf(dev,
		    "=== Phase 8: I2C0 Codec Probe ===\n");
		map_err = bus_space_map(mt, SST_I2C0_PRIV_BASE,
		    0x300, 0, &h);
		if (map_err == 0) {
			uint32_t pmcsr;

			pmcsr = bus_space_read_4(mt, h, 0x84);
			device_printf(dev,
			    "  I2C0 PMCSR: 0x%08x (D%d)\n",
			    pmcsr, pmcsr & 3);

			/* Force D0 */
			if ((pmcsr & 3) != 0) {
				bus_space_write_4(mt, h, 0x84,
				    pmcsr & ~3);
				DELAY(50000);
				pmcsr = bus_space_read_4(mt, h, 0x84);
				device_printf(dev,
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

		device_printf(dev, "\n=== SUMMARY ===\n");
		device_printf(dev, "BAR0 (DSP MMIO): %s\n",
		    sst_test_bar0(sc) ? "ACCESSIBLE" : "DEAD");
		device_printf(dev, "HDA controller: %s\n",
		    hda_vid != 0xFFFFFFFF ? "ENABLED" : "ABSENT");
		if (hda_vid != 0xFFFFFFFF) {
			device_printf(dev,
			    "  VID/DID: 0x%08x at PCI 0:%x.%d\n",
			    hda_vid, PCH_HDA_DEV, PCH_HDA_FUNC);
		}
		device_printf(dev,
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
		device_printf(dev, "  After de-assert RST: CSR=0x%08x "
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

		device_printf(dev, "SHIM configured (catpt boot sequence)\n");
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
		device_printf(dev, "Pre-FW load: CSR=0x%08x (STALL=%d RST=%d)\n",
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
		device_printf(dev, "PCI Command register: 0x%04x\n", cmd);

		cmd |= (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);
		pci_write_config(dev, PCIR_COMMAND, cmd, 2);
		cmd = pci_read_config(dev, PCIR_COMMAND, 2);
		device_printf(dev, "PCI Command after setup: 0x%04x\n", cmd);
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
	device_printf(dev, "BAR0: 0x%lx (size: 0x%lx)\n",
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
	device_printf(dev, "BAR1: 0x%lx (size: 0x%lx)\n",
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
			device_printf(dev,
			    "PCI CMD lost after D3 cycle (0x%04x), restoring\n",
			    cmd);
			cmd |= (PCIM_CMD_MEMEN | PCIM_CMD_BUSMASTEREN);
			pci_write_config(dev, PCIR_COMMAND, cmd, 2);
		}
	}

	/* Now try SRAM enable after DSP power-up */
	error = sst_enable_sram(sc);
	if (error != 0) {
		device_printf(dev, "SRAM enable returned %d, trying direct...\n",
		    error);
		sst_enable_sram_direct(dev);
	}

	/* SRAM Sanitize: dummy reads to prevent byte loss */
	sst_sram_sanitize(sc);

	/* Test BAR0 */
	bar0_ok = sst_test_bar0(sc);
	device_printf(dev, "BAR0 test: %s\n",
	    bar0_ok ? "ACCESSIBLE" : "DEAD (0xFFFFFFFF)");

	/* Allocate IRQ */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
	    &sc->irq_rid, RF_SHAREABLE | RF_ACTIVE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Failed to allocate IRQ\n");
		/* Don't fail yet, maybe we can run without IRQ for init test */
	} else {
		device_printf(dev, "IRQ assigned\n");
	}

	if (bar0_ok) {
		/* Probe DSP memory regions */
		device_printf(dev, "DSP Memory Layout (PCI):\n");
		device_printf(dev, "  IRAM  (0x%05x): 0x%08x\n",
		    SST_IRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_IRAM_OFFSET));
		device_printf(dev, "  DRAM  (0x%05x): 0x%08x\n",
		    SST_DRAM_OFFSET,
		    bus_read_4(sc->mem_res, SST_DRAM_OFFSET));
		device_printf(dev, "  SHIM  (0x%05x): 0x%08x\n",
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
			device_printf(dev,
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

		/* RT286 codec initialization and output enable */
		if (sst_codec_init(sc) == 0) {
			sst_codec_enable_speaker(sc);
			sst_codec_enable_headphone(sc);
		}

		sc->attached = true;
		sc->state = SST_STATE_ATTACHED;
		device_printf(dev, "Intel SST DSP attached successfully\n");
	} else {
		device_printf(dev, "PCI Attach: BAR0 still dead. Hardware is tough.\n");
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

	device_printf(dev, "Suspending...\n");

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

	device_printf(dev, "Suspended\n");
	return (0);
}

static int
sst_acpi_resume(device_t dev)
{
	struct sst_softc *sc = device_get_softc(dev);
	int error;

	device_printf(dev, "Resuming...\n");

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
	device_printf(dev, "Resumed\n");
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

