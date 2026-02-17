/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel SST DSP Power Management
 * Reset, init, ACPI D-state transitions, WPT power cycling.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/pci/pcivar.h>

#include "acpi_intel_sst.h"

void
sst_reset(struct sst_softc *sc)
{
	sst_dbg(sc, SST_DBG_LIFE, "Resetting DSP...\n");
	sst_dsp_stall(sc, true);
	sst_dsp_reset(sc, true);
	DELAY(SST_RESET_DELAY_US);
}

void
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

void
sst_acpi_power_up(struct sst_softc *sc)
{
	ACPI_STATUS status;
	UINT32 sta;

	/* Check _STA */
	status = acpi_GetInteger(sc->handle, "_STA", &sta);
	if (ACPI_SUCCESS(status)) {
		sst_dbg(sc, SST_DBG_LIFE, "ACPI _STA: 0x%x (%s%s%s%s)\n", sta,
		    (sta & 0x01) ? "Present " : "",
		    (sta & 0x02) ? "Enabled " : "",
		    (sta & 0x04) ? "Shown " : "",
		    (sta & 0x08) ? "Functional" : "");
	}

	/* _PS0 (D0 power state) */
	status = AcpiEvaluateObject(sc->handle, "_PS0", NULL, NULL);
	if (ACPI_SUCCESS(status)) {
		sst_dbg(sc, SST_DBG_LIFE, "Called _PS0 successfully\n");
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
			sst_dbg(sc, SST_DBG_LIFE, "_DSM query successful\n");
			if (result.Pointer)
				AcpiOsFree(result.Pointer);

			/* Function 1: enable */
			arg[2].Integer.Value = 1;
			result.Length = ACPI_ALLOCATE_BUFFER;
			result.Pointer = NULL;
			status = AcpiEvaluateObject(sc->handle, "_DSM",
			    &args, &result);
			if (ACPI_SUCCESS(status)) {
				sst_dbg(sc, SST_DBG_LIFE, "_DSM enable called\n");
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
						sst_dbg(sc, SST_DBG_LIFE,
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
			sst_dbg(sc, SST_DBG_LIFE, "PAUD._ON called\n");
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

	sst_dbg(sc, SST_DBG_LIFE, "  --- Power-Down (catpt-exact) ---\n");

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
	sst_dbg(sc, SST_DBG_OPS,
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
int
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
	sst_dbg(sc, SST_DBG_LIFE, "=== WPT Power Cycle (catpt-exact) ===\n");
	sst_dbg(sc, SST_DBG_LIFE,
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
	sst_dbg(sc, SST_DBG_OPS, "  After D0: PMCS=0x%08x(D%d) V0=0x%08x\n",
	    pmcs, pmcs & SST_PMCS_PS_MASK, vdrtctl0);

	/* 4. Set D3PGD + D3SRAMPGD (disable power gating) */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 |= (SST_WPT_VDRTCTL0_D3PGD |
	    SST_WPT_VDRTCTL0_D3SRAMPGD);
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	sst_dbg(sc, SST_DBG_OPS, "  After D3PGD: V0=0x%08x\n", vdrtctl0);

	/*
	 * 5a. Ungate DRAM: CLEAR DSRAMPGE bits (0 = gate disabled = ON)
	 *
	 * Linux catpt catpt_dsp_set_srampge() confirms:
	 *   power_up passes new=0 (CLEAR bits = SRAM powered ON)
	 *   power_down passes new=mask (SET bits = SRAM gated OFF)
	 * "Power Gate Enable": 1=gate enabled=off, 0=gate disabled=on
	 */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	sst_dbg(sc, SST_DBG_TRACE, "  Pre-DRAM PGE: V0=0x%08x\n", vdrtctl0);
	vdrtctl0 &= ~SST_WPT_VDRTCTL0_DSRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	sst_dbg(sc, SST_DBG_TRACE, "  Post-DRAM PGE: V0=0x%08x\n", vdrtctl0);

	/* 5b. Ungate IRAM: CLEAR ISRAMPGE bits (0 = gate disabled = ON) */
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	vdrtctl0 &= ~SST_WPT_VDRTCTL0_ISRAMPGE_MASK;
	bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
	DELAY(60);
	vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
	sst_dbg(sc, SST_DBG_TRACE, "  Post-IRAM PGE: V0=0x%08x\n", vdrtctl0);

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
	sst_dbg(sc, SST_DBG_OPS,
	    "  Final: VDRTCTL0=0x%08x VDRTCTL2=0x%08x PMCS=0x%08x(D%d)\n",
	    vdrtctl0, vdrtctl2, pmcs, pmcs & SST_PMCS_PS_MASK);
	sst_dbg(sc, SST_DBG_OPS,
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
		sst_dbg(sc, SST_DBG_TRACE,
		    "  BAR0 test: IRAM[0]=0x%08x SHIM[0]=0x%08x SRAM_CTRL=0x%08x\n",
		    iram0, shim0, sram_ctrl);
		if (iram0 != SST_INVALID_REG_VALUE ||
		    shim0 != SST_INVALID_REG_VALUE ||
		    sram_ctrl != SST_INVALID_REG_VALUE)
			sst_dbg(sc, SST_DBG_LIFE, "  *** BAR0 ALIVE! ***\n");
	}

	return (0);
}
