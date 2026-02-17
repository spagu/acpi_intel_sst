/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel SST PCH Platform Initialization
 * RCBA FD register, HDA/ADSP enable, IOBP sideband, PCI config dump,
 * PCH state dump.
 */

#include <sys/param.h>
#include <sys/kernel.h>
#include <sys/bus.h>
#include <sys/systm.h>

#include <machine/bus.h>
#include <machine/cpufunc.h>
#include <x86/bus.h>

#include <dev/pci/pcivar.h>
#include <x86/pci_cfgreg.h>

#include "acpi_intel_sst.h"

/* ================================================================
 * Dump PCI Config via BAR1
 * ================================================================ */

void
sst_dump_pci_config(struct sst_softc *sc)
{
	if (sc->shim_res == NULL)
		return;

	sst_dbg(sc, SST_DBG_TRACE, "PCI Config (BAR1 mirror):\n");
	sst_dbg(sc, SST_DBG_TRACE, "  VID/DID:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x00));
	sst_dbg(sc, SST_DBG_TRACE, "  STS/CMD:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x04));
	sst_dbg(sc, SST_DBG_TRACE, "  CLS/REV:  0x%08x\n",
	    bus_read_4(sc->shim_res, 0x08));
	sst_dbg(sc, SST_DBG_TRACE, "  BAR0:     0x%08x\n",
	    bus_read_4(sc->shim_res, 0x10));
	sst_dbg(sc, SST_DBG_TRACE, "  BAR1:     0x%08x\n",
	    bus_read_4(sc->shim_res, 0x14));
	sst_dbg(sc, SST_DBG_TRACE, "  PMCSR:    0x%08x\n",
	    bus_read_4(sc->shim_res, SST_PCI_PMCS));
	sst_dbg(sc, SST_DBG_TRACE, "  VDRTCTL0: 0x%08x\n",
	    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0));
	sst_dbg(sc, SST_DBG_TRACE, "  VDRTCTL2: 0x%08x\n",
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

int
sst_try_enable_hda(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base, fd, fd_new;
	uint32_t hda_vid;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	int error;

	sst_dbg(sc, SST_DBG_LIFE, "=== HDA Controller Enablement ===\n");

	/* Read RCBA from LPC bridge */
	rcba_reg = pci_cfgregread(0, PCH_LPC_BUS, PCH_LPC_DEV,
	    PCH_LPC_FUNC, PCH_LPC_RCBA_REG, 4);
	rcba_base = rcba_reg & PCH_RCBA_MASK;

	sst_dbg(sc, SST_DBG_OPS, "  RCBA reg: 0x%08x, base: 0x%08x, %s\n",
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
	sst_dbg(sc, SST_DBG_OPS, "  FD [0x3418]: 0x%08x\n", fd);
	sst_dbg(sc, SST_DBG_OPS, "    ADSD (bit 1): %s\n",
	    (fd & PCH_FD_ADSD) ? "ADSP Disabled" : "ADSP Enabled");
	sst_dbg(sc, SST_DBG_OPS, "    HDAD (bit 4): %s\n",
	    (fd & PCH_FD_HDAD) ? "HDA Disabled" : "HDA Enabled");

	/* Check current HDA state on PCI bus */
	hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
	    PCH_HDA_FUNC, 0x00, 4);
	sst_dbg(sc, SST_DBG_OPS, "  HDA PCI (0:%x.%d): 0x%08x%s\n",
	    PCH_HDA_DEV, PCH_HDA_FUNC, hda_vid,
	    hda_vid != 0xFFFFFFFF ? " (PRESENT)" : " (absent)");

	if (hda_vid != 0xFFFFFFFF) {
		sst_dbg(sc, SST_DBG_LIFE, "  HDA already present on PCI bus!\n");
		bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);
		return (0);
	}

	/* HDA is absent - try to enable it */
	if (fd & PCH_FD_HDAD) {
		sst_dbg(sc, SST_DBG_LIFE, "  Enabling HDA controller...\n");

		fd_new = fd & ~PCH_FD_HDAD;	/* Clear HDAD to enable HDA */

		sst_dbg(sc, SST_DBG_OPS, "  Writing FD: 0x%08x -> 0x%08x\n",
		    fd, fd_new);
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);	/* 100ms settle */

		/* Verify */
		fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		sst_dbg(sc, SST_DBG_OPS, "  FD after write: 0x%08x\n", fd);
		sst_dbg(sc, SST_DBG_OPS, "    HDAD now: %s\n",
		    (fd & PCH_FD_HDAD) ? "still disabled (write rejected)" :
		    "ENABLED!");

		/* Check if HDA appeared on PCI */
		DELAY(100000);	/* Additional settle time */
		hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
		    PCH_HDA_FUNC, 0x00, 4);
		sst_dbg(sc, SST_DBG_OPS, "  HDA PCI after enable: 0x%08x\n",
		    hda_vid);

		if (hda_vid != 0xFFFFFFFF) {
			uint32_t hda_class = pci_cfgregread(0, PCH_HDA_BUS,
			    PCH_HDA_DEV, PCH_HDA_FUNC, 0x08, 4);
			sst_dbg(sc, SST_DBG_LIFE,
			    "  *** HDA CONTROLLER APPEARED! ***\n");
			sst_dbg(sc, SST_DBG_LIFE,
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

				sst_dbg(sc, SST_DBG_OPS,
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
					sst_dbg(sc, SST_DBG_TRACE,
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
					sst_dbg(sc, SST_DBG_TRACE,
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
					sst_dbg(sc, SST_DBG_TRACE,
					    "  HDA out of reset:"
					    " GCTL=0x%08x (%s)\n", gctl,
					    (gctl & 1) ? "OK" : "FAILED");

					/* Wait for codec discovery */
					DELAY(500000); /* 500ms */

					/* Check STATESTS for codecs */
					statests = bus_space_read_2(ht,
					    hh, 0x0E);
					sst_dbg(sc, SST_DBG_TRACE,
					    "  HDA STATESTS: 0x%04x"
					    " (codecs: %s%s%s%s)\n",
					    statests,
					    (statests & 1) ? "SDI0 " : "",
					    (statests & 2) ? "SDI1 " : "",
					    (statests & 4) ? "SDI2 " : "",
					    statests == 0 ? "NONE" : "");

					gcap = bus_space_read_4(ht, hh,
					    0x00);
					sst_dbg(sc, SST_DBG_TRACE,
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
						sst_dbg(sc, SST_DBG_LIFE,
						    "  Triggering PCI"
						    " rescan...\n");
						bus_topo_lock();
						BUS_RESCAN(pci);
						bus_topo_unlock();
					}
				}
			}
		} else {
			sst_dbg(sc, SST_DBG_LIFE,
			    "  HDA did not appear."
			    " FD write may need reboot.\n");
		}
	} else {
		sst_dbg(sc, SST_DBG_OPS, "  HDAD bit already clear (HDA enabled)\n");
		sst_dbg(sc, SST_DBG_OPS,
		    "  But HDA not on PCI bus - trying toggle...\n");

		/* Try toggling: set HDAD, wait, clear it again */
		fd_new = fd | PCH_FD_HDAD;
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);
		sst_dbg(sc, SST_DBG_TRACE, "  FD after set HDAD: 0x%08x\n",
		    bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD));

		fd_new = fd & ~PCH_FD_HDAD;
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(200000);

		fd = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		sst_dbg(sc, SST_DBG_TRACE, "  FD after clear HDAD: 0x%08x\n", fd);

		hda_vid = pci_cfgregread(0, PCH_HDA_BUS, PCH_HDA_DEV,
		    PCH_HDA_FUNC, 0x00, 4);
		sst_dbg(sc, SST_DBG_OPS,
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
					sst_dbg(sc, SST_DBG_LIFE,
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
		sst_dbg(sc, SST_DBG_TRACE, "  FD2 [0x3428]: 0x%08x\n", fd2);
	}

	/* Scan relevant RCBA registers for debugging */
	{
		uint32_t v;

		v = bus_space_read_4(mem_tag, rcba_handle, 0x2030);
		sst_dbg(sc, SST_DBG_TRACE, "  RCBA [0x2030] LPSS: 0x%08x\n", v);

		v = bus_space_read_4(mem_tag, rcba_handle, 0x3410);
		sst_dbg(sc, SST_DBG_TRACE, "  RCBA [0x3410] CG:   0x%08x\n", v);
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

int __unused
sst_try_enable_adsp(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base, fd, fd_new, fd_verify;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	int error;

	sst_dbg(sc, SST_DBG_LIFE, "=== ADSP Enablement ===\n");

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
	sst_dbg(sc, SST_DBG_OPS, "  FD before: 0x%08x (ADSD=%d, HDAD=%d)\n",
	    fd, (fd & PCH_FD_ADSD) ? 1 : 0, (fd & PCH_FD_HDAD) ? 1 : 0);

	if (!(fd & PCH_FD_ADSD)) {
		sst_dbg(sc, SST_DBG_LIFE, "  ADSP already enabled\n");
		bus_space_unmap(mem_tag, rcba_handle, PCH_RCBA_SIZE);
		return (0);
	}

	/* Clear ADSD to enable ADSP, also set HDAD to disable HDA
	 * (only one of HDA/ADSP should be active) */
	fd_new = fd & ~PCH_FD_ADSD;	/* Enable ADSP */
	fd_new |= PCH_FD_HDAD;		/* Disable HDA (mutual exclusion) */

	sst_dbg(sc, SST_DBG_OPS, "  Writing FD: 0x%08x -> 0x%08x\n",
	    fd, fd_new);
	sst_dbg(sc, SST_DBG_OPS, "  (Clearing ADSD to enable ADSP, setting HDAD)\n");

	bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
	DELAY(100000);	/* 100ms settle */

	/* Verify write took effect */
	fd_verify = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
	sst_dbg(sc, SST_DBG_OPS, "  FD after:  0x%08x (ADSD=%d, HDAD=%d)\n",
	    fd_verify,
	    (fd_verify & PCH_FD_ADSD) ? 1 : 0,
	    (fd_verify & PCH_FD_HDAD) ? 1 : 0);

	if (fd_verify & PCH_FD_ADSD) {
		sst_dbg(sc, SST_DBG_OPS,
		    "  ADSD write REJECTED by hardware\n");
		/* Try just clearing ADSD without touching HDAD */
		fd_new = fd & ~PCH_FD_ADSD;
		sst_dbg(sc, SST_DBG_OPS,
		    "  Retry: writing FD 0x%08x (only clear ADSD)\n", fd_new);
		bus_space_write_4(mem_tag, rcba_handle, PCH_RCBA_FD, fd_new);
		DELAY(100000);
		fd_verify = bus_space_read_4(mem_tag, rcba_handle, PCH_RCBA_FD);
		sst_dbg(sc, SST_DBG_OPS, "  FD retry:  0x%08x (ADSD=%d)\n",
		    fd_verify, (fd_verify & PCH_FD_ADSD) ? 1 : 0);
	}

	if (!(fd_verify & PCH_FD_ADSD)) {
		sst_dbg(sc, SST_DBG_LIFE, "  *** ADSP ENABLED! ***\n");
		sst_dbg(sc, SST_DBG_LIFE, "  Waiting for hardware to stabilize...\n");
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

void
sst_probe_i2c_codec(struct sst_softc *sc)
{

	/*
	 * Legacy I2C codec probe - functionality moved to sst_codec.c.
	 * Full codec init is now done by sst_codec_init() / sst_codec_enable_speaker().
	 */
	sst_dbg(sc, SST_DBG_OPS,
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
void
sst_iobp_probe(struct sst_softc *sc)
{
	uint32_t rcba_reg, rcba_base;
	uint32_t pcicfgctl, pmctl, vdldat1, vdldat2;
	bus_space_tag_t mt;
	bus_space_handle_t rh;
	int error;

	sst_dbg(sc, SST_DBG_TRACE, "=== IOBP Sideband Probe ===\n");

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
	sst_dbg(sc, SST_DBG_TRACE, "  PCICFGCTL: 0x%08x (PCICD=%d ACPIIE=%d)\n",
	    pcicfgctl,
	    !!(pcicfgctl & ADSP_PCICFGCTL_PCICD),
	    !!(pcicfgctl & ADSP_PCICFGCTL_ACPIIE));

	error = sst_iobp_read(mt, rh, ADSP_IOBP_PMCTL, &pmctl);
	if (error == 0)
		sst_dbg(sc, SST_DBG_TRACE, "  PMCTL: 0x%08x %s\n",
		    pmctl, pmctl == 0x3f ? "(OK)" : "(NEED 0x3f!)");
	error = sst_iobp_read(mt, rh, ADSP_IOBP_VDLDAT1, &vdldat1);
	if (error == 0)
		sst_dbg(sc, SST_DBG_TRACE, "  VDLDAT1: 0x%08x\n", vdldat1);
	error = sst_iobp_read(mt, rh, ADSP_IOBP_VDLDAT2, &vdldat2);
	if (error == 0)
		sst_dbg(sc, SST_DBG_TRACE, "  VDLDAT2: 0x%08x\n", vdldat2);

	/* Set PMCTL if needed */
	if (pmctl != ADSP_PMCTL_VALUE) {
		sst_dbg(sc, SST_DBG_OPS, "  Setting PMCTL=0x3f\n");
		sst_iobp_write(mt, rh, ADSP_IOBP_PMCTL, ADSP_PMCTL_VALUE);
		DELAY(10000);
	}

	/* Clear PCICD to enable PCI config access */
	if (pcicfgctl & ADSP_PCICFGCTL_PCICD) {
		uint32_t new_cfg = pcicfgctl & ~(ADSP_PCICFGCTL_PCICD |
		    ADSP_PCICFGCTL_ACPIIE | 0x100);
		sst_dbg(sc, SST_DBG_OPS, "  Clearing PCICD: 0x%08x -> 0x%08x\n",
		    pcicfgctl, new_cfg);
		sst_iobp_write(mt, rh, ADSP_IOBP_PCICFGCTL, new_cfg);
		DELAY(100000);

		/* Verify PCI is visible */
		{
			uint32_t vid = pci_cfgregread(0, 0, 0x13, 0, 0x00, 4);
			sst_dbg(sc, SST_DBG_TRACE, "  PCI VID: 0x%08x %s\n",
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

		sst_dbg(sc, SST_DBG_OPS, "=== PCI Config Power Cycle ===\n");

		v0 = pci_cfgregread(0, 0, 0x13, 0, 0xA0, 4);
		v2 = pci_cfgregread(0, 0, 0x13, 0, 0xA8, 4);
		pmcs = pci_cfgregread(0, 0, 0x13, 0, 0x84, 4);
		sst_dbg(sc, SST_DBG_OPS,
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
		sst_dbg(sc, SST_DBG_OPS,
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
		sst_dbg(sc, SST_DBG_OPS,
		    "  Final: V0=0x%08x V2=0x%08x PMCS=0x%08x(D%d)\n",
		    v0, v2, pmcs, pmcs & 3);
		sst_dbg(sc, SST_DBG_OPS,
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
				sst_dbg(sc, SST_DBG_TRACE,
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
		sst_dbg(sc, SST_DBG_OPS,
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

void
sst_dump_pch_state(struct sst_softc *sc)
{
	uint32_t v, pmbase, gpiobase;
	bus_space_tag_t mem_tag;
	bus_space_handle_t rcba_handle;
	uint32_t rcba_reg, rcba_base;
	int error;

	sst_dbg(sc, SST_DBG_TRACE, "=== PCH State Dump ===\n");

	/* LPC bridge PCI config registers */
	sst_dbg(sc, SST_DBG_TRACE, "  LPC (0:1F.0) VID/DID: 0x%08x\n",
	    pci_cfgregread(0, 0, 0x1F, 0, 0x00, 4));

	pmbase = pci_cfgregread(0, 0, 0x1F, 0, 0x40, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC PMBASE [0x40]: 0x%08x (I/O: 0x%04x)\n",
	    pmbase, pmbase & 0xFF80);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x44, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC ACPI_CNTL [0x44]: 0x%08x%s\n",
	    v, (v & 0x80) ? " (ACPI enabled)" : " (ACPI disabled)");

	gpiobase = pci_cfgregread(0, 0, 0x1F, 0, 0x48, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC GPIOBASE [0x48]: 0x%08x\n", gpiobase);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x4C, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC GPIO_CNTL [0x4C]: 0x%08x\n", v);

	/* GCS (General Control and Status) */
	v = pci_cfgregread(0, 0, 0x1F, 0, 0xA0, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC GCS [0xA0]: 0x%08x\n", v);

	/* LPC Decode ranges */
	v = pci_cfgregread(0, 0, 0x1F, 0, 0x80, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC ILB_BASE [0x80]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x84, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC IOAPIC_BASE [0x84]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x88, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC SPI_BASE [0x88]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0x8C, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC MPHY_BASE [0x8C]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0xDC, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC BDE [0xDC]: 0x%08x\n", v);

	v = pci_cfgregread(0, 0, 0x1F, 0, 0xF0, 4);
	sst_dbg(sc, SST_DBG_TRACE, "  LPC RCBA [0xF0]: 0x%08x\n", v);

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
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x0038] SOFTRSTCTRL: 0x%08x\n", v);

			/* D31IP - Device 31 Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3100);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3100] D31IP: 0x%08x\n", v);

			/* D27IP - Device 27 (HDA) Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3110);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3110] D27IP: 0x%08x\n", v);

			/* D19IP - Device 19 (LPSS) Interrupt Pin */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3130);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3130] D19IP: 0x%08x\n", v);

			/* LPSS I/O fabric configuration */
			/* Check known LPSS-related RCBA offsets */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x2330);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x2330] SCC: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x2334);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x2334] SCC2: 0x%08x\n", v);

			/* Function Disable 2 */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3428);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3428] FD2: 0x%08x\n", v);

			/* Check DISPBDF and other SBI registers */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3420);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3420] DISPBDF: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3424);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3424] FD_LOCK: 0x%08x\n", v);

			/* Broadwell LPSS fabric specific registers */
			/* PMC function bar decode */
			int i;
			for (i = 0x2000; i <= 0x2050; i += 4) {
				v = bus_space_read_4(mem_tag, rcba_handle, i);
				if (v != 0)
					sst_dbg(sc, SST_DBG_TRACE,
					    "  RCBA [0x%04x]: 0x%08x\n",
					    i, v);
			}

			/* IOSFBCTL / IO sideband fabric */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3400);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3400] IOSFCTL: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3404);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3404] IOSFDAT: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3408);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3408] IOSFST: 0x%08x\n", v);

			/* CG - Clock Gating */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3410);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3410] CG: 0x%08x\n", v);

			v = bus_space_read_4(mem_tag, rcba_handle, 0x3414);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3414] CG2: 0x%08x\n", v);

			/* FD */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x3418);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  RCBA [0x3418] FD: 0x%08x\n", v);

			/* Check if there's an LPSS memory base register */
			v = bus_space_read_4(mem_tag, rcba_handle, 0x340C);
			sst_dbg(sc, SST_DBG_TRACE,
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
			sst_dbg(sc, SST_DBG_TRACE,
			    "  ACPI PM1_STS: 0x%04x, PM1_CNT: 0x%08x\n",
			    pm1_sts, pm1_cnt);

			/* GPE0 status */
			v = inl(pm_base + 0x20);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  ACPI GPE0_STS: 0x%08x\n", v);
		}
	}

	/* Direct-map BAR0 and try reading (bypass resource allocation) */
	{
		bus_space_handle_t bar0_handle;

		sst_dbg(sc, SST_DBG_TRACE,
		    "  Direct BAR0 map test (0xFE000000)...\n");
		error = bus_space_map(mem_tag, 0xFE000000, 0x1000, 0,
		    &bar0_handle);
		if (error == 0) {
			uint32_t v0 = bus_space_read_4(mem_tag, bar0_handle, 0);
			uint32_t v4 = bus_space_read_4(mem_tag, bar0_handle, 4);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Direct BAR0[0]=0x%08x [4]=0x%08x\n", v0, v4);

			/* Try SHIM offset */
			if (0xC0000 < 0x1000) {
				/* Can't reach SHIM with 4K map */
			}
			bus_space_unmap(mem_tag, bar0_handle, 0x1000);
		} else {
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Direct BAR0 map failed: %d\n", error);
		}

		/* Also try mapping just the SHIM region */
		sst_dbg(sc, SST_DBG_TRACE,
		    "  Direct SHIM map test (0xFE0C0000)...\n");
		error = bus_space_map(mem_tag, 0xFE0C0000, 0x1000, 0,
		    &bar0_handle);
		if (error == 0) {
			uint32_t v0 = bus_space_read_4(mem_tag, bar0_handle, 0);
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Direct SHIM[0]=0x%08x\n", v0);
			bus_space_unmap(mem_tag, bar0_handle, 0x1000);
		} else {
			sst_dbg(sc, SST_DBG_TRACE,
			    "  Direct SHIM map failed: %d\n", error);
		}
	}
}
