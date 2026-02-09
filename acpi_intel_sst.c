/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel Smart Sound Technology (SST) ACPI Driver for FreeBSD
 * Target: Intel Broadwell-U (INT3438)
 *
 * Phase 1-5: ACPI, DSP, Firmware, IPC, I2S/SSP, DMA, PCM/sound(4)
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

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include "acpi_intel_sst.h"

/* Driver version for debugging */
#define SST_DRV_VERSION "0.5.0"

/* Forward declarations */
static int sst_acpi_probe(device_t dev);
static int sst_acpi_attach(device_t dev);
static int sst_acpi_detach(device_t dev);
static int sst_acpi_suspend(device_t dev);
static int sst_acpi_resume(device_t dev);
static void sst_intr(void *arg);

/* Helper functions */
static void sst_reset(struct sst_softc *sc);
static void sst_init(struct sst_softc *sc);

/* Supported ACPI IDs */
static char *sst_ids[] = {
	SST_ACPI_ID_BDW,
	SST_ACPI_ID_HSW,
	NULL
};

/**
 * @brief Main interrupt handler
 */
static void
sst_intr(void *arg)
{
	struct sst_softc *sc = arg;
	uint32_t pisr;

	/* Read platform interrupt status */
	pisr = sst_shim_read(sc, SST_SHIM_PISR);

	if (pisr == 0 || pisr == SST_INVALID_REG_VALUE)
		return;

	/* Handle IPC interrupts */
	sst_ipc_intr(sc);

	/* Handle DMA interrupts */
	sst_dma_intr(sc);

	/* Clear platform interrupt status */
	sst_shim_write(sc, SST_SHIM_PISR, pisr);
}

/**
 * @brief Resets the DSP core.
 */
static void
sst_reset(struct sst_softc *sc)
{
	uint32_t csr;

	device_printf(sc->dev, "Resetting DSP...\n");

	/* 1. Assert Reset and Stall */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	csr |= (SST_CSR_RST | SST_CSR_STALL);
	sst_shim_write(sc, SST_SHIM_CSR, csr);

	/* Wait for reset to propagate */
	DELAY(SST_RESET_DELAY_US);

	device_printf(sc->dev, "DSP in Reset/Stall state. CSR: 0x%08x\n",
		      sst_shim_read(sc, SST_SHIM_CSR));
}

/**
 * @brief Initialize DSP hardware
 */
static void
sst_init(struct sst_softc *sc)
{
	/* Mask all interrupts initially */
	sst_shim_write(sc, SST_SHIM_IMRX, SST_IPC_BUSY | SST_IPC_DONE);
	sst_shim_write(sc, SST_SHIM_IMRD, SST_IPC_BUSY | SST_IPC_DONE);

	sst_reset(sc);
}

/**
 * @brief Probes for the Intel SST device in ACPI tree.
 */
static int
sst_acpi_probe(device_t dev)
{
	int rv;

	if (acpi_disabled("sst"))
		return (ENXIO);

	rv = ACPI_ID_PROBE(device_get_parent(dev), dev, sst_ids, NULL);
	if (rv <= 0) {
		device_set_desc(dev, "Intel Broadwell-U Audio DSP (SST)");
	}

	return (rv);
}

/**
 * @brief Attaches the driver to the device.
 */
static int
sst_acpi_attach(device_t dev)
{
	struct sst_softc *sc;
	uint32_t csr, ipcx;
	int error = 0;

	sc = device_get_softc(dev);
	sc->dev = dev;
	sc->handle = acpi_get_handle(dev);
	sc->mem_res = NULL;
	sc->shim_res = NULL;
	sc->irq_res = NULL;
	sc->irq_cookie = NULL;
	sc->attached = false;
	sc->state = SST_STATE_NONE;

	/* Initialize mutex */
	mtx_init(&sc->sc_mtx, "sst_sc", NULL, MTX_DEF);

	device_printf(dev, "Intel SST Driver v%s loading\n", SST_DRV_VERSION);

	/* 1. Enable Power Resource (_PS0) if available */
	if (ACPI_FAILURE(acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0))) {
		device_printf(dev, "Warning: Failed to set power state to D0\n");
	}

	/* 2. Allocate Memory Resources (MMIO) */
	/* BAR0: DSP Memory (IRAM/DRAM) */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					     &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate DSP memory resource\n");
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "DSP Memory: 0x%lx, Size: 0x%lx\n",
		      rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* BAR1: PCI Extended Config (for power gating control) */
	sc->shim_rid = 1;
	sc->shim_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					      &sc->shim_rid, RF_ACTIVE);
	if (sc->shim_res == NULL) {
		device_printf(dev, "Failed to allocate PCI config resource\n");
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "PCI Config: 0x%lx, Size: 0x%lx\n",
		      rman_get_start(sc->shim_res), rman_get_size(sc->shim_res));

	/*
	 * 2.1 Access PCI config space via BAR1 (memory-mapped PCI config)
	 * BAR1 at 0xfe100000 is a mirror of the PCI configuration space.
	 * pci_read_config() doesn't work for ACPI devices.
	 */
	{
		uint32_t cmd, vdrtctl0, pmcsr;
		uint8_t cap_ptr;
		int pm_offset = 0;

		/* Dump PCI config space via BAR1 (shim_res) */
		device_printf(dev, "PCI Config Space (via BAR1 mirror):\n");
		device_printf(dev, "  DevID/VenID: 0x%08x\n",
		    bus_read_4(sc->shim_res, 0x00));
		cmd = bus_read_4(sc->shim_res, 0x04);
		device_printf(dev, "  Status/Cmd:  0x%08x\n", cmd);
		device_printf(dev, "  Class/Rev:   0x%08x\n",
		    bus_read_4(sc->shim_res, 0x08));
		device_printf(dev, "  PCI BAR0:    0x%08x\n",
		    bus_read_4(sc->shim_res, 0x10));
		device_printf(dev, "  PCI BAR1:    0x%08x\n",
		    bus_read_4(sc->shim_res, 0x14));

		/* Find PM capability */
		cap_ptr = bus_read_1(sc->shim_res, 0x34);
		device_printf(dev, "  Cap Ptr:     0x%02x\n", cap_ptr);

		while (cap_ptr >= 0x40 && cap_ptr != 0xFF) {
			uint8_t cap_id = bus_read_1(sc->shim_res, cap_ptr);
			device_printf(dev, "  Cap@0x%02x: ID=0x%02x\n",
			    cap_ptr, cap_id);
			if (cap_id == 0x01) {
				pm_offset = cap_ptr;
				break;
			}
			cap_ptr = bus_read_1(sc->shim_res, cap_ptr + 1);
		}

		if (pm_offset > 0) {
			/* Read PMCSR (PM Control/Status) at offset +4 */
			pmcsr = bus_read_4(sc->shim_res, pm_offset + 4);
			device_printf(dev, "  PMCSR:       0x%08x (D%d)\n",
			    pmcsr, pmcsr & 0x03);

			/* Set power state to D0 if not already */
			if ((pmcsr & 0x03) != 0) {
				device_printf(dev,
				    "Setting power state to D0...\n");
				pmcsr &= ~0x03;  /* Clear D-state bits */
				bus_write_4(sc->shim_res, pm_offset + 4, pmcsr);

				/*
				 * Wait 100ms for D3cold->D0 transition
				 * PCI spec requires at least 10ms
				 */
				DELAY(100000);

				pmcsr = bus_read_4(sc->shim_res, pm_offset + 4);
				device_printf(dev,
				    "  PMCSR after: 0x%08x (D%d)\n",
				    pmcsr, pmcsr & 0x03);
			}
		}

		/*
		 * After D3->D0 transition, re-enable memory space
		 * and bus mastering (may have been disabled in D3)
		 */
		cmd = bus_read_4(sc->shim_res, 0x04);
		device_printf(dev, "  Cmd after D0: 0x%08x\n", cmd);
		if ((cmd & 0x06) != 0x06) {
			cmd |= 0x06;  /* Memory Space + Bus Master */
			bus_write_4(sc->shim_res, 0x04, cmd);
			DELAY(1000);
			cmd = bus_read_4(sc->shim_res, 0x04);
			device_printf(dev, "  Cmd enabled: 0x%08x\n", cmd);
		}

		/*
		 * Now disable Intel-specific power gating via VDRTCTL0
		 * This is at offset 0xA0 in PCI config space
		 */
		vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
		device_printf(dev, "  VDRTCTL0:    0x%08x\n", vdrtctl0);

		/*
		 * Disable D3 power gating (catpt driver sequence):
		 * - Set DSRAMPGE bits 0-7 = 0xFF (enable all SRAM)
		 * - Set D3SRAMPGD bit 8 (disable D3 SRAM power gate)
		 * - Set D3PGD bit 16 (disable D3 power gate)
		 */
		vdrtctl0 |= SST_VDRTCTL0_DSRAMPGE_MASK;  /* Bits 0-7 */
		vdrtctl0 |= SST_VDRTCTL0_D3SRAMPGD;      /* Bit 8 */
		vdrtctl0 |= SST_VDRTCTL0_D3PGD;          /* Bit 16 */
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);

		/* Wait for DSP power up */
		DELAY(100000);  /* 100ms */

		/* Read power control after */
		vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
		device_printf(dev, "  VDRTCTL0 after: 0x%08x\n", vdrtctl0);

		/* Read VDRTCTL2 for clock gating status */
		device_printf(dev, "  VDRTCTL2:    0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2));

		/* Final command register check */
		cmd = bus_read_4(sc->shim_res, 0x04);
		device_printf(dev, "  Final Cmd:   0x%08x\n", cmd);

		/* Verify BARs are valid */
		device_printf(dev, "  Final BAR0:  0x%08x\n",
		    bus_read_4(sc->shim_res, 0x10));
		device_printf(dev, "  Final BAR1:  0x%08x\n",
		    bus_read_4(sc->shim_res, 0x14));
	}

	/* 2.2 Probe DSP memory layout */
	device_printf(dev, "Probing DSP memory layout:\n");
	device_printf(dev, "  BAR0+0x00000 (IRAM): 0x%08x\n",
	    bus_read_4(sc->mem_res, 0x00000));
	device_printf(dev, "  BAR0+0x80000 (DRAM): 0x%08x\n",
	    bus_read_4(sc->mem_res, 0x80000));
	device_printf(dev, "  BAR0+0xC0000 (SHIM): 0x%08x\n",
	    bus_read_4(sc->mem_res, 0xC0000));
	device_printf(dev, "  BAR0+0xE0000 (MBOX): 0x%08x\n",
	    bus_read_4(sc->mem_res, 0xE0000));
	if (sc->shim_res != NULL) {
		device_printf(dev, "  BAR1+0x00 (SHIM mirror?): 0x%08x\n",
		    bus_read_4(sc->shim_res, 0x00));
	}

	/* Try SHIM at BAR0 + 0xC0000 first */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);

	device_printf(dev, "SHIM Register Dump (BAR0+0x%x):\n", SST_SHIM_OFFSET);
	device_printf(dev, "  CSR : 0x%08x\n", csr);
	device_printf(dev, "  IPCX: 0x%08x\n", ipcx);
	device_printf(dev, "  PISR: 0x%08x\n", sst_shim_read(sc, SST_SHIM_PISR));
	device_printf(dev, "  IMRX: 0x%08x\n", sst_shim_read(sc, SST_SHIM_IMRX));

	/* Sanity check */
	if (csr == SST_INVALID_REG_VALUE) {
		device_printf(dev, "Error: Registers read 0xFFFFFFFF\n");
		error = ENXIO;
		goto fail;
	}

	/* 3. Allocate Interrupt Resource */
	sc->irq_rid = 0;
	sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
					     &sc->irq_rid,
					     RF_ACTIVE | RF_SHAREABLE);
	if (sc->irq_res == NULL) {
		device_printf(dev, "Warning: Failed to allocate IRQ resource\n");
	} else {
		device_printf(dev, "IRQ: %lu\n", rman_get_start(sc->irq_res));

		/* Setup interrupt handler */
		error = bus_setup_intr(dev, sc->irq_res,
				       INTR_TYPE_AV | INTR_MPSAFE,
				       NULL, sst_intr, sc, &sc->irq_cookie);
		if (error) {
			device_printf(dev, "Failed to setup interrupt: %d\n",
				      error);
			/* Non-fatal, continue */
			sc->irq_cookie = NULL;
		}
	}

	/* 4. Initialize IPC subsystem */
	error = sst_ipc_init(sc);
	if (error) {
		device_printf(dev, "Failed to initialize IPC\n");
		goto fail;
	}

	/* 5. Initialize Firmware subsystem */
	error = sst_fw_init(sc);
	if (error) {
		device_printf(dev, "Failed to initialize firmware subsystem\n");
		goto fail;
	}

	/* 6. Initialize Hardware */
	sst_init(sc);

	/* 7. Initialize DMA controller */
	error = sst_dma_init(sc);
	if (error) {
		device_printf(dev, "Failed to initialize DMA\n");
		goto fail;
	}

	/* 8. Initialize SSP (I2S) controller */
	error = sst_ssp_init(sc);
	if (error) {
		device_printf(dev, "Failed to initialize SSP\n");
		goto fail;
	}

	/* 9. Initialize PCM subsystem */
	error = sst_pcm_init(sc);
	if (error) {
		device_printf(dev, "Failed to initialize PCM\n");
		goto fail;
	}

	/* 10. Load and boot firmware */
	error = sst_fw_load(sc);
	if (error) {
		device_printf(dev, "Firmware load failed: %d\n", error);
		device_printf(dev, "Driver attached without firmware\n");
		/* Non-fatal - driver still usable for debugging */
		error = 0;
	} else {
		error = sst_fw_boot(sc);
		if (error) {
			device_printf(dev, "DSP boot failed: %d\n", error);
			/* Non-fatal */
			error = 0;
		} else {
			/* Get firmware version */
			sst_ipc_get_fw_version(sc, NULL);
		}
	}

	/* 11. Register PCM device with sound(4) */
	if (sc->fw.state == SST_FW_STATE_RUNNING) {
		error = sst_pcm_register(sc);
		if (error) {
			device_printf(dev, "PCM registration failed: %d\n",
			    error);
			/* Non-fatal */
			error = 0;
		}
	}

	/* 12. Initialize jack detection */
	error = sst_jack_init(sc);
	if (error) {
		device_printf(dev, "Jack detection init failed: %d\n", error);
		/* Non-fatal */
		error = 0;
	} else {
		sst_jack_sysctl_init(sc);
		sst_jack_enable(sc);
	}

	/* 13. Mark attached */
	sc->attached = true;
	sc->state = SST_STATE_ATTACHED;

	device_printf(dev, "Intel SST DSP attached successfully\n");

	return (0);

fail:
	sst_acpi_detach(dev);
	return (error);
}

/**
 * @brief Detaches the driver, releasing resources.
 */
static int
sst_acpi_detach(device_t dev)
{
	struct sst_softc *sc;

	sc = device_get_softc(dev);

	/* Cleanup jack detection */
	sst_jack_fini(sc);

	/* Cleanup PCM */
	sst_pcm_fini(sc);

	/* Cleanup SSP */
	sst_ssp_fini(sc);

	/* Cleanup DMA */
	sst_dma_fini(sc);

	/* Unload firmware */
	sst_fw_fini(sc);

	/* Cleanup IPC */
	sst_ipc_fini(sc);

	/* Teardown interrupt */
	if (sc->irq_cookie != NULL) {
		bus_teardown_intr(dev, sc->irq_res, sc->irq_cookie);
		sc->irq_cookie = NULL;
	}

	/* Release IRQ */
	if (sc->irq_res != NULL) {
		bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
		sc->irq_res = NULL;
	}

	/* Release SHIM Memory */
	if (sc->shim_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->shim_rid,
				     sc->shim_res);
		sc->shim_res = NULL;
	}

	/* Release DSP Memory */
	if (sc->mem_res != NULL) {
		bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid,
				     sc->mem_res);
		sc->mem_res = NULL;
	}

	/* Destroy mutex */
	if (mtx_initialized(&sc->sc_mtx))
		mtx_destroy(&sc->sc_mtx);

	/* Power down (_PS3) */
	if (sc->handle != NULL) {
		acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);
	}

	return (0);
}

/**
 * @brief Suspend handler
 */
static int
sst_acpi_suspend(device_t dev)
{
	struct sst_softc *sc;

	sc = device_get_softc(dev);

	device_printf(dev, "Suspending...\n");

	/* Reset DSP */
	sst_reset(sc);

	/* Power down */
	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3);

	sc->state = SST_STATE_SUSPENDED;

	return (0);
}

/**
 * @brief Resume handler
 */
static int
sst_acpi_resume(device_t dev)
{
	struct sst_softc *sc;
	int error;

	sc = device_get_softc(dev);

	device_printf(dev, "Resuming...\n");

	/* Power up */
	acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0);
	DELAY(10000); /* 10ms settle time */

	/* Reinitialize */
	sst_init(sc);

	/* Re-initialize IPC */
	sc->ipc.ready = false;
	sc->ipc.state = SST_IPC_STATE_IDLE;

	/* Reload and boot firmware */
	if (sc->fw.state == SST_FW_STATE_RUNNING) {
		sc->fw.state = SST_FW_STATE_LOADED;
		error = sst_fw_boot(sc);
		if (error) {
			device_printf(dev, "Resume: DSP boot failed\n");
			sc->state = SST_STATE_ERROR;
			return (error);
		}
	}

	sc->state = SST_STATE_RUNNING;

	return (0);
}

/* Device Method Table */
static device_method_t sst_methods[] = {
	/* Device interface */
	DEVMETHOD(device_probe,		sst_acpi_probe),
	DEVMETHOD(device_attach,	sst_acpi_attach),
	DEVMETHOD(device_detach,	sst_acpi_detach),
	DEVMETHOD(device_suspend,	sst_acpi_suspend),
	DEVMETHOD(device_resume,	sst_acpi_resume),

	DEVMETHOD_END
};

/* Driver Definition */
static driver_t sst_driver = {
	"acpi_intel_sst",
	sst_methods,
	sizeof(struct sst_softc),
};

/* Driver Module Request */
DRIVER_MODULE(acpi_intel_sst, acpi, sst_driver, 0, 0);
MODULE_DEPEND(acpi_intel_sst, acpi, 1, 1, 1);
MODULE_DEPEND(acpi_intel_sst, firmware, 1, 1, 1);
MODULE_VERSION(acpi_intel_sst, 5);
