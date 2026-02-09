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

	/* 2. Allocate Memory Resource (MMIO) */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					     &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate memory resource\n");
		error = ENXIO;
		goto fail;
	}

	/* Validate MMIO region size */
	if (rman_get_size(sc->mem_res) < SST_MIN_MMIO_SIZE) {
		device_printf(dev, "MMIO region too small: 0x%lx (need 0x%x)\n",
			      rman_get_size(sc->mem_res), SST_MIN_MMIO_SIZE);
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "MMIO Base: 0x%lx, Size: 0x%lx\n",
		      rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* 2.1 Verify Hardware - Dump Registers */
	csr = sst_shim_read(sc, SST_SHIM_CSR);
	ipcx = sst_shim_read(sc, SST_SHIM_IPCX);

	device_printf(dev, "Register Dump:\n");
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

	/* 12. Mark attached */
	sc->attached = true;
	sc->state = SST_STATE_ATTACHED;

	device_printf(dev, "Intel SST DSP attached successfully (Phase 1-5)\n");

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

	/* Release Memory */
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
MODULE_DEPEND(acpi_intel_sst, sound, 1, 1, 1);
MODULE_VERSION(acpi_intel_sst, 5);
