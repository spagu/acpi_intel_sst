/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Intel Smart Sound Technology (SST) ACPI Driver for FreeBSD
 * Target: Intel Broadwell-U (INT3438)
 *
 * Phase 1-2: ACPI Attachment, Resource Allocation & DSP Init
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
#define SST_DRV_VERSION "0.2.0"

/* Forward declarations */
static int sst_acpi_probe(device_t dev);
static int sst_acpi_attach(device_t dev);
static int sst_acpi_detach(device_t dev);

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
 * @brief Resets the DSP core.
 */
static void
sst_reset(struct sst_softc *sc)
{
    uint32_t csr;

    device_printf(sc->dev, "Resetting DSP...\n");

    /* 1. Assert Reset and Stall */
    csr = bus_read_4(sc->mem_res, SST_SHIM_CSR);
    csr |= (SST_CSR_RST | SST_CSR_STALL);
    bus_write_4(sc->mem_res, SST_SHIM_CSR, csr);

    /* Wait for reset to propagate */
    DELAY(1000); 

    /* 2. Release Stall (keep Reset asserted? Or release both? usually sequence is specific) */
    /* Linux haswell-dsp.c: 
       sst_dsp_shim_update_bits_unlocked(sst, SST_CSR, 
          SST_CSR_RST | SST_CSR_STALL, 
          SST_CSR_RST | SST_CSR_STALL); 
       return 0; -> This just puts it IN reset.
       
       To generic_boot(sst):
        // Set Reset | Stall
        // Clear Reset (keep Stall)
        // Load FW
        // Clear Stall -> Run
    */
    
    /* For now, we leave it in Reset/Stall state until we have FW loading logic. */
    device_printf(sc->dev, "DSP in Reset/Stall state. CSR: 0x%08x\n", 
                  bus_read_4(sc->mem_res, SST_SHIM_CSR));
}

static void
sst_init(struct sst_softc *sc)
{
    /* Mask all interrupts initially */
    bus_write_4(sc->mem_res, SST_SHIM_IMRX, 0xFFFF0000); // Mask High 16 bits? Or all? Usually set bit to mask.
    /* Check docs/Linux. Haswell: sst_dsp_shim_update_bits(..., SST_IMRX, SST_IMRX_BUSY | SST_IMRX_DONE, SST_IMRX_BUSY | SST_IMRX_DONE); 
       Masking usually means setting the bit.
    */
    bus_write_4(sc->mem_res, SST_SHIM_IMRX, SST_IPC_BUSY | SST_IPC_DONE);
    bus_write_4(sc->mem_res, SST_SHIM_IMRD, SST_IPC_BUSY | SST_IPC_DONE);

    sst_reset(sc);
}

/**
 * @brief Probes for the Intel SST device in ACPI tree.
 * 
 * @param dev The device structure.
 * @return BUS_PROBE_DEFAULT if found, ENXIO otherwise.
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
 * @brief Attaches the driver to the device, allocating resources.
 *
 * Uses goto-based cleanup pattern for proper resource management.
 *
 * @param dev The device structure.
 * @return 0 on success, error code otherwise.
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
    sc->attached = false;

    device_printf(dev, "Intel SST Driver v%s loading\n", SST_DRV_VERSION);

    /* 1. Enable Power Resource (_PS0) if available */
    if (ACPI_FAILURE(acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0))) {
        device_printf(dev, "Warning: Failed to set power state to D0\n");
        /* Continue - some devices might be always on */
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
    csr = bus_read_4(sc->mem_res, SST_SHIM_CSR);
    ipcx = bus_read_4(sc->mem_res, SST_SHIM_IPCX);

    device_printf(dev, "Register Dump:\n");
    device_printf(dev, "  CSR : 0x%08x\n", csr);
    device_printf(dev, "  IPCX: 0x%08x\n", ipcx);
    device_printf(dev, "  PISR: 0x%08x\n", bus_read_4(sc->mem_res, SST_SHIM_PISR));
    device_printf(dev, "  IMRX: 0x%08x\n", bus_read_4(sc->mem_res, SST_SHIM_IMRX));

    /* Sanity check - 0xFFFFFFFF indicates hardware issue */
    if (csr == SST_INVALID_REG_VALUE) {
        device_printf(dev, "Error: Registers read 0xFFFFFFFF - device may be "
                      "powered down or MMIO mapping invalid\n");
        error = ENXIO;
        goto fail;
    }

    /* 3. Allocate Interrupt Resource */
    sc->irq_rid = 0;
    sc->irq_res = bus_alloc_resource_any(dev, SYS_RES_IRQ,
                                         &sc->irq_rid, RF_ACTIVE | RF_SHAREABLE);
    if (sc->irq_res == NULL) {
        device_printf(dev, "Warning: Failed to allocate IRQ resource\n");
        /* Non-fatal in early development - polling mode fallback */
    } else {
        device_printf(dev, "IRQ: %lu\n", rman_get_start(sc->irq_res));
    }

    /* Initialize Hardware */
    sst_init(sc);

    /* 4. Mark attached */
    sc->attached = true;

    device_printf(dev, "Intel SST DSP attached successfully (Phase 1+2)\n");

    return (0);

fail:
    sst_acpi_detach(dev);
    return (error);
}

/**
 * @brief Detaches the driver, releasing resources.
 * 
 * @param dev The device structure.
 * @return 0 on success.
 */
static int
sst_acpi_detach(device_t dev)
{
    struct sst_softc *sc;

    sc = device_get_softc(dev);

    /* Release IRQ */
    if (sc->irq_res != NULL) {
        /* If we setup an interrupt handler, teardown here: bus_teardown_intr(...) */
        bus_release_resource(dev, SYS_RES_IRQ, sc->irq_rid, sc->irq_res);
        sc->irq_res = NULL;
    }

    /* Release Memory */
    if (sc->mem_res != NULL) {
        bus_release_resource(dev, SYS_RES_MEMORY, sc->mem_rid, sc->mem_res);
        sc->mem_res = NULL;
    }

    /* Power down (_PS3) */
    if (ACPI_FAILURE(acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D3))) {
        /* Ignore error on detach */
    }

    return (0);
}

/* Device Method Table */
static device_method_t sst_methods[] = {
    /* Device interface */
    DEVMETHOD(device_probe,     sst_acpi_probe),
    DEVMETHOD(device_attach,    sst_acpi_attach),
    DEVMETHOD(device_detach,    sst_acpi_detach),
    
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
