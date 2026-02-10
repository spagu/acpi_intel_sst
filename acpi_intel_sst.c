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
#include <machine/cpufunc.h>	/* inl/outl for GPIO access */
#include <x86/bus.h>		/* X86_BUS_SPACE_MEM */

#include <contrib/dev/acpica/include/acpi.h>
#include <dev/acpica/acpivar.h>

#include <dev/pci/pcivar.h>
#include <dev/pci/pcireg.h>
#include <x86/pci_cfgreg.h>

#include "acpi_intel_sst.h"

/* Intel SST PCI IDs */
#define PCI_VENDOR_INTEL	0x8086
#define PCI_DEVICE_SST_BDW	0x9CB6	/* Wildcat Point-LP Smart Sound */

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

	/*
	 * 1. Power on sequence - try multiple ACPI methods
	 */

	/* Check device status via _STA */
	{
		ACPI_STATUS status;
		UINT32 sta;

		status = acpi_GetInteger(sc->handle, "_STA", &sta);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "ACPI _STA: 0x%x (%s%s%s%s)\n",
			    sta,
			    (sta & 0x01) ? "Present " : "",
			    (sta & 0x02) ? "Enabled " : "",
			    (sta & 0x04) ? "Shown " : "",
			    (sta & 0x08) ? "Functional" : "");
		}
	}

	/* Try _ON method first (some devices use this) */
	{
		ACPI_STATUS status;
		status = AcpiEvaluateObject(sc->handle, "_ON", NULL, NULL);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "Called _ON method successfully\n");
			DELAY(100000);  /* 100ms */
		}
	}

	/* Try _PS0 (D0 power state) */
	{
		ACPI_STATUS status;
		status = AcpiEvaluateObject(sc->handle, "_PS0", NULL, NULL);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "Called _PS0 method successfully\n");
			DELAY(100000);
		} else {
			device_printf(dev, "No _PS0 method (status=0x%x)\n",
			    status);
		}
	}

	/* Also try acpi_pwr_switch_consumer */
	if (ACPI_FAILURE(acpi_pwr_switch_consumer(sc->handle, ACPI_STATE_D0))) {
		device_printf(dev, "Warning: acpi_pwr_switch_consumer failed\n");
	}

	/* Try _INI method if available (ACPI device initialization) */
	{
		ACPI_STATUS status;
		status = AcpiEvaluateObject(sc->handle, "_INI", NULL, NULL);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "Called _INI method successfully\n");
			DELAY(100000);  /* 100ms */
		} else if (status != AE_NOT_FOUND) {
			device_printf(dev, "Warning: _INI method failed (0x%x)\n",
			    status);
		}
	}

	/*
	 * Try to find and power on the parent LPE device
	 * The SST DSP might be gated by the LPE (Low Power Engine)
	 */
	{
		ACPI_HANDLE parent;
		ACPI_STATUS status;

		status = AcpiGetParent(sc->handle, &parent);
		if (ACPI_SUCCESS(status) && parent != NULL) {
			char path[128];
			ACPI_BUFFER buf = { sizeof(path), path };

			AcpiGetName(parent, ACPI_FULL_PATHNAME, &buf);
			device_printf(dev, "Parent device: %s\n", path);

			/* Try to power on parent */
			AcpiEvaluateObject(parent, "_PS0", NULL, NULL);
		}
	}

	DELAY(200000);  /* 200ms total settle time */

	/*
	 * Try _DSM (Device Specific Method) if available
	 * Intel audio devices sometimes require _DSM calls
	 */
	{
		ACPI_OBJECT_LIST args;
		ACPI_OBJECT arg[4];
		ACPI_BUFFER result = { ACPI_ALLOCATE_BUFFER, NULL };
		ACPI_STATUS status;

		/* Intel Audio DSM UUID: a69f886e-6ceb-4594-a41f-7b5dce24c553 */
		static uint8_t intel_dsm_uuid[] = {
			0x6e, 0x88, 0x9f, 0xa6, 0xeb, 0x6c, 0x94, 0x45,
			0xa4, 0x1f, 0x7b, 0x5d, 0xce, 0x24, 0xc5, 0x53
		};

		arg[0].Type = ACPI_TYPE_BUFFER;
		arg[0].Buffer.Length = 16;
		arg[0].Buffer.Pointer = intel_dsm_uuid;

		arg[1].Type = ACPI_TYPE_INTEGER;
		arg[1].Integer.Value = 1;  /* Revision */

		arg[2].Type = ACPI_TYPE_INTEGER;
		arg[2].Integer.Value = 0;  /* Function 0: query */

		arg[3].Type = ACPI_TYPE_PACKAGE;
		arg[3].Package.Count = 0;
		arg[3].Package.Elements = NULL;

		args.Count = 4;
		args.Pointer = arg;

		status = AcpiEvaluateObject(sc->handle, "_DSM", &args, &result);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "_DSM query successful\n");
			if (result.Pointer)
				AcpiOsFree(result.Pointer);

			/* Try function 1: enable */
			arg[2].Integer.Value = 1;
			result.Length = ACPI_ALLOCATE_BUFFER;
			result.Pointer = NULL;

			status = AcpiEvaluateObject(sc->handle, "_DSM",
			    &args, &result);
			if (ACPI_SUCCESS(status)) {
				device_printf(dev,
				    "_DSM enable function called\n");
				if (result.Pointer)
					AcpiOsFree(result.Pointer);
			}
		} else {
			device_printf(dev, "No _DSM method\n");
		}
	}

	/*
	 * Check for power resource dependencies (_PR0)
	 */
	{
		ACPI_STATUS status;
		ACPI_BUFFER result = { ACPI_ALLOCATE_BUFFER, NULL };

		status = AcpiEvaluateObject(sc->handle, "_PR0", NULL, &result);
		if (ACPI_SUCCESS(status)) {
			ACPI_OBJECT *obj = result.Pointer;
			if (obj && obj->Type == ACPI_TYPE_PACKAGE) {
				device_printf(dev,
				    "_PR0 has %d power resources\n",
				    obj->Package.Count);

				/* Try to turn on each power resource */
				for (UINT32 i = 0; i < obj->Package.Count; i++) {
					ACPI_OBJECT *ref =
					    &obj->Package.Elements[i];
					if (ref->Type == ACPI_TYPE_LOCAL_REFERENCE) {
						AcpiEvaluateObject(
						    ref->Reference.Handle,
						    "_ON", NULL, NULL);
						device_printf(dev,
						    "  Turned on power resource %d\n", i);
					}
				}
				DELAY(100000);
			}
			if (result.Pointer)
				AcpiOsFree(result.Pointer);
		}
	}

	/*
	 * 1.5 Check PCH RCBA for Function Disable bits
	 * RCBA (Root Complex Base Address) contains Function Disable registers
	 * FD2 at RCBA+0x3428 may have ADSP disabled at PCH level
	 * RCBA is at LPC bridge (0:31:0) config offset 0xF0
	 */
	{
		uint32_t rcba_reg, rcba_base;
		uint32_t fd2;
		bus_space_tag_t mem_tag;
		bus_space_handle_t rcba_handle;
		int map_err;

		/* Read RCBA from LPC bridge config space */
		rcba_reg = pci_cfgregread(0, 0, 0x1F, 0, 0xF0, 4);
		rcba_base = rcba_reg & 0xFFFFC000;  /* Bits 14-31 */

		device_printf(dev, "PCH RCBA Check:\n");
		device_printf(dev, "  RCBA reg (0xF0): 0x%08x\n", rcba_reg);
		device_printf(dev, "  RCBA base: 0x%08x\n", rcba_base);
		device_printf(dev, "  RCBA enabled: %s\n",
		    (rcba_reg & 1) ? "yes" : "no");

		if (rcba_base != 0 && rcba_base != 0xFFFFC000 &&
		    (rcba_reg & 1)) {
			/*
			 * Map RCBA to read/write FD2
			 * FD2 (Function Disable 2) is at offset 0x3428
			 * ADSPD bit is bit 1 in FD2
			 */
			mem_tag = X86_BUS_SPACE_MEM;
			map_err = bus_space_map(mem_tag, rcba_base, 0x4000,
			    0, &rcba_handle);

			if (map_err == 0) {
				uint32_t fd1;

				/* Read FD1 at offset 0x3418 */
				fd1 = bus_space_read_4(mem_tag, rcba_handle,
				    0x3418);
				device_printf(dev, "  FD1 (0x3418): 0x%08x\n",
				    fd1);

				/* Read FD2 at offset 0x3428 */
				fd2 = bus_space_read_4(mem_tag, rcba_handle,
				    0x3428);
				device_printf(dev, "  FD2 (0x3428): 0x%08x\n",
				    fd2);

				/*
				 * Check if ADSPD (Audio DSP Disable) bit 1 is set
				 * If set, the ADSP is disabled at PCH level
				 */
				if (fd2 & (1 << 1)) {
					device_printf(dev,
					    "WARNING: ADSP is DISABLED in PCH FD2!\n");
					device_printf(dev,
					    "  Attempting to enable ADSP...\n");

					/* Clear ADSPD bit to enable ADSP */
					fd2 &= ~(1 << 1);
					bus_space_write_4(mem_tag, rcba_handle,
					    0x3428, fd2);
					DELAY(100000);  /* 100ms */

					fd2 = bus_space_read_4(mem_tag,
					    rcba_handle, 0x3428);
					device_printf(dev,
					    "  FD2 after: 0x%08x\n", fd2);
				} else {
					device_printf(dev,
					    "  ADSP is enabled in FD2\n");
				}

				bus_space_unmap(mem_tag, rcba_handle, 0x4000);
			} else {
				device_printf(dev,
				    "  Failed to map RCBA: %d\n", map_err);
			}
		}
	}

	/*
	 * 1.6 Direct GPIO control for audio power
	 * ACPI PAUD power resource uses GPIO 0x4C to control audio power
	 * GPIO base is at LPC bridge (0:0x1F:0) config offset 0x48, bits 7-15
	 * GPIO register = GPIO_BASE + 0x100 + (pin * 8)
	 * Bit 31 = output value
	 */
	{
		uint32_t lpc_gpio_reg, gpio_base;
		uint32_t gpio_addr, gpio_val;

		/* Read GPIO base from LPC bridge config space */
		/* LPC bridge is at bus 0, device 0x1F, function 0 */
		lpc_gpio_reg = pci_cfgregread(0, 0, 0x1F, 0, 0x48, 4);
		gpio_base = (lpc_gpio_reg >> 7) << 7;  /* bits 7-15, shifted */

		device_printf(dev, "GPIO Control:\n");
		device_printf(dev, "  LPC GPIO reg (0x48): 0x%08x\n",
		    lpc_gpio_reg);
		device_printf(dev, "  GPIO Base: 0x%08x\n", gpio_base);

		if (gpio_base != 0 && gpio_base != 0xFFFFFFFF) {
			/* GPIO 0x4C (76) register address */
			gpio_addr = gpio_base + 0x100 + (0x4C * 8);
			device_printf(dev, "  GPIO 0x4C addr: 0x%08x\n",
			    gpio_addr);

			/* Read current GPIO 0x4C state via I/O port */
			gpio_val = inl(gpio_addr);
			device_printf(dev, "  GPIO 0x4C value: 0x%08x "
			    "(bit31=%d)\n", gpio_val, (gpio_val >> 31) & 1);

			/* If bit 31 is not set, try to enable audio power */
			if ((gpio_val & (1U << 31)) == 0) {
				device_printf(dev,
				    "  Enabling audio power (GPIO 0x4C)...\n");
				gpio_val |= (1U << 31);
				outl(gpio_addr, gpio_val);
				DELAY(100000);  /* 100ms */

				/* Verify */
				gpio_val = inl(gpio_addr);
				device_printf(dev,
				    "  GPIO 0x4C after: 0x%08x (bit31=%d)\n",
				    gpio_val, (gpio_val >> 31) & 1);
			}
		}
	}

	/*
	 * 2. Allocate BAR1 FIRST for power control
	 * BAR0 must be allocated AFTER power-up to get valid mapping
	 */
	sc->shim_rid = 1;
	sc->shim_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					      &sc->shim_rid, RF_ACTIVE);
	if (sc->shim_res == NULL) {
		device_printf(dev, "Failed to allocate PCI config resource\n");
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "PCI Config (BAR1): 0x%lx, Size: 0x%lx\n",
		      rman_get_start(sc->shim_res), rman_get_size(sc->shim_res));

	/*
	 * Try to get PCI address via _ADR method
	 * Format: 0xDDDDFFFF where DDDD=device, FFFF=function
	 */
	{
		ACPI_STATUS status;
		UINT32 adr;

		status = acpi_GetInteger(sc->handle, "_ADR", &adr);
		if (ACPI_SUCCESS(status)) {
			device_printf(dev, "ACPI _ADR: 0x%x (dev=%u, func=%u)\n",
			    adr,
			    (adr >> 16) & 0xFFFF,
			    adr & 0xFFFF);
		}
	}

	/*
	 * WPT (Wildcat Point = Broadwell-U) Power-Up Sequence
	 * Based on Linux catpt driver dsp.c:catpt_dsp_power_up()
	 *
	 * CRITICAL: WPT has DIFFERENT bit positions than LPT (Haswell)!
	 * WPT VDRTCTL0:
	 *   Bit 0:  D3PGD (D3 Power Gate Disable)
	 *   Bit 1:  D3SRAMPGD (D3 SRAM Power Gate Disable)
	 *   Bits 2-11: ISRAMPGE (Instruction SRAM Power Gate Enable)
	 *   Bits 12-19: DSRAMPGE (Data SRAM Power Gate Enable)
	 * WPT VDRTCTL2:
	 *   Bit 1:  DCLCGE (Dynamic Clock Gating Enable)
	 *   Bit 10: DTCGE (Trunk Clock Gating Enable)
	 *   Bit 31: APLLSE (Audio PLL Shutdown Enable)
	 */
	{
		uint32_t vdrtctl0, vdrtctl2, pmcs;

		device_printf(dev, "=== WPT (Broadwell) Power-Up Sequence ===\n");

		/*
		 * Step 1: Disable clock gating FIRST
		 * Must disable before any other power operations
		 */
		vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
		device_printf(dev, "Step 1: VDRTCTL2 before: 0x%08x\n", vdrtctl2);

		/* Clear DCLCGE (bit 1) to disable dynamic clock gating */
		vdrtctl2 &= ~SST_VDRTCTL2_DCLCGE;
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
		DELAY(50000);  /* 50ms */

		/*
		 * Step 2: Set D0 power state via PMCS
		 */
		pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
		device_printf(dev, "Step 2: PMCS before: 0x%08x (D%d)\n",
		    pmcs, pmcs & SST_PMCS_PS_MASK);

		if ((pmcs & SST_PMCS_PS_MASK) != SST_PMCS_PS_D0) {
			pmcs = (pmcs & ~SST_PMCS_PS_MASK) | SST_PMCS_PS_D0;
			bus_write_4(sc->shim_res, SST_PCI_PMCS, pmcs);
			DELAY(100000);  /* 100ms for D3->D0 transition */
			pmcs = bus_read_4(sc->shim_res, SST_PCI_PMCS);
			device_printf(dev, "  PMCS after: 0x%08x (D%d)\n",
			    pmcs, pmcs & SST_PMCS_PS_MASK);
		}

		/*
		 * Step 3: Disable D3 power gating (WPT bits 0-1)
		 */
		vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
		device_printf(dev, "Step 3: VDRTCTL0 before: 0x%08x\n", vdrtctl0);

		/* Set D3PGD (bit 0) and D3SRAMPGD (bit 1) to disable D3 power gating */
		vdrtctl0 |= SST_WPT_VDRTCTL0_D3PGD;      /* Bit 0 */
		vdrtctl0 |= SST_WPT_VDRTCTL0_D3SRAMPGD;  /* Bit 1 */
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
		DELAY(50000);  /* 50ms */

		/*
		 * Step 4: Power on ALL SRAM banks
		 * CLEAR ISRAMPGE (bits 2-11) and DSRAMPGE (bits 12-19)
		 * Setting these bits = power gate ENABLED = SRAM OFF
		 * Clearing these bits = power gate DISABLED = SRAM ON
		 */
		vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
		vdrtctl0 &= ~SST_WPT_VDRTCTL0_ISRAMPGE_MASK;  /* Clear bits 2-11 */
		vdrtctl0 &= ~SST_WPT_VDRTCTL0_DSRAMPGE_MASK;  /* Clear bits 12-19 */
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
		DELAY(60);  /* 60us per catpt driver */

		vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
		device_printf(dev, "Step 4: VDRTCTL0 after SRAM on: 0x%08x\n",
		    vdrtctl0);

		/*
		 * Step 5: Enable Audio PLL (WPT: clear APLLSE bit 31 in VDRTCTL2)
		 */
		vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
		device_printf(dev, "Step 5: VDRTCTL2 before PLL: 0x%08x\n", vdrtctl2);

		if (vdrtctl2 & SST_VDRTCTL2_APLLSE_MASK) {
			vdrtctl2 &= ~SST_VDRTCTL2_APLLSE_MASK;  /* Clear bit 31 */
			bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
			DELAY(100000);  /* 100ms for PLL lock */
		}

		/*
		 * Step 6: Also clear DTCGE (bit 10) for trunk clock
		 */
		vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
		vdrtctl2 &= ~SST_VDRTCTL2_DTCGE;
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);
		DELAY(50000);

		vdrtctl2 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2);
		device_printf(dev, "Step 6: VDRTCTL2 after: 0x%08x\n", vdrtctl2);

		/*
		 * Step 7: Re-enable clock gating (per catpt driver)
		 */
		vdrtctl2 |= SST_VDRTCTL2_DCLCGE;
		bus_write_4(sc->shim_res, SST_PCI_VDRTCTL2, vdrtctl2);

		/* Final state dump */
		device_printf(dev, "=== Power-Up Complete ===\n");
		device_printf(dev, "  VDRTCTL0: 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0));
		device_printf(dev, "  VDRTCTL2: 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_VDRTCTL2));
		device_printf(dev, "  PMCS:     0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_PMCS));
	}

	/*
	 * Step 8: Dump additional PCI extended config registers
	 * These registers control IPC and DMA access
	 */
	{
		device_printf(dev, "Step 8: Extended PCI config dump...\n");
		device_printf(dev, "  IMC (0xE4): 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IMC));
		device_printf(dev, "  IMD (0xEC): 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IMD));
		device_printf(dev, "  IPCC (0xE0): 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IPCC));
		device_printf(dev, "  IPCD (0xE8): 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IPCD_REG));
	}

	/*
	 * Step 9: Clear Interrupt Masks via IMC register
	 * This enables IPC doorbell and completion interrupts
	 */
	{
		uint32_t imc;

		device_printf(dev, "Step 9: Clearing interrupt masks...\n");
		imc = bus_read_4(sc->shim_res, SST_PCI_IMC);
		device_printf(dev, "  IMC before: 0x%08x\n", imc);

		/* Write to IMC to clear doorbell and completion bits */
		bus_write_4(sc->shim_res, SST_PCI_IMC,
		    SST_IMC_IPCDB | SST_IMC_IPCCD);
		DELAY(1000);

		imc = bus_read_4(sc->shim_res, SST_PCI_IMC);
		device_printf(dev, "  IMC after: 0x%08x\n", imc);
	}

	/*
	 * Step 10: Set default IMD (Interrupt Mask Set) register
	 */
	{
		device_printf(dev, "Step 10: Setting IMD default...\n");
		device_printf(dev, "  IMD before: 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IMD));

		bus_write_4(sc->shim_res, SST_PCI_IMD, SST_IMD_DEFAULT);
		DELAY(1000);

		device_printf(dev, "  IMD after: 0x%08x\n",
		    bus_read_4(sc->shim_res, SST_PCI_IMD));
	}

	/*
	 * 2.1 Verify BAR1 is PCI config mirror and read diagnostic info
	 * BAR1 at 0xfe100000 can be used to access PCI config space
	 * Power-up was already done above with correct WPT sequence
	 */
	{
		uint32_t cmd;

		/* Dump PCI config space via BAR1 (shim_res) for diagnostics */
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

		/* Force enable Memory Space (bit 1) and Bus Master (bit 2) */
		{
			uint16_t cmd16 = bus_read_2(sc->shim_res, 0x04);
			if ((cmd16 & 0x06) != 0x06) {
				device_printf(dev, "Enabling Memory + BusMaster...\n");
				cmd16 |= 0x06;
				bus_write_2(sc->shim_res, 0x04, cmd16);
				DELAY(10000);
				device_printf(dev, "  Cmd after:  0x%04x\n",
				    bus_read_2(sc->shim_res, 0x04));
			}
		}

		/* Final BAR verification */
		device_printf(dev, "  Final BAR0:  0x%08x\n",
		    bus_read_4(sc->shim_res, 0x10));
	}

	/*
	 * 2.2 Test direct BAR0 access via manual mapping
	 * This helps diagnose if the issue is FreeBSD resource allocation
	 * or actual hardware inaccessibility
	 */
	{
		uint32_t bar0_addr = bus_read_4(sc->shim_res, 0x10);
		device_printf(dev, "Testing BAR0 @ 0x%08x directly...\n",
		    bar0_addr);

		/* BAR0 should be 0xFE000000 based on PCI config */
		if (bar0_addr != 0 && bar0_addr != 0xFFFFFFFF) {
			/*
			 * Try mapping a small region directly using
			 * bus_space operations to test if memory responds
			 */
			bus_space_tag_t bst = rman_get_bustag(sc->shim_res);
			bus_space_handle_t bsh;
			int map_err;

			map_err = bus_space_map(bst, bar0_addr & ~0xF,
			    0x1000, 0, &bsh);
			if (map_err == 0) {
				uint32_t test_val;

				/* Try reading SHIM CSR at 0xC0000 */
				device_printf(dev,
				    "Direct mapping successful!\n");
				/*
				 * Note: This maps only first 4KB,
				 * can't reach 0xC0000
				 * Just test first word
				 */
				test_val = bus_space_read_4(bst, bsh, 0);
				device_printf(dev,
				    "  Direct BAR0[0]: 0x%08x\n", test_val);

				bus_space_unmap(bst, bsh, 0x1000);
			} else {
				device_printf(dev,
				    "Direct mapping failed: %d\n", map_err);
			}
		}
	}

	/*
	 * 2.3 NOW allocate BAR0 after power-up is complete
	 * This ensures the memory mapping is valid
	 */
	sc->mem_rid = 0;
	sc->mem_res = bus_alloc_resource_any(dev, SYS_RES_MEMORY,
					     &sc->mem_rid, RF_ACTIVE);
	if (sc->mem_res == NULL) {
		device_printf(dev, "Failed to allocate DSP memory resource\n");
		error = ENXIO;
		goto fail;
	}

	device_printf(dev, "DSP Memory (BAR0): 0x%lx, Size: 0x%lx\n",
		      rman_get_start(sc->mem_res), rman_get_size(sc->mem_res));

	/* 2.3 Probe DSP memory layout */
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
