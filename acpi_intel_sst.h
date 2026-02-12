/*-
 * SPDX-License-Identifier: BSD-2-Clause
 *
 * Intel Smart Sound Technology (SST) ACPI Driver for FreeBSD
 * Target: Intel Broadwell-U (INT3438)
 *
 * Copyright (c) 2026 FreeBSD Foundation
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR AND CONTRIBUTORS ``AS IS'' AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 */

#ifndef _ACPI_INTEL_SST_H_
#define _ACPI_INTEL_SST_H_

#include <sys/types.h>
#include <sys/bus.h>
#include <sys/rman.h>
#include <sys/lock.h>
#include <sys/mutex.h>

#include <contrib/dev/acpica/include/acpi.h>

#include "sst_regs.h"
#include "sst_firmware.h"
#include "sst_ipc.h"
#include "sst_ssp.h"
#include "sst_dma.h"
#include "sst_pcm.h"
#include "sst_jack.h"
#include "sst_topology.h"

/* ACPI IDs for Broadwell-U Audio DSP */
#define SST_ACPI_ID_BDW		"INT3438"
#define SST_ACPI_ID_HSW		"INT33C8"

/* Driver State */
enum sst_state {
	SST_STATE_NONE = 0,
	SST_STATE_ATTACHED,
	SST_STATE_RUNNING,
	SST_STATE_SUSPENDED,
	SST_STATE_ERROR
};

/* Software Context (Softc) */
struct sst_softc {
	device_t		dev;
	ACPI_HANDLE		handle;

	/* Memory Resources (MMIO) */
	int			mem_rid;	/* BAR0 - LPE memory (IRAM/DRAM/SHIM) */
	struct resource		*mem_res;
	int			shim_rid;	/* BAR1 - PCI extended config */
	struct resource		*shim_res;	/* Used for power gating control */

	/* Interrupt Resource */
	int			irq_rid;
	struct resource		*irq_res;
	void			*irq_cookie;

	/* Lock for register access */
	struct mtx		sc_mtx;

	/* State */
	enum sst_state		state;
	bool			attached;

	/* Firmware subsystem */
	struct sst_firmware	fw;

	/* IPC subsystem */
	struct sst_ipc		ipc;

	/* SSP (I2S) controller */
	struct sst_ssp		ssp;

	/* DMA controller */
	struct sst_dma		dma;

	/* PCM / sound(4) */
	struct sst_pcm		pcm;

	/* Jack detection */
	struct sst_jack		jack;

	/* Topology (audio pipeline) */
	struct sst_topology	topology;
};

/*
 * Register Access Helpers
 *
 * For Broadwell-U (WPT/catpt), SHIM registers are in BAR1 (shim_res):
 *   - CSR2 at offset 0x80, IPCX at 0x38, IPCD at 0x40, etc.
 *
 * Note: The original SST_SHIM_OFFSET (0xC0000) was for different platforms.
 * Broadwell-U uses BAR1 directly for SHIM registers.
 */
static inline uint32_t
sst_shim_read(struct sst_softc *sc, uint32_t reg)
{
	/* Broadwell-U: SHIM registers are in BAR1 (LPSS Private) */
	return (bus_read_4(sc->shim_res, reg));
}

static inline void
sst_shim_write(struct sst_softc *sc, uint32_t reg, uint32_t val)
{
	/* Broadwell-U: SHIM registers are in BAR1 (LPSS Private) */
	bus_write_4(sc->shim_res, reg, val);
}

static inline void
sst_shim_update_bits(struct sst_softc *sc, uint32_t reg,
		     uint32_t mask, uint32_t val)
{
	uint32_t old, new;

	mtx_lock(&sc->sc_mtx);
	old = sst_shim_read(sc, reg);
	new = (old & ~mask) | (val & mask);
	sst_shim_write(sc, reg, new);
	mtx_unlock(&sc->sc_mtx);
}

/*
 * DSP Memory Access
 */
static inline void
sst_dsp_write(struct sst_softc *sc, uint32_t offset, uint32_t val)
{
	bus_write_4(sc->mem_res, offset, val);
}

static inline uint32_t
sst_dsp_read(struct sst_softc *sc, uint32_t offset)
{
	return (bus_read_4(sc->mem_res, offset));
}

#endif /* _ACPI_INTEL_SST_H_ */
