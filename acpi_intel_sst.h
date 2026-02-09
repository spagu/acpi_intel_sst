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

/* ACPI IDs for Broadwell-U Audio DSP */
#define SST_ACPI_ID_BDW "INT3438"
#define SST_ACPI_ID_HSW "INT33C8"

/* Hardware constants */
#define SST_MIN_MMIO_SIZE    0x1000    /* Minimum expected MMIO region */
#define SST_INVALID_REG_VALUE 0xFFFFFFFF /* Indicates hardware issue */

/* Software Context (Softc) */
struct sst_softc {
    device_t        dev;
    ACPI_HANDLE     handle;

    /* Memory Resource (MMIO) */
    int             mem_rid;
    struct resource *mem_res;

    /* Interrupt Resource */
    int             irq_rid;
    struct resource *irq_res;
    void            *irq_cookie;

    /* State */
    bool            attached;
};

/* Registers (Haswell/Broadwell Shim) */
#define SST_SHIM_CSR        0x00
#define SST_SHIM_PISR       0x08
#define SST_SHIM_PIMR       0x10
#define SST_SHIM_ISRX       0x18
#define SST_SHIM_ISRD       0x20
#define SST_SHIM_IMRX       0x28
#define SST_SHIM_IMRD       0x30
#define SST_SHIM_IPCX       0x38
#define SST_SHIM_IPCD       0x40

/* CSR Bits */
#define SST_CSR_RST         (1 << 1)
#define SST_CSR_STALL       (1 << 0)
#define SST_CSR_DCS_MASK    (0x7 << 4)
#define SST_CSR_DCS(x)      ((x) << 4)

/* IPC Bits */
#define SST_IPC_BUSY        (1 << 31)
#define SST_IPC_DONE        (1 << 30)

#endif /* _ACPI_INTEL_SST_H_ */
