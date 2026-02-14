# Research Findings & Investigation Notes

This document consolidates historical research, debugging notes, and technical
investigations from the development of the Intel SST audio driver for FreeBSD.
These findings were critical in getting the driver working but are preserved here
for reference rather than cluttering the main documentation.

---

## Table of Contents

1. [BAR0 Memory Access Investigation (v0.6.0)](#1-bar0-memory-access-investigation)
2. [IOBP Sideband Interface](#2-iobp-sideband-interface)
3. [LPSS Memory Architecture](#3-lpss-memory-architecture)
4. [SRAM Power Gating Analysis (v0.41.0)](#4-sram-power-gating-analysis)
5. [DSP Boot False Positive (v0.40.0)](#5-dsp-boot-false-positive)
6. [Key Bug Fixes Timeline](#6-key-bug-fixes-timeline)
7. [Linux catpt Reference Data](#7-linux-catpt-reference-data)
8. [DSDT Analysis](#8-dsdt-analysis)
9. [Original Implementation Plan](#9-original-implementation-plan)

---

## 1. BAR0 Memory Access Investigation

*From v0.6.0 investigation (2026-02-12)*

### The Problem

The SST DSP BAR0 memory region (0xFE000000, 1MB) returned `0xFFFFFFFF` for all
reads, while BAR1 (0xFE100000, 4KB PCI config mirror) worked correctly.

### Root Cause

The Dell BIOS places the ADSP device in **ACPI mode** by setting the **PCICD**
bit in the IOBP sideband register `PCICFGCTL` (address `0xd7000500`). This hides
the device from PCI enumeration. The 1MB BAR0 at 0xFE000000 is in a separate PCH
memory decode region from the LPSS private config space at 0xFE100000.

### Resolution

The PCI device at 00:13.0 (8086:9CB6) provides an alternative BAR0 at 0xDF800000
which IS accessible. The driver now uses the PCI attachment path and the correct
WPT power-up sequence (SRAM PGE bit clearing + dummy reads) to access DSP memory.

### What Was Tried

- WPT power-up sequence (VDRTCTL0, PMCS transitions)
- IOMMU disabled (`hw.dmar.enable=0`)
- GPIO audio power enable (GPIO 0x4C)
- PCH RCBA FD register manipulation
- ACPI `_PS0`, `_ON`, `_INI`, `_DSM` methods
- GNVS variable verification
- LPSS address space probing
- ACPI `_OSI` spoofing (`hw.acpi.osi="Windows 2015"`) - did NOT help

---

## 2. IOBP Sideband Interface

The PCH uses an IOBP (I/O Bridge Port) sideband interface to control internal
device configuration, including PCI/ACPI mode switching.

### Register Locations

| Register | RCBA Offset | Purpose |
|----------|-------------|---------|
| IOBPIRI | 0x2330 | Index Register (target address) |
| IOBPD | 0x2334 | Data Register |
| IOBPS | 0x2338 | Status Register (opcode + ready) |
| IOBPU | 0x233A | Magic register |

### ADSP IOBP Registers

| IOBP Address | Name | Purpose |
|-------------|------|---------|
| 0xd7000500 | PCICFGCTL | PCI/ACPI mode control |
| 0xd70001e0 | PMCTL | Power management |
| 0xd7000624 | VDLDAT1 | Voltage/data config |
| 0xd7000628 | VDLDAT2 | IRQ routing |

### PCICFGCTL Bits

| Bit | Name | Description |
|-----|------|-------------|
| 0 | PCICD | PCI Config Disable - hides from PCI |
| 1 | ACPIIE | ACPI Interrupt Enable |
| 7 | SPCBAD | Sideband PCH BAR Disable |

In ACPI mode (Dell XPS 13): PCICD=1, ACPIIE=1.

---

## 3. LPSS Memory Architecture

### Address Space

```
0xFE000000 - 0xFE0FFFFF : ADSP BAR0 (1MB) - separate PCH decode window
0xFE100000 - 0xFE100FFF : ADSP BAR1 (PCI config mirror)
0xFE101000 - 0xFE101FFF : SDMA BAR0 (DMA controller)
0xFE102000 - 0xFE102FFF : SDMA private config
0xFE103000 - 0xFE103FFF : I2C0 BAR0 (DesignWare I2C)
0xFE104000 - 0xFE104FFF : I2C0 private config
0xFE105000 - 0xFE105FFF : I2C1 BAR0 (DesignWare I2C)
0xFE106000 - 0xFE106FFF : I2C1 private config
```

The 0xFE100000 range (LPSS fabric) is alive and accessible.
The 0xFE000000 range (1MB ADSP) requires separate decode enable.

### I2C0 D3-to-D0 Transition

Writing PMCSR at I2C0 private config (0xFE104084) clears D3 state.
After transition, I2C0 BAR0 IC_COMP_TYPE reads 0x44570140 (DesignWare confirmed).

---

## 4. SRAM Power Gating Analysis

*From v0.41.0 investigation (2026-02-13)*

### The Missing catpt_dsp_set_srampge() Logic

Linux catpt has `catpt_dsp_set_srampge()` which does three things after clearing
SRAM power gate bits that our driver was initially missing:

1. **60us delay** after PGE change for power propagation
2. **Dummy read from each newly-ungated block** to prevent byte loss
3. **Separate DRAM/IRAM ungating** (each with own delay + reads)

### Why DRAM Block 16 Failed

- DRAM has 20 blocks of 32KB (VDRTCTL0 bits 12-31)
- Offset 0x84000 = block 16 (bit 28)
- Without dummy readback after ungating, first write gets silently lost
- IRAM worked because lower-numbered blocks may initialize differently

### Full Linux catpt Power-Up Sequence

```
1.  Disable DCLCGE (clock gating)
2.  Set VDRTCTL2 clock gating: CGEALL & ~DCLCGE, then clear DTCGE
3.  Transition to D0
4.  Set D3PGD + D3SRAMPGD
5a. Ungate DRAM: clear DSRAMPGE + udelay(60) + dummy reads
5b. Ungate IRAM: clear ISRAMPGE + udelay(60) + dummy reads
6.  Set register defaults (CSR, SHIM, etc.)
7.  Restore MCLK: set CLKCTL SMOS bits
8.  Select LP clock
9.  Set SSP bank clocks: SBCS0 + SBCS1 in CS1
10. Release reset
11. Re-enable DCLCGE
12. Deassert IPC interrupts
```

### Other BSDs

No BSD has an Intel SST/catpt driver. This project is the first.

| OS | Intel SST Driver |
|----|-----------------|
| OpenBSD | None (azalia HDA only) |
| FreeBSD | **This project** |
| NetBSD | None (hdaudio HDA only) |
| DragonFly | None |

---

## 5. DSP Boot False Positive

*From v0.40.0 investigation (2026-02-13)*

### The Bug

"DSP FW_READY received! IPCD=0xa0011b93 (10ms)" - but firmware wasn't running.

### Root Cause

IPCD register had value 0xa0011b93 from a previous Windows/BIOS session. The
register was not properly cleared. Analysis of 0xa0011b93:

```
bit 31 = 1 -> BUSY
bit 30 = 0 -> DONE (not set)
bit 29 = 1 -> FW_READY
bits 24-28 = 00000 -> global_msg_type = GET_FW_VERSION
```

Valid FW_READY IPC message, but stale. Evidence: IPC timeout occurs immediately
after "boot success" because firmware is not actually running.

### Fix

Compare IPCD value before and after unstall. Only accept FW_READY if the value
changed AND fw_ready bit (29) is set in the new message.

### Memory Map Confirmed (WPT)

| Region | Host Offset | Size |
|--------|-------------|------|
| DRAM | 0x000000 | 640KB |
| IRAM | 0x0A0000 | 320KB |
| SHIM | 0x0FB000 | 4KB |
| SSP0 | 0x0FC000 | 512B |
| SSP1 | 0x0FD000 | 512B |
| DMA0 | 0x0FE000 | 1KB |
| DMA1 | 0x0FF000 | 1KB |

---

## 6. Key Bug Fixes Timeline

| Version | Bug | Impact |
|---------|-----|--------|
| v0.8.0 | SRAM PGE bit polarity inverted (`\|=` -> `&= ~`) | BAR0 dead |
| v0.8.0 | I2C codec on wrong bus (I2C1/0x2C instead of I2C0/0x1C) | No codec |
| v0.8.0 | OSYS too low for LPSS fabric (need >= 0x07DC) | LPSS dead |
| v0.15.0 | SRAM enable needs rising edge (0->1 transition) | SRAM dead |
| v0.22.0 | Dual ACPI+PCI driver conflict reset SRAM | SRAM dead |
| v0.26.0 | Wrong SHIM offset (0xC0000 -> 0xE7000 -> 0xFB000) | No DSP control |
| v0.40.0 | False FW_READY from stale IPCD register | Boot hang |
| v0.41.0 | Missing SRAM dummy reads after ungating | DRAM corrupt |
| v0.47.0 | Stale child device with dangling ivars on reload | Kernel panic |
| v0.49.0 | SRAM PGE polarity + DCLCGE/RST sequence wrong | DSP won't boot |
| v0.50.0 | Wrong stream type (RENDER -> SYSTEM) | No audio |
| v0.51.0 | NID shift in codec verbs (<<20 not <<24) | Wrong codec regs |
| v0.52.0 | Channel map, ISR flooding, I2C read protocol | Distortion |
| v0.53.0 | Page table PFN format (packed 20-bit, not uint32) | Severe distortion |

---

## 7. Linux catpt Reference Data

### IPC Message Format

```c
union catpt_global_msg {
    u32 val;
    struct {
        u32 status:5;          // bits 0-4
        u32 context:19;        // bits 5-23
        u32 global_msg_type:5; // bits 24-28
        u32 fw_ready:1;        // bit 29
        u32 done:1;            // bit 30
        u32 busy:1;            // bit 31
    };
};
```

### Default Register Values (WPT)

- CS_DEFAULT: 0x8480040E
- IMC_DEFAULT: 0x7FFF0003
- IMD_DEFAULT: 0x7FFF0003
- CLKCTL_DEFAULT: 0x000007FF

### Page Table Format

Linux catpt uses packed 20-bit PFNs via `catpt_arrange_page_table()`:
- Two 20-bit PFNs packed into every 5 bytes (40 bits)
- Even entry i: PFN at byte offset (i*5)/2, bits [19:0]
- Odd entry i: PFN at byte offset (i*5)/2, shifted << 4

### Channel Map

`catpt_get_channel_map(STEREO)` returns `GENMASK(31,8) | LEFT(0) | (RIGHT(2) << 4)` = `0xFFFFFF20`.
Uses catpt_channel_index enum (LEFT=0, CENTER=1, RIGHT=2), not sequential indices.

### Firmware Loading

Linux uses DMA (`catpt_dma_memcpy_todsp()`) for firmware loading. MMIO also works
for the SRAM if properly ungated with dummy reads.

---

## 8. DSDT Analysis

### PAUD Power Resource

GPIO 0x4C (76) controls audio amplifier power. The PAUD power resource toggles
this GPIO via the PUAM method.

### ADSP _STA Method

Original requires S0ID=1 or ANCS=1 to return 0x0F (enabled). FreeBSD sets
neither, requiring a DSDT patch. See [acpi/README.md](../acpi/README.md).

### _CRS Resources

BAR addresses come from ACPI NVS variables:
- ADB0 -> BAR0 base (0xFE000000)
- ADB1 -> BAR1 base (0xFE100000)
- ADI0 -> IRQ (3)

### I2C Codec

RT286 codec on I2C0, address 0x1C (not I2C1/0x2C which is the Dell touchpad).
Uses rl6347a I2C protocol: write format `[addr_hi] [addr_lo] [data_hi] [data_lo]`,
read format `[addr_hi] [addr_lo] RESTART [read x4]`.

---

## 9. Original Implementation Plan

*From early development planning (v0.3.0 era)*

### Phase Overview

```
Phase 1 - ACPI driver shell, probe/attach, resources
Phase 2 - DSP init, reset sequence, register access
Phase 3 - Firmware loader, IPC protocol, DSP boot
Phase 4 - I2S/SSP controller, DMA engine
Phase 5 - sound(4) PCM, mixer, jack detection
```

All phases are now complete. The original plan had incorrect memory offsets
(IRAM at 0x00000/64KB, DRAM at 0x10000/64KB, SHIM at 0x40000) which were
corrected to WPT values (DRAM at 0x000000/640KB, IRAM at 0x0A0000/320KB,
SHIM at 0x0FB000) based on Linux catpt source analysis.

### Firmware Format

IntcSST2.bin uses the $SST binary format (not SOF):
- File header with signature, module count, format version
- Module headers with $SST signature, block count, entry point
- Block headers with type (IRAM/DRAM), offset, size
- Firmware is loaded via MMIO writes to DSP SRAM

### IPC Protocol

Uses the catpt mailbox protocol:
- IPCX (0x38): Host->DSP command register (set BUSY to send)
- IPCD (0x40): DSP->Host notification register
- Inbox/outbox in DSP DRAM (offsets from FW_READY mailbox)

---

## References

- [Linux catpt driver source](https://elixir.bootlin.com/linux/latest/source/sound/soc/intel/catpt/)
- [coreboot Broadwell ADSP](https://github.com/coreboot/coreboot/blob/master/src/soc/intel/broadwell/pch/adsp.c)
- [coreboot IOBP implementation](https://github.com/coreboot/coreboot/blob/master/src/southbridge/intel/lynxpoint/iobp.c)
- [CoolStar Windows SST driver](https://github.com/coolstar/csaudiosstcatpt)

---

*Consolidated from: TECHNICAL_FINDINGS.md, FINDINGS_v040.md, FINDINGS_v041.md,
REBOOT_NOTES.md, IMPLEMENTATION_PLAN.md*
