# v0.40.0 Findings - DSP Boot Analysis (2026-02-13)

## Summary

Version 0.40.0 implemented the Linux catpt-based firmware loader, boot sequence,
and corrected memory map. Build succeeds. Module loads and firmware parsing works.
DSP "boot" appeared to succeed but is actually a **false positive**.

## What Works

### Firmware Parsing - CORRECT
- IntcSST2.bin loaded: 260368 bytes, 8 modules, format 254
- Module signatures verified: all "$SST" (matches Linux catpt)
- Module[0]: 15 blocks (13 IRAM + 2 DRAM), size=260112
- Modules 1-7: metadata-only (blocks=0, persistent/scratch sizes only)
- Entry point: 0x00000000 (IRAM base)

### IRAM Writes - WORKING
- IRAM[0] reads back 0x00000d06 after firmware load (not 0xFFFFFFFF)
- 13 IRAM blocks loaded successfully with readback verification passing

### BAR0 Access - ALIVE
- 16 regions alive in BAR0 scan
- SHIM registers fully accessible at 0xFB000 (host offset)
- SSP0, SSP1, DMA0, DMA1 all reading valid values

### Boot Sequence Steps - CORRECT
- CSR set to CATPT_CS_DEFAULT (0x8480040E) after defaults
- RST cleared: CSR=0x0480040C (STALL=1, RST=0) - correct
- STALL cleared for boot: CSR=0x0480000C (STALL=0, RST=0) - correct

## CRITICAL BUGS FOUND

### Bug 1: FALSE POSITIVE FW_READY (STALE IPCD)

**Symptom**: "DSP FW_READY received! IPCD=0xa0011b93 (10ms)"

**Root Cause**: IPCD register had value 0xa0011b93 BEFORE boot.

Evidence:
- BAR0 scan (BEFORE boot): `0x0fb040 SHIM IPCD = 0xa0011b93 ALIVE`
- Boot code writes `sst_shim_write(sc, SST_SHIM_IPCD, 0)` to clear it
- But 10ms after unstall, IPCD reads back as 0xa0011b93 again

**Analysis of 0xa0011b93**:
Using Linux catpt `union catpt_global_msg` format:
```
0xa0011b93 = 1010 0000 0000 0001 0001 1011 1001 0011
bit 31 = 1 → BUSY (set)
bit 30 = 0 → DONE (not set)
bit 29 = 1 → FW_READY (set!)
bits 28-24 = 00000 → global_msg_type = 0 (GET_FW_VERSION)
bits 23-5 = context
bits 4-0 = 10011 = status 19
```

This IS a valid FW_READY IPC message, but it's from a PREVIOUS session
(Windows or BIOS). The IPCD register was not properly cleared.

**Evidence it's stale**: After "boot success", IPC timeout occurs immediately.
Real firmware would respond to IPC messages.

**Fix**: Record IPCD value before unstall. Only accept FW_READY if:
1. IPCD value CHANGED from pre-unstall value, AND
2. fw_ready bit (bit 29) is set in the new message

### Bug 2: DRAM WRITE MISMATCH

**Symptom**: Block at DRAM offset 0x84000:
```
wrote=0x00020001 read=0xa77d5c73
```

**Analysis**:
- IRAM writes persist (verified: IRAM[0]=0x00000d06)
- DRAM writes do NOT persist at offset 0x84000
- Read value 0xa77d5c73 is not 0xFFFFFFFF (region is powered)
- But data is not what we wrote → possible SRAM power gate issue

**DRAM block details from firmware**:
```
Block: type=DRAM(2) offset=0x84000 size=0x16ba8 -> BAR0+0x84000
Block: type=DRAM(2) offset=0x9abb0 size=0xe00 -> BAR0+0x9abb0
```

**Hypothesis**: DRAM at BAR0+0x000000 covers 640KB (20 blocks x 32KB).
Block at 0x84000 = block index 0x84000/0x8000 = block 16.
The WPT VDRTCTL0 DSRAMPGE bits (12-31) control 20 DRAM blocks.
If blocks 16+ are still power-gated, writes won't persist.

Need to verify: Are ALL 20 DSRAMPGE bits cleared in VDRTCTL0?

### Bug 3: IPC TIMEOUT AFTER "BOOT"

**Symptom**: "IPC timeout" immediately after "DSP firmware running!"

**Root Cause**: Direct consequence of Bug 1 (false FW_READY). DSP firmware
isn't actually running, so it can't respond to IPC.

## Register Dump During Boot

```
Initial state:
  SHIM CSR: 0x0480081f

After sst_dsp_set_regs_defaults():
  CSR: 0x8480040e (matches CATPT_CS_DEFAULT)

After de-assert RST:
  CSR: 0x0480040c (STALL=1, RST=0)
  LPCS cleared, SBCS0+SBCS1 set

Before unstall:
  CSR: 0x0480040c (STALL=1, RST=0) - correct

After unstall:
  CSR: 0x0480000c (STALL=0, RST=0) - correct, DSP should run

Boot timeout dump (not reached due to false positive):
  Would need: CSR, IPCD, IPCX, ISRX, IMRX, ISRD, IMRD
```

## Linux catpt Reference Values (Verified)

### Memory Map (WPT)
| Region | Host Offset | Size | Status |
|--------|-------------|------|--------|
| DRAM | 0x000000 | 0x0A0000 (640KB) | Partially alive |
| IRAM | 0x0A0000 | 0x050000 (320KB) | Working |
| SHIM | 0x0FB000 | 0x001000 (4KB) | Working |
| SSP0 | 0x0FC000 | 0x200 (512B) | Working |
| SSP1 | 0x0FD000 | 0x200 (512B) | Working |
| DMA0 | 0x0FE000 | 0x400 (1KB) | Working |
| DMA1 | 0x0FF000 | 0x400 (1KB) | Working |

### Default Register Values (CONFIRMED match Linux catpt)
- CS_DEFAULT: 0x8480040E ✓
- IMC_DEFAULT: 0x7FFF0003 ✓
- IMD_DEFAULT: 0x7FFF0003 ✓
- CLKCTL_DEFAULT: 0x000007FF ✓

### IPC Message Format (from Linux catpt messages.h)
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

### FW_READY Mailbox (must be read after valid FW_READY)
```c
struct catpt_fw_ready {
    u32 inbox_offset;
    u32 outbox_offset;
    u32 inbox_size;
    u32 outbox_size;
    u32 fw_info_size;
    char fw_info[FW_INFO_SIZE_MAX];
};
```

## Next Steps

1. **Fix false FW_READY** - compare IPCD before/after unstall
2. **Fix DRAM power gating** - verify all DSRAMPGE bits cleared
3. **Add fw_ready bit check** - bit 29 in IPCD must be set
4. **Parse FW_READY mailbox** - get inbox/outbox configuration
5. **Investigate whether IPCD write-clear works** - may need special handling
