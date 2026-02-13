# v0.41.0 Findings - SRAM Power Gating & Linux catpt Deep Analysis (2026-02-13)

## Summary

v0.41.0 fixed the false FW_READY detection (stale IPCD). The DRAM write mismatch
at offset 0x84000 remains. Deep analysis of Linux catpt source reveals **three
critical missing steps** in our SRAM power-up sequence that are the probable cause.

## Bug Status

| Bug | Status | Fix |
|-----|--------|-----|
| Bug 1: False FW_READY | **FIXED in v0.41.0** | Compare IPCD before/after unstall |
| Bug 2: DRAM write mismatch | **ROOT CAUSE FOUND** | See below |
| Bug 3: IPC timeout | Blocked by Bug 2 | DSP can't run without valid DRAM |

## ROOT CAUSE: Missing catpt_dsp_set_srampge() Logic

### The Critical Function We're Missing

Linux catpt has `catpt_dsp_set_srampge()` (dsp.c) which does THREE things after
clearing SRAM power gate bits:

```c
static void catpt_dsp_set_srampge(struct catpt_dev *cdev, struct resource *sram,
                                  unsigned long mask, unsigned long new)
{
    unsigned long old;
    u32 off = sram->start;
    unsigned long b = __ffs(mask);

    old = catpt_readl_pci(cdev, VDRTCTL0) & mask;
    if (old == new)
        return;

    /* 1. Write new PGE bits */
    catpt_updatel_pci(cdev, VDRTCTL0, mask, new);

    /* 2. Wait 60us for SRAM power gating to propagate */
    udelay(60);

    /* 3. Dummy read from each NEWLY ENABLED block to prevent byte loss */
    for_each_clear_bit_from(b, &new, fls_long(mask)) {
        u8 buf[4];
        if (test_bit(b, &old)) {  /* was gated (1), now ungated (0) */
            memcpy_fromio(buf, cdev->lpe_ba + off, sizeof(buf));
        }
        off += CATPT_MEMBLOCK_SIZE;  /* 0x8000 = 32KB */
    }
}
```

### What Our Driver Is Missing

1. **`udelay(60)` after PGE change** - 60us delay for SRAM power propagation.
   Our code has a 100ms delay but it's AFTER re-enabling DCLCGE, not between
   PGE change and first SRAM access.

2. **Dummy readback per SRAM block** - Linux explicitly reads 4 bytes from the
   start of each newly-ungated block. The code comment says: "Dummy read as the
   very first access after block enable to prevent byte loss in future operations."
   **This is the most likely cause of our DRAM write mismatch.**

3. **Separate DRAM/IRAM ungating** - Linux calls set_srampge() separately for
   DRAM and IRAM (each with its own 60us delay + dummy reads). Our code clears
   both ISRAMPGE and DSRAMPGE in a single VDRTCTL0 write.

### Why DRAM Block 16 (offset 0x84000) Fails

- DRAM has 20 blocks of 32KB each (bits 12-31 of VDRTCTL0)
- Offset 0x84000 / 0x8000 = block 16 (VDRTCTL0 bit 28)
- Without the dummy readback after ungating, the first write to block 16
  gets silently lost ("byte loss")
- IRAM works because it may have been accessed during the register defaults
  setup or because its blocks are lower-numbered and get initialized differently

### Fix Plan

In `sst_wpt_power_up()`, replace the single VDRTCTL0 write with:

```c
/* Ungate DRAM (clear DSRAMPGE bits) */
vdrtctl0 &= ~SST_WPT_VDRTCTL0_DSRAMPGE_MASK;
bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
DELAY(60);  /* 60us - wait for power propagation */
/* Dummy read from each DRAM block (20 blocks x 32KB) */
for (i = 0; i < 20; i++)
    (void)bus_read_4(sc->mem_res, SST_DRAM_OFFSET + i * SST_MEMBLOCK_SIZE);

/* Ungate IRAM (clear ISRAMPGE bits) */
vdrtctl0 = bus_read_4(sc->shim_res, SST_PCI_VDRTCTL0);
vdrtctl0 &= ~SST_WPT_VDRTCTL0_ISRAMPGE_MASK;
bus_write_4(sc->shim_res, SST_PCI_VDRTCTL0, vdrtctl0);
DELAY(60);
/* Dummy read from each IRAM block (10 blocks x 32KB) */
for (i = 0; i < 10; i++)
    (void)bus_read_4(sc->mem_res, SST_IRAM_OFFSET + i * SST_MEMBLOCK_SIZE);
```

## Full Linux catpt Power-Up Sequence (catpt_dsp_power_up)

Our driver is missing several steps. Here's the COMPLETE Linux sequence:

```
1.  Disable DCLCGE (clock gating)
2.  Set VDRTCTL2 clock gating: CGEALL & ~DCLCGE, then clear DTCGE  ← MISSING
3.  Transition to D0
4.  Set D3PGD + D3SRAMPGD (disable power gating)
5a. Ungate DRAM: clear DSRAMPGE + udelay(60) + dummy reads          ← MISSING
5b. Ungate IRAM: clear ISRAMPGE + udelay(60) + dummy reads          ← MISSING
6.  Set register defaults (CSR, SHIM, etc.)
7.  Restore MCLK: set CLKCTL SMOS bits                              ← MISSING
8.  Select LP clock: catpt_dsp_select_lpclock(false, false)          ← MISSING
9.  Set SSP bank clocks: SBCS0 + SBCS1 in CS1
10. Release reset: catpt_dsp_reset(false)
11. Re-enable DCLCGE
12. Deassert IPC interrupts: clear IMC IPCDB+IPCCD bits              ← MISSING
```

Steps marked "MISSING" need to be added.

## Full Linux catpt Power-Down Sequence (catpt_dsp_power_down)

```
1.  Disable DCLCGE
2.  Assert reset: catpt_dsp_reset(true)
3.  Set SSP bank clocks
4.  Select LP clock: catpt_dsp_select_lpclock(true, false)
5.  Disable MCLK: clear CLKCTL SMOS bits
6.  Set register defaults
7.  Set VDRTCTL2 clock gating pattern + enable DTCGE
8a. Gate DRAM: set DSRAMPGE + udelay(60)
8b. Gate IRAM: set ISRAMPGE + udelay(60)
9.  Set D3PGD, clear D3SRAMPGD
10. Transition to D3hot
11. udelay(50)
12. Re-enable DCLCGE
13. udelay(50)
```

## Missing Register Defines

### CATPT_CLKCTL_CFCIP (BIT 31)
```c
#define CATPT_CLKCTL_CFCIP  BIT(31)  /* Clock Frequency Change In Progress */
```
Used in `catpt_dsp_select_lpclock()` to poll until clock change completes:
```c
catpt_updatel_shim(cdev, CLKCTL, CATPT_CLKCTL_LPCS, newval);
while (catpt_readl_shim(cdev, CLKCTL) & CATPT_CLKCTL_CFCIP)
    udelay(10);
```

### CATPT_CLKCTL_SMOS
```c
#define CATPT_CLKCTL_SMOS  GENMASK(25, 24)  /* SSP MCLK Output Select */
```
Set during power-up to restore master clock.

### CATPT_VDRTCTL2_DTCGE
```c
#define CATPT_VDRTCTL2_DTCGE  BIT(10)  /* Trunk Clock Gating Enable */
```
Already defined in our sst_regs.h as SST_VDRTCTL2_DTCGE.

## Other BSDs - No Intel SST Support

Research confirmed: **No BSD has an Intel SST/catpt driver.**

| OS | Intel SST Driver | Audio Approach |
|----|-----------------|----------------|
| OpenBSD | None | azalia(4) HDA only |
| FreeBSD | **This project (first!)** | snd_hda(4) for HDA mode |
| NetBSD | None | hdaudio(4) HDA only |
| DragonFly | None | HDA inherited from FreeBSD |

The only non-Linux implementation is coolstar's csaudiosstcatpt for Windows
(targeting Chromebooks).

OpenBSD's azalia driver matches PCI class 0x0403 (HD Audio). The Intel SST DSP
uses class 0x0401 (Multimedia Audio Controller) and ACPI enumeration (INT3438),
so azalia cannot attach to it.

On platforms where BIOS configured I2S/DSP mode (like Dell XPS 13 9343),
no BSD has working audio. Linux can force HDA mode via CONFIG_ACPI_REV_OVERRIDE.

## Linux catpt Firmware Loading Method

Linux uses **DMA** (`catpt_dma_memcpy_todsp()`) for firmware loading, not MMIO.
However, the SRAM is also accessible via MMIO (memcpy_toio used for small ops).
Our MMIO approach should work IF the SRAM is properly ungated with dummy reads.

Key insight: `catpt_load_block()` uses DMA with addresses ORed with
`CATPT_DMA_DSP_ADDR_MASK` to reach DSP memory space via the on-chip DMA
controllers at BAR0+0xFE000/0xFF000.

## Next Steps

1. **Implement catpt_dsp_set_srampge() equivalent** with 60us delay + dummy reads
2. **Add VDRTCTL2 clock gating setup** (CGEALL pattern)
3. **Add CLKCTL SMOS + CFCIP** clock management
4. **Add IMC interrupt deassert** after power-up
5. **Test DRAM accessibility** at offset 0x84000 after fix
6. **If MMIO still fails, implement DMA-based firmware loading**
