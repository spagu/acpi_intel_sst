# Technical Findings: Intel SST DSP on Dell XPS 13 9343

## Document Information

- **Date:** 2026-02-12
- **Platform:** Dell XPS 13 9343 (Early 2015)
- **OS:** FreeBSD 15-CURRENT
- **Driver Version:** 0.6.0

---

## 1. Executive Summary

This document details the investigation into enabling Intel Smart Sound Technology (SST) DSP audio on the Dell XPS 13 9343 running FreeBSD. The DSP's BAR0 memory region (0xFE000000, 1MB) returns `0xFFFFFFFF` for all reads, while BAR1 (0xFE100000, 4KB PCI config mirror) works correctly.

**Root Cause Identified:** The Dell BIOS places the ADSP device in **ACPI mode** by setting the **PCICD (PCI Config Disable)** bit in the IOBP sideband register `PCICFGCTL` (address `0xd7000500`). This hides the device from PCI enumeration. The 1MB BAR0 memory window at 0xFE000000 is in a **separate PCH memory decode region** from the LPSS private config space at 0xFE100000. The firmware configures BAR0 decode during boot but puts the device in D3hot as the final step. The OS must use the **IOBP sideband interface** (RCBA+0x2330) to reconfigure memory decode, or properly transition the device from D3 to D0 with all required IOSF fabric setup.

**Key Breakthroughs:**
- LPSS private config space (0xFE100000-0xFE106FFF) is fully alive and decoded
- I2C0 controller at 0xFE103000 comes alive after D3-to-D0 transition via private config
- SDMA BAR0 at 0xFE101000 is accessible (returns 0x00000000)
- ADSP BAR0 at 0xFE000000 is in a **separate 1MB decode window** not covered by LPSS fabric
- FD register bit positions were corrected from DSDT analysis (bit 1=ADSD, not bit 0)
- IOBP sideband interface is the mechanism to control ADSP PCI/ACPI mode and memory decode

**Status:** IN PROGRESS - implementing IOBP sideband access to read/modify PCICFGCTL.

---

## 2. Hardware Configuration

### 2.1 System Information

| Component | Details |
|-----------|---------|
| **Laptop** | Dell XPS 13 9343 (Early 2015) |
| **CPU** | Intel Core i5/i7 Broadwell-U (5th Gen) |
| **PCH** | Intel Wildcat Point-LP (WPT) |
| **Audio DSP** | Intel SST (8086:9CB6) - NOT on PCI bus |
| **Audio Codec** | Realtek ALC3263/RT286 (dual-mode: HDA + I2S) |
| **HDA Controller** | 8086:9CA0 (Wildcat Point-LP HD Audio) - DISABLED by BIOS |
| **Custom DSDT** | `/boot/acpi_dsdt.aml` loaded at boot |

### 2.2 ACPI Device Information

```
Device: _SB.PCI0.ADSP
ACPI ID: INT3438 (Broadwell-U)
_STA: 0xF (Present, Enabled, Shown, Functional)
_HID: INT3438
_SUB: 10280665
_DDN: "Intel(R) Smart Sound Technology (Intel(R) SST)"
```

Note: ADSP has NO `_ADR` (no PCI address) - it is an ACPI-only device.

### 2.3 Memory Resources (from ACPI _CRS)

| BAR | Physical Address | Size | Purpose | Status |
|-----|------------------|------|---------|--------|
| BAR0 | 0xFE000000 | 1MB (0x100000) | DSP Memory (IRAM, DRAM, SHIM, DMA) | DEAD (0xFFFFFFFF) |
| BAR1 | 0xFE100000 | 4KB (0x1000) | PCI Config Space Mirror | WORKS |
| IRQ | 3 | - | ACPI Interrupt | Allocated |

The `_CRS` method reads BAR addresses from ACPI NVS variables:
```asl
B0VL = ADB0  /* BAR0 base from NVS -> 0xFE000000 */
B1VL = ADB1  /* BAR1 base from NVS -> 0xFE100000 */
IRQN = ADI0  /* IRQ from NVS -> 3 */
```

### 2.4 BIOS NVS (GNVS) Variables

Read from GNVS area at 0xDB7EF000 (verified via ACPI evaluation):

| Variable | Value | Meaning |
|----------|-------|---------|
| OSYS | 0x07DD | Windows 8.1 (2013) - already high enough for LPSS |
| S0ID | 1 | Connected Standby enabled |
| ANCS | 0 | Normal (audio uses CS) |
| SMD0 | 1 | LPSS in PCI mode (but PCI functions hidden) |
| SB10 | 0xFE102000 | SDMA BAR (private config) |
| ADB0 | 0xFE000000 | ADSP BAR0 |

### 2.5 PCH Function Disable (FD) Register

RCBA base: 0xFED1C000 (from LPC 0:1F.0 config offset 0xF0)

FD register at RCBA+0x3418 = **0x00368011**

**CORRECTED bit positions** (from DSDT OperationRegion RCRB field definitions):
```
Bit layout: [skip 1], ADSD(1), SATD(1), SMBD(1), HDAD(1), ...
```

| Bit | Name | Value | Meaning |
|-----|------|-------|---------|
| 0 | (reserved) | 1 | - |
| 1 | ADSD | 0 | **ADSP Enabled** |
| 2 | SATD | 0 | SATA Enabled |
| 3 | SMBD | 0 | SMBus Enabled |
| 4 | HDAD | 1 | **HDA Disabled** |

**WARNING:** Earlier driver versions used wrong bit positions (ADSD at bit 0, HDAD at bit 3), which accidentally disabled SMBus when trying to enable ADSP. The FD register has been restored to BIOS default 0x00368011.

### 2.6 Custom DSDT Configuration

The system loads a custom DSDT (`/boot/acpi_dsdt.aml`) that:
- Forces ADSP `_STA` to return 0x0F when ADB0 != 0
- Disables hdac and ig4 drivers via loader.conf hints

```bash
# /boot/loader.conf
acpi_dsdt_load="YES"
acpi_dsdt_name="/boot/acpi_dsdt.aml"
hint.hdac.0.disabled="1"
hint.hdac.1.disabled="1"
ig4_load="NO"
hint.ig4.0.disabled="1"
hint.ig4.1.disabled="1"
acpi_intel_sst_load="YES"
```

---

## 3. LPSS Memory Architecture

### 3.1 LPSS Address Space Map

The Intel Broadwell PCH LPSS (Low Power SubSystem) uses two separate memory regions:

```
0xFE000000 - 0xFE0FFFFF : ADSP BAR0 (1MB) - DSP IRAM/DRAM/SHIM
                           Separate PCH memory decode window
                           STATUS: DEAD (returns 0xFFFFFFFF)

0xFE100000 - 0xFE100FFF : ADSP BAR1 (4KB) - PCI config mirror
0xFE101000 - 0xFE101FFF : SDMA BAR0 (4KB) - DMA controller
0xFE102000 - 0xFE102FFF : SDMA private config
0xFE103000 - 0xFE103FFF : I2C0 BAR0 (4KB) - DesignWare I2C
0xFE104000 - 0xFE104FFF : I2C0 private config
0xFE105000 - 0xFE105FFF : I2C1 BAR0 (4KB) - DesignWare I2C (codec)
0xFE106000 - 0xFE106FFF : I2C1 private config
                           All in LPSS fabric decode range
                           STATUS: ALIVE (devices respond after D0 transition)
```

### 3.2 4KB Block Probe Results

Systematic probe of 0xFE100000-0xFE10F000 (4KB blocks):

| Address | [0x00] | [0x04] | Status |
|---------|--------|--------|--------|
| 0xFE100000 | 0x9CB68086 | 0x00100006 | ADSP private config (ALIVE) |
| 0xFE101000 | 0x00000000 | 0x00000000 | SDMA BAR0 (ALIVE) |
| 0xFE102000 | 0x9CB08086 | 0x00100006 | SDMA private config (ALIVE) |
| 0xFE103000 | 0xFFFFFFFF | 0xFFFFFFFF | I2C0 BAR0 (dead until D0 transition) |
| 0xFE104000 | 0x9CE18086 | 0x00100006 | I2C0 private config (ALIVE) |
| 0xFE105000 | 0xFFFFFFFF | 0xFFFFFFFF | I2C1 BAR0 (dead until D0 transition) |
| 0xFE106000 | 0x9CE28086 | 0x00100006 | I2C1 private config (ALIVE) |
| 0xFE107000+ | 0xFFFFFFFF | 0xFFFFFFFF | Undecoded |

### 3.3 I2C0 D3-to-D0 Transition (SUCCESS)

After writing PMCSR at I2C0 private config (0xFE104000+0x84) to clear D3 bits:

```
I2C0 PMCSR: 0x00000003 (D3hot)
I2C0 PMCSR after D0: 0x00000000 (D0)
I2C0 BAR0 IC_COMP_TYPE: 0x44570140 - DesignWare ALIVE!
```

This proves the D3-to-D0 transition via private config space WORKS for LPSS devices within the 0xFE100000 range.

### 3.4 LPSS Device Private Configs

| Device | Config Addr | VID/DID | BAR0 | PMCSR | Power State |
|--------|------------|---------|------|-------|-------------|
| ADSP | 0xFE100000 | 0x9CB68086 | 0xFE000000 | D0 (after our write) | Powered |
| SDMA | 0xFE102000 | 0x9CB08086 | 0xFE101000 | D0 | Powered |
| I2C0 | 0xFE104000 | 0x9CE18086 | 0xFE103000 | D3hot (until we transition) | Sleeping |
| I2C1 | 0xFE106000 | 0x9CE28086 | 0xFE105000 | D3hot | Sleeping |

### 3.5 Why BAR0 Is Dead

**The 1MB ADSP BAR0 (0xFE000000) is in a completely different PCH memory decode window than the 4KB LPSS private configs/BAR0s at 0xFE100000+.** The LPSS fabric decodes the 0xFE100000-0xFE106FFF range, but the separate 1MB DSP memory window requires its own decode enable in the PCH.

BAR0 remap experiment confirmed this:
- Writing 0xFE108000 to ADSP BAR0 register reads back as 0xFE100000 (1MB alignment mask)
- The ADSP has a 1MB minimum BAR0 size, so it cannot fit within the LPSS fabric range

---

## 4. IOBP Sideband Interface (Key Discovery)

### 4.1 Overview

The PCH uses an **IOBP (I/O Bridge Port)** sideband interface to control internal device configuration, including PCI/ACPI mode switching and memory decode enables. This is accessed through indexed registers at RCBA offsets.

Source: coreboot `src/southbridge/intel/lynxpoint/iobp.c` and `src/soc/intel/broadwell/pch/adsp.c`

### 4.2 IOBP Register Locations

| Register | RCBA Offset | Purpose |
|----------|-------------|---------|
| IOBPIRI | 0x2330 | Index Register (target IOBP address) |
| IOBPD | 0x2334 | Data Register |
| IOBPS | 0x2338 | Status Register (opcode + ready bit) |
| IOBPU | 0x233A | Undocumented/Magic register |

### 4.3 IOBP Access Protocol

From coreboot `pch_iobp_write()`:

```
1. Poll IOBPS[0] until 0 (not busy)
2. Write target address to IOBPIRI (0x2330)
3. Write opcode 0x0700 (WRITE) to IOBPS (0x2338)
   - For READ: use opcode 0x0600
4. Write data to IOBPD (0x2334)
5. Write magic value 0xF000 to IOBPU (0x233A)
6. Set ready bit in IOBPS (OR with 0x01)
7. Poll IOBPS[0] until 0 (transaction complete)
8. Verify success: check IOBPS TX_MASK bits (bits 1-2 should be 0)
```

### 4.4 ADSP IOBP Register Addresses

From coreboot `src/soc/intel/broadwell/include/soc/adsp.h`:

| IOBP Address | Name | Default | Purpose |
|-------------|------|---------|---------|
| 0xd7000500 | **PCICFGCTL** | varies | PCI Configuration Control (KEY REGISTER) |
| 0xd70001e0 | PMCTL | 0x3F | Power Management Control |
| 0xd7000624 | VDLDAT1 | 0x00040100 | Voltage/data line config |
| 0xd7000628 | VDLDAT2 | varies | IRQ routing override |

### 4.5 PCICFGCTL Register (0xd7000500)

This is the critical register that controls ADSP PCI visibility and memory decode:

| Bit | Name | Description |
|-----|------|-------------|
| 0 | **PCICD** | PCI Config Disable - hides device from PCI enumeration |
| 1 | **ACPIIE** | ACPI Interrupt Enable - routes IRQ to ACPI instead of PCI |
| 7 | **SPCBAD** | Sideband PCH BAR Disable - may prevent BAR0 decode |

**In ACPI mode** (our case), firmware sets: PCICD=1, ACPIIE=1

**In PCI mode**, firmware clears: PCICD=0, ACPIIE=0

### 4.6 Firmware ADSP Init Sequence (from coreboot)

The complete initialization performed by firmware during boot:

```c
// 1. Enable PCI memory + bus master (temporary)
pci_or_config16(dev, PCI_COMMAND, PCI_COMMAND_MASTER | PCI_COMMAND_MEMORY);

// 2. Program SHIM LTR via BAR0
write32(bar0 + SHIM_BASE + ADSP_SHIM_LTRC, ADSP_SHIM_LTRC_VALUE);

// 3. Clock/power via PCI config
pci_write_config32(dev, ADSP_PCI_VDRTCTL2, 0x00000fff);
pch_iobp_write(ADSP_IOBP_VDLDAT1, 0x00040100);

// 4. D3 power gating configuration (WPT-specific)

// 5. PSF Snoop to SA
RCBA32_OR(0x3350, (1 << 10));

// 6. Power management via IOBP
pch_iobp_write(ADSP_IOBP_PMCTL, 0x3f);

// --- ACPI Mode Specific ---

// 7. Save BARs to NVS for ACPI _CRS
dev_nvs->bar0 = bar0_base;  // 0xFE000000
dev_nvs->bar1 = bar1_base;  // 0xFE100000

// 8. HIDE PCI device
pch_iobp_update(PCICFGCTL, ~0, PCICD);  // Set PCICD bit

// 9. Route interrupt to IRQ3
pch_iobp_write(VDLDAT2, ACPI_IRQ3);
RCBA32_OR(ACPIIRQEN, ADSP_ACPI_IRQEN);

// 10. Set ACPI interrupt enable
pch_iobp_update(PCICFGCTL, ~SPCBAD, ACPIIE);

// 11. Put ADSP in D3hot (LAST STEP)
write32(bar1 + PCH_PCS, read32(bar1 + PCH_PCS) | D3HOT);
```

### 4.7 How Linux Handles This

Linux's catpt driver (`sound/soc/intel/catpt/device.c`) simply `ioremap()`s the BAR0 address from ACPI `_CRS` resources. It does NOT need to "enable" BAR0 decode because the firmware already configured the PCH address decoder before handing off to the OS.

```c
// Linux catpt: just maps the resource, no IOBP needed
cdev->lpe_ba = devm_platform_get_and_ioremap_resource(pdev, 0, &res);
```

**This means BAR0 decode SHOULD be active** - the firmware set it up. The question is why our reads return 0xFFFFFFFF despite the decode being configured.

---

## 5. WPT Memory Layout (Corrected)

### 5.1 Linux catpt WPT Descriptor

The WPT (Broadwell-U) DSP memory layout from Linux catpt differs from our initial assumptions:

```c
static struct catpt_spec wpt_desc = {
    .core_id           = 0x02,
    .fw_name           = "intel/IntcSST2.bin",
    .host_dram_offset  = 0x000000,   // DRAM at BAR0 + 0x000000
    .host_iram_offset  = 0x0A0000,   // IRAM at BAR0 + 0x0A0000
    .host_shim_offset  = 0x0FB000,   // SHIM at BAR0 + 0x0FB000
    .host_dma_offset   = { 0x0FE000, 0x0FF000 },
    .host_ssp_offset   = { 0x0FC000, 0x0FD000 },
};
```

### 5.2 Corrected DSP Memory Map

| Region | Offset | Size | Description |
|--------|--------|------|-------------|
| DRAM | 0x000000 | ~640KB | Data RAM |
| IRAM | 0x0A0000 | 80KB | Instruction RAM |
| SHIM | 0x0FB000 | 4KB | Control Registers |
| SSP0 | 0x0FC000 | 4KB | I2S Port 0 |
| SSP1 | 0x0FD000 | 4KB | I2S Port 1 |
| DMA0 | 0x0FE000 | 4KB | DMA Channel 0 |
| DMA1 | 0x0FF000 | 4KB | DMA Channel 1 |

**Note:** This differs significantly from the LPT (Haswell) layout used in our earlier code.

---

## 6. DSDT Analysis

### 6.1 PAUD Power Resource

The PAUD power resource controls audio power via GPIO:

```asl
PowerResource (PAUD, 0x00, 0x0000)
{
    Name (PSTA, One)
    Name (ONTM, Zero)
    Name (_STA, One)

    Method (_ON, 0, NotSerialized)
    {
        _STA = One
        PUAM ()
    }

    Method (_OFF, 0, NotSerialized)
    {
        _STA = Zero
        PUAM ()
    }

    Method (PUAM, 0, Serialized)
    {
        If (((_STA == Zero) && (UAMS != Zero)))
        {
            If ((RDGP (0x4C) == One))
            {
                WTGP (0x4C, Zero)  // Clear GPIO 76 - disable audio amp
                PSTA = Zero
                ONTM = Zero
            }
        }
        // When _STA == One, PUAM enables GPIO 76 (audio amp power)
    }
}
```

GPIO 0x4C (76) controls audio amplifier/codec power. Current state: **enabled** (bit 31 = 1).

### 6.2 ADSP _PS0 Method

The ADSP power-on method just waits for a timer delay (no hardware init):

```asl
Method (_PS0, 0, Serialized)
{
    If ((^^PAUD.ONTM == Zero))
    {
        Return (Zero)
    }
    Local0 = ((Timer - ^^PAUD.ONTM) / 0x2710)
    Local1 = (DSPD + VRRD)
    If ((Local0 < Local1))
    {
        Sleep ((Local1 - Local0))
    }
}
```

### 6.3 ADSP _CRS Resource Template

```asl
Name (RBUF, ResourceTemplate ()
{
    Memory32Fixed (ReadWrite, 0x00000000, 0x00100000, _Y30)  // BAR0: 1MB
    Memory32Fixed (ReadWrite, 0x00000000, 0x00001000, _Y31)  // BAR1: 4KB
    Interrupt (ResourceConsumer, Level, ActiveLow, Exclusive, , , _Y32) {3}
})

Method (_CRS, 0, NotSerialized)
{
    B0VL = ADB0  // Fill BAR0 base from NVS
    B1VL = ADB1  // Fill BAR1 base from NVS
    If ((ADI0 != Zero)) { IRQN = ADI0 }
    Return (RBUF)
}
```

### 6.4 I2C Codec Configuration

The I2C0 and I2C1 controllers have child devices for the RT286 codec:
- I2C1 target address: **0x2C** (RT286/ALC3263 I2C address)
- Codec supports HDA verb commands over I2C protocol
- Write format: `[addr_hi] [addr_lo] [data_hi] [data_lo]`
- Read format: `[addr_hi] [addr_lo] RESTART [read x4]`

---

## 7. Power Management State

### 7.1 Current Register Values (after driver init)

| Register | Via BAR1 | Value | Meaning |
|----------|----------|-------|---------|
| VID/DID | 0x00 | 0x9CB68086 | Intel SST Broadwell |
| CMD/STS | 0x04 | 0x00100006 | Memory + Bus Master enabled |
| BAR0 | 0x10 | 0xFE000000 | DSP memory base |
| BAR1 | 0x14 | 0xFE100000 | PCI config base |
| PMCSR | 0x84 | 0x00000008 | **D0** (after our transition) |
| VDRTCTL0 | 0xA0 | 0x000FFFFF | All SRAM powered ON |
| VDRTCTL2 | 0xA8 | 0x00000BFF | Clocks configured |

### 7.2 VDRTCTL0 Bit Breakdown (0x000FFFFF)

| Bits | Mask | Value | Meaning |
|------|------|-------|---------|
| 0 | D3PGD | 1 | D3 power gating disabled (DSP powered) |
| 1 | D3SRAMPGD | 1 | D3 SRAM power gating disabled |
| 2-11 | ISRAMPGE | 0x3FF | All 10 IRAM blocks powered ON |
| 12-19 | DSRAMPGE | 0xFF | All 8 DRAM blocks powered ON |

### 7.3 RCBA Register Summary

| Offset | Name | Value | Notes |
|--------|------|-------|-------|
| 0x2330 | IOBPIRI (SCC) | varies | IOBP sideband index |
| 0x2334 | IOBPD (SCC2) | varies | IOBP sideband data |
| 0x3100 | D31IP | varies | Device 31 interrupt pin |
| 0x3110 | D27IP | varies | Device 27 (HDA) interrupt pin |
| 0x3350 | PSF_SNOOP | varies | PSF snoop to SA |
| 0x3400 | IOSFCTL | 0x00000000 | IOSF fabric control |
| 0x3404 | IOSFDAT | 0x00000000 | IOSF fabric data |
| 0x3410 | CG | varies | Clock gating |
| 0x3418 | **FD** | 0x00368011 | Function Disable |
| 0x3424 | FD_LOCK | 0x00060010 | Possible FD lock |
| 0x3428 | FD2 | 0x0000001D | Function Disable 2 |

---

## 8. Theories and Next Steps

### 8.1 Theory A: IOBP PCICFGCTL needs reconfiguration

The firmware set PCICD=1 (PCI hidden) and possibly SPCBAD=1 (BAR disable). We need to:
1. Implement IOBP read/write via RCBA+0x2330
2. Read PCICFGCTL at IOBP address 0xd7000500
3. Try clearing PCICD to restore PCI visibility
4. Try clearing SPCBAD to enable BAR0 decode

### 8.2 Theory B: D3-to-D0 transition doesn't propagate to BAR0 decode

The PMCSR write via BAR1 changes the power state register value but may not actually transition the DSP silicon. The IOBP PMCTL register (0xd70001e0) may need to be written to 0x3F first.

### 8.3 Theory C: PSF Snoop bit missing

Coreboot sets `RCBA[0x3350] |= (1 << 10)` (PSF Snoop to System Agent). This may be required for memory transactions to reach the ADSP. Need to verify this bit is set.

### 8.4 Alternative Path: I2C Codec Communication

Since I2C0 comes alive after D3-to-D0 transition, I2C1 (codec at 0x2C) should too. This provides an alternative audio path:
1. Transition I2C1 to D0 via private config at 0xFE106000+0x84
2. Configure DesignWare I2C controller at 0xFE105000
3. Send HDA verb commands over I2C to RT286 codec
4. If codec responds, we have a working I2C audio path

### 8.5 Alternative Path: Enable PCH HDA Controller

The HDA controller (8086:9CA0) is disabled (HDAD bit 4 = 1 in FD register). If enabled:
1. Clear HDAD in FD register (needs reboot to take effect)
2. HDA controller appears at PCI 0:1B.0
3. FreeBSD's hdac(4) driver handles everything
4. RT286 codec supports HDA mode natively

**Note:** FD register writes are accepted and verified via readback, but PCH hardware routing only changes after reboot (FD is sampled during reset).

---

## 9. Register Definitions Reference

### 9.1 FD Register Bit Map (CORRECTED)

```
RCBA + 0x3418 (Function Disable):

Bit  0: (reserved/skip)
Bit  1: ADSD  - Audio DSP Disable    (0 = enabled)
Bit  2: SATD  - SATA Disable         (0 = enabled)
Bit  3: SMBD  - SMBus Disable        (0 = enabled)
Bit  4: HDAD  - HD Audio Disable     (1 = DISABLED)
Bits 5+: other functions...

BIOS default: 0x00368011
  = ADSP enabled, SATA enabled, SMBus enabled, HDA DISABLED
```

### 9.2 IOBP Sideband Opcodes

| Opcode | IOBPS Value | Direction |
|--------|-------------|-----------|
| READ | 0x0600 | Read from IOBP target |
| WRITE | 0x0700 | Write to IOBP target |

### 9.3 ADSP IOBP Target Addresses

| Address | Name | Notes |
|---------|------|-------|
| 0xd7000500 | PCICFGCTL | PCI/ACPI mode control |
| 0xd70001e0 | PMCTL | Power management control |
| 0xd7000624 | VDLDAT1 | Voltage/data config |
| 0xd7000628 | VDLDAT2 | IRQ routing |

---

## 10. Comparison: PCI Mode vs ACPI Mode

### 10.1 PCI Mode (Linux, some Windows configs)

- Device appears at Bus 0, Device 0x13 (19), Function 0
- PCI BAR0/BAR1 configured by PCI subsystem
- Interrupt via PCI IRQ 23
- PCICFGCTL: PCICD=0, ACPIIE=0
- OS discovers device via PCI enumeration

### 10.2 ACPI Mode (Our Dell XPS 13 9343)

- PCI config space HIDDEN via PCICFGCTL PCICD=1
- Device appears as ACPI INT3438
- BAR0/BAR1 addresses from NVS variables (ADB0, ADB1)
- Interrupt via ACPI IRQ 3
- PCICFGCTL: PCICD=1, ACPIIE=1
- Device put in D3hot as final firmware step

### 10.3 Key Difference

In PCI mode, the PCI bridge handles BAR routing automatically. In ACPI mode, the firmware pre-configures everything, then hides PCI. The OS just needs to transition from D3 to D0 and the memory should become accessible. If it doesn't, the IOBP sideband may need additional configuration.

---

## 11. Windows RWEverything Dump Analysis

Windows dumps from `rwdumps/` directory confirm:

### 11.1 ADSP at 0xFE100000 (Private Config)

```
VID/DID: 0x9CB68086
CMD/STS: 0x00100006 (Memory + Bus Master)
BAR0:    0xFE000000
BAR1:    0xFE100000
PMCSR:   D3hot (device sleeping when dump taken)
```

The device is in D3hot in Windows too when not actively playing audio.

### 11.2 I2C1 at 0xFE105000

Windows dump shows DesignWare I2C controller with target address 0x2C (RT286 codec), confirming the I2C path exists and codec is at the expected address.

### 11.3 HDA Controller

HDA at PCI 0:1B.0 is ABSENT in Windows too - confirmed by 0xFFFFFFFF VID/DID. The Dell BIOS disables HDA when ADSP mode is active.

---

## 12. Files Modified During Investigation

| File | Changes |
|------|---------|
| `acpi_intel_sst.c` | v0.6.0 - Complete rewrite with ACPI power-up, WPT sequence, BAR0 test, HDA/ADSP enablement, I2C probe, PCH state dump, GNVS reading, LPSS decode tests |
| `sst_regs.h` | Corrected FD bit definitions, added GNVS base, I2C registers, RCBA defines |
| `docs/TECHNICAL_FINDINGS.md` | This document - updated with all findings |
| `/boot/loader.conf` | Custom DSDT, disabled hdac/ig4, loads acpi_intel_sst |

---

## 13. References

### Source Code References
- [coreboot broadwell ADSP init](https://github.com/coreboot/coreboot/blob/master/src/soc/intel/broadwell/pch/adsp.c)
- [coreboot broadwell ADSP header](https://github.com/coreboot/coreboot/blob/master/src/soc/intel/broadwell/include/soc/adsp.h)
- [coreboot IOBP implementation](https://github.com/coreboot/coreboot/blob/master/src/southbridge/intel/lynxpoint/iobp.c)
- [Linux catpt driver](https://elixir.bootlin.com/linux/latest/source/sound/soc/intel/catpt/device.c)

### Datasheets
- Intel 9 Series PCH Datasheet (Wildcat Point)
- Intel 8 Series PCH Datasheet (Lynx Point)
- Intel Broadwell-U Platform Reference

### Community Resources
- [ArchWiki: Dell XPS 13 (9343)](https://wiki.archlinux.org/title/Dell_XPS_13_(9343))
- [GitHub: rbreaves/XPS-13-9343-DSDT](https://github.com/rbreaves/XPS-13-9343-DSDT)

---

*Document updated during Intel SST driver development for FreeBSD.*
*Last updated: 2026-02-12*
