# ACPI DSDT Patches for Dell XPS 13 9343

This folder contains ACPI DSDT files for the Dell XPS 13 9343, with patches to enable Intel SST (Smart Sound Technology) audio.

## Files

| File | Description |
|------|-------------|
| `DSDT.dat` | Original DSDT binary from Dell XPS 13 9343 BIOS |
| `DSDT.dsl` | Decompiled original DSDT (ASL source) |
| `DSDT_patched.dsl` | **Patched DSDT to ENABLE ADSP (SST) for FreeBSD** |
| `SSDT-ADSP-Disable.dsl` | Simple SSDT overlay to disable ADSP (alternative, may not work) |

## Background

The Dell XPS 13 9343 uses a dual-mode Realtek ALC3263 audio codec:
- **HDA mode** - Standard Intel HD Audio, works with FreeBSD `hdac` driver
- **I2S mode** - Requires Intel SST DSP driver (`acpi_intel_sst`)

The BIOS uses ACPI variables to determine which mode to use:
- `S0ID == 1` (Connected Standby) → I2S/SST mode (ADSP enabled)
- `ANCS == 1` (Audio Native Controller Support) → I2S/SST mode (ADSP enabled)
- Otherwise → HDA mode (ADSP disabled)

**Problem:** FreeBSD sets neither S0ID nor ANCS, so ADSP is disabled by default.

**Solution:** Patch DSDT to unconditionally enable ADSP (return 0x0F from _STA).

## Audio Mode Selection

| Mode | Driver | Patch Required | Status |
|------|--------|----------------|--------|
| **SST (I2S)** | `acpi_intel_sst` | `Return (0x0F)` | **Current patch** |
| HDA | `hdac` | `Return (Zero)` | Alternative |

## Patch Details

The patched DSDT modifies the `_SB.PCI0.ADSP._STA` method:

**Original (line ~10881):**
```asl
Method (_STA, 0, NotSerialized)
{
    If ((ADB0 == Zero))
        Return (Zero)
    If ((OSYS < 0x07DC))
        Return (Zero)
    If ((EOD == Zero))
        Return (0x0D)
    If ((S0ID == One))
        Return (0x0F)   // Enabled only if S0ID set
    If ((ANCS == One))
        Return (0x0F)   // Enabled only if ANCS set
    Return (Zero)       // Otherwise disabled
}
```

**Patched (SST mode):**
```asl
Method (_STA, 0, NotSerialized)
{
    If ((ADB0 == Zero))
    {
        Return (Zero)   // No BAR0 configured
    }
    Return (0x0F)       // Force ADSP enabled for SST driver
}
```

### _STA Return Values

| Value | Meaning |
|-------|---------|
| `0x0F` | Device is present, enabled, shown in UI, and functioning |
| `0x0D` | Device is present and functioning but disabled |
| `0x00` | Device is not present |

## Fixes Applied to DSDT_patched.dsl

The patched DSDT includes fixes for iasl disassembly issues:

1. **External method declarations** - Fixed `ECRD` and `MDBG` from `IntObj` to `MethodObj`
2. **Broken method calls** - Fixed `MDBG(Arg0)` and `ECRD(RefOf(...))` calls
3. **Orphan packages** - Wrapped 16 orphan `Package` declarations in `Name(PPXB, ...)`
4. **ADSP._STA patch** - Returns 0x0F to enable SST device

## Installation on FreeBSD

### Prerequisites

1. **Install iasl compiler:**
   ```bash
   pkg install acpica-tools
   ```

2. **Get the SST firmware:**
   ```bash
   fetch -o /boot/firmware/intel/IntcSST2.bin \
     http://ftp.debian.org/debian/pool/non-free/f/firmware-nonfree/firmware-intel-sound_20230625-2_all.deb
   # Extract from .deb if needed
   ```

### Step 1: Compile the patched DSDT

```bash
cd /path/to/acpi_intel_sst/acpi
iasl DSDT_patched.dsl
# Creates: DSDT_patched.aml
```

If compilation fails, you may need the SSDT tables from your system:
```bash
# Extract all ACPI tables
acpidump -dt > acpi_tables.txt
acpidump -t -o SSDT1.dat SSDT1
acpidump -t -o SSDT2.dat SSDT2
# ... repeat for all SSDTs

# Recompile with SSDTs
iasl -e SSDT*.dat DSDT_patched.dsl
```

### Step 2: Install the compiled AML

```bash
sudo cp DSDT_patched.aml /boot/acpi_dsdt.aml
```

### Step 3: Configure loader.conf

Add to `/boot/loader.conf`:
```bash
# Load custom DSDT with SST enabled
acpi_dsdt_load="YES"
acpi_dsdt_name="/boot/acpi_dsdt.aml"

# Disable HDA controller (conflicts with SST)
hint.hdac.0.disabled="1"
hint.hdac.1.disabled="1"
```

### Step 4: Build and install the SST driver

```bash
cd /path/to/acpi_intel_sst
make
sudo make install
# Add to /boot/loader.conf:
# acpi_intel_sst_load="YES"
```

### Step 5: Cold Reboot

**Important:** You must perform a cold boot (power off completely), not a soft reboot:
```bash
sudo shutdown -p now
# Wait 10 seconds, then power on
```

### Step 6: Verify

```bash
# Check if custom DSDT was loaded
dmesg | grep -i dsdt

# Check for SST device
dmesg | grep -i sst
devinfo | grep INT3438

# Check for sound device
cat /dev/sndstat
mixer
```

## Expected Results

With the patched DSDT and SST driver:

1. **ADSP device enabled** - `devinfo` should show INT3438/acpi_intel_sst0
2. **SST driver attached** - `dmesg` should show SST initialization
3. **BAR0 accessible** - No 0xFFFFFFFF reads from MMIO
4. **Sound device available** - `/dev/dsp` and `/dev/mixer` should exist
5. **Audio playback** - `mixer vol 80 && play -n synth 1 sine 440` should produce sound

## Switching to HDA Mode

If SST mode doesn't work, you can switch back to HDA mode:

1. Edit `DSDT_patched.dsl` line ~10874:
   ```asl
   Return (Zero)  // Force disabled - HDA mode
   ```

2. Recompile and install:
   ```bash
   iasl DSDT_patched.dsl
   sudo cp DSDT_patched.aml /boot/acpi_dsdt.aml
   ```

3. Update `/boot/loader.conf`:
   ```bash
   # Remove or comment out:
   # hint.hdac.0.disabled="1"
   # acpi_intel_sst_load="YES"
   ```

4. Cold reboot

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Compilation errors | Include SSDT tables: `iasl -e SSDT*.aml DSDT_patched.dsl` |
| DSDT not loading | Check `/boot/loader.conf` syntax |
| BAR0 returns 0xFFFFFFFF | Check PCH FD2 register for ADSPD bit |
| No audio after patch | Ensure cold boot (not soft reboot) |
| SST probe fails | Check firmware at `/boot/firmware/intel/IntcSST2.bin` |

## References

- [ArchWiki: Dell XPS 13 (9343)](https://wiki.archlinux.org/title/Dell_XPS_13_(9343))
- [FreeBSD Forums: Loading ACPI DSDT AML](https://forums.freebsd.org/threads/loading-acpi-dsdt-aml.46692/)
- [GitHub: rbreaves/XPS-13-9343-DSDT](https://github.com/rbreaves/XPS-13-9343-DSDT)
- [GitHub: major/xps-13-9343-dsdt](https://github.com/major/xps-13-9343-dsdt)
- [Linux catpt driver](https://github.com/torvalds/linux/tree/master/sound/soc/intel/catpt)

## Warnings

- **Backup your data** before testing DSDT modifications
- **Test carefully** - incorrect DSDT can cause boot failures
- **BIOS updates** may require re-extracting and re-patching DSDT
- **Dual-booting** with Windows may require 2 cold boots to switch audio modes
