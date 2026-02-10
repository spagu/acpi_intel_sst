# ACPI DSDT Patches for Dell XPS 13 9343

This folder contains ACPI DSDT files for the Dell XPS 13 9343, including patches to force HDA audio mode instead of I2S/SST mode.

## Files

| File | Description |
|------|-------------|
| `DSDT.dat` | Original DSDT binary from Dell XPS 13 9343 BIOS |
| `DSDT.dsl` | Decompiled original DSDT (ASL source) |
| `DSDT_patched.dsl` | **Patched DSDT to disable ADSP (SST) device** |

## Background

The Dell XPS 13 9343 uses a dual-mode Realtek ALC3263 audio codec:
- **HDA mode** - Standard Intel HD Audio, works with FreeBSD `hdac` driver
- **I2S mode** - Requires Intel SST DSP driver (not fully functional on FreeBSD)

The BIOS uses the ACPI `OSYS` variable to determine which mode to use:
- `OSYS >= 0x07DC` (Windows 2012+) → I2S/SST mode (ADSP enabled)
- `OSYS < 0x07DC` → HDA mode (ADSP disabled)

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
        Return (0x0F)
    If ((ANCS == One))
        Return (0x0F)
    Return (Zero)
}
```

**Patched:**
```asl
Method (_STA, 0, NotSerialized)
{
    Return (Zero)  // Force disabled - HDA mode
}
```

## Installation on FreeBSD

### Step 1: Compile the patched DSDT

```bash
cd /path/to/acpi_intel_sst/acpi
iasl DSDT_patched.dsl
# Creates: DSDT_patched.aml
```

If compilation fails with external method errors, you may need to include SSDT tables:
```bash
iasl -e SSDT*.dat -d DSDT.dat
```

### Step 2: Install the compiled AML

```bash
sudo cp DSDT_patched.aml /boot/acpi_dsdt.aml
```

### Step 3: Configure loader.conf

Add to `/boot/loader.conf`:
```bash
acpi_dsdt_load="YES"
acpi_dsdt_name="/boot/acpi_dsdt.aml"
```

### Step 4: Cold Reboot

**Important:** You must perform a cold boot (power off completely), not a soft reboot:
```bash
sudo shutdown -p now
# Wait 10 seconds, then power on
```

### Step 5: Verify

```bash
# Check if custom DSDT was loaded
dmesg | grep -i dsdt

# Check for HDA audio
cat /dev/sndstat
mixer
dmesg | grep -i hda
```

## Expected Results

With the patched DSDT:

1. **ADSP device disabled** - `devinfo` should not show INT3438
2. **HDA controller active** - `hdac1` should enumerate Realtek codec
3. **Sound device available** - `/dev/dsp` and `/dev/mixer` should exist
4. **Audio playback** - `mixer vol 80 && play -n synth 1 sine 440` should produce sound

## Troubleshooting

| Issue | Solution |
|-------|----------|
| Compilation errors | Include SSDT tables: `iasl -e SSDT*.aml DSDT_patched.dsl` |
| DSDT not loading | Check `/boot/loader.conf` syntax |
| No audio after patch | Ensure cold boot (not soft reboot) |
| Still seeing ADSP | Check `sysctl hw.acpi` for DSDT override status |

## References

- [ArchWiki: Dell XPS 13 (9343)](https://wiki.archlinux.org/title/Dell_XPS_13_(9343))
- [FreeBSD Forums: Loading ACPI DSDT AML](https://forums.freebsd.org/threads/loading-acpi-dsdt-aml.46692/)
- [GitHub: rbreaves/XPS-13-9343-DSDT](https://github.com/rbreaves/XPS-13-9343-DSDT)
- [GitHub: major/xps-13-9343-dsdt](https://github.com/major/xps-13-9343-dsdt)

## Warnings

- **Backup your data** before testing DSDT modifications
- **Test carefully** - incorrect DSDT can cause boot failures
- **BIOS updates** may require re-extracting and re-patching DSDT
- **Dual-booting** with Windows may require 2 cold boots to switch audio modes
