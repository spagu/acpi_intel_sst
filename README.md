# Intel SST Audio Driver for FreeBSD

[![FreeBSD](https://img.shields.io/badge/FreeBSD-15--CURRENT-red?logo=freebsd&logoColor=white)](https://www.freebsd.org/)
[![License](https://img.shields.io/badge/License-BSD--3--Clause-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Intel%20Broadwell--U-0071C5?logo=intel&logoColor=white)](https://ark.intel.com/)

> **FreeBSD kernel module for Intel Smart Sound Technology (SST) DSP audio**
> Audio playback working on Broadwell-U platforms with Realtek RT286/ALC3263 codec

This is the first Intel SST audio driver for any BSD operating system.

---

## Installation

### Prerequisites

- FreeBSD 15-CURRENT (or 14.x with source tree)
- FreeBSD source tree at `/usr/src`
- Intel Broadwell-U or Haswell platform
- Intel SST firmware file (`IntcSST2.bin`)

### 1. Build

```bash
git clone https://github.com/spagu/acpi_intel_sst.git
cd acpi_intel_sst
make
```

### 2. Install Firmware

```bash
sudo mkdir -p /boot/firmware/intel
fetch -o /tmp/fw.deb \
  'http://ftp.debian.org/debian/pool/non-free-firmware/f/firmware-nonfree/firmware-intel-sound_20230210-5_all.deb'
cd /tmp && ar x fw.deb && tar xf data.tar.xz
sudo cp lib/firmware/intel/IntcSST2.bin /boot/firmware/intel/
```

See [Firmware](#firmware) section for alternative methods.

### 3. DSDT Patch (Dell XPS 13 9343)

The Dell BIOS disables the ADSP device unless Connected Standby is active.
A DSDT patch forces it on. See [acpi/README.md](acpi/README.md) for full instructions.

```bash
cd acpi_intel_sst/acpi
iasl DSDT_patched.dsl
sudo cp DSDT_patched.aml /boot/acpi_dsdt.aml
```

### 4. Configure /boot/loader.conf

Add the following to `/boot/loader.conf`:

```
# Custom DSDT with ADSP enabled
acpi_dsdt_load="YES"
acpi_dsdt_name="/boot/acpi_dsdt.aml"

# Required: DSDT needs OSYS >= 0x07DC to initialize LPSS fabric
hw.acpi.install_interface="Windows 2012"

# Disable GPU HDMI audio controller (conflicts with SST IRQ)
hint.hdac.0.disabled="1"

# Load SST driver at boot
acpi_intel_sst_load="YES"

# Disable ig4 I2C driver (SST driver accesses I2C0 directly for codec control)
ig4_load="NO"
hint.ig4.0.disabled="1"
hint.ig4.1.disabled="1"
```

**Why each line is needed:**

| Setting | Reason |
|---------|--------|
| `acpi_dsdt_*` | Custom DSDT forces ADSP device enabled |
| `hw.acpi.install_interface` | DSDT requires OSYS >= 0x07DC to route memory to BAR0 |
| `hint.hdac.0.disabled` | Disables Intel GPU audio (hdac0 at pci0:0:3:0, 8086:160c) |
| `acpi_intel_sst_load` | Auto-load SST driver at boot |
| `ig4_load` / `hint.ig4.*` | Prevents ig4 driver from claiming I2C0, which SST uses directly for RT286 codec |

Note: `hdac1` (PCH HDA at pci0:0:27:0, 8086:9ca0) does not need to be disabled.
It has no codecs on the HDA link (RT286 is on I2S), so it's harmless.

### 5. Cold Reboot and Test

A **cold boot** (power off, not reboot) is required for DSDT changes to take effect.

```bash
sudo shutdown -p now
# Wait 10 seconds, power on

# After boot, verify:
cat /dev/sndstat
mixer vol 80
play -n synth 3 sine 440  # requires audio/sox
```

### Manual Loading (without loader.conf)

```bash
sudo kldload ./acpi_intel_sst.ko
cat /dev/sndstat
mixer vol 80
play -n synth 3 sine 440
```

---

## Current Status (v0.53.0)

**Audio playback works.** The driver boots the DSP firmware, configures the RT286
codec over I2C, and streams audio through SSP0 to the speakers and headphone jack.

| Component | Status |
|-----------|--------|
| ACPI/PCI probe & attach | Working |
| Power management (WPT sequence) | Working |
| DSP firmware boot (IntcSST2.bin) | Working |
| IPC framework (catpt protocol) | Working |
| RT286 codec I2C control | Working |
| SSP/I2S audio output | Working |
| PCM sound(4) integration | Working |
| Jack detection (GPIO polling) | Working |
| Audio playback (4 streams max) | **Working** |
| Audio capture | Disabled (DSP limitation) |
| Volume control via IPC | Not yet |
| Suspend/resume | Not yet |

See [STATUS.md](STATUS.md) for detailed status and next steps.
See [CHANGELOG.md](CHANGELOG.md) for version history.

---

## Hardware

| Component | Details |
|-----------|---------|
| **Platform** | Intel Broadwell-U (5th Gen Core) |
| **Tested Device** | Dell XPS 13 9343 (2015) |
| **DSP** | Intel SST (PCI 8086:9CB6 / ACPI INT3438) |
| **Codec** | Realtek RT286 / ALC3263 on I2C0, address 0x1C |
| **Transport** | I2S via SSP0 (playback) / SSP1 (capture, future) |
| **PCH** | Intel Wildcat Point-LP (WPT) |

### Supported ACPI/PCI IDs

| ID | Platform | Status |
|----|----------|--------|
| `INT3438` (ACPI) | Intel Broadwell-U | Tested |
| `INT33C8` (ACPI) | Intel Haswell | Untested |
| `8086:9CB6` (PCI) | Intel Broadwell-U | Tested |
| `8086:9C76` (PCI) | Intel Haswell | Untested |

---

## Architecture

```
sound(4)  <->  pcm child device (4 play + 2 capture channels)
                   | ivars
              acpi_intel_sst (sst_softc)
                   | IPC (catpt protocol)
              DSP Firmware (IntcSST2.bin)
                   | SSP0 (I2S, 48kHz/16bit/2ch)
              RT286/ALC3263 codec (I2C0, addr 0x1C)
                   |
              Speaker / Headphone
```

### Source Files

| File | Purpose |
|------|---------|
| acpi_intel_sst.c | Main driver: ACPI/PCI probe, attach, power, ISR |
| sst_firmware.c | Firmware load, parse ($SST format), DSP boot |
| sst_ipc.c | Host<->DSP IPC messaging (catpt protocol) |
| sst_ipc.h | IPC protocol definitions and structs |
| sst_codec.c | RT286 codec control over I2C (DesignWare I2C0 at 0xFE103000) |
| sst_codec.h | Codec register definitions (HDA verb encoding) |
| sst_pcm.c | sound(4) PCM integration, DMA page tables, DSP stream alloc |
| sst_ssp.c | I2S/SSP port configuration (2 ports) |
| sst_dma.c | DMA controller (DesignWare DW-DMAC, 8 channels) |
| sst_jack.c | Headphone jack detection (GPIO polling, debounce) |
| sst_topology.c | Audio pipeline configuration (default Broadwell-U topology) |
| sst_regs.h | Hardware register definitions (SHIM, VDRTCTL, SSP, DMA) |

---

## Firmware

The driver loads firmware via FreeBSD's `firmware_get("intel/IntcSST2.bin")`,
which looks in `/boot/firmware/intel/IntcSST2.bin`.

### Obtaining Firmware

**Option 1: Debian package (recommended)**

```bash
sudo mkdir -p /boot/firmware/intel
fetch -o /tmp/fw.deb \
  'http://ftp.debian.org/debian/pool/non-free-firmware/f/firmware-nonfree/firmware-intel-sound_20230210-5_all.deb'
cd /tmp && ar x fw.deb && tar xf data.tar.xz
sudo cp lib/firmware/intel/IntcSST2.bin /boot/firmware/intel/
```

**Option 2: From a Linux system**

```bash
# On Debian/Ubuntu:
apt download firmware-intel-sound
dpkg -x firmware-intel-sound*.deb /tmp/fw
cp /tmp/fw/lib/firmware/intel/IntcSST2.bin /path/to/freebsd/boot/firmware/intel/
```

**Option 3: Extract from Windows driver**

Look for `IntcSST2.bin` inside the Intel Smart Sound Technology driver package.

### Firmware Compatibility

| File | Platform | Compatible |
|------|----------|------------|
| `IntcSST2.bin` | Broadwell-U / Haswell | **Yes** |
| `fw_sst_0f28.bin` | Baytrail (Atom Z3xxx) | No - wrong platform |
| `fw_sst_22a8.bin` | Cherrytrail | No - wrong platform |

---

## Usage

```bash
# Load driver
sudo kldload ./acpi_intel_sst.ko

# Check sound device
cat /dev/sndstat

# Set volume and play
mixer vol 80
play -n synth 3 sine 440          # test tone (needs audio/sox)
cat /path/to/file.wav > /dev/dsp  # play WAV file

# Unload
sudo kldunload acpi_intel_sst
```

---

## Known Issues

1. **Capture disabled** - capture channels are registered but skipped in trigger;
   DSP firmware doesn't support simultaneous playback+capture on the same SSP port
2. **No volume control via IPC** - mixer widget exists but changes don't reach DSP
3. **No suspend/resume** - driver doesn't handle D3 transitions during sleep
4. **Dell XPS 13 9343 only** - untested on other Broadwell-U/Haswell platforms

---

## Other Broadwell-U Devices

These devices use the same Intel SST DSP and may work with this driver (untested):

| Manufacturer | Model |
|--------------|-------|
| Dell | XPS 13 9343 (**confirmed**) |
| HP | Spectre x360, EliteBook 720/750/850 G2, Folio 1040 G2 |
| Lenovo | ThinkPad X250, X1 Carbon Gen 3, Yoga 3 14 |
| Asus | Zenbook UX303LA/LB, UX305LA |
| Acer | Aspire R13, Aspire S7-393 |

If you have one of these running FreeBSD, please test and report results.

---

## Documentation

| File | Description |
|------|-------------|
| [STATUS.md](STATUS.md) | Current driver status, known issues, next steps |
| [CHANGELOG.md](CHANGELOG.md) | Detailed version history |
| [CONTRIBUTING.md](CONTRIBUTING.md) | How to contribute |
| [acpi/README.md](acpi/README.md) | DSDT patch instructions for Dell XPS 13 9343 |
| [docs/RESEARCH_FINDINGS.md](docs/RESEARCH_FINDINGS.md) | Historical research: BAR0 investigation, SRAM power gating, IOBP sideband, catpt reference data |

---

## References

- [Linux catpt driver](https://github.com/torvalds/linux/tree/master/sound/soc/intel/catpt) - reference implementation
- [CoolStar Windows SST driver](https://github.com/coolstar/csaudiosstcatpt)
- [Debian firmware-intel-sound](https://packages.debian.org/bookworm/firmware-intel-sound)
- [ArchWiki: Dell XPS 13 (9343)](https://wiki.archlinux.org/title/Dell_XPS_13_(9343))
- [coreboot Broadwell ADSP init](https://github.com/coreboot/coreboot/blob/master/src/soc/intel/broadwell/pch/adsp.c)

---

## License

BSD-3-Clause. See [LICENSE](LICENSE) for details.

---

## Contributing

Contributions welcome! See [CONTRIBUTING.md](CONTRIBUTING.md).
