# 🔊 Intel SST Audio Driver for FreeBSD

[![Build](https://github.com/spagu/acpi_intel_sst/actions/workflows/build.yml/badge.svg)](https://github.com/spagu/acpi_intel_sst/actions/workflows/build.yml)
[![FreeBSD](https://img.shields.io/badge/FreeBSD-15--CURRENT-red?logo=freebsd&logoColor=white)](https://www.freebsd.org/)
[![License](https://img.shields.io/badge/License-BSD--2--Clause-blue.svg)](LICENSE)
[![Platform](https://img.shields.io/badge/Platform-Intel%20Broadwell--U-0071C5?logo=intel&logoColor=white)](https://ark.intel.com/)
[![Status](https://img.shields.io/badge/Status-Development-yellow)](https://github.com/spagu/acpi_intel_sst)
[![PRs Welcome](https://img.shields.io/badge/PRs-welcome-brightgreen.svg)](CONTRIBUTING.md)

> **Experimental FreeBSD kernel module driver for Intel Smart Sound Technology (SST) DSP**
> Enabling analog audio on Broadwell-U platforms with Realtek ALC3263 codec

---

## 📋 Table of Contents

- [Overview](#-overview)
- [Hardware Support](#-hardware-support)
- [Architecture](#-architecture)
- [Current Status](#-current-status)
- [Requirements](#-requirements)
- [Installation](#-installation)
- [Usage](#-usage)
- [Debugging](#-debugging)
- [Project Roadmap](#-project-roadmap)
- [Technical Details](#-technical-details)
- [Implementation Plan](#-implementation-plan)
- [Contributing](#-contributing)
- [License](#-license)

---

## 🎯 Overview

This project provides an **ACPI kernel module driver** for the Intel Smart Sound Technology (SST) Digital Signal Processor found on Intel Broadwell-U platforms. The driver enables analog audio output on systems like the Dell XPS 13 9343, where audio is routed through the DSP via I2S (not traditional HDA).

### The Problem

On Broadwell-U platforms with Realtek ALC3263:
- ❌ Standard `snd_hda` driver only provides HDMI/DP audio
- ❌ Analog audio (speakers/headphones) requires DSP initialization
- ❌ FreeBSD has no SST driver - audio DSP shows as "unknown device"

### The Solution

This driver:
- ✅ Attaches to ACPI device `INT3438` / `INT33C8`
- ✅ Initializes DSP hardware (memory mapping, power management)
- ✅ Prepares foundation for firmware loading and audio playback

---

## 💻 Hardware Support

| Component | Details |
|-----------|---------|
| **Platform** | Intel Broadwell-U (5th Gen Core) |
| **Tested Device** | Dell XPS 13 9343 (2015) |
| **PCI Controller** | Intel Broadwell-U Audio (8086:160c) |
| **ACPI DSP** | INT3438 (`_SB.PCI0.ADSP`) |
| **Codec** | Realtek ALC3263 (VEN_10EC DEV_0282) |
| **Transport** | I2S via DSP (not HDA) |

### Supported ACPI IDs

| ID | Platform |
|----|----------|
| `INT3438` | Intel Broadwell-U |
| `INT33C8` | Intel Haswell |

### Compatible Devices (Realtek ALC3263)

The following devices use the Realtek ALC3263 codec and may benefit from this driver:

| Device | Year | Notes |
|--------|------|-------|
| **Dell XPS 13 9343** | 2015 | ✅ Primary development target |
| Dell XPS 12 9250 | 2016 | Hybrid tablet/laptop |
| Dell XPS 13 9310 | 2020 | 11th Gen Intel |
| Dell XPS 13 9350 | 2015 | Skylake platform |
| Dell XPS 13 9370 | 2018 | 8th Gen Intel |
| Dell XPS 17 9700 | 2020 | 10th Gen Intel |
| Dell XPS 17 9720 | 2022 | 12th Gen Intel |
| Dell XPS 17 9730 | 2023 | 13th Gen Intel |
| Dell Vostro 7590 | 2019 | 9th Gen Intel |

> ⚠️ **Note:** Some models use ALC3263 as USB audio codec (e.g., for docking stations) rather than internal laptop audio. Compatibility may vary depending on the specific audio routing in each model. The driver is primarily designed for I2S/DSP-based implementations found in Broadwell-U platforms.

### Other Broadwell-U Devices (INT3438 SST)

These Intel Broadwell-U (5th Gen) laptops use the same Intel SST DSP architecture and may work with this driver:

| Manufacturer | Model | Display | Notes |
|--------------|-------|---------|-------|
| **Dell** | XPS 13 9343 | 13.3" | ✅ Confirmed |
| Dell | Inspiron 15 7000 | 15.6" | Broadwell-U |
| Dell | Inspiron 17 7000 | 17.3" | Broadwell-U |
| HP | Spectre x360 | 13.3" | Convertible |
| HP | EliteBook 720 G2 | 12.5" | Business |
| HP | EliteBook 750 G2 | 15.6" | Business |
| HP | EliteBook 850 G2 | 15.6" | Business |
| HP | EliteBook Folio 1040 G2 | 14" | Business |
| Lenovo | ThinkPad X250 | 12.5" | Business |
| Lenovo | ThinkPad X1 Carbon (2015) | 14" | Gen 3 |
| Lenovo | ThinkPad Yoga 12 | 12.5" | Convertible |
| Lenovo | Yoga 3 14 | 14" | Consumer |
| Asus | Zenbook UX303LA/LB | 13.3" | Ultrabook |
| Asus | Zenbook UX305LA | 13.3" | Ultrabook |
| Acer | Aspire R13 | 13.3" | Convertible |
| Acer | Aspire S7-393 | 13.3" | Ultrabook |
| LG | Gram 14 (2015) | 14" | Lightweight |
| Fujitsu | Lifebook T725 | 12.5" | Convertible |

> 📝 **Testing needed:** If you have one of these devices running FreeBSD, please test and report results!

---

## 🏗 Architecture

```
┌─────────────────────────────────────────────────────────┐
│                    User Applications                     │
│                    (cat /dev/dsp)                        │
├─────────────────────────────────────────────────────────┤
│                    sound(4) Framework                    │
│                    (/dev/dsp, mixer)                     │
├─────────────────────────────────────────────────────────┤
│                    PCM Glue Driver                       │
│                    (pcmchan interface)                   │
├─────────────────────────────────────────────────────────┤
│                    I2S / SSP Controller                  │
│                    (BCLK, MCLK, data)                    │
├─────────────────────────────────────────────────────────┤
│                    IPC + Topology                        │
│                    (stream control)                      │
├─────────────────────────────────────────────────────────┤
│                    DSP Firmware                          │
│                    (SST / SOF format)                    │
├─────────────────────────────────────────────────────────┤
│  ┌─────────────────────────────────────────────────┐    │
│  │           acpi_intel_sst.ko (This Driver)       │◄───┤ Phase 1-3 ✓
│  │  • ACPI Probe/Attach                            │    │
│  │  • MMIO Resource Allocation                     │    │
│  │  • IRQ Handling                                 │    │
│  │  • DSP Reset/Init                               │    │
│  └─────────────────────────────────────────────────┘    │
├─────────────────────────────────────────────────────────┤
│                    ACPI Subsystem                        │
│                    (INT3438, _PS0, PAUD)                 │
├─────────────────────────────────────────────────────────┤
│                    Hardware                              │
│                    Intel SST DSP + Realtek ALC3263       │
└─────────────────────────────────────────────────────────┘
```

---

## 📊 Current Status

### Implemented (Phase 1-3)

| Feature | Status | Description |
|---------|--------|-------------|
| ACPI Driver Shell | ✅ Done | Basic driver framework |
| Device Probing | ✅ Done | Match on INT3438/INT33C8 |
| Power Management | ✅ Done | D0/D3 + suspend/resume |
| Memory Resources | ✅ Done | MMIO BAR allocation |
| IRQ Resources | ✅ Done | Interrupt handler registered |
| DSP Reset | ✅ Done | Assert reset/stall sequence |
| Register Access | ✅ Done | Thread-safe SHIM read/write |
| Firmware Loading | ✅ Done | SST binary format parser |
| IPC Protocol | ✅ Done | Host-DSP mailbox communication |
| DSP Boot | ✅ Done | Load FW, release reset, wait ready |

### Planned (Phase 4-5)

| Feature | Status | Description |
|---------|--------|-------------|
| Topology Loading | ⏳ TODO | Audio pipeline config |
| I2S Controller | ⏳ TODO | SSP driver for codec |
| PCM Integration | ⏳ TODO | sound(4) framework |
| Mixer Support | ⏳ TODO | Volume control |

---

## 📦 Requirements

### Build Requirements

- FreeBSD 15-CURRENT (or 14.x with modifications)
- FreeBSD source tree at `/usr/src`
- GCC or Clang compiler
- Make utility

### Runtime Requirements

- Intel Broadwell-U or Haswell platform
- ACPI tables with INT3438/INT33C8 device
- (Future) SST firmware files in `/boot/firmware/`

---

## 🔧 Installation

### Building from Source

```bash
# Clone the repository
git clone https://github.com/spagu/acpi_intel_sst.git
cd acpi_intel_sst

# Build the kernel module
make

# Verify build output
ls -la acpi_intel_sst.ko
```

### Loading the Module

```bash
# Load the module (requires root)
sudo kldload ./acpi_intel_sst.ko

# Verify loading
kldstat | grep sst

# Check dmesg for driver messages
dmesg | grep -i sst
```

### Unloading the Module

```bash
sudo kldunload acpi_intel_sst
```

---

## 📖 Usage

### Verify Device Detection

```bash
# Check if device is attached
devinfo -v | grep -A5 sst

# View device resources
pciconf -lv | grep -A5 160c

# Check ACPI device status
acpidump -dt | grep -i INT3438
```

### Expected Output (dmesg)

```
acpi_intel_sst0: <Intel Broadwell-U Audio DSP (SST)> on acpi0
acpi_intel_sst0: MMIO Base: 0xf0500000, Size: 0x8000
acpi_intel_sst0: IRQ: 24
acpi_intel_sst0: Register Dump:
acpi_intel_sst0:   CSR : 0x00000003
acpi_intel_sst0:   IPCX: 0x00000000
acpi_intel_sst0: DSP in Reset/Stall state. CSR: 0x00000003
acpi_intel_sst0: Intel SST DSP attached successfully (Phase 1+2)
```

---

## 🔍 Debugging

### Enable Verbose Logging

```bash
# Set debug flags (if implemented)
sysctl hw.acpi.verbose=1

# Monitor kernel messages
tail -f /var/log/messages
```

### Register Dump

The driver outputs key registers on attach:

| Register | Offset | Description |
|----------|--------|-------------|
| CSR | 0x00 | Control/Status Register |
| PISR | 0x08 | Platform Interrupt Status |
| IMRX | 0x28 | Interrupt Mask (RX) |
| IPCX | 0x38 | IPC Control |

### Troubleshooting

| Issue | Solution |
|-------|----------|
| "unknown device" | Driver not loaded or ACPI ID mismatch |
| "0xFFFFFFFF registers" | Device powered down, check PAUD power resource |
| "Failed to allocate memory" | ACPI _CRS resource issue |
| Module won't load | Check FreeBSD version compatibility |

---

## 🗺 Project Roadmap

```
Phase 0 ✓ - Preparation
├── Repository setup
├── Build system (Makefile)
└── Documentation

Phase 1 ✓ - ACPI Driver (MVP-1)
├── ACPI probe/attach
├── Resource allocation
└── Power management

Phase 2 ✓ - DSP Init (MVP-2)
├── MMIO mapping
├── Reset sequence
└── Register access

Phase 3 ✓ - IPC & Firmware
├── Firmware loader (SST binary format)
├── IPC protocol (mailbox communication)
├── DSP boot sequence
└── Interrupt handler

Phase 4 ⏳ - I2S/SSP
├── I2S controller
├── Clock configuration
└── Data streaming

Phase 5 ⏳ - Audio Integration
├── sound(4) PCM driver
├── Mixer support
└── Jack detection
```

---

## 🔬 Technical Details

### Why Not HDA?

On Broadwell-U with Realtek ALC3263:

- The `snd_hda` driver handles HDMI/DP audio only
- Analog audio is routed through Intel SST DSP
- The codec communicates via **I2S**, not HDA
- DSP requires firmware and IPC initialization

### DSP Boot Sequence

```
1. Set D0 Power State (_PS0)
2. Map MMIO Resources
3. Assert Reset + Stall (CSR)
4. Load Firmware (TODO)
5. Release Reset (keep Stall)
6. Start Firmware (TODO)
7. Clear Stall → DSP Running
8. Initialize IPC (TODO)
```

### SHIM Registers

| Register | Offset | Purpose |
|----------|--------|---------|
| CSR | 0x00 | DSP Control/Status |
| PISR | 0x08 | Platform Interrupt Status |
| PIMR | 0x10 | Platform Interrupt Mask |
| ISRX | 0x18 | IPC Status RX |
| ISRD | 0x20 | IPC Status Done |
| IMRX | 0x28 | IPC Mask RX |
| IMRD | 0x30 | IPC Mask Done |
| IPCX | 0x38 | IPC Command |
| IPCD | 0x40 | IPC Data |

---

## 📋 Implementation Plan

Detailed implementation plan for Phase 2-3 is available in [docs/IMPLEMENTATION_PLAN.md](docs/IMPLEMENTATION_PLAN.md).

**Next Steps:**
- Phase 4: I2S/SSP controller, clock configuration
- Phase 5: sound(4) PCM driver integration

---

## 🤝 Contributing

Contributions are welcome! Please see [CONTRIBUTING.md](CONTRIBUTING.md) for guidelines.

### Areas Needing Help

- 🔴 I2S/SSP controller driver
- 🔴 sound(4) PCM integration
- 🟡 Testing on different Broadwell-U devices
- 🟢 Documentation improvements

---

## 📄 License

This project is licensed under the BSD-2-Clause License. See [LICENSE](LICENSE) for details.

```
Copyright (c) 2026
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:
...
```

---

## 🙏 Acknowledgments

- FreeBSD Audio Developers
- Linux SOF (Sound Open Firmware) Project
- Intel for partial documentation

## 📚 References

- [Intel Broadwell U/Y Platform Documentation](https://www.intel.com/content/www/us/en/products/platforms/details/broadwell-u-y/docs.html)
- [Intel SST Audio Drivers](https://www.intel.com/content/www/us/en/products/platforms/details/broadwell-u-y/downloads.html)
- [Broadwell Ultrabooks List](https://www.ultrabookreview.com/5165-broadwell-ultrabooks/)
- [Linux SOF Project](https://github.com/thesofproject/linux)

---

<div align="center">

**Made with ❤️ for the FreeBSD Community**

[Report Bug](https://github.com/spagu/acpi_intel_sst/issues) · [Request Feature](https://github.com/spagu/acpi_intel_sst/issues)

</div>
