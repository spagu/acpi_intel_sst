# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Fixed
- **Critical: WPT (Broadwell-U) power-up sequence** - VDRTCTL0 register bits were using LPT (Haswell) positions
  - WPT uses bits 0-1 for D3PGD/D3SRAMPGD (not bits 8/16 like LPT)
  - WPT uses bits 2-11 for ISRAMPGE, bits 12-19 for DSRAMPGE
  - WPT places APLLSE at VDRTCTL2 bit 31 (not VDRTCTL0 bit 0 like LPT)
  - This was causing BAR0 memory to return 0xFFFFFFFF

### Added
- **ACPI `_INI` method call** - calls ACPI initialization method during device attach
- **Known Issues documentation** - documented Dell XPS 13 9343 BAR0 memory access issue
  - Comprehensive troubleshooting guide in README.md
  - HDA controller disable workaround (`hint.hdac.0.disabled="1"`)
  - ig4iic0 (I2C) also fails - entire LPSS memory region inaccessible
  - Combined loader.conf workarounds for LPSS devices
  - ACPI `_OSI` spoofing workaround (`hw.acpi.osi="Windows 2015"`)
  - Comparison: How Linux/Windows handle LPSS (intel-lpss driver, Intel Serial IO)
- **Technical Findings Document** - `docs/TECHNICAL_FINDINGS.md`
  - Complete investigation timeline and methodology
  - Register dumps and power sequence analysis
  - Linux vs Windows LPSS handling comparison
  - Root cause hypothesis and conclusions
- **Extended PCI config register support** - additional registers from Linux catpt driver
  - IMC (0xE4): Interrupt Mask Clear register
  - IMD (0xEC): Interrupt Mask Set register (default: 0x7FFF0003)
  - IPCC (0xE0): IPC Clear register
  - IPCD (0xE8): IPC Set register
  - HMDC: Host Memory DMA Control register
  - LTRC: Low Trunk Clock register
- **PCH RCBA Function Disable check** - checks FD2 register at RCBA+0x3428 for ADSP disable bit
  - If ADSPD (bit 1) is set in FD2, ADSP is disabled at PCH level
  - Automatically attempts to clear the disable bit if detected
- **WPT-specific register definitions** in `sst_regs.h`
  - `SST_WPT_VDRTCTL0_*` defines for Wildcat Point (Broadwell-U)
  - `SST_LPT_VDRTCTL0_*` defines for Lynx Point (Haswell)
  - PMCS power management control/status register defines
  - LPSS private register definitions
- **Firmware documentation** - comprehensive guide for obtaining Intel SST firmware
  - Firmware location: `/boot/firmware/intel/IntcSST2.bin`
  - Direct download from Debian package on FreeBSD
  - Multiple acquisition methods (Linux packages, Windows drivers, community repos)
  - Firmware compatibility table with platform warnings
  - Clear warning about using wrong firmware (fw_sst_0f28.bin is for Baytrail, not Broadwell!)
  - Verification instructions with file size check
- GitHub Actions CI workflow for build validation (FreeBSD 14.1, 14.2)
- Code style linting (tabs, line length, trailing whitespace)
- Compatible devices list (Dell XPS 12/13/17, Vostro 7590)
- Broadwell-U devices compatibility table (HP, Lenovo, Asus, Acer, LG, Fujitsu)
- References section with Intel documentation links
- **Jack Detection** (`sst_jack.c`) - headphone/microphone auto-detection
  - Polling-based GPIO monitoring
  - Debounce logic for stable detection
  - Automatic speaker/headphone switching
  - Sysctl interface for status and control
  - Statistics tracking (insertion counts)

### Planned
- Topology loading (dynamic audio pipeline)
- ALSA compatibility layer

---

## [0.5.0] - 2026-02-09

### Added
- **Phase 5: sound(4) PCM Integration**
- PCM driver (`sst_pcm.c`) - FreeBSD sound(4) framework integration
- Playback and capture channel support
- DMA buffer allocation with bus_dma(9)
- Mixer controls (volume, mute)
- Device registration at `/dev/dsp`

### Technical
- Channel methods: init, free, setformat, setspeed, trigger, getptr
- Supported formats: S16_LE, S24_LE, S32_LE (stereo)
- Sample rates: 8kHz - 192kHz
- Block-based DMA with circular buffers
- Mixer: PCM and master volume controls

### Changed
- Driver version bumped to 0.5.0
- MODULE_VERSION updated to 5
- Added MODULE_DEPEND for sound subsystem

---

## [0.4.0] - 2026-02-09

### Added
- **Phase 4: I2S/SSP & DMA Controller Implementation**
- SSP (I2S) controller driver (`sst_ssp.c`) - Serial Synchronous Port
- DMA controller driver (`sst_dma.c`) - 8-channel DMA engine
- I2S audio format configuration (sample rate, bits, channels)
- Master/Slave mode support for SSP
- DMA channel allocation and management
- Circular buffer support for audio streaming
- DMA completion callbacks
- SSP register dump for debugging

### Technical
- SSP supports I2S, Left-Justified, DSP-A/B formats
- Configurable BCLK/MCLK dividers
- 8 DMA channels with hardware handshaking
- Memory-to-Peripheral and Peripheral-to-Memory transfers
- FIFO threshold configuration

### Changed
- Driver version bumped to 0.4.0
- MODULE_VERSION updated to 4
- Interrupt handler extended for DMA

---

## [0.3.0] - 2026-02-09

### Added
- **Phase 3: Firmware & IPC Implementation**
- Firmware loader (`sst_firmware.c`) - parses Intel SST binary format
- IPC protocol (`sst_ipc.c`) - host-DSP communication
- Interrupt handler with proper ISR registration
- Suspend/resume power management handlers
- Mutex protection for register access
- Register access helper functions (`sst_shim_read/write/update_bits`)
- DSP boot sequence with firmware loading
- Get firmware version IPC command
- Separate register definitions (`sst_regs.h`)

### Changed
- Driver version bumped to 0.3.0
- Modular architecture with separate header/source files
- Added MODULE_DEPEND for firmware subsystem
- Improved error handling throughout

### Technical
- SST firmware header parsing ($SST signature)
- Module and block header parsing ($MOD signature)
- IRAM/DRAM block loading to DSP memory
- IPC mailbox communication (Host->DSP, DSP->Host)
- Condition variable based IPC wait mechanism

---

## [0.2.0] - 2026-02-09

### Added
- DSP initialization with reset/stall sequence
- SHIM register access (CSR, PISR, IMRX, IPCX)
- Interrupt masking during initialization
- Hardware validation checks (0xFFFFFFFF detection)
- MMIO region size validation
- Comprehensive test script (`tests/test_module.sh`)
- Professional documentation with badges and UTF-8 icons
- CONTRIBUTING.md guidelines
- LICENSE file (BSD-2-Clause)

### Changed
- Improved error handling with goto-based cleanup pattern
- Added SPDX license identifier to source files
- Enhanced logging with driver version information
- Better resource initialization order

### Security
- Added bounds checking for MMIO region size
- Proper NULL initialization of resource pointers
- Validated register reads before proceeding

---

## [0.1.0] - 2026-02-08

### Added
- Initial ACPI driver framework
- Device probing for INT3438 and INT33C8 ACPI IDs
- Memory resource (MMIO) allocation
- IRQ resource allocation
- Power management support (_PS0/_PS3)
- Basic Makefile for FreeBSD kernel module
- Initial README documentation

### Technical Details
- Driver attaches to `acpi0` bus
- Supports Intel Broadwell-U and Haswell platforms
- Uses standard FreeBSD ACPI driver model

---

## Types of Changes

- `Added` for new features
- `Changed` for changes in existing functionality
- `Deprecated` for soon-to-be removed features
- `Removed` for now removed features
- `Fixed` for any bug fixes
- `Security` for vulnerability fixes

[Unreleased]: https://github.com/spagu/acpi_intel_sst/compare/v0.5.0...HEAD
[0.5.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.4.0...v0.5.0
[0.4.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/spagu/acpi_intel_sst/releases/tag/v0.1.0
