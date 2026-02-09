# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Added
- GitHub Actions CI workflow for build validation (FreeBSD 14.1, 14.2)
- Code style linting (tabs, line length, trailing whitespace)
- Compatible devices list (Dell XPS 12/13/17, Vostro 7590)
- Broadwell-U devices compatibility table (HP, Lenovo, Asus, Acer, LG, Fujitsu)
- References section with Intel documentation links

### Planned
- sound(4) PCM integration
- Mixer support

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

[Unreleased]: https://github.com/spagu/acpi_intel_sst/compare/v0.4.0...HEAD
[0.4.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.3.0...v0.4.0
[0.3.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.2.0...v0.3.0
[0.2.0]: https://github.com/spagu/acpi_intel_sst/compare/v0.1.0...v0.2.0
[0.1.0]: https://github.com/spagu/acpi_intel_sst/releases/tag/v0.1.0
