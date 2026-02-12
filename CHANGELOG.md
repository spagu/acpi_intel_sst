# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [Unreleased]

### Research Findings (v0.6.0)
- **IOBP Sideband Interface Discovery** - identified the PCH mechanism controlling ADSP BAR0
  - IOBP registers at RCBA+0x2330/0x2334/0x2338/0x233A
  - PCICFGCTL register at IOBP address 0xd7000500 controls PCI/ACPI mode
  - PCICD bit (bit 0) hides device from PCI enumeration
  - SPCBAD bit (bit 7) may control BAR0 memory decode
  - PMCTL at 0xd70001e0 controls power management
  - Source: coreboot `src/soc/intel/broadwell/pch/adsp.c`
- **LPSS Memory Architecture Mapped** - full address space analysis
  - 0xFE100000-0xFE106FFF: LPSS fabric (ALIVE - private configs + BAR0s)
  - 0xFE000000-0xFE0FFFFF: Separate 1MB ADSP decode window (DEAD)
  - SDMA BAR0 at 0xFE101000 responds (returns 0x00000000)
  - I2C0 BAR0 at 0xFE103000 alive after D3-to-D0 transition
  - I2C0 IC_COMP_TYPE = 0x44570140 (DesignWare confirmed!)
- **FD Register Bit Positions Corrected** - from DSDT OperationRegion analysis
  - DSDT field layout: skip(1), ADSD(1), SATD(1), SMBD(1), HDAD(1)
  - Bit 1 = ADSD (was incorrectly at bit 0)
  - Bit 4 = HDAD (was incorrectly at bit 3)
  - Previous code accidentally disabled SMBus; FD restored to 0x00368011
- **GNVS Variables Read** - BIOS NVS area decoded
  - OSYS=0x07DD (Win8.1), S0ID=1, ANCS=0, SMD0=1 (PCI mode)
  - ADB0=0xFE000000, SB10=0xFE102000
- **DSDT _CRS Analysis** - BAR addresses from NVS variables (ADB0/ADB1/ADI0)
- **WPT Memory Layout Correction** - from Linux catpt spec
  - DRAM at 0x000000, IRAM at 0x0A0000, SHIM at 0x0FB000
  - Differs from LPT (Haswell) layout used in earlier code
- **Firmware Init Sequence Documented** - from coreboot source
  - 11-step init including IOBP writes, PSF snoop, D3hot as final step

### Added
- **Brute Force Memory Scanner** - when BAR0 returns 0xFFFFFFFF, scan memory regions
  - Scans common LPSS addresses (0xf0000000-0xfe000000 range)
  - Looks for SST DevID (0x9CB68086) or firmware magic ($SST)
  - Scans around working BAR1 address to find alternative mappings
  - Helps identify if DSP is mapped at different address than ACPI reports
- **Complete IPC Stream Allocation API** (`sst_ipc.c`) - full DSP stream management
  - `sst_ipc_alloc_stream()` - allocate audio stream on DSP
  - `sst_ipc_free_stream()` - release DSP stream resources
  - `sst_ipc_stream_pause/resume/reset()` - stream control
  - `sst_ipc_stream_set_params()` - set volume, mute per stream
  - `sst_ipc_stream_get_position()` - get current playback position
  - `sst_ipc_set_mixer/get_mixer()` - master mixer control
  - `sst_ipc_set_dx()` - power state management (D0/D3)
- **Stream Allocation Structures** (`sst_ipc.h`)
  - `struct sst_audio_format` - sample rate, bit depth, channels, format
  - `struct sst_alloc_stream_req/rsp` - stream allocation request/response
  - `struct sst_stream_params` - per-stream volume control
  - `struct sst_stream_position` - playback position tracking
  - `struct sst_mixer_params` - master mixer parameters
  - `struct sst_dx_state` - power state structure
- **Additional IPC Message Types**
  - `SST_IPC_GLBL_REQUEST_DUMP` - debug dump
  - `SST_IPC_GLBL_SET_DEVICE_FORMATS` - device format configuration
  - `SST_IPC_GLBL_SET_DX` / `ENTER_DX_STATE` - power management
  - `SST_IPC_GLBL_NOTIFICATION` - DSP notifications
  - Stream messages: RESET, MUTE, UNMUTE
  - Notification types: POSITION_CHANGED, GLITCH, UNDERRUN, OVERRUN
  - Stream types: RENDER, CAPTURE, SYSTEM, LOOPBACK
  - Audio format IDs: PCM, MP3, AAC, WMA
- **PCM DSP Stream Integration** (`sst_pcm.c`)
  - PCM trigger now allocates DSP stream via IPC
  - Stream ID tracking per channel
  - DSP stream pause/resume on playback start/stop
  - Automatic stream cleanup on channel close
  - Mixer updates propagate to DSP via IPC

### Changed
- **DSDT Patch: SST Mode Enabled** - changed patch to ENABLE ADSP instead of disabling
  - Root cause: FreeBSD doesn't set ACPI S0ID or ANCS variables
  - Original _STA method required S0ID=1 or ANCS=1 to return 0x0F (enabled)
  - New patch unconditionally returns 0x0F if ADB0 (BAR0 address) is configured
  - HDA mode instructions remain available in `acpi/README.md`
  - This change is required for SST audio to work on Dell XPS 13 9343

### Fixed
- **Critical: SRAM power gate logic was inverted** - ISRAMPGE/DSRAMPGE bits must be SET, not CLEARED
  - Linux catpt driver sets ISRAMPGE_MASK | DSRAMPGE_MASK to power ON SRAM
  - Our code was clearing these bits, which powered OFF the SRAM
  - This caused BAR0 (DSP memory) to return 0xFFFFFFFF
  - VDRTCTL0 should be 0x000FFFFF (all SRAM enabled), not 0x00000003
- **Critical: WPT (Broadwell-U) power-up sequence** - VDRTCTL0 register bits were using LPT (Haswell) positions
  - WPT uses bits 0-1 for D3PGD/D3SRAMPGD (not bits 8/16 like LPT)
  - WPT uses bits 2-11 for ISRAMPGE, bits 12-19 for DSRAMPGE
  - WPT places APLLSE at VDRTCTL2 bit 31 (not VDRTCTL0 bit 0 like LPT)

### Added
- **ACPI `_INI` method call** - calls ACPI initialization method during device attach
- **HDA Mode Alternative (DSDT Override)** - documented workaround for Dell XPS 13 9343
  - Discovery: Dell XPS 13 9343 uses dual-mode Realtek ALC3263 (HDA + I2S)
  - BIOS uses ACPI `OSYS` variable to select audio mode at cold boot
  - Linux has `CONFIG_ACPI_REV_OVERRIDE_POSSIBLE` with `acpi_rev_override` parameter
  - FreeBSD has NO equivalent - must use DSDT override method
  - Complete DSDT modification instructions in README.md
  - DSDT patching resources from GitHub repos (rbreaves, major)
  - If HDA mode works, standard `hdac` driver handles audio (no SST driver needed)
- **ACPI DSDT Patch Files** - `acpi/` folder with ready-to-use patches
  - `acpi/DSDT.dat` - Original DSDT binary from Dell XPS 13 9343
  - `acpi/DSDT.dsl` - Decompiled original DSDT (ASL source)
  - `acpi/DSDT_patched.dsl` - Patched DSDT that disables ADSP to force HDA mode
  - `acpi/README.md` - Installation instructions for FreeBSD
  - Patch modifies `_SB.PCI0.ADSP._STA` to always return Zero (disabled)
- **Known Issues documentation** - documented Dell XPS 13 9343 BAR0 memory access issue
  - Comprehensive troubleshooting guide in README.md
  - HDA controller disable workaround (`hint.hdac.0.disabled="1"`)
  - ig4iic0 (I2C) also fails - entire LPSS memory region inaccessible
  - Combined loader.conf workarounds for LPSS devices
  - ACPI `_OSI` spoofing workaround (`hw.acpi.osi="Windows 2015"`) - **TESTED: Did NOT help**
  - Comparison: How Linux/Windows handle LPSS (intel-lpss driver, Intel Serial IO)
- **Technical Findings Document** - `docs/TECHNICAL_FINDINGS.md`
  - Complete investigation timeline and methodology
  - Register dumps and power sequence analysis
  - Linux vs Windows LPSS handling comparison
  - HDA vs I2S mode switching via ACPI `_REV`
  - DSDT override solution with step-by-step instructions
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
- **Topology Loading** (`sst_topology.c`) - dynamic audio pipeline configuration
  - `struct sst_pipeline` - audio pipeline with modules
  - `struct sst_widget` - PCM, PGA, MUX, MIXER, DAI widgets
  - `struct sst_route` - widget interconnections
  - `sst_topology_init/fini()` - lifecycle management
  - `sst_topology_load_default()` - Broadwell-U default pipelines
  - `sst_topology_create_pipeline()` - allocate pipeline on DSP
  - `sst_topology_start/stop_pipeline()` - pipeline control
  - Default playback pipeline: PCM0 -> PGA0 -> SSP0 (speakers)
  - Default capture pipeline: SSP1 -> PGA1 -> PCM1 (microphone)
  - Max 8 pipelines, 32 widgets, 64 routes per topology
- **Multi-stream Support** (`sst_pcm.c`) - multiple simultaneous audio streams
  - 4 simultaneous playback streams (SST_PCM_MAX_PLAY)
  - 2 simultaneous capture streams (SST_PCM_MAX_REC)
  - Dynamic channel allocation from pool
  - Per-stream DMA buffer and DSP stream allocation
  - Stream allocation bitmap tracking
  - Master mixer applies volume to all active streams
  - Independent stream lifecycle (alloc/free per open/close)

### Planned
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
