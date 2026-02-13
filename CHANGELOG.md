# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.1.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [0.29.0] - 2026-02-13

### CRITICAL FIX: WPT/Broadwell DSP Ready Uses DONE Bit, Not BUSY

- **Root Cause Found**: DSP ready detection missed DONE bit!
  - WPT (Broadwell) DSP uses DONE (bit 30) for ready notification
  - Code was only checking BUSY (bit 31) for ready
  - IPCD=0x44200209 has DONE=1 → DSP WAS signaling ready!

- **Fix Applied to `sst_ipc.c`**:
  - `sst_ipc_poll_ready()`: Now also sets `sc->ipc.ready = true` on DONE
  - `sst_ipc_intr()`: Same fix for interrupt mode
  - First DONE after boot is treated as ready notification on WPT

- **Code Change**:
  ```c
  /* BEFORE: Only checked BUSY for ready */
  if (ipcd & SST_IPC_DONE) {
      /* Processed DONE but didn't set ready! */
  }

  /* AFTER: Also set ready on first DONE */
  if (ipcd & SST_IPC_DONE) {
      if (!sc->ipc.ready) {
          sc->ipc.ready = true;
          device_printf(sc->dev,
              "DSP signaled ready via DONE (polled): IPCD=0x%08x\n", ipcd);
      }
  }
  ```

- **Expected Result**: DSP boot should succeed, fw.state becomes RUNNING,
  PCM registration should proceed

---

## [0.28.0] - 2026-02-13

### CRITICAL FIX: pcm_register() API - Missing pcm_init() Call

- **Root Cause Found**: `pcm_init()` was never called before `pcm_register()`!
  - FreeBSD 15 sound(4) API requires initialization before registration
  - Code was jumping directly to `pcm_register()` without setting up devinfo

- **FreeBSD 15 sound(4) API Sequence**:
  1. `pcm_init(dev, devinfo)` - Initialize PCM with driver data pointer
  2. `pcm_addchan(dev, dir, class, devinfo)` - Add playback/capture channels
  3. `pcm_register(dev, status)` - Finalize registration with status string

- **Fix Applied to `sst_pcm.c`**:
  ```c
  /* BEFORE (wrong order): */
  sc->pcm.dev = device_add_child(sc->dev, "pcm", -1);
  error = pcm_register(sc->pcm.dev, status);
  pcm_addchan(...);  /* Called AFTER register */

  /* AFTER (correct order): */
  pcm_init(sc->dev, sc);              /* 1. Initialize with devinfo */
  pcm_addchan(sc->dev, PCMDIR_PLAY, &sst_chan_class, sc);  /* 2. Add channels */
  pcm_addchan(sc->dev, PCMDIR_REC, &sst_chan_class, sc);
  error = pcm_register(sc->dev, status);  /* 3. Finalize registration */
  ```

- **Also Fixed**:
  - Removed unnecessary `device_add_child()` - not needed for sound(4)
  - Removed unused `sc->pcm.dev` field from `struct sst_pcm`
  - Updated `sst_pcm_unregister()` to use `sc->dev`

- **Expected Result**: `/dev/dsp` should now appear and `/dev/sndstat` should list device

---

## [0.27.0] - 2026-02-12

### CRITICAL FIX: IPC Protocol - Check BUSY bit for DSP Ready Detection

- **Root Cause Found**: IPC ready detection was checking the wrong bit!
  - Code was checking `SST_IPC_DONE` (bit 30) in IPCD
  - DSP signals ready by setting `SST_IPC_BUSY` (bit 31) in IPCD
  - IPCD=0x84200209 showed BUSY=1 (bit 31 set), but code missed it

- **IPC Protocol Clarification**:
  - **IPCD** (DSP->Host): DSP sets **BUSY** to send notification/message to Host
  - **IPCX** (Host->DSP): Host sets **BUSY** to send command to DSP
  - To acknowledge DSP message: clear BUSY, set DONE in IPCD

- **Fix Applied to `sst_ipc.c`**:
  - `sst_ipc_poll_ready()`: Now checks BUSY in IPCD first (DSP message detection)
  - `sst_ipc_intr()`: Fixed interrupt handler to check BUSY before DONE
  - Proper acknowledgment: `(ipcd & ~SST_IPC_BUSY) | SST_IPC_DONE`

- **Code Change**:
  ```c
  /* BEFORE (wrong): */
  if (ipcd & SST_IPC_DONE) { ... }

  /* AFTER (correct): */
  if (ipcd & SST_IPC_BUSY) {
      /* DSP sent message - acknowledge */
      sst_shim_write(sc, SST_SHIM_IPCD,
          (ipcd & ~SST_IPC_BUSY) | SST_IPC_DONE);
      if (!sc->ipc.ready) {
          sc->ipc.ready = true;
          ...
      }
  }
  ```

- **Expected Result**: DSP ready detection should now work correctly with
  IPCD=0x84200209 (BUSY bit set = DSP sent ready notification)

---

## [0.26.0] - 2026-02-12

### CRITICAL FIX: Correct SHIM Offset for WPT/Broadwell

- **Root Cause Found**: SHIM offset was wrong for WPT (Broadwell-U)!
  - Old offset: 0xC0000 (for Skylake+)
  - Correct offset for WPT: **0xE7000** (from Linux catpt driver `wpt_spec`)
  - BAR1+0x80 is LPSS private status register, NOT DSP control!

- **Fix**: Updated SHIM offset and access
  - `SST_SHIM_OFFSET` changed from 0xC0000 to **0xE7000**
  - `sst_shim_read/write` reverted to use BAR0 (mem_res) + offset
  - Back to using `SST_SHIM_CSR` (offset 0x00 within SHIM)

- **Key Insight from Linux catpt driver**:
  ```c
  static const struct catpt_spec wpt_spec = {
      .shim_offset = 0xe7000,  /* WPT SHIM offset in BAR0 */
      ...
  };
  ```

- **Memory Layout for WPT/Broadwell-U**:
  | Region | BAR | Offset | Size |
  |--------|-----|--------|------|
  | IRAM | BAR0 | 0x00000 | 80KB |
  | DRAM | BAR0 | 0x80000 | 160KB |
  | SHIM | BAR0 | **0xE7000** | 4KB |
  | LPSS Private | BAR1 | 0x00 | 4KB |

---

## [0.25.0] - 2026-02-12

### Attempted Fix: BAR1 for SHIM (Incorrect)

- Tried using BAR1 for SHIM access - didn't work
- BAR1+0x80 is LPSS private status, read-only (`0x40030001`)
- Writes to CSR2 at BAR1+0x80 have no effect

---

## [0.24.2] - 2026-02-12

### Added: IPC Polling Mode and Boot Diagnostics

- **Added**: IPC polling mode when IRQ unavailable
  - `sst_ipc_poll_ready()` - polls IPCD/IPCX registers for DSP ready
  - `sst_ipc_wait_ready()` now uses polling when `sc->irq_res == NULL`
  - Prints CSR/ISR/IPCD values every second during boot wait

- **Added**: DSP boot sequence diagnostics
  - Prints CSR value at each boot step (initial, RST+STALL, clear RST, clear STALL)
  - Helps identify where boot sequence fails

---

## [0.24.1] - 2026-02-12

### Fixed: Firmware Format Detection

- **Fixed**: Firmware parser now handles IntcSST2.bin with `$SST` module signatures
  - IntcSST2.bin has format=254 (< 256) but uses `$SST` for modules instead of `$MOD`
  - Parser now detects nested `$SST` headers and falls back to raw binary loading
  - Raw binary loading splits firmware: first half to IRAM, second half to DRAM

### Progress: DSP Boot Attempted

- **Working**:
  - All subsystems initialize successfully
  - Jack detection works (Headphone inserted detected!)
  - Firmware loads to DSP memory (IRAM=81920, DRAM=163840 bytes)
  - DSP boot sequence starts

- **Blocked**: DSP boot timeout (error 60 = ETIMEDOUT)
  - DSP never signals ready via IPC
  - Possible causes:
    1. **IRQ not allocated** - "Failed to allocate IRQ" - IPC relies on interrupts
    2. **Reset bit stuck** - Reg 0x80 = 0x40030001, bit 0 won't clear
    3. **Wrong firmware split** - 50/50 IRAM/DRAM split may be incorrect
    4. **Entry point** - Using 0x0, might need different address

- **Key Register State** (BAR1 / LPSS Private):
  | Register | Offset | Value | Notes |
  |----------|--------|-------|-------|
  | VDRTCTL0 | 0xA0 | 0x00000000 | Power OK |
  | VDRTCTL2 | 0xA8 | 0x80000FFF | Clock config OK |
  | DSP Control | 0x80 | 0x40030001 | **Bit 0 stuck** (reset?) |

- **Next Steps**:
  1. Investigate IRQ allocation failure
  2. Find correct way to release DSP from reset (reg 0x80)
  3. Analyze actual IntcSST2.bin firmware layout

---

## [0.24.0] - 2026-02-12

### Full DSP Initialization in PCI Mode

- **Added**: Complete DSP initialization sequence to `sst_pci_attach()`
  - IPC subsystem initialization (`sst_ipc_init`)
  - Firmware subsystem initialization (`sst_fw_init`)
  - SHIM configuration for Broadwell-U (catpt) mode
  - DMA subsystem initialization (`sst_dma_init`)
  - SSP (I2S) subsystem initialization (`sst_ssp_init`)
  - PCM subsystem initialization (`sst_pcm_init`)
  - Topology (audio pipeline) initialization (`sst_topology_init`)

- **Added**: Firmware loading and DSP boot in PCI mode
  - `sst_fw_load()` - Loads firmware from `/boot/firmware/intel/IntcSST2.bin`
  - `sst_fw_boot()` - Boots DSP with loaded firmware
  - `sst_ipc_get_fw_version()` - Gets firmware version
  - `sst_topology_load_default()` - Loads default audio topology

- **Added**: Audio subsystem registration
  - `sst_pcm_register()` - Registers PCM device when firmware is running
  - `sst_jack_init()` / `sst_jack_enable()` - Jack detection

- **Fixed**: `sst_pci_detach()` now properly cleans up all subsystems
  - Calls all `*_fini()` functions in reverse order
  - Properly nullifies released resources

---

## [0.23.0] - 2026-02-12

### Cleanup: Removed SRAM Checkpoint Diagnostics

- Removed 38 `sst_check_sram_immediate()` checkpoint calls
- Driver code cleaned up for production readiness
- Kept `sst_check_sram_immediate()` function for future debugging if needed

---

## [0.22.0] - 2026-02-12

### SUCCESS: SRAM Survives Driver Load!

- **Problem Solved**: Dual driver conflict was causing SRAM reset
  - Module had two drivers: `acpi_intel_sst` (ACPI) and `sst_pci` (PCI)
  - Something between ACPI attach completion and PCI probe reset SRAM
  - Disabling ACPI driver fixes the issue!

- **Solution**: PCI driver only mode
  - Commented out `DRIVER_MODULE(acpi_intel_sst, acpi, ...)`
  - Commented out `sst_driver` and `sst_methods` (unused)
  - Only PCI driver remains active

- **Verified Working**:
  ```
  sst: [PCI_PROBE] SRAM[0]=0xc31883d6 CTRL=0xdf000078 => ALIVE
  sst: [ATTACH_ENTRY] SRAM[0]=0xc31883d6 CTRL=0xdf000078 => ALIVE
  sst: [SRAM_ENABLE_ENTRY] SRAM[0]=0xc31883d6 CTRL=0xdf000078 => ALIVE
  ```

- **Working Workflow**:
  1. Pre-activate SRAM via `dd` (rising edge trigger)
  2. Load driver: `kldload acpi_intel_sst`
  3. Driver sees SRAM as ALIVE and can work with DSP memory

---

## [0.21.0] - 2026-02-12

### Investigation: SRAM Reset During Driver Load

- **Critical Discovery**: Pre-activated SRAM is reset before driver code runs
  - User activates SRAM via `dd` → verified working (reads `0xc31883d6`)
  - User runs `kldload acpi_intel_sst`
  - By the time driver's first line executes, SRAM is dead (`0xffffffff`)

- **Hypothesis**: FreeBSD PCI subsystem resets device during attach
  - Could be: PCI probe, BAR setup, power state transition
  - Could be: ACPI _PS0 method, _DSM method, or power resource
  - Need to identify exact point where reset occurs

- **Diagnostic Approach**: Added checkpoint-based SRAM monitoring
  - `sst_check_sram_immediate()` - direct SRAM status check without device setup
  - Checkpoints at: ATTACH_ENTRY, AFTER_SOFTC, AFTER_MTX_INIT, BEFORE_PCI_READ, AFTER_PCI_READ, BEFORE_PCI_WRITE, AFTER_PCI_WRITE
  - Will identify exact operation that causes SRAM reset

- **Technical Findings**:
  - SRAM control register at BAR0+0xFB000 (physical 0xDF8FB000)
  - SRAM alive state: reads non-0xFFFFFFFF from SRAM[0], CTRL shows 0x78663178 or similar
  - SRAM dead state: reads 0xFFFFFFFF from SRAM[0], CTRL shows 0x8480040e

---

## [0.20.0] - 2026-02-12

### CONCLUSION: Manual SRAM Activation Required

- **Root Cause Identified**: Kernel driver writes cannot trigger SRAM hardware
  - Atomic 32-bit writes: values written correctly, but hardware ignores
  - Byte writes (any method): values corrupted
  - Manual `dd` via `/dev/mem`: values correct AND hardware processes them

- **Hardware Quirk**: The SST DSP SRAM requires byte-level bus transactions
  - `/dev/mem` path generates proper byte lane strobes
  - Kernel driver writes (any method) don't trigger the state machine
  - This is a fundamental limitation of how x86 kernel MMIO works

- **Solution**: Pre-activate SRAM before loading driver
  ```bash
  # Before kldload acpi_intel_sst:
  printf '\x00\x04\x80\x84' | dd of=/dev/mem bs=1 seek=$((0xdf8fb000)) conv=notrunc 2>/dev/null
  sleep 0.1
  printf '\x1f\x04\x80\x84' | dd of=/dev/mem bs=1 seek=$((0xdf8fb000)) conv=notrunc 2>/dev/null
  sleep 0.1
  kldload acpi_intel_sst
  ```

- **Driver Behavior**:
  - Checks if SRAM was pre-activated
  - If active, proceeds with DSP initialization
  - If not, prints clear instructions for manual activation

---

## [0.18.0] - 2026-02-12

### Changed: Atomic 32-bit Writes with mfence

- **v0.17.0 bus_space FAILED** - Still getting corrupted values (`0x1c200000` instead of `0x84800400`)
  - Even with `bus_space_barrier()` after each byte, writes are corrupted
  - The byte-by-byte approach fundamentally doesn't work from kernel

- **New Approach**: Single atomic 32-bit writes
  - Use `pmap_mapdev_attr()` with `VM_MEMATTR_UNCACHEABLE`
  - Write entire 32-bit value in one operation via volatile pointer
  - Use `__asm __volatile("mfence")` for full memory barriers
  - Theory: Hardware might require atomic 32-bit writes, not byte writes

- **Key Observation**: Manual `dd` byte writes work, driver byte writes don't
  - `/dev/mem` path has some special property we can't replicate
  - dd writes go through userspace → kernel → uiomove → hardware
  - Driver writes go directly to hardware but get corrupted

---

## [0.17.0] - 2026-02-12

### Changed: bus_space API with Explicit Barriers

- **Problem Identified**: Driver volatile pointer byte writes produce `0x1c20001f` instead of `0x8480041f`
  - Upper bytes are being corrupted: `0x1c2000xx` instead of `0x848004xx`
  - This happens even with `VM_MEMATTR_UNCACHEABLE` mapping
  - Manual `dd` byte writes work, but driver writes don't

- **New Approach**: Use FreeBSD `bus_space` API for proper MMIO serialization
  - `bus_space_map()` instead of `pmap_mapdev_attr()`
  - `bus_space_write_1()` for byte-level writes
  - `bus_space_barrier()` after EACH byte to force completion
  - Read-back after each write to ensure hardware sees it
  - 1ms DELAY between bytes for timing

- **Key Insight**: The issue might be write combining or CPU reordering
  - `bus_space_barrier(BUS_SPACE_BARRIER_WRITE)` should prevent this
  - Read-back forces the write to complete before continuing

---

## [0.16.0] - 2026-02-12

### Attempt: Byte-by-Byte Writes with pmap_mapdev_attr

- Tried writing SRAM control register byte-by-byte using volatile pointers
- Used `pmap_mapdev_attr()` with `VM_MEMATTR_UNCACHEABLE`
- **Failed**: Writes produce corrupted values (`0x1c20001f` instead of `0x8480041f`)
- Upper bytes are being reordered or combined incorrectly
- Manual `dd if=/dev/mem` byte writes work, but driver writes don't

---

## [0.15.0] - 2026-02-12

### BREAKTHROUGH: Rising Edge Trigger for SRAM Enable

- **ROOT CAUSE FOUND** - Hardware requires a **rising edge (0→1 transition)** on enable bits
  - Simply writing `0x8480041f` (with bits already set) does NOT trigger SRAM power-up
  - Must first CLEAR bits 0-4, then SET them to create the rising edge
  - This is why manual dd worked but driver writes didn't!

- **Working Sequence**:
  1. Read current CTRL value (0x8480041f)
  2. Clear bits 0-4: write `0x84800400`
  3. Wait 10ms
  4. Set bits 0-4: write `0x8480041f`
  5. Wait 100ms
  6. SRAM becomes accessible! CTRL changes to 0x78663178

- **Result**: SRAM[0] reads 0xc31883d6 instead of 0xFFFFFFFF - **DSP memory is ALIVE!**

### Changed

- `sst_enable_sram_direct()` now implements rising edge trigger sequence
- Driver version bumped to 0.15.0-RisingEdge

---

## [0.14.0] - 2026-02-12

### Critical Discovery: SRAM Enable via BAR0+0xFB000

- **PCI SST Device Found** - SST is visible at PCI 00:13.0 (8086:9CB6) with BAR0 at 0xDF800000
  - This is the REAL BAR0, not the ACPI-declared 0xFE000000 which is dead
  - BAR1 at 0xDF900000 contains PCI config mirror / LPSS private registers

- **SRAM Control Register Discovered** - BAR0+0xFB000 controls SRAM power
  - Default value: 0x8480041f (SRAM disabled, returns 0xFFFFFFFF)
  - After enable: 0x78663178 (hardware processed, SRAM accessible)
  - Setting bits 0-4 (0x1F) triggers SRAM power-up sequence

- **Uncached Memory Mapping** - switch from `bus_space_map` to `pmap_mapdev_attr`
  - Uses `VM_MEMATTR_UNCACHEABLE` to ensure writes go directly to hardware
  - Added `#include <vm/vm.h>` and `#include <vm/pmap.h>` for pmap functions

---

## [0.8.0] - 2026-02-12

### Critical Bug Fixes

- **SRAM Power Gate Inversion** (root cause of BAR0 returning 0xFFFFFFFF)
  - `sst_wpt_power_up()`: changed `|=` to `&= ~` for ISRAMPGE/DSRAMPGE masks
  - Same fix applied in `sst_iobp_probe()`
  - Old code **enabled** power gating (= turned SRAM OFF) instead of disabling it
  - After fix: VDRTCTL0 = 0x00000003 (D3PGD + D3SRAMPGD set, PGE bits clear = SRAM ON)
  - Evidence: VDRTCTL0 went from 0x00000001 (SRAM on) to 0x000FFFFF (SRAM off!) with old code

- **I2C Codec on Wrong Bus** - driver was probing I2C1/0x2C (touchpad) instead of I2C0/0x1C (RT286)
  - RT286/ALC3263 audio codec is at **I2C0 (iicbus6), address 0x1C** (INT343A)
  - 0x2C on I2C1 is the Dell touchpad (DLL0665), NOT the audio codec
  - Updated all I2C probe code to target I2C0 at 0xFE103000

- **OSYS Too Low for LPSS Fabric** - DSDT requires OSYS >= 0x07DC (Windows 2012)
  - FreeBSD reported OSYS = 0x07D9 (Windows 7), so DSDT never initialized LPSS fabric
  - Fix: added `hw.acpi.install_interface="Windows 2012"` to /boot/loader.conf
  - This makes DSDT set OSYS = 0x07DD, enabling LPSS memory routing to BAR0

### Added

- **Phase 0: PCI BAR Fixup** in attach - detects if PCI rescan relocated ADSP BARs
  away from ACPI-declared addresses (0xFE000000/0xFE100000) and restores them
- **HDA Manual Reset Sequence** - pre-initializes HDA controller before hdac(4) attaches
  to prevent "stuck in reset" error when controller was just dynamically enabled
  - Performs full HDA enter-reset / exit-reset / codec discovery sequence
  - Reports STATESTS (which codec SDI lines responded)
- **Phase 7: HDA Enable + BAR0 Re-test** - tries HDA enable via RCBA FD register
  then re-tests BAR0 (HDA enable may activate address decoder as side effect)
- **Phase 8: I2C0 Codec Probe** - probes RT286 codec on I2C0 after D0 transition
- **Phase 9: Summary** - consolidated status report (BAR0, HDA, next steps)
- **SHIM catpt Configuration** in dsp_init path:
  - 24MHz SRAM bank clock via BAR1 CS1 register
  - CLKCTL SMOS=2 (38.4kHz reference)
  - Low Trunk Clock (LTRC) value
  - Host DMA access enabled for all channels (HMDC)
- **I2C0 register definitions** in sst_regs.h (SST_I2C0_BASE, SST_I2C0_CODEC_ADDR,
  SST_I2C0_PRIV_BASE)

### Key Discovery: HDA is a Dead End

- hdac1 attaches at PCI 0:1B.0, controller works (CORB/RIRB OK)
- **Zero codecs found** on HDA link - STATESTS = 0x0000
- RT286/ALC3263 is on **I2S bus** (through SST DSP), NOT on HDA link
- BIOS configured hardware for I2S mode exclusively
- **Only path to audio = SST DSP** (BAR0 + firmware + I2S + I2C codec control)

### Changed

- Driver version bumped to 0.8.0
- I2C probe targets I2C0/0x1C instead of I2C1/0x2C
- Phase numbering updated (HDA enable moved to Phase 7, codec probe to Phase 8)

### Technical Note: First Complete Boot Expected

After reboot with all three fixes applied simultaneously:
1. OSYS >= 0x07DC (LPSS fabric initialized by DSDT)
2. SRAM powered ON (PGE bits cleared correctly)
3. BAR addresses correct (Phase 0 fixup restores if relocated)

If BAR0 becomes accessible, driver enters dsp_init path:
IRQ allocation, IPC init, firmware load (IntcSST2.bin), DMA/SSP/PCM/topology init.

---

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
