# Intel SST Audio Driver - Session Status
## Date: 2026-02-15
## Version: v0.57.0

---

## LAST SESSION SUMMARY

### Suspend/Resume Stability (Issue #10)

The driver's suspend/resume was previously minimal — suspend only reset the DSP
and powered to D3, resume attempted a bare DSP boot with empty SRAM. Result:
broken audio after any suspend/resume cycle. Now fully implemented with ordered
teardown and complete reconstruction.

### Changes in v0.57.0

1. **Firmware SRAM reload** - `sst_fw_reload()` re-writes cached firmware from
   host memory to SRAM after D3 power loss. The firmware(9) handle and
   `sc->fw.data` persist in host memory across suspend; only SRAM is lost.

2. **Suspend teardown** - `sst_acpi_suspend()` now performs ordered shutdown:
   jack disable → PCM stream teardown → SSP stop → codec shutdown → topology
   clear → DSP reset → D3 power-off.

3. **Resume reconstruction** - `sst_acpi_resume()` fully rebuilds the audio
   pipeline: D0 → SHIM init → firmware reload → DSP boot → FW version query →
   module regions → stage caps → topology rebuild → SSP0 device formats →
   codec re-init → speaker/headphone enable → mixer restore → jack enable →
   ramp arm.

4. **Volume ramp-in** - 5-step linear ramp (~50ms) from silence to target
   volume on first PCMTRIG_START after resume, preventing speaker pop.

5. **Mixer state on stream start** - Volume, HPF cutoff, and limiter threshold
   are now applied via IPC on every PCMTRIG_START, also fixing parameter loss
   after stall recovery.

6. **PCM suspend/resume helpers** - `sst_pcm_suspend()` tears down streams and
   stops timers; `sst_pcm_resume()` restores HPF/limiter widget params from
   saved mixer state.

---

## CURRENT STATE OF DRIVER

### What Works
- ACPI probe/attach (INT3438)
- PCI fallback attach (0x9CB6)
- Power management (catpt WPT sequence, VDRTCTL0/2)
- SRAM enable with dummy reads
- Firmware parsing ($SST format, MMIO 32-bit writes)
- DSP firmware boot (FW_READY + GET_FW_VERSION)
- IPC framework (send/recv/ISR, catpt protocol)
- DMA controller init (8 channels, DesignWare)
- SSP/I2S port configuration (firmware-managed)
- PCM sound(4) registration (child device pattern)
- Jack detection (GPIO polling)
- RT286 codec I2C communication
- Codec initialization (PLL, DAC power, pin control, I2S mode)
- DSP stream allocation (SYSTEM type, page table, module entry)
- DSP position polling (read_pos_regaddr from DRAM)
- Volume control (dB→Q1.31, rate-limited IPC)
- HPF biquad (speaker protection, BASS mixer)
- Peak limiter (speaker protection, TREBLE mixer)
- DSP stage capability detection
- Dynamic pipeline topology
- DSP stream stall recovery
- **Audio playback - working!**
- **Suspend/resume (S3) - stable across multiple cycles**
- **Resume volume ramp (anti-pop)**

### Known Issues
1. **Audio capture disabled** - Channels registered but skipped; DSP can't do
   simultaneous play+capture on same SSP.
2. **Single platform tested** - Only Dell XPS 13 9343; other Broadwell-U/Haswell
   untested.

### Architecture
```
sound(4)  ←→  pcm child device (PCM_SOFTC_SIZE softc)
                   ↕ ivars
              acpi_intel_sst (sst_softc)
                   ↕ IPC (catpt)
              DSP Firmware (IntcSST2.bin)
                   ↕ HPF + Gain + Limiter
              DSP Pipeline
                   ↕ SSP0 (I2S)
              RT286/ALC3263 codec (I2C0 control)
                   ↕
              Speaker / Headphone
```

### Key File Map
| File | Purpose | Status |
|------|---------|--------|
| acpi_intel_sst.c | Main driver, power, suspend/resume, ISR | Working |
| sst_firmware.c | FW load/parse/boot/reload | Working |
| sst_ipc.c | Host↔DSP messaging (catpt) | Working |
| sst_ipc.h | IPC protocol defs | Done |
| sst_codec.c | RT286 codec over I2C | Working |
| sst_codec.h | Codec register definitions | Done |
| sst_dma.c | DMA controller | Working |
| sst_pcm.c | sound(4) integration, suspend/resume, ramp | Working |
| sst_ssp.c | I2S codec interface | Working |
| sst_topology.c | Audio pipeline + HPF biquad + limiter | Working |
| sst_jack.c | Headphone detect | Working |
| sst_regs.h | Register definitions | Done |

---

## NEXT STEPS (priority order)

### Phase 1: Audio Capture
- [ ] Enable capture stream allocation on SSP1
- [ ] Test microphone input via ADC path
- [ ] Jack-based input source switching

### Phase 2: Production Readiness
- [ ] Remove remaining debug printf from attach path
- [ ] Module unload/reload stability hardening
- [ ] Test on other Broadwell-U/Haswell platforms

---

## COMMIT HISTORY
```
v0.57.0 - Suspend/resume stability: full teardown/rebuild, FW reload, volume ramp (issue #10)
v0.55.0 - Limiter before SSP0 output, limiter IPC, TREBLE mixer control (issue #3)
v0.54.0 - HPF in playback pipeline, biquad IPC, BASS mixer control (issue #2)
v0.52.0 - First audio output! Fix NID shift, channel map, I2C, ISR debug
v0.51.0 - fix NID shift in codec verbs (<<20 -> <<24) + disable capture stream
v0.50.0 - fix playback - SYSTEM stream type, catpt IPC sequence, codec I2S mode
v0.49.0 - DSP firmware boots! Fix SRAM PGE polarity + DCLCGE/RST sequence
v0.48.0 - RT286 codec driver, DSP-managed PCM, IPC fixes
v0.47.0 - fix kernel panic on module reload - stale child device with dangling ivars
v0.46.0 - catpt IPC struct rewrite - match Linux catpt protocol exactly
v0.45.0 - catpt IPC interrupt rework + child attach crash guard
v0.44.0 - Fix kernel panic: IRQ/mutex init ordering + safety guards
v0.43.0 - catpt IPC protocol, child PCM device, FW_READY mailbox
```
