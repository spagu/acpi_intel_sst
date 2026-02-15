# Intel SST Audio Driver - Session Status
## Date: 2026-02-15
## Version: v0.58.0

---

## LAST SESSION SUMMARY

### Parametric EQ Presets (Issue #4)

Replaced the frequency-based HPF cutoff selection with named EQ presets. The
catpt DSP supports only one 2nd-order biquad stage per stream (SET_BIQUAD IPC
has no stage index), so each preset defines a complete filter profile.

### Changes in v0.58.0

1. **EQ preset enum** - `enum sst_eq_preset_id` with three presets: FLAT
   (bypass), STOCK_SPEAKER (HPF 150Hz), MOD_SPEAKER (HPF 100Hz, warmer).

2. **EQ preset table** - `sst_eq_presets[]` replaces `sst_hpf_biquad[]` with
   named entries containing pre-computed Q2.30 biquad coefficients.

3. **Preset API** - `sst_topology_set_widget_eq_preset()` replaces
   `sst_topology_set_widget_hpf()`.

4. **BASS mixer** - Now maps 0=flat, 1-50=stock_speaker, 51-100=mod_speaker
   instead of frequency-indexed cutoffs.

5. **Stall recovery** - EQ preset now restored alongside volume after DSP
   stream re-allocation.

6. **Suspend/resume** - Restores `eq_preset` instead of `hpf_cutoff`.

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
- EQ presets (speaker protection, BASS mixer: flat/stock/mod)
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
| sst_topology.c | Audio pipeline + EQ presets + limiter | Working |
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
v0.58.0 - EQ presets: named biquad presets replace HPF cutoff selection (issue #4)
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
