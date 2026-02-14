# Intel SST Audio Driver - Session Status
## Date: 2026-02-14
## Version: v0.55.0

---

## LAST SESSION SUMMARY

### Audio Output Achieved (with distortion)

After fixing the NID shift bug from v0.51.0, the codec now communicates correctly
over I2C and the DSP streams audio data through SSP0 to the RT286 codec. Sound is
audible for the first time, though significant distortion ("machine gun" / stuttering)
remains.

### All Fixes in v0.52.0

1. **NID shift <<24 → <<20 (CRITICAL)** - v0.51.0 had introduced wrong NID shift
   in sst_codec.h. HDA verb encoding uses NID in bits 27:20 (shift by 20), not
   bits 31:24 (shift by 24). Verified against Linux rl6347a I2C source. Fixed in
   all 18 macros. Codec register readbacks now return correct values.

2. **ISR debug printf removal** - Removed `device_printf` from IPC notification
   handler in interrupt path. Was firing hundreds of times per second during
   playback, causing audio jitter.

3. **Channel map fix** - Changed stereo channel_map from `0xFFFFFF20` to
   `0xFFFFFF10`. Linux catpt uses sequential indices (0,1,2...) for channel
   mapping, not the catpt_channel_index enum values (LEFT=0, RIGHT=2).

4. **I2C separate read transactions** - Implemented `sst_i2c_recv()` for
   separate write+read I2C transactions (matching Linux rl6347a protocol).

5. **RX FIFO drain** - Added FIFO drain before I2C write to clear stale data.

6. **SET_DEVICE_FORMATS at init** - Send SSP device format IPC once at attach
   (like Linux catpt_dai_pcm_new), before any stream allocation.

7. **Both speaker + headphone enabled** - Enable both output paths at init.

8. **Codec PLL rearm** - Re-enable codec PLL after SSP starts clocking I2S,
   so codec locks to active BCLK.

9. **Poll debug reduction** - Reduced position poll debug from 20 to 3 prints.

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
- RT286 codec I2C communication (NID shift FIXED)
- Codec initialization (PLL, DAC power, pin control, I2S mode)
- DSP stream allocation (SYSTEM type, page table, module entry)
- DSP position polling (read_pos_regaddr from DRAM)
- **Audio output - sound is audible!**

### Known Issues
1. **Audio distortion** - "machine gun" / stuttering effect during playback.
   Sound is recognizable but heavily distorted. Possible causes:
   - SSP SSCR0 DSS=7 may indicate 8-bit data width (firmware config)
   - SSTSA=0x103 shows 3 active time slots for 2ch audio (slot 8 suspicious)
   - DMA page table PFN format / buffer sync may need investigation
   - Possible interleaving mismatch (PER_CHANNEL vs PER_SAMPLE)

### Architecture
```
sound(4)  ←→  pcm child device (PCM_SOFTC_SIZE softc)
                   ↕ ivars
              acpi_intel_sst (sst_softc)
                   ↕ IPC (catpt)
              DSP Firmware (IntcSST2.bin)
                   ↕ SSP0 (I2S)
              RT286/ALC3263 codec (I2C0 control)
                   ↕
              Speaker / Headphone
```

### Key File Map
| File | Purpose | Status |
|------|---------|--------|
| acpi_intel_sst.c | Main driver, power, ISR | Working |
| sst_firmware.c | FW load/parse/boot | Working |
| sst_ipc.c | Host↔DSP messaging (catpt) | Working |
| sst_ipc.h | IPC protocol defs | Done |
| sst_codec.c | RT286 codec over I2C | Working |
| sst_codec.h | Codec register definitions | Done |
| sst_dma.c | DMA controller | Working |
| sst_pcm.c | sound(4) integration | 90% |
| sst_ssp.c | I2S codec interface | Working |
| sst_topology.c | Audio pipeline + HPF biquad + limiter | Working |
| sst_jack.c | Headphone detect | Working |
| sst_regs.h | Register definitions | Done |

---

## NEXT STEPS (priority order)

### Phase 1: Fix Audio Distortion
- [ ] Investigate SSP data width (SSCR0 DSS field) - 8-bit vs 16-bit
- [ ] Check SSTSA time slot configuration - 3 slots for 2ch audio
- [ ] Verify page table PFN format matches catpt expectations
- [ ] Try PER_SAMPLE interleaving instead of PER_CHANNEL
- [ ] Add bus_dmamap_sync to audio buffer before DSP reads
- [ ] Compare full IPC payload byte-for-byte with Linux catpt

### Phase 2: Audio Quality Polish
- [ ] Investigate write position updates for SYSTEM streams
- [ ] Volume control via IPC (SET_VOLUME)
- [ ] Proper capture stream support
- [ ] Jack-based output switching (speaker vs headphone)

### Phase 3: Production Readiness
- [ ] Remove remaining debug printf from attach path
- [ ] Error recovery (stream restart on underrun)
- [ ] Power management (D3 suspend/resume)
- [ ] Module unload/reload stability

---

## COMMIT HISTORY
```
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
