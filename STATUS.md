# Intel SST Audio Driver - Session Status
## Date: 2026-02-13
## Version: v0.44.0 (pending commit)

---

## LAST SESSION SUMMARY

### Kernel Panic Root Cause (FIXED)
**IRQ handler registered before IPC mutex initialized.**

In `acpi_intel_sst.c` ACPI attach path:
- `bus_setup_intr()` was at line 2904 (ISR active immediately)
- `sst_ipc_init()` was at line 2914 (mutex initialized here)
- With shared IRQ (RF_SHAREABLE), another device could trigger ISR instantly
- ISR calls `sst_ipc_intr()` → `mtx_lock(&sc->ipc.lock)` on uninitialized mutex → PANIC

**Fix:** Moved `sst_ipc_init()` before `bus_setup_intr()`.

### All Fixes in v0.44.0
1. **IRQ/mutex ordering** - sst_ipc_init() now runs BEFORE bus_setup_intr()
2. **ISR spurious interrupt guard** - Reject 0xFFFFFFFF register reads in sst_ipc_intr()
3. **FW_READY mbox bounds** - Check mbox_offset + sizeof(fw_ready) fits in DRAM
4. **pcm_unregister guard** - Only call pcm_unregister if registration succeeded

### Changes in v0.43.0 (committed this session)
- **Child PCM device** for sound(4) - device_add_child("pcm") with PCM_SOFTC_SIZE
- **catpt IPC protocol** - IPCX DONE=reply, IPCD BUSY=notification
- **IPC message types** renumbered to match Linux catpt enum values
- **FW_READY mailbox** extracted from IPCD bits[28:0] << 3
- **IPC header macros** rewritten for catpt format

---

## CURRENT STATE OF DRIVER

### What Works
- ACPI probe/attach (INT3438)
- PCI fallback attach (0x9CB6)
- Power management (catpt WPT sequence, VDRTCTL0/2)
- SRAM enable with dummy reads (v0.42.0)
- Firmware parsing ($SST format, MMIO 32-bit writes)
- IPC framework (send/recv/timeout/polling)
- DMA controller init (8 channels, DesignWare)
- SSP/I2S port configuration
- PCM sound(4) registration (child device pattern)
- Jack detection (GPIO polling)
- Topology framework (default pipeline)

### What's NOT Working Yet (path to MP3 playback)
1. **Firmware boot** - Not verified end-to-end, FW_READY may not fire
2. **IPC stream allocation** - Stub only, no actual DSP stream created
3. **PCM trigger** - Doesn't start DMA/SSP for real audio I/O
4. **DMA circular buffer** - Only single-block, needs linked list for audio
5. **Codec (Realtek ALC3263)** - No driver, unknown if DSP or host controls it

### Architecture
```
sound(4)  ←→  pcm child device (PCM_SOFTC_SIZE softc)
                   ↕ ivars
              acpi_intel_sst (sst_softc)
                   ↕
              DSP Hardware (BAR0: DRAM/IRAM/SHIM/SSP/DMA)
                   ↕
              IntcSST2.bin firmware
                   ↕
              Realtek ALC3263 codec (I2S via SSP0/SSP1)
```

### Key File Map
| File | Purpose | Status |
|------|---------|--------|
| acpi_intel_sst.c | Main driver, power, ISR | Working |
| sst_firmware.c | FW load/parse/boot | 70% |
| sst_ipc.c | Host↔DSP messaging | 40% |
| sst_ipc.h | IPC protocol defs (catpt) | Done |
| sst_dma.c | DMA controller | 50% |
| sst_pcm.c | sound(4) integration | 80% |
| sst_ssp.c | I2S codec interface | 50% |
| sst_topology.c | Audio pipeline | Stub |
| sst_jack.c | Headphone detect | 60% |
| sst_regs.h | Register definitions | Done |

---

## NEXT STEPS (priority order)

### Phase 1: Verify DSP Boot
- [ ] Load module, check dmesg for FW_READY
- [ ] If FW_READY fires: verify mailbox address, check fw_ready struct
- [ ] GET_FW_VERSION as first IPC sanity test
- [ ] If no FW_READY: check CLKCTL SMOS, add more debug to boot sequence

### Phase 2: Stream Allocation (IPC)
- [ ] Implement real catpt ALLOCATE_STREAM IPC command
- [ ] Handle response with stream_hw_id
- [ ] Test with simple stream alloc/free cycle

### Phase 3: Audio Path
- [ ] Connect PCM trigger → DMA start + SSP enable
- [ ] Implement circular DMA (linked list)
- [ ] Position tracking from DMA controller
- [ ] Test with `cat /dev/urandom > /dev/dsp`

### Phase 4: Codec
- [ ] Research Linux catpt codec integration
- [ ] Determine if DSP FW handles codec or if we need I2C init
- [ ] Test I2S signal with oscilloscope/logic analyzer if available

---

## COMMIT HISTORY
```
v0.44.0 - Fix kernel panic: IRQ/mutex init ordering + safety guards
v0.43.0 - catpt IPC protocol, child PCM device, FW_READY mailbox
v0.42.0 - Fix SRAM sanitize - add dummy reads after power gating
v0.41.0 - Fix stale IPCD false FW_READY, add catpt IPC format
v0.37.0 - Fix VDRTCTL0 bit positions to match Linux catpt
v0.35.0 - Exact catpt power sequence with corrected register layout
```
