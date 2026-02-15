# Troubleshooting

## DSP Stream Stall During Volume Adjustment

### Symptom

Audio stops completely when adjusting the mixer volume (e.g. dragging a
slider in a browser or desktop mixer). The stream goes permanently silent
and does not recover even when volume is raised back up. Reloading the
module (`kldunload` + `kldload`) is the only way to restore audio.

### Root Cause

The catpt DSP firmware stalls its DMA engine after processing many
`SET_VOLUME` IPC messages in rapid succession. Each mixer volume change
sends 2 IPC commands (left + right channel via `STAGE_MESSAGE /
SET_VOLUME`). When a user drags a slider, dozens of updates arrive per
second.

The DSP core stays alive (IPC replies still return `status=0`), but the
stream's DMA position register stops advancing permanently. Neither
`RESUME` nor `RESET + PAUSE + RESUME` recovers the stream; only a full
`FREE + ALLOC` cycle restarts it.

**Diagnostic evidence (poll timer log):**

```
poll[3000]: pos=6208  stall=0    intr=2812   <- normal
poll[3472]: pos=4416  stall=200  intr=3067   <- STALLED
poll[7000]: pos=4416  stall=3728 intr=3067   <- permanently dead
```

The stall threshold appears to be cumulative: roughly 350-450
`SET_VOLUME` IPC messages total (175-225 volume changes) before the
stream dies, regardless of spacing.

### Current Mitigations (v0.55.0+)

Two defenses in `sst_pcm.c`:

1. **Rate limiting** (`sst_mixer_set`, `hz / 2` = 500 ms)
   - At most one `SET_VOLUME` IPC per 500 ms.
   - Intermediate values are stored and flushed by the poll timer
     callout once the cooldown expires (`vol_pending` / `vol_ticks`).

2. **Stall recovery** (`sst_pcm_poll`, `stall_count == 200`)
   - If the DSP position register is unchanged for 200 consecutive
     polls (~1 second), the driver automatically:
     1. Frees the dead stream (`RESET + FREE`)
     2. Allocates a new stream (same ring buffer / page table)
     3. Prepares it (`RESET + PAUSE`)
     4. Resumes (`RESUME`, re-enable HMDC, SSP start, codec PLL rearm)
     5. Restores the current volume level
   - Recovery causes a brief audible glitch (~1 s gap).

### Potential Future Improvements

If the issue reoccurs or the recovery glitch is unacceptable:

- **Codec-side volume** -- Control volume through the RT286 codec's
  analog DAC gain register (NID 0x02, verb `SET_AMP_GAIN_MUTE`) via
  I2C instead of DSP IPC.  This bypasses the DSP entirely and has no
  risk of stalling the stream. Requires mapping the mixer 0-100 range
  to the codec's -65.25 dB .. 0 dB gain steps.

- **Longer rate limit** -- Increasing the cooldown beyond 500 ms
  further reduces IPC traffic but makes the volume slider feel
  sluggish.

- **IPC-free volume via DRAM** -- The DSP exposes per-stream volume
  registers in DRAM (`volume_regaddr` from `ALLOC_STREAM` response).
  Writing directly to these registers (if the firmware honours them)
  would avoid IPC entirely.

### Diagnostic Checklist

```sh
# 1. Check if the stream stalled
dmesg | grep "stalled"

# 2. Check if recovery fired
dmesg | grep "recovered"

# 3. Count SET_VOLUME IPC messages sent
dmesg | grep "IPCX=0x46301" | wc -l

# 4. Check poll timer health (should show stall=0 when healthy)
dmesg | grep "poll\[" | tail -10

# 5. Full reload if all else fails
kldunload acpi_intel_sst && kldload /path/to/acpi_intel_sst.ko
```

### Related Commits

- `924393d` -- rate-limit SET_VOLUME IPC and recover stalled DSP streams
- `3553990` -- remove spurious SET_MIXER_PARAMS IPC from volume path
