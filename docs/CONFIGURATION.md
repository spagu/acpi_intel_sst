# Configuration Reference

Complete reference for all configurable parameters in the Intel SST audio driver for FreeBSD.

For a quick overview, see the [Sysctl Configuration Reference](../README.md#sysctl-configuration-reference) in the main README.

---

## Table of Contents

- [Overview](#overview)
- [Boot-Time Configuration](#boot-time-configuration)
- [Runtime Sysctl Reference](#runtime-sysctl-reference)
  - [Debug](#debug)
  - [DSP Audio Parameters](#dsp-audio-parameters)
  - [Volume Ramp-In](#volume-ramp-in)
  - [Jack Detection](#jack-detection)
  - [DSP Telemetry](#dsp-telemetry)
- [Compile-Time Options](#compile-time-options)
- [Audio Profiles](#audio-profiles)
  - [Transparent / Bypass](#transparent--bypass)
  - [Speaker Protection (Default)](#speaker-protection-default)
  - [Warm Speaker](#warm-speaker)
  - [Headphone](#headphone)
- [Parameter Interactions](#parameter-interactions)
- [Persistence](#persistence)

---

## Overview

The driver exposes three layers of configuration:

| Layer | When | Mechanism | Scope |
|:------|:-----|:----------|:------|
| **Compile-time** | `make` | `#define` constants, Makefile flags | Buffer sizes, stream limits, polling intervals, headroom |
| **Boot-time** | Kernel load | `/boot/loader.conf`, `/boot/device.hints` | Initial debug level, driver enable/disable |
| **Runtime** | Any time | `sysctl dev.acpi_intel_sst.0.*` | DSP processing, volume ramp, debug verbosity, jack detection |

Runtime sysctl changes take effect **immediately** on active streams without pipeline restart and persist across suspend/resume (S3) cycles, but are lost on reboot unless saved to `/etc/sysctl.conf` (see [Persistence](#persistence)).

---

## Boot-Time Configuration

### /boot/loader.conf

These entries are required for the driver to attach on Dell XPS 13 9343 (and similar Broadwell-U platforms):

```bash
# Custom DSDT with ADSP enabled
acpi_dsdt_load="YES"
acpi_dsdt_name="/boot/acpi_dsdt.aml"

# Required: DSDT needs OSYS >= 0x07DC to initialize LPSS fabric
hw.acpi.install_interface="Windows 2012"

# Disable GPU HDMI audio controller (conflicts with SST IRQ)
hint.hdac.0.disabled="1"

# Load SST driver at boot
acpi_intel_sst_load="YES"

# Disable ig4 I2C driver (SST driver accesses I2C0 directly for codec control)
ig4_load="NO"
hint.ig4.0.disabled="1"
hint.ig4.1.disabled="1"
```

| Entry | Purpose |
|:------|:--------|
| `acpi_dsdt_*` | Load custom DSDT that forces the ADSP device enabled |
| `hw.acpi.install_interface` | DSDT requires OSYS >= 0x07DC to route memory to BAR0 |
| `hint.hdac.0.disabled` | Disables Intel GPU audio (hdac0 at pci0:0:3:0) which conflicts with SST IRQ |
| `acpi_intel_sst_load` | Auto-load the SST driver at boot |
| `ig4_load` / `hint.ig4.*` | Prevents ig4 driver from claiming I2C0, which SST uses directly for RT286 codec |

### /boot/device.hints

The driver reads device hints at attach time. Currently supported:

```bash
# Set initial debug verbosity (0-3)
hint.acpi_intel_sst.0.debug="3"
```

This is equivalent to `sysctl dev.acpi_intel_sst.0.debug=3` but takes effect before the first firmware load, which is useful for debugging early attach issues.

---

## Runtime Sysctl Reference

All parameters live under `dev.acpi_intel_sst.0.*`. Values take effect immediately on active streams.

### Debug

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `debug` | RW | int | 0-3 | 1 | Debug verbosity level |

**Debug levels:**

| Level | Name | Output |
|:-----:|:-----|:-------|
| 0 | `SST_DBG_QUIET` | Errors and attach/detach only |
| 1 | `SST_DBG_LIFE` | Lifecycle events: firmware load, stream alloc/free |
| 2 | `SST_DBG_OPS` | Operational: IPC messages, volume changes, EQ changes |
| 3 | `SST_DBG_TRACE` | Trace: poll cycles, register dumps, timing |

**Source:** `sc->debug_level` in `acpi_intel_sst.h`

### DSP Audio Parameters

#### EQ Preset

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `eq_preset` | RW | int | 0-2 | 1 | EQ preset shortcut |

| Value | Name | Effect |
|:-----:|:-----|:-------|
| 0 | flat | Bypass (unity biquad coefficients) |
| 1 | stock_speaker | HPF at 150 Hz (factory speaker protection) |
| 2 | mod_speaker | HPF at 100 Hz (warmer response) |

Writing `eq_preset` sets `hpf_cutoff` to the corresponding frequency and switches to HPF biquad mode. All presets use 2nd-order Butterworth filters with Q2.30 fixed-point coefficients.

**Source:** `sst_eq_preset_sysctl()` in `sst_topology.c`

#### HPF Cutoff

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `hpf_cutoff` | RW | int (Hz) | 0, 50-500 | 150 | High-pass filter cutoff frequency |

Values snap to the nearest entry in the coefficient table:

| Hz | Description |
|:---|:------------|
| 0 | Flat bypass (unity passthrough) |
| 50, 60, 80 | Subsonic rumble filter |
| 100, 120, 150 | Speaker protection range |
| 200, 250, 300, 350, 400, 500 | Aggressive high-pass |

Writing `hpf_cutoff` directly overrides `eq_preset` and forces HPF biquad mode. If PEQ is active (`peq_freq > 0`), HPF is latched but not applied until PEQ is disabled.

**Source:** `sst_hpf_cutoff_sysctl()` in `sst_topology.c`

#### Limiter Threshold

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `limiter_threshold` | RW | int | 0-8 | 5 | Peak limiter threshold preset index |

| Index | Threshold | Attack | Default Release |
|:-----:|:---------:|:------:|:---------------:|
| 0 | bypass | - | - |
| 1 | -24 dBFS | 1 ms | 50 ms |
| 2 | -18 dBFS | 1 ms | 75 ms |
| 3 | -12 dBFS | 1 ms | 100 ms |
| 4 | -9 dBFS | 1 ms | 115 ms |
| 5 | -6 dBFS | 1 ms | 135 ms |
| 6 | -3 dBFS | 1 ms | 160 ms |
| 7 | -1 dBFS | 1 ms | 180 ms |
| 8 | 0 dBFS | 1 ms | 200 ms |

Threshold values are stored as Q2.30 linear amplitude internally. The limiter operates in the DSP pipeline between the gain stage and the SSP output.

**Source:** `sst_limiter_threshold_sysctl()` in `sst_topology.c`

#### Limiter Release Override

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `limiter_release` | RW | int (us) | 0, 10000-500000 | 0 | Limiter release time override |

When set to a non-zero value, this overrides the default release time from the selected `limiter_threshold` preset. Set to 0 to revert to the preset default. Range is 10 ms to 500 ms (specified in microseconds).

**Source:** `sst_limiter_release_sysctl()` in `sst_topology.c`

#### Parametric EQ (PEQ)

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `peq_freq` | RW | int (Hz) | 0, 200-16000 | 0 | PEQ center frequency (0 = off) |
| `peq_gain` | RW | int (dB) | -12 to +12 | 0 | PEQ boost/cut |
| `peq_q` | RW | int (Q x 100) | 30-1000 | 71 | PEQ Q factor (71 = Q 0.71) |

Setting `peq_freq > 0` switches the single biquad stage from HPF to PEQ mode. Setting `peq_freq = 0` reverts to HPF mode using the current `hpf_cutoff` value.

The `peq_q` value is the Q factor multiplied by 100 (e.g., 71 = Q 0.71, 141 = Q 1.41, 1000 = Q 10.0). The default of 71 (Q 0.71) provides a gentle, musical bandwidth.

Positive `peq_gain` values engage the gain budget system, automatically reducing the volume ceiling by the boost amount to prevent clipping (see [Parameter Interactions](#parameter-interactions)).

**Source:** `sst_peq_freq_sysctl()`, `sst_peq_gain_sysctl()`, `sst_peq_q_sysctl()` in `sst_topology.c`

### Volume Ramp-In

| Sysctl | RW | Type | Range | Default | Description |
|:-------|:--:|:-----|:------|:--------|:------------|
| `ramp_ms` | RW | int (ms) | 0-500 | 50 | Volume ramp-in on playback start |
| `resume_ramp_ms` | RW | int (ms) | 1-500 | 50 | Volume ramp-in after S3 resume |
| `ramp_curve` | RW | int | 0-2 | 0 | Ramp curve shape |

**Ramp curves:**

| Value | Name | Behavior |
|:-----:|:-----|:---------|
| 0 | Logarithmic | Perceptual dB-linear, slow start / fast finish (default) |
| 1 | Linear | Constant gain increase per step |
| 2 | S-curve | Sinusoidal ease-in-out, slow start and finish |

The ramp engine runs at 10 ms tick intervals (`SST_RAMP_TICK_MS`). A 50 ms ramp produces 5 steps; the maximum 500 ms ramp produces 50 steps.

- `ramp_ms` applies only on initial playback start (`PCMTRIG_START`). Set to 0 to disable.
- `resume_ramp_ms` applies unconditionally after S3 resume to prevent speaker pops. Minimum is 1 ms.
- `ramp_curve` affects both `ramp_ms` and `resume_ramp_ms`.

**Source:** `sc->pcm.ramp_ms`, `sc->pcm.resume_ramp_ms`, `sc->pcm.ramp_curve` in `sst_pcm.h`

> **Deep Dive:** See [`VOLUME_RAMPING.md`](VOLUME_RAMPING.md) for a detailed explanation and visual comparison of ramp curves.

### Jack Detection

| Sysctl | RW | Type | Description |
|:-------|:--:|:-----|:------------|
| `jack.headphone` | RO | int | Headphone jack state (0 = removed, 1 = inserted) |
| `jack.microphone` | RO | int | Microphone jack state (0 = removed, 1 = inserted) |
| `jack.enabled` | RW | int | Enable/disable jack detection polling (0/1) |
| `jack.hp_insertions` | RO | uint | Headphone insertion count (lifetime) |
| `jack.mic_insertions` | RO | uint | Microphone insertion count (lifetime) |
| `jack.poll_count` | RO | uint | Jack polling cycle count |

Jack detection uses GPIO polling with debounce:

| Parameter | Value |
|:----------|:------|
| Poll interval | 250 ms (`SST_JACK_POLL_INTERVAL_MS`) |
| Debounce time | 100 ms (`SST_JACK_DEBOUNCE_MS`) |
| Debounce count | 3 stable readings (`SST_JACK_DEBOUNCE_COUNT`) |

GPIO pin mapping:

| GPIO | Function |
|:----:|:---------|
| 0 | Headphone detect |
| 1 | Microphone detect |
| 2 | Headphone mute |
| 3 | Speaker mute |

**Source:** `sst_jack.c`, `sst_jack.h`

### DSP Telemetry

| Sysctl | RW | Type | Description |
|:-------|:--:|:-----|:------------|
| `telemetry.peak_left` | RO | uint | Left channel peak level (Q1.31 raw) |
| `telemetry.peak_right` | RO | uint | Right channel peak level (Q1.31 raw) |
| `telemetry.peak_db_left` | RO | string | Left channel peak in dB (e.g. "-3.2 dB") |
| `telemetry.peak_db_right` | RO | string | Right channel peak in dB (e.g. "-3.2 dB") |
| `telemetry.clip_count` | RO | uint | Cumulative clipping events |
| `telemetry.clip_reset` | WO | int | Write 1 to reset clip counter |
| `telemetry.limiter_active` | RO | int | Limiter currently engaging (0/1) |

**Peak levels** are read from DSP memory via MMIO. The Q1.31 format maps `0x7FFFFFFF` = 0 dBFS (full-scale) and `0x00000000` = silence. Readings are rate-limited to a minimum 10 ms interval to prevent sysctl query flooding.

**Clipping detection** triggers when the peak level reaches Q1.31 >= `0x7F000000` (approximately -0.06 dBFS), which is the headroom policy threshold.

**Limiter active** indicates the DSP limiter is currently reducing gain (volume register reads below unity `0x7FFFFFFF` in Q2.30).

**Source:** `sst_peak_sysctl()`, `sst_peak_db_sysctl()`, `sst_clip_reset_sysctl()`, `sst_limiter_active_sysctl()` in `sst_topology.c`

---

## Compile-Time Options

### Makefile Flags

The `src/Makefile` supports the following build options:

```makefile
# Uncomment for verbose kernel debug output and debug symbols
# CFLAGS+= -DDEBUG -g
```

When `-DDEBUG` is defined, the FreeBSD kernel enables additional `device_printf` output at the lowest level. This is separate from the driver's own `debug` sysctl (which controls SST-specific verbosity).

### Key #define Constants

These constants in the source headers control fixed driver behavior. Changing them requires recompilation.

#### PCM Buffer Configuration (`sst_pcm.h`)

| Constant | Value | Description |
|:---------|:------|:------------|
| `SST_PCM_MAX_PLAY` | 4 | Maximum simultaneous playback streams |
| `SST_PCM_MAX_REC` | 2 | Maximum simultaneous capture streams |
| `SST_PCM_MAX_STREAMS` | 6 | Total maximum streams (play + capture) |
| `SST_PCM_BUFFER_SIZE` | 64 KB | DMA buffer size per stream |
| `SST_PCM_BLOCK_SIZE` | 4 KB | DMA block size |
| `SST_PCM_BLOCK_COUNT` | 16 | Number of blocks per buffer |

#### Volume Ramp (`sst_pcm.h`)

| Constant | Value | Description |
|:---------|:------|:------------|
| `SST_RAMP_TICK_MS` | 10 ms | Ramp engine tick interval |
| `SST_RAMP_MS_DEFAULT` | 50 ms | Default ramp duration |
| `SST_RAMP_MS_MAX` | 500 ms | Maximum ramp duration |

#### Topology Limits (`sst_topology.h`)

| Constant | Value | Description |
|:---------|:------|:------------|
| `SST_TPLG_MAX_PIPELINES` | 8 | Maximum DSP pipelines |
| `SST_TPLG_MAX_WIDGETS` | 32 | Maximum topology widgets |
| `SST_TPLG_MAX_ROUTES` | 64 | Maximum audio routes |
| `SST_TPLG_MAX_MODULES` | 16 | Maximum DSP modules |
| `SST_TPLG_NAME_LEN` | 32 | Maximum name length for topology elements |

#### Gain Staging (`acpi_intel_sst.h`, `sst_topology.h`)

| Constant | Value | Description |
|:---------|:------|:------------|
| `SST_HEADROOM_DB` | 3 dB | Headroom below 0 dBFS |
| `SST_HEADROOM_HALF_DB` | 6 | Headroom in 0.5 dB steps (used by mixer) |

The 3 dB headroom policy means the maximum digital volume is -3 dBFS, leaving headroom for inter-sample peaks and DSP processing. See [`GAIN_STAGING.md`](GAIN_STAGING.md) for details.

#### Jack Detection (`sst_jack.h`)

| Constant | Value | Description |
|:---------|:------|:------------|
| `SST_JACK_POLL_INTERVAL_MS` | 250 ms | GPIO polling interval |
| `SST_JACK_DEBOUNCE_MS` | 100 ms | Debounce window |
| `SST_JACK_DEBOUNCE_COUNT` | 3 | Stable readings required before state change |

---

## Audio Profiles

Pre-configured parameter sets for common use cases. Apply with `sysctl` commands or add to `/etc/sysctl.conf` for persistence.

### Transparent / Bypass

Disable all DSP processing for a clean, unaltered signal path. Useful for external DACs, headphone amps, or when the application handles its own DSP.

```sh
sysctl dev.acpi_intel_sst.0.hpf_cutoff=0
sysctl dev.acpi_intel_sst.0.limiter_threshold=0
sysctl dev.acpi_intel_sst.0.peq_freq=0
sysctl dev.acpi_intel_sst.0.ramp_ms=0
```

| Parameter | Value | Effect |
|:----------|:------|:-------|
| `hpf_cutoff` | 0 | HPF bypassed (flat, unity biquad) |
| `limiter_threshold` | 0 | Limiter bypassed (no gain reduction) |
| `peq_freq` | 0 | PEQ disabled (HPF mode, which is also bypassed) |
| `ramp_ms` | 0 | No fade-in on playback start |

> **Note:** `resume_ramp_ms` is intentionally left at its default (50 ms). The resume ramp prevents speaker pops after S3 wake and should generally remain enabled. Set `resume_ramp_ms=1` for a near-instant resume if desired.

> **Warning:** With the limiter bypassed and HPF off, there is no speaker protection. Use caution at high volumes with built-in laptop speakers to avoid damage from DC offset or excessive excursion.

### Speaker Protection (Default)

The factory-default configuration. Protects built-in laptop speakers with a high-pass filter and peak limiter.

```sh
sysctl dev.acpi_intel_sst.0.eq_preset=1
sysctl dev.acpi_intel_sst.0.limiter_threshold=5
sysctl dev.acpi_intel_sst.0.limiter_release=0
sysctl dev.acpi_intel_sst.0.peq_freq=0
sysctl dev.acpi_intel_sst.0.ramp_ms=50
sysctl dev.acpi_intel_sst.0.ramp_curve=0
```

| Parameter | Value | Effect |
|:----------|:------|:-------|
| `eq_preset` | 1 | Stock speaker HPF at 150 Hz |
| `limiter_threshold` | 5 | Peak limiter at -6 dBFS |
| `limiter_release` | 0 | Use preset default (135 ms) |
| `peq_freq` | 0 | PEQ off (HPF mode) |
| `ramp_ms` | 50 | 50 ms logarithmic fade-in |
| `ramp_curve` | 0 | Logarithmic (perceptual) |

### Warm Speaker

A relaxed speaker profile with a lower HPF cutoff for more bass extension and a lower limiter for wider dynamic range. Good for music listening on speakers.

```sh
sysctl dev.acpi_intel_sst.0.hpf_cutoff=100
sysctl dev.acpi_intel_sst.0.limiter_threshold=3
sysctl dev.acpi_intel_sst.0.limiter_release=0
sysctl dev.acpi_intel_sst.0.peq_freq=0
sysctl dev.acpi_intel_sst.0.ramp_ms=50
sysctl dev.acpi_intel_sst.0.ramp_curve=0
```

| Parameter | Value | Effect |
|:----------|:------|:-------|
| `hpf_cutoff` | 100 | HPF at 100 Hz (more bass, less protection) |
| `limiter_threshold` | 3 | Peak limiter at -12 dBFS (wider dynamic range) |
| `limiter_release` | 0 | Use preset default (100 ms) |
| `peq_freq` | 0 | PEQ off (HPF mode) |
| `ramp_ms` | 50 | 50 ms logarithmic fade-in |
| `ramp_curve` | 0 | Logarithmic (perceptual) |

### Headphone

Optimized for headphone listening. No HPF needed (headphones handle full-range), with a safety limiter near 0 dBFS to protect hearing from unexpected volume spikes.

```sh
sysctl dev.acpi_intel_sst.0.hpf_cutoff=0
sysctl dev.acpi_intel_sst.0.limiter_threshold=7
sysctl dev.acpi_intel_sst.0.peq_freq=0
sysctl dev.acpi_intel_sst.0.ramp_ms=50
sysctl dev.acpi_intel_sst.0.ramp_curve=2
```

| Parameter | Value | Effect |
|:----------|:------|:-------|
| `hpf_cutoff` | 0 | HPF bypassed (full-range for headphones) |
| `limiter_threshold` | 7 | Safety limiter at -1 dBFS (hearing protection) |
| `peq_freq` | 0 | PEQ off |
| `ramp_ms` | 50 | 50 ms fade-in |
| `ramp_curve` | 2 | S-curve (smooth ease-in-out, comfortable for headphones) |

---

## Parameter Interactions

### Biquad Mutual Exclusivity

The DSP has a **single biquad filter stage** shared between HPF and PEQ. Only one can be active at a time:

```
peq_freq = 0  -->  HPF mode (uses hpf_cutoff)
peq_freq > 0  -->  PEQ mode (uses peq_freq, peq_gain, peq_q)
```

When switching from PEQ to HPF mode (by setting `peq_freq=0`), the HPF reverts to the current `hpf_cutoff` value. The HPF cutoff is always remembered even while PEQ is active.

The biquad mode is tracked internally as `SST_BIQUAD_MODE_HPF` (0) or `SST_BIQUAD_MODE_PEQ` (1).

### Gain Budget

When PEQ applies positive gain (boost), the driver automatically reduces the maximum volume ceiling by the same amount to prevent clipping:

```
effective_ceiling = max_volume - peq_gain_dB - SST_HEADROOM_DB
```

For example, with `peq_gain=6` and 3 dB headroom, the effective ceiling is 9 dB below 0 dBFS. This is enforced automatically by the mixer — the user does not need to manually reduce volume.

Negative `peq_gain` (cut) does not affect the volume ceiling.

### Headroom Policy

The driver enforces a fixed 3 dB headroom (`SST_HEADROOM_DB`) below 0 dBFS at all times. This accounts for:

- Inter-sample peaks that may exceed 0 dBFS after D/A conversion
- DSP processing artifacts
- Codec analog stage overhead

The headroom is applied in the mixer gain mapping. The 100% mixer position maps to -3 dBFS, not 0 dBFS.

Clipping detection (`telemetry.clip_count`) triggers at Q1.31 >= `0x7F000000` (~-0.06 dBFS), which is well into the headroom zone and indicates the signal is approaching the absolute ceiling despite the headroom policy.

> **Deep Dive:** See [`GAIN_STAGING.md`](GAIN_STAGING.md) for the full gain staging architecture.

### EQ Preset vs. Direct HPF Control

`eq_preset` is a convenience shortcut that sets `hpf_cutoff` to a predefined value. Writing `hpf_cutoff` directly overrides the preset. The two controls are linked:

- `eq_preset=1` sets `hpf_cutoff=150`
- `hpf_cutoff=200` (direct write) overrides the preset
- `eq_preset=0` sets `hpf_cutoff=0` (flat bypass)

### Limiter Release Override

`limiter_release` overrides only the release time from the selected `limiter_threshold` preset. Attack time and threshold dBFS remain as defined by the preset. Set `limiter_release=0` to revert to the preset's default release time.

---

## Persistence

Sysctl values reset to defaults on every boot. To persist custom settings, add them to `/etc/sysctl.conf`:

```bash
# /etc/sysctl.conf - Intel SST audio driver configuration
#
# Transparent mode (no DSP processing)
dev.acpi_intel_sst.0.hpf_cutoff=0
dev.acpi_intel_sst.0.limiter_threshold=0
dev.acpi_intel_sst.0.peq_freq=0
dev.acpi_intel_sst.0.ramp_ms=0
```

These are applied at boot by `sysctl(8)` after the driver loads.

For boot-time settings that must take effect before the driver loads (e.g., debug level for firmware load debugging), use `/boot/device.hints`:

```bash
# /boot/device.hints
hint.acpi_intel_sst.0.debug="3"
```

> **Tip:** To quickly switch between profiles, create shell scripts:
>
> ```bash
> #!/bin/sh
> # transparent.sh - disable all DSP processing
> sysctl dev.acpi_intel_sst.0.hpf_cutoff=0
> sysctl dev.acpi_intel_sst.0.limiter_threshold=0
> sysctl dev.acpi_intel_sst.0.peq_freq=0
> sysctl dev.acpi_intel_sst.0.ramp_ms=0
> echo "Transparent mode enabled"
> ```
