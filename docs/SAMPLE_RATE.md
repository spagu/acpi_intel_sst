
# Sample Rate Behavior

## Overview

The `acpi_intel_sst` driver operates at a **fixed 48 kHz sample rate**. The entire audio pipeline — from DSP firmware topology to the I2S link between the Intel SST DSP and the Realtek RT286 codec — runs exclusively at 48,000 Hz. This is not a limitation of the driver software; it is a hardware and firmware constraint inherited from Intel's SST platform design.

All audio reaching the driver is expected to be 48 kHz stereo. Content at other sample rates (e.g., 44.1 kHz CD audio) is resampled by FreeBSD's sound(4) framework before it ever reaches the driver.

---

## Why 48 kHz

The 48 kHz rate is hardcoded at three independent levels in the audio pipeline. Changing the sample rate would require coordinated modifications across all three.

### 1. SSP Clock Divider

The I2S serial port (SSP0) clock is derived from a 24 MHz master clock with a fixed divider:

```
MCLK = 24 MHz
clock_divider = 9
BCLK = 24,000,000 / (9 + 1) = 2,400,000 Hz
Sample rate = BCLK / (2 channels x 25 bits) = 48,000 Hz
```

From `src/acpi_intel_sst.c:1087-1100`:
```c
devfmt.mclk = SST_MCLK_FREQ_24_MHZ;
devfmt.clock_divider = 9;
devfmt.channels = 2;
```

The divider value `9` is sent to the DSP firmware via the `SET_DEVICE_FORMATS` IPC command at driver attach time. This configures the SSP hardware before any audio stream is created.

### 2. DSP Topology Widgets

Every widget (processing module) in both playback and capture pipelines is initialized with `sample_rate = 48000`:

From `src/sst_topology.c:571-761`:
```c
/* Playback pipeline widgets */
w->sample_rate = 48000;  /* pcm0p — PCM input */
w->sample_rate = 48000;  /* HPF1.0 — high-pass filter */
w->sample_rate = 48000;  /* PGA1.0 — gain/volume */
w->sample_rate = 48000;  /* LIMITER1.0 — peak limiter */
w->sample_rate = 48000;  /* ssp0-out — DAI output */

/* Capture pipeline widgets */
w->sample_rate = 48000;  /* ssp0-in — DAI input */
w->sample_rate = 48000;  /* PGA2.0 — capture gain */
w->sample_rate = 48000;  /* pcm0c — PCM output */
```

These values are sent to the DSP firmware when allocating stream resources. The firmware uses them to configure internal buffer sizes, DMA transfer rates, and processing block lengths.

### 3. Biquad Filter Coefficients

The driver computes biquad filter coefficients (for the high-pass filter and parametric EQ) using 48 kHz as the design sample rate:

From `src/sst_topology.c:276`:
```c
/* For DSP biquad design at 48kHz, omega = 2*pi*fc/48000 ... */
```

The integer sine/cosine routines and coefficient calculations are all based on `omega = 2 * pi * fc / 48000`. Using these coefficients at a different sample rate would shift filter frequencies and alter the response shape.

---

## Where Resampling Happens

Resampling is handled entirely by **FreeBSD's sound(4) subsystem**, specifically the `vchan` (virtual channel) layer. The driver itself performs no sample rate conversion.

```
Application (44.1 kHz)
    |
    v
sound(4) / vchan  <-- resampling happens here
    |
    v
acpi_intel_sst (48 kHz fixed)
    |
    v
DSP firmware (48 kHz pipeline)
    |
    v
SSP0 / I2S (48 kHz bit clock)
    |
    v
RT286 codec / speakers
```

While the PCM channel capabilities advertise a range of 8,000–192,000 Hz to the sound(4) framework (`src/sst_pcm.c:49-54`), this is intentional — it allows `vchan` to accept audio at any standard rate from applications and resample it to the hardware rate.

The default channel speed is set to 48,000 Hz (`src/sst_pcm.c:513`):
```c
ch->speed = 48000;
```

FreeBSD's `vchan` resampler uses linear interpolation by default. For higher quality resampling, users can adjust the `vchan` algorithm via sysctl.

---

## Bit-Perfect 48 kHz

Native 48 kHz content passes through the pipeline without any sample rate conversion. When combined with DSP bypass mode (flat EQ, no limiting), 48 kHz audio reaches the DAC with minimal processing:

1. **Source**: 48 kHz PCM from application
2. **vchan**: No resampling needed (rate matches hardware)
3. **Driver**: Passes samples directly to DSP via DMA
4. **DSP pipeline**: Gain stage at 0 dB, biquad bypassed, limiter bypassed
5. **I2S/SSP0**: 48 kHz bit clock delivers samples to codec

To configure bypass mode:
```sh
# Flat EQ (bypass biquad filter)
sysctl dev.acpi_intel_sst.0.hpf_cutoff=0

# Bypass limiter
sysctl dev.acpi_intel_sst.0.limiter_threshold=0

# EQ preset flat
sysctl dev.acpi_intel_sst.0.eq_preset=0
```

> **Note:** The -3 dB headroom policy (`SST_HEADROOM_DB`) is still applied by default in the gain stage. This is by design to prevent inter-sample peak clipping at the DAC. See [GAIN_STAGING.md](GAIN_STAGING.md) for details.

---

## 44.1 kHz Content

CD-quality audio (44,100 Hz) and related rates (22,050 Hz, 88,200 Hz) are resampled to 48,000 Hz by `vchan` before reaching the driver. This is the most common real-world scenario, since much music is distributed at 44.1 kHz.

### Quality Implications

- **44,100 to 48,000 Hz** is not an integer ratio (147:160), so the resampler must interpolate. This introduces a small amount of quantization noise, though it is typically below audible thresholds.
- FreeBSD's default `vchan` resampler is adequate for casual listening. For critical listening, the resampling quality can be tuned:

```sh
# Check current resampling quality
sysctl dev.pcm.0.play.vchanformat
sysctl dev.pcm.0.play.vchanrate

# Force hardware rate (should already be 48000)
sysctl dev.pcm.0.play.vchanrate=48000
```

### Practical Advice

For the best results with 44.1 kHz content:

1. **Accept the resample** — the quality loss is negligible for most use cases.
2. **Use a player with built-in resampling** — applications like `mpv` or `ffplay` can use high-quality internal resamplers (SoX, libsamplerate) before sending audio to the OS.
3. **Keep DSP effects minimal** — since the signal has already been resampled, additional processing (EQ, limiting) operates on the resampled version, which is fine.

---

## Future Considerations

Supporting native 44.1 kHz playback would require changes at all three levels of the pipeline:

### 1. SSP Clock Divider

A different `clock_divider` value would be needed. However, 24 MHz does not divide evenly to produce a 44,100 Hz bit clock:

```
Required BCLK = 44100 x 2 x 25 = 2,205,000 Hz
Divider = (24,000,000 / 2,205,000) - 1 = 9.884...  (not integer)
```

This means 44.1 kHz would require either a different master clock frequency (e.g., 22.5792 MHz from a separate oscillator) or a fractional clock divider, neither of which is available on the Broadwell-U SST hardware.

### 2. DSP Topology

All pipeline widget `sample_rate` fields would need to match the new rate. The DSP firmware would need to support dynamic rate configuration or multiple pipeline configurations.

### 3. Biquad Coefficients

Filter coefficients would need to be recomputed for the new sample rate. The driver's integer trig functions (`src/sst_topology.c:276`) would need to use `omega = 2*pi*fc/44100` instead of `48000`.

### Bottom Line

Native 44.1 kHz support is **not feasible** on this hardware without a different clock source. The 48 kHz fixed rate with `vchan` resampling is the correct and expected architecture for this platform — it matches what Intel's own Linux `catpt` driver does.
