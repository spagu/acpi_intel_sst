
# :control_knobs: Volume Ramping (Soft Start)

<div align="center">
  <img src="transient_response.png" alt="Transient Response Comparison" width="600">
</div>

## What is Volume Ramping?

Volume ramping (or "soft start") is an essential audio pipeline feature designed to enhance the listening experience by **eliminating transient noise** and **protecting hardware**.

When audio playback initializes—or the system wakes from sleep—DACs and amplifiers can produce instant DC offsets or high-frequency transients. Without mitigation, this manifests as:
1.  **"Pops" and "Clicks"**: Audible digital artifacts as the hardware stabilizes.
2.  **Sudden Blasts**: Instantaneous 0-to-100% volume jumps that can startle the listener or damage sensitive tweeters.

**The Solution:** The `acpi_intel_sst` driver implements a hardware-timed envelope generator. Instead of jumping to the target volume, the DSP smoothly interpolates gain from $-\infty$ dB to the target level over a configurable duration (default: 50ms).

---

## :headphones: Audiophile Deep Dive

For the critical listener, volume ramping is more than just "anti-pop"—it's about **signal integrity during state transitions**.

### Why Ramping Curves Matter

The human ear perceives loudness **logarithmically**, not linearly. A linear voltage increase (`0.1V -> 0.2V -> 0.3V`) does *not* sound like a smooth volume swell; it sounds like a sudden jump followed by a slow crawl.

To achieve a **perceptually smooth** fade-in, the gain must increase exponentially (linear in Decibels).

<img src="volume_ramp_curves.png" alt="Volume Ramp Curves" width="600">

### The Curves

1.  **Red Curve (Logarithmic / dB-Linear)** :star: **Audiophile Choice**
    *   **Physics**: Gain increases exponentially ($y = x^2$ or similar).
    *   **Perception**: Sounds perfectly linear to the human ear.
    *   **Benefit**: The "fade-in" feels natural and transparent. It spends less time in the inaudible noise floor and more time in the audible range.

2.  **Blue Curve (Linear)**
    *   **Physics**: Constant voltage change ($y = x$).
    *   **Perception**: Sounds "rushed". The volume seems to jump to 80% loudness almost instantly, then changes very little for the rest of the ramp.
    *   **Use Case**: Diagnostic testing or old-school 8-bit game audio simulation.

3.  **Green Curve (S-Curve / Sigmoid)**
    *   **Physics**: Ease-in, ease-out ($y = 1 / (1 + e^{-x})$).
    *   **Perception**: A gentle "mechanical" ramp. Very slow start, fast middle, smooth landing.
    *   **Use Case**: System UI sounds, notifications (avoids "clicky" attacks on short samples).

---

## :wrench: Configuration

Customize the envelope behavior via `sysctl`. Changes take effect immediately on the next stream start.

### 1. Ramp Duration (`ramp_ms`)
Controls the envelope attack time for standard playback.

*   `sysctl dev.acpi_intel_sst.0.ramp_ms` (0-500 ms)
    *   **0 ms**: **Purist Mode / Bit-Perfect**. Signals start instantly. Necessary for low-latency pro-audio work or system alerts, but risks pops.
    *   **50 ms** (Default): **Transparent**. Fast enough to feel "instant" (>20Hz cycle), slow enough to suppress all DC offset pops.
    *   **200+ ms**: **Cinematic**. Creates a noticeable "fade-in" effect.

### 2. Resume Duration (`resume_ramp_ms`)
Separate control for S3 sleep wake-up. Hardware often requires longer stabilization time after power-gating.

*   `sysctl dev.acpi_intel_sst.0.resume_ramp_ms` (Default: 50 ms)
    *   *Tip:* If you hear a "thump" when opening your laptop lid, try increasing this to **100-200ms**.

### 3. Curve Shape (`ramp_curve`)
Selects the interpolation algorithm.

*   `sysctl dev.acpi_intel_sst.0.ramp_curve`
    *   `0`: **Logarithmic** (Recommended for Music/Hi-Fi)
    *   `1`: **Linear**
    *   `2`: **S-Curve**

---

### :microscope: Technical Implementation

The ramping logic is implemented in the `sst_pcm` driver layer, utilizing the `callout(9)` high-resolution kernel timer.

```c
/* Logarithmic ramp table (perceptual linearization) */
static const int sst_ramp_log[] = {
     10,   /* Step 1: -40 dB (start barely audible) */
     32,   /* Step 2: -30 dB */
     80,   /* Step 3: -22 dB */
    ...
    1000,  /* Step 10: 0 dB  (Unity) */
};
```

When a stream opens:
1.  Target volume `V_target` is cached.
2.  Hardware volume is set to `0` (Mute).
3.  A timer fires every `10ms`.
4.  Each tick, the driver calculates: $Gain_{current} = V_{target} \times Table[step]$.
5.  The new Q1.31 gain coefficient is sent to the DSP via IPC `SET_VOLUME`.

This ensures the ramp happens **digitally** before the DAC, guaranteeing a pop-free output even if the analog amplifier has a noisy power-on characteristic.
