
# Volume Ramping (Soft Start)

<img src="volume_ramp_curves.png" alt="Volume Ramp Curves" width="600">

## What is Volume Ramping?

Volume ramping (or "soft start") is a feature designed to enhance the audio experience by eliminating sudden, jarring sounds.

When audio playback starts, or when the system wakes up from sleep, the audio hardware (DAC, amplifier) often powers on abruptly. Without ramping, this can cause:
1.  **"Pops" and "Clicks"**: Transient noise as the hardware stabilizes.
2.  **Sudden Blasts**: If you left your volume at 100%, music might start instantly at full blast, which is startling and potentially damaging to hearing or speakers.

**Volume Ramping** solves this by starting playback at 0% volume and smoothly increasing it to your target volume over a short duration (e.g., 50 milliseconds). This feels like a natural "fade-in".

## Configuration

You can customize the ramping behavior using `sysctl` variables.

### 1. Duration (`ramp_ms`)

Controls how long the fade-in lasts during normal playback start.

*   **Variable**: `dev.acpi_intel_sst.0.ramp_ms`
*   **Range**: `0` to `500` (milliseconds)
*   **Default**: `50` ms
*   **Effect**:
    *   `0`: **Disabled**. Audio starts instantly. Good for system sounds (alerts, notifications) where you need immediate feedback, but may cause pops.
    *   `50-100`: **Recommended**. Short enough to feel instant, long enough to mask pops.
    *   `500`: **Fade Effect**. Noticeable half-second fade-in. Good for smooth listening.

```bash
# Set ramp to 100ms (smoother)
sysctl dev.acpi_intel_sst.0.ramp_ms=100

# Disable ramping (instant start)
sysctl dev.acpi_intel_sst.0.ramp_ms=0
```

### 2. Resume Duration (`resume_ramp_ms`)

Controls the fade-in duration specifically after waking from system suspend (S3 sleep). Hardware often needs more time to stabilize after sleep to avoid a loud "thump".

*   **Variable**: `dev.acpi_intel_sst.0.resume_ramp_ms`
*   **Range**: `0` to `500` (milliseconds)
*   **Default**: `50` ms (driver default, often good to increase this if you hear pops on wake)

```bash
# Increase resume ramp to hide wake-up pops
sysctl dev.acpi_intel_sst.0.resume_ramp_ms=200
```

### 3. Ramp Curve (`ramp_curve`)

Controls the *shape* of the volume increase. This changes how the fade-in "feels" to your ear.

*   **Variable**: `dev.acpi_intel_sst.0.ramp_curve`
*   **Values**: `0, 1, 2`
*   **Default**: `0` (Logarithmic)

| Value | Type | Description | Best For |
|:---:|:---|:---|:---|
| **0** | **Logarithmic** | **Perceptually Linear**. Since human hearing is logarithmic (we hear in decibels), this curve increases amplitude exponentially. To your ear, it sounds like a smooth, constant increase in loudness from silence to full volume. | **Music, Movies, General Use** (Recommended) |
| **1** | **Linear** | **Mathematically Linear**. Amplitude increases at a constant rate. To your ear, this sounds like it gets loud *very quickly* at the start, and then changes slowly at the end. | Diagnostic / Testing |
| **2** | **S-Curve** | **Sigmoid**. Starts slowly, accelerates in the middle, and slows down at the end. Provides a smooth mechanical "ease-in" and "ease-out". | Sound Effects / UI sounds |

#### Visual Comparison

*   **Red (Logarithmic)**: Starts flat (quiet) and rises sharply at the end. This counteracts the ear's sensitivity to low volumes, resulting in a smooth perceived fade.
*   **Blue (Linear)**: Straight line. Sounds rushed at the beginning.
*   **Green (S-Curve)**: Smooth start and end. Use if you prefer a "soft mechanical" feel.

```bash
# Set curve to S-Curve
sysctl dev.acpi_intel_sst.0.ramp_curve=2
```
