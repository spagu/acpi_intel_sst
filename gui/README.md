# sst-panel

A GTK3 control panel for the `acpi_intel_sst` driver.

## Why

The driver exposes 26 sysctls: a parametric equaliser, a limiter, volume
ramps, jack detection and live telemetry. They work perfectly well from the
command line, but tuning an equaliser by typing sysctl names is miserable, and
the peak meters only mean anything if you can watch them move.

## Running

```sh
sst-panel
```

Reading needs no privilege. **Changing** a setting does, because the sysctls
are root-writable only. The panel checks on startup and, when it cannot write,
says so in a banner and leaves the controls visible but disabled - an ordinary
user can still watch the telemetry, which is usually what you want when
diagnosing something.

To change settings, either run it as root:

```sh
sudo sst-panel
```

or allow the sysctl writes for a group. The narrow form, which permits exactly
this driver's tree and nothing else:

```
%wheel ALL=(root) NOPASSWD: /sbin/sysctl dev.acpi_intel_sst.0.*
```

## What is on each tab

| tab | contents |
|---|---|
| Korektor | preset, parametric band (frequency, gain, Q), high-pass cutoff |
| Limiter | threshold, release, live peak meters, clip counter |
| Rampy | volume ramp duration, resume ramp, ramp shape |
| Gniazdo | jack detection toggle and current state |
| Diagnostyka | driver debug level, codec I2C error count |

The telemetry refreshes four times a second. Everything else is read once at
startup and written when you move a control.

## Requirements

- `gtk3`
- `py311-pygobject` or newer (tested against py312-pygobject 3.54.5)
- the `acpi_intel_sst` driver loaded; the panel says so plainly if it is not

## Files

| file | role |
|---|---|
| `sst_sysctl.py` | access layer - reads and writes the sysctl tree |
| `sst_panel.py` | the interface |
| `sst-panel.desktop` | menu entry |
