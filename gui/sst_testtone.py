# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Test signal for comparing equaliser settings by ear.

Reading a response curve tells you what a preset does; hearing it tells you
whether you want it. The two are not the same, and the second is the one that
decides.

The signal is a logarithmic sweep from 40 Hz to 16 kHz. A sweep rather than
music because it is the fairest way to hear an equaliser: it visits every
band in turn, so a boost at 2 kHz is audible as the sweep passes through
rather than hidden behind whatever the track happens to be doing there.
"""

import math
import os
import struct
import subprocess
import tempfile

RATE = 48000
SECONDS = 2.5
F_START, F_END = 40.0, 16000.0
AMPLITUDE = 0.28          # comfortable, and leaves room for a boosted preset

_cached = None


def _generate():
    """
    Build the sweep as raw 16-bit stereo.

    Phase is accumulated rather than computed per sample from an instantaneous
    frequency: the naive version produces a discontinuity that clicks, which
    on a test signal sounds exactly like the distortion you are listening for.
    """
    n = int(RATE * SECONDS)
    k = math.log(F_END / F_START)
    out = bytearray()
    phase = 0.0

    for i in range(n):
        t = i / n
        f = F_START * math.exp(k * t)
        phase += 2.0 * math.pi * f / RATE

        # fade the ends so the signal starts and stops without a click
        env = 1.0
        edge = int(RATE * 0.02)
        if i < edge:
            env = i / edge
        elif i > n - edge:
            env = (n - i) / edge

        v = int(32767 * AMPLITUDE * env * math.sin(phase))
        out += struct.pack("<hh", v, v)

    return bytes(out)


def sample_path():
    """Write the sweep to a temporary file once and reuse it."""
    global _cached
    if _cached and os.path.exists(_cached):
        return _cached
    fd, path = tempfile.mkstemp(prefix="sst-sweep-", suffix=".raw")
    with os.fdopen(fd, "wb") as f:
        f.write(_generate())
    _cached = path
    return path


def player_command(path):
    """
    How to play the raw sample.

    mpv is preferred because it takes the format on the command line and
    exits when done. Falling back to a plain copy onto /dev/dsp keeps the
    feature working on a system with no player installed at all - the device
    accepts raw 16-bit stereo directly, which is the whole point of OSS.
    """
    from shutil import which

    if which("mpv"):
        return ["mpv", "--no-video", "--really-quiet", "--ao=oss",
                "--demuxer=rawaudio",
                f"--demuxer-rawaudio-rate={RATE}",
                "--demuxer-rawaudio-channels=2",
                "--demuxer-rawaudio-format=s16le", path]
    return ["sh", "-c", f"cat {path} > /dev/dsp"]


def play_async(path=None):
    """Start playback and return the process, so the caller can wait or kill."""
    path = path or sample_path()
    return subprocess.Popen(
        player_command(path),
        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)


def duration_ms():
    return int(SECONDS * 1000)
