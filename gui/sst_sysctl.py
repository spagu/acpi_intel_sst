# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Rafal Rabczuk
"""
Access layer for the acpi_intel_sst sysctl tree.

Everything the driver exposes is a sysctl, so this is deliberately thin: read
with `sysctl -n`, write with `sysctl name=value`. Going through the command
rather than sysctlbyname(3) keeps the panel dependency-free and makes failures
legible - the error text the kernel returns is what the user sees.
"""

import subprocess

BASE = "dev.acpi_intel_sst.0"


class NotLoaded(Exception):
    """The driver is not present, so there is nothing to show."""


class ReadOnly(Exception):
    """The write was refused, almost always for want of privilege."""


def _run(args):
    return subprocess.run(
        args, capture_output=True, text=True, timeout=5,
    )


def available():
    """Is the driver loaded at all?"""
    return _run(["sysctl", "-n", f"{BASE}.debug"]).returncode == 0


def get(name):
    """Read one value as a string, or None when it does not exist."""
    r = _run(["sysctl", "-n", f"{BASE}.{name}"])
    if r.returncode != 0:
        return None
    return r.stdout.strip()


def get_int(name, default=0):
    v = get(name)
    if v is None:
        return default
    # peak_db_* come back as "-96.0 dB"; take the leading number
    try:
        return int(float(v.split()[0]))
    except (ValueError, IndexError):
        return default


def get_float(name, default=0.0):
    v = get(name)
    if v is None:
        return default
    try:
        return float(v.split()[0])
    except (ValueError, IndexError):
        return default


def set_int(name, value):
    """
    Write one value.

    Raises ReadOnly with the kernel's own message rather than a generic one:
    "Operation not permitted" and "Invalid argument" mean very different
    things to whoever is looking at the panel.
    """
    r = _run(["sysctl", f"{BASE}.{name}={int(value)}"])
    if r.returncode != 0:
        msg = (r.stderr or r.stdout).strip()
        raise ReadOnly(msg or f"nie udało się zapisać {name}")


def writable():
    """
    Can this process change settings?

    Established by writing a value back unchanged rather than by checking the
    uid, because the answer also depends on how the panel was launched and on
    any sudo rule in place.
    """
    cur = get("debug")
    if cur is None:
        return False
    try:
        set_int("debug", int(cur))
        return True
    except ReadOnly:
        return False


def snapshot(names):
    """Read several values in one go, for the periodic refresh."""
    return {n: get(n) for n in names}
