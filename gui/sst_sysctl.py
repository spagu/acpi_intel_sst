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
    """
    Run a command and return its result as text.

    errors="replace" is not optional here: /var/log/messages carries whatever
    a driver chose to print, and a stray non-UTF-8 byte anywhere in it would
    otherwise raise UnicodeDecodeError and take out version detection and the
    diagnostic report - the two things most wanted when something is wrong.
    """
    return subprocess.run(
        args, capture_output=True, timeout=15,
        text=True, encoding="utf-8", errors="replace",
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


def driver_version():
    """
    Version of the loaded driver.

    Taken from the line it prints on attach, because the driver exposes no
    version sysctl. dmesg is preferred over the log file: after a long uptime
    the ring buffer may have wrapped, and the log then still has it.
    """
    for cmd in (["dmesg"], ["cat", "/var/log/messages"]):
        r = _run(cmd)
        if r.returncode != 0:
            continue
        hits = [ln for ln in r.stdout.splitlines()
                if "Intel SST Driver v" in ln]
        if hits:
            return hits[-1].split("Intel SST Driver v")[-1].split()[0]
    return None


def dump():
    """
    Everything worth pasting into a bug report.

    Collected in one go so the reporter does not have to know which of two
    dozen sysctls matter - a report missing the one relevant line costs a
    round trip, and people reasonably do not want to guess.
    """
    lines = []

    v = driver_version()
    lines.append(f"driver:  acpi_intel_sst {v or 'unknown'}")

    for cmd, label in (
        (["uname", "-a"], "system"),
        (["sysctl", "-n", "hw.model"], "cpu"),
        (["kenv", "smbios.system.product"], "machine"),
        (["kenv", "smbios.bios.version"], "bios"),
    ):
        r = _run(cmd)
        if r.returncode == 0 and r.stdout.strip():
            lines.append(f"{label}:  {r.stdout.strip()}")

    lines.append("")
    lines.append("--- sysctl ---")
    r = _run(["sysctl", BASE])
    if r.returncode == 0:
        for ln in r.stdout.splitlines():
            if "%" not in ln.split(":")[0]:
                lines.append(ln)

    lines.append("")
    lines.append("--- recent driver messages ---")
    r = _run(["dmesg"])
    if r.returncode == 0:
        hits = [ln for ln in r.stdout.splitlines() if "acpi_intel_sst" in ln]
        lines.extend(hits[-25:] if len(hits) > 25 else hits)

    r = _run(["cat", "/dev/sndstat"])
    if r.returncode == 0:
        lines.append("")
        lines.append("--- sndstat ---")
        lines.extend(r.stdout.strip().splitlines())

    return "\n".join(lines)
