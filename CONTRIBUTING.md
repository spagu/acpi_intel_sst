# Contributing to Intel SST Audio Driver for FreeBSD

First off, thank you for considering contributing to this project! Every contribution helps bring audio support to FreeBSD users on Broadwell-U platforms.

## Table of Contents

- [Code of Conduct](#code-of-conduct)
- [Getting Started](#getting-started)
- [Development Setup](#development-setup)
- [Coding Standards](#coding-standards)
- [Submitting Changes](#submitting-changes)
- [Testing](#testing)
- [Areas Needing Help](#areas-needing-help)

---

## Code of Conduct

This project follows the [FreeBSD Code of Conduct](https://www.freebsd.org/internal/code-of-conduct/). Please be respectful and constructive in all interactions.

---

## Getting Started

### Prerequisites

- FreeBSD 15-CURRENT (or 14.x)
- FreeBSD source tree at `/usr/src`
- Git
- Basic knowledge of FreeBSD kernel development

### Fork and Clone

```bash
# Fork the repository on GitHub, then:
git clone https://github.com/YOUR_USERNAME/acpi_intel_sst.git
cd acpi_intel_sst
git remote add upstream https://github.com/spagu/acpi_intel_sst.git
```

---

## Development Setup

### Building

```bash
# Build the module
make

# Clean build artifacts
make clean
```

### Testing on Hardware

If you have compatible hardware (Intel Broadwell-U/Haswell):

```bash
# Load the module
sudo kldload ./acpi_intel_sst.ko

# Check dmesg for output
dmesg | grep sst

# Unload
sudo kldunload acpi_intel_sst
```

### Testing Without Hardware

You can still contribute to code review, documentation, and static analysis without compatible hardware.

---

## Coding Standards

### Style Guide

Follow the FreeBSD kernel style guide:

```c
/* Function comments use this style */
static int
function_name(int arg1, char *arg2)
{
    int local_var;

    /* Block comments for logic */
    if (condition) {
        statement;
    }

    return (0);
}
```

### Key Points

1. **Indentation**: Use tabs (8 spaces width)
2. **Line Length**: Keep lines under 80 characters
3. **Braces**: Opening brace on same line for control statements
4. **Function Definitions**: Return type on separate line
5. **Comments**: Use `/* */` style, not `//`
6. **Variable Names**: Use lowercase with underscores (snake_case)
7. **Constants**: Use UPPERCASE with underscores

### Headers

All source files should include:

```c
/*-
 * SPDX-License-Identifier: BSD-3-Clause
 *
 * Copyright (c) YEAR Your Name
 * All rights reserved.
 */
```

---

## Submitting Changes

### Branch Naming

Use descriptive branch names:
- `feature/firmware-loader`
- `fix/mmio-bounds-check`
- `docs/update-readme`

### Commit Messages

Follow conventional commit format:

```
type(scope): short description

Longer description if needed. Explain what and why,
not how.

Fixes: #123
```

Types: `feat`, `fix`, `docs`, `style`, `refactor`, `test`, `chore`

### Pull Request Process

1. Update documentation if needed
2. Add tests for new functionality
3. Ensure the module builds without warnings
4. Run the test script if possible
5. Create a PR with clear description
6. Respond to review feedback

---

## Testing

### Running Tests

```bash
# Run the test suite (requires root and compatible hardware)
sudo ./tests/test_module.sh --verbose

# Build-only test (no hardware needed)
make clean && make
```

### What to Test

- Module loads without panic
- Module unloads cleanly
- Resources are properly allocated/freed
- No memory leaks (check with `vmstat`)
- Proper error messages in dmesg

---

## Areas Needing Help

### High Priority

| Area | Difficulty | Description |
|------|------------|-------------|
| Firmware Loading | Hard | Implement SST/SOF firmware loader |
| IPC Protocol | Hard | Host-DSP communication layer |
| I2S Controller | Medium | SSP driver for codec |

### Medium Priority

| Area | Difficulty | Description |
|------|------------|-------------|
| Interrupt Handler | Medium | Implement IRQ handling |
| Power Management | Medium | Proper D3 suspend/resume |
| Testing | Easy | Test on different hardware |

### Documentation

| Area | Difficulty | Description |
|------|------------|-------------|
| Register Docs | Medium | Document SHIM registers |
| Architecture | Easy | Improve diagrams |
| Troubleshooting | Easy | Add common issues |

---

## Resources

### FreeBSD Development

- [FreeBSD Developer's Handbook](https://docs.freebsd.org/en/books/developers-handbook/)
- [Writing Device Drivers](https://docs.freebsd.org/en/books/arch-handbook/driverbasics/)
- [ACPI on FreeBSD](https://wiki.freebsd.org/ACPI)

### Intel SST

- [Linux SOF Project](https://github.com/thesofproject/linux)
- [Intel Audio DSP Documentation](https://01.org/sound-open-firmware)

---

## Questions?

- Open an issue for bugs or feature requests
- Discussions for general questions

Thank you for contributing!
