#!/bin/sh
set -x

dmesg | grep -i sst

# Check PCH Device Control register (LPC bridge)
pciconf -r pci0:31:0:0 0x80
pciconf -r pci0:31:0:0 0x84

# Check if there's a RCBA (Root Complex Base Address) we can access
pciconf -r pci0:31:0:0 0xF0

acpiconf | grep -i audio

devinfo -rv | grep -A10 acpi0

# Check if there's a GPIO controller
pciconf -lv | grep -i gpio

cat /boot/firmware/intel/IntcSST2.bin | hexdump -C | head

cat /dev/sndstat

acpidump -dt
