#!/bin/sh
set -x

ls -la /dev/mem

dd if=/dev/mem bs=4 count=1 skip=$((0xFE000000/4)) 2>/dev/null | hexdump -C

devinfo -r | grep -i "0xfe0"

vmstat -m | head

sysctl dev.drm

mixer 

kldstat

lspci

pciconf -lv | grep -A5 "vgapci0"
pciconf -r pci0:31:0:0 0x80
pciconf -r pci0:31:0:0 0x84
pciconf -r pci0:31:0:0 0xF0

acpiconf | grep -i audio

devinfo -rv | grep -A10 acpi0
    
pciconf -lv | grep -i gpio

cat /boot/firmware/intel/IntcSST2.bin | hexdump -C | head

cat /dev/sndstat

sysctl hw.dmar.enable

cat /boot/loader.conf

dmesg

acpidump -dt
