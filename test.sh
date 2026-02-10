#!/bin/sh
set -x

ls -la /boot/acpi_dsdt.aml

ls -la /dev/mem

dd if=/dev/mem bs=4 count=1 skip=$((0xFE000000/4)) 2>/dev/null | hexdump -C

devinfo -r

vmstat -m | head

sysctl dev.drm

mixer 

sysctl hw.acpi

kldstat

lspci

pciconf -lv

acpiconf | grep -i audio

devinfo -rv | grep -A10 acpi0
    
cat /boot/firmware/intel/IntcSST2.bin | hexdump -C | head

cat /dev/sndstat

sysctl -a

cat /boot/loader.conf

dmesg

acpidump -dt
