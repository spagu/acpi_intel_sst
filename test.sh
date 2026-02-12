#!/bin/sh
kldload ./acpi_intel_sst.ko || echo "Module already loaded or failed"
devctl detach pci0:19:0
devctl attach pci0:19:0
dmesg | tail -n 100 | grep -E "sst|acpi_intel_sst"
