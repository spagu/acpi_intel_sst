# Firmware module for acpi_intel_sst
#
# SPDX-License-Identifier: BSD-3-Clause
# Copyright (c) 2026 Tradik Limited
#
# Wraps IntcSST2.bin into acpi_intel_sst_fw.ko so the loader can
# preload it from /boot/loader.conf:
#
#     acpi_intel_sst_fw_load="YES"
#     acpi_intel_sst_load="YES"
#
# firmware_get(9) in the driver then finds "intel/IntcSST2.bin" already
# registered, which is the only way to load the driver from loader.conf
# on a ZFS root: at that point /boot/firmware is not readable yet
# (issue #51).  When the driver is loaded later (kld_list in rc.conf,
# kldload), this module is optional and /boot/firmware/intel/IntcSST2.bin
# is used instead.

KMOD=	acpi_intel_sst_fw

.PATH:	${.CURDIR}/ubuntu

# file:image-name:version - the name must match SST_FW_PATH_BDW
FIRMWS=	IntcSST2.bin:intel/IntcSST2.bin:1

.include <bsd.kmod.mk>
