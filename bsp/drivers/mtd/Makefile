# SPDX-License-Identifier: GPL-2.0
#
# Makefile for the memory technology device drivers.
#

# Core functionality.

ccflags-y += -I $(srctree)/include/linux/mtd

obj-$(CONFIG_AW_MTD)           		+= parsers/
obj-$(CONFIG_AW_MTD)			+= awnand/
obj-$(CONFIG_AW_MTD_PSTORE)		+= mtdpstore.o
obj-$(CONFIG_SPIF_SUNXI)			+= spi-nor-6.18/controllers/
