# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2003-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

obj-y	=  ls2k.o
obj-y	+= lowlevel_init.o tlb.o
obj-$(CONFIG_LOONGSON2K_SPI) += spi.o
obj-$(CONFIG_LOONGSON2K_I2C) += ls2k-i2c.o

ifndef CONFIG_DM_PCI
obj-$(CONFIG_PCI) += pci.o
endif
