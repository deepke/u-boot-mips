# SPDX-License-Identifier: GPL-2.0+
#
# (C) Copyright 2003-2006
# Wolfgang Denk, DENX Software Engineering, wd@denx.de.

obj-y	=  ls1a500.o
obj-y	+= lowlevel_init.o tlb.o
obj-$(CONFIG_LS1A500_SPI) += spi.o
obj-$(CONFIG_LS1A500_I2C) += ls1a500-i2c.o
ifneq ($(CONFIG_DM_VIDEO)$(CONFIG_CFB_CONSOLE),)
obj-y += dc.o
endif
ifndef CONFIG_DM_PCI
obj-$(CONFIG_PCI) += pci.o
endif
