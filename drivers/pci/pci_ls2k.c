// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2015 Google, Inc
 */

#include <common.h>
#include <dm.h>
#include <pci.h>
#define PCI_ACCESS_READ  0
#define PCI_ACCESS_WRITE 1
#define PCIBIOS_SUCCESSFUL              0x00
#define PCIBIOS_DEVICE_NOT_FOUND        0x86

#define HT1LO_PCICFG_BASE      (int)0xba000000
#define HT1LO_PCICFG_BASE_TP1  (int)0xbb000000
int ls2k_pci_config_access(unsigned char access_type,
   pci_dev_t tag, int where, u32 * data)
{

	int busnum ;
	u32 addr, type;
	u32 addr_i, cfg_addr, reg_data;
	u32 datarp;
	void *addrp;
	int device;
	int function;
	int reg = where & ~3;

	busnum = PCI_BUS(tag);
	device = PCI_DEV(tag);
	function = PCI_FUNC(tag);
		

	if(busnum == 0){
		/* in-chip virtual-bus has no parent,
		    so access is routed to PORT_HEAD */
		if (device > 31 || device == 2) {
			*data = -1; /* only one Controller lay on a virtual-bus */
			return PCIBIOS_DEVICE_NOT_FOUND;
		}

		addr = (device << 11) | (function << 8) | reg;
		addrp = (void *)(HT1LO_PCICFG_BASE | (addr & 0xffff));
		type = 0;

	} else {
			/* the bus is child of virtual-bus(pcie slot),
			 * so use Type 0 access for device on it
			 */
			if (device > 0) {
				*data = -1;
				return PCIBIOS_DEVICE_NOT_FOUND;
			}


		addr = (busnum << 16) | (device << 11) | (function << 8) | reg;
		addrp = (void *)(HT1LO_PCICFG_BASE_TP1 | (addr));
		type = 0x10000;
	}

	if (access_type == PCI_ACCESS_WRITE)
		*(volatile unsigned int *)addrp = cpu_to_le32(*data);
	else {
		*data = le32_to_cpu(*(volatile unsigned int *)addrp);
		if(busnum == 0 && reg == PCI_CLASS_REVISION && (*data>>16) == PCI_CLASS_PROCESSOR_MIPS)
		*data = (PCI_CLASS_BRIDGE_PCI<<16) | (*data & 0xffff);
		if(busnum == 0 && reg == 0x3c &&  (*data &0xff00) == 0)
		*data |= 0x100;

		if (*data == 0xffffffff) {
			*data = -1;
			return PCIBIOS_DEVICE_NOT_FOUND;
		}
	}

	return PCIBIOS_SUCCESSFUL;
}

static int _pci_ls2k_read_config(struct udevice *bus, pci_dev_t bdf, uint offset,
				ulong *valuep, enum pci_size_t size)
{
	u32 data = 0;
	int ret = ls2k_pci_config_access(PCI_ACCESS_READ, bdf, offset, &data);

	if (size == PCI_SIZE_8)
		*valuep = (data >> ((offset & 3) << 3)) & 0xff;
	else if (size == PCI_SIZE_16)
		*valuep = (data >> ((offset & 3) << 3)) & 0xffff;
	else
		*valuep = data;
	return 0;
}

static int _pci_ls2k_write_config(struct udevice *bus, pci_dev_t bdf,
				 uint offset, ulong value, enum pci_size_t size)
{

	u32 data = 0;
	int ret;

	if (size == PCI_SIZE_32)
		data = value;
	else {
		ret = ls2k_pci_config_access(PCI_ACCESS_READ,
				bdf, offset, &data);
		if (ret != PCIBIOS_SUCCESSFUL)
			return 0;

		if (size == PCI_SIZE_8)
			data = (data & ~(0xff << ((offset & 3) << 3))) |
			    (value << ((offset & 3) << 3));
		else if (size == PCI_SIZE_16)
			data = (data & ~(0xffff << ((offset & 3) << 3))) |
			    (value << ((offset & 3) << 3));
	}

	ret = ls2k_pci_config_access(PCI_ACCESS_WRITE,
			bdf, offset, &data);

	return 0;
}

static const struct dm_pci_ops pci_ls2k_ops = {
	.read_config	= _pci_ls2k_read_config,
	.write_config	= _pci_ls2k_write_config,
};

static const struct udevice_id pci_ls2k_ids[] = {
	{ .compatible = "pci-ls2k" },
	{ }
};

U_BOOT_DRIVER(pci_ls2k) = {
	.name	= "pci_ls2k",
	.id	= UCLASS_PCI,
	.of_match = pci_ls2k_ids,
	.ops	= &pci_ls2k_ops,
};
