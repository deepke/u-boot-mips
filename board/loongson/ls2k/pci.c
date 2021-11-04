/*
 * Loongson 2F cpu's bonito likely PCI controller 
 *
 * (C) Copyright 2009 
 * Yanhua,  yanh@.lemote.com 
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at
 * your option) any later version.
 */

#include <common.h>
#include <pci.h>
#include <asm/addrspace.h>
#include <config.h>
#include <asm/io.h>
#define ls2h_readl(phys) readl((phys)|0xa0000000)
#define ls2h_writel(val, phys) writel(val, (phys)|0xa0000000)


#define PCI_ACCESS_READ  0
#define PCI_ACCESS_WRITE 1
#define PCIBIOS_SUCCESSFUL              0x00
#define PCIBIOS_DEVICE_NOT_FOUND        0x86

#define PCI_CACHE_LINE_SIZE 0x0c    /* 8 bits */
#define PCI_BASE_ADDRESS_0  0x10    /* 32 bits */
#define PCI_BASE_ADDRESS_1  0x14    /* 32 bits [htype 0,1 only] */
#define PCI_BASE_ADDRESS_2  0x18    /* 32 bits [htype 0 only] */
#define PCI_BASE_ADDRESS_3  0x1c    /* 32 bits */
#define PCI_BASE_ADDRESS_4  0x20    /* 32 bits */
#define PCI_BASE_ADDRESS_5  0x24    /* 32 bits */
#define PCI_INTERRUPT_LINE  0x3c    /* 8 bits */
#define  PCI_BASE_ADDRESS_SPACE_IO 0x01
#define  PCI_BASE_ADDRESS_SPACE_MEMORY 0x00
#define  PCI_BASE_ADDRESS_MEM_PREFETCH  0x08    /* prefetchable? */
#define IORESOURCE_IO       0x00000100  /* Resource type */
#define IORESOURCE_MEM      0x00000200
#define IORESOURCE_IRQ      0x00000400
#define IORESOURCE_DMA      0x00000800
#define IORESOURCE_PREFETCH 0x00001000  /* No side effects */
#define PCI_CLASS_BRIDGE_PCI		0x0604

#define PCI_CLASS_REVISION	0x08	/* High 24 bits are class, low 8 revision */
#define PCI_REVISION_ID		0x08	/* Revision ID */
#define PCI_CLASS_PROG		0x09	/* Reg. Level Programming Interface */
#define PCI_CLASS_DEVICE	0x0a	/* Device class */

#define le32_to_cpu(x) (x)
#define le64_to_cpu(x) (x)
#define cpu_to_le32(x) (x)
#define cpu_to_le64(x) (x)


#define CKSEG1ADDR(x) (x|0xa0000000)

typedef struct ls2h_pci_controller {
struct pci_controller hose;
int port;
} ls2h_pci_controller;

static u32 ls2h_pcie_bar_translate(unsigned char access_type, u32 bar_in, unsigned char portnum)
{
	return bar_in;
}

#define HT1LO_PCICFG_BASE      0xba000000
#define HT1LO_PCICFG_BASE_TP1  0xbb000000

//#define MYDBG printf("%s:%d\n", __FUNCTION__, __LINE__);
#define MYDBG

int ls2h_pci_config_access(unsigned char access_type,
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
		

MYDBG
	// if (!bus->parent) {
	if(busnum == 0){
		/* in-chip virtual-bus has no parent,
		    so access is routed to PORT_HEAD */
		if (device > 31 || device == 2) {
			*data = -1; /* only one Controller lay on a virtual-bus */
MYDBG
			return PCIBIOS_DEVICE_NOT_FOUND;
		}

		addr = (device << 11) | (function << 8) | reg;
		addrp = (void *)(HT1LO_PCICFG_BASE | (addr & 0xffff));
		type = 0;


	} else {
		if ((busnum%7) == 1) {
			/* the bus is child of virtual-bus(pcie slot),
			 * so use Type 0 access for device on it
			 */
			if (device > 0) {
				*data = -1;
				return PCIBIOS_DEVICE_NOT_FOUND;
			}
		}


		addr = (busnum << 16) | (device << 11) | (function << 8) | reg;
		addrp = (void *)(HT1LO_PCICFG_BASE_TP1 | (addr));
		type = 0x10000;
	}

MYDBG

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
int ls2h_pcibios_read_port(pci_dev_t tag, int where, int size, u32 * val)
{
	u32 data = 0;
	*val = -1;

	if (ls2h_pci_config_access(PCI_ACCESS_READ, tag, where, &data))
		return PCIBIOS_DEVICE_NOT_FOUND;

	if (size == 1)
		*val = (data >> ((where & 3) << 3)) & 0xff;
	else if (size == 2)
		*val = (data >> ((where & 3) << 3)) & 0xffff;
	else
		*val = data;

	return PCIBIOS_SUCCESSFUL;
}


int ls2h_pcibios_write_port(pci_dev_t tag, int where, int size, u32 val)
{
	u32 data = 0;

	if (size == 4)
		data = val;
	else {
		if (ls2h_pci_config_access(PCI_ACCESS_READ, tag, where, &data))
			return PCIBIOS_DEVICE_NOT_FOUND;

		if (size == 1)
			data = (data & ~(0xff << ((where & 3) << 3))) |
					(val << ((where & 3) << 3));
		else if (size == 2)
			data = (data & ~(0xffff << ((where & 3) << 3))) |
					(val << ((where & 3) << 3));
	}

	if (ls2h_pci_config_access(PCI_ACCESS_WRITE, tag,  where, &data))
		return PCIBIOS_DEVICE_NOT_FOUND;

	return PCIBIOS_SUCCESSFUL;
}

static int ls2h_pci_read_config_byte (struct pci_controller* hose, pci_dev_t dev, u32 reg, u8 * val)
{
	u32 val32;
	int ret;

	ret = ls2h_pcibios_read_port (dev, reg, 1, &val32);
	*val = val32;
	return ret;
}

static int ls2h_pci_read_config_word (struct pci_controller* hose, pci_dev_t dev, u32 reg, u16 * val)
{
	u32 val32;
	int ret;

	ret = ls2h_pcibios_read_port (dev, reg, 2, &val32);
	*val = val32;
	return ret;
}

static int ls2h_pci_read_config_dword (struct pci_controller* hose, pci_dev_t dev, u32 reg, u32 * val)
{
	u32 val32;
	int ret;

	ret = ls2h_pcibios_read_port (dev, reg, 4, &val32);
	*val = val32;
	return ret;
}


static int ls2h_pci_write_config_byte (struct pci_controller* hose, pci_dev_t dev, u32 reg, u8 val)
{
	return ls2h_pcibios_write_port (dev, reg, 1, val);
}

static int ls2h_pci_write_config_word (struct pci_controller* hose, pci_dev_t dev, u32 reg, u16 val)
{
	return ls2h_pcibios_write_port (dev, reg, 2, val);
}

static int ls2h_pci_write_config_dword (struct pci_controller* hose, pci_dev_t dev, u32 reg, u32 val)
{
	return ls2h_pcibios_write_port (dev, reg, 4, val);
}



struct pciiomem{
	long mem_bus_start, mem_phys_start, mem_size;
	long io_bus_start, io_phys_start, io_size;
} pciiomem[1] = {
	{0x10000000, 0x10000000, 0x08000000, 0x00004000, 0xb8000000, 0x100000-0x4000},
};

static ls2h_pci_controller hosts[1];
void pci_init_board (void)
{
	int i;
        struct pci_controller *hose;
        int last_busno;


	for(i=0;i<1;i++)
	{
		hosts[i].port = i;
		hose = &hosts[i].hose;
		hose->read_byte = ls2h_pci_read_config_byte;
		hose->read_word = ls2h_pci_read_config_word;
		hose->read_dword = ls2h_pci_read_config_dword;
		hose->write_byte = ls2h_pci_write_config_byte;
		hose->write_word = ls2h_pci_write_config_word;
		hose->write_dword = ls2h_pci_write_config_dword;
	}


        for(i=0, last_busno = 0;i<1;i++)
	{
        hose = &hosts[i].hose;

	hose->first_busno = last_busno;
	hose->last_busno = 0xff;


	/* PCI memory space #1 */
	pci_set_region (hose->regions + 0,
			pciiomem[i].mem_bus_start, pciiomem[i].mem_phys_start, pciiomem[i].mem_size, PCI_REGION_MEM);
	/* 
	 * PCI I/O space 
	 * FIXME, I don't like the urgly hacking
	 */
	pci_set_region (hose->regions + 1,
			pciiomem[i].io_bus_start, pciiomem[i].io_phys_start, pciiomem[i].io_size, PCI_REGION_IO);
	/* 
	 * System memory space 
	 * This is also a hacking, although I don't like it
	 * In fact the phys is virt address
	 */
	pci_set_region (hose->regions + 2,
			0x00000000,
			0x00000000,
			0x80000000, PCI_REGION_MEM | PCI_REGION_SYS_MEMORY);

	hose->region_count = 3;

	pci_register_hose (hose);

	hose->last_busno = pci_hose_scan (hose);
	last_busno = hose->last_busno + 1;
	printf("last_busno = %d\n", last_busno);
	}

}

#if 0
void *pci_map_bar(pci_dev_t pdev, int bar, int flags)
{
	pci_addr_t pci_bus_addr;
	u32 bar_response;

	/* read BAR address */
	pci_read_config_dword(pdev, bar, &bar_response);
	pci_bus_addr = (pci_addr_t)(bar_response & ~0xf);

	/*
	 * Pass "0" as the length argument to pci_bus_to_virt.  The arg
	 * isn't actualy used on any platform because u-boot assumes a static
	 * linear mapping.  In the future, this could read the BAR size
	 * and pass that as the size if needed.
	 */
	return uncached(pci_bus_addr);
}
#endif

