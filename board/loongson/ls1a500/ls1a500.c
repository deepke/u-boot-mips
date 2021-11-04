/*
 * Board initialize code for Lemote YL8089.
 *
 * (C) Yanhua <yanh@lemote.com> 2009
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2, or (at
 * your option) any later version.
 */

#include <common.h>
#include <command.h>
#include <asm/addrspace.h>
#include <asm/io.h>
#include <asm/reboot.h>
#include <pci.h>
#include <netdev.h>
#include <configs/ls2k.h>
//#include <linux/mtd/nand.h>
#include <video_fb.h>
#include <config.h>
#include <linux/sizes.h>
#include "ls1a500.h"
#include "pincfgs.c"

DECLARE_GLOBAL_DATA_PTR;

void flush_cache(ulong start_addr, ulong size)
{
}

void flush_dcache_range(unsigned long start, unsigned long stop)
{
}

void invalidate_dcache_range(ulong start_addr, ulong stop)
{
}


#if defined(CONFIG_SHOW_BOOT_PROGRESS)
void show_boot_progress(int progress)
{
	printf("Boot reached stage %d\n", progress);
}
#endif

#define ACPIBASE (int)0xbff6c000 
void tgt_reboot()
{
	*(volatile unsigned int *)(ACPIBASE + 0x30) = 1;
}

void tgt_poweroff()
{
	*(volatile unsigned int *)(ACPIBASE + 0xc) &= 0xffffffff;
	*(volatile unsigned int *)(ACPIBASE + 0x14) = 0x3c00;
}

void _machine_restart(void)
{
	unsigned long hi, lo;
	void (*f)(void) = (void *) (int)0xbfc00000;

	tgt_reboot();

	while (1);
	/* Not reach here normally */
	f();
}

int dram_init(void)
{
	gd->ram_size = CONFIG_SYS_MEM_SIZE;

	return 0;
}

int checkboard (void)
{
	printf("Board: ls2h ");

	printf("(CPU Speed %d MHz/ Bus @ %d MHz)\n", 800, 266);

	set_io_port_base(0x0);

	return 0;
}


s32  gmac_initialize(char* dev_name,u64 base_addr);

int board_early_init_f(void)
{
//*(volatile int *) (int)0xbfd00200 |= 0x01 << 26;
return 0;
}

int board_early_init_r(void)
{
	struct udevice *dev;

	cfg_all_pin_multi(default_pin_cfgs);
	ls1a500_devices_fixup();
	/*init dwc3*/
	*(volatile int *)(int)0xbf06ff04 |= 0x00040000;
#define GCTL ((int)0xbf060000+0xc110)
	*(volatile int *)GCTL = (*(volatile int *)GCTL & ~(3<<12))|(1<<12);
#define GUSB3PIPECTL ((int)0xbf060000+0xc2c0)
	*(volatile int *)GUSB3PIPECTL = (*(volatile int *)GUSB3PIPECTL & ~(3<<1))|(0<<1);
	*(volatile unsigned int *)0xbfe10104 &= ~(1<<5); /*disable usb prefetch*/	//xwr todo: maybe we do not need do this
        tlb_init_default();
#ifdef CONFIG_DM_PCI
	/* Trigger PCIe devices detection */
	pci_init();
#endif
	/* Cause the SATA device to do its early init */
	uclass_first_device(UCLASS_AHCI, &dev);
	return 0;
}

#ifdef CONFIG_CFB_CONSOLE
GraphicDevice sm712fb;
void *
video_hw_init(void)
{
	int i;
	GraphicDevice *pGD = (GraphicDevice *)&sm712fb;	

	pGD->frameAdrs = dc_init();
	pGD->winSizeX = FB_XSIZE;
	pGD->winSizeY = FB_YSIZE;
	pGD->plnSizeX = pGD->winSizeX;
	pGD->plnSizeY = pGD->winSizeY;

	pGD->gdfBytesPP = 16/8;
	pGD->gdfIndex = GDF_16BIT_565RGB;

	/* FIXME, Now we just hard write the memory size */
	pGD->memSize = 2*1024*1024;
	for(i=0;i<pGD->winSizeX*pGD->winSizeY*pGD->gdfBytesPP;i+=4)
	*(volatile int *)(pGD->frameAdrs+i) = 0;


	return (void *)pGD;
}

/* 
 * for 8 bit depth 
 * FIXME currently not implemented
 * */
void video_set_lut(unsigned int index, unsigned char r, unsigned char g, unsigned char b)
{

}
#endif

unsigned long long memorysize;
unsigned long long memorysize_high;
#define INTPOL1_REG 0xbfd00068
int tlb_init_default();

int misc_init_f(void)
{
#ifdef ARB_LEVEL
	save_board_ddrparam(0);
#endif
 return 0;
}

int misc_init_r(void)
{
	register volatile int raw_memsz asm ("k1");
	unsigned long long memsz;
	char env[512];
#ifdef CONFIG_SCSI_AHCI
#ifdef CONFIG_SCSI_AHCI_PLAT
	pci_dev_t pdev;
	u32 mmio_base;

	pdev=pci_find_device(SCSI_VEND_ID,SCSI_DEV_ID,0); /* get PCI Device ID */
	mmio_base = (u32)pci_map_bar(pdev, 0x10,
						PCI_REGION_MEM);
	if(!ahci_init(mmio_base))
	scsi_scan(1);
#elif !defined(CONFIG_DM_SCSI)
	puts("SCSI:  ");
	scsi_init();
#endif
#endif

	memsz = raw_memsz & 0xff;
	memsz = memsz << 29;
	memsz = memsz - 0x1000000;
	memsz = memsz >> 20;
	printf("memsz %lld\n", memsz);

	memorysize = memsz > 240 ? 240 << 20 : memsz << 20;
	memorysize_high = memsz > 240 ? (memsz - 240) << 20 : 0;

	sprintf(env, "%lld", memorysize / (1024 * 1024));
	//setenv ("memsize", env);

	sprintf(env, "%lld", memorysize_high / (1024 * 1024));
	//setenv ("highmemsize", env);
#ifdef CONFIG_DM_VIDEO
	dc_init();
#endif

#if 0
	/*disable spi instruct fetch before enter spi io mode*/
	if(*(volatile int *)(int)0xbfe10080 == 0x1fc000f2)
		*(volatile int *)(int)0xbfe10080 = 0x1fc00082;
#endif

	return 0;
}


int get_update(char *p)
{
}

void tgt_flashinfo(void *p, size_t * t)
{
 *t = 0x100000;
}

struct pci_config_data {
		int bus;
		int dev;
		int func;
		int interrupt;
		int primary;
		int secondary;
		int subordinate;
		unsigned int mem_start;
		unsigned int mem_end;
		unsigned int io_start;
		unsigned int io_end;
#define PCI_DEV		0x1
#define PCI_BRIDGE	0x2
		int type;
}__attribute__((aligned(4)));

struct pci_config_data pci_config_array[] = {
			/*	PCIE0-PORT0	*/
[0] = {
.bus = 0, .dev = 0x0, .func = 0, .interrupt = 40, .primary = 0, .secondary = 1,
.subordinate = 1, .mem_start = 0x40100000, .mem_end = 0x4fffffff, .type = PCI_BRIDGE,
.io_start = 0x18000000, .io_end = 0x180fffff,
},
			/*	PCIE0-PORT1	*/
[1] = {
.bus = 0, .dev = 0x1, .func = 0, .interrupt = 41, .primary = 0, .secondary = 4,
.subordinate = 4, .mem_start = 0x50000000, .mem_end = 0x53ffffff, .type = PCI_BRIDGE,
.io_start = 0x18100000, .io_end = 0x181fffff,
},
};

int pci_config_array_size = ARRAY_SIZE(pci_config_array);
int ls2k_version();
void ls_pcie_config_set(void)
{
	int i;

	for(i = 0;i < ARRAY_SIZE(pci_config_array);i++){
			ls_pcie_mem_fixup(pci_config_array + i);
			ls_pcie_interrupt_fixup(pci_config_array + i);
			ls_pcie_busnr_fixup(pci_config_array + i);
			ls_pcie_payload_fixup(pci_config_array + i);
	}
}


#define LS2K_PCI_IO_MASK 0x1ffffff


u32 _pci_conf_read(int bdf, int addr, enum pci_size_t size);
int _pci_conf_write(int bdf, int addr, u32 val, enum pci_size_t size);

#define _pci_conf_read32(bdf, addr) _pci_conf_read(bdf, addr, PCI_SIZE_32)
#define _pci_conf_write32(bdf, addr, val) _pci_conf_write(bdf, addr, val, PCI_SIZE_32)
#define _pci_conf_read16(bdf, addr) _pci_conf_read(bdf, addr, PCI_SIZE_16)
#define _pci_conf_write16(bdf, addr, val) _pci_conf_write(bdf, addr, val, PCI_SIZE_16)
void ls_pcie_mem_fixup(struct pci_config_data *pdata)
{
	unsigned int dev;
	unsigned int val;
	unsigned int io_start;
	unsigned int io_end;

	dev = PCI_BDF(pdata->bus, pdata->dev, pdata->func);
	val = _pci_conf_read32(dev, 0x00);
	/*	device on the slot	*/
	if ( val != 0xffffffff){
			if(pdata->type == PCI_DEV){
					/*write bar*/
					_pci_conf_write32(dev, 0x10, pdata->mem_start);
			}else{
					_pci_conf_write32(dev, 0x10, 0x0);
					/*write memory base and memory limit*/
					val = ((pdata->mem_start >> 16)&0xfff0)|(pdata->mem_end&0xfff00000);
					_pci_conf_write32(dev, 0x20, val);
					_pci_conf_write32(dev, 0x24, val);

					io_start = pdata->io_start & LS2K_PCI_IO_MASK;
					io_end = pdata->io_end & LS2K_PCI_IO_MASK;
					/*write io upper 16bit base and io upper 16bit limit*/
					val = ((io_start >> 16)&0xffff)|(io_end&0xffff0000);
					_pci_conf_write32(dev, 0x30, val);
					/*write io base and io limit*/
					val = ((io_start >> 8)&0xf0)|(io_end & 0xf000);
					val|= 0x1 | (0x1 << 8);
					_pci_conf_write16(dev, 0x1c, val);
			}
	}
}

void ls_pcie_busnr_fixup(struct pci_config_data *pdata)
{
	unsigned int dev;
	unsigned int val;

	dev = PCI_BDF(pdata->bus, pdata->dev, pdata->func);
	val = _pci_conf_read32(dev, 0x00);
	/*	device on the slot	*/
	if ( val != 0xffffffff){
			if(pdata->type == PCI_BRIDGE){
					/*write primary ,secondary and subordinate*/
					val = pdata->primary |(pdata->secondary << 8)|(pdata->subordinate << 16);
					_pci_conf_write32(dev, 0x18, val);
			}
	}
}

void ls_pcie_interrupt_fixup(struct pci_config_data *pdata)
{
	unsigned int dev;
	unsigned int val;

	dev = PCI_BDF(pdata->bus, pdata->dev, pdata->func);
	val = _pci_conf_read32(dev, 0x00);
	/*	device on the slot	*/
	if ( val != 0xffffffff)
			_pci_conf_write16(dev, 0x3c, pdata->interrupt|0x100);

	//mask the unused device
#if 0
#define PCICFG30_RECFG	0xbfe13808 /*GMAC0*/
#define PCICFG31_RECFG	0xbfe13810 /*GMAC1*/
#define PCICFG40_RECFG	0xbfe13818 /*OTG*/
#define PCICFG41_RECFG	0xbfe13820 /*EHCI*/
#define PCICFG42_RECFG	0xbfe13828 /*OHCI*/
#define PCICFG5_RECFG	0xbfe13830 /*GPU*/
#define PCICFG6_RECFG	0xbfe13838 /*DC*/
#define PCICFG7_RECFG	0xbfe13840 /*HDA*/
#define PCICFG8_RECFG	0xbfe13848 /*SATA*/
#define PCICFGf_RECFG	0xbfe13850 /*DMA*/
	//pcicfg31_recfg for gmac1
	inl(PCICFG31_RECFG) |= 0xf;
	dev = PCI_BDF(0, 3, 1);
	_pci_conf_write32(dev, 0, 0xffffffff);
	inl(PCICFG31_RECFG) &= 0xfffffff0;
#endif
}
#define  PCI_EXP_DEVCTL_READRQ  0x7000	/* Max_Read_Request_Size */
#define  PCI_EXP_DEVCTL_PAYLOAD 0x00e0  /* Max_Payload_Size */

void ls_pcie_payload_fixup(struct pci_config_data *pdata)
{
	unsigned int dev;
	unsigned int val;
	u16 max_payload_spt, control;

	dev = PCI_BDF(pdata->bus, pdata->dev, pdata->func);
	val = _pci_conf_read32(dev, 0x00);
	/*	device on the slot	*/
	if ( val != 0xffffffff){
			if(pdata->type == PCI_BRIDGE){
					/*set Max_Payload_Size & Max_Read_Request_Size*/
					max_payload_spt = 1;
					control = _pci_conf_read16(dev, 0x78);
					control &= (~PCI_EXP_DEVCTL_PAYLOAD & ~PCI_EXP_DEVCTL_READRQ);
					control |= ((max_payload_spt << 5) | (max_payload_spt << 12));
					_pci_conf_write16(dev, 0x78, control);
			}
	}
}

int board_fixup_before_linux(void)
{
	ls_pcie_config_set();
	/*set spi to memory mode before enter kernel. This 
 * 	fixup cpu guess wrong and fetch instruction from spi memory space 
 * 	when spi in io mode, which will cause spi logic deadlock.
 * 	 */
	*(volatile char *)(int)0xbfd00004 |= 1;
	*(volatile char *)(int)0xbfd40004 |= 1;
	return 0;
}

u32 _pci_conf_read(int bdf, int addr, enum pci_size_t size)
{
	u32 val = 0;
#ifdef CONFIG_DM_PCI
	int ret;
	struct udevice *dev;
	ret = dm_pci_bus_find_bdf(bdf, &dev);
	if (ret) {
		return -1;
	}
	dm_pci_read_config(dev, addr, &val, size);
#else
	switch (size) {
	case PCI_SIZE_8:
		pci_read_config_byte(dev, addr, &val);
		break;
	case PCI_SIZE_16:
		pci_read_config_word(dev, addr, &val);
		break;
	case PCI_SIZE_32:
	default:
		pci_read_config_dword(dev, addr, &val);
		break;
	}
#endif
	return val;
}

int _pci_conf_write(int bdf, int addr, u32 val, enum pci_size_t size)
{
#ifdef CONFIG_DM_PCI
	int ret;
	struct udevice *dev;
	ret = dm_pci_bus_find_bdf(bdf, &dev);
	if (ret) {
		return ret;
	}
	dm_pci_write_config(dev, addr, val, size);
	return 0;
#else
	switch (size) {
		case PCI_SIZE_8:
			pci_write_config_byte(bdf, addr, val);
			break;
		case PCI_SIZE_16:
			pci_write_config_word(bdf, addr, val);
			break;
		case PCI_SIZE_32:
		default:
			pci_write_config_dword(bdf, addr, value);
			break;
	}
#endif
	return 0;
}
