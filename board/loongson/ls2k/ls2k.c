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
#include "ls2k.h"

DECLARE_GLOBAL_DATA_PTR;
#undef readq
#undef writeq
#define  readq readq_addr64
#define  writeq writeq_addr64

static inline unsigned long long readq_addr64(unsigned long long addr)
{
unsigned long long a = addr;
unsigned long long ret;
asm volatile( ".set mips64;ld $2,%0;ld $3,($2);sd $3,%1;.set mips0;\n"::"m"(a),"m"(ret):"$2","$3","memory")
;
return ret;
}


static inline void writeq_addr64(unsigned long long v ,unsigned long long addr)
{
unsigned long long a = addr;
asm volatile( ".set mips64;ld $2,%0; ld $3,%1;sd $3,($2);.set mips0;\n"::"m"(a),"m"(v):"$2","$3","memory")
;
}

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

void tgt_reboot()
{
	*(volatile unsigned int *)(int)0xbfe07030 = 1;
}

void tgt_poweroff()
{
	*(volatile unsigned int *)(int)0xbfe0700c &= 0xffffffff;
	*(volatile unsigned int *)(int)0xbfe07014 = 0x3c00;
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
int board_eth_init(struct bd_info *bis)
{
	int i = 0;
#ifdef CONFIG_GMAC 
	char* name = "syn0";
	pci_dev_t busdevfunc;
	void *Gmac_base;

	busdevfunc=pci_find_device(PCI_VENDOR_ID_LOONGSON,PCI_DEVICE_ID_LOONGSON_GMAC,0); /* get PCI Device ID */
	if(busdevfunc==-1) {
		printf("Error gmac Controller (%04X,%04X) not found\n",PCI_VENDOR_ID_LOONGSON, PCI_DEVICE_ID_LOONGSON_GMAC);
		return 0;
	}
	Gmac_base = pci_map_bar(busdevfunc, PCI_BASE_ADDRESS_0, PCI_REGION_MEM);
	i = gmac_initialize(name, Gmac_base);
#endif
	return i;
}

int board_early_init_f(void)
{
*(volatile int *) (int)0xbfd00200 |= 0x01 << 26;
return 0;
}

int board_early_init_r(void)
{
	struct udevice *dev;
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

extern char wait_for_smp_call[];
unsigned long long memorysize_total;
int misc_init_r(void)
{
	register volatile int raw_memsz asm ("k1");
	unsigned long long memsz;
	char env[512];
	asm volatile(".set mips64;sd %1,(%0);.set mips0;"::"r"(0xbfe11120),"r"(&wait_for_smp_call));
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
	memorysize_total = memsz;

	memorysize = memsz > 240 ? 240 << 20 : memsz << 20;
	memorysize_high = memsz > 240 ? (memsz - 240) << 20 : 0;

	sprintf(env, "%lld", memorysize / (1024 * 1024));
	//setenv ("memsize", env);

	sprintf(env, "%lld", memorysize_high / (1024 * 1024));
	//setenv ("highmemsize", env);

	/*disable spi instruct fetch before enter spi io mode*/
	if(*(volatile int *)(int)0xbfe10080 == 0x1fc000f2)
		*(volatile int *)(int)0xbfe10080 = 0x1fc000d2;

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
			/*		APB		*/
[0] = {
.bus = 0, .dev = 0x2, .func = 0, .interrupt = 0, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x1fe00000, .mem_end = 0x1fe0ffff, .type = PCI_DEV,
},
			/*		GMAC0	*/
[1] = {
.bus = 0, .dev = 0x3, .func = 0, .interrupt = 20, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40040000, .mem_end = 0x4004ffff, .type = PCI_DEV,
},
			/*		GMAC1	*/
[2] = {
.bus = 0, .dev = 0x3, .func = 1, .interrupt = 22, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40050000, .mem_end = 0x4005ffff, .type = PCI_DEV,
},
			/*		OTG		*/
[3] = {
.bus = 0, .dev = 0x4, .func = 0, .interrupt = 57, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40000000, .mem_end = 0x4003ffff, .type = PCI_DEV,
},
			/*		EHCI	*/
[4] = {
.bus = 0, .dev = 0x4, .func = 1, .interrupt = 58, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40060000, .mem_end = 0x4006ffff, .type = PCI_DEV,
},
			/*		OHCI	*/
[5] = {
.bus = 0, .dev = 0x4, .func = 2, .interrupt = 59, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40070000, .mem_end = 0x4007ffff, .type = PCI_DEV,
},
			/*		GPU		*/
[6] = {
.bus = 0, .dev = 0x5, .func = 0, .interrupt = 37, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x40080000, .mem_end = 0x400bffff, .type = PCI_DEV,
},
			/*		DC		*/
[7] = {
.bus = 0, .dev = 0x6, .func = 0, .interrupt = 36, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x400c0000, .mem_end = 0x400cffff, .type = PCI_DEV,
},
			/*		HDA		*/
[8] = {
.bus = 0, .dev = 0x7, .func = 0, .interrupt = 12, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x400d0000, .mem_end = 0x400dffff, .type = PCI_DEV,
},
			/*		SATA	*/
[9] = {
.bus = 0, .dev = 0x8, .func = 0, .interrupt = 27, .primary = 0, .secondary = 0,
.subordinate = 0, .mem_start = 0x400e0000, .mem_end = 0x400effff, .type = PCI_DEV,
},
			/*	PCIE0-PORT0	*/
[10] = {
.bus = 0, .dev = 0x9, .func = 0, .interrupt = 40, .primary = 0, .secondary = 1,
.subordinate = 1, .mem_start = 0x40100000, .mem_end = 0x4fffffff, .type = PCI_BRIDGE,
.io_start = 0x18000000, .io_end = 0x180fffff,
},
			/*	PCIE0-PORT1	*/
[11] = {
.bus = 0, .dev = 0xa, .func = 0, .interrupt = 41, .primary = 0, .secondary = 4,
.subordinate = 4, .mem_start = 0x50000000, .mem_end = 0x53ffffff, .type = PCI_BRIDGE,
.io_start = 0x18100000, .io_end = 0x181fffff,
},
			/*	PCIE0-PORT2	*/
[12] = {
.bus = 0, .dev = 0xb, .func = 0, .interrupt = 42, .primary = 0, .secondary = 8,
.subordinate = 8, .mem_start = 0x54000000, .mem_end = 0x57ffffff, .type = PCI_BRIDGE,
.io_start = 0x18200000, .io_end = 0x182fffff,
},
			/*	PCIE0-PORT3	*/
[13] = {
.bus = 0, .dev = 0xc, .func = 0, .interrupt = 43, .primary = 0, .secondary = 0xc,
.subordinate = 0xc, .mem_start = 0x58000000, .mem_end = 0x5fffffff, .type = PCI_BRIDGE,
.io_start = 0x18300000, .io_end = 0x183fffff,
},
			/*	PCIE1-PORT0	*/
[14] = {
.bus = 0, .dev = 0xd, .func = 0, .interrupt = 44, .primary = 0, .secondary = 0x10,
.subordinate = 0x10, .mem_start = 0x60000000, .mem_end = 0x77ffffff, .type = PCI_BRIDGE,
.io_start = 0x18400000, .io_end = 0x184fffff,
},
			/*	PCIE1-PORT1	*/
[15] = {
.bus = 0, .dev = 0xe, .func = 0, .interrupt = 45, .primary = 0, .secondary = 0x14,
.subordinate = 0x14, .mem_start = 0x78000000, .mem_end = 0x7fffffff, .type = PCI_BRIDGE,
.io_start = 0x18500000, .io_end = 0x185fffff,
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
	ls_pci_msi_window_config();

	if(!ls2k_version())
		ls_set_io_noncoherent();
	else
	{
		/*set dc coherent*/
		*(volatile int *)(int)0xbfe10430 |= 8; 
		map_gpu_addr();
	}
	write_c0_ebase((int)0x80000000);

}

void map_gpu_addr(void)
{
	if (memorysize_total == 0x800) {
		writeq( 0x20000000,(s64)(int)0xbfe10038 );
		writeq( 0xffffffffe0000000,(s64)(int)0xbfe10078 );
		writeq( 0x00000000600000f0,(s64)(int)0xbfe100b8 );
	} else if (memorysize_total == 0x1000) {
		writeq( 0x20000000,(s64)(int)0xbfe10038 );
		writeq( 0xffffffffe0000000,(s64)(int)0xbfe10078 );
		writeq( 0x00000000e00000f0,(s64)(int)0xbfe100b8 );
	} else if (memorysize_total == 0x2000) {
		writeq( 0x20000000,(s64)(int)0xbfe10038 );
		writeq( 0xffffffffe0000000,(s64)(int)0xbfe10078 );
		writeq( 0x00000001e00000f0,(s64)(int)0xbfe100b8 );
	} else {
		printf ("Now this Memory size %lld MB is not support mapping GPU address.\n", memorysize_total);
	}
}

void ls_set_io_noncoherent(void)
{
		u64 val;

		val = readq((s64)(int)0xbfe10420);
		val &= 0xfffffff8fffffffeULL; //pcie, usb, hda, gmac
		writeq( val,(s64)(int)0xbfe10420 );

		val = readq((s64)(int)0xbfe10430);
		val &= 0xfffffffffffffff3ULL; //dc, gpu
		writeq( val,(s64)(int)0xbfe10430 );

		val = readq((s64)(int)0xbfe10450);
		val &= 0xfffffffffffffbffULL; //sata
		writeq( val,(s64)(int)0xbfe10450 );

		val = readq((s64)(int)0xbfe10c00);
		val |= 0x2; //apbdma0
		writeq( val,(s64)(int)0xbfe10c00 );

		val = readq((s64)(int)0xbfe10c10);
		val |= 0x2; //apbdma1
		writeq( val,(s64)(int)0xbfe10c10 );

		val = readq((s64)(int)0xbfe10c20);
		val |= 0x2; //apbdma2
		writeq( val,(s64)(int)0xbfe10c20 );

		val = readq((s64)(int)0xbfe10c30);
		val |= 0x2; //apbdma3
		writeq( val,(s64)(int)0xbfe10c30 );

		val = readq((s64)(int)0xbfe10c40);
		val |= 0x2; //apbdma4
		writeq( val,(s64)(int)0xbfe10c40 );
}

void ls_pci_msi_window_config(void)
{
	/*config msi window*/
	writeq( 0x000000001fe10000ULL,(s64)(int)0xbfe12500 );
	writeq( 0xffffffffffff0000ULL,(s64)(int)0xbfe12540 );
	writeq( 0x000000001fe10081ULL,(s64)(int)0xbfe12580 );
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
