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
#include <configs/ls1a.h>
//#include <linux/mtd/nand.h>
#include <video_fb.h>
#include <config.h>
#include <linux/sizes.h>
#include "ls1a.h"

DECLARE_GLOBAL_DATA_PTR;

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
	printf("Board: ls1a ");

	printf("(CPU Speed %d MHz/ Bus @ %d MHz)\n", 800, 266);

	set_io_port_base(0x0);

	return 0;
}


s32  gmac_initialize(char* dev_name,u64 base_addr);
int board_eth_init(bd_t *bis)
{
	int i = 0;
#if defined(CONFIG_GMAC) && defined(CONFIG_DM_ETH)
	char* name = "syn0";

	#define Gmac_base 0xbfe10000
	i = gmac_initialize(name, Gmac_base);
#endif
	return i;
}

int board_early_init_f(void)
{
/*disable usb reset*/
	*(volatile int *)0xbff10204 = 0x40000000;
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
int tlb_init_default();

int misc_init_f(void)
{
 return 0;
}

extern char wait_for_smp_call[];
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
#else
	puts("SCSI:  ");
	scsi_init();
#endif
#endif

	memorysize = raw_memsz;
	printf("memorysize %lld\n", memsz);

	/*disable spi instruct fetch before enter spi io mode*/
/*fix cpu guess window*/
//asm volatile("mtc0 %0,$22,5;mtc0 %1, $22, 4; mtc0 %2, $22, 5;mtc0 %3, $22, 4;mtc0 %4,$22,7"::"r"(0),"r"(0x10000000),"r"(1),"r"(0xf0000000),"r"(1));
*(volatile int *)0xbfd80080 = 0x1fc00083;
*(volatile int *)0xbfd800a0 = 0x1c200082;
*(volatile int *)0xbfd800a8 = 0x1f000093;
        //tlb_init_default();

	return 0;
}


int get_update(char *p)
{
}

void tgt_flashinfo(void *p, size_t * t)
{
 *t = 0x100000;
}

dma_addr_t __phys_to_dma(void *dev, long addr)
{
	return addr;
}

unsigned long __dma_to_phys(void *dev,
	dma_addr_t dma_addr)
{
	return dma_addr & 0x7fffffff;
}

void *
dma_alloc_coherent(void *dev, size_t size, dma_addr_t *dma_handle,
		   gfp_t flag)
{
	long addr;
	addr = (unsigned long)memalign(ARCH_DMA_MINALIGN, size);
	flush_dcache_range(addr, addr+size);
	*dma_handle  = __phys_to_dma(NULL, virt_to_phys(addr));
	return CKSEG1ADDR(addr);
}

void *
dma_alloc_noncoherent(void *dev, size_t size, dma_addr_t *dma_handle,
		   gfp_t flag)
{
	long addr;
	addr = (unsigned long)memalign(ARCH_DMA_MINALIGN, size);
	flush_dcache_range(addr, addr+size);
	*dma_handle  = __phys_to_dma(NULL, virt_to_phys(addr));
	return addr;
}


void dma_free_coherent(void *dev, size_t size, void *cpu_addr,
		    dma_addr_t dma_handle)
{
	free(CKSEG0ADDR(cpu_addr));
}

dma_addr_t
dma_map_single(void *dev, void *ptr, size_t size,
	       int direction)
{
	unsigned long addr = (unsigned long)ptr;
	flush_dcache_range(addr, addr+size);
	return __phys_to_dma(NULL, virt_to_phys(addr));
}

void
dma_unmap_single(void *dev, dma_addr_t dma_addr, size_t size,
		 int direction)
{
	unsigned long addr = phys_to_virt(__dma_to_phys(dev, dma_addr));
	flush_dcache_range(addr, addr+size);
}
