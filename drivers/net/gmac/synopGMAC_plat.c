/**\file
 *  This file defines the wrapper for the platform/OS related functions
 *  The function definitions needs to be modified according to the platform 
 *  and the Operating system used.
 *  This file should be handled with greatest care while porting the driver
 *  to a different platform running different operating system other than
 *  Linux 2.6.xx.
 * \internal
 * ----------------------------REVISION HISTORY-----------------------------
 * Synopsys			01/Aug/2007			Created
 */
 
#include "synopGMAC_plat.h"
#define NET_MEMALIGN 32
#ifndef tobus
#define tobus(x) (x)
#endif

dma_addr_t __attribute__((weak)) gmac_dmamap(unsigned long va,u32 size)
{
	return tobus(va);
}


/**
  * This is a wrapper function for Memory allocation routine. In linux Kernel 
  * it it kmalloc function
  * @param[in] bytes in bytes to allocate
  */

void *plat_alloc_memory(u32 bytes) 
{
	return  memalign(NET_MEMALIGN, bytes);
}

/**
  * This is a wrapper function for consistent dma-able Memory allocation routine. 
  * In linux Kernel, it depends on pci dev structure
  * @param[in] bytes in bytes to allocate
  */

//void *plat_alloc_consistent_dmaable_memory(struct synopGMACdevice *dev, u32 size, u32 *addr) 
void *plat_alloc_consistent_dmaable_memory(struct pci_dev *pcidev, u32 size, u32 *dma_addr) 
{
void *buf;
	buf = memalign(NET_MEMALIGN, size);
	flush_cache(buf, size);
	//map to uncached memory
	*dma_addr = gmac_dmamap(buf, size);
#ifndef CONFIG_DMA_COHERENT
	buf = (void *)CACHED_TO_UNCACHED(buf);
#endif
 return buf;
}


/**
  * This is a wrapper function for freeing consistent dma-able Memory.
  * In linux Kernel, it depends on pci dev structure
  * @param[in] bytes in bytes to allocate
  */


//void plat_free_consistent_dmaable_memory(void * addr) 
void plat_free_consistent_dmaable_memory(struct pci_dev *pcidev, u32 size, void * addr,u32 dma_addr) 
{
	free(PHYS_TO_CACHED(UNCACHED_TO_PHYS(addr)));
 return;
}



/**
  * This is a wrapper function for Memory free routine. In linux Kernel 
  * it it kfree function
  * @param[in] buffer pointer to be freed
  */
void plat_free_memory(void *buffer) 
{
	free(buffer);
	return ;
}



dma_addr_t plat_dma_map_single(void *hwdev, void *ptr,
		                    u32 size)
{
	    unsigned long addr = (unsigned long) ptr;
//CPU_IOFlushDCache(addr,size, direction);
	flush_cache(addr, size);
return gmac_dmamap(addr, size);
}

/**
  * This is a wrapper function for platform dependent delay 
  * Take care while passing the argument to this function 
  * @param[in] buffer pointer to be freed
  */
void plat_delay(u32 delay)
{
	while (delay--);
	return;
}


