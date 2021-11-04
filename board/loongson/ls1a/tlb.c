#include <common.h>
#include <asm/mipsregs.h>
#include <configs/ls2k.h>
#include <config.h>
#define ST0_ERL			0x00000004
#define PM_4K		0x00000000
#define PM_16K		0x00006000
#define PM_64K		0x0001e000
#define PM_256K		0x0007e000
#define PM_1M		0x001fe000
#define PM_4M		0x007fe000
#define PM_16M		0x01ffe000
#define PM_64M		0x07ffe000
#define PM_256M		0x1fffe000

#define PAGE_SHIFT	12
#define PAGE_SIZE	(1UL << PAGE_SHIFT)
#define PAGE_MASK       (~((1 << PAGE_SHIFT) - 1))

#define ASID_MASK	0xff


int tlb_set(int tlbs, int tlbe,int cachetype,unsigned int virtaddr, unsigned long long phyaddr)
{
	int pid;
	int eflag=0;
	int i;

	pid = read_c0_entryhi() & ASID_MASK;

	for(i=tlbs;i<tlbe;i++)
	{
		write_c0_index(i);
		write_c0_pagemask(PM_16M);
		write_c0_entryhi(virtaddr | (pid));
		write_c0_entrylo0((phyaddr >> 6)|cachetype); //uncached,global
		write_c0_entrylo1(((phyaddr+(16<<20)) >> 6)|cachetype);
		tlb_write_indexed();
		virtaddr += 32<<20;
		phyaddr += 32<<20;
		if(virtaddr>=0x80000000)break;
	}

	return 0;
}

/*
0-0x3fffffff: uncache 0-1G
0x40000000-0x7fffffff: cached 0-1G
*/

int tlb_init_default()
{
	tlb_set(0, 64, 0, 0x80000000, 0);
	tlb_set(0,  32, (2<<3)|7,          0, 0x100000000ULL);
	tlb_set(32, 64, (3<<3)|7, 0x40000000, 0x100000000ULL);
        return 0;
}

