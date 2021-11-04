/*	$Id: start.S,v 1.3 2004/05/17 10:39:22 wlin Exp $ */

/*
 * Copyright (c) 2001 Opsycon AB  (www.opsycon.se)
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *    must display the following acknowledgement:
 *	This product includes software developed by Opsycon AB, Sweden.
 * 4. The name of the author may not be used to endorse or promote products
 *    derived from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE AUTHOR ``AS IS'' AND ANY EXPRESS
 * OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
 * ARE DISCLAIMED.  IN NO EVENT SHALL THE AUTHOR BE LIABLE FOR ANY
 * DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS
 * OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY
 * OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF
 * SUCH DAMAGE.
 *
 */

#ifndef _KERNEL
#define _KERNEL
#endif

#include <config.h>
#include <asm/addrspace.h>
#include <asm/regdef.h>
#include <asm/mipsregs.h>
#include <asm/asm.h>
#include "ns16550.h"
#include "ls1a.h"
#include "../mm/2H_chip_define.h"

#define TTYDEBUG
#ifdef TTYDEBUG
#define	TTYDBG(x) \
	.pushsection .rodata;98: .asciz x; .popsection; la a0, 98b; bal stringserial; nop
#else
#define TTYDBG(x)
#endif
#define HAVE_TARGET_GETCHAR
#define	PRINTSTR(x) \
	.pushsection .rodata;98: .asciz x; .popsection; la a0, 98b; bal stringserial; nop

#define CONFIG_CACHE_64K_4WAY 1 

#define tmpsize		s1
#define msize		s2

/*
 * Coprocessor 0 register names
 */
#define CP0_INDEX $0
#define CP0_RANDOM $1
#define CP0_ENTRYLO0 $2
#define CP0_ENTRYLO1 $3
#define CP0_CONF $3
#define CP0_CONTEXT $4
#define CP0_PAGEMASK $5
#define CP0_WIRED $6
#define CP0_INFO $7
#define CP0_BADVADDR $8
#define CP0_COUNT $9
#define CP0_ENTRYHI $10
#define CP0_COMPARE $11
#define CP0_STATUS $12
#define CP0_CAUSE $13
#define CP0_EPC $14
#define CP0_PRID $15
#define CP0_CONFIG $16
#define CP0_LLADDR $17
#define CP0_WATCHLO $18
#define CP0_WATCHHI $19
#define CP0_XCONTEXT $20
#define CP0_FRAMEMASK $21
#define CP0_DIAGNOSTIC $22
#define CP0_PERFORMANCE $25
#define CP0_ECC $26
#define CP0_CACHEERR $27
#define CP0_TAGLO $28
#define CP0_TAGHI $29
#define CP0_ERROREPC $30

#define CP0_DEBUG  $23
#define CP0_DEPC   $24
#define CP0_DESAVE $31

#ifdef ACPI_S3_MODE

#define S3_REG_SAVE_AREA        0xffffffffa00f0000
#define GEN_RTC_1_REG        	0xffffffffbfef0050
#define PM1_CNT                 0xffffffffbfef0014
#define INTIEN0                 0xbfd00044
#define INTIEN1                 0xbfd0005c
#define INTIEN2                 0xbfd00074
#define INTIEN3                 0xbfd0008c
#define INTIEN4                 0xbfd000a4

#endif




/*
 *   Register usage:
 *
 *	s0	link versus load offset, used to relocate absolute adresses.
 *	s1	ddr param
 *	s2	memory size.
 *      s3      dimm param
 */


#ifdef BOOT_FROM_NAND

#define Index_Store_Tag_D			0x09
#define Index_Store_Tag_S			0x0B 
#define Index_Store_Tag_I                       0x08        


#define CONFREG_BASE 0xbfd00000
#define APB_BASE 0xbfe80000
#define NAND_BASE 0x60000
#define NAND_DMA_OFFSET 0x40
#define REMAP_REG 0xbfd80088
//#define REMAP_REG 0x1fd80088
#define REMAP_DDR 0x00e000f0 //0x80e00000 
#define REMAP_DDR_DMA 0x00e00000 //0x80e00000 

#endif

	.set	noreorder
	.set 	mips32
	.globl	__main
	.globl	lowlevel_init
lowlevel_init:
	move k0, ra

	bal	locate			/* Get current execute address */
	nop

uncached:

/*
 *  Reboot vector usable from outside pmon.
 */
	.align	9
ext_map_and_reboot:
	move	k1, ra		#save ra
	la	a0, v200_msg
	bal	stringserial
	nop
	b	exc_common

	.align	7			/* bfc00280 */
	move	k1, ra	#save ra
	la	a0, v280_msg
	bal	stringserial
	nop
	b	exc_common

/* Cache error */
	.align	8			/* bfc00300 */
	PRINTSTR("\r\nPANIC! Unexpected Cache Error exception! ")
	mfc0	a0, CP0_CACHEERR
	bal	hexserial
	nop
	b	exc_common

/* General exception */
	.align	7			/* bfc00380 */
	move	k1, ra		#save ra
	la	a0, v380_msg
	bal	stringserial
	nop
	b	exc_common
	
	.align	8			/* bfc00400 */
	move	k1, ra		#save ra
	la	a0, v400_msg
	bal	stringserial
	nop
	b	exc_common

	.align	7			/* bfc00480 */
#la	a0, v480_msg
#	bal	stringserial
#	nop
#b	exc_common
#if 0
	li s0,(0xbfc00000-0x81000000)
	PRINTSTR("ACPI_MEM_CEHCK=")
	li	a1, 0
	li	t0, 0xa0000000
	li	t2, 0xa0100000
1:	lw	t1,(t0)
	addu	a1, a1, t1
	addiu t0, t0, 4
	bne	t0,t2,1b
	nop
	addu a0,a1,zero
	bal	hexserial
	nop
	PRINTSTR("\r\n")
#endif

	/*acpi: set ddr autorefresh and suspend */
	#enable ddr confspace
	li  t0, 0xbfd00420	
	lw	t1, 0x0(t0)
	li  t2, 0xfffffffd
	and t1, t1, t2
	sw  t1, 0x0(t0)

	li	t0, 0xaffffe30
	lw	t1, 0x4(t0)
	li	t2, 0x1
	or	t1, t1, t2
	sw	t1, 0x4(t0)

	#disable ddr confspace
	li  t0, 0xbfd00420	
	lw	t1, 0x0(t0)
	li  t2, 0x1
	or  t1, t1, t2
	sw  t1, 0x0(t0)


	li	t0, 0xbfe7c008
	lw	t1, 0x0(t0)
	ori t1, t1, 0x2000
	sw	t1, 0x0(t0)
	
	.align	8			/* bfc00500 */
exc_common:
	PRINTSTR("\r\nCAUSE=")
	mfc0	a0, CP0_CAUSE
	bal	hexserial
	nop
	PRINTSTR("\r\nSTATUS=")
	mfc0	a0, CP0_STATUS
	bal	hexserial
	nop
	PRINTSTR("\r\nERRORPC=")
	mfc0	a0, CP0_ERROREPC

	bal	hexserial
	nop
	PRINTSTR("\r\nEPC=")
	mfc0	a0, CP0_EPC
	bal	hexserial
	nop
	PRINTSTR("\r\nra=")
	move	a0, k1
	bal	hexserial
	nop
1:
	b 1b
	nop



/*
 *  We get here from executing a bal to get the PC value of the current execute
 *  location into ra. Check to see if we run from ROM or if this is ramloaded.
 */
locate:
	move	s0, zero

	#bal	CPU_TLBClear
	#nop
	li	t0, ST0_BEV|0xc0000000
        mtc0    t0, CP0_STATUS
        mtc0    zero,CP0_CAUSE
	.set noreorder
/*spi speed*/
    li  t0, 0xbfe80000
    li  t1, 0x47  // div 4, fast_read + burst_en + memory_en
    sb  t1, 0x4(t0) // sfc_param
1:
#define DDRCFG_DATA (0x8888|(CPU_MULT-4)|((DDR_MULT-3)<<8))
    li  a1, DDRCFG_DATA   // 1fboard 25MHz
2:
    li  a0, 0xbfe78030
    sw  a1, 0x0(a0)
    sw  a1, 0x0(a0)
    sw  a1, 0x0(a0)
    nop
/*disable all gpio*/
	li a0,0xbfd00000
	sw zero,0x10c0(a0)
	sw zero,0x10c4(a0)
	sw zero,0x10c8(a0)
	sw zero,0x420(a0)

	li a0,0
	bal	initserial
	nop
	


	PRINTSTR("U-BOOT Initializing...\r\n");

// use only 8wins
#define CPU_WIN_BASE 0xbfd00000
#define CPU_WIN_MASK 0xbfd00040
#define CPU_WIN_MMAP 0xbfd00080

#define set_cpu_window(id, base, mask, mmap) \
        li      t0, CPU_WIN_BASE          ;  \
        sw      $0, 0x80+id*8(t0)         ;  \
        li      t1, base                  ;  \
        sw      t1, 0x00+id*8(t0)         ;  \
        sw      $0, 0x04+id*8(t0)         ;  \
        li      t1, mask                  ;  \
        sw      t1, 0x40+id*8(t0)         ;  \
        sw      $0, 0x44+id*8(t0)         ;  \
        li      t1, mmap                  ;  \
        sw      t1, 0x80+id*8(t0)         ;  \
        sw      $0, 0x84+id*8(t0)

#if 1//ndef    USE_PCI /* fixup cpu window */
cpu_win_fixup:
        //
        // hit         = (paddr & mask) == (mmap & mask)
        // mapped_addr =  paddr &~mask | mmap & mask
        //
        // mmap[7] -> enable
		// mmap[5] -> block trans enable,cache
        // mmap[4] -> fetch ins
        // mmap[1:0] -> destination
        //
        // NOTE: the address windows has priority, win0 > win1 > ... > win7
        set_cpu_window(7, 0x1fc00000, 0xfff00000, 0x1fc000f3) // boot rom
        set_cpu_window(0, 0x1fc00000, 0xfff00000, 0x1fc000f3) // boot rom
        set_cpu_window(1, 0x10000000, 0xf8000000, 0x10000081) // PCI mem0, mem1,    disabled
        set_cpu_window(2, 0x18000000, 0xfc000000, 0x18000081) // PCI mem2           disabled
        set_cpu_window(3, 0x1c000000, 0xffe00000, 0x1c000081) // PCI cfg/IO/header  disabled
        set_cpu_window(4, 0x1c200000, 0xffe00000, 0x1c200082) // gpu 1c2 /dc 1c3
        set_cpu_window(5, 0x1f000000, 0xff000000, 0x1f000083) // AXIMUX
        set_cpu_window(6, 0x40000000, 0xc0000000, 0x000000f0) // DDR 1GB
        set_cpu_window(7, 0x00000000, 0x00000000, 0x000000f0) // everything else
        // after this fixup, the kernel code should be compiled with
        // uncached instruction fetch patch
#endif 

	/* 
	 * Now determine DRAM configuration and size by
	 * reading the I2C EEROM on the DIMMS
	 */

#if 0
	/*
	 * set gpio 66 to make ddr #CKE low
	 */
	li	t0,0xbfd010c8
	li	t1,0x4
	sw	t1,(t0)
	li	t0,0xbfd010f8
	li	t1,0x0
	sw	t1,(t0)
#endif

#ddr2config by cww 20090901
        PRINTSTR("DDR2 config begin_2\r\n")
        bal     ddr2_config
        nop
        PRINTSTR("DDR2 config end\r\n")

	li a0,0xbfd00000
	sw zero,0x420(a0)
#ifdef CONFIG_DDR16BIT
/*16bit ddr and disable conf*/ 
	li a1,0x3
        li      msize,0x08000000
#else
/*disable conf*/
	li a1,0x2
        li      msize,0x10000000
#endif

#ifndef GMAC0
        li  a2, 0x800000 //bit 23   gmac0 shut
        or  a1,a2    
#endif
#ifndef GMAC1
        li  a2, 0x1000000 //bit 24  gmac1 shut
        or  a1,a2    
#endif

/*100M phy*/
#ifdef  CONFIG_PHY100M 
#ifdef GMAC0
        li a2,0x500 //bit 8, bit 10
        or  a1,a2    
#endif
#ifdef GMAC1
        li  a2,0xac0 //bit 9, bit 11, bit 6, bit 7
        or  a1,a2    
#endif
#endif
	sw a1,0x420(a0);


#define CF_7_SE         (1 << 3)        /* Secondary cache enable */
#define CF_7_SC         (1 << 31)       /* Secondary cache not present */
#define CF_7_TE         (1 << 12)       /* Tertiary cache enable */
#define CF_7_TC         (1 << 17)       /* Tertiary cache not present */
#define CF_7_TS         (3 << 20)       /* Tertiary cache size */
#define CF_7_TS_AL      20              /* Shift to align */
#define NOP8 nop;nop;nop;nop;nop;nop;nop;nop
do_caches:
	TTYDBG("Init caches...\r\n")

	li	s7, 0                   /* no L2 cache */
	li	s8, 0                   /* no L3 cache */

	TTYDBG("godson1 caches found\r\n")
    bal     cache_init
    nop

	TTYDBG("Init caches done, cfg = ")
	mfc0	a0, CP0_CONFIG
	bal	hexserial
	nop
	TTYDBG("\r\n\r\n")


    mfc0   a0,CP0_CONFIG
    and    a0,a0,~((1<<12) | 3)
	or     a0,a0,3
    mtc0   a0,CP0_CONFIG
	

acpi_begin:

/* Access ACPI controller to check out if the machine is resuming from suspend 
   	   Zeng Lu <zenglu@loongson.cn> */
#	li	t0,	0xbfe7c000
#	lw	t1,(t0)
#and t1,	t1,	(1<<15)//WAK_STS
#	beqz	t1,	acpi_end
#	nop
	/*clear wakeup events */
	li	t0, 0xbfe7c000
	li	t1, -1
	sw	t1, (t0)
	li	t0, 0xbfe7c020
	li	t1, -1
	sw	t1, (t0)

	li	t0,	0xbfe7c008
	lw	t1,	(t0)
#if 0 //clear slp_type if needed,this bit will be cleared in os
	li	t2, 0
	sw	t2, (t0)
#endif

	srl	t1,	10
	and	t1,	t1,	7//SLP_TYP
	sub	t1,	t1,	5
	bnez	t1,	acpi_end	/* Resuming from suspend */
	nop

	PRINTSTR("ACPI_CONTEXT=")
	li	t0,	0xa01ffc00
	li	t1,	0xa01ffc48
1:	lw	a0,	(t0)
	bal	hexserial
	nop
	PRINTSTR("\r\n")
	addiu t0,t0,4
	bne t0,t1,1b
	nop

	 mfc0   a0,CP0_CONFIG
    and    a0,a0,~((1<<12) | 3)
	or     a0,a0,3
    mtc0   a0,CP0_CONFIG

#if 0
	PRINTSTR("ACPI_MEM_CEHCK=")
	li	a1, 0
	#li	t0, 0xa0000000
	#li	t2, 0xa0100000
	li	t0, 0x80200000
	li	t2, 0x81200000
1:	lw	t1,(t0)
	addu	a1, a1, t1
	addiu t0, t0, 4
	bne	t0,t2,1b
	nop
	addu a0,a1,zero
	#li t0, 0x80100000
	#sw a1, (t0)
	bal	hexserial
	nop
	PRINTSTR("\r\n")
#endif


	bal	CPU_TLBClear
	nop
	PRINTSTR("ACPI_RESUME\r\n")
	bal	suspend_resume
	nop
	/* Resume will never get here */
1:
	b	1b
	nop
acpi_end:/* Startup as usual */


PRINTSTR("begin test\r\n");

###lcd soft_reset and panel config&timing
	li a0,0xbc301240
	li a1,0x00100000
	sw a1,0x0(a0)
	li a1,0x00000000
	sw a1,0x0(a0)

	li a0,0xbc3013c0
	li a1,0x80001111
	sw a1,0x0(a0)
	li a1,0x33333333
	sw a1,0x20(a0)

bootnow:
	jr	k0
	nop

/*
 *  Clear the TLB. Normally called from start.S.
 */
#if __mips64
#define MTC0 dmtc0
#else 
#define MTC0 mtc0
#endif

LEAF(CPU_TLBClear)
	li	a3, 0			# First TLB index.

	li	a2, PM_4K
	MTC0   a2, CP0_PAGEMASK   # Whatever...

1:
	MTC0   zero, CP0_ENTRYHI	# Clear entry high.
	MTC0   zero, CP0_ENTRYLO0	# Clear entry low0.
	MTC0   zero, CP0_ENTRYLO1	# Clear entry low1.

	mtc0    a3, CP0_INDEX	# Set the index.
	addiu	a3, 1
	li	a2, 64
	nop
	nop
	tlbwi				# Write the TLB

	bne	a3, a2, 1b
	nop

	jr	ra
	nop
END(CPU_TLBClear)

/*
 *  Set up the TLB. Normally called from start.S.
 */

/*
 * Resume the CPU state, jump to the kernel
 */
LEAF(suspend_resume)
	li	t0,	0xa01ffc00
	lw	ra,	(t0)
	lw	sp,	4(t0)
	lw	s8,	8(t0)
	lw	gp,	12(t0)
	lw	s0,	16(t0)
	lw	s1,	20(t0)
	lw	s2,	24(t0)
	lw	s3,	28(t0)
	lw	s4,	32(t0)
	lw	s5,	36(t0)
	lw	s6,	40(t0)
	lw	s7,	44(t0)

	lw	k0,	48(t0)
	lw	k1,	52(t0)

	lw	v0,	56(t0)
	lw	v1,	60(t0)

	lw	t1,	64(t0)
	mtc0	t1,	$12
	lw	t1,	68(t0)
	mtc0	t1,	$4
	lw	t1,	72(t0)
	mtc0	t1,	$5

	jr	ra
	nop
END(suspend_resume)

LEAF(stringserial)
	move	a2, ra
	addu	a1, a0, s0
	lbu	a0, 0(a1)
1:
	beqz	a0, 2f
	nop
	bal	tgt_putchar
	addiu	a1, 1
	b	1b
	lbu	a0, 0(a1)

2:
	j	a2
	nop
END(stringserial)

LEAF(hexserial)
	nop
	move	a2, ra
	move	a1, a0
	li	a3, 7
1:
	rol	a0, a1, 4
	move	a1, a0
	and	a0, 0xf
	la	v0, hexchar
	addu	v0, s0
	addu	v0, a0
	bal	tgt_putchar
	lbu	a0, 0(v0)

	bnez	a3, 1b
	addu	a3, -1

	j	a2
	nop
END(hexserial)

LEAF(tgt_putchar)
	nop
	.set noat
	move 	AT,ra
	la	v0, CONFIG_SYS_NS16550_COM1
	bal 1f
	nop
	jr AT
	nop
	.set at
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v1, LSR_TXRDY
	beqz	v1, 1b
	nop

	sb	a0, NSREG(NS16550_DATA)(v0)
	j	ra
	nop	
END(tgt_putchar)

LEAF(tgt_testchar)
	la	v0, CONFIG_SYS_NS16550_COM1
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v0,v1, LSR_RXRDY
	jr ra
	nop
END(tgt_testchar)

LEAF(tgt_getchar)
	la	v0, CONFIG_SYS_NS16550_COM1
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v1, LSR_RXRDY
	beqz	v1, 1b
	nop
	lb	v0, NSREG(NS16550_DATA)(v0)
	jr ra
	nop
END(tgt_getchar)


/* baud rate definitions, matching include/termios.h */
#define B0      0
#define B50     50      
#define B75     75
#define B110    110
#define B134    134
#define B150    150
#define B200    200
#define B300    300
#define B600    600
#define B1200   1200
#define B1800   1800
#define B2400   2400
#define B4800   4800
#define B9600   9600
#define B19200  19200
#define B38400  38400
#define B57600  57600
#define B115200 115200

LEAF(initserial)
.set noat
	move 	AT,ra
	la	v0, CONFIG_SYS_NS16550_COM1
	bal 1f
	nop
	jr AT
	nop
.set at
	
1:
	li	v1, FIFO_ENABLE|FIFO_RCV_RST|FIFO_XMT_RST|FIFO_TRIGGER_4
	sb	v1, NSREG(NS16550_FIFO)(v0)
	li	v1, CFCR_DLAB
	sb	v1, NSREG(NS16550_CFCR)(v0)
 	li	v1, ((APB_CLK*DDR_MULT)/(2*16*CONS_BAUD))     // 8MHz crystal,   M[7:3]=6     1fboard
	sb	v1, NSREG(NS16550_DATA)(v0)
	srl	v1, 8
	sb	v1, NSREG(NS16550_IER)(v0)
	li	v1, CFCR_8BITS
	sb	v1, NSREG(NS16550_CFCR)(v0)
	li	v1, MCR_DTR|MCR_RTS
	sb	v1, NSREG(NS16550_MCR)(v0)
	li	v1, 0x0
	sb	v1, NSREG(NS16550_IER)(v0)
        
        #disable all interrupt
        li      v1, 0x0
        sb      v1, NSREG(NS16550_IER)(v0)
	j	ra
	nop
END(initserial)

__main:
	j	ra
	nop


	.rdata
transmit_pat_msg:
	.asciz	"\r\nInvalid transmit pattern.  Must be DDDD or DDxDDx\r\n"
v200_msg:
	.asciz	"\r\nPANIC! Unexpected TLB refill exception!\r\n"
v280_msg:
	.asciz	"\r\nPANIC! Unexpected XTLB refill exception!\r\n"
v380_msg:
	.asciz	"\r\nPANIC! Unexpected General exception!\r\n"
v400_msg:
	.asciz	"\r\nPANIC! Unexpected Interrupt exception!\r\n"
v480_msg:
	.asciz	"\r\nPANIC! You have been in the Ejtag Debug MOde Trap is 0!\r\n"
hexchar:
	.ascii	"0123456789abcdef"

	.text
	.align	2
/*
 *   I2C Functions used in early startup code to get SPD info from
 *   SDRAM modules. This code must be entirely PIC and RAM independent.
 */




#define Index_Store_Tag_D			0x09
#define Index_Invalidate_I			0x00
#define Index_Writeback_Inv_D			0x01

LEAF(nullfunction)
	jr ra
	nop
END(nullfunction)

	.ent	cache_init
    .global cache_init
    .set noreorder
cache_init:
        move t1,ra
####part 2####
cache_detect_4way:
    .set mips32
        mfc0    t4, CP0_CONFIG,1
	lui	v0, 0x7
        and    v0, t4, v0    
	srl	t3, v0, 16    #ic

	li	t5,0x800 #32*64
	srl	v1,t4,22
	andi	v1,7
	sll	t5,v1   #InstCacheSetSize
	sll	t5,t3	#t6 InstCacheSize


        andi    v0, t4, 0x0380
        srl     t7, v0, 7     #dc

	li	t6,0x800 #32*64
	srl	v1,t4,13
	andi	v1,7
	sll	t6,v1  #DataCacheSetSize
	sll	t6,t7  #t5 DataCacheSize

####part 3####
#	.set	mips32
	lui	a0, 0x8000
	addu	a1, $0, t5
	addu	a2, $0, t6
cache_init_d2way:
#a0=0x80000000, a1=icache_size, a2=dcache_size
#a3, v0 and v1 used as local registers
	mtc0	$0, CP0_TAGHI
	addu	v0, $0, a0
	addu	v1, a0, a2
1:	slt	a3, v0, v1
	beq	a3, $0, 1f
	nop
	mtc0	$0, CP0_TAGLO
	cache	Index_Store_Tag_D, 0x0(v0)	# 1 way
4:	beq	$0, $0, 1b
	addiu	v0, v0, 0x20
1:
cache_flush_i2way:
	addu	v0, $0, a0
	addu	v1, a0, a1
1:	slt	a3, v0, v1
	beq	a3, $0, 1f
	nop
	cache	Index_Invalidate_I, 0x0(v0)	# 1 way
4:	beq	$0, $0, 1b
	addiu	v0, v0, 0x20
1:
cache_flush_d2way:
	addu	v0, $0, a0
	addu	v1, a0, a2
1:	slt	a3, v0, v1
	beq	a3, $0, 1f
	nop
	cache	Index_Writeback_Inv_D, 0x0(v0) 	#1 way
4:	beq	$0, $0, 1b
	addiu	v0, v0, 0x20
	.set	mips0

1:
cache_init_finish:
    jr	t1
    nop
    .set reorder
	.end 	cache_init


#define REG_ADDRESS 0x0
#define CONFIG_BASE 0xaffffe00

#ddr2 by cww 20090901
#if 1
#include "ddr2fconfig.S"
#endif

