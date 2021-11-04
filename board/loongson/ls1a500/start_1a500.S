/* $Id: start.S,v 1.3 2004/05/17 10:39:22 wlin Exp $ */

/*
 * Copyright (c) 2001 Opsycon AB  (www.opsycon.se)
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *	notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *	notice, this list of conditions and the following disclaimer in the
 *	documentation and/or other materials provided with the distribution.
 * 3. All advertising materials mentioning features or use of this software
 *	must display the following acknowledgement:
 *	This product includes software developed by Opsycon AB, Sweden.
 * 4. The name of the author may not be used to endorse or promote products
 *	derived from this software without specific prior written permission.
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

#include <asm.h>
#include <regnum.h>
#include <cpu.h>
#include <pte.h>

#include "pmon/dev/ns16550.h"
#include "target/bonito.h"
#include "target/ls1a500.h"

#include "target/cacheops.h"	//mtf

#define TTYDEBUG
#ifdef TTYDEBUG
#define TTYDBG(x) \
	.rdata;98: .asciz x; .text; la a0, 98b; bal stringserial; nop
#else
#define TTYDBG(x)
#endif
#define HAVE_TARGET_GETCHAR
#define PRINTSTR(x) \
	.rdata;98: .asciz x; .text; la a0, 98b; bal stringserial; nop

#define CONFIG_CACHE_64K_4WAY 1
##define USEPCI

#define tmpsize		s1
#define msize		s2
#define sdShape		s3
#define bonito		s4
#define dbg		s5
#define sdCfg		s6

/*
 * Coprocessor 0 register names
 */
#define CP0_INDEX		$0
#define CP0_RANDOM		$1
#define CP0_ENTRYLO0		$2
#define CP0_ENTRYLO1		$3
#define CP0_CONF		$3
#define CP0_CONTEXT		$4
#define CP0_PAGEMASK		$5
#define CP0_WIRED		$6
#define CP0_INFO		$7
#define CP0_BADVADDR		$8
#define CP0_COUNT		$9
#define CP0_ENTRYHI		$10
#define CP0_COMPARE		$11
#define CP0_STATUS		$12
#define CP0_CAUSE		$13
#define CP0_EPC			$14
#define CP0_PRID		$15
#define CP0_CONFIG		$16
#define CP0_LLADDR		$17
#define CP0_WATCHLO		$18
#define CP0_WATCHHI		$19
#define CP0_XCONTEXT		$20
#define CP0_FRAMEMASK		$21
#define CP0_DIAGNOSTIC		$22
#define CP0_PERFORMANCE		$25
#define CP0_ECC			$26
#define CP0_CACHEERR		$27
#define CP0_TAGLO		$28
#define CP0_TAGHI		$29
#define CP0_ERROREPC		$30

#define CP0_DEBUG		$23
#define CP0_DEPC		$24
#define CP0_DESAVE		$31

/*
 * Register usage:
 * 
 * s0 link versus load offset, used to relocate absolute adresses.
 * s1 free
 * s2 memory size.
 * s3 sdShape.
 * s4 Bonito base address.
 * s5 dbg.
 * s6 sdCfg.
 * s7 rasave.
 * s8 L3 Cache size.
 */

	.set	noreorder
	.set 	mips3
	.globl	__main
	.globl	lowlevel_init
lowlevel_init:
	move fp, ra
	bal watchdog_close
	//bal watchdog_open
	nop

	/* init processor state at first*/
/* NOTE!! Not more that 16 instructions here!!! Right now it's FULL! */

	mtc0	zero, CP0_STATUS //cuckoo
	mtc0	zero, CP0_CAUSE
	li	t0, ST0_BEV	/* Exception to Boostrap Location */
	mtc0	t0, CP0_STATUS //cuckoo

	.set	mips32
	mfc0	t0, $16, 6		#Store fill
	.set	mips3
	li	t1, 0xfffffeff
	and	t0, t1, t0
	.set	mips32
	mtc0	t0, $16, 6		#Store fill
	.set	mips3
#if 0
	/* spi speedup */
	li	t0, 0xbfd00000
	li	t1, 0x47		//todo
	sb	t1, 0x4(t0)
#endif

	bal	locate			/* Get current execute address */
	nop

/*
 *  Exception vectors here for rom, before we are up and running. Catch
 *  whatever comes up before we have a fully fledged exception handler.
 */
	.align	9			/* bfc00200 */
	dla	a0,_start
	move	k1, ra		#save ra
	dla	a0, v200_msg
	bal	stringserial
	nop
	b	exc_common

	.align	7			/* bfc00280 */
	move	k1, ra	#save ra
	dla	a0, v280_msg
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
	dla	a0,_start
	move	k1, ra		#save ra
	dla	a0, v380_msg
	bal	stringserial
	nop
	b	exc_common

	.align	8			/* bfc00400 */
	move	k1, ra		#save ra
	dla	a0, v400_msg
	bal	stringserial
	nop

#if 1
	b	exc_common
	nop
#endif
	/* Debug exception */
	.align  7           /* bfc00480 */
	move	k1, ra		#save ra
	dla	a0, v480_msg
	bal	stringserial
	nop

	b	exc_common
	nop

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
    	b	1b
    	nop


#define SUPPORT_PWRUP_FROM_SELFREF
#ifdef  SUPPORT_PWRUP_FROM_SELFREF
#define GET_PWRUP_FROM_SREFRESH \
	move	a1, $0;
#endif


/*
 *  We get here from executing a bal to get the PC value of the current execute
 *  location into ra. Check to see if we run from ROM or if this is ramloaded.
 */
locate:
	move	s0, zero
	
	mfc0    t0, CP0_STATUS
        li     t1, 0x64000000|ST0_KX|ST0_SX|ST0_UX|ST0_BEV      # {cu3,cu2,cu1,cu0}<={0110, status_fr<=1,0xe0 to enable 64bit space
	or      t0, t0, t1
        mtc0    t0, CP0_STATUS
        mtc0    zero,CP0_CAUSE




	bal	beep_on
	nop
	li	a0, 0x800
1:
	addiu	a0, -1
	nop
	bnez	a0, 1b
	nop
	bal	beep_off
	nop


/*cfg pcie */
	li	t0, 0xbfe10540
	dli	t1, 0x14fff1002
	sd	t1, 0x0(t0)
	sd	t1, 0x20(t0)

	dli	t1, 0x14fff1102
	sd	t1, 0x0(t0)
	sd	t1, 0x20(t0)

	dli	t1, 0x14fff1202
	sd	t1, 0x0(t0)
	sd	t1, 0x20(t0)

	dli	t1, 0x14fff1302
	sd	t1, 0x0(t0)
	sd	t1, 0x20(t0)

/*pcie 0 port 0*/
	dli	t0, 0x900000fe0800000c
	li	t1, 0xfff9ffff
	lw	t2, 0x0(t0)
	and	t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700001c
	lw	t2, 0x0(t0)
	li	t1, (0x1 << 26)
	or	t2, t1
	sw	t2, 0x0(t0)

	dli	t0, 0x9000000000000000
	li	t1, 0x16000000
	or	t0, t0, t1

	li	t1, (0x7<<18)|(0x7<<2)
	not	t1, t1
	lw	t2, 0x54(t0)
	and	t2, t2, t1
	sw	t2, 0x54(t0)

	lw	t2, 0x58(t0)
	and	t2, t2, t1
	sw	t2, 0x58(t0)

	dli	t1, 0xff204f
	sw	t1, 0x0(t0)

/*pcie 0 port 1*/
	dli	t0, 0x900000fe0800080c
	li	t1, 0xfff9ffff
	lw	t2, 0x0(t0)
	and	t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700081c
	lw	t2, 0x0(t0)
	li	t1, (0x1 << 26)
	or	t2, t1
	sw	t2, 0x0(t0)

	dli	t0, 0x9000000000000000
	li	t1, 0x16001000
	or	t0, t0, t1

	li	t1, (0x7<<18)|(0x7<<2)
	not	t1, t1
	lw	t2, 0x54(t0)
	and	t2, t2, t1
	sw	t2, 0x54(t0)

	lw	t2, 0x58(t0)
	and	t2, t2, t1
	sw	t2, 0x58(t0)

	dli	t1, 0xff204f
	sw	t1, 0x0(t0)

/*enable pcie/dc/gmac/sata/usb/hda device cache*/
	li	t0, 0xbfe10110
	lw	t1, 0x0(t0)
	li	t2, 0x12fe8
	or	t1, t2
	sw	t1, 0x0(t0)

#if 1
	/*cfg pins to main, uart2 use GPIO 60/61*/
	li	t0, 0xbfe104ac
	li	t1, 0
	li	t2, 0x770000

#1:
	sw	t2, 0x0(t0)
#	addiu	t0, 4
#	addiu	t1, 1
#	bltu	t1, 20, 1b
#	nop
#endif

	bal	initserial
	nop

	PRINTSTR("\r\ninitserial good ^_^...\r\n")
	nop

##define VOLTIGE_CTRL
#ifdef VOLTIGE_CTRL
	move	s1, zero
	TTYDBG("\r\nnode 0 N Voltage  write :\r\n")
	bal	v_n_ctrl
	nop
	TTYDBG("\r\nnode 0 N Voltage  read :\r\n")
	bal	v_n_ctrl_read
	nop
#endif
#if 1
#include "clksetting_1a500.S"

	//clk chaneged need this code
	bal	initserial_later
	nop
#endif

#if 1
    //sata ctrl soft reset 
    li      t0, 0xbfe10108
    lw      t1, 0x0(t0)
    li      t2, (1 << 11)
    or      t1, t1, t2
    sw      t1, 0x0(t0)
    //init SATA
    //use t0 as global variable
    li      t0, 0xbfe10570
    //configure sata phy parameter
    li      t1, 0x30c31cf9
    sw      t1, 0x4(t0)
    sw      t1, 0xc(t0)
    li      t1, 0xf300040f
    sw      t1, 0x0(t0)
    sw      t1, 0x8(t0)

#if 0
    //power down phy
    lw      t1, 0x4(t0)
    li      t2, (1 << 31)
    or      t1, t1, t2
    sw      t1, 0x4(t0)
#endif
    //assert phy reset
    lw      t1, 0x0(t0)
    li      t2, (0x3 << 2)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x0(t0)
#if 1
    //switch refclk
    lw      t1, 0x0(t0)
    li      t2, (1 << 1)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x0(t0)
#endif
#if 0
    //switch refclk
    lw      t1, 0x0(t0)
    li      t2, (1 << 1)
    or     t1, t1, t2
    sw      t1, 0x0(t0)
#endif

    //delay a while
    li      t1, 0x1000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop

#if 1
    //power up phy
    lw      t1, 0x4(t0)
    li      t2, (1 << 31)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x4(t0)
#endif
    //deassert phy reset
    lw      t1, 0x0(t0)
    li      t2, (0x3 << 2)
    or      t1, t1, t2
    sw      t1, 0x0(t0)

    //delay a while
    li      t1, 0x1000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop

    TTYDBG("SATA0 enabled\r\n")

#if 0
    //power down phy
    lw      t1, 0xc(t0)
    li      t2, (1 << 31)
    or      t1, t1, t2
    sw      t1, 0xc(t0)
#endif

    //assert phy reset
    lw      t1, 0x8(t0)
    li      t2, (0x3 << 2)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x8(t0)

#if 1
    //switch refclk
    lw      t1, 0x8(t0)
    li      t2, (1 << 1)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x8(t0)
#endif
#if 0
    //switch refclk
    lw      t1, 0x0(t0)
    li      t2, (1 << 1)
    or      t1, t1, t2
    sw      t1, 0x0(t0)
#endif

    //delay a while
    li      t1, 0x1000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop

#if 1
    //power up phy
    lw      t1, 0xc(t0)
    li      t2, (1 << 31)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0xc(t0)
#endif

    //deassert phy reset
    lw      t1, 0x8(t0)
    li      t2, (0x3 << 2)
    or      t1, t1, t2
    sw      t1, 0x8(t0)

    //delay a while
    li      t1, 0x1000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop
 
   TTYDBG("SATA1 enabled\r\n")
    //deassert sata ctrl soft reset 
    li      t0, 0xbfe10108
    lw      t1, 0x0(t0)
    li      t2, (1 << 11)
    not	    t2, t2
    and      t1, t1, t2
    sw      t1, 0x0(t0)
#endif

start_now:

	PRINTSTR("\r\nPMON2000 MIPS Initializing. Standby...\r\n")

	bnez	s0, 1f
	nop
	li	a0, 128
	la	v0, initmips
	jr	v0
	nop

1:
	mfc0	a0, COP_0_CONFIG		/* enable kseg0 cachability */
	ori	a0, a0, 0x3		// ENABLE
	mtc0	a0, COP_0_CONFIG

#include "pcitlb.S" /* map 0x4000000-0x7fffffff to 0xc0000000 */


#ifdef USEPCI
	li	t0, 0xbfe10430
	lw	a2, 0x0(t0)
	// pcie0 and pcie1
	lui	t1, 0x3
#endif

#if 1	//DDR
//##########################################
//DDR config start
//cxk
#include "ddr_dir/lsmc_ddr_param_define.h"
#include "ddr_dir/ddr_config_define.h"
//#define DDR_DLL_BYPASS
//#define DISABLE_HARD_LEVELING
#define DISABLE_DIMM_ECC
#define PRINT_MSG
#ifdef  ARB_LEVEL
#define AUTO_ARB_LEVEL
#endif
#ifdef  AUTO_ARB_LEVEL
#define CHECK_ARB_LEVEL_FREQ
#ifdef  AUTO_DDR_CONFIG
#define CHECK_ARB_LEVEL_DIMM
#endif
//#define DEBUG_AUTO_ARB_LEVEL
#endif
//#define  DISABLE_DDR_A15
//#define DEBUG_DDR
//#define DEBUG_DDR_PARAM
//#define PRINT_DDR_LEVELING
//#define DLL_DELAY_LOOP
//#define NO_AUTO_TRFC	//adjust TRFC param manually if defined
	TTYDBG("\r\nStart Init Memory, wait a while......\r\n")
####################################
	move	msize, $0
	move	s3, $0
//!!!!important--s1 must be correctly set

	TTYDBG("NODE 0 MEMORY CONFIG BEGIN\r\n")
#ifdef AUTO_DDR_CONFIG
	dli	s1, 0xff100004  //set use MC1 or MC0 or MC1/0 and give All device id
#else
	//dli	s1, 0xc2e30400c2e30404
	//dli     s1, 0xf0a31004		//8G
	dli     s1, 0xf8a10404		//2G
//	dli     s1, 0xf8a18404		//2G
#endif
#include "ddr_dir/loongson3_ddr2_config.S"

	/*judge the node0 whether have memory*/
	and	a0, msize, 0xff
//	beqz	a0, beep_on
//	nop

#if 0	//xwr todo: cfg what?
//close default internal mapping in ddr controller
	li	t0, 0xbfe10424
	lb	a0, 0x1(t0)
	and	a0, a0, 0xfd
	sb	a0, 0x1(t0)
	sync

	li	t0, 0xbfe10420
	ld	a0, 0x0(t0)
	move	t6, a0
	dsrl	a0, t6, 32
	bal	hexserial
	nop
	move	a0, t6
	bal	hexserial
	nop
	PRINTSTR("\r\n")
#endif


/* test memory */
	li	t0, 0xa0000000
	dli	a0, 0x5555555555555555
	sd	a0, 0x0(t0)
	dli	a0, 0xaaaaaaaaaaaaaaaa
	sd	a0, 0x8(t0)
	dli	a0, 0x3333333333333333
	sd	a0, 0x10(t0)
	dli	a0, 0xcccccccccccccccc
	sd	a0, 0x18(t0)
	dli	a0, 0x7777777777777777
	sd	a0, 0x20(t0)
	dli	a0, 0x8888888888888888
	sd	a0, 0x28(t0)
	dli	a0, 0x1111111111111111
	sd	a0, 0x30(t0)
	dli	a0, 0xeeeeeeeeeeeeeeee
	sd	a0, 0x38(t0)


	PRINTSTR("The uncache data is:\r\n")
	dli	t1, 8
	dli	t5, 0x9000000000000000
1:
	ld	t6, 0x0(t5)
	move	a0, t5
	and	a0, a0, 0xfff
	bal	hexserial
	nop
	PRINTSTR(":  ")
	dsrl	a0, t6, 32
	bal	hexserial
	nop
	move	a0, t6
	bal	hexserial
	nop
	PRINTSTR("\r\n")

	daddiu	t1, t1, -1
	daddiu	t5, t5, 8
	bnez	t1, 1b
	nop

	PRINTSTR("The cached  data is:\r\n")
	dli	t1, 8
	dli	t5, 0x9800000000000000
1:
	ld	t6, 0x0(t5)
	move	a0, t5
	and	a0, a0, 0xfff
	bal	hexserial
	nop
	PRINTSTR(":  ")
	dsrl	a0, t6, 32
	bal	hexserial
	nop
	move	a0, t6
	bal	hexserial
	nop
	PRINTSTR("\r\n")

	daddiu	t1, t1, -1
	daddiu	t5, t5, 8
	bnez	t1, 1b
	nop

##########################################
#ifdef  DEBUG_DDR
	PRINTSTR("\r\nDo test?(0xf: skip): ")
	bal	inputaddress
	nop
	and	v0, v0, 0xf
	dli	a1, 0x1
	bgt	v0, a1, 2f
	nop
	dli	s1, 0x0010000080000000
	PRINTSTR("\r\ndefault s1 = 0x");
	dsrl	a0, s1, 32
	bal	hexserial
	nop
	PRINTSTR("__")
	move	a0, s1
	bal	hexserial
	nop
	PRINTSTR("\r\nChange test param s1(0: skip)?: ")
	bal	inputaddress
	nop
	beqz	v0, 1f
	nop
	move	s1, v0
1:
	dli	t1, 0x0010
	bal	test_mem
	nop
	move	t1, v0
	PRINTSTR("\r\n")
	dsrl	a0, t1, 32
	bal	hexserial
	nop
	move	a0, t1
	bal	hexserial
	nop
	beqz	t1, 2f
	nop
	PRINTSTR("  Error found!!\r\n")
2:
#endif

#ifdef AUTO_ARB_LEVEL
#include "ddr_dir/store_auto_arb_level_info.S"
#endif

#endif	/* DDR */

#include "machine/newtest/newdebug.S"

bootnow:
	move	k1,msize
	move 	ra, fp
	jr	ra
	nop

#if __mips64
#define MTC0 dmtc0
#else
#define MTC0 mtc0
#endif

#ifdef VOLTIGE_CTRL
#include "ls1a500_vctrl.S"
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
 * Simple character printing routine used before full initialization
 */
/* baud rate definitions, matching include/termios.h */
#define B0	0
#define B50	50
#define B75	75
#define B110	110
#define B134	134
#define B150	150
#define B200	200
#define B300	300
#define B600	600
#define B1200	1200
#define B1800	1800
#define B2400	2400
#define B4800	4800
#define B9600	9600
#define B19200	19200
#define B38400	38400
#define B57600	57600
#define B115200	115200

#define UART_REF_CLK	100000000
#define UART_DIV_HI	(((UART_REF_CLK + (115200*8)) / (115200*16)) >> 8)
#define UART_DIV_LO	(((UART_REF_CLK + (115200*8)) / (115200*16)) & 0xff)
LEAF(initserial)
	.set noat
	move	AT, ra

        la      a0, CONFIG_SYS_NS16550_COM1
	li	t1, CFCR_DLAB
	sb	t1, NSREG(NS16550_CFCR)(a0)

	li	t1, UART_DIV_HI
	sb	t1, 1(a0)
	li	t1, UART_DIV_LO
	sb	t1, 0(a0)
	li	t1, 3	#CFCR_8BITS
	sb	t1, 3(a0)	#NSREG(NS16550_CFCR)(v0)

	li	t1, 71
	sb	t1, 2(a0)

	j	ra
	nop
	.set at
END(initserial)
#if 1
#define UART_REF_CLK_PLL	(GMAC_FREQ * GMAC_DIV / SB_DIV * 1000000)	//SB PLL
#define UART_DIV_HI_2		(((UART_REF_CLK_PLL + (115200*8)) / (115200*16)) >> 8)
#define UART_DIV_LO_2		(((UART_REF_CLK_PLL + (115200*8)) / (115200*16)) & 0xff)
LEAF(initserial_later)
	.set noat
	move	AT,ra

        la      a0, CONFIG_SYS_NS16550_COM1
	li	t1, CFCR_DLAB
	sb	t1, NSREG(NS16550_CFCR)(a0)

	li	t1, UART_DIV_HI_2
	sb	t1, 1(a0)
	li	t1, UART_DIV_LO_2
	sb	t1, 0(a0)
1:
	lb	a1, 0(a0)
	bne	a1, t1, 1b
	nop

	li	t1, 3	#CFCR_8BITS
	sb	t1, 3(a0)	#NSREG(NS16550_CFCR)(v0)

	li	t1, 71
	sb	t1, 2(a0)

	j	ra
	nop
	.set at
END(initserial_later)
#endif
LEAF(tgt_putchar)
	.set noat
	move	AT, ra
	la  v0, CONFIG_SYS_NS16550_COM1
	bal	1f
	nop
	jr	AT
	nop
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v1, LSR_TXRDY
	beqz	v1, 1b
	nop

	sb	a0, NSREG(NS16550_DATA)(v0)
	j	ra
	nop
	.set at
END(tgt_putchar)

LEAF(tgt_testchar)
	.set noat
	move	AT, ra
        la      v0, CONFIG_SYS_NS16550_COM1
	bal	1f
	nop
	jr	AT
	nop
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v0, v1, LSR_RXRDY
	jr	ra
	nop
	.set	at
END(tgt_testchar)

LEAF(tgt_getchar)
	.set	noat
	move	AT, ra
	la	v0, CONFIG_SYS_NS16550_COM1
	bal	1f
	nop
	jr	AT
	nop
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v1, LSR_RXRDY
	beqz	v1, 1b
	nop
	lb	v0, NSREG(NS16550_DATA)(v0)
	jr	ra
	nop
	.set at
END(tgt_getchar)

LEAF(stringserial)
	move	a2, ra
	move	a1, a0
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
	move	a2, ra
	move	a1, a0
	li	a3, 7
1:
	rol	a0, a1, 4
	move	a1, a0
	and	a0, 0xf
	la	v0, hexchar
	addu	v0, a0
	bal	tgt_putchar
	lbu	a0, 0(v0)

	bnez	a3, 1b
	addu	a3, -1

	j	a2
	nop
END(hexserial)

LEAF(hexserial64)
	move	t7, ra
	move	t6, a0
	dsrl	a0, 32
	bal	hexserial
	nop
	move	a0, t6
	bal	hexserial
	nop
	jr	t7
	nop
END(hexserial64)

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
hexchar:
	.ascii	"0123456789abcdef"

	.text
	.align	2

#define Index_Store_Tag_I		0x08
#define Index_Store_Tag_D		0x09
#define Index_Invalidate_I		0x00
#define Index_Writeback_Inv_D		0x01
#define Index_Writeback_Inv_S		0x01
#define Index_Store_Tag_S		0x0b

LEAF(nullfunction)
	jr	ra
	nop
END(nullfunction)

LEAF(tlb_init)
	mtc0	$0, CP0_WIRED
	mtc0	$0, CP0_PAGEMASK
tlb_flush_all:
	lui	a0, 0x8000
	addiu	a1, $0, 64
	#a0=KSEG0,a1 = tlbsize, v0, v1, a3 used as local registers
	mtc0	$0, CP0_ENTRYLO0
	mtc0	$0, CP0_ENTRYLO1
	mfc0	v0, CP0_WIRED
	addu	v1, $0, a0
1:	sltu	a3, v0, a1
	beq	a3, $0, 1f
	nop
	mtc0	v1, CP0_ENTRYHI
	mtc0	v0, CP0_INDEX
	tlbwi
	addiu	v1, v1, 0x2000
	beq	$0, $0, 1b
	addiu	v0, v0, 1
1:
	/* tlb_init finish */
	tlbp
END(tlb_init)

LEAF(get_userenv)
	jr	ra
	nop
END(get_userenv)

LEAF(beep_off)
	/* enable gpio46 */
	li	t1, 0xbfe104a4
	lw	t0, 0(t1)
	and	t0, 0xf8ffffff
	sw	t0, 0(t1)

	/* enable gpio46 output */
	li	t1, 0xbfe10434
	lw	t0, 0(t1)
	and	t0, ~(0x1 << 14)
	sw	t0, 0(t1)

	/* set gpio46 high */
	li	t1, 0xbfe10444
	lw	t0, 0(t1)
	or	t0, (0x1 << 14)
	sw	t0, 0(t1)

	jr	ra
	nop
END(beep_off)

LEAF(beep_on)
	/* enable gpio46 */
	li	t1, 0xbfe104a4
	lw	t0, 0(t1)
	and	t0, 0xf8ffffff
	sw	t0, 0(t1)

	/* enable gpio46 output */
	li	t1, 0xbfe10434
	lw	t0, 0(t1)
	and	t0, ~(0x1 << 14)
	sw	t0, 0(t1)

	/* set gpio46 low */
	li	t1, 0xbfe10444
	lw	t0, 0(t1)
	and	t0, ~(0x1 << 14)
	sw	t0, 0(t1)

	jr	ra
	nop
END(beep_on)

#if 1 //used i2c read ddr
#include "i2c.S"
#endif

#######################################
#ifdef  AUTO_DDR_CONFIG
//#include "ddr_dir/detect_node_dimm.S"
#include "ddr_dir/detect_node_dimm_all.S"
#endif

#include "ddr_dir/ls3A8_ddr_config.S"
#ifdef DDR3_DIMM
#include "ddr_dir/loongson3C_ddr3_leveling.S"
#endif
#ifdef ARB_LEVEL
#include "ddr_dir/ARB_level_new.S"
#endif
#ifdef  DEBUG_DDR
#include "ddr_dir/Test_Mem.S"
#endif
#######################################

LEAF(watchdog_open)
	//disable watch DOG.
#if 1	//use internal watch dog
	li	t1, 0xbff6c000
	li	t2, 0x2fffffff
	sw	t2, 0x38(t1)

	lw	t2, 0x30(t1)
	li	t3, 0x2
	or	t2, t3
	sw	t2, 0x30(t1)	//enable watchdog

	li	t2, 0x1
	sw	t2, 0x34(t1)	//set watchdog time
#else	//GPIO0:WDI, GPIO1=0:close, GPIO1=1:1~3s
	/* multi cfg */
	li	t1, LS1A500_GPIO_MULTI_CFG
	lw	t2, 0(t1)
	li	t3, 0xff
	not	t3, t3
	and	t2, t2, t3
	sw	t2, 0(t1)

	/*gpio1 output high*/
	li	t1, LS1A500_GPIO_00_63_DIR
	lw	t2, 0(t1)
	li	t3, 0x2
	not	t3, t3
	and	t2, t2, t3
	sw	t2, 0(t1)

	lw	t2, 0x10(t1)
	ori	t2, t2, 2
	sw	t2, 0x10(t1)
	nop
#endif
	jr		ra
	nop
END(watchdog_open)

LEAF(watchdog_close)
	//disable watch DOG.
#if 1	//use internal watch dog
	li	t1, 0xbff6c030
	lw	t2, 0(t1)
	li	t3, 0x2
	not	t3, t3
	and	t2, t3
	sw	t2, 0(t1)
#else	//GPIO0:WDI, GPIO1=0:close, GPIO1=1:1~3s
	/* multi cfg */
	li	t1, LS1A500_GPIO_MULTI_CFG
	lw	t2, 0(t1)
	li	t3, 0xff
	not	t3, t3
	and	t2, t2, t3
	sw	t2, 0(t1)

	/*gpio1 output zero*/
	li	t1, LS1A500_GPIO_00_63_DIR
	lw	t2, 0(t1)
	li	t3, 0x2
	not	t3, t3
	and	t2, t2, t3
	sw	t2, 0(t1)

	lw	t2, 0x10(t1)
	and	t2, t2, t3
	sw	t2, 0x10(t1)
	nop
#endif
	jr		ra
	nop
END(watchdog_close)

	.text
	.global nvram_offs
	.align 12
nvram_offs:
	.dword 0x0
	.align 12
#######################################

	.rdata
	.global ddr2_reg_data
	.global ddr3_reg_data

	.align 5
#include "loongson_mc2_param.S"
	.global poweroff_kernel
	.ent	poweroff_kernel
poweroff_kernel:
	.set	noreorder
	mfc0	t0, CP0_STATUS
	li	t1, 0xfffffffe
	and	t0, t1, t0
	mtc0	t0, CP0_STATUS
	j	tgt_poweroff
	lui	sp, 0x8ffc
	.end	poweroff_kernel

	.global reboot_kernel
	.ent	reboot_kernel
reboot_kernel:
	mfc0	t0, CP0_STATUS
	li	t1, 0xfffffffe
	and	t0, t1, t0
	mtc0	t0, CP0_STATUS
	j	tgt_reboot
	lui	sp, 0x8ffc
	.end	reboot_kernel
