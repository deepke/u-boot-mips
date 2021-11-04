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
#include "ls2k.h"
#include "../mm/2H_chip_define.h"

#define TTYDEBUG
#ifdef TTYDEBUG
#define	TTYDBG(x) \
	.pushsection .rodata;98: .asciz x; .popsection; dla a0, 98b; bal stringserial; nop
#else
#define TTYDBG(x)
#endif
#define HAVE_TARGET_GETCHAR
#define	PRINTSTR(x) \
	.pushsection .rodata;98: .asciz x; .popsection; dla a0, 98b; bal stringserial; nop

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
#ifndef PCIE_LANE_FLIP
#define PCIE_LANE_FLIP 0x300 /*pcie1 port0 rx, tx lane revert*/
#endif

	.set	noreorder
	.set 	mips3
	.globl	__main
	.globl	lowlevel_init
lowlevel_init:
	move fp, ra
	/*set all spi cs to 1, default input*/
	li v0,0xbfff0225
	li v1,0xff
	sb v1,(v0)
#if 1 //fix the hardware poweroff error.

	bal	ls2k_version
	nop
	bnez	v0,2f
	nop

	.set    mips32
	mfc0    t0, $15, 1      #EBASE
	.set    mips3
	andi    t0, t0, 0x3ff
	bnez    t0, 2f
	nop

	lui	t0, 0xba00
	lui	t1, 0x1fe0
	sw	t1, 0x1010(t0) /* config bar for APB */
	lw	t2, 0x1004(t0)
	ori	t2, t2, 0x2
	sw	t2, 0x1004(t0)

	li t0,0xbfe0700c
	lw t1,0x0(t0)
	and t2,t1,(1 << 11)
	beqz  t2,2f
	nop

	li t0,0xbfe0700c
	lw t1, 0x0(t0)
	sw t1,0x0(t0)
	li t2,0x3c00
	li t0,0xbfe07014
	sw t2,0x0(t0)
2:
#endif
	bal watchdog_close
	nop
#ifdef GPIO_PINS
	li	v0, 0xbfe10420
	ld	v1, (v0)
	dli	a0, LS2K_SELGPIO
	or	v1, a0
	xor	v1, a0
	sd	v1, (v0)
#endif

#ifdef GPIO_OUT_PINS
	li v0, 0xbfe10500
	ld v1, (v0)
	dli a0, GPIO_OUT_PINS
	or v1, a0
	xor v1, a0
	sd v1, (v0)
#endif

#ifdef GPIO_OUT_CLR
	li v0, 0xbfe10500
	ld v1, 0x10(v0)
	dli a0, GPIO_OUT_CLR
	or v1, a0
	xor v1, a0
	sd v1, 0x10(v0)
#endif

#ifdef GPIO_OUT_SET
	li v0, 0xbfe10500
	ld v1, 0x10(v0)
	dli a0, GPIO_OUT_SET
	or v1, a0
	sd v1, 0x10(v0)
#endif

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

	/* spi speedup */
	li  t0, 0xbfff0220
	li  t1, 0x47
	sb  t1, 0x4(t0)

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

	b	exc_common
	nop

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
    move    a1, $0;
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


#ifdef  RESERVED_COREMASK
        .set    mips64
        mfc0    t0, $15, 1      #EBASE
	.set	mips0
        andi    t0, t0, 0x3ff

	li	t1, 0x1
	sllv	t1, t1, t0
	and	t3, t1, RESERVED_COREMASK
	bnez	t3, wait_to_be_killed
	nop

	li	v0, 0xbfe104d4
	lw	t2, 0x0(v0)
	or	t2, 3
	xori	t2, RESERVED_COREMASK
	sw	t2, 0x0(v0)
	
	b	cp0_main
	nop

wait_to_be_killed:

	b	wait_to_be_killed
	nop
#endif


cp0_main:
        .set    mips32
        mfc0    t0, $15, 1      #EBASE
        .set    mips3
        andi    t0, t0, 0x3ff
        bnez    t0, wait_for_smp
        nop

	bal	beep_on
	nop

	li	a0, 0x8000
1:
	addiu	a0, -1
	nop
	bnez	a0, 1b
	nop
	bal	beep_off//mtf
	nop

	lui	t0, 0xba00
	lui	t1, 0x1fe0
	sw	t1, 0x1010(t0) /* config bar for APB */
	lw	t2, 0x1004(t0)
	ori	t2, t2, 0x2
	sw	t2, 0x1004(t0)

#ifdef LOWPOWER
	li v0, 0xbfc00000 + NVRAM_OFFS + SHUTDEV_OFFS
	lbu t1, 3(v0)
	xor t1, 0x5a
	bnez t1, 1f
	nop
	lbu t1, 0(v0)
	and t1, 0xc
	xor t1, 0xc
	beqz t1, pcie_cfg_done
	nop
1:
#endif
#if 1	//xwr pcie signal test
        li      t0, 0xbfe10000

	li	t1, 0xc2492331
	sw	t1, 0x580(t0)
	sw	t1, 0x5a0(t0)

	li	t1, 0xff3ff0a8
	sw	t1, 0x584(t0)
	sw	t1, 0x5a4(t0)

	li	t1, 0x27fff
	sw	t1, 0x588(t0)
	sw	t1, 0x5a8(t0)

#endif
/* mtf add for cfg pcie */
	
        li      t0, 0xbfe10590
        dli     t1, 0x14fff1002
        sd      t1, 0x0(t0)
        sd      t1, 0x20(t0)

        dli     t1, 0x14fff1102
        sd      t1, 0x0(t0)
        sd      t1, 0x20(t0)
	
        dli     t1, 0x14fff1202
        sd      t1, 0x0(t0)
        sd      t1, 0x20(t0)
	
        dli     t1, 0x14fff1302
        sd      t1, 0x0(t0)
        sd      t1, 0x20(t0)
	
	li	t0, 0xbfe10430
	lw	t1, 0x0(t0)
	or	t1, t1, 0x30000	//pcie enable
	sw	t1, 0x0(t0)

//pcie1 port0
	dli	t0, 0x900000fe0800680c
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700681c
    lw  t2, 0x0(t0)
    li  t1, (0x1 << 26)
    or  t2, t1
    sw  t2, 0x0(t0)


	dli	t0, 0x900000fe00006800
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 0x10
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x10000000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x10000000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)

	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*4)) & 3)
        sw      t1, 0x0(t0)
  
//pcie1 port1
	dli	t0, 0x900000fe0800700c
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700701c
    lw  t2, 0x0(t0)
    li  t1, (0x1 << 26)
    or  t2, t1
    sw  t2, 0x0(t0)

	dli	t0, 0x900000fe00007000
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 0x20
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x10100000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x10100000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)

	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*5)) & 3)
        sw      t1, 0x0(t0)
  
//pcie0 port0
	dli	t0, 0x900000fe0800480c	//other pcie controller
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700481c
    lw  t2, 0x0(t0)
    li  t1, (0x1 << 26)
    or  t2, t1
    sw  t2, 0x0(t0)

	dli	t0, 0x900000fe00004800
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 1
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x11000000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x11000000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)

	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*0)) & 3)
        sw      t1, 0x0(t0)
  
//pcie0 port1
	dli	t0, 0x900000fe0800500c	//other pcie controller
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe0700501c
    lw  t2, 0x0(t0)
    li  t1, (0x1 << 26)
    or  t2, t1
    sw  t2, 0x0(t0)

	dli	t0, 0x900000fe00005000
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 2
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x11100000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x11100000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)

	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*1)) & 3)
        sw      t1, 0x0(t0)
  
//pcie0 port2
	dli	t0, 0x900000fe0800580c	//other pcie controller
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe00005800
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 4
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x11200000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x11200000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)


	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*2)) & 3) 
        sw      t1, 0x0(t0)
  
//pcie0 port3
	dli	t0, 0x900000fe0800600c	//other pcie controller
	li	t1, 0xfff9ffff
        lw      t2, 0x0(t0)
        and     t1, t1, t2
	or	t1, 0x20000
	sw	t1, 0x0(t0)

	dli	t0, 0x900000fe00006000
#if defined(FORCE_PCIE_GEN1) && FORCE_PCIE_GEN1 & 8
//pcie capability link control 2, target link speed(bit 3:0)
    lw 	    t1, 0xa0(t0)
    or	    t1, 3
    xor     t1, 2
    sw      t1, 0xa0(t0)
#endif
	li	t1, 0x11300000
	sw	t1, 0x10(t0)

	dli	t0, 0x9000000000000000
	li      t1, 0x11300000
	or	t0, t0, t1

    li  t1, (0x7<<18)|(0x7<<2)
    not t1, t1
    lw  t2, 0x54(t0)
    and t2, t2, t1
    sw  t2, 0x54(t0)

    lw  t2, 0x58(t0)
    and t2, t2, t1
    sw  t2, 0x58(t0)


	dli	t1, 0xff204c | ((PCIE_LANE_FLIP>>(2*3)) & 3)
        sw      t1, 0x0(t0)

pcie_cfg_done:
	li	a0,(100000000/ 16)/CONFIG_BAUDRATE
	bal	initserial
	nop

	PRINTSTR("\r\ninitserial good ^_^...\r\n")
	nop

	bal	ls2k_version
	nop
	bnez	v0,2f
	nop
#if !defined(GPIO_OUT_PINS) || ((GPIO_OUT_PINS &(1<<34)) && (!defined(GPIO_OUT_SET) || (GPIO_OUT_SET & (1<<34))))
	//gmac1 phy reset
	li t1,0xbfe10500
	li t2,(1 << 2) //34 - 2
	not t2,t2
	lw t3,0x4(t1)
	and t2,t3
	sw t2,0x4(t1)

	li t2,(1 << 2)//34 -2
	lw t3,0x14(t1)
	not t2,t2
	and t2,t3 
	sw t2,0x14(t1)
#endif
2:

#include "loongson3_clksetting.S"

	li	a0,(125000000/ 16)/CONFIG_BAUDRATE
	bal 	initserial
	nop

	bal	ls2k_version
	nop
	bnez	v0,2f
	nop
#if !defined(GPIO_OUT_PINS) || ((GPIO_OUT_PINS &(1<<34)) && (!defined(GPIO_OUT_SET) || (GPIO_OUT_SET & (1<<34))))
	//gmac1 phy reset
	li t1,0xbfe10500
	li t2,(1 << 2) //34 - 2
	lw t3,0x14(t1)
	or t2,t3 
	sw t2,0x14(t1)
#endif
2:

#if 1 /* Config SATA : use internel clock */
	li	    t0, 0xbfe10000

    li      t1, 0x30c31cf9
    sw      t1, 0x454(t0)
    li      t1, 0xf300040f
    sw      t1, 0x450(t0)

#if 1
	PRINTSTR("\r\nUSE internel SATA ref clock\r\n")
	li	t1, 0xbfe10450
	ld	a0, 0x0(t1)
	li	a1, 0x2
	not	a1, a1
	and	a0, a0, a1
	sd	a0, 0x0(t1)
	sync

	ld	a0, 0x0(t1)
	li	a1, 0x4
    or  a0, a1
	sd	a0, 0x0(t1)
	sync

	ld	a0, 0x0(t1)
	li	a1, 0x8
    or  a0, a1
	sd	a0, 0x0(t1)
	sync

	ld	a0, 0x0(t1)
	bal	hexserial64
	nop
#endif

#if 0
    li      t0, 0xbfe10000

    //assert p0 reset
    lw      t1, 0x450(t0)
    li      t2, (1 << 3)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x450(t0)

    //assert phy reset
    lw      t1, 0x450(t0)
    li      t2, (1 << 2)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x450(t0)

    //power down phy
    lw      t1, 0x454(t0)
    li      t2, (1 << 31)
    or      t1, t1, t2
    sw      t1, 0x454(t0)

    //switch refclk
    lw      t1, 0x450(t0)
    li      t2, (1 << 1)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x450(t0)

    //delay a while
    li      t1, 0x1000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop


    //power up phy
    lw      t1, 0x454(t0)
    li      t2, (1 << 31)
    not     t2, t2
    and     t1, t1, t2
    sw      t1, 0x454(t0)

    //deassert phy reset
    lw      t1, 0x450(t0)
    li      t2, (1 << 2)
    or      t1, t1, t2
    sw      t1, 0x450(t0)
	
    //deassert p0 reset
    lw      t1, 0x450(t0)
    li      t2, (1 << 3)
    or      t1, t1, t2
    sw      t1, 0x450(t0)

    //delay a while
    li      t1, 0x10000
1:
    subu    t1, t1, 1
    bnez    t1, 1b
    nop
#endif
#endif

#if 0 /* Config SATA TX signal*/
        li      t0, 0xbfe10458
        dli     t1, 0x1403f1002
        sd      t1, 0x0(t0)
#endif


#if 1 // Fix the Gmac0  multi-func to enable Gmac1
	li	t0, 0xbfe13800
	dli	a0, 0xffffff0000ffffff
	sd	a0, 0x08(t0)

	li	t0, 0xba001800
	li	a0, 0x0080ff08
	sw	a0, 0x0c(t0)
#endif

#if 1 // Set the invalid BAR to read only
	li	t0, 0xbfe13800
	dli	a0, 0xff00ff0000fffff0
	sd	a0, 0x00(t0)
	sd	a0, 0x08(t0)
	sd	a0, 0x10(t0)
	sd	a0, 0x18(t0)
	sd	a0, 0x20(t0)
	sd	a0, 0x28(t0)
	sd	a0, 0x30(t0)
	sd	a0, 0x38(t0)
	sd	a0, 0x40(t0)
	sd	a0, 0x48(t0)
	sd	a0, 0x50(t0)
#endif

start_now:

        mfc0	a0, CP0_CONFIG		/* enable kseg0 cachability */
        ori     a0, a0, 0x3           // ENABLE
        mtc0   a0, CP0_CONFIG

	li	t0, 0xbfe10430
	lw 	a2, 0x0(t0)
	// pcie0 and pcie1
	lui	t1, 0x3
	// enable dvo0 and dvo1 pin output 
    	ori     t1, t1, 0x12
	or	a2, a2, t1
   	sw      a2, 0x0(t0)
	

	li	t0, 0xbfe10420
	//sdio
	lw	t2, 0x0(t0)
	lui	t1, 0x10
	//enable pwm0, pwm1, i2c0, i2c1, nand, sata, i2s, gmac1
	//no hda, no ac97
	ori	t1, t1, 0x3f48
	or	t2, t2, t1
	sw	t2, 0x0(t0)
	

//##########################################
//DDR config start
//cxk
#include "../mm/lsmc_ddr_param_define.h"
#include "../mm/ddr_config_define.h"
//#define DDR_DLL_BYPASS
#define DISABLE_DIMM_ECC
#define PRINT_MSG
#ifdef  ARB_LEVEL
#define AUTO_ARB_LEVEL
#endif
#ifdef  AUTO_ARB_LEVEL
//#define CHECK_ARB_LEVEL_FREQ
#ifdef  AUTO_DDR_CONFIG
#define CHECK_ARB_LEVEL_DIMM
#endif
//#define DEBUG_AUTO_ARB_LEVEL
#endif
#define  DISABLE_DDR_A15
//#define DEBUG_DDR
//#define DEBUG_DDR_PARAM
//#define PRINT_DDR_LEVELING

    TTYDBG("\r\nStart Init Memory, wait a while......\r\n")
####################################
    move    msize, $0
    move    s3, $0
//!!!!important--s1 must be correctly set

    TTYDBG("NODE 0 MEMORY CONFIG BEGIN\r\n")
#ifdef  AUTO_DDR_CONFIG
    dli     s1, 0xff100004  //set use MC1 or MC0 or MC1/0 and give All device id
#elif defined(DDR_S1)
	dli     s1, DDR_S1
#else
	//dli     s1, 0xc2e30400c2e30404
	//dli     s1, 0xc1a10404
	//dli	s1, 0xc0a10400c0a10400
	//dli	s1, 0xc0a18404
	//dli	s1, 0xf0a31004
	//dli	s1, 0xc1a10404
    dli     s1, ( MC_SDRAM_TYPE_DDR3    /* sdram type: DDR3/DDR2 */ \
                | MC_DIMM_ECC_NO        /* dimm ECC: YES/NO */ \
                | MC_DIMM_BUF_REG_NO    /* dimm buffer register: YES/NO, for RDIMM use YES, all else use NO*/ \
                | MC_DIMM_WIDTH_64      /* memory data width: 64/32 */ \
                | MC_SDRAM_ROW_15       /* sdram row address number: 15~11 */ \
                | MC_SDRAM_COL_10       /* sdram column address number: 12~9 */ \
                | MC_SDRAM_BANK_8       /* sdram bank number: 8/4 */ \
                | MC_ADDR_MIRROR_YES    /* for standard DDR3 UDIMM, use YES, else use NO */ \
                | MC_SDRAM_WIDTH_X16     /* SDRAM device data width: 8/16 */ \
                | MC_USE_CS_0         /* the CS pins the sdram connected on(split by '_', from small to big) */ \
                | MC_MEMSIZE_(2)        /* MC memory size, unit: 512MB */ \
                | USE_MC_0)
#endif
#include "loongson3_ddr2_config.S"

	/*judge the node0 whether have memory*/
	and     a0, msize, 0xff
//	beqz    a0, beep_on
//	nop


//close default internal mapping in ddr controller
	li      t0, 0xbfe10424
	lb      a0, 0x1(t0)
	and     a0, a0, 0xfd
	sb      a0, 0x1(t0)
	sync

	li      t0, 0xbfe10420
	ld      a0, 0x0(t0)
	move    t6, a0
	dsrl    a0, t6, 32
	bal     hexserial
	nop
	move    a0, t6
	bal     hexserial
	nop
	PRINTSTR("\r\n")

/* test memory */
        li      t0, 0xa0000000
        dli     a0, 0x5555555555555555
        sd      a0, 0x0(t0)
        dli     a0, 0xaaaaaaaaaaaaaaaa
        sd      a0, 0x8(t0)
        dli     a0, 0x3333333333333333
        sd      a0, 0x10(t0)
        dli     a0, 0xcccccccccccccccc
        sd      a0, 0x18(t0)
        dli     a0, 0x7777777777777777
        sd      a0, 0x20(t0)
        dli     a0, 0x8888888888888888
        sd      a0, 0x28(t0)
        dli     a0, 0x1111111111111111
        sd      a0, 0x30(t0)
        dli     a0, 0xeeeeeeeeeeeeeeee
        sd      a0, 0x38(t0)


	PRINTSTR("The uncache data is:\r\n")
	dli     t1, 8
	dli     t5, 0x9000000000000000
1:
	ld      t6, 0x0(t5)
	move    a0, t5
	and     a0, a0, 0xfff
	bal     hexserial
	nop
	PRINTSTR(":  ")
	dsrl    a0, t6, 32
	bal     hexserial
	nop
	move    a0, t6
	bal     hexserial
	nop
	PRINTSTR("\r\n")

	daddiu  t1, t1, -1
	daddiu  t5, t5, 8
	bnez    t1, 1b
	nop

	PRINTSTR("The cached  data is:\r\n")
	dli     t1, 8
	dli     t5, 0x9800000000000000
1:
	ld      t6, 0x0(t5)
	move    a0, t5
	and     a0, a0, 0xfff
	bal     hexserial
	nop
	PRINTSTR(":  ")
	dsrl    a0, t6, 32
	bal     hexserial
	nop
	move    a0, t6
	bal     hexserial
	nop
	PRINTSTR("\r\n")

	daddiu  t1, t1, -1
	daddiu  t5, t5, 8
	bnez    t1, 1b
	nop

##########################################
#ifdef  DEBUG_DDR
	PRINTSTR("\r\nDo test?(0xf: skip): ")
	bal     inputaddress
	nop
	and     v0, v0, 0xf
	dli     a1, 0x1
	bgt     v0, a1, 2f
	nop
	dli     s1, 0x0010000080000000
	PRINTSTR("\r\ndefault s1 = 0x");
	dsrl    a0, s1, 32
	bal     hexserial
	nop
	PRINTSTR("__")
	move    a0, s1
	bal     hexserial
	nop
	PRINTSTR("\r\nChange test param s1(0: skip)?: ")
	bal     inputaddress
	nop
	beqz    v0, 1f
	nop
	move    s1, v0
1:
	dli     t1, 0x0010
	bal     test_mem
	nop
	move    t1, v0
	PRINTSTR("\r\n")
	dsrl    a0, t1, 32
	bal     hexserial
	nop
	move    a0, t1
	bal     hexserial
	nop
	beqz    t1, 2f
	nop
	PRINTSTR("  Error found!!\r\n")
	

2:
#endif

#ifdef  AUTO_ARB_LEVEL
#include "../mm/store_auto_arb_level_info.S"
#endif

#include "../mm/newtest/newdebug.S"

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
.global wait_for_smp;
.global wait_for_smp_call;
wait_for_smp:
	mfc0	t1, CP0_CONFIG
	ori	t1, t1, 0x3
	mtc0	t1, CP0_CONFIG
	la	t1, 1f
	addu	t1, s0
	li	v0, 0x9fffffff
	and	t1, v0
	jr	t1
	nop
1:
	li	t0, 0xbfe11120
	sd	zero, 0(t0)
	sd	t1, 8(t0)

wait_for_smp_call:
1:
	ld	t1, 0(t0)
	beqz	t1, 1b
	ld	sp, 8(t0)
	sd	zero,0(t0)
	ld	gp, 16(t0)

	jr	t1
	nop


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


LEAF(ls2k_version)
	.set    mips32
	
	mfc0    a0, CP0_PRID
	.set    mips3
	andi    a0, a0, 0xf
	li	a1, 0x3
	bne     a0, a1, 2f
	li	v0, 0
	li	v0, 0x1
2:
	j	ra
	nop
END(ls2k_version)
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

#define Index_Store_Tag_D			0x09
#define Index_Invalidate_I			0x00
#define Index_Writeback_Inv_D			0x01
#define Index_Writeback_Inv_S			0x01//delete scache  
#define Index_Store_Tag_S			0x0B 

LEAF(nullfunction)
	jr ra
	nop
END(nullfunction)

#define CP0_ECC  $26

LEAF(tlb_init)
        mtc0    $0, CP0_WIRED
        mtc0    $0, CP0_PAGEMASK
tlb_flush_all:
        lui     a0, 0x8000
        addiu   a1, $0, 64
        #a0=KSEG0,a1 = tlbsize, v0, v1, a3 used as local registers
        mtc0    $0, CP0_ENTRYLO0
        mtc0    $0, CP0_ENTRYLO1
        mfc0    v0, CP0_WIRED
        addu    v1, $0, a0
1:      sltu    a3, v0, a1
        beq     a3, $0, 1f
        nop
        mtc0    v1, CP0_ENTRYHI
        mtc0    v0, CP0_INDEX
        tlbwi
        addiu   v1, v1, 0x2000
        beq     $0, $0, 1b
        addiu   v0, v0, 1
1:
        ###tlb_init finish####
        tlbp
END(tlb_init)
###############################
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

        la      v0, CONFIG_SYS_NS16550_COM1
	#li	a0,(125000000/ 16)/CONFIG_BAUDRATE
	nop
	bal	1f
	nop

	jr AT
	nop
1:
        li      v1, FIFO_ENABLE|FIFO_RCV_RST|FIFO_XMT_RST|FIFO_TRIGGER_4
        sb      v1, NSREG(NS16550_FIFO)(v0)
        li      v1, CFCR_DLAB                  #DLAB
        sb      v1, NSREG(NS16550_CFCR)(v0)
        sb      a0, NSREG(NS16550_DATA)(v0)
        srl     a0, 8
        sb      a0, NSREG(NS16550_IER)(v0)     #set BRDH
        li      v1, CFCR_8BITS                 #8bit
        sb      v1, NSREG(NS16550_CFCR)(v0)
        li      v1, MCR_DTR|MCR_RTS
        sb      v1, NSREG(NS16550_MCR)(v0)
        li      v1, 0x0
        sb      v1, NSREG(NS16550_IER)(v0)

        j       ra
        nop
		.set at
END(initserial)


LEAF(tgt_putchar)
	.set noat
	move AT,ra
	la  v0, CONFIG_SYS_NS16550_COM1
	bal 1f
	nop
	jr AT
	nop
  
1:
	lbu v1, NSREG(NS16550_LSR)(v0)
	and v1, LSR_TXRDY
	beqz    v1, 1b
	nop
						 
	sb  a0, NSREG(NS16550_DATA)(v0)
	j   ra
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
        lbu     v1, NSREG(NS16550_LSR)(v0)
        and     v0, v1, LSR_RXRDY
        jr	ra
        nop
		.set at
END(tgt_testchar)

LEAF(tgt_getchar)
	.set noat
	move AT,ra
	la	v0, CONFIG_SYS_NS16550_COM1
	bal 1f
	nop
	jr AT
	nop
1:
	lbu	v1, NSREG(NS16550_LSR)(v0)
	and	v1, LSR_RXRDY
	beqz	v1, 1b
	nop
	lb	v0, NSREG(NS16550_DATA)(v0)
	jr ra
	nop
		.set at
END(tgt_getchar)

LEAF(beep_on)
	/* enable gpio35 output */
	li	t1, 0xbfe10500 + BEEP_GPIO*4/32
	li	v1, (1<<(BEEP_GPIO&31))
	lw	t0, 0(t1)
	or	t0, v1
	xor	t0, v1
	sw	t0, 0(t1)
	/* set gpio35 high */
	lw	t0, 0x10(t1)
	or	t0, v1
	sw	t0, 0x10(t1)
	
	jr	ra
	nop
END(beep_on)
	
LEAF(beep_off)
	/* enable gpio35 output */
	li	t1, 0xbfe10500 + BEEP_GPIO*4/32
	li	v1, (1<<(BEEP_GPIO&31))
	lw	t0, 0(t1)
	or	t0, v1
	xor	t0, v1
	sw	t0, 0(t1)
	/* set gpio35 high */
	lw	t0, 0x10(t1)
	or	t0, v1
	xor	t0, v1
	sw	t0, 0x10(t1)

	jr	ra
	nop
END(beep_off)

#include "i2c.S"

LEAF(spd_info_store)
	move    t8,ra

	TTYDBG("\r\n spd_info_store begin.\r\n")

	dli    t5, 0xffffffff8fffa000;

	dli    t7, 0xa9
	dli    t6, 0xad

	move	a0, t5
	daddiu	a1, a0, 0x200
1:
	sb	zero, 0(a0)
	daddiu	a0, 1
	bltu	a0, a1, 1b
	nop

4:
	move    a0, t7
	dli     a1, 0x2
	//GET_I2C_NODE_ID_a2
	bal     i2cread
	nop
	dli     t3, 0x80
	bltu    v0, t3, 2f
	nop
	move    t3, t5
	daddiu  t3, 0x100;
	move    t4, t5
1:
	sb      zero,0(t4)
	daddiu  t4, 0x1
	bltu    t4, t3, 1b
	nop
	b       3f
	nop
2:
	move    t4, t5
	dli     t0, 0x0 //used as counter

1:
	move    a0, t7
	move    a1, t0
	//GET_I2C_NODE_ID_a2
	bal     i2cread
	nop
	sb      v0, 0(t4)
	dli     a1, 0x100
	daddiu  t4, 0x1
	daddiu  t0, 0x1
	bne     t0, a1, 1b
	nop
3:
	daddiu  t5, 0x100
	daddiu  t7, 0x2
	bltu    t7, t6, 4b
	nop

	TTYDBG("\r\n spd_info_store done.\r\n")
	jr      t8
	nop
END(spd_info_store)

#######################################
#ifdef  AUTO_DDR_CONFIG
#include "../mm/detect_node_dimm_all.S"
#endif

#include "ls3A8_ddr_config.S"
#ifdef DDR3_DIMM
#include "loongson3C_ddr3_leveling.S"
#endif
#ifdef ARB_LEVEL
//#include "../mm/ARB_level_new.S"
#endif
#ifdef  DEBUG_DDR
#include "../mm/Test_Mem.S"
#endif

LEAF(watchdog_close)
	//disable watch DOG.
	/*gpio 3 output zero*/
	li t1,0xbfe10500

	li t2,(1 << 3)
	not t2,t2
	lw t3,0x0(t1)
	and t2,t3
	sw t2,0x0(t1)

	li t2,(1 << 3)
	lw t3,0x10(t1)
	not t2,t2
	and t2,t3
	sw t2,0x10(t1)
	nop

	jr		ra
	nop
END(watchdog_close)
    .align  5
#ifdef CONFIG_DDR_32BIT
#include "loongson_mc2_param-ddr32bit.S"
#elif defined(CONFIG_DDR_16BIT)
#include "loongson_mc2_param-ddr16bit.S"
#else
#include "loongson_mc2_param.S"
#endif

#ifdef  ARB_LEVEL
	.text
	.global c0_mc0_level_info 
	.global c0_mc1_level_info 
#ifdef  MULTI_CHIP
	.global c1_mc0_level_info 
	.global c1_mc1_level_info 
#endif

#include "../mm/loongson3A3_ddr_param.lvled.S"
#ifdef  MULTI_CHIP
#include "../mm/loongson3A3_ddr_param_c1.lvled.S"
#endif

#endif

	.text
	.global  nvram_offs
	.align 12
nvram_offs:
	.dword 0x0
	.align 12
	.dword 0x0

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
