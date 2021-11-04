/*whd : loongson3_clksetting.S
	change the PLL settings of each core
ATTENTION:
	 Using S1 for passing the NODE ID

	PLL
	40M > refclk/L1_div_ref >20M
	3.2G > refclk/ L1_div_ref*L1_loopc > 1.2G
	node_clock=refclk/L1_div_ref*L1_loopc/L1_divout/L2_divout
*/
#define SOFT_CLKSEL

#ifdef SOFT_CLKSEL
/* NODE PLL */
#define CORE_FREQ	400				//CPU 800Mhz
#define NODE_REFC	4
#define NODE_DIV	1
#define NODE_LOOPC	(CORE_FREQ*NODE_REFC*NODE_DIV/100)

/* DDR PLL */
#define DDR_FREQ	250				//MEM 400~600Mhz
#define DDR_REFC	4
#define DDR_DIV		1
#define DDR_LOOPC	(DDR_FREQ*DDR_REFC*DDR_DIV/100)
#define HDA_DIV		((DDR_FREQ*DDR_DIV+12)/24)		//HDA 24MHz
#define NETWORK_DIV	((DDR_FREQ*DDR_DIV+100)/200)		//NETWORK 200~400MHz
 

/* SOC PLL */
#define GMAC_FREQ	125				//GMAC 125MHz
#define GPU_FREQ	200				//GPU 200~300MHz
#define SB_FREQ		100				//SB 100~200MHz
#define SOC_LOOPC	(GMAC_FREQ*SOC_REFC*GMAC_DIV/100)
#define SOC_REFC	4
#define GMAC_DIV	16
#define GPU_DIV		((GMAC_FREQ*GMAC_DIV+GPU_FREQ/2)/GPU_FREQ)
#define SB_DIV		((GMAC_FREQ*GMAC_DIV+SB_FREQ/2)/SB_FREQ)

#if ((DDR_LOOPC > 255) | (NODE_LOOPC > 255) | (SOC_LOOPC > 255))
PLL LOOPC overflow
#endif

/* PIX PLL */
#define PIX0_LOOPC	109				//100~200MHz
#define PIX0_REFC	5
#define PIX0_DIV	20

#define PIX1_LOOPC	109
#define PIX1_REFC	5
#define PIX1_DIV	20

#define SEL_PLL0	(0x1)
#define SEL_PLL1	(0x2)
#define SEL_PLL2	(0x4)
#define PLL_L1_ENA	(0x1 << 3)
#define PLL_L1_LOCKED	(0x1 << 7)

	TTYDBG ("Soft CLK SEL adjust begin\r\n")
	TTYDBG ("\r\nNODE	:")

	li	t0, 0xbfe10400
	li	t1, (0x1 << 5)	//power down pll L1 first
	sw	t1, 0x0(t0)
	li	t1, (NODE_DIV << 24) | (NODE_LOOPC << 16) | (NODE_REFC << 8)
	sw	t1, 0x0(t0)
	ori	t1, PLL_L1_ENA
	sw	t1, 0x0(t0)

11:
	lw	a0, 0x0(t0)
	li	a1, PLL_L1_LOCKED
	and	a0, a1, a0
	beqz	a0, 11b //wait_locked_sys
	nop

	lw	a0, 0x0(t0)
	ori	a0, a0, SEL_PLL0
	sw	a0, 0x0(t0)

	bal	hexserial
	nop

	TTYDBG ("\r\nDDR	:")

	li	t0, 0xbfe10408
	li	t1, (0x1 << 5)	//power down pll first
	sw	t1, 0x0(t0)
	li	t1, (DDR_DIV << 24) | (DDR_LOOPC << 16) | (DDR_REFC << 8)
	li	t2, (HDA_DIV << 8) | (NETWORK_DIV)
	sw	t1, 0x0(t0)
	sw	t2, 0x4(t0)
	ori	t1, PLL_L1_ENA
	sw	t1, 0x0(t0)

21:
	lw	a0, 0x0(t0)
	li	a1, PLL_L1_LOCKED
	and	a0, a0, a1
	beqz	a0, 21b //wait_locked_ddr
	nop

	lw	a0, 0x0(t0)
	ori	a0, a0, SEL_PLL0 | SEL_PLL1 | SEL_PLL2
	sw	a0, 0x0(t0)

	bal	hexserial
	nop

	TTYDBG ("\r\nSOC	:")

	li	t0, 0xbfe10410
	li	t1, (0x1 << 5)	//power down pll first
	sw	t1, 0x0(t0)
	li	t1, (GPU_DIV << 24) | (SOC_LOOPC << 16) | (SOC_REFC << 8)
	li	t2, (GMAC_DIV << 8) | (SB_DIV)
	sw	t1, 0x0(t0)
	sw	t2, 0x4(t0)
	ori	t1, PLL_L1_ENA
	sw	t1, 0x0(t0)

21:
	lw	a0, 0x0(t0)
	li	a1, PLL_L1_LOCKED
	and	a0, a0, a1
	beqz	a0, 21b //wait_locked_ddr
	nop

	lw	a0, 0x0(t0)
	ori	a0, a0, SEL_PLL0 | SEL_PLL1 | SEL_PLL2
	sw	a0, 0x0(t0)

	bal	hexserial
	nop

	TTYDBG ("\r\nPIX0	:")

	li	t0, 0xbfe10418		//pll_pix0
	li	t1, (0x1 << 5)	//power down pll first
	sw	t1, 0x0(t0)
	li	t1, (PIX0_DIV << 24) | (PIX0_REFC << 8) | (PIX0_LOOPC << 16)
	sw	t1, 0x0(t0)
	ori	t1, PLL_L1_ENA
	sw	t1, 0x0(t0)

21:
	lw	a0, 0x0(t0)
	li	a1, PLL_L1_LOCKED
	and	a0, a0, a1
	beqz	a0, 21b
	nop

	lw	a0, 0x0(t0)
	ori	a0, a0, SEL_PLL0
	sw	a0, 0x0(t0)

	bal	hexserial
	nop

	TTYDBG ("\r\nPIX1	:")

	li	t0, 0xbfe10420		//pll_pix1
	li	t1, (0x1 << 5)	//power down pll first
	sw	t1, 0x0(t0)
	li	t1, (PIX1_DIV << 24) | (PIX1_REFC << 8) | (PIX1_LOOPC << 16)
	sw	t1, 0x0(t0)
	ori	t1, PLL_L1_ENA
	sw	t1, 0x0(t0)

21:
	lw	a0, 0x0(t0)
	li	a1, PLL_L1_LOCKED
	and	a0, a0, a1
	beqz	a0, 21b
	nop

	lw	a0, 0x0(t0)
	ori	a0, a0, SEL_PLL0
	sw	a0, 0x0(t0)

	bal	hexserial
	nop


#endif
