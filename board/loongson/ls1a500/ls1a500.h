#ifndef _LS2H_H
#define _LS2H_H

/* CHIP CONFIG regs */
#define LS1A500_GENERAL_CFG0				(0xbfe10100)
#define LS1A500_GENERAL_CFG1				(0xbfe10104)
#define LS1A500_GENERAL_CFG2				(0xbfe10108)
#define LS1A500_GENERAL_CFG3				(0xbfe1010c)
#define LS1A500_GENERAL_CFG4				(0xbfe10110)
#define LS1A500_GENERAL_CFG5				(0xbfe10114)
#define LS1A500_SAMPLE_CFG0				(0xbfe10120)
#define LS1A500_CHIP_HPT_LO				(0xbfe10130)
#define LS1A500_CHIP_HPT_HI				(0xbfe10134)

#define GPIO_SKIP_64_OFFSET				(0x20)
#define LS1A500_GPIO_00_63_DIR				(0xbfe10430)
#define LS1A500_GPIO_00_63_IN				(0xbfe10438)
#define LS1A500_GPIO_00_63_OUT				(0xbfe10440)

#define LS1A500_GPIO_MULTI_CFG				(0xbfe10490)
#define LS1A500_GPIO_IRQ_EN_CFG				(0xbfe104e0)
#define LS1A500_USB_PHY_CFG0				(0xbfe10500)
#define LS1A500_USB_PHY_CFG1				(0xbfe10504)
#define LS1A500_USB_PHY_CFG2				(0xbfe10508)
#define LS1A500_USB_PHY_CFG3				(0xbfe1050c)
#define LS1A500_PCIE0_CFG0				(0xbfe10530)
#define LS1A500_PCIE0_CFG1				(0xbfe10534)
#define LS1A500_PCIE0_CFG2				(0xbfe10538)
#define LS1A500_PCIE0_CFG3				(0xbfe1053c)
#define LS1A500_PCIE0_PHY_CFG0				(0xbfe10540)
#define LS1A500_PCIE0_PHY_CFG1				(0xbfe10544)


#define LS1A500_PIXCLK0_CTRL0_REG			(0xbfe10418)
#define LS1A500_PIXCLK0_CTRL1_REG			(0xbfe1041c)
#define LS1A500_PIXCLK1_CTRL0_REG			(0xbfe10420)
#define LS1A500_PIXCLK1_CTRL1_REG			(0xbfe10424)

#define PIXCLK_CTRL0_PSTDIV_SET	(1 << 31)
#define PIXCLK_CTRL0_PSTDIV_PD	(1 << 30)
#define PIXCLK_CTRL0_PSTDIV_DF	(0X3f << 24)
#define PIXCLK_CTRL0_PLL_PD	(1 << 7)
#define PIXCLK_CTRL0_PLL_LDF	(0xff << 16)
#define PIXCLK_CTRL0_PLL_ODF	(0x3 << 5)
#define PIXCLK_CTRL0_PLL_IDF	(0X7 << 2)
#define PIXCLK_CTRL0_REF_SEL	0X3

#define readb(addr)	(*(volatile unsigned char *)(addr))
#define readw(addr)	(*(volatile unsigned short *)(addr))
#define readl(addr)	(*(volatile unsigned int *)(addr))
#define readq(addr)	(*(volatile unsigned long long *)(addr))

#define writeb(b, addr)	((*(volatile unsigned char *)(addr)) = (b))
#define writew(w, addr)	((*(volatile unsigned short *)(addr)) = (w))
#define writel(l, addr)	((*(volatile unsigned int *)(addr)) = (l))
#define writeq(q, addr)	((*(volatile unsigned long long *)(addr)) = (q))

#define writel_reg_bit( addr, clear_bit, bit_val)	(*(volatile unsigned int*)(addr)) = (*(volatile unsigned int*)(addr)) & (~clear_bit) | bit_val

/* OTG regs */

/* USB regs */
#define LS1A500_EHCI_BASE				(0xbf050000)

/* GMAC regs */

/* HDA regs */

/* SATAregs */
#define LS1A500_SATA_BASE				(0xbf040000)

/* GPU regs */

/* DC regs */
#define	LS1A500_DC_BASE					(0xbf010000)
#define LS1A500_FB_CFG_DVO_REG				(0x1240)
#define LS1A500_FB_CFG_VGA_REG				(0x1250)
#define LS1A500_FB_ADDR0_DVO_REG			(0x1260)
#define LS1A500_FB_ADDR0_VGA_REG			(0x1270)
#define LS1A500_FB_STRI_DVO_REG				(0x1280)
#define LS1A500_FB_STRI_VGA_REG				(0x1290)
#define LS1A500_FB_ADDR1_DVO_REG			(0x1580)
#define LS1A500_FB_ADDR1_VGA_REG			(0x1590)

#define LS1A500_FB_CUR_CFG_REG				(0x1520)
#define LS1A500_FB_CUR_ADDR_REG				(0x1530)
#define LS1A500_FB_CUR_LOC_ADDR_REG			(0x1540)
#define LS1A500_FB_CUR_BACK_REG				(0x1550)
#define LS1A500_FB_CUR_FORE_REG				(0x1560)

#define LS1A500_FB_DAC_CTRL_REG				(0x1600)

/* SPI regs */
#define LS1A500_SPI0_BASE				(0xbfd00000)
#define LS1A500_SPI1_BASE				(0xbfd40000)
#define LS1A500_SPI2_BASE				(0xbff50000)
#define LS1A500_SPI3_BASE				(0xbff51000)
#define LS1A500_SPI4_BASE				(0xbff52000)
#define LS1A500_SPI5_BASE				(0xbff53000)

/* UART regs */
#define LS1A500_UART0_REG_BASE				(0xbff40000)
#define LS1A500_UART1_REG_BASE				(0xbff40400)
#define LS1A500_UART2_REG_BASE				(0xbff40800)
#define LS1A500_UART3_REG_BASE				(0xbff40c00)
#define LS1A500_UART4_REG_BASE				(0xbff41000)
#define LS1A500_UART5_REG_BASE				(0xbff41400)
#define LS1A500_UART6_REG_BASE				(0xbff41800)
#define LS1A500_UART7_REG_BASE				(0xbff41c00)
#define LS1A500_UART8_REG_BASE				(0xbff42000)
#define LS1A500_UART9_REG_BASE				(0xbff42400)

/* I2C regs */
//APB configured addr 0x1fe0,i2c0 addr is 0x1fe01000
#define LS1A500_I2C0_REG_BASE				0xbff48000
#define LS1A500_I2C0_PRER_LO_REG			(LS1A500_I2C0_REG_BASE + 0x0)
#define LS1A500_I2C0_PRER_HI_REG			(LS1A500_I2C0_REG_BASE + 0x1)
#define LS1A500_I2C0_CTR_REG   				(LS1A500_I2C0_REG_BASE + 0x2)
#define LS1A500_I2C0_TXR_REG   				(LS1A500_I2C0_REG_BASE + 0x3)
#define LS1A500_I2C0_RXR_REG    			(LS1A500_I2C0_REG_BASE + 0x3)
#define LS1A500_I2C0_CR_REG     			(LS1A500_I2C0_REG_BASE + 0x4)
#define LS1A500_I2C0_SR_REG     			(LS1A500_I2C0_REG_BASE + 0x4)

#define LS1A500_I2C1_REG_BASE				0xbff48800
#define LS1A500_I2C1_PRER_LO_REG			(LS1A500_I2C1_REG_BASE + 0x0)
#define LS1A500_I2C1_PRER_HI_REG			(LS1A500_I2C1_REG_BASE + 0x1)
#define LS1A500_I2C1_CTR_REG   				(LS1A500_I2C1_REG_BASE + 0x2)
#define LS1A500_I2C1_TXR_REG   				(LS1A500_I2C1_REG_BASE + 0x3)
#define LS1A500_I2C1_RXR_REG    			(LS1A500_I2C1_REG_BASE + 0x3)
#define LS1A500_I2C1_CR_REG     			(LS1A500_I2C1_REG_BASE + 0x4)
#define LS1A500_I2C1_SR_REG     			(LS1A500_I2C1_REG_BASE + 0x4)

#define LS1A500_I2C4_REG_BASE				0xbff4a000	//PIX0

#define CR_START					0x80
#define CR_STOP						0x40
#define CR_READ						0x20
#define CR_WRITE					0x10
#define CR_ACK						0x8
#define CR_IACK						0x1

#define SR_NOACK					0x80
#define SR_BUSY						0x40
#define SR_AL						0x20
#define SR_TIP						0x2
#define	SR_IF						0x1

/* PWM regs */
#define LS1A500_PWM0_REG_BASE				(0xbff5c000)
#define LS1A500_PWM1_REG_BASE				(0xbff5c010)

/* SDIO regs */
#define LS1A500_SDIO0_BASE 				0xbff64000
#define LS1A500_SDIO1_BASE 				0xbff66000

/* HPET regs */
#define LS1A500_HPET0_BASE 				0xbff68000
#define LS1A500_HPET0_PERIOD				LS1A500_HPET0_BASE + 0x4
#define LS1A500_HPET0_CONF				LS1A500_HPET0_BASE + 0x10 
#define LS1A500_HPET0_MAIN				LS1A500_HPET0_BASE + 0xF0 

#define LS1A500_HPET1_BASE 				0xbff69000
#define LS1A500_HPET2_BASE 				0xbff6a000
#define LS1A500_HPET3_BASE 				0xbff6b000

/* AC97 regs */
#define LS1A500_AC97_REG_BASE				(0xbff54000)

/* NAND regs */
#define LS1A500_NAND_REG_BASE				(0xbff58000)
#define LS1A500_NAND_CMD_REG				(LS1A500_NAND_REG_BASE + 0x0000)
#define LS1A500_NAND_ADDR_C_REG				(LS1A500_NAND_REG_BASE + 0x0004)
#define LS1A500_NAND_ADDR_R_REG				(LS1A500_NAND_REG_BASE + 0x0008)
#define LS1A500_NAND_TIMING_REG				(LS1A500_NAND_REG_BASE + 0x000c)
#define LS1A500_NAND_IDL_REG				(LS1A500_NAND_REG_BASE + 0x0010)
#define LS1A500_NAND_STA_IDH_REG			(LS1A500_NAND_REG_BASE + 0x0014)
#define LS1A500_NAND_PARAM_REG				(LS1A500_NAND_REG_BASE + 0x0018)
#define LS1A500_NAND_OP_NUM_REG				(LS1A500_NAND_REG_BASE + 0x001c)
#define LS1A500_NAND_CSRDY_MAP_REG			(LS1A500_NAND_REG_BASE + 0x0020)
#define LS1A500_NAND_DMA_ACC_REG			(LS1A500_NAND_REG_BASE + 0x0040)

/* ACPI regs */
#define LS1A500_ACPI_REG_BASE				(0xbff6c000)
#define LS1A500_PM_SOC_REG				(LS1A500_ACPI_REG_BASE + 0x0000)
#define LS1A500_PM_RESUME_REG				(LS1A500_ACPI_REG_BASE + 0x0004)
#define LS1A500_PM_RTC_REG				(LS1A500_ACPI_REG_BASE + 0x0008)
#define LS1A500_PM1_STS_REG				(LS1A500_ACPI_REG_BASE + 0x000c)
#define LS1A500_PM1_EN_REG				(LS1A500_ACPI_REG_BASE + 0x0010)
#define LS1A500_PM1_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x0014)
#define LS1A500_PM1_TMR_REG				(LS1A500_ACPI_REG_BASE + 0x0018)
#define LS1A500_P_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x001c)
#define LS1A500_P_LVL2_REG				(LS1A500_ACPI_REG_BASE + 0x0020)
#define LS1A500_P_LVL3_REG				(LS1A500_ACPI_REG_BASE + 0x0024)
#define LS1A500_GPE0_STS_REG				(LS1A500_ACPI_REG_BASE + 0x0028)
#define LS1A500_GPE0_EN_REG				(LS1A500_ACPI_REG_BASE + 0x002c)
#define LS1A500_RST_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x0030)
#define LS1A500_WD_SET_REG				(LS1A500_ACPI_REG_BASE + 0x0034)
#define LS1A500_WD_TIMER_REG				(LS1A500_ACPI_REG_BASE + 0x0038)
#define LS1A500_DVFS_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x003c)
#define LS1A500_DVFS_STS_REG				(LS1A500_ACPI_REG_BASE + 0x0040)
#define LS1A500_MS_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x0044)
#define LS1A500_MS_THT_REG				(LS1A500_ACPI_REG_BASE + 0x0048)
#define	LS1A500_THSENS_CNT_REG				(LS1A500_ACPI_REG_BASE + 0x004c)
#define LS1A500_GEN_RTC1_REG				(LS1A500_ACPI_REG_BASE + 0x0050)
#define LS1A500_GEN_RTC2_REG				(LS1A500_ACPI_REG_BASE + 0x0054)

/* DMA regs */
#define LS1A500_DMA_ORDER0				(0xbfe10c00)
#define LS1A500_DMA_ORDER1				(0xbfe10c10)
#define LS1A500_DMA_ORDER2				(0xbfe10c20)
#define LS1A500_DMA_ORDER3				(0xbfe10c30)


/* RTC regs */
#define LS1A500_RTC_REG_BASE				(0xbff6c100)
#define	LS1A500_TOY_TRIM_REG				(LS1A500_RTC_REG_BASE + 0x0020)
#define	LS1A500_TOY_WRITE0_REG				(LS1A500_RTC_REG_BASE + 0x0024)
#define	LS1A500_TOY_WRITE1_REG				(LS1A500_RTC_REG_BASE + 0x0028)
#define	LS1A500_TOY_READ0_REG				(LS1A500_RTC_REG_BASE + 0x002c)
#define	LS1A500_TOY_READ1_REG				(LS1A500_RTC_REG_BASE + 0x0030)
#define	LS1A500_TOY_MATCH0_REG				(LS1A500_RTC_REG_BASE + 0x0034)
#define	LS1A500_TOY_MATCH1_REG				(LS1A500_RTC_REG_BASE + 0x0038)
#define	LS1A500_TOY_MATCH2_REG				(LS1A500_RTC_REG_BASE + 0x003c)
#define	LS1A500_RTC_CTRL_REG				(LS1A500_RTC_REG_BASE + 0x0040)
#define	LS1A500_RTC_TRIM_REG				(LS1A500_RTC_REG_BASE + 0x0060)
#define	LS1A500_RTC_WRITE0_REG				(LS1A500_RTC_REG_BASE + 0x0064)
#define	LS1A500_RTC_READ0_REG				(LS1A500_RTC_REG_BASE + 0x0068)
#define	LS1A500_RTC_MATCH0_REG				(LS1A500_RTC_REG_BASE + 0x006c)
#define	LS1A500_RTC_MATCH1_REG				(LS1A500_RTC_REG_BASE + 0x0070)
#define	LS1A500_RTC_MATCH2_REG				(LS1A500_RTC_REG_BASE + 0x0074)

/* LPC regs */
#define LS1A500_LPC_MEM_BASE				(0xbc000000)
#define LS1A500_LPC_IO_BASE				(0xbf0d0000)
#define LS1A500_LPC_REG_BASE				(0xbf0e0000)
#define LS1A500_LPC_CFG0_REG				(LS1A500_LPC_REG_BASE + 0x0)
#define LS1A500_LPC_CFG1_REG				(LS1A500_LPC_REG_BASE + 0x4)
#define LS1A500_LPC_CFG2_REG				(LS1A500_LPC_REG_BASE + 0x8)
#define LS1A500_LPC_CFG3_REG				(LS1A500_LPC_REG_BASE + 0xc)

#if 0
#define LS2H_PCIE_MAX_PORTNUM       3
#define LS2H_PCIE_PORT0             0
#define LS2H_PCIE_PORT1             1
#define LS2H_PCIE_PORT2             2
#define LS2H_PCIE_PORT3             3
#define LS2H_PCIE_GET_PORTNUM(sysdata) \
        ((((struct pci_controller *)(sysdata))->mem_resource->start \
                        & ~LS2H_PCIE_MEM0_DOWN_MASK) >> 25)

#define LS2H_CHIP_CFG_REG_CLK_CTRL3     0x22c
#define LS2H_CLK_CTRL3_BIT_PEREF_EN(portnum) (1 << (24 + portnum))

#define LS2H_PCIE_MEM0_BASE_PORT(portnum)       (0x10000000 + (portnum << 25))
#define LS2H_PCIE_IO_BASE_PORT(portnum)         (0x18100000 + (portnum << 22))

#define LS2H_PCIE_REG_BASE_PORT(portnum)        (0x18118000 + (portnum << 22))
#define LS2H_PCIE_PORT_REG_CTR0			0x0
#define LS2H_PCIE_REG_CTR0_BIT_LTSSM_EN			(1 << 3)
#define LS2H_PCIE_REG_CTR0_BIT_REQ_L1			(1 << 12)
#define LS2H_PCIE_REG_CTR0_BIT_RDY_L23			(1 << 13)
#define LS2H_PCIE_PORT_REG_STAT1		0xC
#define LS2H_PCIE_REG_STAT1_MASK_LTSSM		0x0000003f
#define LS2H_PCIE_REG_STAT1_BIT_LINKUP			(1 << 6)
#define LS2H_PCIE_PORT_REG_CFGADDR		0x24
#define LS2H_PCIE_PORT_REG_CTR_STAT		0x28
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISX4			(1 << 26)
#define LS2H_PCIE_REG_CTR_STAT_BIT_ISRC			(1 << 27)

#define LS2H_PCIE_PORT_HEAD_BASE_PORT(portnum)  (0x18114000 + (portnum << 22))
#define LS2H_PCIE_DEV_HEAD_BASE_PORT(portnum)   (0x18116000 + (portnum << 22))

#define LIE_IN_WINDOW(addr,base,mask)   ((addr & mask) == base)
#define MAP_2_WINDOW(addr,mmap,mask)    ((addr & (~(mask))) | (mmap & mask))
#define LS2H_PCIE_MEM0_DOWN_BASE		0x10000000
#define LS2H_PCIE_MEM0_DOWN_MASK		0xf8000000
#define LS2H_PCIE_MEM0_UP_BASE			0x10000000
#define LS2H_PCIE_MEM0_UP_MASK			0xfe000000
#define LS2H_PCIE_IO_DOWN_BASE			0x18100000
#define LS2H_PCIE_IO_DOWN_MASK			0xff3f0000
#define LS2H_PCIE_IO_UP_BASE			0x0
#define LS2H_PCIE_IO_UP_MASK			0xffff0000
#endif

/* S3 Need */
#define STR_XBAR_CONFIG_NODE_a0(OFFSET, BASE, MASK, MMAP) \
        daddi   v0, t0, OFFSET;     \
        dli     t1, BASE;           \
        or      t1, t1, a0;         \
        sd      t1, 0x00(v0);       \
        dli     t1, MASK;           \
        sd      t1, 0x40(v0);       \
        dli     t1, MMAP;           \
        sd      t1, 0x80(v0);

#endif /*_LS2H_H*/
