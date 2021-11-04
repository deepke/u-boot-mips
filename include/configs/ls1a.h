/* SPDX-License-Identifier: GPL-2.0+ */
/*
 * (C) Copyright 2003
 * Wolfgang Denk, DENX Software Engineering, wd@denx.de.
 */

/*
 * This file contains the configuration parameters for qemu-mips64 target.
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#define APB_CLK 33333333
#define CPU_MULT 7
#define DDR_MULT 3
#define LS1FSOC 1

#ifndef CPU_CLOCK_RATE
#define CPU_CLOCK_RATE	APB_CLK*CPU_MULT	/* 800 MHz clock for the MIPS core */
#endif
#define CPU_TCLOCK_RATE CPU_CLOCK_RATE 
#define CONFIG_SYS_MIPS_TIMER_FREQ	(CPU_TCLOCK_RATE/4)

#define CONFIG_TIMESTAMP		/* Print image info with timestamp */

#define CONFIG_EXTRA_ENV_SETTINGS					\
        "serverip=10.0.0.19\0" \
        "ipaddr=10.0.0.9\0" \
        "ethaddr=10:84:7F:B5:9D:FC\0" \
	"addmisc=setenv bootargs ${bootargs} "				\
		"console=ttyS0,${baudrate} "				\
		"panic=1\0"						\
	"bootfile=vmlinux-ls1a\0"					\
	"load=tftp ffffffff80500000 ${u-boot}\0"			\
	"bootcmd=tftp 84000000;bootelf 84000000 g console=ttyS0,115200 rdinit=/sbin/init initcall_debug=1 loglevel=20\0"			\
	"memsize=256\0" \
	"memsize_high=256\0" \
	""

#define CONFIG_BOOTCOMMAND	"bootp;bootelf"

/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE

/*
 * Command line configuration.
 */

#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	1
#define CONFIG_SYS_NS16550_CLK		115200
#define CONFIG_SYS_NS16550_COM1		0xbfe40000

#define CONFIG_SYS_IDE_MAXBUS		2
#define CONFIG_SYS_ATA_IDE0_OFFSET	0x1f0
#define CONFIG_SYS_ATA_IDE1_OFFSET	0x170
#define CONFIG_SYS_ATA_DATA_OFFSET	0
#define CONFIG_SYS_ATA_REG_OFFSET	0
#define CONFIG_SYS_ATA_BASE_ADDR	0xffffffffb4000000

#define CONFIG_SYS_IDE_MAXDEVICE	4

/*
 * Miscellaneous configurable options
 */



/*
 * Memory map
 */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

#ifdef CONFIG_64BIT
# define CONFIG_SYS_SDRAM_BASE		0xffffffff80000000
#else
# define CONFIG_SYS_SDRAM_BASE		0x80000000
#endif
#define CONFIG_SYS_MEM_SIZE		(128 * 1024 * 1024)

#define CONFIG_SYS_INIT_SP_OFFSET	0x400000

#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x01000000)
#define CONFIG_SYS_MEMTEST_START	(CONFIG_SYS_SDRAM_BASE + 0x00100000)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_SDRAM_BASE + 0x00800000)

#define CONFIG_SYS_MALLOC_LEN		(2 *1024 * 1024)
#define CONFIG_SYS_BOOTPARAMS_LEN	(128 * 1024)
#define CONFIG_SYS_BOOTM_LEN		(64 * 1024 * 1024)

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */
/* The following #defines are needed to get flash environment right */

/* We boot from this flash, selected with dip switch */
#define CONFIG_SYS_FLASH_BASE		0xffffffffbfc00000
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_MAX_FLASH_SECT	128
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE

/* Address and size of Primary Environment Sector */
#define CONFIG_ENV_OFFSET  0xf0000
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SIZE		0x8000
//#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + CONFIG_ENV_OFFSET)

#define CONFIG_ENV_OVERWRITE	1

/*USB*/
#define CONFIG_CMD_USB
#define CONFIG_USB_STORAGE
#define CONFIG_SYS_USB_EVENT_POLL 

#if 0
/*USB/EHCI*/
#define CONFIG_USB_EHCI
#define CONFIG_USB_EHCI_PCI
#define CONFIG_PCI_EHCI_DEVICE			0
#define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS 15
#define CONFIG_EHCI_DCACHE
#endif
#if 1
/*USB/OHCI*/
#define CONFIG_USB_KEYBOARD 
#define CONFIG_USB_OHCI_NEW
//#define CONFIG_PCI_OHCI
//#define CONFIG_SYS_USB_OHCI_BOARD_INIT
//#define CONFIG_SYS_USB_OHCI_CPU_INIT
#define CONFIG_SYS_USB_OHCI_REGS_BASE 0xbfe08000
#define  CONFIG_SYS_USB_OHCI_SLOT_NAME "ls1b_ohci"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS	15
#endif
#define CONFIG_SPL_START_S_PATH		"arch/mips/loongson/ls1a"
#define DEBUG


#define frombus(x) ((long)(x)|0xa0000000)
#define tobus(x)    (((unsigned long)(x)&0x1fffffff) )

#define CONFIG_DEBUG_UART_CLOCK (APB_CLK*DDR_MULT/2)
//#define CONFIG_NAND_SUPPORT 1
#define CONFIG_SYS_MAX_NAND_DEVICE 4

#endif /* __CONFIG_H */
