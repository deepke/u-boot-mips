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

#define CONFIG_TIMESTAMP		/* Print image info with timestamp */

#define CONFIG_EXTRA_ENV_SETTINGS					\
	"addmisc=setenv bootargs ${bootargs} "				\
		"console=ttyS0,${baudrate} "				\
		"panic=1\0"						\
	"bootfile=uImage\0"					\
	"load=tftp ffffffff80500000 ${u-boot}\0"			\
        "serverip=10.50.122.29\0" \
        "ipaddr=10.0.0.9\0" \
	"stdin=serial,usbkbd\0" \
	"stdout=serial,vidconsole\0" \
	"stderr=serial,vidconsole\0" \
	"bootargs=console=ttyS0,115200 initcall_debug=1 loglevel=20\0" \
	"bootcmd1=usb start;fatload usb 0 0x88000000 boot/uImage;bootm 0x88000000;\0" \
	""

#define CONFIG_BOOTCOMMAND	"bootp;bootm"

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
#define CONFIG_SYS_NS16550_COM1		0xffffffffbff40800

#ifdef CONFIG_SYS_BIG_ENDIAN
#define CONFIG_IDE_SWAP_IO
#endif

#define CONFIG_SYS_IDE_MAXBUS		2
#define CONFIG_SYS_ATA_IDE0_OFFSET	0x1f0
#define CONFIG_SYS_ATA_IDE1_OFFSET	0x170
#define CONFIG_SYS_ATA_DATA_OFFSET	0
#define CONFIG_SYS_ATA_REG_OFFSET	0
#define CONFIG_SYS_ATA_BASE_ADDR	0xffffffffbf040000

#define CONFIG_SYS_IDE_MAXDEVICE	4

/*
 * Miscellaneous configurable options
 */



#define CONFIG_SYS_MHZ			132

#define CONFIG_SYS_MIPS_TIMER_FREQ	(CONFIG_SYS_MHZ * 1000000)

/*
 * Memory map
 */
#define CONFIG_SYS_MONITOR_BASE		CONFIG_SYS_TEXT_BASE

#ifdef CONFIG_64BIT
# define CONFIG_SYS_SDRAM_BASE		0xffffffff80000000
#else
# define CONFIG_SYS_SDRAM_BASE		0x80000000
#endif
#define CONFIG_SYS_MEM_SIZE		(256 * 1024 * 1024)
#define CONFIG_LINUX_DTB_BY_A2		1

#define CONFIG_SYS_INIT_SP_OFFSET	0x400000

#define CONFIG_SYS_LOAD_ADDR		(CONFIG_SYS_SDRAM_BASE + 0x08000000)
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
#define CONFIG_SYS_FLASH_BASE		0x1fc00000
#define CONFIG_SYS_MAX_FLASH_BANKS	1
#define CONFIG_SYS_MAX_FLASH_SECT	128
#define CONFIG_SYS_FLASH_CFI
#define CONFIG_FLASH_CFI_DRIVER
#define CONFIG_SYS_FLASH_USE_BUFFER_WRITE

/* Address and size of Primary Environment Sector */
#define CONFIG_ENV_OFFSET  0xf0000
#define CONFIG_ENV_SECT_SIZE		(64 * 1024)
#define CONFIG_ENV_SIZE		0x8000
//#define CONFIG_ENV_ADDR		(CONFIG_SYS_FLASH_BASE + (1 << 20) - CONFIG_ENV_SIZE)

#define CONFIG_ENV_OVERWRITE	1

/*USB*/
#define CONFIG_CMD_USB
#define CONFIG_DOS_PARTITION
#define CONFIG_USB_STORAGE
#define CONFIG_CMD_ELF
#define CONFIG_CMD_EXT2
#define CONFIG_CMD_EXT4
#define CONFIG_BOOTM_LINUX
#define CONFIG_SYS_USB_EVENT_POLL 
#define CONFIG_CMD_FAT

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
#define CONFIG_SPL_START_S_PATH		"arch/mips/loongson/ls1a500"
//#define DEBUG

#define CONFIG_SYS_SCSI_MAX_SCSI_ID	1
#define CONFIG_SYS_SCSI_MAX_LUN		1
#define CONFIG_SYS_SCSI_MAX_DEVICE	(CONFIG_SYS_SCSI_MAX_SCSI_ID * \
					CONFIG_SYS_SCSI_MAX_LUN)
#define SCSI_VEND_ID PCI_VENDOR_ID_LOONGSON
#define SCSI_DEV_ID  PCI_DEVICE_ID_LOONGSON_SATA
#define frombus(x) ((long)(x)|0x80000000)
#define tobus(x)    (((unsigned long)(x)&0x1fffffff) )
#define CONFIG_DMA_COHERENT 1
#define FB_XSIZE 1280
#define FB_YSIZE 1024
#define NOGPU

#endif /* __CONFIG_H */
