/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 * Gary Jennejohn <garyj@denx.de>
 * David Mueller <d.mueller@elsoft.ch>
 
 * Configuation settings for the SAMSUNG IJ3K2440 board.
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

#include <asm/sizes.h>
/*
 * High Level Configuration Options
 * (easy to change)
 */
#define CONFIG_ARM920T	1	/* This is an ARM920T Core	*/
#define CONFIG_S3C24X0	1	/* in a SAMSUNG S3C24x0-type SoC	*/
#define CONFIG_S3C2440	1	/* specifically a SAMSUNG S3C2440 SoC	*/
#define CONFIG_IJ3K2440	1	/* on a SAMSUNG IJ3K2440 Board  */

/* input clock of PLL */
#define CONFIG_SYS_CLK_FREQ	12000000/* the IJ3K2440 has 12MHz input clock */


#define USE_920T_MMU		1
#undef CONFIG_USE_IRQ			/* we don't need IRQ/FIQ stuff */

/*
 * Size of malloc() pool
 */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + 2048*1024)
//#define CONFIG_SYS_GBL_DATA_SIZE	128	/* size in bytes reserved for initial data */

/*
 * Hardware drivers
 */
#define CONFIG_NET_MULTI
#define CONFIG_DRIVER_DM9000            1
#define CONFIG_DRIVER_NO_EEPROM         1
#define CONFIG_DM9000_BASE              0x20000300
#define DM9000_IO                       (CONFIG_DM9000_BASE)
#define DM9000_DATA                     (CONFIG_DM9000_BASE+4)

/*
 * select serial console configuration
 */
#define CONFIG_S3C24X0_SERIAL
//#define CONFIG_SERIAL_MULTI
#define CONFIG_SERIAL2          1      /* we use SERIAL 1 on IJ3K2440 */

/************************************************************
 * RTC
 ************************************************************/
#define	CONFIG_RTC_S3C24X0	1

/* allow to overwrite serial and ethaddr */
//#define CONFIG_ENV_OVERWRITE

//#define CONFIG_BAUDRATE		115200
#define CONFIG_BAUDRATE		57600


/*
 * BOOTP options
 */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_BOOTPATH
#define CONFIG_BOOTP_GATEWAY
#define CONFIG_BOOTP_HOSTNAME


/*
 * Command line configuration.
 */
#include <config_cmd_default.h>

#define CONFIG_CMDLINE_EDITING
#define CONFIG_CMD_CACHE
#define CONFIG_CMD_DATE
//#define CONFIG_CMD_ELF
#define CONFIG_CMD_FAT
#define CONFIG_CMD_JFFS2
/* for ubi */
#define CONFIG_CMD_UBIFS
#define CONFIG_CMD_UBI
#define CONFIG_RBTREE
#define CONFIG_LZO

#define CONFIG_CMD_NAND
#undef CONFIG_CMD_FPGA
#define CONFIG_SYS_HUSH_PARSER
#ifdef	CONFIG_SYS_HUSH_PARSER
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#endif


#define CONFIG_BOOTDELAY	0
#define CONFIG_AUTOBOOT_STOP_STR "v"
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_BOOTARGS	""
/*#define CONFIG_ETHADDR	08:00:3e:26:0a:5b */
//#define CONFIG_NETMASK          255.255.255.0
//#define CONFIG_IPADDR		10.1.2.13
//#define CONFIG_SERVERIP		10.1.2.5
#define CONFIG_BOOTCOMMAND	"run chkMfg; run bootboard"

#define CONFIG_DOS_PARTITION	1

#if defined(CONFIG_CMD_KGDB)
#define CONFIG_KGDB_BAUDRATE	115200		/* speed to run kgdb serial port */
/* what's this ? it's not used anywhere */
#define CONFIG_KGDB_SER_INDEX	1		/* which serial port to use */
#endif

/*
 * Miscellaneous configurable options
 */
#define	CONFIG_SYS_LONGHELP				/* undef to save memory		*/
#define	CONFIG_SYS_PROMPT		"IJ3K2440 # "	/* Monitor Command Prompt	*/
#define	CONFIG_SYS_CBSIZE		256		/* Console I/O Buffer Size	*/
#define	CONFIG_SYS_PBSIZE (CONFIG_SYS_CBSIZE+sizeof(CONFIG_SYS_PROMPT)+16) /* Print Buffer Size */
#define	CONFIG_SYS_MAXARGS		16		/* max number of command args	*/
#define CONFIG_SYS_BARGSIZE		CONFIG_SYS_CBSIZE	/* Boot Argument Buffer Size	*/

#define CONFIG_SYS_MEMTEST_START	0x30000000	/* memtest works on	*/
#define CONFIG_SYS_MEMTEST_END		0x33F00000	/* 63 MB in DRAM	*/

#define	CONFIG_SYS_LOAD_ADDR		0x32000000	/* default load address	*/

#define	CONFIG_SYS_HZ			1000

/* valid baudrates */
#define CONFIG_SYS_BAUDRATE_TABLE	{ 9600, 19200, 38400, 57600, 115200 }

/*-----------------------------------------------------------------------
 * Stack sizes
 *
 * The stack sizes are set up in start.S using the settings below
 */
#define CONFIG_STACKSIZE	(128*1024)	/* regular stack */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ	(8*1024)	/* IRQ stack */
#define CONFIG_STACKSIZE_FIQ	(4*1024)	/* FIQ stack */
#endif

/* for s3c ohci driver */
//#define CONFIG_USE_CPU_USB_OHCI
//#define CONFIG_USB_OHCI
/* end of s3c ohci drivers */
#define CONFIG_SYS_NO_DCACHE /* allows usb to work ?? */
#define CONFIG_USB_OHCI_NEW	1
#define CONFIG_CMD_USB		1
#define CONFIG_USB_STORAGE


#define CONFIG_SYS_USB_OHCI_CPU_INIT	1
#define CONFIG_SYS_USB_OHCI_REGS_BASE	0x49000000 /* S3C24X0_USB_HOST_BASE */
#define CONFIG_SYS_USB_OHCI_SLOT_NAME	"s3c2440"
#define CONFIG_SYS_USB_OHCI_MAX_ROOT_PORTS 	2

/*-----------------------------------------------------------------------
 * Physical Memory Map
 */
#define CONFIG_NR_DRAM_BANKS	1	   /* we have 1 bank of DRAM */
#define PHYS_SDRAM_1		0x30000000 /* SDRAM Bank #1 */
#define PHYS_SDRAM_1_SIZE	0x04000000 /* 64 MB */

#define PHYS_FLASH_1		0x00000000 /* Flash Bank #1 */

#define CONFIG_SYS_FLASH_BASE		PHYS_FLASH_1

/* video settings */
#define CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SILENT_CONSOLE
//#define CONFIG_KEYBOARD

#define CONFIG_VIDEO_SW_CURSOR
#define VIDEO_FB_16BPP_WORD_SWAP
#define CONFIG_VIDEO_S3C24X0
#define LCD_VIDEO_ADDR           0x31000000
#define CONFIG_SYS_DEFAULT_VIDEO_MODE 0x311

/*
 * When booting from NAND, it is impossible to access the lowest addresses
 * due to the SteppingStone being in the way. Luckily the NOR doesn't really
 * care about the highest 16 bits of address, so we set the controlers
 * registers to go and poke over there, instead.
 */
#define CONFIG_FLASH_BASE	(0x0000 + PHYS_FLASH_1)

/*
 * NOR FLASH organization
 * Now uses the standard CFI interface
 */
#define PHYS_FLASH_SIZE		0x00200000 /* 2MB */
//#define CONFIG_SYS_FLASH_CFI	1
//#define CONFIG_FLASH_CFI_DRIVER	1
#define CONFIG_SYS_FLASH_CFI_WIDTH	FLASH_CFI_16BIT
#define CONFIG_SYS_MONITOR_BASE	0x0
#define CONFIG_SYS_MAX_FLASH_BANKS	1	/* max number of memory banks */
#define CONFIG_SYS_MAX_FLASH_SECT	512	/* 512 * 4096 sectors, or 32 * 64k blocks */
#define CONFIG_FLASH_SHOW_PROGRESS	1

/*-----------------------------------------------------------------------*/
#define CONFIG_SYS_SDRAM_BASE   (PHYS_SDRAM_1)
/*#define CONFIG_SYS_INIT_SP_ADDR (GENERATED_GBL_DATA_SIZE + PHYS_SDRAM_1)*/
#define CONFIG_SYS_INIT_SP_ADDR	(CONFIG_SYS_SDRAM_BASE + SZ_4K \
				- GENERATED_GBL_DATA_SIZE)
//#define CONFIG_SYS_INIT_SP_ADDR (CONFIG_SYS_LOAD_ADDR - 0x1000000)
#define CONFIG_BOARD_EARLY_INIT_F
/*-----------------------------------------------------------------------*/

/*-----------------------------------------------------------------------
 * FLASH and environment organization
 */

#define CONFIG_ENV_ADDR          (PHYS_FLASH_1 + 0x40000)

#define CONFIG_SYS_MAX_FLASH_BANKS	1	/* max number of memory banks */
/* timeout values are in ticks */
#define CONFIG_SYS_FLASH_ERASE_TOUT	(5*CONFIG_SYS_HZ) /* Timeout for Flash Erase */
#define CONFIG_SYS_FLASH_WRITE_TOUT	(5*CONFIG_SYS_HZ) /* Timeout for Flash Write */

/* ATAG configuration */
#define CONFIG_INITRD_TAG		1
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_CMDLINE_TAG		1

//#define	CONFIG_ENV_IS_IN_FLASH	1
#define CONFIG_ENV_IS_IN_NAND   1
#define CONFIG_ENV_SIZE		0x20000	/* Total Size of Environment Sector */
//#define CONFIG_ENV_OFFSET       0x40000
#define CONFIG_ENV_OFFSET_OOB   1

#define CONFIG_SYS_NAND_MAX_CHIPS	1
#define CONFIG_SYS_NAND_BASE	0x4E000000 /* S3C2440_NAND_BASE */
#define CONFIG_SYS_MAX_NAND_DEVICE	1
#define CONFIG_NAND_S3C2410
#define CONFIG_S3C2410_NAND_BBT
#define CONFIG_MTD_NAND_VERIFY_WRITE

#define CONFIG_AUTO_UPDATE

#define MTDIDS_DEFAULT		"nand0=ij3k2440-nand"
#define CONFIG_CMD_MTDPARTS
#define CONFIG_MTDPARTS_DEFAULT "mtdparts=mtdparts=ij3k2440-nand:256k@0(u-boot),128k(env),5m(kernel),-(root)"

#define CONFIG_MTD_DEVICE
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_PING
//#define CONFIG_YAFFS2
#define CONFIG_MISC_INIT_R
#define CONFIG_PRE_VIDEO_INIT

#define CONFIG_EXTRA_ENV_SETTINGS	\
	CONFIG_MTDPARTS_DEFAULT "\0" \
	"fileaddr=32000000\0" \
        "autostart=n\0" \
	"bootboard=run chkNandEnv; run chkVid; run chkBoot\0" \
        "chkip=if test $ipaddr -ne \"\"; then print ipaddr; else bootp; fi\0" \
        "chksrvr=if test $srvrip -ne \"\"; then setenv serverip $srvrip; print serverip; fi\0" \
        "chkscript=source ${fileaddr}\0" \
	"netboot=run chkip; run chksrvr; run chkscript\0" \
	"chkBoot=if nboot.e kernel; then bootm; else run netboot; fi\0" \
	"chkMfg=if iminfo 100000; then source 100000; fi\0" \
	"chkNandEnv=if nand env.oob get; then echo Env OK; else nand env.oob set env; fi\0" \
	"chkVid=if print videomode; then echo Video OK; else vid set; setenv stdout vga; setenv stderr vga; saveenv; reset; fi\0" \
	"vid_02=x:800,y:480,depth:16,pclk:1,hs:152,vs:3,up:29,lo:3,ri:40,le:40,hs:48,vmode:188,sync:2825\0" \
	"set_bootargs=setenv bootargs ${bootargs_base} ${bootargs_init} ${bootargs_other} ${bootargs_rootfs}\0" \
	""
#endif	/* __CONFIG_H */
