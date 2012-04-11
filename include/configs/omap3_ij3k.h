/*
 * (C) Copyright 2006-2008
 * Texas Instruments.
 * Richard Woodruff <r-woodruff2@ti.com>
 * Syed Mohammed Khasim <x0khasim@ti.com>
 *
 * (C) Copyright 2009
 * Frederik Kriewitz <frederik@kriewitz.eu>
 *
 * Configuration settings for the ij3k-omap board.
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
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	 See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#ifndef __CONFIG_H
#define __CONFIG_H

/* High Level Configuration Options */
#define CONFIG_ARMV7		1	/* This is an ARM V7 CPU core */
#define CONFIG_OMAP		1	/* in a TI OMAP core */
#define CONFIG_OMAP34XX		1	/* which is a 34XX */
#define CONFIG_OMAP3430		1	/* which is in a 3430 */
#define CONFIG_OMAP3_IJ3K	1	/* working with omap-ij3k */

#define	CONFIG_SYS_TEXT_BASE	0x80008000

#define CONFIG_SDRC	/* The chip has SDRC controller */

#include <asm/arch/cpu.h>		/* get chip and board defs */
#include <asm/arch/omap3.h>

/* Display CPU and Board information */
//#define CONFIG_DISPLAY_CPUINFO		1
//#define CONFIG_DISPLAY_BOARDINFO	1

/* Clock Defines */
#define V_OSCK				26000000	/* Clock output from T2 */
#define V_SCLK				(V_OSCK >> 1)

#undef CONFIG_USE_IRQ			/* no support for IRQs */
#define CONFIG_MISC_INIT_R

#define CONFIG_CMDLINE_TAG		1	/* enable passing of ATAGs */
#define CONFIG_SETUP_MEMORY_TAGS	1
#define CONFIG_INITRD_TAG		1
#define CONFIG_REVISION_TAG		1

#define CONFIG_OF_LIBFDT		1

/* Size of malloc() pool */
#define CONFIG_ENV_SIZE			(128 << 10)	/* 128 KiB */
						/* Sector */
#define CONFIG_SYS_MALLOC_LEN		(CONFIG_ENV_SIZE + (512 << 10))

/* Hardware drivers */

/* DDR - I use Micron DDR */
#define CONFIG_OMAP3_MICRON_DDR		1

#define SER_PORT                        1
/* DM9000 */
#define CONFIG_NET_MULTI		1
#define CONFIG_NET_RETRY_COUNT		20
#define	CONFIG_DRIVER_DM9000		1
#if 3==SER_PORT
#define	CONFIG_DM9000_BASE		0x2c000000
#define	DM9000_DATA			(CONFIG_DM9000_BASE + 0x400)
#define CONFIG_DM9000_NO_SROM		1
#else
#define	CONFIG_DM9000_BASE		(0x2c000000 + 0x300)
#define	DM9000_DATA			(CONFIG_DM9000_BASE + 0x4)
#endif
#define	DM9000_IO			CONFIG_DM9000_BASE
#define	CONFIG_DM9000_USE_16BIT		1
#undef	CONFIG_DM9000_DEBUG

/* my additions */
#define CONFIG_NET_BOOTPS_PORT           6767
#define CONFIG_CMD_PING
#define CONFIG_MTD_PARTITIONS
#define CONFIG_CMD_UBI
#ifdef CONFIG_CMD_UBI
#define CONFIG_CMD_UBIFS
#define CONFIG_RBTREE
#define CONFIG_LZO
#endif
#define CONFIG_CMDLINE_EDITING

#define CONFIG_BOOTDELAY	0
#define CONFIG_AUTOBOOT_STOP_STR "v"
#define CONFIG_ZERO_BOOTDELAY_CHECK
#define CONFIG_AUTOBOOT_KEYED
#define CONFIG_SYS_DCACHE_OFF
/* USB */
#define CONFIG_CMD_USB

#ifdef CONFIG_CMD_USB
//#define CONFIG_MUSB_UDC			1
#define CONFIG_MUSB_HCD                 1
#define CONFIG_USB_OMAP3		1
#define CONFIG_TWL4030_USB		1

/* USB EHCI */
//#define CONFIG_USB_EHCI
#define CONFIG_SYS_USB_EHCI_MAX_ROOT_PORTS 3
#define CONFIG_USB_STORAGE
#endif

/* video settings */
#define CONFIG_VIDEO
#ifdef CONFIG_VIDEO
#define CONFIG_CFB_CONSOLE
#define CONFIG_SYS_CONSOLE_IS_IN_ENV
#define CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
#define CONFIG_VGA_AS_SINGLE_DEVICE
#define CONFIG_SILENT_CONSOLE

#define CONFIG_VIDEO_SW_CURSOR
#define VIDEO_FB_16BPP_WORD_SWAP
#define CONFIG_VIDEO_OMAP3
#define LCD_VIDEO_ADDR           0x81000000
#define CONFIG_SYS_DEFAULT_VIDEO_MODE 0x311
#endif
/* End of my additions */

/* NS16550 Configuration */
#define CONFIG_SYS_NS16550
#define CONFIG_SYS_NS16550_SERIAL
#define CONFIG_SYS_NS16550_REG_SIZE	(-4)
#define CONFIG_SYS_NS16550_CLK		48000000 /* 48MHz (APLL96/2) */

/* select serial console configuration */
#define CONFIG_CONS_INDEX		SER_PORT
#if 1==CONFIG_CONS_INDEX
#define CONFIG_SYS_NS16550_COM1		OMAP34XX_UART1
#define CONFIG_SERIAL1			CONFIG_CONS_INDEX
#elif 3==CONFIG_CONS_INDEX
#define CONFIG_SYS_NS16550_COM3		OMAP34XX_UART3
#define CONFIG_SERIAL3			CONFIG_CONS_INDEX
#endif
#define CONFIG_BAUDRATE			115200
#define CONFIG_SYS_BAUDRATE_TABLE	{4800, 9600, 19200, 38400, 57600,\
					115200}

/* MMC */
#define CONFIG_MMC			1
#define CONFIG_GENERIC_MMC             1
#define CONFIG_OMAP_HSMMC               1
#define CONFIG_SPL_MMC_SUPPORT          1
#define CONFIG_DOS_PARTITION		1

/* I2C */
#define CONFIG_HARD_I2C			1
#define CONFIG_SYS_I2C_SPEED		100000
#define CONFIG_SYS_I2C_SLAVE		1
#define CONFIG_SYS_I2C_BUS		0
#define CONFIG_SYS_I2C_BUS_SELECT	1
#define CONFIG_DRIVER_OMAP34XX_I2C	1

/* TWL4030 */
#define CONFIG_TWL4030_POWER		1
#define CONFIG_TWL4030_LED		1

/* Board NAND Info */
#define CONFIG_SYS_NO_FLASH		/* no NOR flash */
#define CONFIG_MTD_DEVICE		/* needed for mtdparts commands */
#define MTDIDS_DEFAULT			"nand0=nand"
#define MTDPARTS_DEFAULT		"mtdparts=nand:" \
						"512k(x-loader)," \
						"1920k(u-boot)," \
						"128k(env)," \
						"128k(upgrade)," \
						"4m(kernel)," \
						"-(fs)"

#define CONFIG_NAND_OMAP_GPMC
#define CONFIG_SYS_NAND_ADDR		NAND_BASE	/* physical address */
							/* to access nand */
#define CONFIG_SYS_NAND_BASE		NAND_BASE	/* physical address */
							/* to access nand at */
							/* CS0 */
#define GPMC_NAND_ECC_LP_x16_LAYOUT	1

#define CONFIG_SYS_MAX_NAND_DEVICE	1		/* Max number of NAND */
							/* devices */

/* commands to include */
#include <config_cmd_default.h>

#define CONFIG_CMD_FAT			/* FAT support			*/
#define CONFIG_CMD_I2C			/* I2C serial bus support	*/
#define CONFIG_CMD_MMC			/* MMC support			*/
#define CONFIG_CMD_MTDPARTS		/* Enable MTD parts commands	*/
#define CONFIG_CMD_NAND			/* NAND support			*/
#define CONFIG_CMD_NAND_LOCK_UNLOCK	/* nand (un)lock commands	*/

#undef CONFIG_CMD_LOADB
#undef CONFIG_CMD_LOADS
#undef CONFIG_CMD_NFS
#undef CONFIG_CMD_FPGA			/* FPGA configuration Support	*/
#undef CONFIG_CMD_IMI			/* iminfo			*/

/* BOOTP/DHCP options */
#define CONFIG_BOOTP_BOOTFILESIZE
#define CONFIG_BOOTP_SEND_HOSTNAME
#undef CONFIG_BOOTP_VENDOREX

/* Environment information */
#define CONFIG_ENV_OVERWRITE /* allow to overwrite serial and ethaddr */

/* see .c file for mmc settings */

#define CONFIG_EXTRA_ENV_SETTINGS \
	"loadaddr=0x82000000\0" \
        "autostart=n\0" \
	/*"console=console=ttyO2,115200n8\0"*/ \
	/*"video=vram=4M omapfb.mode=lcd:800x480 "*/ \
		/*"omapdss.def_disp=lcd\0"*/ \
                /*"video=omapfb:mode:7inch_LCD\0"*/ \
	/*"nfsopts=hard,tcp,rsize=65536,wsize=65536\0"*/ \
	"nandrootfs=ubi.mtd=5 root=ubi0:rootfs rootfstype=ubifs\0" \
	"chkVid=find ${disp} modedb res; set video vram=4M omapfb.mode=${res} omapdss.def_disp=lcd\0" \
	"modedb=00(stn)01(lcd:800x480)02(lcd:800x480)04(lcd:800x480)08(lcd:640x480)\0" \
	"nandargs=run chkVid; " \
		"setenv bootargs " \
                "${kopts} " \
                "${video} " \
		"${nandrootfs}\0" \
        "mtdparts=" MTDPARTS_DEFAULT "\0" \
        "mtdids=" MTDIDS_DEFAULT "\0" \
	"nandboot=echo Booting from nand ...; " \
		"run nandargs; " \
		/*"nand read ${loadaddr} 280000 400000; "*/ \
		"bootm ${loadaddr}\0" \
	/*"netboot=echo Booting from network ...; " \
		"dhcp ${loadaddr}; " \
		"run netargs; " \
		"bootm ${loadaddr}\0"*/ \
	"chkImg=if test $btn -eq 0 && nboot.e kernel; then run nandboot; elif test $btn -ne 2; then run upgrade; else echo cmd; fi\0" \
	"upgrade=if run usbload; then ; else run chkip; fi; if imxtract; then source ${fileaddr}; else echo 'Error in image'; fi\0" \
	"usbload=usb start; fatload usb 0 ${loadaddr} install.img\0" \
	"chkip=if test $ipaddr -ne \"\" && test $serverip -ne \"\"; then run netboot; else bootp; fi\0" \
	"write_img=nand erase.part ${name}; nand write.i ${loadaddr} ${name} ${filesize}; set name;\0" \
	"netboot=if tftp uImage; then run nandargs; bootm ${loadaddr}; fi\0" \
	"load_fs=if tftp ubi.img; then set name fs; run write_img; fi\0" \
	"load_krnl=if tftp uImage; then set name kernel; run write_img; fi\0" \
	"load_uboot=if tftp u-boot.bin; then set name u-boot; run write_img; fi\0" \
	"load_xload=if tftp x-load.bin.ift_NAND; then nandecc hw; set name x-loader; run write_img; fi\0" \
	"stdout=vga\0" \
	"stderr=vga\0" \
	""

#define CONFIG_BOOTCOMMAND "run chkImg"

/* Miscellaneous configurable options */
#define CONFIG_SYS_LONGHELP		/* undef to save memory */
#define CONFIG_SYS_HUSH_PARSER		/* use "hush" command parser */
#define CONFIG_AUTO_COMPLETE		1
#define CONFIG_SYS_PROMPT_HUSH_PS2	"> "
#define CONFIG_SYS_PROMPT		"OMAP3 inkjet # "
#define CONFIG_SYS_CBSIZE		512	/* Console I/O Buffer Size */
/* Print Buffer Size */
#define CONFIG_SYS_PBSIZE		(CONFIG_SYS_CBSIZE + \
					sizeof(CONFIG_SYS_PROMPT) + 16)
#define CONFIG_SYS_MAXARGS		128	/* max number of command args */

/* Boot Argument Buffer Size */
#define CONFIG_SYS_BARGSIZE		(CONFIG_SYS_CBSIZE)

#define CONFIG_SYS_MEMTEST_START	(OMAP34XX_SDRC_CS0 + 0x07000000)
#define CONFIG_SYS_MEMTEST_END		(CONFIG_SYS_MEMTEST_START + \
					0x01000000) /* 16MB */

#define CONFIG_SYS_LOAD_ADDR		(OMAP34XX_SDRC_CS0 + 0x02000000)

/*
 * OMAP3 has 12 GP timers, they can be driven by the system clock
 * (12/13/16.8/19.2/38.4MHz) or by 32KHz clock. We use 13MHz (V_SCLK).
 * This rate is divided by a local divisor.
 */
#define CONFIG_SYS_TIMERBASE		(OMAP34XX_GPT2)
#define CONFIG_SYS_PTV			2 /* Divisor: 2^(PTV+1) => 8 */
#define CONFIG_SYS_HZ			1000

/* The stack sizes are set up in start.S using the settings below */
#define CONFIG_STACKSIZE	(128 << 10)	/* regular stack 128 KiB */
#ifdef CONFIG_USE_IRQ
#define CONFIG_STACKSIZE_IRQ		(4 << 10)	/* IRQ stack 4 KiB */
#define CONFIG_STACKSIZE_FIQ		(4 << 10)	/* FIQ stack 4 KiB */
#endif

/*  Physical Memory Map  */
#define CONFIG_NR_DRAM_BANKS		2 /* CS1 may or may not be populated */
#define PHYS_SDRAM_1			OMAP34XX_SDRC_CS0
#define PHYS_SDRAM_1_SIZE		(128 << 20)	/* at least 128 MiB */
#define PHYS_SDRAM_2			OMAP34XX_SDRC_CS1

/* SDRAM Bank Allocation method */
#define SDRC_R_B_C			1

/* NAND and environment organization  */
#define PISMO1_NAND_SIZE		GPMC_SIZE_128M

#define CONFIG_SYS_MONITOR_LEN		(256 << 10)	/* Reserve 2 sectors */

#define CONFIG_ENV_IS_IN_NAND		1
#define SMNAND_ENV_OFFSET		0x260000 /* environment starts here */

#define CONFIG_ENV_OFFSET		SMNAND_ENV_OFFSET

#define CONFIG_SYS_SDRAM_BASE          PHYS_SDRAM_1
#define CONFIG_SYS_INIT_RAM_ADDR        0x4020f800
#define CONFIG_SYS_INIT_RAM_SIZE        0x800
#define CONFIG_SYS_INIT_SP_ADDR         (CONFIG_SYS_INIT_RAM_ADDR + \
		                                         CONFIG_SYS_INIT_RAM_SIZE - \
		                                         GENERATED_GBL_DATA_SIZE)

#endif /* __CONFIG_H */
