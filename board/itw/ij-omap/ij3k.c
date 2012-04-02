/*
 * (C) Copyright 2004-2008
 * Texas Instruments, <www.ti.com>
 *
 * Author :
 *	Sunil Kumar <sunilsaini05@gmail.com>
 *	Shashi Ranjan <shashiranjanmca05@gmail.com>
 *
 * (C) Copyright 2009
 * Frederik Kriewitz <frederik@kriewitz.eu>
 *
 * Derived from Beagle Board and 3430 SDP code by
 *	Richard Woodruff <r-woodruff2@ti.com>
 *	Syed Mohammed Khasim <khasim@ti.com>
 *
 * Derived from Beagle Board and devkit8k 
 * (C) Copyright 2012
 * Craig Nauman <cnauman@diagraph.com>
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
#include <common.h>
#include <twl4030.h>
#include <asm/io.h>
#include <asm/arch/mux.h>
#include <asm/arch/sys_proto.h>
#include <asm/arch/mem.h>
#include <asm/arch/mmc_host_def.h>
#include <asm/mach-types.h>
#include <mmc.h>
#include <asm/arch/dss.h>
#include <environment.h>
#include <asm/gpio.h>

#include <video_fb.h> /* for video_hw_init */

#include "ij3k.h"

#define pr_debug(fmt, args...) debug(fmt, ##args)

#ifdef CONFIG_DRIVER_DM9000
#include <net.h>
#include <netdev.h>
#endif

DECLARE_GLOBAL_DATA_PTR;

/*
 * Routine: board_init
 * Description: Early hardware init.
 */
int board_init(void)
{
	gpmc_init(); /* in SRAM or SDRAM, finish GPMC */
	/* board id for Linux */
	gd->bd->bi_arch_number = MACH_TYPE_OMAP3_IJ3K;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

        gd->flags |= GD_FLG_SILENT;
#ifdef CONFIG_VIDEO
        gd->fb_base = LCD_VIDEO_ADDR;
#endif

	return 0;
}

/************************************************
 * get_sysboot_value(void) - return SYS_BOOT[4:0]
 ************************************************/
u32 get_sysboot_value(void)
{
	static struct ctrl *ctrl_base = (struct ctrl *)OMAP34XX_CTRL_BASE;
        int mode;
        mode = (readl(&ctrl_base->status) >> 5) & 1;
        return mode;
}

static int con_override = 0;
void CheckMMC(void) {
    char buf[8];
    if (get_sysboot_value()) {
        set_default_env("## Resetting to the default environ\n");
	setenv("mmcdev", "0");
	setenv("loadbootenv", "fatload mmc ${mmcdev} ${loadaddr} uEnv.txt");
	setenv("importbootenv", "echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize");
        setenv("bootcmd", "if mmc rescan; then " \
			      "if run loadbootenv; then " \
			          "run importbootenv; " \
			      "fi;" \
                              "if test -n $uenvcmd; then " \
			          "run uenvcmd; " \
		              "fi;" \
                          "fi;");
        con_override |= 1;
    }
    sprintf(buf, "%01d", con_override);
    setenv("btn", buf);
}

#ifdef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
int overwrite_console (void) {
    int gpio_pins[] = {107, 108,/* 109, 110,*/ 0}, i, ret = 0;

    con_override = 0;
    for (i=0; gpio_pins[i]; i++) {
	gpio_request(gpio_pins[i], "");
	gpio_direction_input(gpio_pins[i]);
	con_override |= (~gpio_get_value(gpio_pins[i]) & 1);
        con_override <<= 1;
        gpio_free(gpio_pins[i]);
    }
    ret = ((con_override >> 1) & 1); 

//    if (ret)
        gd->flags &= ~GD_FLG_SILENT; // undo the silent treatment
    return ret;
}
#endif

#ifdef CONFIG_CMD_I2C
void i2c_init_r(void) {
	unsigned char byte;
#ifdef CONFIG_DRIVER_OMAP34XX_I2C
	i2c_init(CONFIG_SYS_I2C_SPEED, CONFIG_SYS_I2C_SLAVE);
#endif
	byte = 0x20;
	i2c_write(0x4B, 0x7A, 1, &byte, 1);
	byte = 0x03;
	i2c_write(0x4B, 0x7D, 1, &byte, 1);
	byte = 0xE0;
	i2c_write(0x4B, 0x8E, 1, &byte, 1);
	byte = 0x05;
	i2c_write(0x4B, 0x91, 1, &byte, 1);
	byte = 0x20;
	i2c_write(0x4B, 0x96, 1, &byte, 1);
	byte = 0x03;
	i2c_write(0x4B, 0x99, 1, &byte, 1);
	byte = 0x33;
	i2c_write(0x4A, 0xEE, 1, &byte, 1);

        *((uint *) 0x49058034) = 0xFFFFFAF9;
        *((uint *) 0x49056034) = 0x0F9F0FFF;
        *((uint *) 0x49058094) = 0x00000506;
        *((uint *) 0x49056094) = 0xF060F000;
}
#endif

#ifdef CONFIG_VIDEO
#if 0
struct omap_panel lcd =
{	/* Sharp LQ043T1DG01 4.3" Display */
	.acbi = 0,
	.acb = 0,
	.is_tft = 1,
	.config = OMAP_DSS_LCD_TFT | OMAP_DSS_LCD_IVS | OMAP_DSS_LCD_IHS /*| OMAP_DSS_LCD_IEO*/,
	.timings = {
		.x_res = 800, //480,
		.y_res = 480, //272,

		.pixel_clock	= 16500, //9000,

		.hsw		= 128, //42,
		.hfp		= 10, //3,
		.hbp		= 10, //2,

		/* Note: The vertical timings were copied from the Linux sources,
		 *	but they appear to be incorrect.  The first line of data
		 *	is not shown on the display.  Maybe I have a newer LCD?
		 */
		.vsw		= 2, //11,
		.vfp		= 4, //3,
		.vbp		= 11, //2,
	},

	.data_lines = 16,

	.fb_address1 = LCD_VIDEO_ADDR, //0x805CB000,
	.fb_address2 = LCD_VIDEO_ADDR, //0x805CB000,
};
#endif
#define DVI_BEAGLE_ORANGE_COL          0x00FF8000
static const struct panel_config lcd_cfg_ij = {
	.timing_h	= 0x1a4024c9, /* Horizantal timing */
	.timing_v	= 0x02c00509, /* Vertical timing */
	.pol_freq	= 0x00007028, /* Pol Freq */
	.divisor	= 0x00010001, /* 96MHz Pixel Clock */
//	.lcd_size	= 0x02ff03ff, /* 1024x768 */
	.lcd_size	= 0x01df031f, /* 800x480 */
	.panel_type	= 0x01, /* TFT */
	.data_lines	= 0x03, /* 24 Bit RGB */
	.load_mode	= 0x02, /* Frame Mode */
	.panel_color	= DVI_BEAGLE_ORANGE_COL /* ORANGE */
};
#endif

#ifdef CONFIG_VIDEO
void vidmem_clear(int width, int height, int bytesPP) {
    int sze = width * height * bytesPP;
    void * addr = (void *)gd->fb_base;

    memset(addr, 0, sze);
}

void init_smps(void) {
#ifdef CONFIG_CMD_I2C
#define SMARTREFLEX_ENABLE     (1<<3)
    twl4030_i2c_write_u8(TWL4030_CHIP_PM_MASTER, SMARTREFLEX_ENABLE, 
            TWL4030_PM_MASTER_DCDC_GLOBAL_CFG);

    *((uint *) 0x4830722c) = 0x30201e00;
    *((uint *) 0x48307234) = 0x00120000;
    *((uint *) 0x48307238) = 0x00000008;
    *((uint *) 0x48307298) = 0x000000ff;
    *((uint *) 0x48307294) = 0x000000ff;
    *((uint *) 0x483072a0) = 0x000000ff;
    *((uint *) 0x48307220) = 0x00120012;
    *((uint *) 0x48307224) = 0x00010000;
    *((uint *) 0x48307290) = 0x0fff0fff;
    *((uint *) 0x48307230) = 0x2c000000;
    *((uint *) 0x483072d0) = 0x00002c06;
#endif
}
#endif

                          // 5.7" ES nothing selected
#define LCD_10_4_XLS (89) // 10.4" XLS
#define LCD_7_0_HH   (71) // 7" handheld
#define LCD_7_0_ES   (72) // 7" ES+
#define LCD_10_2_XLS (73) // 10.2" controller
#define LCD_PWREN    (164)
int get_vidtype(void) {
    int pin, val = 0, i = 0, poffset, pval; //, offset;
    int dss_pins[] = {1, 2, 3, 19, 0};
    char buf[4];

    gpio_request(LCD_PWREN, "");
    gpio_direction_output(LCD_PWREN,0);
    gpio_set_value(LCD_PWREN,0);
    udelay(1000);
    for (i = 0; dss_pins[i]; i++) { 
        poffset = CP(DSS_DATA0) + dss_pins[i]*2;
        pin = 70 + dss_pins[i];
        MUX_VAL(poffset, (IEN | PTU | EN | M4));
        pval = gpio_get_value(pin);
	val |= ((pval & 1) << i);
        MUX_VAL(poffset, (IDIS | PTD | DIS | M0));
    }
    val = (~val) & 0xf;
    gpio_free(LCD_PWREN);

    sprintf(buf, "%02d", val);
    setenv("disp", buf);
    return val;
}

#define SYSLED2 (186)
void init_leds(void) {
    int led_lst[] = {163, 164, SYSLED2, 0}, i = 0;
    for (i = 0; led_lst[i]; i++) {
	gpio_request(led_lst[i], "");
	gpio_direction_output(led_lst[i],0);
	gpio_set_value(led_lst[i],1);
    }
    gpio_set_value(SYSLED2, 0);
}
extern void        read4030Var(int);
/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{
#if defined(CONFIG_DRIVER_DM9000) && defined(CONFIG_DM9000_NO_SROM)
	struct ctrl_id *id_base = (struct ctrl_id *)OMAP34XX_ID_L4_IO_BASE;
	uchar enetaddr[6];
	u32 die_id_0;
#endif
#ifdef CONFIG_TWL4030_POWER
	twl4030_power_init();
#endif
#ifdef CONFIG_TWL4030_LED
	twl4030_led_init(0/*TWL4030_LED_LEDEN_LEDAON | TWL4030_LED_LEDEN_LEDBON*/);
#endif
#ifdef CONFIG_VIDEO
        init_smps();
/*
 * Configure DSS to display background color on DVID
 * Configure VENC to display color bar on S-Video
 */
/*	lcd.logo_width = 800; //width;
	lcd.logo_height = 480; //height;
        omapdss_init(&lcd);*/
	//omap3_dss_venc_config(&venc_config_std_tv, VENC_HEIGHT, VENC_WIDTH);
//	omap3_dss_panel_config(&lcd_cfg_ij);
#endif

#ifdef CONFIG_DRIVER_DM9000
	/* Configure GPMC registers for DM9000 */
	writel(NET_GPMC_CONFIG1, &gpmc_cfg->cs[6].config1);
	writel(NET_GPMC_CONFIG2, &gpmc_cfg->cs[6].config2);
	writel(NET_GPMC_CONFIG3, &gpmc_cfg->cs[6].config3);
	writel(NET_GPMC_CONFIG4, &gpmc_cfg->cs[6].config4);
	writel(NET_GPMC_CONFIG5, &gpmc_cfg->cs[6].config5);
	writel(NET_GPMC_CONFIG6, &gpmc_cfg->cs[6].config6);
	writel(NET_GPMC_CONFIG7, &gpmc_cfg->cs[6].config7);

#ifdef CONFIG_DM9000_NO_SROM
	/* Use OMAP DIE_ID as MAC address */
//	if (!eth_getenv_enetaddr("ethaddr", enetaddr)) {
//		printf("ethaddr not set, using Die ID\n");
		die_id_0 = readl(&id_base->die_id_0);
		enetaddr[0] = 0x00; /* locally administered */
		enetaddr[1] = 0x06; //readl(&id_base->die_id_1) & 0xff;
		enetaddr[2] = 0xb3; //(die_id_0 & 0xff000000) >> 24;
		enetaddr[3] = (die_id_0 & 0x00ff0000) >> 16;
		enetaddr[4] = (die_id_0 & 0x0000ff00) >> 8;
		enetaddr[5] = (die_id_0 & 0x000000ff);
		eth_setenv_enetaddr("ethaddr", enetaddr);
//	}
#endif /* CONFIG_DM9000_NO_SROM */
#endif

//	dieid_num_r();
        CheckMMC();
#ifdef CONFIG_VIDEO
//        set_vidtype();
#endif
        init_leds();
	return 0;
}

/*
 * Routine: set_muxconf_regs
 * Description: Setting up the configuration Mux registers specific to the
 *		hardware. Many pins need to be moved from protect to primary
 *		mode.
 */
void set_muxconf_regs(void)
{
	MUX_DEVKIT8000();
}

#ifdef CONFIG_GENERIC_MMC
int board_mmc_init(bd_t *bis)
{
       omap_mmc_init(0);
       return 0;
}
#endif

#ifdef CONFIG_DRIVER_DM9000
/*
 * Routine: board_eth_init
 * Description: Setting up the Ethernet hardware.
 */
int board_eth_init(bd_t *bis)
{
	int ret;
        ret = dm9000_initialize(bis);
#ifndef CONFIG_DM9000_NO_SROM
        {
        	struct eth_device *dev = eth_get_dev();
                eth_setenv_enetaddr("ethaddr", dev->enetaddr);
        }
#endif
	return ret;
}
#endif

#ifdef CONFIG_VIDEO
void go_omap3_dss_enable(int x) {
	/* Request and Activate Panel Logic Power Supply Pin */
	gpio_request(176, "");
	gpio_direction_output(176,0);
	gpio_set_value(176,1);
}
#endif
#ifdef CONFIG_CFB_CONSOLE
GraphicDevice smi;
#define TIMING(sw,fp,bp) ( \
        ((bp & 0xfff) << 20) | \
        ((fp & 0xfff) << 8) | \
        (sw & 0xff)\
        )
#define DIV(lcd, pcd) (((lcd & 0xff) << 16) | (pcd & 0xff))
#define SIZE(x, y) ((y-1) << 16 | (x-1))

enum omap_panel_config {
	OMAP_DSS_LCD_IVS		= 1<<0,
	OMAP_DSS_LCD_IHS		= 1<<1,
	OMAP_DSS_LCD_IPC		= 1<<2,
	OMAP_DSS_LCD_IEO		= 1<<3,
	OMAP_DSS_LCD_RF			= 1<<4,
	OMAP_DSS_LCD_ONOFF		= 1<<5,

	OMAP_DSS_LCD_TFT		= 1<<20,
};

static unsigned int omapdss_set_pol_freq(enum omap_panel_config config, unsigned char acbi, unsigned char acb)
{
	unsigned int l = 0;

	l |= (config & OMAP_DSS_LCD_ONOFF) ? (1<<17) : 0;
	l |= (config & OMAP_DSS_LCD_RF) ? (1<<16) : 0;
	l |= (config & OMAP_DSS_LCD_IEO) ? (1<<15) : 0;
	l |= (config & OMAP_DSS_LCD_IPC) ? (1<<14) : 0;
	l |= (config & OMAP_DSS_LCD_IHS) ? (1<<13) : 0;
	l |= (config & OMAP_DSS_LCD_IVS) ? (1<<12) : 0;
	l |= ((acbi & 0x0F) << 8);
	l |= acb;

        return l;
}

void * video_hw_init(void) {
    //register GraphicDevice *pGD = (GraphicDevice *)&smi;
    struct panel_config pcfg;
    int vidtype = get_vidtype();
    int hsw=0, hfp=0, hbp=0;
    int vsw=0, vfp=0, vbp=0;
    int lclk = 1, pclk=2, pol_flags = 0;
    int acbi = 0, acb = 0;
 
    pol_flags = OMAP_DSS_LCD_ONOFF | OMAP_DSS_LCD_RF;
    if (vidtype) {
        smi.winSizeX = smi.plnSizeX = 800;
        smi.winSizeY = smi.plnSizeY = 480;
        if (8 == vidtype) smi.winSizeX = smi.plnSizeX = 640;
        smi.gdfBytesPP = 2;
        smi.gdfIndex = GDF_16BIT_565RGB;
        hsw = 0x4f; hfp = 0x03f; hbp = 0x08f;
        vsw = 0x04; vfp = 0x01c; vbp = 0x048;
//        pcfg.timing_h   = 0x08f03f4f;
//        pcfg.timing_v   = 0x04801c04;
        pcfg.panel_type = 1;
    } else {
        smi.winSizeX = smi.plnSizeX = 320;
        smi.winSizeY = smi.plnSizeY = 240;
        smi.gdfBytesPP = 1;
        pcfg.panel_type = 0;
        pol_flags = 0;
        smi.gdfIndex = GDF__8BIT_332RGB;
        /*hsw = 0x00; hfp = 0x001; hbp = 0x001;
        vsw = 0x01; vfp = 0x001; vbp = 0x001;
        lclk = 5; pclk = 8;*/
        lclk = 4; // 48 Mhz original
        pclk = 10;
        /* something at the top of the screen
        hsw = 0x01; hfp = 0x002; hbp = 0x004;
        vsw = 0x01; vfp = 0x000; vbp = 0x000;*/
        /*hsw = 0x02; hfp = 0x004; hbp = 0x008;
        vsw = 0x02; vfp = 0x001; vbp = 0x001;*/
        hsw = 0x08; hfp = 0x00f; hbp = 0x00f;
        vsw = 0x08; vfp = 0x004; vbp = 0x004;
        pclk = 25;
        acbi = 0/*1*/; acb = 0; //1; 
    }
    pcfg.timing_h = TIMING(hsw, hfp, hbp);
    pcfg.timing_v = TIMING(vsw, vfp, vbp);
    pcfg.data_lines = 3;
    pcfg.lcd_size   = SIZE(smi.winSizeX, smi.winSizeY);
    pcfg.load_mode  = 0; //0x00000204 >> FRAME_MODE_SHIFT;
    smi.frameAdrs = LCD_VIDEO_ADDR;
    smi.memSize = smi.winSizeX * smi.winSizeY * smi.gdfBytesPP;
    pcfg.panel_color= 0;
    pcfg.divisor    = DIV(lclk, pclk);
    pcfg.pol_freq   = omapdss_set_pol_freq(pol_flags, acbi, acb); // 0, 0); //0x30000;
    vidmem_clear(smi.winSizeX, smi.winSizeY, smi.gdfBytesPP);
    //printf("Done w/ lcd init\n");
    omap3_dss_panel_config(&pcfg);
    omap3_dss_enable();
    return ((void*)&smi);
}
#endif
static int do_lookup(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
    char buf[32], *pc, *modedb, *end;
    int len=0;

	if (argc < 2)
		return 0; //cmd_usage(cmdtp);
    /* now extract value from modedb env var */
    sprintf(buf, "%s(", argv[1]);

    modedb = getenv(argv[2]);
    if (modedb) {
        pc = strstr(modedb, buf);
        if (pc) {
            pc += strlen(buf); /* skip over value and : */
            end = strchr(pc, ')');
            if (end) len = end - pc;
            else len = strlen(pc);
            if (len < sizeof(buf)) {
                strncpy(buf, pc, len);
                buf[len] = '\0';
                setenv(argv[3], buf);
            }
        }
    }
    return 1;
}

U_BOOT_CMD(
	find,4,4,do_lookup,
	"",
	""
);

typedef struct Items {
    char * name;
    unsigned long addr;
};

void dump_vid_regs(void) {
    struct Items item[] = {
        {"clken",   0x48004d00},
        {"fclken",  0x48004e00},
        {"iclken",  0x48004e10},
        {"idlest",  0x48004e20},
        {"autoidle",0x48004e30},
        {"clksel",  0x48004e40},

        {"sys-cfg", 0x48050010},
        {"dss-ctl", 0x48050040},
        {"dss-clk", 0x4805005c},
        {"control", 0x48050440},
        {"config",  0x48050444},
        {"timing_h",0x48050464},
        {"timing_v",0x48050468},
        {"pol_freq",0x4805046c},
        {"divisor", 0x48050470},
        {"lcd-size",0x4805047c},
        {"gfx-size",0x4805048c},
        {"gfx-attr",0x480504a0},
        {0, 0},
    };
    int i;

            for (i=0; 0 != item[i].name; i++) {
                printf("%8s:(0x%08lx)0x%08lx\n", item[i].name, 
                            item[i].addr, *((unsigned long *)item[i].addr));
            }
}

int do_tst (cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]) {
	if (argc < 2)
		return 0; //cmd_usage(cmdtp);

	if (strcmp (argv[1],"reg") == 0) {
            dump_vid_regs();
        } else {
		return 0; //cmd_usage(cmdtp);
        }
        return 1;
}

U_BOOT_CMD(
	tst,4,1,do_tst,
	"",
	""
);
