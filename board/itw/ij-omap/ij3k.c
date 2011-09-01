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
#include <asm/mach-types.h>
#include <mmc.h>
#include <asm/arch/omapdss.h>
#include <environment.h>
#include <asm/arch/gpio.h>

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
	gd->bd->bi_arch_number = MACH_TYPE_DEVKIT8000;
	/* boot param addr */
	gd->bd->bi_boot_params = (OMAP34XX_SDRC_CS0 + 0x100);

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

#define OVERRIDE_KEY (26)
static int con_override = 0;
void CheckMMC(void) {
    char buf[8];

	omap_request_gpio(OVERRIDE_KEY);
	omap_set_gpio_direction(OVERRIDE_KEY, 1);
	con_override = (~omap_get_gpio_datain(OVERRIDE_KEY) << 1) & 0x2;
    if (get_sysboot_value()) {
        set_default_env("## Resetting to the default environ\n");
	setenv("mmcdev", "1");
	setenv("loadbootenv", "fatload mmc ${mmcdev} ${loadaddr} uEnv.txt");
	setenv("importbootenv", "echo Importing environment from mmc ...; " \
		"env import -t $loadaddr $filesize");
        setenv("bootcmd", "if mmc init ${mmcdev}; then " \
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
extern void omap3_dss_enable(void);

#ifdef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
int overwrite_console (void) {
/*    if (1 == con_override) {
        if (0 == serial_assign(s3c24xx_serial1_device.name)) {
            blue_LED_on();
            serial_init();
        }
    }*/
    gd->flags &= ~GD_FLG_SILENT; // undo the silent treatment
    return (1 == con_override); 
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

#ifdef CONFIG_VIDEO
void vidmem_clear(void) {
    int i, sze = lcd.logo_width * lcd.logo_height * sizeof(short);
    for (i=0; i < sze; i+=2) {
        *((unsigned short*)(lcd.fb_address1 + i)) = 0x0000;
    }
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

/*
 * Routine: misc_init_r
 * Description: Configure board specific parts
 */
int misc_init_r(void)
{
	struct ctrl_id *id_base = (struct ctrl_id *)OMAP34XX_ID_L4_IO_BASE;

#ifdef CONFIG_DRIVER_DM9000
	uchar enetaddr[6];
	u32 die_id_0;
#endif
#ifdef CONFIG_TWL4030_POWER
	twl4030_power_init();
#endif
#ifdef CONFIG_TWL4030_LED
	twl4030_led_init(/*TWL4030_LED_LEDEN_LEDAON |*/ TWL4030_LED_LEDEN_LEDBON);
#endif
#ifdef CONFIG_VIDEO
        init_smps();
/*
 * Configure DSS to display background color on DVID
 * Configure VENC to display color bar on S-Video
 */
	lcd.logo_width = 800; //width;
	lcd.logo_height = 480; //height;
        omapdss_init(&lcd);
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

	/* Use OMAP DIE_ID as MAC address */
        // TODO: use eeprom instead
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
#endif

	dieid_num_r();

        CheckMMC();
#ifdef CONFIG_VIDEO
        vidmem_clear();
	omap3_dss_enable();
#endif
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

#ifdef CONFIG_DRIVER_DM9000
/*
 * Routine: board_eth_init
 * Description: Setting up the Ethernet hardware.
 */
int board_eth_init(bd_t *bis)
{
	return dm9000_initialize(bis);
}
#endif

extern void dumpregs(void);
void go_omap3_dss_enable(int x);
int do_tst(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]) {
        if (1 < argc) {
            if (0 == strcmp(argv[1], "set")) {
//                board_pre_video_init();
#ifdef CONFIG_VIDEO
                go_omap3_dss_enable(1); //*argv[2] - '0');
            } else if (0 == strcmp(argv[1], "down")) {
                writel(0, 0x48050440); // dispc->control
            } else if (0 == strcmp(argv[1], "smps")) {
                init_smps();
            } else if (0 == strcmp(argv[1], "clear")) {
                vidmem_clear();
#endif
            } else if (0 == strcmp(argv[1], "reg")) {
#define SHOWREG(a, b) { \
    printf("%8s:(0x%08x)0x%08x\n", a, b,  \
            (int)readl(((unsigned long)b))); \
}
    SHOWREG("gpio-oe", 0x48310034);
    SHOWREG("setdatout", 0x48310094);
    SHOWREG("clksel2_pll", 0x48004d44);
    SHOWREG("clkseldss", 0x48004e40);
    SHOWREG("clken", 0x48004d00);
    puts("\n");
    SHOWREG("FCLKEN", 0x48004e00);
    SHOWREG("ICLKEN", 0x48004e10);
    SHOWREG("IDLEST", 0x48004e20);
    SHOWREG("AutoIdle", 0x48004e30);
    SHOWREG("ClkSel", 0x48004e40);
    puts("\n");
    SHOWREG("timing_h", 0x48050464);
    SHOWREG("timing_v", 0x48050468);
    SHOWREG("pol_freq", 0x4805046c);
    SHOWREG("divisor", 0x48050470);
    SHOWREG("lcd_size", 0x4805047c);
    SHOWREG("config", 0x48050444);
    SHOWREG("control", 0x48050440);
    SHOWREG("defcolor", 0x4805044c);
    SHOWREG("sys-cfg", 0x48050010);
    SHOWREG("sys-stat", 0x48050014);
    SHOWREG("dss-ctl", 0x48050040);
    SHOWREG("dss-clk", 0x4805005c);
    SHOWREG("dc-addr0", 0x48050480);
    SHOWREG("dc-addr1", 0x48050484);

//            } else {
//                serial_assign(argv[1]);
            }
        }
        return 0;
}
U_BOOT_CMD(
        tst, 2, 1, do_tst,
        "test stuff", "Usage: tst reg|set|clear|smps|down"
);

//#ifdef CONFIG_DRIVER_OMAPDSS
//#ifdef CONFIG_DISPLAY_SHARP_LQ043T1DG01
//#include "logos/logo_480x272.h"

//#endif

//void omapdss_init_alt(struct omap_panel *lcd);
#ifdef CONFIG_VIDEO
void go_omap3_dss_enable(int x) {
	/* Setup LCD to display included Splash header */
//	lcd.logo = header_data;

	/* Request and Activate Backlight Power Supply */
//	omap_request_gpio(182);
//	omap_set_gpio_direction(182,0);
//	omap_set_gpio_dataout(182,1);

	/* Request and Deactivate Panel Backlight PWM Pin */
//	omap_request_gpio(181);
//	omap_set_gpio_direction(181,0);
//	omap_set_gpio_dataout(181,0);

	/* Request and Activate Panel Logic Power Supply Pin */
	omap_request_gpio(176);
	omap_set_gpio_direction(176,0);
	omap_set_gpio_dataout(176,1);

	/* Initialize the Display System */
        /*if (1==x)*/ omapdss_init(&lcd);
        //else if (2==x) omapdss_init_alt(&lcd);

	/* Activate the Backlight PWM Pin */
//	omap_set_gpio_dataout(181,1);
}
#endif

//#endif

