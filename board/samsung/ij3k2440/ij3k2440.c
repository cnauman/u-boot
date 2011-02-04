/*
 * (C) Copyright 2002
 * Sysgo Real-Time Solutions, GmbH <www.elinos.com>
 * Marius Groeger <mgroeger@sysgo.de>
 *
 * (C) Copyright 2002
 * David Mueller, ELSOFT AG, <d.mueller@elsoft.ch>
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
#include <netdev.h>
#include <asm/arch/s3c24x0_cpu.h>
#include <video_fb.h>

DECLARE_GLOBAL_DATA_PTR;

/* FCLK = 405 MHz, HCLK = 101 MHz, PCLK = 50 MHz, UCLK = 48 MHz */
#define CLKDIVN_VAL	5
#define M_MDIV		0x7f
#define M_PDIV		0x2
#define M_SDIV		0x1

#define U_M_MDIV	0x38
#define U_M_PDIV	0x2
#define U_M_SDIV	0x2

static inline void delay (unsigned long loops)
{
	__asm__ volatile ("1:\n"
	  "subs %0, %1, #1\n"
	  "bne 1b":"=r" (loops):"0" (loops));
}

/*
 * Miscellaneous platform dependent initialisations
 */
#define GPIO_DISP_MASK ((1 << 11) | (1 << 10) | (1 << 9) | (1 << 8))
static int display_id = 0, con_override=0;

int board_early_init_f(void) {
	struct s3c24x0_clock_power * const clk_power =
					s3c24x0_get_base_clock_power();
	/* to reduce PLL lock time, adjust the LOCKTIME register */
	clk_power->locktime = 0xFFFFFF;
	clk_power->clkdivn = CLKDIVN_VAL;

	/* configure UPLL */
	clk_power->upllcon = ((U_M_MDIV << 12) + (U_M_PDIV << 4) + U_M_SDIV);
	/* some delay between MPLL and UPLL */
	delay (10);
	/* configure MPLL */
	clk_power->mpllcon = ((M_MDIV << 12) + (M_PDIV << 4) + M_SDIV);

	/* some delay between MPLL and UPLL */
	delay (8000);

	return 0;
}

#define MACH_TYPE_MINI2440 1999
int board_init (void)
{
	struct s3c24x0_gpio * const gpio = s3c24x0_get_base_gpio();
#ifndef CONFIG_BOARD_EARLY_INIT_F
	board_early_init_f();
#endif

        /* get id from display board */
        gpio->gpccon = GPIO_DISP_MASK;
        gpio->gpcup = GPIO_DISP_MASK;
        display_id = (gpio->gpcdat & GPIO_DISP_MASK) >> 8;

        /* console override buttons use the lighter/darker */
        /* lighter => gpj11, darker => gpj10 */
        gpio->gpjcon |= ((1 << 11) | (1 << 10));
        gpio->gpjup |= ((1 << 11) | (1 << 10));
        con_override = ~(gpio->gpjdat >> 10) & 0x3;

        gpio->gpdcon = GPIO_DISP_MASK;
        gpio->gpdup = GPIO_DISP_MASK;
        display_id |= (gpio->gpddat & GPIO_DISP_MASK) >> 4;
        display_id = (~display_id & 0xff);
        /* done with display type */

	gpio->gpacon = 0x007FFFFF;	/* Port A is all "special" */
	/* port B outputs reconfigured */
	gpio->gpbcon = 	
		(0x1 <<  0) | // GPB0	OUT	TOUT0		PWM Buzzer
		(0x2 <<  2) | // GPB1	OUT	TOUT1		LCD Backlight
		(0x1 <<  4) | // GPB2	OUT	L3MODE
		(0x1 <<  6) | // GBP3	OUT	L3DATA
		(0x1 <<  8) | // GBP4	OUT	L3CLOCK
		(0x1 << 10) | // GBP5	OUT	LED1
		(0x1 << 12) | // GBP6	OUT	LED2
		(0x1 << 14) | // GBP7	OUT	LED3
		(0x1 << 16) | // GBP8	OUT	LED4
		(0x2 << 18) | // GBP9	---	nXDACK0		CON5 EBI
		(0x2 << 20) | // GBP10	---	nXDREQ0		CON5 EBI
		0;
	gpio->gpbup	= (1 << 10) - 1; // disable pullup on all 10 pins
	gpio->gpbdat	= 	
		(0 << 5) | /* turn LED 1 on */
		(1 << 6) | /* turn LED 1 off */
		(1 << 7) | /* turn LED 1 off */
		(1 << 8) | /* turn LED 1 off */
		0;

	/* lcd signals on C and D */
	gpio->gpccon	= (0xAAAAAAAA &	/* all default IN but ... */
				~(0x3 << 10)) |	/* not pin 5 ... */
				(0x1 << 10);	/* that is output (USBD) */
	gpio->gpcup	= 0xffffffff;
	gpio->gpcdat	= 0;
	
	gpio->gpdcon = 0xAAAAAAAA;
	gpio->gpdup = 0xFFFFFFFF;
	gpio->gpecon = 0xAAAAAAAA;
	gpio->gpeup = 0x0000FFFF;
	gpio->gpfcon 	= 
		(0x1 <<  0) | // GPG0	EINT0	OUT
		(0x1 <<  2) | // GPG1	EINT1	OUT
		(0x1 <<  4) | // GPG2	EINT2	OUT
		(0x1 <<  6) | // GPG3	EINT3	OUT
		(0x1 <<  8) | // GPG4	EINT4	OUT
		(0x1 << 10) | // GPG5	EINT5	OUT
		(0x1 << 12) | // GPG6	EINT6	OUT
		(0x0 << 14) | // GPG7	EINT7	IN	DM9000
		0;
	gpio->gpfdat	= 0;
	gpio->gpfup	= 
		((1 << 7) - 1) // all disabled
		& ~( 1 << 7 ) // but for the ethernet one, we need it.
		;

	gpio->gpgcon 	=
		(0x0 <<  0) | // GPG0	EINT8	IN	Key1
		(0x1 <<  2) | // GPG1	EINT9	OUT		Con5
		(0x1 <<  4) | // GPG2	EINT10	OUT
		(0x0 <<  6) | // GPG3	EINT11	IN	Key2
		(0x0 <<  8) | // GPG4	EINT12	IN	Smart Screen Interrupt
		(0x0 << 10) | // GPG5	EINT13	IN	Key3
		(0x0 << 12) | // GPG6	EINT14	IN	Key4
		(0x0 << 14) | // GPG7	EINT15	IN	Key5
		(0x1 << 16) | // GPG8	EINT16	OUT	nCD_SD
		(0x1 << 18) | // GPG9	EINT17	OUT
		(0x1 << 20) | // GPG10	EINT18	OUT
		(0x0 << 22) | // GPG11	EINT19	IN	Key6
		(0x0 << 24) | // GPG12	EINT18	IN	// GPG[12..15] need to be inputs
		(0x0 << 26) | // GPG13	EINT18	IN	// hard pullups
		(0x0 << 28) | // GPG14	EINT18	IN
		(0x0 << 30) | // GPG15	EINT18	IN
		0;
	gpio->gpgup = (1 << 15) -1;	// disable pullups for all pins
	
	gpio->gphcon = 
		(0x2 <<  0) | // GPH0	nCTS0			---
		(0x2 <<  2) | // GPH1	nRTS0			---
		(0x2 <<  4) | // GPH2	TXD0			---
		(0x2 <<  6) | // GPH3	RXD0			---
		(0x2 <<  8) | // GPH4	TXD1			---
		(0x2 << 10) | // GPH5	RXD1			---
		(0x2 << 12) | // GPH6	[TXD2]	nRTS1
		(0x2 << 14) | // GPH7	[RXD2]	nCTS1
		(0x1 << 16) | // GPH8	UEXTCLK			OUT
		(0x1 << 18) | // GPH9	CLKOUT0			OUT
		(0x1 << 20) | // GPH10	CLKOUT1			OUT
		0;
	gpio->gphup = (1 << 10) - 1; // disable pullups for all pins

	gpio->extint0=0x22222222;
	gpio->extint1=0x22222222;
	gpio->extint2=0x22222222;

	/* USB Device Part */
	/* GPC5 is reset for USB Device */

	gpio->gpcdat |= ( 1 << 5) ; 
	udelay(20000);
	gpio->gpcdat &= ~( 1 << 5) ; 
	udelay(20000);
	gpio->gpcdat |= ( 1 << 5) ; 

	/* temporary arch number of IJ3K2440-Board */
	gd->bd->bi_arch_number = MACH_TYPE_MINI2440;

	/* adress of boot parameters */
	gd->bd->bi_boot_params = 0x30000100;

	icache_enable();
	dcache_enable();

        /* set environ var for display type*/

	return 0;
}

void board_video_init(GraphicDevice *pGD) 
{ 
    struct s3c24x0_lcd * const lcd = s3c24x0_get_base_lcd(); 
	 
    /* FIXME: select LCM type by env variable */ 
	/* Configuration for GTA01 LCM on QT2410 */ 
	
	/*lcd->lcdcon5 = 0x00000f09; */

	lcd->lpcsel  = 0x00000000; 
} 

/* TODO: I believe this can just use the built-in version */
int dram_init (void)
{
//	struct s3c24x0_memctl * const mem = s3c24x0_get_base_memctl();
    
//	gd->bd->bi_dram[0].size = PHYS_SDRAM_1_SIZE;
	/* if the early bootloader found 128MB, lets tell the kernel */
//	if ((mem->BANKCON[6] & 0x3) == 0x2)
//		gd->bd->bi_dram[0].size = 128*1024*1024;
//	else
	gd->ram_size = get_ram_size((long *)PHYS_SDRAM_1, 64*1024*1024);
        return 0;
}

void dram_init_banksize(void)
{
	gd->bd->bi_dram[0].start = PHYS_SDRAM_1;
		gd->bd->bi_dram[0].size = 64*1024*1024;
}

#ifdef CONFIG_DRIVER_DM9000
int board_eth_init(bd_t *bis)
{
	return dm9000_initialize(bis);
}
#endif

int misc_init_r (void)
{
#ifdef CONFIG_AUTO_UPDATE
	{
		extern int do_auto_update(void);
		/* this has priority over all else */
                if (2 == con_override) do_auto_update();
	}
#endif
	return (0);
}

#ifdef CONFIG_SERIAL_MULTI
#include <serial.h>

/*struct serial_device *default_serial_console(void) {
	struct serial_device * uart = &s3c24xx_serial0_device;
	if (1 & (3 & con_override)) uart = &s3c24xx_serial1_device;
	return uart;
}*/
#endif

#ifdef CONFIG_SYS_CONSOLE_OVERWRITE_ROUTINE
int overwrite_console (void) {
    //if (1 == con_override) {
        if (0 == serial_assign(s3c24xx_serial1_device.name)) {
            blue_LED_on();
            serial_init();
        }
    //}
    return con_override & 1; 
}
#endif

int     drv_video_init (void);
int do_testcmd(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]) {
    if (1 < argc) {
        int flag = *argv[1] - '0';
        if (flag) drv_video_init ();
    }
    return 0;
}

U_BOOT_CMD(
	tstcmd, 2, 1, do_testcmd,
	"simple test cmd", "Usage: tstcmd value"
);

int do_vid(cmd_tbl_t * cmdtp, int flag, int argc, char * const argv[]) {
        char buf[12];
        if (1 < argc) {
            if (0 == strcmp(argv[1], "set")) {
                sprintf(buf, "vid_%02x", display_id);
                setenv("videomode", buf);
            } else if (0 == strcmp(argv[1], "reg")) {
                struct s3c24x0_lcd * const lcd = s3c24x0_get_base_lcd(); 
	        printf("lcdcon1: 0x%08x\n", lcd->lcdcon1);
        	printf("lcdcon2: 0x%08x\n", lcd->lcdcon2);
	        printf("lcdcon3: 0x%08x\n", lcd->lcdcon3);
        	printf("lcdcon4: 0x%08x\n", lcd->lcdcon4);
	        printf("lcdcon5: 0x%08x\n", lcd->lcdcon5);
//            } else {
//                serial_assign(argv[1]);
            }
        }
        return 0;
}
U_BOOT_CMD(
	vid, 2, 1, do_vid,
	"set the videomode variable", "Usage: vid reg|set"
);

void LED_on(int led) {
	struct s3c24x0_gpio * const gpio = s3c24x0_get_base_gpio();
    	gpio->gpbcon = 	
		(0x1 << 10) | // GBP5	OUT	LED1
		(0x1 << 12) | // GBP6	OUT	LED2
		(0x1 << 14) | // GBP7	OUT	LED3
		(0x1 << 16) | // GBP8	OUT	LED4
		0;
	gpio->gpbup	= (1 << 10) - 1; // disable pullup on all 10 pins
	gpio->gpbdat	= (~led & 0xf) << 5;
}
void yellow_LED_on(void) { LED_on(6); }
void green_LED_on(void) { LED_on(7); }
void blue_LED_on(void) { LED_on(8); }
void red_LED_off(void) { LED_on(0xf); }
