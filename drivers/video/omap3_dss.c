/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 * Syed Mohammed Khasim <khasim@ti.com>
 *
 * Referred to Linux DSS driver files for OMAP3
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 of
 * the License.
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
#include <asm/io.h>
#include <asm/arch/dss.h>

#include <video_fb.h> /* for video_hw_init */

#if 1
////////////////////////////////////////////////
#include <asm/arch/omapdss.h>
#include <common.h>

void omapdss_init(struct omap_panel *lcd)
{
	unsigned int i;
	unsigned char pixel[3];
	int offset = 0;
	char *logo = lcd->logo;
	int logo_size = lcd->logo_width * lcd->logo_height * 2;	// 16 bpp

	struct dss *dss_base = (struct dss*)DSS_BASE;
	struct dispc *dispc_base = (struct dispc*)DISPC_BASE;
	struct venc *venc_base = (struct venc*)VENC_BASE;

	/* Fill in data from provided header */
	/*for(i=0; i<logo_size; i+=2)
	{
                HEADER_PIXEL(logo, pixel);
		*((unsigned short*)(lcd->fb_address1 + i + offset)) = ((pixel[0] & 0xF8) << 8) |
						      ((pixel[1] & 0xFC) << 3) |
						      ((pixel[2] & 0xF8) >> 3);
	}*/

	// Setup LCD Interface IO and Clocks
	*((uint*) 0x48310034)		= 0xBFFFFFFF;	// GPIO1: GPIO_OE
	*((uint*) 0x48310094)		= 0x40000000;	// GPIO1: GPIO_SETDATAOUT
	*((uint*) 0x48004D44)		= 0x0001B00C;	// CM: Clock_Control_Reg_CM: CM_CLKSEL2_PLL
	*((uint*) 0x48004E40)		= 0x00001006;	// CM: DSS_CM: CM_CLKSEL_DSS
	*((uint*) 0x48004D00)		= 0x00370037;	// CM: Clock_Control_Reg_CM: CM_CLKEN_PLL

	udelay(1000);	// Wait a bit to let the DSS clocks stabilize and lock

   *((uint*)0x48050440)            &= ~0x3;	// dis. lcd & digit
   udelay(5000);
   *((uint*)0x48050410)            |= 0x00000002;	// reset dispc

	// Display Sub System (DSS) Initialization
	dss_base->sysconfig		= 0x00000001;	// Enable Auto-Idle
	dss_base->control		= 0x00000078;	// Settings for various clocks (was 0x00000078)
	dss_base->sdi_control		= 0x00000000;
	dss_base->pll_control		= 0x00000000;

	// Display Controller (DISPC) and Graphics Overlay (GFX) Initialization
	dispc_base->sysconfig		= 0x00002015;	// SMART_STANDBY | NO_IDLE | ENWAKEUP | AUTOIDLE
	dispc_base->config		= 0x00000004;	// LOADMODE = FRAME_ONLY
	dispc_base->default_color[0]	= 0x00000000;	// LCD Default Color (24-bit)
	dispc_base->default_color[1]	= 0x00000000;	// DIG Default Color (24-bit)
	dispc_base->trans_color[0]	= 0x00000000;
	dispc_base->trans_color[1]	= 0x00000000;

	// Set LCD Parameters (Polarity Bits, Size, Timings)
	omapdss_set_pol_freq(lcd->config, lcd->acbi, lcd->acb);
	omapdss_set_lcd_timings(&lcd->timings, lcd->is_tft);	// Set the panel timings and size

	dispc_base->gfx_ba[0]		= lcd->fb_address1;	// Frame buffer address
	dispc_base->gfx_ba[1]		= lcd->fb_address2;	// Double buffer address (same)
	dispc_base->gfx_position	= 0x00000000;		// GFX Window Position on Screen
	dispc_base->gfx_size		= dispc_base->size_lcd;	// Size of GFX Window (same as LCD Size)
	dispc_base->gfx_attributes	= 0x0000008d;		// GFXBURSTSIZE = 16x32bit | GFXFORMAT = RGB16 | GFX_ENABLE
	dispc_base->gfx_fifo_threshold	= 0x03ff03c0;		// Not sure how to customize this...
	dispc_base->gfx_row_inc		= 0x00000001;		// Don't skip lines
	dispc_base->gfx_pixel_inc	= 0x00000001;		// Don't skip pixels
	dispc_base->gfx_window_skip	= 0x00000000;		// Don't skip windows
	dispc_base->gfx_table_ba	= 0x00000000;		// Not using a graphics table

	// Enable the LCD Display
	udelay(1000);
	dispc_base->control		= 0x00018169;		// TFTDATALINES | GOLCD | TFT | LCDENABLE
	udelay(1000);
}

void omapdss_set_pol_freq(enum omap_panel_config config, unsigned char acbi, unsigned char acb)
{
	struct dispc *dispc_base = (struct dispc*)DISPC_BASE;
	unsigned int l = 0;

	l |= (config & OMAP_DSS_LCD_ONOFF) ? (1<<17) : 0;
	l |= (config & OMAP_DSS_LCD_RF) ? (1<<16) : 0;
	l |= (config & OMAP_DSS_LCD_IEO) ? (1<<15) : 0;
	l |= (config & OMAP_DSS_LCD_IPC) ? (1<<14) : 0;
	l |= (config & OMAP_DSS_LCD_IHS) ? (1<<13) : 0;
	l |= (config & OMAP_DSS_LCD_IVS) ? (1<<12) : 0;
	l |= ((acbi & 0x0F) << 8);
	l |= acb;

	dispc_base->pol_freq = l;
}

void omapdss_set_lcd_timings(struct omap_video_timings *timings, unsigned char is_tft)
{
	struct dispc *dispc_base = (struct dispc*)DISPC_BASE;

	// Calculate the DSS clocks in kHz (using DSS1_ALWON_FCL)
	unsigned int sys_clock = 13;
	unsigned int dpll4_mult = (*((unsigned int*)0x48004D44) >> 8) & 0x7FF;		// CM_CLKSEL2_PLL - 0x1B0 (432)
	unsigned int dpll4_div = *((unsigned int*)0x48004D44) & 0x7F;			// CM_CLKSEL2_PLL - 0xC (12)
	unsigned int dpll4_clock = sys_clock * 2 * dpll4_mult / (dpll4_div + 1);	// AM3517: 864 MHz

	// Adjust DPLL4 clock to kHz so we get better precision for the LCD Clock
	dpll4_clock *= 1000;

	unsigned char found = 0;
	unsigned int dss1_div = 1;
	unsigned int p_div = 1;
	unsigned int l_div = is_tft ? 2 : 3;

	while((dss1_div < 0x10) && !found)
	{
		p_div = dpll4_clock / dss1_div / l_div / timings->pixel_clock;

		if(p_div <= 255)
		{
			found = 1;
			break;
		}

		dss1_div++;
	}

	// Write the DSS Clock Divider, Pixel Clock Divider, and Logic Clock Divider
	*((unsigned int *)0x48004E40) = dss1_div;
	dispc_base->divisor = (l_div << 16) | (p_div);

	// Calculate and set the panel H/V register values
/*	if (!cpu_is_omap3630() && !cpu_is_omap3517() && !cpu_is_omap3505() &&
			(cpu_is_omap24xx() || (cpu_is_omap34xx() && omap_rev_lt_3_0())))
	{*/
		dispc_base->timing_h = FLD_VAL(timings->hsw-1, 5, 0) | FLD_VAL(timings->hfp-1, 15, 8) | FLD_VAL(timings->hbp-1, 27, 20);
		dispc_base->timing_v = FLD_VAL(timings->vsw-1, 5, 0) | FLD_VAL(timings->vfp, 15, 8) | FLD_VAL(timings->vbp, 27, 20);
/*	}
	else
	{
		dispc_base->timing_h = FLD_VAL(timings->hsw-1, 7, 0) | FLD_VAL(timings->hfp-1, 19, 8) | FLD_VAL(timings->hbp-1, 31, 20);
		dispc_base->timing_v = FLD_VAL(timings->vsw-1, 7, 0) | FLD_VAL(timings->vfp, 19, 8) | FLD_VAL(timings->vbp, 31, 20);
	}*/

	/* Set the LCD size values */
	dispc_base->size_lcd = FLD_VAL(timings->y_res - 1, 26, 16) | FLD_VAL(timings->x_res - 1, 10, 0);
}
//////////////////////////////////////////////////////////////////////////////
//#else
/*
 * Configure VENC for a given Mode (NTSC / PAL)
 */
void omap3_dss_venc_config(const struct venc_regs *venc_cfg,
				u32 height, u32 width)
{
	struct venc_regs *venc = (struct venc_regs *) OMAP3_VENC_BASE;
	struct dss_regs *dss = (struct dss_regs *) OMAP3_DSS_BASE;
	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;

	writel(venc_cfg->status, &venc->status);
	writel(venc_cfg->f_control, &venc->f_control);
	writel(venc_cfg->vidout_ctrl, &venc->vidout_ctrl);
	writel(venc_cfg->sync_ctrl, &venc->sync_ctrl);
	writel(venc_cfg->llen, &venc->llen);
	writel(venc_cfg->flens, &venc->flens);
	writel(venc_cfg->hfltr_ctrl, &venc->hfltr_ctrl);
	writel(venc_cfg->cc_carr_wss_carr, &venc->cc_carr_wss_carr);
	writel(venc_cfg->c_phase, &venc->c_phase);
	writel(venc_cfg->gain_u, &venc->gain_u);
	writel(venc_cfg->gain_v, &venc->gain_v);
	writel(venc_cfg->gain_y, &venc->gain_y);
	writel(venc_cfg->black_level, &venc->black_level);
	writel(venc_cfg->blank_level, &venc->blank_level);
	writel(venc_cfg->x_color, &venc->x_color);
	writel(venc_cfg->m_control, &venc->m_control);
	writel(venc_cfg->bstamp_wss_data, &venc->bstamp_wss_data);
	writel(venc_cfg->s_carr, &venc->s_carr);
	writel(venc_cfg->line21, &venc->line21);
	writel(venc_cfg->ln_sel, &venc->ln_sel);
	writel(venc_cfg->l21__wc_ctl, &venc->l21__wc_ctl);
	writel(venc_cfg->htrigger_vtrigger, &venc->htrigger_vtrigger);
	writel(venc_cfg->savid__eavid, &venc->savid__eavid);
	writel(venc_cfg->flen__fal, &venc->flen__fal);
	writel(venc_cfg->lal__phase_reset, &venc->lal__phase_reset);
	writel(venc_cfg->hs_int_start_stop_x,
				&venc->hs_int_start_stop_x);
	writel(venc_cfg->hs_ext_start_stop_x,
				&venc->hs_ext_start_stop_x);
	writel(venc_cfg->vs_int_start_x, &venc->vs_int_start_x);
	writel(venc_cfg->vs_int_stop_x__vs_int_start_y,
			&venc->vs_int_stop_x__vs_int_start_y);
	writel(venc_cfg->vs_int_stop_y__vs_ext_start_x,
			&venc->vs_int_stop_y__vs_ext_start_x);
	writel(venc_cfg->vs_ext_stop_x__vs_ext_start_y,
			&venc->vs_ext_stop_x__vs_ext_start_y);
	writel(venc_cfg->vs_ext_stop_y, &venc->vs_ext_stop_y);
	writel(venc_cfg->avid_start_stop_x, &venc->avid_start_stop_x);
	writel(venc_cfg->avid_start_stop_y, &venc->avid_start_stop_y);
	writel(venc_cfg->fid_int_start_x__fid_int_start_y,
				&venc->fid_int_start_x__fid_int_start_y);
	writel(venc_cfg->fid_int_offset_y__fid_ext_start_x,
				&venc->fid_int_offset_y__fid_ext_start_x);
	writel(venc_cfg->fid_ext_start_y__fid_ext_offset_y,
				&venc->fid_ext_start_y__fid_ext_offset_y);
	writel(venc_cfg->tvdetgp_int_start_stop_x,
				&venc->tvdetgp_int_start_stop_x);
	writel(venc_cfg->tvdetgp_int_start_stop_y,
				&venc->tvdetgp_int_start_stop_y);
	writel(venc_cfg->gen_ctrl, &venc->gen_ctrl);
	writel(venc_cfg->output_control, &venc->output_control);
	writel(venc_cfg->dac_b__dac_c, &venc->dac_b__dac_c);

	/* Configure DSS for VENC Settings */
	writel(VENC_DSS_CONFIG, &dss->control);

	/* Configure height and width for Digital out */
	writel(((height << DIG_LPP_SHIFT) | width), &dispc->size_dig);
}

/*
 * Configure Panel Specific Parameters
 */
void omap3_dss_panel_config(const struct panel_config *panel_cfg)
{
	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;

	writel(panel_cfg->timing_h, &dispc->timing_h);
	writel(panel_cfg->timing_v, &dispc->timing_v);
	writel(panel_cfg->pol_freq, &dispc->pol_freq);
	writel(panel_cfg->divisor, &dispc->divisor);
	writel(panel_cfg->lcd_size, &dispc->size_lcd);
	writel((panel_cfg->load_mode << FRAME_MODE_SHIFT), &dispc->config);
	writel(((panel_cfg->panel_type << TFTSTN_SHIFT) |
		(panel_cfg->data_lines << DATALINES_SHIFT)), &dispc->control);
	writel(panel_cfg->panel_color, &dispc->default_color0);
}

/*
 * Enable LCD and DIGITAL OUT in DSS
 */
void omap3_dss_enable(void)
{
	struct dispc_regs *dispc = (struct dispc_regs *) OMAP3_DISPC_BASE;
	u32 l = 0;

	l = readl(&dispc->control);
	l |= DISPC_ENABLE & ~(DIG_ENABLE);
	writel(l, &dispc->control);
////////////////////////////////////
omap_request_gpio(176);
omap_set_gpio_direction(176,0);
omap_set_gpio_dataout(176,1);
}
#endif

#ifdef CONFIG_CFB_CONSOLE
GraphicDevice smi;

void * video_hw_init(void) {
    //register GraphicDevice *pGD = (GraphicDevice *)&smi;
    struct panel_config pcfg;
    printf("Initialize lcd\n");
    smi.winSizeX = smi.plnSizeX = 800;
    smi.winSizeY = smi.plnSizeY = 480;
    smi.gdfBytesPP = 2;
    smi.gdfIndex = GDF_16BIT_565RGB;
    smi.frameAdrs = LCD_VIDEO_ADDR;
    //writel(LCD_VIDEO_ADDR, 0x48050480); //DISPC_GFX_BAj
    //writel(LCD_VIDEO_ADDR, 0x48050484); //DISPC_GFX_BAj
    smi.memSize = smi.winSizeX * smi.winSizeY * smi.gdfBytesPP;
    pcfg.timing_h   = 0x08f03f4f;
    pcfg.timing_v   = 0x04801c04;
    pcfg.divisor    = 0x00010005;
    pcfg.lcd_size   = 0x0207031f; //smi.winSizeX << 16 | smi.winSizeY;
    pcfg.load_mode  = 0x00000204 >> FRAME_MODE_SHIFT;
    pcfg.panel_type = 1; //0x00018309;
    pcfg.data_lines = 3; //0x00018309;
    pcfg.panel_color= 0x00000000;
    //printf("Done w/ lcd init\n");
    //omap3_dss_panel_config(&pcfg);
    //printf("Done w/ panel cfg\n");
//    omap3_dss_enable();
//    dumpregs();
    return ((void*)&smi);
}
#endif
