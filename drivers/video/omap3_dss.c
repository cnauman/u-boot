/*
 * (C) Copyright 2010
 * Texas Instruments, <www.ti.com>
 * Syed Mohammed Khasim <khasim@ti.com>
 *
 * Referred to Linux Kernel DSS driver files for OMAP3 by
 * Tomi Valkeinen from drivers/video/omap2/dss/
 *
 * See file CREDITS for list of people who contributed to this
 * project.
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation's version 2 and any
 * later version the License.
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
#include "videomodes.h"

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
	writel(LCD_VIDEO_ADDR, &dispc->gfx_ba[0]);
	writel(dispc->gfx_ba[0], &dispc->gfx_ba[1]);
	writel(panel_cfg->divisor, &dispc->divisor);
	writel(panel_cfg->lcd_size, &dispc->size_lcd);
	writel(dispc->size_lcd, &dispc->gfx_size);
        writel(panel_cfg->gfx_attrib, &dispc->gfx_attributes);
	writel((panel_cfg->load_mode << FRAME_MODE_SHIFT), &dispc->config);
	writel(((panel_cfg->panel_type << TFTSTN_SHIFT) |
		(panel_cfg->data_lines << DATALINES_SHIFT))|3<<29, &dispc->control);
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

	l |= DISPC_ENABLE;
	writel(l, &dispc->control);

	/* check for L3 interconnect errors */
	udelay(1);
	if (readl(0x68000510)) {
		udelay(3); //printf("Resetting DSS L3 interconnect\n");
		writel(1, 0x68005420); /* reset DSS L3 interconnect */
		udelay(1);
		writel(0, 0x68005420); /* reset done for DSS L3 interconnect */
	}
}


#ifdef CONFIG_CFB_CONSOLE

GraphicDevice smi;

static void __pre_setup_video_env(void)
{
    /* do nothing */
}
void pre_setup_video_env(void)
	__attribute__((weak, alias("__pre_setup_video_env")));

static void vidmem_clear(void * addr, int width, int height, int bytesPP) {
    int sze = width * height * bytesPP;
    memset(addr, 0, sze);
}

#define TIMING(sw,fp,bp) ( \
        ((bp & 0xfff) << 20) | \
        ((fp & 0xfff) << 8) | \
        (sw & 0xff)\
        )
#define DIV(lcd, pcd) (((lcd & 0xff) << 16) | (pcd & 0xff))
#define SIZE(x, y) ((y-1) << 16 | (x-1))

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
/*    int hsw=0, hfp=0, hbp=0;
    int vsw=0, vfp=0, vbp=0;
    int lclk = 1, pclk=2, pol_flags = 0;
    int acbi = 0, acb = 0;*/
	int bpp = 0;
	struct ctfb_res_modes *res_mode;
	struct ctfb_res_modes var_mode;
        char * penv;
	pre_setup_video_env();
	if (NULL == (penv = getenv("ub_vid"))) return NULL;

	res_mode = (struct ctfb_res_modes *) &var_mode;
	bpp = video_get_params (res_mode, penv);

	smi.winSizeX = res_mode->xres;
	smi.winSizeY = res_mode->yres;
	smi.plnSizeX = res_mode->xres;
	smi.plnSizeY = res_mode->yres;

        smi.gdfBytesPP = bpp;
        if (2 == bpp) smi.gdfIndex = GDF_16BIT_565RGB;
        else if (1 == bpp) smi.gdfIndex = GDF__8BIT_332RGB;
        pcfg.panel_type = res_mode->vmode&1;

    pcfg.timing_h = TIMING(res_mode->hsync_len, res_mode->left_margin, res_mode->right_margin); //hsw, hfp, hbp);
    pcfg.timing_v = TIMING(res_mode->vsync_len, res_mode->upper_margin, res_mode->lower_margin); //vsw, vfp, vbp);
    pcfg.data_lines = 3;
    pcfg.lcd_size   = SIZE(smi.winSizeX, smi.winSizeY);
    pcfg.load_mode  = 0; //0x00000204 >> FRAME_MODE_SHIFT;
    smi.frameAdrs = LCD_VIDEO_ADDR;
    smi.memSize = smi.winSizeX * smi.winSizeY * smi.gdfBytesPP;
    pcfg.panel_color= 0;
    pcfg.divisor    = DIV(res_mode->pixclock >> 8, res_mode->pixclock & 0xff); //lclk, pclk);
    pcfg.pol_freq   = omapdss_set_pol_freq(res_mode->vmode >>1, res_mode->sync >> 8, res_mode->sync & 0xff); //pol_flags, acbi, acb); // 0, 0); //0x30000;
    //printf("Done w/ lcd init\n");
        /*if (!panel_cfg->panel_type)
	    pcfg.gfx_attrib = GFX_BURST(2) | GFX_FMT(RGB_16) //GFX_FMT(BMP_8) 
                    | GFX_EN;
        else*/
	    pcfg.gfx_attrib = GFX_BURST(2) | GFX_FMT(RGB_16) | GFX_EN;
    vidmem_clear(smi.frameAdrs, smi.winSizeX, smi.winSizeY, smi.gdfBytesPP);
    omap3_dss_panel_config(&pcfg);
    omap3_dss_enable();

    return ((void*)&smi);
}
#endif
