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

#ifndef DSS_H
#define DSS_H

/*
 * DSS Base Registers
 */
#define OMAP3_DSS_BASE		0x48050040
#define OMAP3_DISPC_BASE	0x48050440
#define OMAP3_VENC_BASE		0x48050C00

/* DSS Registers */
struct dss_regs {
	u32 control;				/* 0x40 */
	u32 sdi_control;			/* 0x44 */
	u32 pll_control;			/* 0x48 */
};

/* DISPC Registers */
struct dispc_regs {
	u32 control;				/* 0x40 */
	u32 config;				/* 0x44 */
	u32 reserve_2;				/* 0x48 */
	u32 default_color0;			/* 0x4C */
	u32 default_color1;			/* 0x50 */
	u32 trans_color0;			/* 0x54 */
	u32 trans_color1;			/* 0x58 */
	u32 line_status;			/* 0x5C */
	u32 line_number;			/* 0x60 */
	u32 timing_h;				/* 0x64 */
	u32 timing_v;				/* 0x68 */
	u32 pol_freq;				/* 0x6C */
	u32 divisor;				/* 0x70 */
	u32 global_alpha;			/* 0x74 */
	u32 size_dig;				/* 0x78 */
	u32 size_lcd;				/* 0x7C */
        u32 gfx_ba[2];                          /* 0x80, 0x84 */
        u32 gfx_position;                       /* 0x88 */
        u32 gfx_size;                           /* 0x8C */
        u32 res[4];                             /* 0x90-0x9c */
        u32 gfx_attributes;                     /* 0xa0 */
};

/* VENC Registers */
struct venc_regs {
	u32 rev_id;				/* 0x00 */
	u32 status;				/* 0x04 */
	u32 f_control;				/* 0x08 */
	u32 reserve_1;				/* 0x0C */
	u32 vidout_ctrl;			/* 0x10 */
	u32 sync_ctrl;				/* 0x14 */
	u32 reserve_2;				/* 0x18 */
	u32 llen;				/* 0x1C */
	u32 flens;				/* 0x20 */
	u32 hfltr_ctrl;				/* 0x24 */
	u32 cc_carr_wss_carr;			/* 0x28 */
	u32 c_phase;				/* 0x2C */
	u32 gain_u;				/* 0x30 */
	u32 gain_v;				/* 0x34 */
	u32 gain_y;				/* 0x38 */
	u32 black_level;			/* 0x3C */
	u32 blank_level;			/* 0x40 */
	u32 x_color;				/* 0x44 */
	u32 m_control;				/* 0x48 */
	u32 bstamp_wss_data;			/* 0x4C */
	u32 s_carr;				/* 0x50 */
	u32 line21;				/* 0x54 */
	u32 ln_sel;				/* 0x58 */
	u32 l21__wc_ctl;			/* 0x5C */
	u32 htrigger_vtrigger;			/* 0x60 */
	u32 savid__eavid;			/* 0x64 */
	u32 flen__fal;				/* 0x68 */
	u32 lal__phase_reset;			/* 0x6C */
	u32 hs_int_start_stop_x;		/* 0x70 */
	u32 hs_ext_start_stop_x;		/* 0x74 */
	u32 vs_int_start_x;			/* 0x78 */
	u32 vs_int_stop_x__vs_int_start_y;	/* 0x7C */
	u32 vs_int_stop_y__vs_ext_start_x;	/* 0x80 */
	u32 vs_ext_stop_x__vs_ext_start_y;	/* 0x84 */
	u32 vs_ext_stop_y;			/* 0x88 */
	u32 reserve_3;				/* 0x8C */
	u32 avid_start_stop_x;			/* 0x90 */
	u32 avid_start_stop_y;			/* 0x94 */
	u32 reserve_4;				/* 0x98 */
	u32 reserve_5;				/* 0x9C */
	u32 fid_int_start_x__fid_int_start_y;	/* 0xA0 */
	u32 fid_int_offset_y__fid_ext_start_x;	/* 0xA4 */
	u32 fid_ext_start_y__fid_ext_offset_y;	/* 0xA8 */
	u32 reserve_6;				/* 0xAC */
	u32 tvdetgp_int_start_stop_x;		/* 0xB0 */
	u32 tvdetgp_int_start_stop_y;		/* 0xB4 */
	u32 gen_ctrl;				/* 0xB8 */
	u32 reserve_7;				/* 0xBC */
	u32 reserve_8;				/* 0xC0 */
	u32 output_control;			/* 0xC4 */
	u32 dac_b__dac_c;			/* 0xC8 */
	u32 height_width;			/* 0xCC */
};

/* Few Register Offsets */
#define FRAME_MODE_SHIFT			1
#define TFTSTN_SHIFT				3
#define DATALINES_SHIFT				8

/* Enabling Display controller */
#define LCD_ENABLE				1
#define DIG_ENABLE				(1 << 1)
#define GO_LCD					(1 << 5)
#define GO_DIG					(1 << 6)
#define GP_OUT0					(1 << 15)
#define GP_OUT1					(1 << 16)

#define DISPC_ENABLE				(LCD_ENABLE | \
						 DIG_ENABLE | \
						 GO_LCD | \
						 GO_DIG | \
						 GP_OUT0| \
						 GP_OUT1)

/* Configure VENC DSS Params */
#define VENC_CLK_ENABLE				(1 << 3)
#define DAC_DEMEN				(1 << 4)
#define DAC_POWERDN				(1 << 5)
#define VENC_OUT_SEL				(1 << 6)
#define DIG_LPP_SHIFT				16
#define VENC_DSS_CONFIG				(VENC_CLK_ENABLE | \
						 DAC_DEMEN | \
						 DAC_POWERDN | \
						 VENC_OUT_SEL)

/* Gfx params */
#define RGB_16                                  (6)
#define BMP_8                                   (3)
#define GFX_FMT(val)                            ((val) << 1)
#define GFX_BURST(val)                          ((val) << 6)
#define GFX_EN                                  (1)

/*
 * Panel Configuration
 */
struct panel_config {
	u32 timing_h;
	u32 timing_v;
	u32 pol_freq;
	u32 divisor;
	u32 lcd_size;
	u32 panel_type;
	u32 data_lines;
	u32 load_mode;
	u32 panel_color;
        u32 gfx_attrib;
};

/*
 * Generic DSS Functions
 */
void omap3_dss_venc_config(const struct venc_regs *venc_cfg,
			u32 height, u32 width);
void omap3_dss_panel_config(const struct panel_config *panel_cfg);
void omap3_dss_enable(void);

enum omap_panel_config {
	OMAP_DSS_LCD_IVS		= 1<<0,
	OMAP_DSS_LCD_IHS		= 1<<1,
	OMAP_DSS_LCD_IPC		= 1<<2,
	OMAP_DSS_LCD_IEO		= 1<<3,
	OMAP_DSS_LCD_RF			= 1<<4,
	OMAP_DSS_LCD_ONOFF		= 1<<5,

	OMAP_DSS_LCD_TFT		= 1<<20,
};

#endif /* DSS_H */
