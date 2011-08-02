#ifndef __OMAPDSS_H
#define __OMAPDSS_H

/* OMAP TRM gives bitfields as start:end, where start is the higher bit
   number. For example 7:0 */
#define FLD_MASK(start, end)	(((1 << ((start) - (end) + 1)) - 1) << (end))
#define FLD_VAL(val, start, end) (((val) << (end)) & FLD_MASK(start, end))
#define FLD_GET(val, start, end) (((val) & FLD_MASK(start, end)) >> (end))
#define FLD_MOD(orig, val, start, end) \
	(((orig) & ~FLD_MASK(start, end)) | FLD_VAL(val, start, end))

/* GIMP Image Header Helper Macro */
#define HEADER_PIXEL(data,pixel) {\
pixel[0] = (((data[0] - 33) << 2) | ((data[1] - 33) >> 4)); \
pixel[1] = ((((data[1] - 33) & 0xF) << 4) | ((data[2] - 33) >> 2)); \
pixel[2] = ((((data[2] - 33) & 0x3) << 6) | ((data[3] - 33))); \
data += 4; \
}

enum omap_panel_config {
	OMAP_DSS_LCD_IVS		= 1<<0,
	OMAP_DSS_LCD_IHS		= 1<<1,
	OMAP_DSS_LCD_IPC		= 1<<2,
	OMAP_DSS_LCD_IEO		= 1<<3,
	OMAP_DSS_LCD_RF			= 1<<4,
	OMAP_DSS_LCD_ONOFF		= 1<<5,

	OMAP_DSS_LCD_TFT		= 1<<20,
};

struct omap_video_timings {
	/* Unit: pixels */
	unsigned short x_res;
	/* Unit: pixels */
	unsigned short y_res;
	/* Unit: KHz */
	unsigned int pixel_clock;
	/* Unit: pixel clocks */
	unsigned short hsw;	/* Horizontal synchronization pulse width */
	/* Unit: pixel clocks */
	unsigned short hfp;	/* Horizontal front porch */
	/* Unit: pixel clocks */
	unsigned short hbp;	/* Horizontal back porch */
	/* Unit: line clocks */
	unsigned short vsw;	/* Vertical synchronization pulse width */
	/* Unit: line clocks */
	unsigned short vfp;	/* Vertical front porch */
	/* Unit: line clocks */
	unsigned short vbp;	/* Vertical back porch */
};

struct omap_panel{
	int acbi;	/* ac-bias pin transitions per interrupt */
	int acb;	/* ac-bias pin frequency */

	unsigned char is_tft;		/* TFT = 1 or STN = 0 */
	enum omap_panel_config config;
	struct omap_video_timings timings;
	unsigned char data_lines;	/* pixel bit width */

	//unsigned int dss_dpll4;		/* CM DSS Clock - Get this from Linux */
	//unsigned int dispc_divisor;	/* DISPC Clock Dividers - Get this from Linux */

	unsigned int fb_address1;	/* Location of framebuffer in memory */
	unsigned int fb_address2;	/* Location of doublebuffer in memory */

	char *logo;			/* Location of Logo Data (GIMP Header) */
	unsigned int logo_width;	/* Width of Logo */
	unsigned int logo_height;	/* Height of Logo */
};

/* OMAP34xx series DSS register definition */
#define DSS_BASE 0x48050000
struct dss{				// 0x48050000
	unsigned int revisionnumber;		// 000
	unsigned char res1[0xC];			// 004 ~ 010
	unsigned int sysconfig;			// 010
	unsigned int sysstatus;			// 014
	unsigned int irqstatus;			// 018
	unsigned char res2[0x24];			// 01C ~ 040
	unsigned int control;			// 040
	unsigned int sdi_control;		// 044
	unsigned int pll_control;		// 048
	unsigned char res3[0x10];			// 04C ~ 05C
	unsigned int sdi_status;			// 05C
};

#define DISPC_BASE 0x48050400
struct fir_coef_h{
	unsigned int h;
	unsigned int hv;
};

struct dispc{					// 0x48050400
	unsigned int revision;				// 000
	unsigned char res1[0xC];			// 004 ~ 010
	unsigned int sysconfig;				// 010
	unsigned int sysstatus;				// 014
	unsigned int irqstatus;				// 018
	unsigned int irqenable;				// 01C
	unsigned char res2[0x20];			// 020 ~ 040
	unsigned int control;				// 040
	unsigned int config;				// 044
	unsigned char res3[0x4];			// 048
	unsigned int default_color[2];			// 04C, 050
	unsigned int trans_color[2];			// 054, 058
	unsigned int line_status;			// 05C
	unsigned int line_number;			// 060
	unsigned int timing_h;				// 064
	unsigned int timing_v;				// 068
	unsigned int pol_freq;				// 06C
	unsigned int divisor;				// 070
	unsigned int global_alpha;			// 074
	unsigned int size_dig;				// 078
	unsigned int size_lcd;				// 07C
	unsigned int gfx_ba[2];				// 080, 084
	unsigned int gfx_position;			// 088
	unsigned int gfx_size;				// 08C
	unsigned char res4[0x10];			// 090 ~ 0A0
	unsigned int gfx_attributes;			// 0A0
	unsigned int gfx_fifo_threshold;		// 0A4
	unsigned int gfx_fifo_size_status;		// 0A8
	unsigned int gfx_row_inc;			// 0AC
	unsigned int gfx_pixel_inc;			// 0B0
	unsigned int gfx_window_skip;			// 0B4
	unsigned int gfx_table_ba;			// 0B8
	unsigned int vid1_ba[2];			// 0BC, 0C0
	unsigned int vid1_position;			// 0C4
	unsigned int vid1_size;				// 0C8
	unsigned int vid1_attributes;			// 0CC
	unsigned int vid1_fifo_threshold;		// 0D0
	unsigned int vid1_fifo_size_status;		// 0D4
	unsigned int vid1_row_inc;			// 0D8
	unsigned int vid1_pixel_inc;			// 0DC
	unsigned int vid1_fir;				// 0E0
	unsigned int vid1_picture_size;			// 0E4
	unsigned int vid1_accu[2];			// 0E8, 0EC
	struct fir_coef_h vid1_fir_coef_h[8];		// 0F0, 0F8, 100, 108, 110, 118, 120, 128
	unsigned int vid1_conv_coef0;			// 130
	unsigned int vid1_conv_coef1;			// 134
	unsigned int vid1_conv_coef2;			// 138
	unsigned int vid1_conv_coef3;			// 13C
	unsigned int vid1_conv_coef4;			// 140
	unsigned char res5[8];				// 144 ~ 14C
	unsigned int vid2_ba[2];			// 14C, 150
	unsigned int vid2_position;			// 154
	unsigned int vid2_size;				// 158
	unsigned int vid2_attributes;			// 15C
	unsigned int vid2_fifo_threshold;		// 160
	unsigned int vid2_fifo_size_status;		// 164
	unsigned int vid2_row_inc;			// 168
	unsigned int vid2_pixel_inc;			// 16C
	unsigned int vid2_fir;				// 170
	unsigned int vid2_picture_size;			// 174
	unsigned int vid2_accu[2];			// 178, 17C
	struct fir_coef_h vid2_fir_coef_h[8];		// 180, 188, 190, 198, 1A0, 1A8, 1B0, 1B8
	unsigned int vid2_conv_coef0;			// 1C0
	unsigned int vid2_conv_coef1;			// 1C4
	unsigned int vid2_conv_coef2;			// 1C8
	unsigned int vid2_conv_coef3;			// 1CC
	unsigned int vid2_conv_coef4;			// 1D0
	unsigned int data_cycle[3];			// 1D4, 1D8, 1DC
	unsigned int vid1_fir_coef_v[8];		// 1E0, 1E4, 1E8, 1EC, 1F0, 1F4, 1F8, 1FC
	unsigned int vid2_fir_coef_v[8];		// 200, 204, 208, 20C, 210, 214, 218, 21C
	unsigned int cpr_coef_r;			// 220
	unsigned int cpr_coef_g;			// 224
	unsigned int cpr_coef_b;			// 228
	unsigned int gfx_preload;			// 22C
	unsigned int vid1_preload;			// 230
	unsigned int vid2_preload;			// 234
};

#define VENC_BASE 0x48050C00
struct venc{
	unsigned int rev_id;				// 0x00, 	0x48050C00
	unsigned int status;				// 0x04,	0x48050C04
	unsigned int f_control;				// 0x08,	0x48050C08
	unsigned int res1;
	unsigned int vidout_ctrl;			// 0x10,	0x48050C10
	unsigned int sync_ctrl;				// 0x14,	0x48050C14
	unsigned int res2;
	unsigned int llen;				// 0x1C,	0x48050C1C
	unsigned int flens;				// 0x20,	0x48050C20
	unsigned int hfltr_ctrl;				// 0x24,	0x48050C24
	unsigned int cc_carr_wss_carr;			// 0x28,	0x48050C28
	unsigned int c_phase;				// 0x2C,	0x48050C2C
	unsigned int gain_u;				// 0x30,	0x48050C30
	unsigned int gain_v;				// 0x34,	0x48050C34
	unsigned int gain_y;				// 0x38,	0x48050C38
	unsigned int black_level;			// 0x3C,	0x48050C3C
	unsigned int blank_level;			// 0x40,	0x48050C40
	unsigned int x_color;				// 0x44,	0x48050C44
	unsigned int m_control;				// 0x48,	0x48050C48
	unsigned int bstamp_wss_data;			// 0x4C,	0x48050C4C
	unsigned int s_carr;				// 0x50,	0x48050C50
	unsigned int line21;				// 0x54,	0x48050C54
	unsigned int ln_sel;				// 0x58,	0x48050C58
	unsigned int l21_wc_ctl;				// 0x5C,	0x48050C5C
	unsigned int htrigger_vtrigger;			// 0x60,	0x48050C60
	unsigned int savid_eavid;			// 0x64,	0x48050C64
	unsigned int flen_fal;				// 0x68,	0x48050C68
	unsigned int lal_phase_reset;			// 0x6C,	0x48050C6C
	unsigned int hs_int_start_stop_x;		// 0x70,	0x48050C70
	unsigned int hs_ext_start_stop_x;		// 0x74,	0x48050C74
	unsigned int vs_int_start_x;			// 0x78,	0x48050C78
	unsigned int vs_int_stop_x_vs_int_start_y;	// 0x7C,	0x48050C7C
	unsigned int vs_int_stop_y_vs_ext_start_x;	// 0x80,	0x48050C80
	unsigned int vs_ext_stop_x_vs_ext_start_y;	// 0x84,	0x48050C84
	unsigned int vs_ext_stop_y;			// 0x88,	0x48050C88
	unsigned int res3;
	unsigned int avid_start_stop_x;			// 0x90,	0x48050C90
	unsigned int avid_start_stop_y;			// 0x94,	0x48050C94
	unsigned int res4[2];
	unsigned int fid_int_start_x_fid_int_start_y;	// 0xA0,	0x48050CA0
	unsigned int fid_int_offset_y_fid_ext_start_x;	// 0xA4,	0x48050CA4
	unsigned int fid_ext_start_y_fid_ext_offset_y;	// 0xA8,	0x48050CA8
	unsigned int res5;
	unsigned int tvdetgp_int_start_stop_x;		// 0xB0,	0x48050CB0
	unsigned int tvdetgp_int_start_stop_y;		// 0xB4,	0x48050CB4
	unsigned int gen_ctrl;				// 0xB8,	0x48050CB8
	unsigned int res6[2];
	unsigned int output_control;			// 0xC4,	0x48050CC4
	unsigned int output_test;			// 0xC8,	0x48050CC8
};

void omapdss_init(struct omap_panel *lcd);
void omapdss_set_pol_freq(enum omap_panel_config config, unsigned char acbi, unsigned char acb);
void omapdss_set_lcd_timings(struct omap_video_timings *timings, unsigned char is_tft);

#endif

