/*
 * (C) Copyright 2006 by OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
 *
 * This program is free software; you can redistribute it and/or
 * modify it under the terms of the GNU General Public License as
 * published by the Free Software Foundation; either version 2 of
 * the License, or (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.	See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place, Suite 330, Boston,
 * MA 02111-1307 USA
 */

#include <common.h>

#include <video_fb.h>
#include "videomodes.h"
#include <asm/arch/s3c24x0_cpu.h>

//#define MVAL		(0)
//#define MVAL_USED 	(0)		//0=each frame   1=rate by MVAL
//#define INVVDEN		(1)		//0=normal       1=inverted
//#define BSWP		(0)		//Byte swap control
//#define HWSWP		(0) (1)		//Half word swap control

//TFT 240320
//#define LCD_XSIZE_TFT_240320 	(480) //(240)	
//#define LCD_YSIZE_TFT_240320 	(640) //(320)

//TFT240320
//#define HOZVAL_TFT_240320	(LCD_XSIZE_TFT_240320-1)
//#define LINEVAL_TFT_240320	(LCD_YSIZE_TFT_240320-1)

//Timing parameter for NEC3.5"
#define VBPD_240320		(29) //(3)		
#define VFPD_240320		(3) //(10)
#define VSPW_240320		(3) //(1)

#define HBPD_240320		(40) //(5)
#define HFPD_240320		(40) //(2)
#define HSPW_240320		(48) //(36)

//#define CLKVAL_TFT_240320	(3) 	
//FCLK=101.25MHz,HCLK=50.625MHz,VCLK=6.33MHz

/*
 * Export Graphic Device
 */
GraphicDevice smi;

#define VIDEO_MEM_SIZE	0x200000	/* 480x640x16bit = 614400 bytes */
//#define VIDEO_MEM_SIZE	0xa0000	/* 480x640x16bit = 614400 bytes  rnd up*/

extern void board_video_init(GraphicDevice *pGD);

/*******************************************************************************
 *
 * Init video chip with common Linux graphic modes (lilo)
 */
void *video_hw_init (void)
{
	struct s3c24x0_lcd * lcd = s3c24x0_get_base_lcd();
	register GraphicDevice *pGD = (GraphicDevice *)&smi;
	int videomode;
	/*unsigned long t1, hsynch, vsynch;*/
	char *penv;
	int tmp, i, bits_per_pixel;
	struct ctfb_res_modes *res_mode;
	struct ctfb_res_modes var_mode;
	/*unsigned char videoout;*/

	/* Search for video chip */
	printf("Video: ");

	tmp = 0;

	videomode = CONFIG_SYS_DEFAULT_VIDEO_MODE;
	/* get video mode via environment */
	if ((penv = getenv ("videomode")) != NULL) {
		/* deceide if it is a string */
		if (penv[0] <= '9') {
			videomode = (int) simple_strtoul (penv, NULL, 16);
			tmp = 1;
		}
	} else {
		tmp = 1;
	}
	if (tmp) {
		/* parameter are vesa modes */
		/* search params */
		for (i = 0; i < VESA_MODES_COUNT; i++) {
			if (vesa_modes[i].vesanr == videomode)
				break;
		}
		if (i == VESA_MODES_COUNT) {
			printf ("no VESA Mode found, switching to mode 0x%x ", CONFIG_SYS_DEFAULT_VIDEO_MODE);
			i = 0;
		}
		res_mode =
			(struct ctfb_res_modes *) &res_mode_init[vesa_modes[i].
								 resindex];
		bits_per_pixel = vesa_modes[i].bits_per_pixel;
	} else {

		res_mode = (struct ctfb_res_modes *) &var_mode;
		bits_per_pixel = video_get_params (res_mode, penv);
	}

	/* calculate hsynch and vsynch freq (info only) */
	/*t1 = (res_mode->left_margin + res_mode->xres +
	      res_mode->right_margin + res_mode->hsync_len) / 8;
	t1 *= 8;
	t1 *= res_mode->pixclock;
	t1 /= 1000;
	hsynch = 1000000000L / t1;
	t1 *=
		(res_mode->upper_margin + res_mode->yres +
		 res_mode->lower_margin + res_mode->vsync_len);
	t1 /= 1000;
	vsynch = 1000000000L / t1;*/

	/* fill in Graphic device struct */
	sprintf (pGD->modeIdent, "%dx%dx%d" /*"%ldkHz %ldHz"*/, res_mode->xres,
		 res_mode->yres, bits_per_pixel/*, (hsynch / 1000),
		 (vsynch / 1000)*/);
	printf ("%s\n", pGD->modeIdent);
	pGD->winSizeX = res_mode->xres;
	pGD->winSizeY = res_mode->yres;
	pGD->plnSizeX = res_mode->xres;
	pGD->plnSizeY = res_mode->yres;
	switch (bits_per_pixel) {
	case 8:
		pGD->gdfBytesPP = 1;
		pGD->gdfIndex = GDF__8BIT_INDEX;
		break;
	case 15:
		pGD->gdfBytesPP = 2;
		pGD->gdfIndex = GDF_15BIT_555RGB;
		break;
	case 16:
		pGD->gdfBytesPP = 2;
		pGD->gdfIndex = GDF_16BIT_565RGB;
		break;
	case 24:
		pGD->gdfBytesPP = 3;
		pGD->gdfIndex = GDF_24BIT_888RGB;
		break;
	}

	pGD->frameAdrs = LCD_VIDEO_ADDR;
	pGD->memSize = pGD->winSizeX * pGD->winSizeY * pGD->gdfBytesPP;

	lcd->lcdcon1 = /*0x178;*/ (res_mode->pixclock << 8) |
			(res_mode->vmode << 1);
	lcd->lcdcon2 = /*0x1c77c082;*/((res_mode->upper_margin - 1) << 24) |
			((pGD->winSizeY - 1) << 14) |
			((res_mode->lower_margin - 1) << 6) |
			(res_mode->vsync_len - 1);
	lcd->lcdcon3 = /*0x013b1f27; */((res_mode->right_margin - 1) << 19) |
			((pGD->winSizeX - 1) << 8) |
			(res_mode->left_margin - 1); 
	lcd->lcdcon4 = /*0x2f; */ (res_mode->hsync_len - 1); 

	lcd->lcdcon5 = /* 0xb09 */ res_mode->sync;

	board_video_init(pGD);

	lcd->lcdsaddr1 = pGD->frameAdrs >> 1;

	/* This marks the end of the frame buffer. */
	lcd->lcdsaddr2 = (lcd->lcdsaddr1&0x1fffff) + (pGD->winSizeX+0) * pGD->winSizeY;
	lcd->lcdsaddr3 = pGD->winSizeX;

	/* Clear video memory */
	memset((void *)pGD->frameAdrs, 0, pGD->memSize);

	/* Enable  Display  */
	lcd->lcdcon1 |= 0x01;	/* ENVID = 1 */

	return ((void*)&smi);
}

void
video_set_lut (unsigned int index,	/* color number */
	       unsigned char r,	/* red */
	       unsigned char g,	/* green */
	       unsigned char b	/* blue */
    )
{
}

