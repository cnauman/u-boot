/*
 * (C) Copyright 2006 OpenMoko, Inc.
 * Author: Harald Welte <laforge@openmoko.org>
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

#include <nand.h>
#include <asm/arch/s3c24x0_cpu.h>
#include <asm/io.h>

#ifdef CONFIG_S3C2410
#define NAND_TYPE                   s3c2410_nand 
#define GET_NAND_BASE()             s3c2410_get_base_nand()

#define S3C24x0_NFCONF_EN          (1<<15)
#define S3C2410_NFCONF_512BYTE     (1<<14)
#define S3C2410_NFCONF_4STEP       (1<<13)
#define S3C2410_NFCONF_INITECC     (1<<12)
#define S3C24x0_NFCONF_nFCE        (1<<11)
#define S3C24x0_NFCONF_TACLS(x)    ((x)<<8)
#define S3C24x0_NFCONF_TWRPH0(x)   ((x)<<4)
#define S3C24x0_NFCONF_TWRPH1(x)   ((x)<<0)

#define S3C24x0_ADDR_NALE 4
#define S3C24x0_ADDR_NCLE 8
#define S3C24x0_ADDR_DATA 0 

#elif CONFIG_S3C2440
#define NAND_TYPE                       s3c2440_nand 
#define GET_NAND_BASE()                 s3c2440_get_base_nand()
#define S3C24x0_NFCONF_EN               (1<<0)
#define S3C24x0_NFCONT_nFCE		(1<<1)
#define S3C24x0_NFCONF_TACLS(x)		((x)<<12)
#define S3C24x0_NFCONF_TWRPH0(x)	((x)<<8)
#define S3C24x0_NFCONF_TWRPH1(x)	((x)<<4)


#define S3C24x0_ADDR_NALE 0x8
#define S3C24x0_ADDR_NCLE 0xc 
#define S3C24x0_ADDR_DATA 0x10 
#else
#error Missing S3C24x0 CPU configuration
#endif

#ifdef CONFIG_NAND_SPL

/* in the early stage of NAND flash booting, printf() is not available */
#define printf(fmt, args...)

static void nand_read_buf(struct mtd_info *mtd, u_char *buf, int len)
{
	int i;
	struct nand_chip *this = mtd->priv;

	for (i = 0; i < len; i++)
		buf[i] = readb(this->IO_ADDR_R);
}
#endif

static void s3c24x0_hwcontrol(struct mtd_info *mtd, int cmd, unsigned int ctrl)
{
	struct nand_chip *chip = mtd->priv;
	struct NAND_TYPE *nand = GET_NAND_BASE();

	debug("hwcontrol(): 0x%02x 0x%02x\n", cmd, ctrl);

	if (ctrl & NAND_CTRL_CHANGE) {
		ulong IO_ADDR_W = (ulong)nand;

		if (!(ctrl & NAND_CLE))
			IO_ADDR_W |= S3C24x0_ADDR_NCLE;
		if (!(ctrl & NAND_ALE))
			IO_ADDR_W |= S3C24x0_ADDR_NALE;
                if (!(ctrl & (NAND_ALE | NAND_CLE)))
			IO_ADDR_W = (ulong)nand + S3C24x0_ADDR_DATA;

		chip->IO_ADDR_W = (void *)IO_ADDR_W;
#ifdef CONFIG_S3C2410
		if (ctrl & NAND_NCE)
			writel(readl(&nand->nfconf) & ~S3C24x0_NFCONF_nFCE,
			       &nand->nfconf);
		else
			writel(readl(&nand->nfconf) | S3C24x0_NFCONF_nFCE,
			       &nand->nfconf);
#elif CONFIG_S3C2440
		if (ctrl & NAND_NCE)
			writel(readl(&nand->nfcont) & ~S3C24x0_NFCONT_nFCE,
			       &nand->nfcont);
		else
			writel(readl(&nand->nfcont) | S3C24x0_NFCONT_nFCE,
			       &nand->nfcont);
#endif
	}

	if (cmd != NAND_CMD_NONE)
		writeb(cmd, chip->IO_ADDR_W);
}

static int s3c24x0_dev_ready(struct mtd_info *mtd)
{
	struct s3c2410_nand *nand = s3c2410_get_base_nand();
	debug("dev_ready\n");
	return readl(&nand->nfstat) & 0x01;
}

#ifdef CONFIG_S3C2410_NAND_HWECC
void s3c2410_nand_enable_hwecc(struct mtd_info *mtd, int mode)
{
	struct s3c2410_nand *nand = s3c2410_get_base_nand();
	debug("s3c2410_nand_enable_hwecc(%p, %d)\n", mtd, mode);
	writel(readl(&nand->nfconf) | S3C2410_NFCONF_INITECC, &nand->nfconf);
}

static int s3c2410_nand_calculate_ecc(struct mtd_info *mtd, const u_char *dat,
				      u_char *ecc_code)
{

	struct s3c2410_nand *nand = s3c2410_get_base_nand();
	ecc_code[0] = readb(&nand->nfecc);
	ecc_code[1] = readb(&nand->nfecc + 1);
	ecc_code[2] = readb(&nand->nfecc + 2);
	debug("s3c2410_nand_calculate_hwecc(%p,): 0x%02x 0x%02x 0x%02x\n",
	       mtd , ecc_code[0], ecc_code[1], ecc_code[2]);

	return 0;
}

static int s3c2410_nand_correct_data(struct mtd_info *mtd, u_char *dat,
				     u_char *read_ecc, u_char *calc_ecc)
{
	if (read_ecc[0] == calc_ecc[0] &&
	    read_ecc[1] == calc_ecc[1] &&
	    read_ecc[2] == calc_ecc[2])
		return 0;

	printf("s3c2410_nand_correct_data: not implemented\n");
	return -1;
}
#endif

int board_nand_init(struct nand_chip *nand)
{
	u_int32_t cfg=0;
	u_int8_t tacls, twrph0, twrph1;
	struct s3c24x0_clock_power *clk_power = s3c24x0_get_base_clock_power();
	struct NAND_TYPE *nand_reg = GET_NAND_BASE();

	debug("board_nand_init()\n");

	writel(readl(&clk_power->clkcon) | (1 << 4), &clk_power->clkcon);

	/* initialize hardware */
#if defined(CONFIG_S3C24XX_CUSTOM_NAND_TIMING)
	tacls  = CONFIG_S3C24XX_TACLS;
	twrph0 = CONFIG_S3C24XX_TWRPH0;
	twrph1 =  CONFIG_S3C24XX_TWRPH1;
#else
	tacls = 7; //4;
	twrph0 = 7; //8;
	twrph1 = 7; //8;
#endif
        
#ifdef CONFIG_S3C2410
	cfg = S3C24x0_NFCONF_EN;
	cfg |= S3C24x0_NFCONF_TACLS(tacls - 1);
	cfg |= S3C24x0_NFCONF_TWRPH0(twrph0 - 1);
	cfg |= S3C24x0_NFCONF_TWRPH1(twrph1 - 1);
	writel(cfg, &nand_reg->nfconf);
#elif defined(CONFIG_S3C2440)
        cfg = 0;
	cfg |= S3C24x0_NFCONF_TACLS(tacls);
	cfg |= S3C24x0_NFCONF_TWRPH0(twrph0);
	cfg |= S3C24x0_NFCONF_TWRPH1(twrph1);
	writel(cfg, &nand_reg->nfconf);

	cfg = S3C24x0_NFCONF_EN;
//        cfg |= S3C24x0_NFCONT_nFCE;
	writel(cfg, &nand_reg->nfcont);
#endif

	/* initialize nand_chip data structure */
	nand->IO_ADDR_R = nand->IO_ADDR_W = (void *)&nand_reg->nfdata;

	nand->select_chip = NULL;

	/* read_buf and write_buf are default */
	/* read_byte and write_byte are default */
#ifdef CONFIG_NAND_SPL
	nand->read_buf = nand_read_buf;
#endif

	/* hwcontrol always must be implemented */
	nand->cmd_ctrl = s3c24x0_hwcontrol;

	nand->dev_ready = s3c24x0_dev_ready;

#ifdef CONFIG_S3C2410_NAND_HWECC
	nand->ecc.hwctl = s3c2410_nand_enable_hwecc;
	nand->ecc.calculate = s3c2410_nand_calculate_ecc;
	nand->ecc.correct = s3c2410_nand_correct_data;
	nand->ecc.mode = NAND_ECC_HW;
	nand->ecc.size = CONFIG_SYS_NAND_ECCSIZE;
	nand->ecc.bytes = CONFIG_SYS_NAND_ECCBYTES;
#else
	nand->ecc.mode = NAND_ECC_SOFT;
#endif

#ifdef CONFIG_S3C2410_NAND_BBT
	nand->options = NAND_USE_FLASH_BBT;
#else
	nand->options = 0;
#endif

	debug("end of nand_init\n");

	return 0;
}
