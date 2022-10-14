/*
 * Copyright (c) 2018 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/compat.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_address.h>
#include <linux/delay.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/device.h>
#include <linux/sysfs.h>
#include <linux/fs.h>
#include <linux/major.h>
#include <linux/kobject.h>
#include <linux/dma-mapping.h>
#include <linux/cdev.h>
#include <linux/uaccess.h>
#include <linux/list.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <asm/fb.h>
#include <linux/completion.h>

#define VCD_VERSION "0.0.15"

#define VCD_IOC_MAGIC		'v'
#define VCD_IOCGETINFO		_IOR(VCD_IOC_MAGIC,  1, struct vcd_info)
#define VCD_IOCSENDCMD		_IOW(VCD_IOC_MAGIC,  2, unsigned int)
#define VCD_IOCCHKRES		_IOR(VCD_IOC_MAGIC,  3, int)
#define VCD_IOCGETDIFF		_IOR(VCD_IOC_MAGIC,  4, struct rect)
#define VCD_IOCDIFFCNT		_IOR(VCD_IOC_MAGIC,  5, int)
#define VCD_IOCDEMODE		_IOR(VCD_IOC_MAGIC,  6, int)
#define VCD_IOCRESET		_IO(VCD_IOC_MAGIC, 7)
#define VCD_GETREG		_IOR(VCD_IOC_MAGIC, 8, struct vcd_info)
#define VCD_SETREG		_IOW(VCD_IOC_MAGIC, 9, struct vcd_info)
#define VCD_SHORT_RESET		_IO(VCD_IOC_MAGIC, 10)
#define VCD_IOCSETDISPLAY	_IOW(VCD_IOC_MAGIC, 11, unsigned int)
#define VCD_IOC_MAXNR		11

#define VCD_OP_TIMEOUT msecs_to_jiffies(100)

#define DEVICE_NAME "vcd"

#define RECT_TILE_W	16
#define RECT_TILE_H	16
#define VCD_INIT_WIDTH	640
#define VCD_INIT_HIGHT	480
#define VCD_MAX_WIDTH	2047
#define VCD_MAX_HIGHT	1536
#define VCD_MIN_LP	512
#define VCD_MAX_LP	4096

/* VCD  Register */
#define VCD_DIFF_TBL 0x0000
#define VCD_FBA_ADR	0x8000
#define VCD_FBB_ADR	0x8004

#define VCD_FB_LP	0x8008
#define  VCD_FB_LP_MASK 0xffff
#define  VCD_FBB_LP_OFFSET 16

#define VCD_CAP_RES	0x800C
#define  VCD_CAPRES_MASK 0x7ff

#define VCD_DVO_DEL 0x8010
#define  VCD_DVO_DEL_VERT_HOFF GENMASK(31, 27)
#define  VCD_DVO_DEL_MASK 0x7ff
#define  VCD_DVO_DEL_VERT_HOFF_OFFSET 27
#define  VCD_DVO_DEL_VSYNC_DEL_OFFSET 16
#define  VCD_DVO_DEL_HSYNC_DEL_OFFSET 0

#define VCD_MODE	0x8014
#define  VCD_MODE_VCDE	BIT(0)
#define  VCD_MODE_CM565	BIT(1)
#define  VCD_MODE_IDBC	BIT(3)
#define  VCD_MODE_COLOR_CNVRT	GENMASK(5, 4)
#define  VCD_MODE_DAT_INV	BIT(6)
#define  VCD_MODE_CLK_EDGE	BIT(8)
#define  VCD_MODE_HS_EDGE	BIT(9)
#define  VCD_MODE_VS_EDGE	BIT(10)
#define  VCD_MODE_DE_HS		BIT(11)
#define  VCD_MODE_KVM_BW_SET	BIT(16)
#define  VCD_MODE_COLOR_NORM	0x0
#define  VCD_MODE_COLOR_222		0x1
#define  VCD_MODE_COLOR_666		0x2
#define  VCD_MODE_COLOR_888		0x3
#define  VCD_MODE_CM_555		0x0
#define  VCD_MODE_CM_565		0x1
#define  VCD_MODE_COLOR_CNVRT_OFFSET 4

#define VCD_CMD			0x8018
#define  VCD_CMD_GO	BIT(0)
#define  VCD_CMD_RST	BIT(1)
#define  VCD_CMD_OP_MASK	0x70
#define  VCD_CMD_OP_OFFSET	4
#define  VCD_CMD_OP_CAPTURE	0
#define  VCD_CMD_OP_COMPARE_TWO	1
#define  VCD_CMD_OP_COMPARE	2

#define	VCD_STAT		0x801C
#define	 VCD_STAT_IRQ	BIT(31)
#define	 VCD_STAT_BUSY	BIT(30)
#define	 VCD_STAT_BSD3	BIT(13)
#define	 VCD_STAT_BSD2	BIT(12)
#define	 VCD_STAT_HSYNC	BIT(11)
#define	 VCD_STAT_VSYNC	BIT(10)
#define	 VCD_STAT_HLC_CHG	BIT(9)
#define	 VCD_STAT_HAC_CHG	BIT(8)
#define	 VCD_STAT_HHT_CHG	BIT(7)
#define	 VCD_STAT_HCT_CHG	BIT(6)
#define	 VCD_STAT_VHT_CHG	BIT(5)
#define	 VCD_STAT_VCT_CHG	BIT(4)
#define	 VCD_STAT_IFOR	BIT(3)
#define	 VCD_STAT_IFOT	BIT(2)
#define	 VCD_STAT_BSD1	BIT(1)
#define	 VCD_STAT_DONE	BIT(0)
#define	 VCD_STAT_CLEAR	0x3FFF
#define	 VCD_STAT_CURR_LINE_OFFSET 16
#define	 VCD_STAT_CURR_LINE 0x7ff0000

#define VCD_INTE	0x8020
#define  VCD_INTE_DONE_IE	BIT(0)
#define  VCD_INTE_BSD_IE	BIT(1)
#define  VCD_INTE_IFOT_IE	BIT(2)
#define  VCD_INTE_IFOR_IE	BIT(3)
#define  VCD_INTE_VCT_CHG_IE	BIT(4)
#define  VCD_INTE_VHT_CHG_IE	BIT(5)
#define  VCD_INTE_HCT_CHG_IE	BIT(6)
#define  VCD_INTE_HHT_CHG_IE	BIT(7)
#define  VCD_INTE_HAC_CHG_IE	BIT(8)
#define  VCD_INTE_HLC_CHG_IE	BIT(9)
#define  VCD_INTE_VSYNC_IE	BIT(10)
#define  VCD_INTE_HSYNC_IE	BIT(11)
#define  VCD_INTE_BSD2_IE	BIT(12)
#define  VCD_INTE_BSD3_IE	BIT(13)
#define  VCD_INTE_VAL	(VCD_INTE_DONE_IE)

#define VCD_RCHG	0x8028
#define VCD_RCHG_TIM_PRSCL_OFFSET 9
#define VCD_RCHG_IG_CHG2_OFFSET 6
#define VCD_RCHG_IG_CHG1_OFFSET 3
#define VCD_RCHG_IG_CHG0_OFFSET 0
#define VCD_RCHG_TIM_PRSCL  GENMASK(12, VCD_RCHG_TIM_PRSCL_OFFSET)
#define VCD_RCHG_IG_CHG2  GENMASK(8, VCD_RCHG_IG_CHG2_OFFSET)
#define VCD_RCHG_IG_CHG1  GENMASK(5, VCD_RCHG_IG_CHG1_OFFSET)
#define VCD_RCHG_IG_CHG0  GENMASK(2, VCD_RCHG_IG_CHG0_OFFSET)

#define VCD_HOR_CYC_TIM	0x802C
#define VCD_HOR_CYC_TIM_NEW	BIT(31)
#define VCD_HOR_CYC_TIM_HCT_DIF	BIT(30)
#define VCD_HOR_CYC_TIM_VALUE	GENMASK(11, 0)
#define VCD_HOR_AC_TIM_MASK     0x3fff

#define VCD_HOR_CYC_LAST	0x8030
#define VCD_HOR_CYC_LAST_VALUE	GENMASK(11, 0)

#define VCD_HOR_HI_TIM	0x8034
#define VCD_HOR_HI_TIM_NEW	BIT(31)
#define VCD_HOR_HI_TIM_HHT_DIF	BIT(30)
#define VCD_HOR_HI_TIM_VALUE	GENMASK(11, 0)

#define VCD_HOR_HI_LAST	0x8038
#define VCD_HOR_HI_LAST_VALUE	GENMASK(11, 0)

#define VCD_VER_CYC_TIM	0x803C
#define VCD_VER_CYC_TIM_NEW	BIT(31)
#define VCD_VER_CYC_TIM_VCT_DIF	BIT(30)
#define VCD_VER_CYC_TIM_VALUE	GENMASK(23, 0)

#define VCD_VER_CYC_LAST	0x8040
#define VCD_VER_CYC_LAST_VALUE	GENMASK(23, 0)

#define VCD_VER_HI_TIM	0x8044
#define VCD_VER_HI_TIM_NEW	BIT(31)
#define VCD_VER_HI_TIM_VHT_DIF	BIT(30)
#define VCD_VER_HI_TIM_VALUE	GENMASK(23, 0)

#define VCD_VER_HI_LAST	0x8048
#define VCD_VER_HI_LAST_VALUE	GENMASK(23, 0)

#define VCD_HOR_AC_TIM	0x804C
#define VCD_HOR_AC_TIM_NEW	BIT(31)
#define VCD_HOR_AC_TIM_HAC_DIF	BIT(30)
#define VCD_HOR_AC_TIM_VALUE	GENMASK(13, 0)

#define VCD_HOR_AC_LAST	0x8050
#define VCD_HOR_AC_LAST_VALUE	GENMASK(13, 0)

#define VCD_HOR_LIN_TIM	0x8054
#define VCD_HOR_LIN_TIM_NEW	BIT(31)
#define VCD_HOR_LIN_TIM_HLC_DIF	BIT(30)
#define VCD_HOR_LIN_TIM_VALUE	GENMASK(11, 0)

#define VCD_HOR_LIN_LAST	0x8058
#define VCD_HOR_LIN_LAST_VALUE	GENMASK(11, 0)

#define VCD_FIFO		0x805C
#define  VCD_FIFO_TH	0x100350ff

/* GCR  Register */
#define INTCR 0x3c
#define  INTCR_GFXIFDIS	(BIT(8) | BIT(9))
#define  INTCR_GIRST	BIT(17)
#define  INTCR_LDDRB	BIT(18)
#define  INTCR_DACOFF	BIT(15)
#define  INTCR_DEHS	BIT(27)
#define  INTCR_KVMSI	BIT(28)

#define INTCR2 0x60
#define  INTCR2_GIRST2	BIT(2)
#define  INTCR2_GIHCRST	BIT(5)
#define  INTCR2_GIVCRST	BIT(6)

#define INTCR3 0x9c
#define INTCR3_GMMAP_MASK	GENMASK(10, 8)
#define INTCR3_GMMAP_128MB	0x00000000
#define INTCR3_GMMAP_256MB	0x00000100
#define INTCR3_GMMAP_512MB	0x00000200
#define INTCR3_GMMAP_1GB	0x00000300
#define INTCR3_GMMAP_2GB	0x00000400

#define ADDR_GMMAP_128MB	0x07000000
#define ADDR_GMMAP_256MB	0x0f000000
#define ADDR_GMMAP_512MB	0x1f000000
#define ADDR_GMMAP_1GB		0x3f000000
#define ADDR_GMMAP_2GB		0x7f000000

/* Total 16MB, but 4MB preserved*/
#define GMMAP_LENGTH	0xC00000

#define MFSEL1 0x0c
#define  MFSEL1_DVH1SEL	BIT(27)

/* GFXI Register */
#define GFXI_START 0xE000
#define GFXI_FIFO 0xE050
#define GFXI_MASK 0x00FF

#define DISPST	0
#define  DISPST_MGAMODE BIT(7)
#define  DISPST_HSCROFF BIT(1)

#define HVCNTL	0x10
#define  HVCNTL_MASK	0xff
#define HVCNTH	0x14
#define  HVCNTH_MASK	0x07
#define HBPCNTL	0x18
#define  HBPCNTL_MASK	0xff
#define HBPCNTH	0x1c
#define  HBPCNTH_MASK	0x01
#define VVCNTL	0x20
#define  VVCNTL_MASK	0xff
#define VVCNTH	0x24
#define  VVCNTH_MASK	0x07
#define VPBCNTL	0x28
#define  VPBCNTL_MASK	0xff
#define VPBCNTH	0x2C
#define  VPBCNTH_MASK	0x01

#define GPLLINDIV	0x40
#define  GPLLINDIV_MASK	0x3f
#define  GPLLFBDV8_MASK	0x80
#define  GPLLINDIV_OFFSET	0
#define  GPLLFBDV8_OFFSET	7

#define GPLLFBDIV	0x44
#define  GPLLFBDIV_MASK	0xff

#define GPLLST	0x48
#define  GPLLFBDV109_MASK	0xc0
#define  GPLLFBDV109_OFFSET	6
#define  GPLLST_PLLOTDIV1_MASK	0x07
#define  GPLLST_PLLOTDIV2_MASK	0x38
#define  GPLLST_PLLOTDIV1_OFFSET	0
#define  GPLLST_PLLOTDIV2_OFFSET	3

#define VCD_R_MAX	31
#define VCD_G_MAX	63
#define VCD_B_MAX	31
#define VCD_R_SHIFT	11
#define VCD_G_SHIFT	5
#define VCD_B_SHIFT	0

#define VCD_KVM_BW_PCLK 120000000UL

#define GET_RES_RERTY			3
#define GET_RES_TIMEOUT	300
#define GET_RES_VALID_RERTY		3
#define GET_RES_VALID_TIMEOUT	100

struct class *vcd_class;
static const char vcd_name[] = "NPCM750 VCD";

struct vcd_info {
	u32 vcd_fb;
	u32 pixelclk;
	u32 line_pitch;
	u32 hdisp;
	u32 hfrontporch;
	u32 hsync;
	u32 hbackporch;
	u32 vdisp;
	u32 vfrontporch;
	u32 vsync;
	u32 vbackporch;
	u32 refresh_rate;
	u32 hpositive;
	u32 vpositive;
	u32 bpp;
	u32 r_max;
	u32 g_max;
	u32 b_max;
	u32 r_shift;
	u32 g_shift;
	u32 b_shift;
	u32 mode;
	u32 reg;
	u32 reg_val;
};

struct rect {
	u32 x;
	u32 y;
	u32 w;
	u32 h;
};

struct rect_list {
	struct rect r;
	struct list_head list;
};

struct rect_info {
	struct rect_list *list;
	struct rect_list *first;
	struct list_head *head;
	int index;
	int tile_perline;
	int tile_perrow;
	int offset_perline;
	int tile_size;
	int tile_cnt;
};

struct npcm750_vcd {
	struct mutex mlock; /*for iotcl*/
	spinlock_t lock;	/*for irq*/
	struct device *dev;
	struct device *dev_p;
	struct cdev dev_cdev;
	struct vcd_info info;
	struct list_head list;
	void __iomem *base;
	struct regmap *vcd_regmap;
	struct regmap *gcr_regmap;
	struct regmap *gfx_regmap;
	u32 frame_len;
	u32 frame_start;
	u32 rect_cnt;
	u32 status;
	char *video_name;
	int cmd;
	dev_t dev_t;
	atomic_t clients;
	int de_mode;
	int irq;
	struct completion complete;
	u32 hortact;
};

typedef struct
{
	u8* name;
	u32 hdisp;	// displayed pixels i.e. width
	u32 vdisp;	// displayed lines i.e. height
} res_tlb;

static const res_tlb res_tlbs[] = {
	{"320 x 200", 320, 200},
	{"320 x 240", 320, 240},
	{"640 x 480", 640, 480},
	{"720 x 400", 720, 400},
	{"768 x 576", 768, 576},
	{"800 x 480", 800, 480},
	{"800 x 600", 800, 600},
	{"832 x 624", 832, 624},
	{"848 x 480", 848, 480},
	{"854 x 480", 854, 480},
	{"1024 x 600", 1024, 600},
	{"1024 x 768", 1024, 768},
	{"1152 x 768", 1152, 768},
	{"1152 x 864", 1152, 864},
	{"1152 x 870", 1152, 870},
	{"1152 x 900", 1152, 900},
	{"1280 x 720", 1280, 720},
	{"1280 x 768", 1280, 768},
	{"1280 x 800", 1280, 800},
	{"1280 x 854", 1280, 854},
	{"1280 x 960", 1280, 960},
	{"1280 x 1024", 1280, 1024},
	{"1360 x 768", 1360, 768},
	{"1366 x 768", 1366, 768},
	{"1440 x 900", 1440, 900},
	{"1440 x 960", 1440, 960},
	{"1440 x 1050", 1440, 1050},
	{"1440 x 1080", 1440, 1080},
	{"1600 x 900", 1600, 900},
	{"1600 x 1050", 1600, 1050},
	{"1600 x 1200", 1600, 1200},
	{"1680 x 1050", 1680, 1050},
	{"1920 x 1080", 1920, 1080},
	{"1920 x 1200", 1920, 1200},
};

static const size_t restlb_cnt = sizeof(res_tlbs) / sizeof(res_tlb);

static void npcm750_vcd_claer_gmmap(struct npcm750_vcd *priv)
{
	struct regmap *gcr = priv->gcr_regmap;
	u32 intcr3, gmmap;
	void __iomem *baseptr;

	regmap_read(gcr, INTCR3, &intcr3);
	gmmap = (intcr3 & INTCR3_GMMAP_MASK);

	switch (gmmap){
	case INTCR3_GMMAP_128MB:
		baseptr = ioremap(ADDR_GMMAP_128MB, GMMAP_LENGTH);
		break;
	case INTCR3_GMMAP_256MB:
		baseptr = ioremap(ADDR_GMMAP_256MB, GMMAP_LENGTH);
		break;
	case INTCR3_GMMAP_512MB:
		baseptr = ioremap(ADDR_GMMAP_512MB, GMMAP_LENGTH);
		break;
	case INTCR3_GMMAP_1GB:
		baseptr = ioremap(ADDR_GMMAP_1GB, GMMAP_LENGTH);
		break;
	case INTCR3_GMMAP_2GB:
		baseptr = ioremap(ADDR_GMMAP_2GB, GMMAP_LENGTH);
		break;
	}

	memset(baseptr, 0, GMMAP_LENGTH);
	iounmap(baseptr);
}

static u8 npcm750_vcd_is_mga(struct npcm750_vcd *priv)
{
	struct regmap *gfxi = priv->gfx_regmap;
	u32 dispst;

	regmap_read(gfxi, DISPST, &dispst);
	return ((dispst & DISPST_MGAMODE) == DISPST_MGAMODE);
}

static u32 npcm750_vcd_hres(struct npcm750_vcd *priv)
{
	struct regmap *gfxi = priv->gfx_regmap;
	u32 hvcnth, hvcntl, apb_hor_res;

	regmap_read(gfxi, HVCNTH, &hvcnth);
	regmap_read(gfxi, HVCNTL, &hvcntl);

	apb_hor_res = (((hvcnth & HVCNTH_MASK) << 8)
		+ (hvcntl & HVCNTL_MASK) + 1);

	return (apb_hor_res > VCD_MAX_WIDTH) ?
		VCD_MAX_WIDTH : apb_hor_res;
}

static u32 npcm750_vcd_vres(struct npcm750_vcd *priv)
{
	struct regmap *gfxi = priv->gfx_regmap;
	u32 vvcnth, vvcntl, apb_ver_res;

	regmap_read(gfxi, VVCNTH, &vvcnth);
	regmap_read(gfxi, VVCNTL, &vvcntl);

	apb_ver_res = (((vvcnth & VVCNTH_MASK) << 8)
		+ (vvcntl & VVCNTL_MASK));

	return (apb_ver_res > VCD_MAX_HIGHT) ?
		VCD_MAX_HIGHT : apb_ver_res;
}

static void npcm750_vcd_local_display(struct npcm750_vcd *priv, u8 enable)
{
	struct regmap *gcr = priv->gcr_regmap;

	if (enable) {
		regmap_update_bits(gcr, INTCR, INTCR_LDDRB, ~INTCR_LDDRB);
		regmap_update_bits(gcr, INTCR, INTCR_DACOFF, ~INTCR_DACOFF);
	} else {
		regmap_update_bits(gcr, INTCR, INTCR_LDDRB, INTCR_LDDRB);
		regmap_update_bits(gcr, INTCR, INTCR_DACOFF, INTCR_DACOFF);
	}
}

static int npcm750_vcd_dvod(struct npcm750_vcd *priv, u32 hdelay, u32 vdelay)
{
	struct regmap *vcd = priv->vcd_regmap;

	regmap_write(vcd, VCD_DVO_DEL,
			(hdelay & VCD_DVO_DEL_MASK) |
			((vdelay & VCD_DVO_DEL_MASK) << VCD_DVO_DEL_VSYNC_DEL_OFFSET));

	return 0;
}

static int npcm750_vcd_get_bpp(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 mode, color_cnvr;

	regmap_read(vcd, VCD_MODE, &mode);

	color_cnvr = (mode & VCD_MODE_COLOR_CNVRT) >>
		VCD_MODE_COLOR_CNVRT_OFFSET;

	switch (color_cnvr) {
	case VCD_MODE_COLOR_NORM:
		return 2;
	case VCD_MODE_COLOR_222:
	case VCD_MODE_COLOR_666:
		return 1;
	case VCD_MODE_COLOR_888:
		return 4;
	}
	return 0;
}

static void npcm750_vcd_set_linepitch(struct npcm750_vcd *priv, u32 linebytes)
{
	struct regmap *vcd = priv->vcd_regmap;
	/* Pitch must be a power of 2, >= linebytes,*/
	/* at least 512, and no more than 4096. */
	u32 pitch = VCD_MIN_LP;

	while ((pitch < linebytes) && (pitch < VCD_MAX_LP))
		pitch *= 2;

	regmap_write(vcd, VCD_FB_LP, (pitch << VCD_FBB_LP_OFFSET) | pitch);
}

static u32 npcm750_vcd_get_linepitch(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 linepitch;

	regmap_read(vcd, VCD_FB_LP, &linepitch);

	return linepitch & VCD_FB_LP_MASK;
}

static int npcm750_vcd_ready(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 lp, res;

	regmap_write(vcd, VCD_FB_LP, 0xffffffff);
	regmap_write(vcd, VCD_CAP_RES, 0xffffffff);

	regmap_read(vcd, VCD_FB_LP, &lp);
	regmap_read(vcd, VCD_CAP_RES, &res);

	if ((lp != 0xfe00fe00) || (res != 0x7ff07ff)) {
		dev_err(priv->dev, "vcd hw is not ready\n");
		return -ENODEV;
	}

	return 0;
}

static u32 npcm750_vcd_pclk(struct npcm750_vcd *priv)
{
	struct regmap *gfxi = priv->gfx_regmap;
	u32 tmp, pllfbdiv, pllinotdiv, gpllfbdiv;
	u8 gpllfbdv109, gpllfbdv8, gpllindiv;
	u8 gpllst_pllotdiv1, gpllst_pllotdiv2;

	regmap_read(gfxi, GPLLST, &tmp);
	gpllfbdv109 = (tmp & GPLLFBDV109_MASK) >> GPLLFBDV109_OFFSET;
	gpllst_pllotdiv1 = tmp & GPLLST_PLLOTDIV1_MASK;
	gpllst_pllotdiv2 =
		(tmp & GPLLST_PLLOTDIV2_MASK) >> GPLLST_PLLOTDIV2_OFFSET;

	regmap_read(gfxi, GPLLINDIV, &tmp);
	gpllfbdv8 = (tmp & GPLLFBDV8_MASK) >> GPLLFBDV8_OFFSET;
	gpllindiv = (tmp & GPLLINDIV_MASK);

	regmap_read(gfxi, GPLLFBDIV, &tmp);
	gpllfbdiv = tmp & GPLLFBDIV_MASK;

	pllfbdiv = (512 * gpllfbdv109 + 256 * gpllfbdv8 + gpllfbdiv);
	pllinotdiv = (gpllindiv * gpllst_pllotdiv1 * gpllst_pllotdiv2);
	if (pllfbdiv == 0 || pllinotdiv == 0)
		return 0;

	return ((pllfbdiv * 25) / pllinotdiv) * 1000;
}

static int
npcm750_vcd_capres(struct npcm750_vcd *priv, u32 width, u32 height)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 res = (height & VCD_CAPRES_MASK)
		| ((width & VCD_CAPRES_MASK) << 16);
	u32 cap_res;

	if ((width > VCD_MAX_WIDTH) || (height > VCD_MAX_HIGHT))
		return -EINVAL;

	regmap_write(vcd, VCD_CAP_RES, res);
	regmap_read(vcd, VCD_CAP_RES, &cap_res);

	/* Read back the register to check that the values were valid */
	if (cap_res !=  res)
		return -EINVAL;

	return 0;
}
static void
npcm750_short_vcd_reset(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 stat;

	regmap_write(vcd, VCD_INTE, 0);

	regmap_update_bits(vcd, VCD_CMD, VCD_CMD_RST, VCD_CMD_RST);
	while (!regmap_read(vcd, VCD_STAT, &stat) & !(stat & VCD_STAT_DONE))
		continue;

	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);
	regmap_write(vcd, VCD_INTE, VCD_INTE_VAL);
}

static int npcm750_vcd_reset(struct npcm750_vcd *priv)
{
	struct regmap *gcr = priv->gcr_regmap;
	struct regmap *vcd = priv->vcd_regmap;
	u32 stat;

	regmap_update_bits(vcd, VCD_CMD, VCD_CMD_RST, VCD_CMD_RST);

	while (!regmap_read(vcd, VCD_STAT, &stat) & !(stat & VCD_STAT_DONE))
		continue;

	/* Active graphic reset */
	regmap_update_bits(
		gcr, INTCR2, INTCR2_GIRST2, INTCR2_GIRST2);

	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	/* Inactive graphic reset */
	regmap_update_bits(
		gcr, INTCR2, INTCR2_GIRST2, ~INTCR2_GIRST2);

	return 0;
}

static void npcm750_vcd_io_reset(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;

	regmap_write(vcd, VCD_INTE, 0);
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);
	npcm750_vcd_reset(priv);
	regmap_write(vcd, VCD_INTE, VCD_INTE_VAL);
}

static void npcm750_vcd_dehs(struct npcm750_vcd *priv, int is_de)
{
	struct regmap *gcr = priv->gcr_regmap;
	struct regmap *vcd = priv->vcd_regmap;

	if (is_de) {
		regmap_update_bits(
			vcd, VCD_MODE, VCD_MODE_DE_HS, ~VCD_MODE_DE_HS);
		regmap_update_bits(
			gcr, INTCR, INTCR_DEHS, ~INTCR_DEHS);
	} else {
		regmap_update_bits(
			vcd, VCD_MODE, VCD_MODE_DE_HS, VCD_MODE_DE_HS);
		regmap_update_bits(
			gcr, INTCR, INTCR_DEHS, INTCR_DEHS);
	}
}

static void npcm750_vcd_kvm_bw(struct npcm750_vcd *priv, u8 bandwidth)
{
	struct regmap *vcd = priv->vcd_regmap;

	if (!npcm750_vcd_is_mga(priv))
		bandwidth = 1;

	if (bandwidth)
		regmap_update_bits(
			vcd,
			VCD_MODE,
			VCD_MODE_KVM_BW_SET,
			VCD_MODE_KVM_BW_SET);
	else
		regmap_update_bits(
			vcd,
			VCD_MODE,
			VCD_MODE_KVM_BW_SET,
			~VCD_MODE_KVM_BW_SET);
}

static void npcm750_vcd_update_info(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;

	priv->info.hdisp = npcm750_vcd_hres(priv);
	priv->info.vdisp = npcm750_vcd_vres(priv);
	priv->video_name = "Digital";
	priv->info.pixelclk = npcm750_vcd_pclk(priv);
	priv->info.bpp = npcm750_vcd_get_bpp(priv);
	priv->info.mode = npcm750_vcd_is_mga(priv);
	priv->info.refresh_rate = 60;
	priv->info.hfrontporch = 0;
	priv->info.hbackporch = 0;
	priv->info.vfrontporch = 0;
	priv->info.vbackporch = 0;
	priv->info.hpositive = 1;
	priv->info.vpositive = 0;

	if (priv->info.hdisp > VCD_MAX_WIDTH)
		priv->info.hdisp = VCD_MAX_WIDTH;

	if (priv->info.vdisp > VCD_MAX_HIGHT)
		priv->info.vdisp = VCD_MAX_HIGHT;

	regmap_read(vcd, VCD_HOR_AC_TIM, &priv->hortact);
	priv->hortact &= VCD_HOR_AC_TIM_MASK;
}

static void npcm750_vcd_detect_video_mode(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 mode;

	npcm750_vcd_capres(priv, priv->info.hdisp, priv->info.vdisp);

	npcm750_vcd_set_linepitch(
		priv, priv->info.hdisp * npcm750_vcd_get_bpp(priv));

	priv->info.line_pitch = npcm750_vcd_get_linepitch(priv);

	npcm750_vcd_kvm_bw(priv, priv->info.pixelclk > VCD_KVM_BW_PCLK);

	npcm750_vcd_reset(priv);

	regmap_read(vcd, VCD_MODE, &mode);

	dev_dbg(priv->dev, "VCD Mode = 0x%x, %s mode\n", mode,
		npcm750_vcd_is_mga(priv) ? "Hi Res" : "VGA");

	dev_dbg(priv->dev, "Resolution: %d x %d, Pixel Clk %zuKHz, Line Pitch %d\n",
			priv->info.hdisp, priv->info.vdisp,
			priv->info.pixelclk,
			priv->info.line_pitch);
}

static u8 npcm750_vcd_is_busy(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 stat;

	regmap_read(vcd, VCD_STAT, &stat);
	stat &= VCD_STAT_BUSY;

	return (stat == VCD_STAT_BUSY);
}

static u8 npcm750_vcd_op_done(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 vdisp;
	u32 curline = ((priv->status & VCD_STAT_CURR_LINE)
		>> VCD_STAT_CURR_LINE_OFFSET);

	regmap_read(vcd, VCD_CAP_RES, &vdisp);
	vdisp &= VCD_CAPRES_MASK;

	return ((priv->status & VCD_STAT_DONE) &&
		!(priv->status & VCD_STAT_BUSY) &&
		(curline == vdisp));
}

static int npcm750_vcd_command(struct npcm750_vcd *priv, u32 value)
{
	struct regmap *vcd = priv->vcd_regmap;
	u32 cmd;

	if (npcm750_vcd_is_busy(priv))
		/* Not ready for another command */
		return -EBUSY;

	/* Clear the status flags that could be set by this command */
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	regmap_read(vcd, VCD_CMD, &cmd);

	cmd &= ~VCD_CMD_OP_MASK;
	cmd |= (value << VCD_CMD_OP_OFFSET);

	regmap_write(vcd, VCD_CMD, cmd);
	regmap_write(vcd, VCD_CMD, cmd | VCD_CMD_GO);
	priv->cmd = value;

	return 0;
}

static int npcm750_vcd_get_resolution(struct npcm750_vcd *priv)
{
	struct regmap *vcd = priv->vcd_regmap;
	size_t i;
	u32 res_retry = GET_RES_RERTY;
	u32 vaild_retry = GET_RES_VALID_RERTY;
	u32 hortact;
	u8 vaild = 0;

	regmap_read(vcd, VCD_HOR_AC_TIM, &hortact);
	hortact &= VCD_HOR_AC_TIM_MASK;

	/* check with GFX registers if resolution changed from last time */
	if ((priv->info.hdisp != npcm750_vcd_hres(priv)) ||
		(priv->info.vdisp != npcm750_vcd_vres(priv)) ||
		(priv->info.pixelclk != npcm750_vcd_pclk(priv)) ||
		(priv->info.mode != npcm750_vcd_is_mga(priv)) ||
		(priv->hortact != hortact)) {

		regmap_write(vcd, VCD_INTE, 0);
		regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

		if (npcm750_vcd_hres(priv) && npcm750_vcd_vres(priv)) {
			struct regmap *gfxi = priv->gfx_regmap;
			u32 dispst;

			/* wait for resolution is available,
			   and it is also captured by host */
			do {
				if (res_retry == 0)
					return -1;
				mdelay(GET_RES_TIMEOUT);
				regmap_read(gfxi, DISPST, &dispst);
				res_retry--;
			} while (npcm750_vcd_vres(priv) < 100 ||
					npcm750_vcd_pclk(priv) == 0 ||
					(dispst & DISPST_HSCROFF));
		}

		/* wait for valid resolution */
		while (vaild_retry--) {
			for (i = 0 ; i < restlb_cnt ; i++) {
				if ((res_tlbs[i].hdisp == npcm750_vcd_hres(priv)) &&
					(res_tlbs[i].vdisp == npcm750_vcd_vres(priv))) {
					vaild = 1;
					break;
				}
			}
			if (vaild)
				break;
			else
				mdelay(GET_RES_VALID_TIMEOUT);
		}

		/* Update video information */
		npcm750_vcd_update_info(priv);

		if (!vaild) {
			dev_dbg(priv->dev, "invalid resolution %d x %d\n",
				npcm750_vcd_hres(priv), npcm750_vcd_vres(priv));
			if (npcm750_vcd_vres(priv) == 0)
				npcm750_vcd_claer_gmmap(priv);
			return -1;
		}

		/* setup resolution change detect register*/
		npcm750_vcd_detect_video_mode(priv);

		/* enable interrupt */
		regmap_write(vcd, VCD_INTE, VCD_INTE_VAL);

		return 1;
	}
	return 0;
}

static void npcm750_vcd_free_diff_table(struct npcm750_vcd *priv)
{
	struct list_head *head, *pos, *nx;
	struct rect_list *tmp;

	priv->rect_cnt = 0;

	head = &priv->list;
	list_for_each_safe(pos, nx, head) {
		tmp = list_entry(pos, struct rect_list, list);
		if (tmp) {
			list_del(&tmp->list);
			kfree(tmp);
		}
	}
}

static void
npcm750_vcd_merge_rect(struct npcm750_vcd *priv, struct rect_info *info)
{
	struct list_head *head = info->head;
	struct rect_list *list = info->list;
	struct rect_list *first = info->first;

	if (!first) {
		first = list;
		info->first = first;
		list_add_tail(&list->list, head);
		priv->rect_cnt++;
	} else {
		if (((list->r.x ==
		      (first->r.x + first->r.w))) &&
		      (list->r.y == first->r.y)) {
			first->r.w += list->r.w;
			kfree(list);
		} else if (((list->r.y ==
			     (first->r.y + first->r.h))) &&
			    (list->r.x == first->r.x)) {
			first->r.h += list->r.h;
			kfree(list);
		} else if (((list->r.y > first->r.y) &&
			    (list->r.y < (first->r.y + first->r.h))) &&
			   ((list->r.x > first->r.x) &&
			    (list->r.x < (first->r.x + first->r.w)))) {
			kfree(list);
		} else {
			list_add_tail(&list->list, head);
			priv->rect_cnt++;
			info->first = list;
		}
	}
}

static struct rect_list *
npcm750_vcd_new_rect(struct npcm750_vcd *priv, int offset, int index)
{
	struct rect_list *list = NULL;

	list = kmalloc(sizeof(*list), GFP_KERNEL);
	if (!list)
		return NULL;

	list->r.x = (offset << 4);
	list->r.y = (index >> 2);
	list->r.w = RECT_TILE_W;
	list->r.h = RECT_TILE_H;
	if ((list->r.x + RECT_TILE_W) > priv->info.hdisp)
		list->r.w = priv->info.hdisp - list->r.x;
	if ((list->r.y + RECT_TILE_H) > priv->info.vdisp)
		list->r.h = priv->info.vdisp - list->r.y;

	return list;
}

static int
npcm750_vcd_rect(struct npcm750_vcd *priv, struct rect_info *info, u32 offset)
{
	int i = info->index;

	if (offset < info->tile_perline) {
		info->list = npcm750_vcd_new_rect(priv, offset, i);
		if (!info->list)
			return -ENOMEM;

		npcm750_vcd_merge_rect(priv, info);
	}
	return 0;
}

static int
npcm750_vcd_build_table(struct npcm750_vcd *priv, struct rect_info *info)
{
	int i = info->index;
	int j, z, ret;
	u32 bitmap;
	struct regmap *vcd = priv->vcd_regmap;

	for (j = 0 ; j < info->offset_perline ; j += 4) {
		regmap_read(vcd, VCD_DIFF_TBL + (j + i), &bitmap);
		if (bitmap != 0) {
			for (z = 0 ; z < 32; z++) {
				if ((bitmap >> z) & 0x01) {
					ret = npcm750_vcd_rect(
							priv,
							info,
							z + (j << 3));
					if (ret < 0)
						return ret;
				}
			}
		}
	}
	info->index += 64;
	return info->tile_perline;
}

static int npcm750_vcd_get_diff_table(struct npcm750_vcd *priv)
{
	struct rect_info info;
	int ret = 0;
	u32 mod, tile_cnt = 0;

	memset(&info, 0, sizeof(struct rect_info));
	info.head = &priv->list;

	info.tile_perline = priv->info.hdisp >> 4;
	mod = priv->info.hdisp % RECT_TILE_W;
	if (mod != 0)
		info.tile_perline += 1;

	info.tile_perrow = priv->info.vdisp >> 4;
	mod = priv->info.vdisp % RECT_TILE_H;
	if (mod != 0)
		info.tile_perrow += 1;

	info.tile_size =
		info.tile_perrow * info.tile_perline;

	info.offset_perline = info.tile_perline >> 5;
	mod = info.tile_perline % 32;
	if (mod != 0)
		info.offset_perline += 1;

	info.offset_perline *= 4;

	do {
		ret = npcm750_vcd_build_table(priv, &info);
		if (ret < 0)
			return ret;
		tile_cnt += ret;
	} while (tile_cnt < info.tile_size);

	return ret;
}

static int npcm750_vcd_init(struct npcm750_vcd *priv)
{
	struct regmap *gcr = priv->gcr_regmap;
	struct regmap *vcd = priv->vcd_regmap;

	/* Enable display of KVM GFX and access to memory */
	regmap_update_bits(gcr, INTCR, INTCR_GFXIFDIS, ~INTCR_GFXIFDIS);

	/* KVM in progress */
	regmap_update_bits(gcr, INTCR, INTCR_KVMSI, INTCR_KVMSI);

	/* Set vrstenw and hrstenw */
	regmap_update_bits(gcr, INTCR2,
			   INTCR2_GIHCRST | INTCR2_GIVCRST,
		INTCR2_GIHCRST | INTCR2_GIVCRST);

	/* Select KVM GFX input */
	if (!priv->de_mode)
		regmap_update_bits(gcr, MFSEL1, MFSEL1_DVH1SEL, MFSEL1_DVH1SEL);
	else
		regmap_update_bits(gcr, MFSEL1, MFSEL1_DVH1SEL, ~MFSEL1_DVH1SEL);

	if (npcm750_vcd_ready(priv))
		return	-ENODEV;

	npcm750_vcd_reset(priv);

	/* Initialise capture resolution to a non-zero value */
	/* so that frame capture will behave sensibly before */
	/* the true resolution has been determined.*/
	if (npcm750_vcd_capres(priv, VCD_INIT_WIDTH, VCD_INIT_HIGHT)) {
		dev_err(priv->dev, "failed to set resolution\n");
		return -EINVAL;
	}

	/* Set the FIFO thresholds */
	regmap_write(vcd, VCD_FIFO, VCD_FIFO_TH);

	/* Set vcd frame physical address */
	regmap_write(vcd, VCD_FBA_ADR, priv->frame_start);
	regmap_write(vcd, VCD_FBB_ADR, priv->frame_start);

	/* Set vcd mode */
	regmap_update_bits(vcd, VCD_MODE, 0xFFFFFFFF,
			    VCD_MODE_CM_565 | VCD_MODE_KVM_BW_SET);

	/* Set DVDE/DVHSYNC */
	npcm750_vcd_dehs(priv, priv->de_mode);

	priv->info.vcd_fb = priv->frame_start;
	priv->info.r_max = VCD_R_MAX;
	priv->info.g_max = VCD_G_MAX;
	priv->info.b_max = VCD_B_MAX;
	priv->info.r_shift = VCD_R_SHIFT;
	priv->info.g_shift = VCD_G_SHIFT;
	priv->info.b_shift = VCD_B_SHIFT;

	/* Enable local disaply */
	npcm750_vcd_local_display(priv, 1);

	/* Update video information */
	npcm750_vcd_update_info(priv);

	/* Detect video mode */
	npcm750_vcd_detect_video_mode(priv);

	/* Enable interrupt */
	regmap_write(vcd, VCD_INTE, VCD_INTE_VAL);

	if (!priv->de_mode) {
		regmap_update_bits(vcd, VCD_RCHG, VCD_RCHG_TIM_PRSCL,
			0x01 << VCD_RCHG_TIM_PRSCL_OFFSET);
	} else {
		npcm750_vcd_dvod(priv, 0, 0);
		regmap_write(vcd, VCD_RCHG, 0);
	}

	return 0;
}

static void npcm750_vcd_stop(struct npcm750_vcd *priv)
{
	struct regmap *gcr = priv->gcr_regmap;
	struct regmap *vcd = priv->vcd_regmap;

	/* Disable display of KVM GFX and access to memory */
	regmap_update_bits(gcr, INTCR, INTCR_GFXIFDIS, INTCR_GFXIFDIS);

	/* KVM is not in progress */
	regmap_update_bits(gcr, INTCR, INTCR_KVMSI, ~INTCR_KVMSI);

	regmap_write(vcd, VCD_INTE, 0);
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);
	regmap_write(vcd, VCD_MODE, 0);
	regmap_write(vcd, VCD_RCHG, 0);

	npcm750_vcd_free_diff_table(priv);
	memset(&priv->info, 0, sizeof(struct vcd_info));
}

static irqreturn_t npcm750_vcd_irq_handler(int irq, void *dev_instance)
{
	struct device *dev = dev_instance;
	struct npcm750_vcd *priv = (struct npcm750_vcd *)dev->driver_data;
	struct regmap *vcd = priv->vcd_regmap;
	u32 status;
	u32 status_ack = 0;

	spin_lock(&priv->lock);

	regmap_read(vcd, VCD_STAT, &status);
	if (status & VCD_STAT_IRQ) {
		if (status & VCD_STAT_DONE) {
			dev_dbg(priv->dev, "VCD_STAT_DONE\n");
			status_ack |= VCD_STAT_DONE;
		}

		if (status & VCD_STAT_HSYNC) {
			dev_dbg(priv->dev, "VCD_STAT_HSYNC\n");
			status_ack |= VCD_STAT_HSYNC;
		}

		if (status & VCD_STAT_VSYNC) {
			dev_dbg(priv->dev, "VCD_STAT_VSYNC\n");
			status_ack |= VCD_STAT_VSYNC;
		}

		if (status & VCD_STAT_HAC_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_HAC_CHG\n");
			status_ack |= VCD_STAT_HAC_CHG;
		}

		if (status & VCD_STAT_HLC_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_HLC_CHG\n");
			status_ack |= VCD_STAT_HLC_CHG;
		}

		if (status & VCD_STAT_HHT_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_HHT_CHG\n");
			status_ack |= VCD_STAT_HHT_CHG;
		}

		if (status & VCD_STAT_HCT_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_HCT_CHG\n");
			status_ack |= VCD_STAT_HCT_CHG;
		}

		if (status & VCD_STAT_VHT_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_VHT_CHG\n");
			status_ack |= VCD_STAT_VHT_CHG;
		}

		if (status & VCD_STAT_VCT_CHG) {
			dev_dbg(priv->dev, "VCD_STAT_VCT_CHG\n");
			status_ack |= VCD_STAT_VCT_CHG;
		}

		if (status & VCD_STAT_BSD1) {
			dev_dbg(priv->dev, "VCD_STAT_BSD1\n");
			status_ack |= VCD_STAT_BSD1;
		}

		if (status & VCD_STAT_BSD2) {
			dev_dbg(priv->dev, "VCD_STAT_BSD2\n");
			status_ack |= VCD_STAT_BSD2;
		}

		if (status & VCD_STAT_BSD3) {
			dev_dbg(priv->dev, "VCD_STAT_BSD3\n");
			status_ack |= VCD_STAT_BSD3;
		}

		if (status & VCD_STAT_IFOT) {
			dev_dbg(priv->dev, "VCD_STAT_IFOT\n");
			status_ack |= VCD_STAT_IFOT;
		}

		if (status & VCD_STAT_IFOR) {
			dev_dbg(priv->dev, "VCD_STAT_IFOR\n");
			status_ack |= VCD_STAT_IFOR;
		}

	}

	regmap_write(vcd, VCD_STAT, status_ack);

	priv->status = status;

	spin_unlock(&priv->lock);

	complete(&priv->complete);

	return IRQ_HANDLED;
}

static int
npcm750_vcd_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct npcm750_vcd *priv = file->private_data;
	u32 start;
	u32 len;

	if (!priv)
		return -ENODEV;

	start = priv->frame_start;
	len = priv->frame_len;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	fb_pgprotect(file, vma, start);

	return vm_iomap_memory(vma, start, len);
}

static int
npcm750_vcd_open(struct inode *inode, struct file *file)
{
	struct npcm750_vcd *priv =
		container_of(inode->i_cdev, struct npcm750_vcd, dev_cdev);

	if (!priv)
		return -ENODEV;

	file->private_data = priv;

	if (atomic_inc_return(&priv->clients) == 1) {
		if (npcm750_vcd_init(priv)) {
			dev_err(priv->dev, "%s: failed to init vcd module\n",
				__func__);
			return -EBUSY;
		}
	}

	dev_dbg(priv->dev, "open: client %d\n", atomic_read(&priv->clients));
	return 0;
}

static int
npcm750_vcd_release(struct inode *inode, struct file *file)
{
	struct npcm750_vcd *priv = file->private_data;

	if (atomic_dec_return(&priv->clients) == 0)
		npcm750_vcd_stop(priv);

	dev_dbg(priv->dev, "close: client %d\n", atomic_read(&priv->clients));
	return 0;
}

static long
npcm750_do_vcd_ioctl(struct npcm750_vcd *priv, unsigned int cmd,
		     unsigned long arg)
{
	struct regmap *vcd = priv->vcd_regmap;
	void __user *argp = (void __user *)arg;
	long ret = 0;

	mutex_lock(&priv->mlock);
	switch (cmd) {
	case VCD_IOCGETINFO:
		ret = copy_to_user(argp, &priv->info, sizeof(priv->info))
			? -EFAULT : 0;
		break;
	case VCD_IOCSENDCMD:
	{
		int vcd_cmd, timeout;

		ret = copy_from_user(&vcd_cmd, argp, sizeof(vcd_cmd))
			? -EFAULT : 0;
		if (!ret) {
			priv->status = 0;
			reinit_completion(&priv->complete);

			npcm750_vcd_free_diff_table(priv);

			if (vcd_cmd != VCD_CMD_OP_CAPTURE)
				regmap_update_bits(vcd, VCD_MODE, VCD_MODE_IDBC,
					VCD_MODE_IDBC);

			regmap_update_bits(vcd, VCD_MODE, VCD_MODE_VCDE,
				VCD_MODE_VCDE);

			npcm750_vcd_command(priv, vcd_cmd);
			timeout = wait_for_completion_interruptible_timeout(&priv->complete,
			    VCD_OP_TIMEOUT);
			if (timeout <= 0 || !npcm750_vcd_op_done(priv)) {
				dev_dbg(priv->dev, "VCD_OP_BUSY\n");

				if (priv->status == 0)
					regmap_read(vcd, VCD_STAT, &priv->status);

				npcm750_vcd_io_reset(priv);
				ret = copy_to_user(argp, &priv->status, sizeof(priv->status))
					? -EFAULT : 0;
			}

			regmap_update_bits(vcd, VCD_MODE, VCD_MODE_VCDE,
				~VCD_MODE_VCDE);

			if (vcd_cmd != VCD_CMD_OP_CAPTURE && timeout > 0) {
				regmap_update_bits(vcd, VCD_MODE, VCD_MODE_IDBC,
					~VCD_MODE_IDBC);
				npcm750_short_vcd_reset(priv);
				npcm750_vcd_get_diff_table(priv);
			}
		}
		break;
	}
	case VCD_IOCCHKRES:
	{
		int changed = npcm750_vcd_get_resolution(priv);

		if (changed < 0) {
			ret = -EFAULT;
			break;
		}

		ret = copy_to_user(argp, &changed, sizeof(changed))
			? -EFAULT : 0;
		break;
	}
	case VCD_IOCGETDIFF:
	{
		struct rect_list *list;
		struct rect r;
		struct list_head *head = &priv->list;

		if (priv->rect_cnt == 0) {
			r.x = 0;
			r.y = 0;
			r.w = priv->info.hdisp;
			r.h = priv->info.vdisp;
		} else {
			list = list_first_entry_or_null(head,
							struct rect_list,
							list);
			if (!list) {
				r.x = 0;
				r.y = 0;
				r.w = 0;
				r.h = 0;
			} else {
				r.x = list->r.x;
				r.y = list->r.y;
				r.w = list->r.w;
				r.h = list->r.h;
			}
			if (list) {
				list_del(&list->list);
				kfree(list);
				priv->rect_cnt--;
			}
		}
		ret = copy_to_user(argp, &r, sizeof(struct rect))
			? -EFAULT : 0;
		break;
	}
	case VCD_IOCDIFFCNT:
		ret = copy_to_user(argp, &priv->rect_cnt, sizeof(int))
			? -EFAULT : 0;
		break;
	case VCD_IOCDEMODE:
	{
		int mode;

		ret = copy_from_user(&mode, argp, sizeof(mode))
			? -EFAULT : 0;
		if (!ret && priv->de_mode != mode) {
			priv->de_mode = mode;
			npcm750_vcd_stop(priv);
			npcm750_vcd_init(priv);
		}

		break;
	}
	case VCD_IOCRESET:
		npcm750_vcd_io_reset(priv);

		break;
	case VCD_GETREG:
	{
		ret = copy_from_user(&priv->info, argp, sizeof(priv->info))
			? -EFAULT : 0;
		if (!ret && priv->info.reg <= VCD_FIFO) {
			regmap_read(vcd, priv->info.reg, &priv->info.reg_val);
		} else if(!ret &&
			(priv->info.reg >= GFXI_START) &&
			(priv->info.reg <= GFXI_FIFO)) {
			struct regmap *gfxi = priv->gfx_regmap;
			u32 value;

			priv->info.reg = (priv->info.reg & GFXI_MASK);
			regmap_read(gfxi, priv->info.reg, &value);
			priv->info.reg_val = value;
		}

		ret = copy_to_user(argp, &priv->info, sizeof(priv->info))
			? -EFAULT : 0;
		break;
	}
	case VCD_SETREG:
	{
		ret = copy_from_user(&priv->info, argp, sizeof(priv->info))
			? -EFAULT : 0;

		if (!ret && priv->info.reg <= VCD_FIFO)
			regmap_write(vcd, priv->info.reg, priv->info.reg_val);
		break;
	}
	case VCD_SHORT_RESET:
	{
		npcm750_short_vcd_reset(priv);
		break;
	}
	case VCD_IOCSETDISPLAY:
	{
		unsigned int enable;

		ret = copy_from_user(&enable, argp, sizeof(enable))
			? -EFAULT : 0;

		if (!ret)
			npcm750_vcd_local_display(priv, enable);

		break;
	}

	default:
		break;
	}
	mutex_unlock(&priv->mlock);
	return ret;
}

static long
npcm750_vcd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct npcm750_vcd *priv = file->private_data;

	if (!priv)
		return -ENODEV;
	return npcm750_do_vcd_ioctl(priv, cmd, arg);
}

static const struct file_operations npcm750_vcd_fops = {
	.owner		= THIS_MODULE,
	.open		= npcm750_vcd_open,
	.release	= npcm750_vcd_release,
	.mmap		= npcm750_vcd_mmap,
	.unlocked_ioctl = npcm750_vcd_ioctl,
};

static int npcm750_vcd_device_create(struct npcm750_vcd *priv)
{
	int ret;
	dev_t dev;

	ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		pr_err("alloc_chrdev_region() failed for vcd\n");
		goto err;
	}

	priv->dev_t = dev;

	cdev_init(&priv->dev_cdev, &npcm750_vcd_fops);
	priv->dev_cdev.owner = THIS_MODULE;
	ret = cdev_add(&priv->dev_cdev, MKDEV(MAJOR(dev),  MINOR(dev)), 1);
	if (ret < 0) {
		pr_err("Couldn't cdev_add for vcd, error=%d\n", ret);
		goto err;
	}

	vcd_class = class_create(THIS_MODULE, DEVICE_NAME);
	if (IS_ERR(vcd_class)) {
		ret = PTR_ERR(vcd_class);
		pr_err("Unable to create vcd class; errno = %d\n", ret);
		vcd_class = NULL;
		goto err;
	}

	priv->dev = device_create(vcd_class, priv->dev_p,
				 MKDEV(MAJOR(dev), MINOR(dev)),
				 priv,
				 DEVICE_NAME);
	if (IS_ERR(priv->dev)) {
		ret = PTR_ERR(priv->dev);
		pr_err("Unable to create device for vcd; errno = %ld\n",
		       PTR_ERR(priv->dev));
		priv->dev = NULL;
		goto err;
	}

	return 0;

err:
	return ret;
}

static const struct regmap_config npcm750_vcd_regmap_cfg = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register   = VCD_FIFO,
};

static int npcm750_vcd_probe(struct platform_device *pdev)
{
	struct npcm750_vcd *priv;
	void __iomem *regs;
	int ret;

	priv = devm_kzalloc(&pdev->dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	spin_lock_init(&priv->lock);
	mutex_init(&priv->mlock);

	priv->gcr_regmap =
		syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(priv->gcr_regmap)) {
		dev_err(&pdev->dev, "%s: failed to find nuvoton,npcm750-gcr\n",
			__func__);
		ret = IS_ERR(priv->gcr_regmap);
		goto err;
	}

	priv->gfx_regmap =
		syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gfxi");
	if (IS_ERR(priv->gfx_regmap)) {
		dev_err(&pdev->dev, "%s: failed to find nuvoton,npcm750-gfxi\n",
			__func__);
		ret = IS_ERR(priv->gfx_regmap);
		goto err;
	}

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "Failed to get regmap!\n");
		ret = PTR_ERR(regs);
		goto err;
	}

	priv->vcd_regmap = devm_regmap_init_mmio(&pdev->dev, regs,
						&npcm750_vcd_regmap_cfg);
	if (IS_ERR(priv->vcd_regmap)) {
		dev_err(&pdev->dev, "Failed to init regmap!\n");
		ret = PTR_ERR(priv->vcd_regmap);
		goto err;
	}

	ret = of_property_read_u32_index(pdev->dev.of_node,
			     "phy-memory", 0, &priv->frame_start);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to find memory address\n",
			__func__);
		goto err;
	}

	ret = of_property_read_u32_index(pdev->dev.of_node,
			     "phy-memory", 1, &priv->frame_len);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to find memory length\n",
			__func__);
		goto err;
	}

	ret = of_property_read_u32(pdev->dev.of_node,
			     "de-mode", &priv->de_mode);
	if (ret)
		priv->de_mode = 1;

	priv->dev_p = &pdev->dev;

	ret = npcm750_vcd_device_create(priv);
	if (ret)
		goto err;

	priv->irq = of_irq_get(pdev->dev.of_node, 0);
	ret = request_irq(priv->irq, npcm750_vcd_irq_handler,
			  IRQF_SHARED, vcd_name, priv->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to request irq for vcd\n",
			__func__);
		goto irq_err;
	}

	platform_set_drvdata(pdev, priv);
	INIT_LIST_HEAD(&priv->list);
	init_completion(&priv->complete);

	pr_info("NPCM750 VCD Driver probed %s\n", VCD_VERSION);
	return 0;

irq_err:
	device_destroy(vcd_class, priv->dev_t);
err:
	kfree(priv);
	return ret;
}

static int npcm750_vcd_remove(struct platform_device *pdev)
{
	struct npcm750_vcd *priv = platform_get_drvdata(pdev);

	npcm750_vcd_stop(priv);

	free_irq(priv->irq, priv->dev);

	device_destroy(vcd_class, priv->dev_t);

	class_destroy(vcd_class);

	cdev_del(&priv->dev_cdev);

	unregister_chrdev_region(priv->dev_t, 1);

	mutex_destroy(&priv->mlock);

	kfree(priv);

	return 0;
}

static const struct of_device_id npcm750_vcd_of_match_table[] = {
	{ .compatible = "nuvoton,npcm750-vcd"},
	{}
};
MODULE_DEVICE_TABLE(of, npcm750_vcd_of_match_table);

static struct platform_driver npcm750_vcd_driver = {
	.driver		= {
		.name	= vcd_name,
		.of_match_table = npcm750_vcd_of_match_table,
	},
	.probe		= npcm750_vcd_probe,
	.remove		= npcm750_vcd_remove,
};

module_platform_driver(npcm750_vcd_driver);
MODULE_DESCRIPTION("Nuvoton NPCM750 VCD Driver");
MODULE_AUTHOR("KW Liu <kwliu@nuvoton.com>");
MODULE_LICENSE("GPL v2");
