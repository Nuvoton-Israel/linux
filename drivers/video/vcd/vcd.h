/*
 * Copyright (c) 2018 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#ifndef __NU_VCD_H__
#define __NU_VCD_H__

#include <linux/list.h>
#include <linux/slab.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>

/* GCR  Register */
#define INTCR_OFFSET 0x3c
#define INTCR2_OFFSET 0x60
#define INTCR3_OFFSET 0x9C
#define MFSEL1_OFFSET 0x0c

#define INTCR_GFXIFDIS	(BIT(8) | BIT(9))
#define INTCR_DEHS	BIT(27)
#define INTCR2_GIRST2	BIT(2)
#define INTCR2_GIHCRST	BIT(5)
#define INTCR2_GIVCRST	BIT(6)
#define MFSEL1_DVH1SEL	BIT(27)

/* VCD  Status */
#define	VCD_STAT_CLEAR	0x3ffe
#define	VCD_STAT_CURR_LINE_OFFSET 16
#define	VCD_STAT_CURR_LINE 0x7ff0000
#define	VCD_STAT_IRQ	BIT(31)
#define	VCD_STAT_BUSY	BIT(30)
#define	VCD_STAT_BSD3	BIT(13)
#define	VCD_STAT_BSD2	BIT(12)
#define	VCD_STAT_HSYNC	BIT(11)
#define	VCD_STAT_VSYNC	BIT(10)
#define	VCD_STAT_HLC_CHG	BIT(9)
#define	VCD_STAT_HAC_CHG	BIT(8)
#define	VCD_STAT_HHT_CHG	BIT(7)
#define	VCD_STAT_HCT_CHG	BIT(6)
#define	VCD_STAT_VHT_CHG	BIT(5)
#define	VCD_STAT_VCT_CHG	BIT(4)
#define	VCD_STAT_IFOR	BIT(3)
#define	VCD_STAT_IFOT	BIT(2)
#define	VCD_STAT_BSD1	BIT(1)
#define	VCD_STAT_DONE	BIT(0)

/* VCD  MODE */
#define VCD_MODE_COLOR_NORM		0x0
#define VCD_MODE_COLOR_222		0x1
#define VCD_MODE_COLOR_666		0x2
#define VCD_MODE_COLOR_888		0x3
#define VCD_MODE_CM_555		0x0
#define VCD_MODE_CM_565		0x1
#define VCD_MODE_COLOR_CNVRT_OFFSET 4
#define VCD_MODE_VCDE	BIT(0)
#define VCD_MODE_CM565	BIT(1)
#define VCD_MODE_IDBC	BIT(3)
#define VCD_MODE_COLOR_CNVRT	(BIT(4) | BIT(5))
#define VCD_MODE_DAT_INV	BIT(6)
#define VCD_MODE_CLK_EDGE	BIT(8)
#define VCD_MODE_HS_EDGE	BIT(9)
#define VCD_MODE_VS_EDGE	BIT(10)
#define VCD_MODE_DE_HS	BIT(11)
#define VCD_MODE_KVM_BW_SET	BIT(16)

/* VCD  Interrupt Enable*/
#define VCD_INTE_DONE_IE	BIT(0)
#define VCD_INTE_BSD_IE	BIT(1)
#define VCD_INTE_IFOT_IE	BIT(2)
#define VCD_INTE_IFOR_IE	BIT(3)
#define VCD_INTE_VCT_CHG_IE	BIT(4)
#define VCD_INTE_VHT_CHG_IE	BIT(5)
#define VCD_INTE_HCT_CHG_IE	BIT(6)
#define VCD_INTE_HHT_CHG_IE	BIT(7)
#define VCD_INTE_HAC_CHG_IE	BIT(8)
#define VCD_INTE_HLC_CHG	BIT(9)
#define VCD_INTE_VSYNC_IE	BIT(10)
#define VCD_INTE_HSYNC_IE	BIT(11)
#define VCD_INTE_BSD2_IE	BIT(12)
#define VCD_INTE_BSD3_IE	BIT(13)
#define VCD_INTE_VAL	(VCD_INTE_DONE_IE | VCD_INTE_IFOR_IE)

/* VCD CMD */
#define VCD_CMD_OP_MASK	0x70
#define VCD_CMD_OP_OFFSET	4
#define VCD_CMD_OP_CAPTURE	0
#define VCD_CMD_OP_CAPTURE_TWO	1
#define VCD_CMD_OP_COMPARE	2
#define VCD_CMD_GO	BIT(0)
#define VCD_CMD_RST	BIT(1)

/* FIFO Thresholds */
#define VCD_FIFO_TH	0x100350ff

/* FB Line pitch */
#define VCD_FB_LP_MASK 0xffff
#define VCD_FBB_LP_OFFSET 16

#define VCD_HOR_LIN_TIM_MASK 0x7ff
#define VCD_HOR_AC_TIM_MASK 0x7ff
#define VCD_CAPRES_MASK 0x7ff

/* GFXI Register */
#define DISPST_OFFSET	0
#define MGAMODE_MASK BIT(7)

#define HVCNTL_OFFSET	0x10
#define HVCNTL_MASK	0xff
#define HVCNTH_OFFSET	0x14
#define HVCNTH_MASK	0x07

#define VVCNTL_OFFSET	0x20
#define VVCNTL_MASK	0xff
#define VVCNTH_OFFSET	0x24
#define VVCNTH_MASK	0x07

#define GPLLINDIV_OFFSET	0x40
#define GPLLINDIV_MASK	0x3f
#define GPLLFBDV8_MASK	0x80
#define GPLLINDIV_BIT	0
#define GPLLFBDV8_BIT	7

#define GPLLFBDIV_OFFSET	0x44
#define GPLLFBDIV_MASK	0xff

#define GPLLST_OFFSET	0x48
#define GPLLFBDV109_MASK	0xc0
#define GPLLFBDV109_BIT	6
#define GPLLST_PLLOTDIV1_MASK	0x07
#define GPLLST_PLLOTDIV2_MASK	0x38
#define GPLLST_PLLOTDIV1_BIT	0
#define GPLLST_PLLOTDIV2_BIT	3

#define VCD_INIT_WIDTH	640
#define VCD_INIT_HIGHT	480
#define VCD_MAX_WIDTH	2047
#define VCD_MAX_HIGHT	1536
#define VCD_MIN_LP	512
#define VCD_MAX_LP	4096

#define write32(x, y) writel(x, (void __iomem *)y)
#define read32(x) readl((void __iomem *)x)

struct vcd_reg {
	u32 diff_tbl[0x2000];
	u32 fba_adr[1];
	u32 fbb_adr[1];
	u32 fb_lp[1];
	u32 cap_res[1];
	u32 dvo_del[1];
	u32 vcd_mode[1];
	u32 vcd_cmd[1];
	u32 vcd_stat[1];
	u32 vcd_inte[1];
	u32 vcd_bsd1[1];
	u32 vcd_rchg[1];
	u32 hor_cyc_tim[1];
	u32 hor_cyc_lst[1];
	u32 hor_hi_tim[1];
	u32 hor_hi_lst[1];
	u32 ver_cyc_tim[1];
	u32 ver_cyc_lst[1];
	u32 ver_hi_tim[1];
	u32 ver_hi_lst[1];
	u32 hor_ac_tim[1];
	u32 hor_ac_lst[1];
	u32 hor_lin_tim[1];
	u32 hor_lin_lst[1];
	u32 vcd_fifo[1];
	u32 resvered[5];
	u32 vcd_bsd2[1];
	u32 vcd_bsd3[1];
};

struct vcd_info {
	u32 vcd_fb;
	u32 pixel_clk;
	u32 line_pitch;
	int hdisp;
	int hfrontporch;
	int hsync;
	int hbackporch;
	int vdisp;
	int vfrontporch;
	int vsync;
	int vbackporch;
	int refresh_rate;
	int hpositive;
	int vpositive;
	int bpp;
	int r_max;
	int g_max;
	int b_max;
	int r_shift;
	int g_shift;
	int b_shift;
};

struct vcd_diff {
	u32 x;
	u32 y;
	u32 w;
	u32 h;
};

struct vcd_diff_list {
	struct vcd_diff diff;
	struct list_head list;
};

struct vcd_list_info {
	struct vcd_diff_list *list;
	struct vcd_diff_list *first;
	struct list_head *head;
	int index;
	int tile_perline;
	int tile_perrow;
	int offset_perline;
	int tile_size;
	int tile_cnt;
};

struct vcd_inst {
	struct mutex mlock; /*for iotcl*/
	spinlock_t lock;	/*for irq*/
	struct device *dev;
	struct device *dev_p;
	struct vcd_reg *reg;
	struct vcd_info info;
	struct vcd_diff diff;
	struct vcd_diff_list list;
	struct vcd_list_info list_info;
	struct regmap *gcr_regmap;
	struct regmap *gfx_regmap;
	char __iomem *smem_base;
	char __iomem *frame_base;
	u8 mga_mode;
	u32 smem_len;
	u32 smem_start;
	u32 frame_len;
	u32 frame_start;
	u32 diff_cnt;
	char *video_name;
	int cmd;
	dev_t dev_id;
};

u8 vcd_is_busy(struct vcd_inst *vcd);
u8 vcd_is_done(struct vcd_inst *vcd);
u8 vcd_is_op_ok(struct vcd_inst *vcd);
u32 vcd_get_status(struct vcd_inst *vcd);
void vcd_clear_status(struct vcd_inst *vcd, u32 flags);
int vcd_command(struct vcd_inst *vcd, u32 value);
int vcd_check_res(struct vcd_inst *vcd);
int vcd_init(struct vcd_inst *vcd);
void vcd_deinit(struct vcd_inst *vcd);
void vcd_free_frame_memory(struct vcd_inst *vcd);
int vcd_is_int_en(struct vcd_inst *vcd);
int vcd_get_diff_table(struct vcd_inst *vcd);
void vcd_free_diff_table(struct vcd_inst *vcd);
int vcd_reset(struct vcd_inst *vcd);

#endif
