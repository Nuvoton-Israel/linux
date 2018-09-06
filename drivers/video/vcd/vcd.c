/*
 * Copyright (c) 2018 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/err.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include "vcd.h"

static void vcd_set_line_pitch(struct vcd_inst *vcd, u32 linebytes)
{
	struct vcd_reg *reg = vcd->reg;
	/* Pitch must be a power of 2, >= linebytes,*/
	/* at least 512, and no more than 4096. */
	u32 pitch = VCD_MIN_LP;

	while ((pitch < linebytes) && (pitch < VCD_MAX_LP))
		pitch *= 2;

	write32((pitch << VCD_FBB_LP_OFFSET) | pitch, reg->fb_lp);
}

static int vcd_bytes_per_pixel(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	u8 color_cnvr = ((read32(reg->vcd_mode)
		& VCD_MODE_COLOR_CNVRT)
		>> VCD_MODE_COLOR_CNVRT_OFFSET);

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

static int
vcd_set_frame_addrs(struct vcd_inst *vcd, u32 phys_addr_a, u32 phys_addr_b)
{
	struct vcd_reg *reg = vcd->reg;

	write32(phys_addr_a, reg->fba_adr);
	write32(phys_addr_b, reg->fbb_adr);

	/* Check the alignment by reading the addresses */
	/* back from the VCD registers */
	if ((read32(reg->fba_adr) != phys_addr_a) ||
	    (read32(reg->fbb_adr) != phys_addr_b))
		return -EFAULT;

	vcd->info.vcd_fb = phys_addr_a;
	return 0;
}

static int vcd_set_color_mode(struct vcd_inst *vcd, u8 cm)
{
	struct vcd_reg *reg = vcd->reg;
	u32 mode = read32(reg->vcd_mode) & ~VCD_MODE_CM565;

	if (cm == VCD_MODE_CM_565) {
		mode |= VCD_MODE_CM565;
		vcd->info.g_max = 63;
	} else {
		vcd->info.g_max = 31;
	}

	vcd->info.r_max = 31;
	vcd->info.b_max = 31;
	vcd->info.r_shift = 11;
	vcd->info.g_shift = 5;
	vcd->info.b_shift = 0;

	write32(mode, reg->vcd_mode);
	return 0;
}

static int vcd_init_diff_bit(struct vcd_inst *vcd, u8 init_diff)
{
	struct vcd_reg *reg = vcd->reg;

	u32 mode = read32(reg->vcd_mode) & ~VCD_MODE_IDBC;

	if (init_diff)
		mode |= VCD_MODE_IDBC;

	write32(mode, reg->vcd_mode);
	return 0;
}

static void vcd_set_int(struct vcd_inst *vcd, u32 flags)
{
	struct vcd_reg *reg = vcd->reg;

	write32(flags, reg->vcd_inte);
}

int vcd_is_int_en(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return (read32(reg->vcd_inte) != 0);
}

static u32 vcd_get_cur_line(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return ((read32(reg->vcd_stat) & VCD_STAT_CURR_LINE)
		>> VCD_STAT_CURR_LINE_OFFSET);
}

static u32 vcd_get_line_pitch(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return read32(reg->fb_lp) & VCD_FB_LP_MASK;
}

static int vcd_is_hw_present(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	write32(0xffffffff, reg->fb_lp);
	write32(0xffffffff, reg->cap_res);

	if ((read32(reg->fb_lp) != 0xfe00fe00) ||
	    (read32(reg->cap_res) != 0x7ff07ff)) {
		pr_err("VCD block not present\n");
		return -ENODEV;
	}
	return 0;
}

static u8 gfx_is_mga_mode(struct vcd_inst *vcd)
{
	struct regmap *gfxi = vcd->gfx_regmap;
	u32 dispst;

	regmap_read(gfxi, DISPST_OFFSET, &dispst);
	return ((dispst & MGAMODE_MASK) == MGAMODE_MASK);
}

static u32 gfx_hor_res(struct vcd_inst *vcd)
{
	struct regmap *gfxi = vcd->gfx_regmap;
	u32 hvcnth, hvcntl;

	regmap_read(gfxi, HVCNTH_OFFSET, &hvcnth);
	regmap_read(gfxi, HVCNTL_OFFSET, &hvcntl);
	return (((hvcnth & HVCNTH_MASK) << 8)
		+ (hvcntl & HVCNTL_MASK) + 1);
}

static u32 gfx_ver_res(struct vcd_inst *vcd)
{
	struct regmap *gfxi = vcd->gfx_regmap;
	u32 vvcnth, vvcntl;

	regmap_read(gfxi, VVCNTH_OFFSET, &vvcnth);
	regmap_read(gfxi, VVCNTL_OFFSET, &vvcntl);
	return (((vvcnth & VVCNTH_MASK) << 8)
		+ (vvcntl & VVCNTL_MASK));
}

static u32 vcd_get_hres(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;
	u32 apb_hor_res, hor_act;

	apb_hor_res = gfx_hor_res(vcd);

	if (gfx_is_mga_mode(vcd))
		return (apb_hor_res > VCD_MAX_WIDTH) ?
			    VCD_MAX_WIDTH : apb_hor_res;

	hor_act = read32(reg->hor_ac_tim) & VCD_HOR_AC_TIM_MASK;
	/* The following 'if' checks if hor_act is wrong */
	if (((apb_hor_res + 40) < hor_act) || (hor_act > 50))
		return ((apb_hor_res & 0xFF0) > VCD_MAX_WIDTH)
			? VCD_MAX_WIDTH : (hor_act & 0xFF0);

	return vcd->info.hdisp;
}

static u32 vcd_get_vres(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;
	u32 apb_ver_res, ver_act;

	if (gfx_is_mga_mode(vcd)) {
		apb_ver_res = gfx_ver_res(vcd);
		return (apb_ver_res > VCD_MAX_HIGHT) ?
			    VCD_MAX_HIGHT : apb_ver_res;
	}

	ver_act = read32(reg->hor_lin_tim) & VCD_HOR_LIN_TIM_MASK;
	if (ver_act > 50)
		return (ver_act > VCD_MAX_HIGHT) ? VCD_MAX_HIGHT : ver_act;
	return vcd->info.vdisp;
}

static u32 vcd_get_pclk(struct vcd_inst *vcd)
{
	struct regmap *gfxi = vcd->gfx_regmap;
	u32 tmp, pllfbdiv, pllinotdiv, gpllfbdiv;
	u8 gpllfbdv109, gpllfbdv8, gpllindiv;
	u8 gpllst_pllotdiv1, gpllst_pllotdiv2;

	regmap_read(gfxi, GPLLST_OFFSET, &tmp);
	gpllfbdv109 = (tmp & GPLLFBDV109_MASK) >> GPLLFBDV109_BIT;
	gpllst_pllotdiv1 = tmp & GPLLST_PLLOTDIV1_MASK;
	gpllst_pllotdiv2 =
		(tmp & GPLLST_PLLOTDIV2_MASK) >> GPLLST_PLLOTDIV2_BIT;

	regmap_read(gfxi, GPLLINDIV_OFFSET, &tmp);
	gpllfbdv8 = (tmp & GPLLFBDV8_MASK) >> GPLLFBDV8_BIT;
	gpllindiv = (tmp & GPLLINDIV_MASK);

	regmap_read(gfxi, GPLLFBDIV_OFFSET, &tmp);
	gpllfbdiv = tmp & GPLLFBDIV_MASK;

	pllfbdiv = (512 * gpllfbdv109 + 256 * gpllfbdv8 + gpllfbdiv);
	pllinotdiv = (gpllindiv * gpllst_pllotdiv1 * gpllst_pllotdiv2);
	if (pllfbdiv == 0 || pllinotdiv == 0)
		return 0;

	return ((pllfbdiv * 25000) / pllinotdiv) * 1000;
}

static int vcd_set_capres(struct vcd_inst *vcd, u32 hor_res, u32 vert_res)
{
	struct vcd_reg *reg = vcd->reg;
	u32 res = (vert_res & VCD_CAPRES_MASK)
		| ((hor_res & VCD_CAPRES_MASK) << 16);

	if ((hor_res > VCD_MAX_WIDTH) || (vert_res > VCD_MAX_HIGHT))
		return -EINVAL;

	write32(res, reg->cap_res);

	/* Read back the register to check that the values were valid */
	if (read32(reg->cap_res) !=  res)
		return -EINVAL;

	return 0;
}

int vcd_reset(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;
	struct regmap *gcr = vcd->gcr_regmap;
	static u8 second_reset = 1;

	write32(read32(reg->vcd_cmd) | VCD_CMD_RST, reg->vcd_cmd);
	while (!(read32(reg->vcd_stat) & VCD_STAT_DONE))
		continue;

	if (second_reset)
		regmap_update_bits(
			gcr, INTCR2_OFFSET, INTCR2_GIRST2, INTCR2_GIRST2);

	write32(0xffffffff, reg->vcd_stat);

	/* Inactive graphic */
	regmap_update_bits(
		gcr, INTCR2_OFFSET, INTCR2_GIRST2, ~INTCR2_GIRST2);
	return 0;
}

static void vcd_set_dehs_mode(struct vcd_inst *vcd, u8 signal_is_de)
{
	struct vcd_reg *reg = vcd->reg;
	struct regmap *gcr = vcd->gcr_regmap;
	u32 tmp = read32(reg->vcd_mode);

	if (signal_is_de)
		tmp &= ~VCD_MODE_DE_HS;
	else
		tmp |= VCD_MODE_DE_HS;

	write32(tmp, reg->vcd_mode);

	if (signal_is_de)
		regmap_update_bits(
			gcr, INTCR_OFFSET, INTCR_DEHS, ~INTCR_DEHS);
	else
		regmap_update_bits(
			gcr, INTCR_OFFSET, INTCR_DEHS, INTCR_DEHS);
}

static void vcd_set_kvm_bw(struct vcd_inst *vcd, u32 bandwidth)
{
	struct vcd_reg *reg = vcd->reg;

	u32 mode = read32(reg->vcd_mode) & ~VCD_MODE_KVM_BW_SET;

	if (gfx_is_mga_mode(vcd) == 0)
		bandwidth = 1;

	if (bandwidth)
		mode |= VCD_MODE_KVM_BW_SET;

	write32(mode, reg->vcd_mode);
}

static u32 vcd_htotal(struct vcd_inst *vcd)
{
	return vcd->info.hdisp + vcd->info.hfrontporch
		+ vcd->info.hsync + vcd->info.hbackporch;
}

static u32 vcd_vtotal(struct vcd_inst *vcd)
{
	return vcd->info.vdisp + vcd->info.vfrontporch
		+ vcd->info.vsync + vcd->info.vbackporch;
}

static int vcd_vsync_period(struct vcd_inst *vcd)
{
	if (vcd->info.pixel_clk == 0)
		return 0;

	return (vcd_htotal(vcd) * vcd_vtotal(vcd) * 200)
		/ (vcd->info.pixel_clk / 5);
}

static void vcd_detect_video_mode(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	vcd->mga_mode = gfx_is_mga_mode(vcd);
	vcd->info.hdisp = vcd_get_hres(vcd);
	vcd->info.vdisp = vcd_get_vres(vcd);
	vcd->video_name = "Digital";
	vcd->info.pixel_clk = vcd_get_pclk(vcd);
	vcd->info.hfrontporch = 0;
	vcd->info.hbackporch = 0;
	vcd->info.vfrontporch = 0;
	vcd->info.vbackporch = 0;
	vcd->info.hpositive = 1;
	vcd->info.vpositive = 0;
	vcd->info.bpp = vcd_bytes_per_pixel(vcd);

	if (vcd->info.hdisp > VCD_MAX_WIDTH)
		vcd->info.hdisp = VCD_MAX_WIDTH;

	if (vcd->info.vdisp > VCD_MAX_HIGHT)
		vcd->info.vdisp = VCD_MAX_HIGHT;

	if (vcd_vsync_period(vcd) > 0)
		vcd->info.refresh_rate = 1000 / vcd_vsync_period(vcd);

	vcd_set_capres(vcd, vcd->info.hdisp, vcd->info.vdisp);
	vcd_set_line_pitch(vcd, vcd->info.hdisp * vcd_bytes_per_pixel(vcd));
	vcd->info.line_pitch = vcd_get_line_pitch(vcd);
	vcd_set_kvm_bw(vcd, vcd->info.pixel_clk > 120000000UL);
	vcd_reset(vcd);

	pr_info("[VCD] vcd_mode = 0x%x, %s mode\n",
		(u32)read32(reg->vcd_mode),
		gfx_is_mga_mode(vcd) ? "Hi Res" : "VGA");

	pr_info("[VCD] digital mode: %d x %d, Pixel Clk %zuKHz, Line Pitch %d\n",
		vcd->info.hdisp, vcd->info.vdisp,
		(u32)(vcd->info.pixel_clk / 1000),
		vcd->info.line_pitch);
}

static int vcd_alloc_frame_memory(struct vcd_inst *vcd)
{
	dma_addr_t map_dma;
	unsigned int map_size;

	map_size = PAGE_ALIGN((vcd->info.hdisp + 16)
		* (vcd->info.vdisp + 16) * 2);

	vcd->smem_base = dma_alloc_coherent(vcd->dev_p,
					    map_size, &map_dma,
					    GFP_KERNEL);
	if (vcd->smem_base) {
		memset(vcd->smem_base, 0x00, map_size);
		vcd->smem_start = map_dma;
		vcd->smem_len = map_size;
	} else {
		pr_err("failed to alloc vcd memory\n");
		return -ENOMEM;
	}

	return 0;
}

void vcd_free_frame_memory(struct vcd_inst *vcd)
{
	if (vcd->smem_base)
		dma_free_coherent(vcd->dev, PAGE_ALIGN(vcd->smem_len),
				  vcd->smem_base, vcd->smem_start);

	vcd->smem_base = NULL;
}

u8 vcd_is_busy(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return ((read32(reg->vcd_stat) & VCD_STAT_BUSY) == VCD_STAT_BUSY);
}

u8 vcd_is_done(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return (read32(reg->vcd_stat)
		& (VCD_STAT_DONE | VCD_STAT_IFOR)) != 0;
}

u8 vcd_is_op_ok(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;
	u32 vdisp = read32(reg->cap_res) & VCD_CAPRES_MASK;
	u32 vcd_stat = read32(reg->vcd_stat);
	u32 mask = VCD_STAT_DONE |
		VCD_STAT_IFOR |
		VCD_STAT_BUSY;

	if (vcd->info.hdisp == 0 ||
	    vcd->info.vdisp == 0 ||
	    vcd->info.pixel_clk == 0)
		return 1;

	if ((vcd->info.hdisp != vcd_get_hres(vcd)) ||
		   (vcd->info.vdisp != vcd_get_vres(vcd)))
		return 1;

	return ((vcd_stat & mask) == 0) && (vcd_get_cur_line(vcd) == vdisp);
}

u32 vcd_get_status(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	return read32(reg->vcd_stat);
}

void vcd_clear_status(struct vcd_inst *vcd, u32 flags)
{
	struct vcd_reg *reg = vcd->reg;

	write32(flags, reg->vcd_stat);
}

int vcd_command(struct vcd_inst *vcd, u32 value)
{
	struct vcd_reg *reg = vcd->reg;
	u32 cmd;

	if (vcd_is_busy(vcd))
		/* Not ready for another command */
		return -EBUSY;

	/* Clear the status flags that could be set by this command */
	write32(read32(reg->vcd_stat)
		| VCD_STAT_IFOR | VCD_STAT_IFOT, reg->vcd_stat);

	cmd = read32(reg->vcd_cmd) & ~VCD_CMD_OP_MASK;
	cmd |= (value << VCD_CMD_OP_OFFSET);

	write32(cmd, reg->vcd_cmd);
	write32(cmd | VCD_CMD_GO, reg->vcd_cmd);

	vcd->cmd = value;
	return 0;
}

int vcd_check_res(struct vcd_inst *vcd)
{
	/* check with GFX registers if resolution changed from last time */
	u8 changed = ((vcd->info.hdisp != vcd_get_hres(vcd)) ||
		(vcd->info.vdisp != vcd_get_vres(vcd)));

	if (changed) {
		vcd_set_int(vcd, 0);
		vcd_detect_video_mode(vcd);
		vcd_free_frame_memory(vcd);
		vcd_alloc_frame_memory(vcd);
		vcd_set_int(vcd, VCD_INTE_VAL);
	}

	return changed;
}

void vcd_free_diff_table(struct vcd_inst *vcd)
{
	struct list_head *head, *pos, *nx;
	struct vcd_diff_list *tmp;

	head = &vcd->list.list;
	list_for_each_safe(pos, nx, head) {
		tmp = list_entry(pos, struct vcd_diff_list, list);
		if (tmp) {
			list_del(&tmp->list);
			kfree(tmp);
		}
	}
}

static void
vcd_merge_rect(struct vcd_inst *vcd, struct vcd_list_info *list_info)
{
	struct list_head *head = &vcd->list.list;
	struct vcd_diff_list *list = list_info->list;
	struct vcd_diff_list *first = list_info->first;

	if (!first) {
		first = list;
		list_info->first = first;
		list_add_tail(&list->list, head);
		vcd->diff_cnt++;
	} else {
		if (((list->diff.x ==
		      (first->diff.x + first->diff.w))) &&
		      (list->diff.y == first->diff.y)) {
			first->diff.w += list->diff.w;
			kfree(list);
		} else if (((list->diff.y ==
			     (first->diff.y + first->diff.h))) &&
			    (list->diff.x == first->diff.x)) {
			first->diff.h += list->diff.h;
			kfree(list);
		} else if (((list->diff.y > first->diff.y) &&
			    (list->diff.y < (first->diff.y + first->diff.h))) &&
			   ((list->diff.x > first->diff.x) &&
			    (list->diff.x < (first->diff.x + first->diff.w)))) {
			kfree(list);
		} else {
			list_add_tail(&list->list, head);
			vcd->diff_cnt++;
			list_info->first = list;
		}
	}
}

static struct vcd_diff_list *
vcd_new_rect(struct vcd_inst *vcd, int offset, int index)
{
	struct vcd_diff_list *list = NULL;

	list = kmalloc(sizeof(*list), GFP_KERNEL);
	if (!list)
		return NULL;

	list->diff.x = (offset << 4);
	list->diff.y = (index >> 2);
	list->diff.w = 16;
	list->diff.h = 16;
	if ((list->diff.x + 16) > vcd->info.hdisp)
		list->diff.w = vcd->info.hdisp - list->diff.x;
	if ((list->diff.y + 16) > vcd->info.vdisp)
		list->diff.h = vcd->info.vdisp - list->diff.y;

	return list;
}

static int
vcd_find_rect(struct vcd_inst *vcd, struct vcd_list_info *info, u32 offset)
{
	int i = info->index;

	if (offset < info->tile_perline) {
		info->list = vcd_new_rect(vcd, offset, i);
		if (!info->list)
			return -ENOMEM;

		vcd_merge_rect(vcd, info);
	}
	return 0;
}

static int
vcd_build_table(struct vcd_inst *vcd, struct vcd_list_info *info)
{
	struct vcd_reg *reg = vcd->reg;
	int i = info->index;
	int j, z;

	for (j = 0 ; j < info->offset_perline ; j += 4) {
		if (read32(reg->diff_tbl + (j + i)) != 0) {
			for (z = 0 ; z < 32; z++) {
				if ((read32(reg->diff_tbl + (j + i)) >> z) &
					  0x01) {
					int ret;
					u32 offset = z + (j << 3);

					ret = vcd_find_rect(vcd, info, offset);
					if (ret < 0)
						return ret;
				}
			}
		}
	}
	info->index += 64;
	return info->tile_perline;
}

int vcd_get_diff_table(struct vcd_inst *vcd)
{
	struct vcd_list_info list_info;
	int ret = 0;
	u32 mod, tile_cnt = 0;

	memset(&list_info, 0, sizeof(struct vcd_list_info));
	list_info.head = &vcd->list.list;

	list_info.tile_perline = vcd->info.hdisp >> 4;
	mod = vcd->info.hdisp % 16;
	if (mod != 0)
		list_info.tile_perline += 1;

	list_info.tile_perrow = vcd->info.vdisp >> 4;
	mod = vcd->info.vdisp % 16;
	if (mod != 0)
		list_info.tile_perrow += 1;

	list_info.tile_size =
		list_info.tile_perrow * list_info.tile_perline;

	list_info.offset_perline = list_info.tile_perline >> 5;
	mod = list_info.tile_perline % 32;
	if (mod != 0)
		list_info.offset_perline += 1;

	list_info.offset_perline *= 4;

	do {
		ret = vcd_build_table(vcd, &list_info);
		if (ret < 0)
			return ret;
		tile_cnt += ret;
	} while (tile_cnt < list_info.tile_size);

	return ret;
}

int vcd_init(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;
	struct regmap *gcr = vcd->gcr_regmap;
	u32 mask = 0;
	u32 data = 0;

	if (vcd_is_hw_present(vcd))
		return  -ENODEV;

	vcd_reset(vcd);

	/* Initialise capture resolution to a non-zero value */
	/* so that frame capture will behave sensibly before */
	/* the true resolution has been determined.*/
	if (vcd_set_capres(vcd, VCD_INIT_WIDTH, VCD_INIT_HIGHT)) {
		pr_err("failed to set capture resolution\n");
		return -EINVAL;
	}

	/* Clear all the 'last' values in the resolution change detection */
	write32(0, reg->hor_cyc_lst);
	write32(0, reg->hor_hi_lst);
	write32(0, reg->hor_ac_lst);
	write32(0, reg->hor_lin_lst);
	write32(0, reg->hor_hi_lst);
	write32(0, reg->ver_cyc_lst);
	write32(0, reg->ver_hi_lst);

	/* Set the FIFO thresholds */
	write32(VCD_FIFO_TH, reg->vcd_fifo);

	/* Data enabled is selected */
	vcd_set_dehs_mode(vcd, 1);

	/* Set kvm bandwidth */
	vcd_set_kvm_bw(vcd, 1);

	/* Enable display of KVM GFX and access to memory */
	regmap_update_bits(gcr, INTCR_OFFSET, INTCR_GFXIFDIS, ~INTCR_GFXIFDIS);

	/* Set vrstenw and hrstenw */
	mask = INTCR2_GIHCRST | INTCR2_GIVCRST;
	data = INTCR2_GIHCRST | INTCR2_GIVCRST;
	regmap_update_bits(gcr, INTCR2_OFFSET, mask, data);

	/* Select KVM GFX input */
	regmap_update_bits(gcr, MFSEL1_OFFSET, MFSEL1_DVH1SEL, ~MFSEL1_DVH1SEL);

	/* Enable the VCD + Vsync edge Rise */
	write32(read32(reg->vcd_mode) | VCD_MODE_VCDE, reg->vcd_mode);
	write32(read32(reg->vcd_mode)
		& (~VCD_MODE_VS_EDGE), reg->vcd_mode);

	vcd_set_frame_addrs(vcd, vcd->frame_start, vcd->frame_start);
	vcd_init_diff_bit(vcd, 1);
	vcd_set_color_mode(vcd, VCD_MODE_CM_565);
	vcd_set_int(vcd, VCD_INTE_VAL);
	vcd_check_res(vcd);
	return 0;
}

void vcd_deinit(struct vcd_inst *vcd)
{
	struct vcd_reg *reg = vcd->reg;

	vcd_set_int(vcd, 0);
	vcd_free_frame_memory(vcd);
	vcd_free_diff_table(vcd);
	write32(read32(reg->vcd_mode) & ~VCD_MODE_VCDE, reg->vcd_mode);
	memset(&vcd->info, 0, sizeof(struct vcd_info));
}
