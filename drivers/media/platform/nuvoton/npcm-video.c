// SPDX-License-Identifier: GPL-2.0
/*
 * Driver for Video Capture/Differentiation Engine (VCD) and Encoding
 * Compression Engine (ECE) present on Nuvoton NPCM SoCs.
 *
 * Copyright (C) 2022 Nuvoton Technologies
 */

#include <linux/atomic.h>
#include <linux/bitfield.h>
#include <linux/bitmap.h>
#include <linux/clk.h>
#include <linux/delay.h>
#include <linux/device.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/jiffies.h>
#include <linux/mfd/syscon.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/of_reserved_mem.h>
#include <linux/platform_device.h>
#include <linux/regmap.h>
#include <linux/reset.h>
#include <linux/sched.h>
#include <linux/spinlock.h>
#include <linux/string.h>
#include <linux/v4l2-controls.h>
#include <linux/videodev2.h>
#include <linux/wait.h>
#include <linux/workqueue.h>
#include <media/v4l2-ctrls.h>
#include <media/v4l2-dev.h>
#include <media/v4l2-device.h>
#include <media/v4l2-dv-timings.h>
#include <media/v4l2-event.h>
#include <media/v4l2-ioctl.h>
#include <media/videobuf2-dma-contig.h>
#include <uapi/linux/npcm-video.h>

#define DEVICE_NAME			"npcm-video"

#define MAX_FRAME_RATE			60
#define MAX_WIDTH			1920
#define MAX_HEIGHT			1200
#define MIN_WIDTH			320
#define MIN_HEIGHT			240
#define MIN_LP				512
#define MAX_LP				4096
#define RECT_W				16
#define RECT_H				16
#define BITMAP_SIZE			32

#define VCD_MODULE_NAME			"vcd"
#define ECE_MODULE_NAME			"ece"

/* VCD Registers */
#define VCD_DIFF_TBL			0x0000
#define VCD_FBA_ADR			0x8000
#define VCD_FBB_ADR			0x8004

#define VCD_FB_LP			0x8008
#define  VCD_FBA_LP			GENMASK(15, 0)
#define  VCD_FBB_LP			GENMASK(31, 16)

#define VCD_CAP_RES			0x800c
#define  VCD_CAP_RES_VERT_RES		GENMASK(10, 0)
#define  VCD_CAP_RES_HOR_RES		GENMASK(26, 16)

#define VCD_MODE			0x8014
#define  VCD_MODE_VCDE			BIT(0)
#define  VCD_MODE_CM565			BIT(1)
#define  VCD_MODE_IDBC			BIT(3)
#define  VCD_MODE_COLOR_CNVRT		GENMASK(5, 4)
#define   VCD_MODE_COLOR_CNVRT_NO_CNVRT	0
#define   VCD_MODE_COLOR_CNVRT_RGB_222	1
#define   VCD_MODE_COLOR_CNVRT_666_MODE	2
#define   VCD_MODE_COLOR_CNVRT_RGB_888	3
#define  VCD_MODE_KVM_BW_SET		BIT(16)

#define VCD_CMD				0x8018
#define  VCD_CMD_GO			BIT(0)
#define  VCD_CMD_RST			BIT(1)
#define  VCD_CMD_OPERATION		GENMASK(6, 4)
#define   VCD_CMD_OPERATION_CAPTURE	0
#define   VCD_CMD_OPERATION_COMPARE	2

#define	VCD_STAT			0x801c
#define	 VCD_STAT_DONE			BIT(0)
#define	 VCD_STAT_IFOT			BIT(2)
#define	 VCD_STAT_IFOR			BIT(3)
#define	 VCD_STAT_BUSY			BIT(30)
#define	VCD_STAT_CLEAR			0x3fff

#define VCD_INTE			0x8020
#define  VCD_INTE_DONE_IE		BIT(0)
#define  VCD_INTE_IFOT_IE		BIT(2)
#define  VCD_INTE_IFOR_IE		BIT(3)

#define VCD_RCHG			0x8028
#define VCD_RCHG_TIM_PRSCL		GENMASK(12, 9)

#define VCD_FIFO			0x805c
#define  VCD_FIFO_TH			0x100350ff

#define VCD_MAX_SRC_BUFFER_SIZE		0x500000 /* 1920 x 1200 x 2 bpp */
#define VCD_KVM_BW_PCLK			120000000UL
#define VCD_BUSY_TIMEOUT_US		300000

/* ECE Registers */
#define ECE_DDA_CTRL			0x0000
#define  ECE_DDA_CTRL_ECEEN		BIT(0)
#define  ECE_DDA_CTRL_INTEN		BIT(8)

#define ECE_DDA_STS			0x0004
#define  ECE_DDA_STS_CDREADY		BIT(8)
#define  ECE_DDA_STS_ACDRDY		BIT(10)

#define ECE_FBR_BA			0x0008
#define ECE_ED_BA			0x000c
#define ECE_RECT_XY			0x0010

#define ECE_RECT_DIMEN			0x0014
#define  ECE_RECT_DIMEN_WR		GENMASK(10, 0)
#define  ECE_RECT_DIMEN_WLTR		GENMASK(14, 11)
#define  ECE_RECT_DIMEN_HR		GENMASK(26, 16)
#define  ECE_RECT_DIMEN_HLTR		GENMASK(30, 27)

#define ECE_RESOL			0x001c
#define  ECE_RESOL_FB_LP_512		0
#define  ECE_RESOL_FB_LP_1024		1
#define  ECE_RESOL_FB_LP_2048		2
#define  ECE_RESOL_FB_LP_2560		3
#define  ECE_RESOL_FB_LP_4096		4

#define ECE_HEX_CTRL			0x0040
#define  ECE_HEX_CTRL_ENCDIS		BIT(0)
#define  ECE_HEX_CTRL_ENC_GAP		GENMASK(12, 8)

#define ECE_HEX_RECT_OFFSET		0x0048
#define  ECE_HEX_RECT_OFFSET_MASK	GENMASK(22, 0)

#define ECE_TILE_W			16
#define ECE_TILE_H			16
#define ECE_POLL_TIMEOUT_US		300000

/* GCR Registers */
#define INTCR				0x3c
#define  INTCR_GFXIFDIS			GENMASK(9, 8)
#define  INTCR_DEHS			BIT(27)

#define INTCR2				0x60
#define  INTCR2_GIRST2			BIT(2)
#define  INTCR2_GIHCRST			BIT(5)
#define  INTCR2_GIVCRST			BIT(6)

#define INTCR3				0x9c
#define  INTCR3_GMMAP			GENMASK(10, 8)
#define   INTCR3_GMMAP_128MB		0
#define   INTCR3_GMMAP_256MB		1
#define   INTCR3_GMMAP_512MB		2
#define   INTCR3_GMMAP_1GB		3
#define   INTCR3_GMMAP_2GB		4

#define INTCR4				0xc0
#define  INTCR4_GMMAP			GENMASK(22, 16)
#define  INTCR4_GMMAP_512MB		0x1f
#define  INTCR4_GMMAP_512MB_ECC		0x1b
#define  INTCR4_GMMAP_1GB		0x3f
#define  INTCR4_GMMAP_1GB_ECC		0x37
#define  INTCR4_GMMAP_2GB		0x7f
#define  INTCR4_GMMAP_2GB_ECC		0x6f

#define ADDR_GMMAP_128MB		0x07000000
#define ADDR_GMMAP_256MB		0x0f000000
#define ADDR_GMMAP_512MB		0x1f000000
#define ADDR_GMMAP_512MB_ECC		0x1b000000
#define ADDR_GMMAP_1GB			0x3f000000
#define ADDR_GMMAP_1GB_ECC		0x37000000
#define ADDR_GMMAP_2GB			0x7f000000
#define ADDR_GMMAP_2GB_ECC		0x6f000000

#define GMMAP_LENGTH			0xc00000 /* Total 16MB, but 4MB preserved*/

#define MFSEL1				0x0c
#define  MFSEL1_DVH1SEL			BIT(27)

/* GFXI Register */
#define DISPST				0x00
#define  DISPST_HSCROFF			BIT(1)
#define  DISPST_MGAMODE			BIT(7)

#define HVCNTL				0x10
#define  HVCNTL_MASK			GENMASK(7, 0)

#define HVCNTH				0x14
#define  HVCNTH_MASK			GENMASK(2, 0)

#define VVCNTL				0x20
#define  VVCNTL_MASK			GENMASK(7, 0)

#define VVCNTH				0x24
#define  VVCNTH_MASK			GENMASK(2, 0)

#define GPLLINDIV			0x40
#define  GPLLINDIV_MASK			GENMASK(5, 0)
#define  GPLLINDIV_GPLLFBDV8		BIT(7)

#define GPLLFBDIV			0x44
#define  GPLLFBDIV_MASK			GENMASK(7, 0)

#define GPLLST				0x48
#define  GPLLST_PLLOTDIV1		GENMASK(2, 0)
#define  GPLLST_PLLOTDIV2		GENMASK(5, 3)
#define  GPLLST_GPLLFBDV109		GENMASK(7, 6)

struct npcm_video_addr {
	size_t size;
	dma_addr_t dma;
	void *virt;
};

struct npcm_video_buffer {
	struct vb2_v4l2_buffer vb;
	struct list_head link;
};

#define to_npcm_video_buffer(x) \
	container_of((x), struct npcm_video_buffer, vb)

enum {
	VIDEO_STREAMING,
	VIDEO_FRAME_INPRG,
	VIDEO_STOPPED,
};

struct rect_list {
	struct v4l2_clip clip;
	struct list_head list;
};

struct rect_list_info {
	struct rect_list *list;
	struct rect_list *first;
	struct list_head *head;
	unsigned int index;
	unsigned int tile_perline;
	unsigned int tile_perrow;
	unsigned int offset_perline;
	unsigned int tile_size;
	unsigned int tile_cnt;
};

struct npcm_ece {
	struct regmap *regmap;
	atomic_t clients;
	struct reset_control *reset;
};

struct npcm_video {
	struct regmap *gcr_regmap;
	struct regmap *gfx_regmap;
	struct regmap *vcd_regmap;

	struct device *dev;
	struct v4l2_ctrl_handler ctrl_handler;
	struct v4l2_device v4l2_dev;
	struct v4l2_pix_format pix_fmt;
	struct v4l2_bt_timings active_timings;
	struct v4l2_bt_timings detected_timings;
	u32 v4l2_input_status;
	struct vb2_queue queue;
	struct video_device vdev;
	struct mutex video_lock;	/* v4l2 and videobuf2 lock */

	struct list_head buffers;
	spinlock_t lock;		/* buffer list lock */
	unsigned long flags;
	unsigned int sequence;

	size_t max_buffer_size;
	struct npcm_video_addr src;
	struct reset_control *reset;
	struct npcm_ece ece;

	unsigned int frame_rate;
	unsigned int vb_index;
	u32 bytesperline;
	u8 bytesperpixel;
	u32 rect_cnt;
	u8 num_buffers;
	struct list_head *list;
	u32 *rect;
	int ctrl_cmd;
	int op_cmd;
};

#define to_npcm_video(x) container_of((x), struct npcm_video, v4l2_dev)

static const struct v4l2_dv_timings_cap npcm_video_timings_cap = {
	.type = V4L2_DV_BT_656_1120,
	.bt = {
		.min_width = MIN_WIDTH,
		.max_width = MAX_WIDTH,
		.min_height = MIN_HEIGHT,
		.max_height = MAX_HEIGHT,
		.min_pixelclock = 6574080, /* 640 x 480 x 24Hz */
		.max_pixelclock = 138240000, /* 1920 x 1200 x 60Hz */
		.standards = V4L2_DV_BT_STD_CEA861 | V4L2_DV_BT_STD_DMT |
			     V4L2_DV_BT_STD_CVT | V4L2_DV_BT_STD_GTF,
		.capabilities = V4L2_DV_BT_CAP_PROGRESSIVE |
				V4L2_DV_BT_CAP_REDUCED_BLANKING |
				V4L2_DV_BT_CAP_CUSTOM,
	},
};

static DECLARE_BITMAP(bitmap, BITMAP_SIZE);

static void npcm_video_ece_prepend_rect_header(u8 *addr, u16 x, u16 y, u16 w,
					       u16 h)
{
	__be16 x_pos = cpu_to_be16(x);
	__be16 y_pos = cpu_to_be16(y);
	__be16 width = cpu_to_be16(w);
	__be16 height = cpu_to_be16(h);
	__be32 encoding = cpu_to_be32(5); /* Hextile encoding */

	memcpy(addr, &x_pos, 2);
	memcpy(addr + 2, &y_pos, 2);
	memcpy(addr + 4, &width, 2);
	memcpy(addr + 6, &height, 2);
	memcpy(addr + 8, &encoding, 4);
}

static unsigned int npcm_video_ece_get_ed_size(struct npcm_video *video,
					       u32 offset, void *addr)
{
	struct regmap *ece = video->ece.regmap;
	u32 size, gap, val;
	int ret;

	ret = regmap_read_poll_timeout(ece, ECE_DDA_STS, val,
				       (val & ECE_DDA_STS_CDREADY),
				       0, ECE_POLL_TIMEOUT_US);

	if (ret) {
		dev_warn(video->dev, "Wait for ECE_DDA_STS_CDREADY timeout\n");
		return 0;
	}

	size = readl(addr + offset);

	regmap_read(ece, ECE_HEX_CTRL, &val);
	gap = FIELD_GET(ECE_HEX_CTRL_ENC_GAP, val);

	dev_dbg(video->dev, "offset = %u, ed_size = %u, gap = %u\n", offset,
		size, gap);

	return size + gap;
}

static void npcm_video_ece_enc_rect(struct npcm_video *video, u32 r_off_x,
				    u32 r_off_y, u32 r_w, u32 r_h)
{
	struct regmap *ece = video->ece.regmap;
	u32 rect_offset = (r_off_y * video->bytesperline) + (r_off_x * 2);
	u32 temp;
	u32 w_tile;
	u32 h_tile;
	u32 w_size = ECE_TILE_W;
	u32 h_size = ECE_TILE_H;

	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_ECEEN, 0);
	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_ECEEN,
			   ECE_DDA_CTRL_ECEEN);
	regmap_write(ece, ECE_DDA_STS, ECE_DDA_STS_CDREADY |
		     ECE_DDA_STS_ACDRDY);
	regmap_write(ece, ECE_RECT_XY, rect_offset);

	w_tile = r_w / ECE_TILE_W;
	h_tile = r_h / ECE_TILE_H;

	if (r_w % ECE_TILE_W) {
		w_tile += 1;
		w_size = r_w % ECE_TILE_W;
	}

	if (r_h % ECE_TILE_H || !h_tile) {
		h_tile += 1;
		h_size = r_h % ECE_TILE_H;
	}

	temp = FIELD_PREP(ECE_RECT_DIMEN_WLTR, w_size - 1) |
	       FIELD_PREP(ECE_RECT_DIMEN_HLTR, h_size - 1) |
	       FIELD_PREP(ECE_RECT_DIMEN_WR, w_tile - 1) |
	       FIELD_PREP(ECE_RECT_DIMEN_HR, h_tile - 1);

	regmap_write(ece, ECE_RECT_DIMEN, temp);
}

static u32 npcm_video_ece_read_rect_offset(struct npcm_video *video)
{
	struct regmap *ece = video->ece.regmap;
	u32 offset;

	regmap_read(ece, ECE_HEX_RECT_OFFSET, &offset);
	return FIELD_GET(ECE_HEX_RECT_OFFSET_MASK, offset);
}

/*
 * Set the line pitch (in bytes) for the frame buffers.
 * Can be on of those values: 512, 1024, 2048, 2560 or 4096 bytes.
 */
static void npcm_video_ece_set_lp(struct npcm_video *video, u32 pitch)
{
	u32 lp;
	struct regmap *ece = video->ece.regmap;

	switch (pitch) {
	case 512:
		lp = ECE_RESOL_FB_LP_512;
		break;
	case 1024:
		lp = ECE_RESOL_FB_LP_1024;
		break;
	case 2048:
		lp = ECE_RESOL_FB_LP_2048;
		break;
	case 2560:
		lp = ECE_RESOL_FB_LP_2560;
		break;
	case 4096:
		lp = ECE_RESOL_FB_LP_4096;
		break;
	default:
		return;
	}

	regmap_write(ece, ECE_RESOL, lp);
}

static void npcm_video_ece_set_fb_addr(struct npcm_video *video, u32 buffer)
{
	struct regmap *ece = video->ece.regmap;

	regmap_write(ece, ECE_FBR_BA, buffer);
}

static void npcm_video_ece_set_enc_dba(struct npcm_video *video, u32 addr)
{
	struct regmap *ece = video->ece.regmap;

	regmap_write(ece, ECE_ED_BA, addr);
}

static void npcm_video_ece_clear_rect_offset(struct npcm_video *video)
{
	struct regmap *ece = video->ece.regmap;

	regmap_write(ece, ECE_HEX_RECT_OFFSET, 0);
}

static void npcm_video_ece_ctrl_reset(struct npcm_video *video)
{
	struct regmap *ece = video->ece.regmap;

	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_ECEEN, 0);
	regmap_update_bits(ece, ECE_HEX_CTRL, ECE_HEX_CTRL_ENCDIS,
			   ECE_HEX_CTRL_ENCDIS);
	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_ECEEN,
			   ECE_DDA_CTRL_ECEEN);
	regmap_update_bits(ece, ECE_HEX_CTRL, ECE_HEX_CTRL_ENCDIS, 0);

	npcm_video_ece_clear_rect_offset(video);
}

static void npcm_video_ece_ip_reset(struct npcm_video *video)
{
	/*
	 * After resetting a module and clearing the reset bit, it should wait
	 * at least 10 us before accessing the module.
	 */
	reset_control_assert(video->ece.reset);
	usleep_range(10, 20);
	reset_control_deassert(video->ece.reset);
	usleep_range(10, 20);
}

static int npcm_video_ece_init(struct npcm_video *video)
{
	npcm_video_ece_ip_reset(video);
	npcm_video_ece_ctrl_reset(video);

	return 0;
}

static int npcm_video_ece_stop(struct npcm_video *video)
{
	struct regmap *ece = video->ece.regmap;

	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_ECEEN, 0);
	regmap_update_bits(ece, ECE_DDA_CTRL, ECE_DDA_CTRL_INTEN, 0);
	regmap_update_bits(ece, ECE_HEX_CTRL, ECE_HEX_CTRL_ENCDIS,
			   ECE_HEX_CTRL_ENCDIS);
	npcm_video_ece_clear_rect_offset(video);

	return 0;
}

static bool npcm_video_alloc_buf(struct npcm_video *video,
				 struct npcm_video_addr *addr, size_t size)
{
	if (size > VCD_MAX_SRC_BUFFER_SIZE)
		size = VCD_MAX_SRC_BUFFER_SIZE;

	addr->virt = dma_alloc_coherent(video->dev, size, &addr->dma,
					GFP_KERNEL);

	if (!addr->virt)
		return false;

	addr->size = size;
	return true;
}

static void npcm_video_free_buf(struct npcm_video *video,
				struct npcm_video_addr *addr)
{
	dma_free_coherent(video->dev, addr->size, addr->virt, addr->dma);
	addr->size = 0;
	addr->dma = 0ULL;
	addr->virt = NULL;
}

static void npcm_video_free_diff_table(struct npcm_video *video)
{
	struct list_head *head, *pos, *nx;
	struct rect_list *tmp;
	unsigned int i;

	for (i = 0; i < video->num_buffers; i++) {
		head = &video->list[i];
		list_for_each_safe(pos, nx, head) {
			tmp = list_entry(pos, struct rect_list, list);
			if (tmp) {
				list_del(&tmp->list);
				kfree(tmp);
			}
		}
	}
}

static int npcm_video_add_rect(struct npcm_video *video, unsigned int index,
			       u32 x, u32 y, u32 w, u32 h)
{
	struct list_head *head = &video->list[index];
	struct rect_list *list = NULL;
	struct v4l2_rect *r;

	list = kzalloc(sizeof(*list), GFP_KERNEL);
	if (!list)
		return 0;

	r = &list->clip.c;
	r->left = x;
	r->top = y;
	r->width = w;
	r->height = h;

	list_add_tail(&list->list, head);
	return 1;
}

static void npcm_video_merge_rect(struct npcm_video *video,
				  struct rect_list_info *info)
{
	struct list_head *head = info->head;
	struct rect_list *list = info->list;
	struct rect_list *first = info->first;
	struct v4l2_rect *r = &list->clip.c;
	struct v4l2_rect *f = &first->clip.c;

	if (!first) {
		first = list;
		info->first = first;
		list_add_tail(&list->list, head);
		video->rect_cnt++;
	} else {
		if ((r->left == (f->left + f->width)) && r->top == f->top) {
			f->width += r->width;
			kfree(list);
		} else if ((r->top == (f->top + f->height)) &&
			   (r->left == f->left)) {
			f->height += r->height;
			kfree(list);
		} else if (((r->top > f->top) &&
			   (r->top < (f->top + f->height))) &&
			   ((r->left > f->left) &&
			   (r->left < (f->left + f->width)))) {
			kfree(list);
		} else {
			list_add_tail(&list->list, head);
			video->rect_cnt++;
			info->first = list;
		}
	}
}

static struct rect_list *npcm_video_new_rect(struct npcm_video *video,
					     unsigned int offset,
					     unsigned int index)
{
	struct v4l2_bt_timings *act = &video->active_timings;
	struct rect_list *list = NULL;
	struct v4l2_rect *r;

	list = kzalloc(sizeof(*list), GFP_KERNEL);
	if (!list)
		return NULL;

	r = &list->clip.c;

	r->left = (offset << 4);
	r->top = (index >> 2);
	r->width = RECT_W;
	r->height = RECT_H;
	if ((r->left + RECT_W) > act->width)
		r->width = act->width - r->left;
	if ((r->top + RECT_H) > act->height)
		r->height = act->height - r->top;

	return list;
}

static int npcm_video_find_rect(struct npcm_video *video,
				struct rect_list_info *info,
				unsigned int offset)
{
	if (offset < info->tile_perline) {
		info->list = npcm_video_new_rect(video, offset, info->index);
		if (!info->list)
			return -ENOMEM;

		npcm_video_merge_rect(video, info);
	}
	return 0;
}

static int npcm_video_build_table(struct npcm_video *video,
				  struct rect_list_info *info)
{
	int ret;
	unsigned int j, bit;
	u32 value;
	struct regmap *vcd = video->vcd_regmap;

	for (j = 0; j < info->offset_perline; j += 4) {
		regmap_read(vcd, VCD_DIFF_TBL + (j + info->index), &value);

		bitmap_from_arr32(bitmap, &value, BITMAP_SIZE);

		for_each_set_bit(bit, bitmap, BITMAP_SIZE) {
			ret = npcm_video_find_rect(video, info, bit + (j << 3));
			if (ret < 0)
				return ret;
		}
	}
	info->index += 64;
	return info->tile_perline;
}

static int npcm_video_get_rect_list(struct npcm_video *video,
				    unsigned int index)
{
	struct v4l2_bt_timings *act = &video->active_timings;
	struct rect_list_info info;
	int ret = 0;
	unsigned int mod, tile_cnt = 0;

	memset(&info, 0, sizeof(struct rect_list_info));
	info.head = &video->list[index];

	info.tile_perline = act->width >> 4;
	mod = act->width % RECT_W;
	if (mod != 0)
		info.tile_perline += 1;

	info.tile_perrow = act->height >> 4;
	mod = act->height % RECT_H;
	if (mod != 0)
		info.tile_perrow += 1;

	info.tile_size = info.tile_perrow * info.tile_perline;

	info.offset_perline = info.tile_perline >> 5;
	mod = info.tile_perline % 32;
	if (mod != 0)
		info.offset_perline += 1;

	info.offset_perline *= 4;

	do {
		ret = npcm_video_build_table(video, &info);
		if (ret < 0)
			return ret;
		tile_cnt += ret;
	} while (tile_cnt < info.tile_size);

	return ret;
}

static unsigned int npcm_video_get_rect_count(struct npcm_video *video)
{
	struct list_head *head, *pos, *nx;
	struct rect_list *tmp;
	unsigned int count;

	if (video->list && video->rect) {
		count = video->rect[video->vb_index];
		head = &video->list[video->vb_index];

		list_for_each_safe(pos, nx, head) {
			tmp = list_entry(pos, struct rect_list, list);
			list_del(&tmp->list);
			kfree(tmp);
		}
	}

	return count;
}

static u8 npcm_video_is_mga(struct npcm_video *video)
{
	struct regmap *gfxi = video->gfx_regmap;
	u32 dispst;

	regmap_read(gfxi, DISPST, &dispst);
	return ((dispst & DISPST_MGAMODE) == DISPST_MGAMODE);
}

static u32 npcm_video_hres(struct npcm_video *video)
{
	struct regmap *gfxi = video->gfx_regmap;
	u32 hvcnth, hvcntl, apb_hor_res;

	regmap_read(gfxi, HVCNTH, &hvcnth);
	regmap_read(gfxi, HVCNTL, &hvcntl);
	apb_hor_res = (((hvcnth & HVCNTH_MASK) << 8) +
		       (hvcntl & HVCNTL_MASK) + 1);

	return apb_hor_res;
}

static u32 npcm_video_vres(struct npcm_video *video)
{
	struct regmap *gfxi = video->gfx_regmap;
	u32 vvcnth, vvcntl, apb_ver_res;

	regmap_read(gfxi, VVCNTH, &vvcnth);
	regmap_read(gfxi, VVCNTL, &vvcntl);

	apb_ver_res = (((vvcnth & VVCNTH_MASK) << 8) + (vvcntl & VVCNTL_MASK));

	return apb_ver_res;
}

static int npcm_video_capres(struct npcm_video *video, u32 hor_res,
			     u32 vert_res)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 res, cap_res;

	if (hor_res > MAX_WIDTH || vert_res > MAX_HEIGHT)
		return -EINVAL;

	res = FIELD_PREP(VCD_CAP_RES_VERT_RES, vert_res) |
	      FIELD_PREP(VCD_CAP_RES_HOR_RES, hor_res);

	regmap_write(vcd, VCD_CAP_RES, res);
	regmap_read(vcd, VCD_CAP_RES, &cap_res);

	if (cap_res != res)
		return -EINVAL;

	return 0;
}

static void npcm_video_vcd_ip_reset(struct npcm_video *video)
{
	/*
	 * After resetting a module and clearing the reset bit, it should wait
	 * at least 10 us before accessing the module.
	 */
	reset_control_assert(video->reset);
	usleep_range(10, 20);
	reset_control_deassert(video->reset);
	usleep_range(10, 20);
}

static void npcm_video_vcd_state_machine_reset(struct npcm_video *video)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 stat;
	int ret;

	regmap_update_bits(vcd, VCD_MODE, VCD_MODE_VCDE, 0);

	regmap_update_bits(vcd, VCD_MODE, VCD_MODE_IDBC, 0);

	regmap_update_bits(vcd, VCD_CMD, VCD_CMD_RST, VCD_CMD_RST);

	ret = regmap_read_poll_timeout(vcd, VCD_STAT, stat,
				       (stat & VCD_STAT_DONE), 0,
				       ECE_POLL_TIMEOUT_US);

	if (ret) {
		dev_warn(video->dev, "Wait for VCD_STAT_DONE timeout\n");
		return;
	}

	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	regmap_update_bits(vcd, VCD_MODE, VCD_MODE_VCDE, VCD_MODE_VCDE);

	regmap_update_bits(vcd, VCD_MODE, VCD_MODE_IDBC, VCD_MODE_IDBC);
}

static int npcm_video_gfx_reset(struct npcm_video *video)
{
	struct regmap *gcr = video->gcr_regmap;

	regmap_update_bits(gcr, INTCR2, INTCR2_GIRST2, INTCR2_GIRST2);

	npcm_video_vcd_state_machine_reset(video);

	regmap_update_bits(gcr, INTCR2, INTCR2_GIRST2, 0);

	return 0;
}

static void npcm_video_kvm_bw(struct npcm_video *video, u8 bandwidth)
{
	struct regmap *vcd = video->vcd_regmap;

	if (!npcm_video_is_mga(video))
		bandwidth = 1;

	if (bandwidth)
		regmap_update_bits(vcd, VCD_MODE, VCD_MODE_KVM_BW_SET,
				   VCD_MODE_KVM_BW_SET);
	else
		regmap_update_bits(vcd, VCD_MODE, VCD_MODE_KVM_BW_SET, 0);
}

static u32 npcm_video_pclk(struct npcm_video *video)
{
	struct regmap *gfxi = video->gfx_regmap;
	u32 tmp, pllfbdiv, pllinotdiv, gpllfbdiv;
	u8 gpllfbdv109, gpllfbdv8, gpllindiv;
	u8 gpllst_pllotdiv1, gpllst_pllotdiv2;

	regmap_read(gfxi, GPLLST, &tmp);
	gpllfbdv109 = FIELD_GET(GPLLST_GPLLFBDV109, tmp);
	gpllst_pllotdiv1 = FIELD_GET(GPLLST_PLLOTDIV1, tmp);
	gpllst_pllotdiv2 = FIELD_GET(GPLLST_PLLOTDIV2, tmp);

	regmap_read(gfxi, GPLLINDIV, &tmp);
	gpllfbdv8 = FIELD_GET(GPLLINDIV_GPLLFBDV8, tmp);
	gpllindiv = FIELD_GET(GPLLINDIV_MASK, tmp);

	regmap_read(gfxi, GPLLFBDIV, &tmp);
	gpllfbdiv = FIELD_GET(GPLLFBDIV_MASK, tmp);

	pllfbdiv = (512 * gpllfbdv109 + 256 * gpllfbdv8 + gpllfbdiv);
	pllinotdiv = (gpllindiv * gpllst_pllotdiv1 * gpllst_pllotdiv2);
	if (pllfbdiv == 0 || pllinotdiv == 0)
		return 0;

	return ((pllfbdiv * 25000) / pllinotdiv) * 1000;
}

static int npcm_video_get_bpp(struct npcm_video *video)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 mode, color_cnvr;

	regmap_read(vcd, VCD_MODE, &mode);

	color_cnvr = FIELD_GET(VCD_MODE_COLOR_CNVRT, mode);

	switch (color_cnvr) {
	case VCD_MODE_COLOR_CNVRT_NO_CNVRT:
		return 2;
	case VCD_MODE_COLOR_CNVRT_RGB_222:
	case VCD_MODE_COLOR_CNVRT_666_MODE:
		return 1;
	case VCD_MODE_COLOR_CNVRT_RGB_888:
		return 4;
	}
	return 0;
}

/*
 * Pitch must be a power of 2, >= linebytes,
 * at least 512, and no more than 4096.
 */
static void npcm_video_set_linepitch(struct npcm_video *video, u32 linebytes)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 pitch = MIN_LP;

	while ((pitch < linebytes) && (pitch < MAX_LP))
		pitch *= 2;

	regmap_write(vcd, VCD_FB_LP, FIELD_PREP(VCD_FBA_LP, pitch) |
		     FIELD_PREP(VCD_FBB_LP, pitch));
}

static u32 npcm_video_get_linepitch(struct npcm_video *video)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 linepitch;

	regmap_read(vcd, VCD_FB_LP, &linepitch);

	return FIELD_GET(VCD_FBA_LP, linepitch);
}

static int npcm_video_command(struct npcm_video *video, u32 value)
{
	struct regmap *vcd = video->vcd_regmap;
	u32 cmd;

	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	regmap_read(vcd, VCD_CMD, &cmd);
	cmd |= FIELD_PREP(VCD_CMD_OPERATION, value);

	regmap_write(vcd, VCD_CMD, cmd);
	regmap_update_bits(vcd, VCD_CMD, VCD_CMD_GO, VCD_CMD_GO);
	video->op_cmd = value;

	return 0;
}

static int npcm_video_init_reg(struct npcm_video *video)
{
	struct regmap *gcr = video->gcr_regmap;
	struct regmap *vcd = video->vcd_regmap;

	/* Selects Data Enable */
	regmap_update_bits(gcr, INTCR, INTCR_DEHS, 0);

	/* Enable display of KVM GFX and access to memory */
	regmap_update_bits(gcr, INTCR, INTCR_GFXIFDIS, 0);

	/* Active Vertical/Horizontal Counters Reset */
	regmap_update_bits(gcr, INTCR2, INTCR2_GIHCRST | INTCR2_GIVCRST,
			   INTCR2_GIHCRST | INTCR2_GIVCRST);

	/* Select KVM GFX input */
	regmap_update_bits(gcr, MFSEL1, MFSEL1_DVH1SEL, 0);

	/* Reset video modules */
	npcm_video_vcd_ip_reset(video);
	npcm_video_gfx_reset(video);

	/* Set the FIFO thresholds */
	regmap_write(vcd, VCD_FIFO, VCD_FIFO_TH);

	/* Set video mode */
	regmap_update_bits(vcd, VCD_MODE, 0xffffffff,
			   VCD_MODE_VCDE | VCD_MODE_CM565 |
			   VCD_MODE_IDBC | VCD_MODE_KVM_BW_SET);

	regmap_write(vcd, VCD_RCHG, FIELD_PREP(VCD_RCHG_TIM_PRSCL, 0xf));

	return 0;
}

static int npcm_video_start_frame(struct npcm_video *video)
{
	unsigned long flags;
	struct npcm_video_buffer *buf;
	struct regmap *vcd = video->vcd_regmap;
	u32 val;
	int ret;

	if (video->v4l2_input_status) {
		dev_dbg(video->dev, "No video signal; skip capture frame\n");
		return 0;
	}

	ret = regmap_read_poll_timeout(vcd, VCD_STAT, val,
				       !(val & VCD_STAT_BUSY), 1000,
				       VCD_BUSY_TIMEOUT_US);

	if (ret) {
		dev_err(video->dev, "Wait for VCD_STAT_BUSY timeout\n");
		return -EBUSY;
	}

	spin_lock_irqsave(&video->lock, flags);
	buf = list_first_entry_or_null(&video->buffers,
				       struct npcm_video_buffer, link);
	if (!buf) {
		spin_unlock_irqrestore(&video->lock, flags);
		dev_dbg(video->dev, "No empty buffers; skip capture frame\n");
		return 0;
	}

	set_bit(VIDEO_FRAME_INPRG, &video->flags);
	spin_unlock_irqrestore(&video->lock, flags);

	npcm_video_vcd_state_machine_reset(video);

	regmap_update_bits(vcd, VCD_INTE, VCD_INTE_DONE_IE | VCD_INTE_IFOT_IE |
			   VCD_INTE_IFOR_IE, VCD_INTE_DONE_IE |
			   VCD_INTE_IFOT_IE | VCD_INTE_IFOR_IE);

	npcm_video_command(video, video->ctrl_cmd);

	return 0;
}

static void npcm_video_bufs_done(struct npcm_video *video,
				 enum vb2_buffer_state state)
{
	unsigned long flags;
	struct npcm_video_buffer *buf;

	spin_lock_irqsave(&video->lock, flags);
	list_for_each_entry(buf, &video->buffers, link)
		vb2_buffer_done(&buf->vb.vb2_buf, state);
	INIT_LIST_HEAD(&video->buffers);
	spin_unlock_irqrestore(&video->lock, flags);
}

static void npcm_video_get_diff_rect(struct npcm_video *video,
				     unsigned int index)
{
	u32 width = video->active_timings.width;
	u32 height = video->active_timings.height;

	if (video->op_cmd != VCD_CMD_OPERATION_CAPTURE) {
		video->rect_cnt = 0;
		npcm_video_get_rect_list(video, index);
		video->rect[index] = video->rect_cnt;
	} else {
		video->rect[index] = npcm_video_add_rect(video, index, 0, 0,
							 width, height);
	}
}

static irqreturn_t npcm_video_irq(int irq, void *arg)
{
	struct npcm_video *video = arg;
	struct regmap *vcd = video->vcd_regmap;
	struct npcm_video_buffer *buf;
	struct rect_list *rect_list;
	struct v4l2_rect *rect;
	u32 status, ed_offset;
	void *addr;
	dma_addr_t vb_dma_addr;
	unsigned int index, ed_size, total_size;

	regmap_read(vcd, VCD_STAT, &status);
	dev_dbg(video->dev, "VCD irq status 0x%x\n", status);

	regmap_write(vcd, VCD_INTE, 0);
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	if (test_bit(VIDEO_STOPPED, &video->flags) ||
	    !test_bit(VIDEO_STREAMING, &video->flags)) {
		clear_bit(VIDEO_FRAME_INPRG, &video->flags);
		return IRQ_NONE;
	}

	if (status & VCD_STAT_DONE) {
		spin_lock(&video->lock);
		buf = list_first_entry_or_null(&video->buffers,
					       struct npcm_video_buffer,
					       link);

		if (!buf) {
			spin_unlock(&video->lock);
			clear_bit(VIDEO_FRAME_INPRG, &video->flags);
			return IRQ_NONE;
		}

		addr = vb2_plane_vaddr(&buf->vb.vb2_buf, 0);
		vb_dma_addr = vb2_dma_contig_plane_dma_addr(&buf->vb.vb2_buf, 0);
		index = buf->vb.vb2_buf.index;

		npcm_video_ece_ctrl_reset(video);
		npcm_video_ece_clear_rect_offset(video);

		npcm_video_ece_set_fb_addr(video, video->src.dma);

		/* Set base address of encoded data to video buffer */
		npcm_video_ece_set_enc_dba(video, vb_dma_addr);

		npcm_video_ece_set_lp(video, video->bytesperline);
		npcm_video_get_diff_rect(video, index);

		total_size = 0;

		list_for_each_entry(rect_list, &video->list[index], list) {
			rect = &rect_list->clip.c;
			ed_offset = npcm_video_ece_read_rect_offset(video);

			npcm_video_ece_enc_rect(video, rect->left,
						rect->top, rect->width,
						rect->height);
			ed_size = npcm_video_ece_get_ed_size(video,
							     ed_offset,
							     addr);

			npcm_video_ece_prepend_rect_header(addr + ed_offset,
							   rect->left,
							   rect->top,
							   rect->width,
							   rect->height);

			total_size += ed_size;
		}

		vb2_set_plane_payload(&buf->vb.vb2_buf, 0, total_size);
		buf->vb.vb2_buf.timestamp = ktime_get_ns();
		buf->vb.sequence = video->sequence++;
		buf->vb.field = V4L2_FIELD_NONE;
		vb2_buffer_done(&buf->vb.vb2_buf, VB2_BUF_STATE_DONE);
		list_del(&buf->link);

		spin_unlock(&video->lock);

		clear_bit(VIDEO_FRAME_INPRG, &video->flags);
	}

	if (status & VCD_STAT_IFOR || status & VCD_STAT_IFOT) {
		dev_warn(video->dev, "VCD FIFO overrun or over thresholds\n");
		npcm_video_vcd_ip_reset(video);
		npcm_video_gfx_reset(video);
		npcm_video_start_frame(video);
	}

	return IRQ_HANDLED;
}

static void npcm_video_clear_gmmap(struct npcm_video *video)
{
	struct regmap *gcr = video->gcr_regmap;
	u32 intcr, gmmap;
	void __iomem *baseptr;

	if (of_device_is_compatible(video->dev->of_node, "nuvoton,npcm750-video")) {
		regmap_read(gcr, INTCR3, &intcr);
		gmmap = FIELD_GET(INTCR3_GMMAP, intcr);

		switch (gmmap) {
		case INTCR3_GMMAP_128MB:
			baseptr = ioremap_wc(ADDR_GMMAP_128MB, GMMAP_LENGTH);
			break;
		case INTCR3_GMMAP_256MB:
			baseptr = ioremap_wc(ADDR_GMMAP_256MB, GMMAP_LENGTH);
			break;
		case INTCR3_GMMAP_512MB:
			baseptr = ioremap_wc(ADDR_GMMAP_512MB, GMMAP_LENGTH);
			break;
		case INTCR3_GMMAP_1GB:
			baseptr = ioremap_wc(ADDR_GMMAP_1GB, GMMAP_LENGTH);
			break;
		case INTCR3_GMMAP_2GB:
			baseptr = ioremap_wc(ADDR_GMMAP_2GB, GMMAP_LENGTH);
			break;
		}
	} else if (of_device_is_compatible(video->dev->of_node, "nuvoton,npcm845-video")) {
		regmap_read(gcr, INTCR4, &intcr);
		gmmap = FIELD_GET(INTCR4_GMMAP, intcr);

		switch (gmmap) {
		case INTCR4_GMMAP_512MB:
			baseptr = ioremap_wc(ADDR_GMMAP_512MB, GMMAP_LENGTH);
			break;
		case INTCR4_GMMAP_512MB_ECC:
			baseptr = ioremap_wc(ADDR_GMMAP_512MB_ECC, GMMAP_LENGTH);
			break;
		case INTCR4_GMMAP_1GB:
			baseptr = ioremap_wc(ADDR_GMMAP_1GB, GMMAP_LENGTH);
			break;
		case INTCR4_GMMAP_1GB_ECC:
			baseptr = ioremap_wc(ADDR_GMMAP_1GB_ECC, GMMAP_LENGTH);
			break;
		case INTCR4_GMMAP_2GB:
			baseptr = ioremap_wc(ADDR_GMMAP_2GB, GMMAP_LENGTH);
			break;
		case INTCR4_GMMAP_2GB_ECC:
			baseptr = ioremap_wc(ADDR_GMMAP_2GB_ECC, GMMAP_LENGTH);
			break;
		}
	}

	memset(baseptr, 0, GMMAP_LENGTH);
	iounmap(baseptr);
}

static void npcm_video_get_resolution(struct npcm_video *video)
{
	struct v4l2_bt_timings *act = &video->active_timings;
	struct v4l2_bt_timings *det = &video->detected_timings;
	struct regmap *gfxi;
	u32 dispst;

	video->v4l2_input_status = 0;

	det->width = npcm_video_hres(video);
	det->height = npcm_video_vres(video);

	if (act->width != det->width || act->height != det->height) {
		dev_dbg(video->dev, "Resolution changed\n");

		npcm_video_bufs_done(video, VB2_BUF_STATE_ERROR);

		if (npcm_video_hres(video) > 0 && npcm_video_vres(video) > 0) {
			gfxi = video->gfx_regmap;

			if (test_bit(VIDEO_STREAMING, &video->flags)) {
				/*
				 * Wait for resolution is available,
				 * and it is also captured by host.
				 */
				do {
					mdelay(100);
					regmap_read(gfxi, DISPST, &dispst);
				} while (npcm_video_vres(video) < 100 ||
					 npcm_video_pclk(video) == 0 ||
					 (dispst & DISPST_HSCROFF));
			}

			det->width = npcm_video_hres(video);
			det->height = npcm_video_vres(video);
			det->pixelclock = npcm_video_pclk(video);
		}
	}

	if (det->width == 0 || det->height == 0) {
		det->width = MIN_WIDTH;
		det->height = MIN_HEIGHT;
		npcm_video_clear_gmmap(video);
		video->v4l2_input_status = V4L2_IN_ST_NO_SIGNAL;
	}

	dev_dbg(video->dev, "Got resolution[%dx%d] -> [%dx%d], status %d\n",
		act->width, act->height, det->width, det->height,
		video->v4l2_input_status);
}

static void npcm_video_set_resolution(struct npcm_video *video)
{
	struct v4l2_bt_timings *act = &video->active_timings;
	struct regmap *vcd = video->vcd_regmap;
	u32 mode;

	/* Set video frame physical address */
	regmap_write(vcd, VCD_FBA_ADR, video->src.dma);
	regmap_write(vcd, VCD_FBB_ADR, video->src.dma);

	npcm_video_capres(video, act->width, act->height);

	video->bytesperpixel = npcm_video_get_bpp(video);
	npcm_video_set_linepitch(video, act->width * video->bytesperpixel);

	video->bytesperline = npcm_video_get_linepitch(video);

	npcm_video_kvm_bw(video, act->pixelclock > VCD_KVM_BW_PCLK);

	npcm_video_gfx_reset(video);

	regmap_read(vcd, VCD_MODE, &mode);

	clear_bit(VIDEO_FRAME_INPRG, &video->flags);

	dev_dbg(video->dev, "VCD mode = 0x%x, %s mode\n", mode,
		npcm_video_is_mga(video) ? "Hi Res" : "VGA");

	dev_dbg(video->dev,
		"Digital mode: %d x %d x %d, pixelclock %lld, bytesperline %d\n",
		act->width, act->height, video->bytesperpixel, act->pixelclock,
		video->bytesperline);
}

static int npcm_video_start(struct npcm_video *video)
{
	int rc;

	rc = npcm_video_init_reg(video);
	if (rc)
		return rc;

	npcm_video_get_resolution(video);

	video->active_timings = video->detected_timings;

	video->max_buffer_size = VCD_MAX_SRC_BUFFER_SIZE;
	if (!npcm_video_alloc_buf(video, &video->src, video->max_buffer_size))
		return -ENOMEM;

	npcm_video_set_resolution(video);

	video->pix_fmt.width = video->active_timings.width;
	video->pix_fmt.height = video->active_timings.height;
	video->pix_fmt.sizeimage = video->max_buffer_size;
	video->pix_fmt.bytesperline = video->bytesperline;

	if (atomic_inc_return(&video->ece.clients) == 1) {
		npcm_video_ece_init(video);
		npcm_video_ece_set_fb_addr(video, video->src.dma);
		npcm_video_ece_set_lp(video, video->bytesperline);

		dev_dbg(video->dev, "ECE open: client %d\n",
			atomic_read(&video->ece.clients));
	}

	return 0;
}

static void npcm_video_stop(struct npcm_video *video)
{
	unsigned long flags;
	struct regmap *vcd = video->vcd_regmap;

	spin_lock_irqsave(&video->lock, flags);
	set_bit(VIDEO_STOPPED, &video->flags);
	spin_unlock_irqrestore(&video->lock, flags);

	regmap_write(vcd, VCD_INTE, 0);
	regmap_write(vcd, VCD_MODE, 0);
	regmap_write(vcd, VCD_RCHG, 0);
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);

	if (video->src.size)
		npcm_video_free_buf(video, &video->src);

	if (video->list)
		npcm_video_free_diff_table(video);

	kfree(video->list);
	video->list = NULL;

	kfree(video->rect);
	video->rect = NULL;

	video->v4l2_input_status = V4L2_IN_ST_NO_SIGNAL;
	video->flags = 0;
	video->ctrl_cmd = VCD_CMD_OPERATION_CAPTURE;

	if (atomic_dec_return(&video->ece.clients) == 0) {
		npcm_video_ece_stop(video);
		dev_dbg(video->dev, "ECE close: client %d\n",
			atomic_read(&video->ece.clients));
	}
}

static int npcm_video_querycap(struct file *file, void *fh,
			       struct v4l2_capability *cap)
{
	strscpy(cap->driver, DEVICE_NAME, sizeof(cap->driver));
	strscpy(cap->card, "NPCM Video Engine", sizeof(cap->card));
	snprintf(cap->bus_info, sizeof(cap->bus_info), "platform:%s",
		 DEVICE_NAME);

	return 0;
}

static int npcm_video_enum_format(struct file *file, void *fh,
				  struct v4l2_fmtdesc *f)
{
	if (f->index)
		return -EINVAL;

	f->pixelformat = V4L2_PIX_FMT_HEXTILE;

	return 0;
}

static int npcm_video_get_format(struct file *file, void *fh,
				 struct v4l2_format *f)
{
	struct npcm_video *video = video_drvdata(file);

	f->fmt.pix = video->pix_fmt;

	return 0;
}

static int npcm_video_enum_input(struct file *file, void *fh,
				 struct v4l2_input *inp)
{
	struct npcm_video *video = video_drvdata(file);

	if (inp->index)
		return -EINVAL;

	strscpy(inp->name, "Host VGA capture", sizeof(inp->name));
	inp->type = V4L2_INPUT_TYPE_CAMERA;
	inp->capabilities = V4L2_IN_CAP_DV_TIMINGS;
	inp->status = video->v4l2_input_status;

	return 0;
}

static int npcm_video_get_input(struct file *file, void *fh, unsigned int *i)
{
	*i = 0;

	return 0;
}

static int npcm_video_set_input(struct file *file, void *fh, unsigned int i)
{
	if (i)
		return -EINVAL;

	return 0;
}

static int npcm_video_get_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	struct npcm_video *video = video_drvdata(file);

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.readbuffers = 3;
	a->parm.capture.timeperframe.numerator = 1;
	if (!video->frame_rate)
		a->parm.capture.timeperframe.denominator = MAX_FRAME_RATE;
	else
		a->parm.capture.timeperframe.denominator = video->frame_rate;

	return 0;
}

static int npcm_video_set_parm(struct file *file, void *fh,
			       struct v4l2_streamparm *a)
{
	unsigned int frame_rate = 0;

	a->parm.capture.capability = V4L2_CAP_TIMEPERFRAME;
	a->parm.capture.readbuffers = 3;

	if (a->parm.capture.timeperframe.numerator)
		frame_rate = a->parm.capture.timeperframe.denominator /
			     a->parm.capture.timeperframe.numerator;

	if (!frame_rate || frame_rate > MAX_FRAME_RATE) {
		frame_rate = 0;
		a->parm.capture.timeperframe.denominator = MAX_FRAME_RATE;
		a->parm.capture.timeperframe.numerator = 1;
	}

	return 0;
}

static int npcm_video_enum_framesizes(struct file *file, void *fh,
				      struct v4l2_frmsizeenum *fsize)
{
	struct npcm_video *video = video_drvdata(file);

	if (fsize->index)
		return -EINVAL;

	if (fsize->pixel_format != V4L2_PIX_FMT_HEXTILE)
		return -EINVAL;

	fsize->discrete.width = video->pix_fmt.width;
	fsize->discrete.height = video->pix_fmt.height;
	fsize->type = V4L2_FRMSIZE_TYPE_DISCRETE;

	return 0;
}

static int npcm_video_enum_frameintervals(struct file *file, void *fh,
					  struct v4l2_frmivalenum *fival)
{
	struct npcm_video *video = video_drvdata(file);

	if (fival->index)
		return -EINVAL;

	if (fival->width != video->detected_timings.width ||
	    fival->height != video->detected_timings.height)
		return -EINVAL;

	if (fival->pixel_format != V4L2_PIX_FMT_HEXTILE)
		return -EINVAL;

	fival->type = V4L2_FRMIVAL_TYPE_CONTINUOUS;

	fival->stepwise.min.denominator = MAX_FRAME_RATE;
	fival->stepwise.min.numerator = 1;
	fival->stepwise.max.denominator = 1;
	fival->stepwise.max.numerator = 1;
	fival->stepwise.step = fival->stepwise.max;

	return 0;
}

static int npcm_video_set_dv_timings(struct file *file, void *fh,
				     struct v4l2_dv_timings *timings)
{
	struct npcm_video *video = video_drvdata(file);

	if (timings->bt.width == video->active_timings.width &&
	    timings->bt.height == video->active_timings.height)
		return 0;

	if (vb2_is_busy(&video->queue)) {
		dev_err(video->dev, "%s device busy\n", __func__);
		return -EBUSY;
	}

	video->active_timings = timings->bt;

	npcm_video_set_resolution(video);

	video->pix_fmt.width = timings->bt.width;
	video->pix_fmt.height = timings->bt.height;
	video->pix_fmt.sizeimage = video->max_buffer_size;
	video->pix_fmt.bytesperline = video->bytesperline;

	timings->type = V4L2_DV_BT_656_1120;

	return 0;
}

static int npcm_video_get_dv_timings(struct file *file, void *fh,
				     struct v4l2_dv_timings *timings)
{
	struct npcm_video *video = video_drvdata(file);

	timings->type = V4L2_DV_BT_656_1120;
	timings->bt = video->active_timings;

	return 0;
}

static int npcm_video_query_dv_timings(struct file *file, void *fh,
				       struct v4l2_dv_timings *timings)
{
	struct npcm_video *video = video_drvdata(file);

	npcm_video_get_resolution(video);

	timings->type = V4L2_DV_BT_656_1120;
	timings->bt = video->detected_timings;

	return video->v4l2_input_status ? -ENOLINK : 0;
}

static int npcm_video_enum_dv_timings(struct file *file, void *fh,
				      struct v4l2_enum_dv_timings *timings)
{
	return v4l2_enum_dv_timings_cap(timings, &npcm_video_timings_cap,
					NULL, NULL);
}

static int npcm_video_dv_timings_cap(struct file *file, void *fh,
				     struct v4l2_dv_timings_cap *cap)
{
	*cap = npcm_video_timings_cap;

	return 0;
}

static int npcm_video_sub_event(struct v4l2_fh *fh,
				const struct v4l2_event_subscription *sub)
{
	switch (sub->type) {
	case V4L2_EVENT_SOURCE_CHANGE:
		return v4l2_src_change_event_subscribe(fh, sub);
	}

	return v4l2_ctrl_subscribe_event(fh, sub);
}

static const struct v4l2_ioctl_ops npcm_video_ioctls = {
	.vidioc_querycap = npcm_video_querycap,

	.vidioc_enum_fmt_vid_cap = npcm_video_enum_format,
	.vidioc_g_fmt_vid_cap = npcm_video_get_format,
	.vidioc_s_fmt_vid_cap = npcm_video_get_format,
	.vidioc_try_fmt_vid_cap = npcm_video_get_format,

	.vidioc_reqbufs = vb2_ioctl_reqbufs,
	.vidioc_querybuf = vb2_ioctl_querybuf,
	.vidioc_qbuf = vb2_ioctl_qbuf,
	.vidioc_expbuf = vb2_ioctl_expbuf,
	.vidioc_dqbuf = vb2_ioctl_dqbuf,
	.vidioc_create_bufs = vb2_ioctl_create_bufs,
	.vidioc_prepare_buf = vb2_ioctl_prepare_buf,
	.vidioc_streamon = vb2_ioctl_streamon,
	.vidioc_streamoff = vb2_ioctl_streamoff,

	.vidioc_enum_input = npcm_video_enum_input,
	.vidioc_g_input = npcm_video_get_input,
	.vidioc_s_input = npcm_video_set_input,

	.vidioc_g_parm = npcm_video_get_parm,
	.vidioc_s_parm = npcm_video_set_parm,
	.vidioc_enum_framesizes = npcm_video_enum_framesizes,
	.vidioc_enum_frameintervals = npcm_video_enum_frameintervals,

	.vidioc_s_dv_timings = npcm_video_set_dv_timings,
	.vidioc_g_dv_timings = npcm_video_get_dv_timings,
	.vidioc_query_dv_timings = npcm_video_query_dv_timings,
	.vidioc_enum_dv_timings = npcm_video_enum_dv_timings,
	.vidioc_dv_timings_cap = npcm_video_dv_timings_cap,

	.vidioc_subscribe_event = npcm_video_sub_event,
	.vidioc_unsubscribe_event = v4l2_event_unsubscribe,
};

static int npcm_video_set_ctrl(struct v4l2_ctrl *ctrl)
{
	struct npcm_video *video = container_of(ctrl->handler,
						struct npcm_video,
						ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_NPCM_CAPTURE_MODE:
		if (ctrl->val == V4L2_NPCM_CAPTURE_MODE_COMPLETE)
			video->ctrl_cmd = VCD_CMD_OPERATION_CAPTURE;
		else if (ctrl->val == V4L2_NPCM_CAPTURE_MODE_DIFF)
			video->ctrl_cmd = VCD_CMD_OPERATION_COMPARE;
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static int npcm_video_get_volatile_ctrl(struct v4l2_ctrl *ctrl)
{
	struct npcm_video *video = container_of(ctrl->handler,
						struct npcm_video,
						ctrl_handler);

	switch (ctrl->id) {
	case V4L2_CID_NPCM_RECT_COUNT:
		ctrl->val = npcm_video_get_rect_count(video);
		break;
	default:
		return -EINVAL;
	}

	return 0;
}

static const struct v4l2_ctrl_ops npcm_video_ctrl_ops = {
	.s_ctrl = npcm_video_set_ctrl,
	.g_volatile_ctrl = npcm_video_get_volatile_ctrl,
};

static const struct v4l2_ctrl_config npcm_ctrl_capture_mode = {
	.ops = &npcm_video_ctrl_ops,
	.id = V4L2_CID_NPCM_CAPTURE_MODE,
	.name = "NPCM Video Capture Mode",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.min = 0,
	.max = V4L2_NPCM_CAPTURE_MODE_DIFF,
	.step = 1,
	.def = 0,
};

static const struct v4l2_ctrl_config npcm_ctrl_rect_count = {
	.ops = &npcm_video_ctrl_ops,
	.id = V4L2_CID_NPCM_RECT_COUNT,
	.name = "NPCM Compressed Hextile Rectangle Count",
	.type = V4L2_CTRL_TYPE_INTEGER,
	.flags = V4L2_CTRL_FLAG_VOLATILE,
	.min = 0,
	.max = (MAX_WIDTH / RECT_W) * (MAX_HEIGHT / RECT_H),
	.step = 1,
	.def = 0,
};

static int npcm_video_open(struct file *file)
{
	int rc;
	struct npcm_video *video = video_drvdata(file);

	mutex_lock(&video->video_lock);

	rc = v4l2_fh_open(file);
	if (rc) {
		mutex_unlock(&video->video_lock);
		return rc;
	}

	if (v4l2_fh_is_singular_file(file))
		npcm_video_start(video);

	mutex_unlock(&video->video_lock);

	return 0;
}

static int npcm_video_release(struct file *file)
{
	int rc;
	struct npcm_video *video = video_drvdata(file);

	mutex_lock(&video->video_lock);

	if (v4l2_fh_is_singular_file(file))
		npcm_video_stop(video);

	rc = _vb2_fop_release(file, NULL);

	mutex_unlock(&video->video_lock);

	return rc;
}

static const struct v4l2_file_operations npcm_video_v4l2_fops = {
	.owner = THIS_MODULE,
	.read = vb2_fop_read,
	.poll = vb2_fop_poll,
	.unlocked_ioctl = video_ioctl2,
	.mmap = vb2_fop_mmap,
	.open = npcm_video_open,
	.release = npcm_video_release,
};

static int npcm_video_queue_setup(struct vb2_queue *q,
				  unsigned int *num_buffers,
				  unsigned int *num_planes,
				  unsigned int sizes[],
				  struct device *alloc_devs[])
{
	struct npcm_video *video = vb2_get_drv_priv(q);
	unsigned int i;

	if (*num_planes) {
		if (sizes[0] < video->max_buffer_size)
			return -EINVAL;

		return 0;
	}

	*num_planes = 1;
	sizes[0] = video->max_buffer_size;

	kfree(video->rect);
	video->rect = NULL;

	video->rect = kcalloc(*num_buffers, sizeof(*video->rect), GFP_KERNEL);

	if (video->list) {
		npcm_video_free_diff_table(video);
		kfree(video->list);
		video->list = NULL;
	}

	video->list = kzalloc(sizeof(*video->list) * *num_buffers, GFP_KERNEL);

	for (i = 0; i < *num_buffers; i++)
		INIT_LIST_HEAD(&video->list[i]);

	video->num_buffers = *num_buffers;

	return 0;
}

static int npcm_video_buf_prepare(struct vb2_buffer *vb)
{
	struct npcm_video *video = vb2_get_drv_priv(vb->vb2_queue);

	if (vb2_plane_size(vb, 0) < video->max_buffer_size)
		return -EINVAL;

	return 0;
}

static int npcm_video_start_streaming(struct vb2_queue *q, unsigned int count)
{
	int rc;
	struct npcm_video *video = vb2_get_drv_priv(q);

	video->sequence = 0;

	rc = npcm_video_start_frame(video);
	if (rc) {
		npcm_video_bufs_done(video, VB2_BUF_STATE_QUEUED);
		return rc;
	}

	set_bit(VIDEO_STREAMING, &video->flags);
	return 0;
}

static void npcm_video_stop_streaming(struct vb2_queue *q)
{
	struct npcm_video *video = vb2_get_drv_priv(q);
	struct regmap *vcd = video->vcd_regmap;

	clear_bit(VIDEO_STREAMING, &video->flags);

	regmap_write(vcd, VCD_INTE, 0);
	regmap_write(vcd, VCD_STAT, VCD_STAT_CLEAR);
	npcm_video_gfx_reset(video);

	npcm_video_bufs_done(video, VB2_BUF_STATE_ERROR);

	video->ctrl_cmd = VCD_CMD_OPERATION_CAPTURE;
}

static void npcm_video_buf_queue(struct vb2_buffer *vb)
{
	struct npcm_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct vb2_v4l2_buffer *vbuf = to_vb2_v4l2_buffer(vb);
	struct npcm_video_buffer *nvb = to_npcm_video_buffer(vbuf);
	unsigned long flags;

	spin_lock_irqsave(&video->lock, flags);
	list_add_tail(&nvb->link, &video->buffers);
	spin_unlock_irqrestore(&video->lock, flags);
}

static void npcm_video_buf_finish(struct vb2_buffer *vb)
{
	struct npcm_video *video = vb2_get_drv_priv(vb->vb2_queue);
	struct regmap *vcd = video->vcd_regmap;
	u32 val;
	int ret;

	ret = regmap_read_poll_timeout(vcd, VCD_STAT, val,
				       !(val & VCD_STAT_BUSY), 1000,
				       VCD_BUSY_TIMEOUT_US);

	if (ret) {
		dev_warn(video->dev, "Wait for VCD_STAT_BUSY timeout\n");
		return;
	}

	/* Capture next frame when a video buffer is dequeued */
	npcm_video_start_frame(video);

	video->vb_index = vb->index;
}

static const struct vb2_ops npcm_video_vb2_ops = {
	.queue_setup = npcm_video_queue_setup,
	.wait_prepare = vb2_ops_wait_prepare,
	.wait_finish = vb2_ops_wait_finish,
	.buf_prepare = npcm_video_buf_prepare,
	.buf_finish = npcm_video_buf_finish,
	.start_streaming = npcm_video_start_streaming,
	.stop_streaming = npcm_video_stop_streaming,
	.buf_queue =  npcm_video_buf_queue,
};

static int npcm_video_setup_video(struct npcm_video *video)
{
	struct v4l2_device *v4l2_dev = &video->v4l2_dev;
	struct video_device *vdev = &video->vdev;
	struct vb2_queue *vbq = &video->queue;
	int rc;

	video->pix_fmt.pixelformat = V4L2_PIX_FMT_HEXTILE;
	video->pix_fmt.field = V4L2_FIELD_NONE;
	video->pix_fmt.colorspace = V4L2_COLORSPACE_SRGB;
	video->v4l2_input_status = V4L2_IN_ST_NO_SIGNAL;

	rc = v4l2_device_register(video->dev, v4l2_dev);
	if (rc) {
		dev_err(video->dev, "Failed to register v4l2 device\n");
		return rc;
	}

	v4l2_ctrl_handler_init(&video->ctrl_handler, 2);
	v4l2_ctrl_new_custom(&video->ctrl_handler, &npcm_ctrl_capture_mode,
			     NULL);
	v4l2_ctrl_new_custom(&video->ctrl_handler, &npcm_ctrl_rect_count, NULL);

	if (video->ctrl_handler.error) {
		dev_err(video->dev, "Failed to init controls: %d\n",
			video->ctrl_handler.error);

		rc = video->ctrl_handler.error;

		v4l2_ctrl_handler_free(&video->ctrl_handler);
		v4l2_device_unregister(v4l2_dev);

		return rc;
	}

	v4l2_dev->ctrl_handler = &video->ctrl_handler;

	vbq->type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
	vbq->io_modes = VB2_MMAP | VB2_READ | VB2_DMABUF;
	vbq->dev = v4l2_dev->dev;
	vbq->lock = &video->video_lock;
	vbq->ops = &npcm_video_vb2_ops;
	vbq->mem_ops = &vb2_dma_contig_memops;
	vbq->drv_priv = video;
	vbq->buf_struct_size = sizeof(struct npcm_video_buffer);
	vbq->timestamp_flags = V4L2_BUF_FLAG_TIMESTAMP_MONOTONIC;
	vbq->min_buffers_needed = 3;

	rc = vb2_queue_init(vbq);
	if (rc) {
		v4l2_ctrl_handler_free(&video->ctrl_handler);
		v4l2_device_unregister(v4l2_dev);

		dev_err(video->dev, "Failed to init vb2 queue\n");
		return rc;
	}

	vdev->queue = vbq;
	vdev->fops = &npcm_video_v4l2_fops;
	vdev->device_caps = V4L2_CAP_VIDEO_CAPTURE | V4L2_CAP_READWRITE |
			    V4L2_CAP_STREAMING;
	vdev->v4l2_dev = v4l2_dev;
	strscpy(vdev->name, DEVICE_NAME, sizeof(vdev->name));
	vdev->vfl_type = VFL_TYPE_VIDEO;
	vdev->vfl_dir = VFL_DIR_RX;
	vdev->release = video_device_release_empty;
	vdev->ioctl_ops = &npcm_video_ioctls;
	vdev->lock = &video->video_lock;

	video_set_drvdata(vdev, video);
	rc = video_register_device(vdev, VFL_TYPE_VIDEO, 0);
	if (rc) {
		vb2_queue_release(vbq);
		v4l2_ctrl_handler_free(&video->ctrl_handler);
		v4l2_device_unregister(v4l2_dev);

		dev_err(video->dev, "Failed to register video device\n");
		return rc;
	}

	return 0;
}

static int npcm_video_init(struct npcm_video *video)
{
	int irq;
	int rc;
	struct device *dev = video->dev;

	irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!irq) {
		dev_err(dev, "Unable to find VCD IRQ\n");
		return -ENODEV;
	}

	rc = devm_request_threaded_irq(dev, irq, NULL, npcm_video_irq,
				       IRQF_ONESHOT, DEVICE_NAME, video);
	if (rc < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", irq);
		return rc;
	}

	of_reserved_mem_device_init(dev);

	rc = dma_set_mask_and_coherent(dev, DMA_BIT_MASK(32));
	if (rc) {
		dev_err(dev, "Failed to set DMA mask\n");
		of_reserved_mem_device_release(dev);
	}

	return rc;
}

static const struct regmap_config npcm_video_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= VCD_FIFO,
};

static const struct regmap_config npcm_video_ece_regmap_cfg = {
	.reg_bits	= 32,
	.reg_stride	= 4,
	.val_bits	= 32,
	.max_register	= ECE_HEX_RECT_OFFSET,
};

static int npcm_video_probe(struct platform_device *pdev)
{
	int rc;
	void __iomem *regs;
	struct npcm_video *video = kzalloc(sizeof(*video), GFP_KERNEL);

	if (!video)
		return -ENOMEM;

	video->frame_rate = MAX_FRAME_RATE;
	video->dev = &pdev->dev;
	spin_lock_init(&video->lock);
	mutex_init(&video->video_lock);
	INIT_LIST_HEAD(&video->buffers);

	regs = devm_platform_ioremap_resource_byname(pdev, VCD_MODULE_NAME);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "Failed to find VCD reg-names in DTS!\n");
		return PTR_ERR(regs);
	}

	video->vcd_regmap = devm_regmap_init_mmio(&pdev->dev, regs,
						  &npcm_video_regmap_cfg);
	if (IS_ERR(video->vcd_regmap)) {
		dev_err(&pdev->dev, "Failed to initialize VCD regmap!\n");
		return PTR_ERR(video->vcd_regmap);
	}

	regs = devm_platform_ioremap_resource_byname(pdev, ECE_MODULE_NAME);
	if (IS_ERR(regs)) {
		dev_err(&pdev->dev, "Failed to find ECE reg-names in DTS!\n");
		return PTR_ERR(regs);
	}

	video->ece.regmap = devm_regmap_init_mmio(&pdev->dev, regs,
						  &npcm_video_ece_regmap_cfg);
	if (IS_ERR(video->ece.regmap)) {
		dev_err(&pdev->dev, "Failed to initialize ECE regmap!\n");
		return PTR_ERR(video->ece.regmap);
	}

	video->reset = devm_reset_control_get(&pdev->dev, VCD_MODULE_NAME);
	if (IS_ERR(video->reset)) {
		dev_err(&pdev->dev, "Failed to find VCD reset-names in DTS!\n");
		return PTR_ERR(video->reset);
	}

	video->ece.reset = devm_reset_control_get(&pdev->dev, ECE_MODULE_NAME);
	if (IS_ERR(video->ece.reset)) {
		dev_err(&pdev->dev, "Failed to find ECE reset-names in DTS!\n");
		return PTR_ERR(video->ece.reset);
	}

	video->gcr_regmap =
		syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "gcr-syscon");
	if (IS_ERR(video->gcr_regmap))
		return PTR_ERR(video->gcr_regmap);

	video->gfx_regmap =
		syscon_regmap_lookup_by_phandle(pdev->dev.of_node, "gfxi-syscon");
	if (IS_ERR(video->gfx_regmap))
		return PTR_ERR(video->gfx_regmap);

	rc = npcm_video_init(video);
	if (rc)
		return rc;

	rc = npcm_video_setup_video(video);
	if (rc)
		return rc;

	dev_info(video->dev, "NPCM video driver probed\n");

	return 0;
}

static int npcm_video_remove(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct v4l2_device *v4l2_dev = dev_get_drvdata(dev);
	struct npcm_video *video = to_npcm_video(v4l2_dev);

	video_unregister_device(&video->vdev);

	vb2_queue_release(&video->queue);

	v4l2_ctrl_handler_free(&video->ctrl_handler);

	v4l2_device_unregister(v4l2_dev);

	npcm_video_ece_stop(video);

	of_reserved_mem_device_release(dev);

	return 0;
}

static const struct of_device_id npcm_video_match[] = {
	{ .compatible = "nuvoton,npcm750-video" },
	{ .compatible = "nuvoton,npcm845-video" },
	{},
};

MODULE_DEVICE_TABLE(of, npcm_video_match);

static struct platform_driver npcm_video_driver = {
	.driver = {
		.name = DEVICE_NAME,
		.of_match_table = npcm_video_match,
	},
	.probe = npcm_video_probe,
	.remove = npcm_video_remove,
};

module_platform_driver(npcm_video_driver);

MODULE_AUTHOR("Joseph Liu<kwliu@nuvoton.com>");
MODULE_AUTHOR("Marvin Lin<kflin@nuvoton.com>");
MODULE_DESCRIPTION("Driver for Nuvoton NPCM Video Capture/Encode Engine");
MODULE_LICENSE("GPL");
