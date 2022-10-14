/*
 * Copyright (c) 2018 Nuvoton Technology corporation.
 *
 * Released under the GPLv2 only.
 * SPDX-License-Identifier: GPL-2.0
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/fs.h>
#include <linux/cdev.h>
#include <linux/slab.h>
#include <linux/device.h>
#include <linux/platform_device.h>
#include <linux/unistd.h>
#include <linux/sched.h>
#include <linux/file.h>
#include <linux/mm.h>
#include <linux/of_platform.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/interrupt.h>
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/fb.h>
#include <linux/regmap.h>
#include <linux/miscdevice.h>
#include <linux/reset.h>

#define ECE_VERSION "1.0.0"

/* ECE Register */
#define DDA_CTRL	0x0000
#define  DDA_CTRL_ECEEN BIT(0)
#define  DDA_CTRL_INTEN BIT(8)
#define  DDA_CTRL_FIFO_NF_IE BIT(9)
#define  DDA_CTRL_ACDRDY_IE BIT(10)

#define DDA_STS	0x0004
#define  DDA_STS_FIFOSTSI GENMASK(2, 0)
#define  DDA_STS_FIFOSTSE GENMASK(6, 4)
#define  DDA_STS_CDREADY BIT(8)
#define  DDA_STS_FIFO_NF BIT(9)
#define  DDA_STS_ACDRDY BIT(10)

#define FBR_BA	0x0008
#define ED_BA	0x000C
#define RECT_XY	0x0010

#define RECT_DIMEN	0x0014
#define	 RECT_DIMEN_HLTR_OFFSET	27
#define	 RECT_DIMEN_HR_OFFSET	16
#define	 RECT_DIMEN_WLTR_OFFSET	11
#define	 RECT_DIMEN_WR_OFFSET	0

#define RESOL	0x001C
#define  RESOL_FB_LP_512	0
#define  RESOL_FB_LP_1024	1
#define  RESOL_FB_LP_2048	2
#define  RESOL_FB_LP_2560	3
#define  RESOL_FB_LP_4096	4

#define HEX_CTRL	0x0040
#define  HEX_CTRL_ENCDIS BIT(0)
#define  HEX_CTRL_ENC_GAP 0x1f00
#define  HEX_CTRL_ENC_GAP_OFFSET 8
#define  HEX_CTRL_ENC_MIN_GAP_SIZE 4

#define HEX_RECT_OFFSET 0x0048

#define DEFAULT_WIDTH 640
#define DEFAULT_HEIGHT 640
#define DEFAULT_LP 2048

#define ECE_MIN_LP	512
#define ECE_MAX_LP	4096
#define ECE_TILE_W	16
#define ECE_TILE_H	16

#define ECE_IOC_MAGIC 'k'
#define ECE_IOCGETED _IOR(ECE_IOC_MAGIC, 1, struct ece_ioctl_cmd)
#define ECE_IOCSETFB _IOW(ECE_IOC_MAGIC, 2, struct ece_ioctl_cmd)
#define ECE_IOCSETLP _IOW(ECE_IOC_MAGIC, 3, struct ece_ioctl_cmd)
#define ECE_IOCGET_OFFSET _IOR(ECE_IOC_MAGIC, 4, u32)
#define ECE_IOCCLEAR_OFFSET _IO(ECE_IOC_MAGIC, 5)
#define ECE_IOCENCADDR_RESET _IO(ECE_IOC_MAGIC, 6)
#define ECE_RESET _IO(ECE_IOC_MAGIC, 7)
#define ECE_IOC_MAXNR 7

#define ECE_OP_TIMEOUT msecs_to_jiffies(100)

#define DEVICE_NAME "nuvoton-ece"

struct ece_ioctl_cmd {
	u32 framebuf;
	u32 gap_len;
	u8 *buf;
	u32 len;
	u32 x;
	u32 y;
	u32 w;
	u32 h;
	u32 lp;
};

struct npcm750_ece {
	void *virt;
	struct regmap *ece_regmap;
	struct mutex mlock; /* for ioctl*/
	spinlock_t lock;	/*for irq*/
	struct device *dev;
	struct miscdevice miscdev;
	struct cdev dev_cdev;
	resource_size_t size;
	resource_size_t dma;
	u32 line_pitch;
	u32 enc_gap;
	u32 status;
	atomic_t clients;
	int irq;
	struct completion complete;
	struct reset_control *reset;
};

static void npcm750_ece_ip_reset(struct npcm750_ece *priv)
{
	reset_control_assert(priv->reset);
	msleep(100);
	reset_control_deassert(priv->reset);
	msleep(100);
}

/* Clear Offset of Compressed Rectangle*/
static void npcm750_ece_clear_rect_offset(struct npcm750_ece *priv)
{
	struct regmap *ece = priv->ece_regmap;

	regmap_write(ece, HEX_RECT_OFFSET, 0);
}

/* Read Offset of Compressed Rectangle*/
static u32 npcm750_ece_read_rect_offset(struct npcm750_ece *priv)
{
	struct regmap *ece = priv->ece_regmap;
	u32 offset;

	regmap_read(ece, HEX_RECT_OFFSET, &offset);
	return offset & 0x003fffff;
}

/* Return data if a rectangle finished to be compressed */
static u32 npcm750_ece_get_ed_size(struct npcm750_ece *priv, u32 offset)
{
	struct regmap *ece = priv->ece_regmap;
	u32 size, gap;
	int timeout;
	void *buffer = priv->virt + offset;

	timeout = wait_for_completion_interruptible_timeout(&priv->complete,
		ECE_OP_TIMEOUT);
	if (!timeout || !(priv->status & DDA_STS_CDREADY)) {
		dev_dbg(priv->dev, "ece compress timeout\n");
		return 0;
	}

	size = (u32)readl(buffer);

	regmap_read(ece, HEX_CTRL, &gap);
	priv->enc_gap = (gap & HEX_CTRL_ENC_GAP) >> HEX_CTRL_ENC_GAP_OFFSET;
	if (priv->enc_gap == 0)
		priv->enc_gap = HEX_CTRL_ENC_MIN_GAP_SIZE;

	return size;
}

/* This routine reset the FIFO as a bypass for Z1 chip */
static void npcm750_ece_fifo_reset_bypass(struct npcm750_ece *priv)
{
	struct regmap *ece = priv->ece_regmap;
	regmap_update_bits(ece, DDA_CTRL, DDA_CTRL_ECEEN, (u32)~DDA_CTRL_ECEEN);
	regmap_update_bits(ece, DDA_CTRL, DDA_CTRL_ECEEN, DDA_CTRL_ECEEN);
}

/* This routine Encode the desired rectangle */
static void npcm750_ece_enc_rect(struct npcm750_ece *priv,
				 u32 r_off_x, u32 r_off_y, u32 r_w, u32 r_h)
{
	struct regmap *ece = priv->ece_regmap;
	u32 rect_offset =
		(r_off_y * priv->line_pitch) + (r_off_x * 2);
	u32 temp;
	u32 w_tile;
	u32 h_tile;
	u32 w_size = ECE_TILE_W;
	u32 h_size = ECE_TILE_H;

	npcm750_ece_fifo_reset_bypass(priv);

	regmap_write(ece, RECT_XY, rect_offset);

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

	temp = ((w_size - 1) << RECT_DIMEN_WLTR_OFFSET)
		| ((h_size - 1) << RECT_DIMEN_HLTR_OFFSET)
		| ((w_tile - 1) << RECT_DIMEN_WR_OFFSET)
		| ((h_tile - 1) << RECT_DIMEN_HR_OFFSET);

	regmap_write(ece, RECT_DIMEN, temp);
}

/* This routine sets the Encoded Data base address */
static void npcm750_ece_set_enc_dba(struct npcm750_ece *priv, u32 addr)
{
	struct regmap *ece = priv->ece_regmap;

	regmap_write(ece, ED_BA, addr);
}

/* This routine sets the Frame Buffer base address */
static void npcm750_ece_set_fb_addr(struct npcm750_ece *priv, u32 buffer)
{
	struct regmap *ece = priv->ece_regmap;

	regmap_write(ece, FBR_BA, buffer);
}

/* Set the line pitch (in bytes) for the frame buffers. */
/* Can be on of those values: 512, 1024, 2048, 2560 or 4096 bytes */
static void npcm750_ece_set_lp(struct npcm750_ece *priv, u32 pitch)
{
	u32 lp;
	struct regmap *ece = priv->ece_regmap;

	switch (pitch) {
	case 512:
		lp = RESOL_FB_LP_512;
		break;
	case 1024:
		lp = RESOL_FB_LP_1024;
		break;
	case 2048:
		lp = RESOL_FB_LP_2048;
		break;
	case 2560:
		lp = RESOL_FB_LP_2560;
		break;
	case 4096:
		lp = RESOL_FB_LP_4096;
		break;
	default:
		return;
	}

	priv->line_pitch = pitch;
	regmap_write(ece, RESOL, lp);
}

/* Stop and reset the ECE state machine */
static void npcm750_ece_reset(struct npcm750_ece *priv)
{
	struct regmap *ece = priv->ece_regmap;

	regmap_update_bits(ece,
				DDA_CTRL, DDA_CTRL_ECEEN, (u32)~DDA_CTRL_ECEEN);
	regmap_update_bits(ece,
				HEX_CTRL, HEX_CTRL_ENCDIS, HEX_CTRL_ENCDIS);
	regmap_update_bits(ece,
				DDA_CTRL, DDA_CTRL_ECEEN, DDA_CTRL_ECEEN);
	regmap_update_bits(ece,
				HEX_CTRL, HEX_CTRL_ENCDIS, (u32)~HEX_CTRL_ENCDIS);

	npcm750_ece_clear_rect_offset(priv);
}

/* Initialise the ECE block and interface library */
static int npcm750_ece_init(struct npcm750_ece *priv)
{
	npcm750_ece_ip_reset(priv);

	npcm750_ece_reset(priv);

	npcm750_ece_set_enc_dba(priv, priv->dma);

	priv->line_pitch = DEFAULT_LP;

	return 0;
}

/* Disable the ECE block*/
static int npcm750_ece_stop(struct npcm750_ece *priv)
{
	struct regmap *ece = priv->ece_regmap;

	regmap_update_bits(ece,
				DDA_CTRL, DDA_CTRL_ECEEN, (u32)~DDA_CTRL_ECEEN);
	regmap_update_bits(ece,
				DDA_CTRL, DDA_CTRL_INTEN, (u32)~DDA_CTRL_INTEN);
	regmap_update_bits(ece,
				HEX_CTRL, HEX_CTRL_ENCDIS, HEX_CTRL_ENCDIS);
	npcm750_ece_clear_rect_offset(priv);

	return 0;
}

static int
npcm750_ece_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct npcm750_ece *priv = file->private_data;

	if (!priv)
		return -ENODEV;

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	fb_pgprotect(file, vma, priv->dma);
	return vm_iomap_memory(vma, priv->dma, priv->size);
}

static int npcm750_ece_open(struct inode *inode, struct file *file)
{
	struct npcm750_ece *priv =
		container_of(file->private_data, struct npcm750_ece, miscdev);

	if (!priv)
		return -ENODEV;

	file->private_data = priv;

	if (atomic_inc_return(&priv->clients) == 1)
		npcm750_ece_init(priv);

	dev_dbg(priv->dev, "open: client %d\n", atomic_read(&priv->clients));

	return 0;
}

static int npcm750_ece_close(struct inode *inode, struct file *file)
{
	struct npcm750_ece *priv = file->private_data;

	if (atomic_dec_return(&priv->clients) == 0)
		npcm750_ece_stop(priv);

	dev_dbg(priv->dev, "close: client %d\n", atomic_read(&priv->clients));
	return 0;
}

long npcm750_ece_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int err = 0;
	struct npcm750_ece *priv = filp->private_data;
	struct regmap *ece = priv->ece_regmap;

	mutex_lock(&priv->mlock);

	switch (cmd) {
	case ECE_IOCCLEAR_OFFSET:
		npcm750_ece_clear_rect_offset(priv);

		break;
	case ECE_IOCGET_OFFSET:
	{
		u32 offset = npcm750_ece_read_rect_offset(priv);

		err = copy_to_user((int __user *)args, &offset, sizeof(offset))
			? -EFAULT : 0;
		break;
	}
	case ECE_IOCSETLP:
	{
		struct ece_ioctl_cmd data;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err)
			break;

		if (!(data.lp % ECE_MIN_LP) && data.lp <= ECE_MAX_LP)
			npcm750_ece_set_lp(priv, data.lp);

		break;
	}
	case ECE_IOCSETFB:
	{
		struct ece_ioctl_cmd data;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err)
			break;

		if (!data.framebuf) {
			err = -EFAULT;
			break;
		}

		npcm750_ece_set_fb_addr(priv, data.framebuf);
		break;
	}
	case ECE_IOCGETED:
	{
		struct ece_ioctl_cmd data;
		u32 ed_size = 0;
		u32 offset = 0;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err)
			break;

		offset = npcm750_ece_read_rect_offset(priv);

		if ((offset + (data.w * data.h * 2) + 12) >= priv->size) {
			err = -EFAULT;
			dev_dbg(priv->dev, "ece may reach beyond memory region\n");
			break;
		}

		reinit_completion(&priv->complete);

		regmap_write(ece, DDA_STS,
			DDA_STS_CDREADY | DDA_STS_ACDRDY);

		regmap_update_bits(ece,
			DDA_CTRL, DDA_CTRL_INTEN, DDA_CTRL_INTEN);

		npcm750_ece_enc_rect(priv, data.x, data.y, data.w, data.h);

		ed_size = npcm750_ece_get_ed_size(priv, offset);

		regmap_update_bits(ece,
			DDA_CTRL, DDA_CTRL_INTEN, (u32)~DDA_CTRL_INTEN);

		if (ed_size == 0) {
			err = -EFAULT;
			break;
		}

		data.gap_len = priv->enc_gap;
		data.len = ed_size;
		err = copy_to_user((int __user *)args, &data, sizeof(data))
			? -EFAULT : 0;
		break;
	}
	case ECE_IOCENCADDR_RESET:
	{
		npcm750_ece_clear_rect_offset(priv);
		npcm750_ece_set_enc_dba(priv, priv->dma);
		priv->line_pitch = DEFAULT_LP;
		break;
	}
	case ECE_RESET:
	{
		u32 gap;

		npcm750_ece_reset(priv);
		npcm750_ece_set_enc_dba(priv, priv->dma);

		regmap_read(ece, HEX_CTRL, &gap);
		priv->enc_gap = (gap & HEX_CTRL_ENC_GAP) >> HEX_CTRL_ENC_GAP_OFFSET;
		if (priv->enc_gap == 0)
			priv->enc_gap = HEX_CTRL_ENC_MIN_GAP_SIZE;

		break;
	}
	default:
		break;
	}

	mutex_unlock(&priv->mlock);

	return err;
}

static irqreturn_t npcm750_ece_irq_handler(int irq, void *arg)
{
	struct npcm750_ece *priv = arg;
	struct regmap *ece = priv->ece_regmap;
	u32 status_ack = 0;
	u32 status;

	spin_lock(&priv->lock);

	regmap_read(ece, DDA_STS, &status);

	priv->status = status;

	if (status & DDA_STS_CDREADY) {
		dev_dbg(priv->dev, "DDA_STS_CDREADY\n");
		status_ack |= DDA_STS_CDREADY;
	}

	if (status & DDA_STS_ACDRDY) {
		dev_dbg(priv->dev, "DDA_STS_ACDRDY\n");
		status_ack |= DDA_STS_ACDRDY;
	}

	regmap_write(ece, DDA_STS, status_ack);

	spin_unlock(&priv->lock);

	complete(&priv->complete);

	return IRQ_HANDLED;
}

struct file_operations const npcm750_ece_fops = {
	.unlocked_ioctl = npcm750_ece_ioctl,
	.open = npcm750_ece_open,
	.release = npcm750_ece_close,
	.mmap = npcm750_ece_mmap,
};

static int npcm750_ece_device_create(struct npcm750_ece *priv)
{
	int ret = 0;
	struct resource res;
	struct device *dev = priv->dev;
	struct device_node *node;

	/* optional. */
	node = of_parse_phandle(dev->of_node, "memory-region", 0);
	if (node) {
		ret = of_address_to_resource(node, 0, &res);
		of_node_put(node);
		if (ret) {
			dev_err(dev, "Couldn't address to resource for reserved memory\n");
			return -ENODEV;
		}

		priv->size = (u32)resource_size(&res);
		priv->dma = (u32)res.start;
	} else {
		dev_err(dev, "Cannnot find memory-region\n");
		return -ENODEV;
	}

	dev_info(dev, "Reserved ECE memory start 0x%x size 0x%x\n", priv->dma, priv->size);

	if (!devm_request_mem_region(dev, priv->dma, priv->size, "ece_ram")) {
		dev_err(dev, "can't reserve ece ram\n");
		return -ENXIO;
	}

	priv->virt = devm_memremap(dev, priv->dma,
					priv->size,
					MEMREMAP_WC);
	if (!priv->virt) {
		dev_err(dev, "%s: cannot map ece memory region\n",
			 __func__);
		ret = -EIO;
		goto err;
	}

	priv->irq = irq_of_parse_and_map(dev->of_node, 0);
	if (!priv->irq) {
		dev_err(dev, "Unable to find ECE IRQ\n");
		ret = -ENODEV;
		goto err;
	}

	ret = devm_request_threaded_irq(dev, priv->irq, NULL, npcm750_ece_irq_handler,
				IRQF_ONESHOT, DEVICE_NAME, priv);
	if (ret < 0) {
		dev_err(dev, "Unable to request IRQ %d\n", priv->irq);
		goto err;
	}

	priv->miscdev.parent = dev;
	priv->miscdev.fops =  &npcm750_ece_fops;
	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name =
		devm_kasprintf(dev, GFP_KERNEL, "%s", "hextile");
	if (!priv->miscdev.name) {
		ret = -ENOMEM;
		goto err;
	}

	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(dev,
			"Unable to register device, err %d\n", ret);
		kfree(priv->miscdev.name);
		goto err;
	}

	return 0;

err:
	return ret;
}

static const struct regmap_config npcm750_ece_regmap_cfg = {
	.reg_bits       = 32,
	.reg_stride     = 4,
	.val_bits       = 32,
	.max_register   = HEX_RECT_OFFSET,
};

static int npcm750_ece_probe(struct platform_device *pdev)
{
	int ret = 0;
	void __iomem *regs;
	struct npcm750_ece *priv = NULL;
	struct device *dev = &pdev->dev;

	priv = devm_kzalloc(dev, sizeof(*priv), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = dev;

	regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(regs)) {
		dev_err(dev, "Failed to get regmap!\n");
		ret = PTR_ERR(regs);
		goto err;
	}

	priv->ece_regmap = devm_regmap_init_mmio(dev, regs,
						&npcm750_ece_regmap_cfg);
	if (IS_ERR(priv->ece_regmap)) {
		dev_err(dev, "Failed to init regmap!\n");
		ret = PTR_ERR(priv->ece_regmap);
		goto err;
	}

	priv->reset = devm_reset_control_get(&pdev->dev, NULL);
	if (IS_ERR(priv->reset)) {
		ret = PTR_ERR(priv->reset);
		goto err;
	}

	ret = npcm750_ece_device_create(priv);
	if (ret) {
		dev_err(dev, "%s: failed to create device\n",
			__func__);
		goto err;
	}

	mutex_init(&priv->mlock);
	spin_lock_init(&priv->lock);
	init_completion(&priv->complete);

	dev_info(dev, "NPCM ECE Driver probed %s\n", ECE_VERSION);
	return 0;

err:
	devm_kfree(dev, priv);
	return ret;
}

static int npcm750_ece_remove(struct platform_device *pdev)
{
	struct npcm750_ece *priv = platform_get_drvdata(pdev);
	struct device *dev = priv->dev;

	npcm750_ece_stop(priv);

	devm_memunmap(dev, priv->virt);

	misc_deregister(&priv->miscdev);

	kfree(priv->miscdev.name);

	mutex_destroy(&priv->mlock);

	kfree(priv);

	return 0;
}

static const struct of_device_id npcm750_ece_of_match_table[] = {
	{ .compatible = "nuvoton,npcm750-ece"},
	{ .compatible = "nuvoton,npcm845-ece"},
	{}
};
MODULE_DEVICE_TABLE(of, npcm750_ece_of_match_table);

static struct platform_driver npcm750_ece_driver = {
	.driver		= {
		.name	= DEVICE_NAME,
		.of_match_table = npcm750_ece_of_match_table,
	},
	.probe		= npcm750_ece_probe,
	.remove		= npcm750_ece_remove,
};

module_platform_driver(npcm750_ece_driver);
MODULE_DESCRIPTION("Nuvoton NPCM ECE Driver");
MODULE_AUTHOR("KW Liu <kwliu@nuvoton.com>");
MODULE_LICENSE("GPL v2");
