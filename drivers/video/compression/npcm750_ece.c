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
#include <linux/delay.h>
#include <linux/dma-mapping.h>
#include <asm/fb.h>

/* ECE registers*/
#define DDA_CTRL        0x0000
#define DDA_STS         0x0004
#define FBR_BA          0x0008
#define ED_BA           0x000C
#define RECT_XY         0x0010
#define RECT_DIMEN      0x0014
#define RESOL           0x001C
#define HEX_CTRL        0x0040
#define HEX_RECT_OFFSET 0x0048

#define FIFOSTSI 0x003
#define FIFOSTSE 0x030
#define ACDREADY BIT(10)
#define CDREADY BIT(8)
#define ENC_GAP 0x1f00
#define ENC_GAP_OFFSET 8
#define ENC_MIN_GAP_SIZE 4

#define	ECE_RECT_DIMEN_HLTR_OFFSET	27
#define	ECE_RECT_DIMEN_HR_OFFSET	16
#define	ECE_RECT_DIMEN_WLTR_OFFSET	11
#define	ECE_RECT_DIMEN_WR_OFFSET	0

#define ECEEN BIT(0)
#define ENCDIS BIT(0)

/* ECE Line pitch*/
#define LP_512	0
#define LP_1024	1
#define LP_2048	2
#define LP_2560	3
#define LP_4096 4

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
#define ECE_IOC_MAXNR 5

struct ece_ioctl_cmd {
	unsigned int framebuf;
	unsigned int gap_len;
	char *buf;
	int len;
	int x;
	int y;
	int w;
	int h;
	int lp;
};

struct npcm750_ece {
	struct mutex mlock; /* protect  ioctl*/
	struct device *dev;
	struct device *dev_p;
	struct cdev *dev_cdevp;
	struct class *ece_class;
	dev_t dev_t;
	void __iomem *base_addr;
	char __iomem *ed_buffer;
	u32 smem_len;
	u32 smem_start;
	u32 lin_pitch;
	u32 enc_gap;
};

struct npcm750_ece *registered_ece;

static void update_bits(struct npcm750_ece *ece, u32 offset,
				unsigned long mask, u32 bits)
{
	u32 t = readl(ece->base_addr + offset);

	t &= ~mask;
	t |= bits & mask;
	writel(t, ece->base_addr + offset);
}

/* Rectangle Compressed Data Ready */
static void
ece_clear_drs(struct npcm750_ece *ece)
{
	update_bits(ece, DDA_STS, CDREADY, CDREADY);
}

/* Clear Offset of Compressed Rectangle*/
static void ece_clear_rect_offset(struct npcm750_ece *ece)
{
	writel(0, ece->base_addr + HEX_RECT_OFFSET);
}

/* Read Offset of Compressed Rectangle*/
static u32 ece_read_rect_offset(struct npcm750_ece *ece)
{
	return readl(ece->base_addr + HEX_RECT_OFFSET);
}

/* Return TRUE if a rectangle finished to be compressed */
static u32 ece_is_rect_compressed(struct npcm750_ece *ece)
{
	u32 temp = readl(ece->base_addr + DDA_STS);

	if (!(temp & CDREADY))
		return 0;

	return 1;
}

static u32 ece_get_ed_size(struct npcm750_ece *ece, u32 offset)
{
	u32 size;
	char *buffer = ece->ed_buffer + offset;

	while (!ece_is_rect_compressed(ece))
		;

	size = (u32)(buffer[0]
			| (buffer[1] << 8)
			| (buffer[2] << 16)
			| (buffer[3] << 24));

	ece_clear_drs(ece);
	return size;
}


/* This routine reset the FIFO as a bypass for Z1 chip */
static void ece_fifo_reset_bypass(struct npcm750_ece *ece)
{
	update_bits(ece, DDA_CTRL, ECEEN, ~ECEEN);
	update_bits(ece, DDA_CTRL, ECEEN, ECEEN);
}

/* This routine Encode the desired rectangle */
static void ece_enc_rect(struct npcm750_ece *ece,
			 u32 r_off_x, u32 r_off_y, u32 r_w, u32 r_h)
{
	u32 rect_offset =
		(r_off_y * ece->lin_pitch) + (r_off_x * 2);
	u32 temp;
	u32 w_tile;
	u32 h_tile;
	u32 w_size = ECE_TILE_W;
	u32 h_size = ECE_TILE_H;

	ece_fifo_reset_bypass(ece);

	writel(rect_offset, ece->base_addr + RECT_XY);

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

	temp = ((w_size - 1) << ECE_RECT_DIMEN_WLTR_OFFSET)
		| ((h_size - 1) << ECE_RECT_DIMEN_HLTR_OFFSET)
		| ((w_tile - 1) << ECE_RECT_DIMEN_WR_OFFSET)
		| ((h_tile - 1) << ECE_RECT_DIMEN_HR_OFFSET);

	writel(temp, ece->base_addr + RECT_DIMEN);
}

/* This routine sets the Encoded Data base address */
static u32 ece_set_enc_dba(struct npcm750_ece *ece, u32 addr)
{
	writel(addr, ece->base_addr + ED_BA);

	return 0;
}

/* This routine sets the Frame Buffer base address */
static u32 ece_set_fb_addr(struct npcm750_ece *ece, u32 buffer)
{
	writel(buffer, ece->base_addr + FBR_BA);

	return 0;
}

/* Set the line pitch (in bytes) for the frame buffers. */
/* Can be on of those values: 512, 1024, 2048, 2560 or 4096 bytes */
static void ece_set_lp(struct npcm750_ece *ece, u32 pitch)
{
	u32 lp;

	switch (pitch) {
	case 512:
		lp = LP_512;
		break;
	case 1024:
		lp = LP_1024;
		break;
	case 2048:
		lp = LP_2048;
		break;
	case 2560:
		lp = LP_2560;
		break;
	case 4096:
		lp = LP_4096;
		break;
	default:
		return;
	}

	ece->lin_pitch = pitch;
	writel(lp, ece->base_addr + RESOL);
}

/* Stop and reset the ECE state machine */
static void ece_reset(struct npcm750_ece *ece)
{
	update_bits(ece, DDA_CTRL, ECEEN, ~ECEEN);
	update_bits(ece, HEX_CTRL, ENCDIS, ENCDIS);
	update_bits(ece, DDA_CTRL, ECEEN, ECEEN);
	update_bits(ece, HEX_CTRL, ENCDIS, ~ENCDIS);

	ece_clear_rect_offset(ece);
}

/* Initialise the ECE block and interface library */
static int ece_initialise(struct npcm750_ece *ece)
{
	ece_reset(ece);
	ece_clear_drs(ece);
	ece_set_enc_dba(ece, ece->smem_start);
	ece->lin_pitch = DEFAULT_LP;

	return 0;
}

/* Disable the ECE block*/
static int ece_deinit(struct npcm750_ece *ece)
{
	update_bits(ece, DDA_CTRL, ECEEN, ~ECEEN);
	update_bits(ece, HEX_CTRL, ENCDIS, ENCDIS);
	ece_clear_rect_offset(ece);
	ece_clear_drs(ece);

	return 0;
}

static int
npcm750_ece_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct npcm750_ece *ece = file->private_data;
	unsigned long start;
	u32 len;

	if (!ece)
		return -ENODEV;

	start = ece->smem_start;
	len = ece->smem_len;

	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	fb_pgprotect(file, vma, start);
	return vm_iomap_memory(vma, start, len);
}

static int npcm750_ece_open(struct inode *inode, struct file *filp)
{
	if (!registered_ece)
		return -ENODEV;

	filp->private_data = registered_ece;

	ece_initialise(registered_ece);

	return 0;
}

static int npcm750_ece_release(struct inode *inode, struct file *filp)
{
	struct npcm750_ece *ece = filp->private_data;

	ece_deinit(ece);

	return 0;
}

long npcm750_ece_ioctl(struct file *filp, unsigned int cmd, unsigned long args)
{
	int err = 0;
	struct npcm750_ece *ece = filp->private_data;

	mutex_lock(&ece->mlock);

	switch (cmd) {
	case ECE_IOCCLEAR_OFFSET:

		ece_clear_rect_offset(ece);

		break;
	case ECE_IOCGET_OFFSET:
	{
		u32 offset = ece_read_rect_offset(ece);

		err = copy_to_user((int __user *)args, &offset, sizeof(offset))
			? -EFAULT : 0;
		break;
	}
	case ECE_IOCSETLP:
	{
		struct ece_ioctl_cmd data;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err) {
			mutex_unlock(&ece->mlock);
			return err;
		}

		if (!(data.lp % ECE_MIN_LP) && data.lp <= ECE_MAX_LP)
			ece_set_lp(ece, data.lp);

		break;
	}
	case ECE_IOCSETFB:
	{
		struct ece_ioctl_cmd data;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err) {
			mutex_unlock(&ece->mlock);
			return err;
		}

		if (!data.framebuf) {
			err = -EFAULT;
			break;
		}

		ece_set_fb_addr(ece, data.framebuf);
		break;
	}
	case ECE_IOCGETED:
	{
		struct ece_ioctl_cmd data;
		u32 ed_size = 0;
		u32 offset = 0;

		err = copy_from_user(&data, (int __user *)args, sizeof(data))
			? -EFAULT : 0;
		if (err) {
			mutex_unlock(&ece->mlock);
			return err;
		}

		offset = ece_read_rect_offset(ece);
		ece_enc_rect(ece, data.x, data.y, data.w, data.h);
		ed_size = ece_get_ed_size(ece, offset);

		ece->enc_gap =
			(readl(ece->base_addr + HEX_CTRL) & ENC_GAP)
			>> ENC_GAP_OFFSET;

		if (ece->enc_gap == 0)
			ece->enc_gap = ENC_MIN_GAP_SIZE;

		data.gap_len = ece->enc_gap;
		data.len = ed_size;
		err = copy_to_user((int __user *)args, &data, sizeof(data))
			? -EFAULT : 0;
		break;
	}
	default:
		break;
	}

	mutex_unlock(&ece->mlock);

	return err;
}

struct file_operations const npcm750_ece_fops = {
	.unlocked_ioctl = npcm750_ece_ioctl,
	.open = npcm750_ece_open,
	.release = npcm750_ece_release,
	.mmap = npcm750_ece_mmap,
};

static int npcm750_ece_device_create(struct npcm750_ece *ece)
{
	int ret;
	dev_t dev;
	struct cdev *dev_cdevp = ece->dev_cdevp;

	ret = alloc_chrdev_region(&dev, 0, 1, "hextile");
	if (ret < 0) {
		pr_err("alloc_chrdev_region() failed for ece\n");
		goto err;
	}

	dev_cdevp = kmalloc(sizeof(*dev_cdevp), GFP_KERNEL);
	if (!dev_cdevp)
		goto err;

	cdev_init(dev_cdevp, &npcm750_ece_fops);
	dev_cdevp->owner = THIS_MODULE;
	ece->dev_t = dev;
	ret = cdev_add(dev_cdevp, MKDEV(MAJOR(dev),  MINOR(dev)), 1);
	if (ret < 0) {
		pr_err("add chr dev failed\n");
		goto err;
	}

	ece->ece_class = class_create(THIS_MODULE, "hextile");
	if (IS_ERR(ece->ece_class)) {
		ret = PTR_ERR(ece->ece_class);
		pr_err("Unable to create ece class; errno = %d\n", ret);
		ece->ece_class = NULL;
		goto err;
	}

	ece->dev = device_create(ece->ece_class, ece->dev_p,
				 MKDEV(MAJOR(dev),  MINOR(dev)),
				 ece, "hextile");
	if (IS_ERR(ece->dev)) {
		/* Not fatal */
		pr_err("Unable to create device for ece; errno = %ld\n",
		       PTR_ERR(ece->dev));
		ece->dev = NULL;
		goto err;
	}
	return 0;

err:
	if (!dev_cdevp)
		kfree(dev_cdevp);
	return ret;
}

static int npcm750_ece_probe(struct platform_device *pdev)
{
	int ret = 0;
	struct npcm750_ece *ece = NULL;

	ece = kzalloc(sizeof(*ece), GFP_KERNEL);
	if (!ece)
		return -ENOMEM;

	mutex_init(&ece->mlock);

	of_property_read_u32(pdev->dev.of_node,
			     "mem-addr", &ece->smem_start);
	of_property_read_u32(pdev->dev.of_node,
			     "mem-size", &ece->smem_len);

	if (request_mem_region(ece->smem_start,
			       ece->smem_len, "npcm750-ece") == NULL) {
		dev_err(&pdev->dev, "%s: failed to request ece memory region\n",
			__func__);
		ret = -EBUSY;
		goto err;
	}

	ece->ed_buffer = ioremap(ece->smem_start, ece->smem_len);
	if (!ece->ed_buffer) {
		dev_err(&pdev->dev, "%s: cannot map ece memory region\n",
			__func__);
		ret = -EIO;
		goto err;
	}

	ece->base_addr = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(ece->base_addr)) {
		dev_err(&pdev->dev, "%s: failed to ioremap ece base address\n",
			__func__);
		ret = PTR_ERR(ece->base_addr);
		goto err;
	}

	ece->dev_p = &pdev->dev;

	ret = npcm750_ece_device_create(ece);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to create device\n",
			__func__);
		goto err;
	}

	registered_ece = ece;

	pr_info("NPCM750 ECE Driver probed\n");
	return 0;

err:
	kfree(ece);
	return ret;
}

static int npcm750_ece_remove(struct platform_device *pdev)
{
	struct npcm750_ece *ece = platform_get_drvdata(pdev);

	device_destroy(ece->ece_class, ece->dev_t);
	kfree(ece);
	registered_ece = NULL;

	return 0;
}

static const struct of_device_id npcm750_ece_of_match_table[] = {
	{ .compatible = "nuvoton,npcm750-ece"},
	{}
};
MODULE_DEVICE_TABLE(of, npcm750_ece_of_match_table);

static struct platform_driver npcm750_ece_driver = {
	.driver		= {
		.name	= "npcm750_ece",
		.of_match_table = npcm750_ece_of_match_table,
	},
	.probe		= npcm750_ece_probe,
	.remove		= npcm750_ece_remove,
};

module_platform_driver(npcm750_ece_driver);
MODULE_DESCRIPTION("Nuvoton NPCM750 ECE Driver");
MODULE_AUTHOR("KW Liu <kwliu@nuvoton.com>");
MODULE_LICENSE("GPL v2");
