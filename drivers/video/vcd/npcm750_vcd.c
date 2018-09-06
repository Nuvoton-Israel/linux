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
#include <asm/fb.h>
#include "vcd.h"

#define VCD_IOC_MAGIC     'v'
#define VCD_IOCGETINFO	_IOR(VCD_IOC_MAGIC,  1, struct vcd_info)
#define VCD_IOCSENDCMD	_IOW(VCD_IOC_MAGIC,  2, int)
#define VCD_IOCCHKRES	_IOR(VCD_IOC_MAGIC,  3, int)
#define VCD_IOCGETDIFF	_IOR(VCD_IOC_MAGIC,  4, struct vcd_diff)
#define VCD_IOCDIFFCNT	_IOR(VCD_IOC_MAGIC,  5, int)
#define VCD_IOC_MAXNR     6

#define VCD_OP_TIMEOUT 100

#define DEVICE_NAME "vcd"

struct class *vcd_class;
static struct vcd_inst *registered_vcd;
static const char vcd_name[] = "NPCM750 VCD";

static irqreturn_t npcm750_vcd_interrupt(int irq, void *dev_instance)
{
	struct device *dev = dev_instance;
	struct vcd_inst *vcd = (struct vcd_inst *)dev->driver_data;
	u32 status, irq_in;
	u8 done = 0;

	spin_lock(&vcd->lock);

	status = vcd_get_status(vcd);
	irq_in = (status & VCD_STAT_IRQ);

	if (irq_in) {
		done = (status & VCD_STAT_DONE);
		if (done) {
			int i;

			if (vcd->smem_base)
				for (i = 0 ; i < vcd->info.vdisp ; i++) {
					u32 hbytes = vcd->info.hdisp * 2;
					u32 dest_of = i * hbytes;
					u32 src_of = i * vcd->info.line_pitch;

					memcpy(
						vcd->smem_base + dest_of,
						vcd->frame_base + src_of,
						hbytes);
				}

			vcd->diff_cnt = 0;
			if (vcd->cmd > 0) {
				vcd_free_diff_table(vcd);
				vcd_get_diff_table(vcd);
			}
		}
	}

	vcd_clear_status(vcd, status & (VCD_STAT_CLEAR | done));
	spin_unlock(&vcd->lock);
	return IRQ_HANDLED;
}

static int
npcm750_vcd_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct vcd_inst *vcd = file->private_data;
	u32 start;
	u32 len;

	if (!vcd)
		return -ENODEV;

	start = vcd->smem_start;
	len = vcd->smem_len;
	vma->vm_page_prot = vm_get_page_prot(vma->vm_flags);
	fb_pgprotect(file, vma, start);

	return vm_iomap_memory(vma, start, len);
}

static int
npcm750_vcd_release(struct inode *inode, struct file *file)
{
	struct vcd_inst *vcd = file->private_data;

	vcd_deinit(vcd);

	return 0;
}

static int
npcm750_vcd_open(struct inode *inode, struct file *file)
{
	int ret = 0;

	if (!registered_vcd)
		return -ENODEV;

	file->private_data = registered_vcd;

	ret = vcd_init(registered_vcd);
	if (ret)
		dev_err(registered_vcd->dev, "%s: failed to init vcd module\n",
			__func__);

	return ret;
}

static long
npcm750_do_vcd_ioctl(struct vcd_inst *vcd, unsigned int cmd,
			 unsigned long arg)
{
	void __user *argp = (void __user *)arg;
	long ret = 0;

	mutex_lock(&vcd->mlock);
	switch (cmd) {
	case VCD_IOCGETINFO:
		ret = copy_to_user(argp, &vcd->info, sizeof(vcd->info))
			? -EFAULT : 0;
		break;
	case VCD_IOCSENDCMD:
	{
		int vcd_cmd;
		unsigned long timeout;

		ret = copy_from_user(&vcd_cmd, argp, sizeof(vcd_cmd))
			? -EFAULT : 0;

		vcd_command(vcd, vcd_cmd);

		/* Wait for cmd to complete */
		timeout = jiffies + VCD_OP_TIMEOUT;
		while (!vcd_is_op_ok(vcd)) {
			if (time_after(jiffies, timeout)) {
				vcd_reset(vcd);
				break;
			}
			cpu_relax();
		}

		break;
	}
	case VCD_IOCCHKRES:
	{
		int changed;

		changed = vcd_check_res(vcd);
		ret = copy_to_user(argp, &changed, sizeof(changed))
			? -EFAULT : 0;
		break;
	}
	case VCD_IOCGETDIFF:
	{
		struct vcd_diff_list *list;
		struct vcd_diff diff;
		struct list_head *head = &vcd->list.list;

		if (vcd->diff_cnt == 0) {
			diff.x = 0;
			diff.y = 0;
			diff.w = vcd->info.hdisp;
			diff.h = vcd->info.vdisp;
		} else {
			list = list_first_entry_or_null(head,
							struct vcd_diff_list,
							list);
			if (!list) {
				diff.x = 0;
				diff.y = 0;
				diff.w = 0;
				diff.h = 0;
			} else {
				diff.x = list->diff.x;
				diff.y = list->diff.y;
				diff.w = list->diff.w;
				diff.h = list->diff.h;
			}
			if (list) {
				list_del(&list->list);
				kfree(list);
				vcd->diff_cnt--;
			}
		}
		ret = copy_to_user(argp, &diff, sizeof(struct vcd_diff))
			? -EFAULT : 0;
		break;
	}
	case VCD_IOCDIFFCNT:
		ret = copy_to_user(argp, &vcd->diff_cnt, sizeof(int))
			? -EFAULT : 0;
		break;
	default:
		break;
	}
	mutex_unlock(&vcd->mlock);
	return ret;
}

static long
npcm750_vcd_ioctl(struct file *file, unsigned int cmd, unsigned long arg)
{
	struct vcd_inst *vcd = file->private_data;

	if (!vcd)
		return -ENODEV;
	return npcm750_do_vcd_ioctl(vcd, cmd, arg);
}

static const struct file_operations npcm750_vcd_fops = {
	.owner		= THIS_MODULE,
	.open		= npcm750_vcd_open,
	.release	= npcm750_vcd_release,
	.mmap		= npcm750_vcd_mmap,
	.unlocked_ioctl = npcm750_vcd_ioctl,
};

static int npcm750_vcd_device_create(struct vcd_inst *vcd)
{
	int ret;
	dev_t dev;
	struct cdev *dev_cdevp = NULL;

	ret = alloc_chrdev_region(&dev, 0, 1, DEVICE_NAME);
	if (ret < 0) {
		pr_err("alloc_chrdev_region() failed for vcd\n");
		goto err;
	}

	vcd->dev_id = dev;

	dev_cdevp = kmalloc(sizeof(*dev_cdevp), GFP_KERNEL);
	if (!dev_cdevp) {
		ret = -ENOMEM;
		goto err;
	}

	cdev_init(dev_cdevp, &npcm750_vcd_fops);
	dev_cdevp->owner = THIS_MODULE;
	ret = cdev_add(dev_cdevp, MKDEV(MAJOR(dev),  MINOR(dev)), 1);
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

	vcd->dev = device_create(vcd_class, vcd->dev_p,
				 MKDEV(MAJOR(dev), MINOR(dev)),
				 vcd,
				 DEVICE_NAME);
	if (IS_ERR(vcd->dev)) {
		ret = PTR_ERR(vcd->dev);
		pr_err("Unable to create device for vcd; errno = %ld\n",
		       PTR_ERR(vcd->dev));
		vcd->dev = NULL;
		goto err;
	}

	return 0;

err:
	if (!dev_cdevp)
		kfree(dev_cdevp);
	return ret;
}

static int npcm750_vcd_probe(struct platform_device *pdev)
{
	struct vcd_inst *vcd;
	void __iomem *reg_base;
	int irq;
	int ret;

	vcd = kzalloc(sizeof(*vcd), GFP_KERNEL);
	if (!vcd)
		return -ENOMEM;

	spin_lock_init(&vcd->lock);
	mutex_init(&vcd->mlock);

	vcd->gcr_regmap =
		syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
	if (IS_ERR(vcd->gcr_regmap)) {
		dev_err(&pdev->dev, "%s: failed to find nuvoton,npcm750-gcr\n",
			__func__);
		ret = IS_ERR(vcd->gcr_regmap);
		goto err;
	}

	vcd->gfx_regmap =
		syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gfxi");
	if (IS_ERR(vcd->gfx_regmap)) {
		dev_err(&pdev->dev, "%s: failed to find nuvoton,npcm750-gfxi\n",
			__func__);
		ret = IS_ERR(vcd->gfx_regmap);
		goto err;
	}

	of_property_read_u32(pdev->dev.of_node,
			     "mem-addr", &vcd->frame_start);
	of_property_read_u32(pdev->dev.of_node,
			     "mem-size", &vcd->frame_len);

	if (request_mem_region(vcd->frame_start,
			       vcd->frame_len, vcd_name) == NULL) {
		dev_err(&pdev->dev, "%s: failed to request vcd memory region\n",
			__func__);
		ret = -EBUSY;
		goto err;
	}

	vcd->frame_base = ioremap(vcd->frame_start, vcd->frame_len);
	if (!vcd->frame_base) {
		dev_err(&pdev->dev, "%s: cannot map vcd memory region\n",
			__func__);
		ret = -EIO;
		goto err;
	}

	reg_base = of_iomap(pdev->dev.of_node, 0);
	if (IS_ERR(reg_base)) {
		dev_err(&pdev->dev, "%s: failed to ioremap vcd base address\n",
			__func__);
		ret = PTR_ERR(reg_base);
		goto err;
	}

	vcd->reg = (struct vcd_reg *)reg_base;
	vcd->dev_p = &pdev->dev;

	ret = npcm750_vcd_device_create(vcd);
	if (ret)
		goto err;

	irq = of_irq_get(pdev->dev.of_node, 0);
	ret = request_irq(irq, npcm750_vcd_interrupt,
			  IRQF_SHARED, vcd_name, vcd->dev);
	if (ret) {
		dev_err(&pdev->dev, "%s: failed to request irq for vcd\n",
			__func__);
		goto err;
	}

	platform_set_drvdata(pdev, vcd);
	INIT_LIST_HEAD(&vcd->list.list);
	registered_vcd = vcd;

	pr_info("NPCM750 VCD Driver probed\n");
	return 0;
err:
	kfree(vcd);
	return ret;
}

static int npcm750_vcd_remove(struct platform_device *pdev)
{
	struct vcd_inst *vcd = platform_get_drvdata(pdev);

	device_destroy(vcd_class, vcd->dev_id);

	vcd_deinit(vcd);

	kfree(vcd);

	registered_vcd = NULL;

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
