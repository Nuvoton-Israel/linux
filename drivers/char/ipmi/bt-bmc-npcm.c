// SPDX-License-Identifier: GPL-2.0+
/*
 * Copyright (c) 2015-2016, IBM Corporation.
 * Copyright (c) 2025, Nuvoton Corporation.
 */

#include <linux/atomic.h>
#include <linux/bt-bmc.h>
#include <linux/errno.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/sched.h>

/*
 * This is a BMC device used to communicate to the host
 */
#define DEVICE_NAME	"ipmi-bt-host"

#define BT_CTRL		0x0
#define   BT_CTRL_B_BUSY		0x80
#define   BT_CTRL_H_BUSY		0x40
#define   BT_CTRL_OEM0			0x20
#define   BT_CTRL_SMS_ATN		0x10
#define   BT_CTRL_B2H_ATN		0x08
#define   BT_CTRL_H2B_ATN		0x04
#define   BT_CTRL_CLR_RD_PTR		0x02
#define   BT_CTRL_CLR_WR_PTR		0x01
#define BT_BMC2HOST	0x4
#define BT_HOST2BMC	0x4
#define BT_INTMASK	0x8
#define   BT_INTMASK_H2B_IRQEN		0x01
#define   BT_INTMASK_H2B_IRQ		0x02
#define   BT_INTMASK_BMC_HWRST		0x80

#define BT_BMC_BUFFER_SIZE 64

struct bt_bmc {
	struct device		dev;
	struct miscdevice	miscdev;
	void __iomem		*base;
	int			irq;
	wait_queue_head_t	queue;
	struct mutex		mutex;
};

static atomic_t open_count = ATOMIC_INIT(0);

static inline u8 bt_inb(struct bt_bmc *bt_bmc, int reg)
{
	return readb(bt_bmc->base + reg);
}

static inline void bt_outb(struct bt_bmc *bt_bmc, u8 data, int reg)
{
	writeb(data, bt_bmc->base + reg);
}

static void clr_rd_ptr(struct bt_bmc *bt_bmc)
{
	bt_outb(bt_bmc, BT_CTRL_CLR_RD_PTR, BT_CTRL);
}

static void clr_wr_ptr(struct bt_bmc *bt_bmc)
{
	bt_outb(bt_bmc, BT_CTRL_CLR_WR_PTR, BT_CTRL);
}

static void clr_h2b_atn(struct bt_bmc *bt_bmc)
{
	bt_outb(bt_bmc, BT_CTRL_H2B_ATN, BT_CTRL);
}

static void set_b_busy(struct bt_bmc *bt_bmc)
{
	if (!(bt_inb(bt_bmc, BT_CTRL) & BT_CTRL_B_BUSY))
		bt_outb(bt_bmc, BT_CTRL_B_BUSY, BT_CTRL);
}

static void clr_b_busy(struct bt_bmc *bt_bmc)
{
	if (bt_inb(bt_bmc, BT_CTRL) & BT_CTRL_B_BUSY)
		bt_outb(bt_bmc, BT_CTRL_B_BUSY, BT_CTRL);
}

static void set_b2h_atn(struct bt_bmc *bt_bmc)
{
	bt_outb(bt_bmc, BT_CTRL_B2H_ATN, BT_CTRL);
}

static ssize_t bt_read_data(struct bt_bmc *bt_bmc, u8 *buf, size_t bytes)
{
	int words = (bytes + 3) / 4;
	u32 *buf32 = (u32 *)buf;
	int i;

	for (i = 0; i < words; i++) {
		*buf32 = readl(bt_bmc->base + BT_HOST2BMC);
		buf32++;
	}

	return bytes;
}

static ssize_t bt_write_data(struct bt_bmc *bt_bmc, u8 *buf, size_t bytes)
{
	int words = (bytes + 3) / 4;
	u32 *buf32 = (u32 *)buf;
	int i;

	for (i = 0; i < words; i++)
		writel(buf32[i], bt_bmc->base + BT_BMC2HOST);

	return bytes;
}

static void set_sms_atn(struct bt_bmc *bt_bmc)
{
	bt_outb(bt_bmc, BT_CTRL_SMS_ATN, BT_CTRL);
}

static struct bt_bmc *file_bt_bmc(struct file *file)
{
	return container_of(file->private_data, struct bt_bmc, miscdev);
}

static int bt_bmc_open(struct inode *inode, struct file *file)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);

	if (atomic_inc_return(&open_count) == 1) {
		clr_b_busy(bt_bmc);
		return 0;
	}

	atomic_dec(&open_count);
	return -EBUSY;
}

/*
 * The BT (Block Transfer) interface means that entire messages are
 * buffered by the host before a notification is sent to the BMC that
 * there is data to be read. The first byte is the length and the
 * message data follows. The read operation just tries to capture the
 * whole before returning it to userspace.
 *
 * BT Message format :
 *
 *    Byte 1  Byte 2     Byte 3  Byte 4  Byte 5:N
 *    Length  NetFn/LUN  Seq     Cmd     Data
 *
 * BT Interface Write Transfer sequence:
 * - Wait for H2B_ATN to be set
 * - Set B_BUSY (indicating BMC is preparing to transfer)
 * - Clear H2B_ATN
 * - Read HOST2BMC buffer
 * - Clear B_BUSY (indicating BMC is done transferring)
 */
static ssize_t bt_bmc_read(struct file *file, char __user *buf,
			   size_t count, loff_t *ppos)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);
	u8 len;
	u8 kbuffer[BT_BMC_BUFFER_SIZE];
	u32 *word = (u32 *)&kbuffer[0];
	ssize_t ret = 0;
	ssize_t nread;

	if (count > BT_BMC_BUFFER_SIZE)
		return -EINVAL;

	WARN_ON(*ppos);

	if (wait_event_interruptible(bt_bmc->queue,
				     bt_inb(bt_bmc, BT_CTRL) & BT_CTRL_H2B_ATN))
		return -ERESTARTSYS;

	mutex_lock(&bt_bmc->mutex);

	if (unlikely(!(bt_inb(bt_bmc, BT_CTRL) & BT_CTRL_H2B_ATN))) {
		ret = -EIO;
		goto out_unlock;
	}

	set_b_busy(bt_bmc);
	clr_h2b_atn(bt_bmc);
	clr_rd_ptr(bt_bmc);

	/*
	 * The BT frames start with the message length, which does not
	 * include the length byte.
	 */
	*word = readl(bt_bmc->base + BT_HOST2BMC);
	len = kbuffer[0];

	/* We pass the length back to userspace as well */
	if (len + 1 > count)
		len = count - 1;

	nread = min_t(ssize_t, len, 3);
	if (copy_to_user(buf, kbuffer, nread + 1)) {
		ret = -EFAULT;
		goto quit;
	}
	len -= nread;
	buf += nread + 1;
	ret += nread + 1;

	while (len) {
		nread = min_t(ssize_t, len, sizeof(kbuffer));

		bt_read_data(bt_bmc, kbuffer, nread);

		if (copy_to_user(buf, kbuffer, nread)) {
			ret = -EFAULT;
			break;
		}
		len -= nread;
		buf += nread;
		ret += nread;
	}
quit:
	clr_b_busy(bt_bmc);

out_unlock:
	mutex_unlock(&bt_bmc->mutex);
	return ret;
}

/*
 * BT Message response format :
 *
 *    Byte 1  Byte 2     Byte 3  Byte 4  Byte 5  Byte 6:N
 *    Length  NetFn/LUN  Seq     Cmd     Code    Data
 *
 * BT Interface Read Transfer sequence:
 * - Wait for H_BUSY to be cleared
 * - Write message to BMC2HOST buffer
 * - Set B2H_ATN (indicating BMC has put data)
 */
static ssize_t bt_bmc_write(struct file *file, const char __user *buf,
			    size_t count, loff_t *ppos)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);
	u8 kbuffer[BT_BMC_BUFFER_SIZE];
	ssize_t ret = 0;
	ssize_t nwritten;

	/*
	 * send a minimum response size
	 */
	if (count < 5 || count > BT_BMC_BUFFER_SIZE)
		return -EINVAL;

	WARN_ON(*ppos);

	if (wait_event_interruptible(bt_bmc->queue,
				     !(bt_inb(bt_bmc, BT_CTRL) &
				       BT_CTRL_H_BUSY)))
		return -ERESTARTSYS;

	mutex_lock(&bt_bmc->mutex);

	if (unlikely(bt_inb(bt_bmc, BT_CTRL) & BT_CTRL_H_BUSY)) {
		ret = -EIO;
		goto out_unlock;
	}

	clr_wr_ptr(bt_bmc);

	while (count) {
		nwritten = min_t(ssize_t, count, sizeof(kbuffer));
		if (copy_from_user(&kbuffer, buf, nwritten)) {
			ret = -EFAULT;
			break;
		}

		bt_write_data(bt_bmc, kbuffer, nwritten);

		count -= nwritten;
		buf += nwritten;
		ret += nwritten;
	}

	set_b2h_atn(bt_bmc);

out_unlock:
	mutex_unlock(&bt_bmc->mutex);
	return ret;
}

static long bt_bmc_ioctl(struct file *file, unsigned int cmd,
			 unsigned long param)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);

	switch (cmd) {
	case BT_BMC_IOCTL_SMS_ATN:
		set_sms_atn(bt_bmc);
		return 0;
	}
	return -EINVAL;
}

static int bt_bmc_release(struct inode *inode, struct file *file)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);

	atomic_dec(&open_count);
	set_b_busy(bt_bmc);
	return 0;
}

static __poll_t bt_bmc_poll(struct file *file, poll_table *wait)
{
	struct bt_bmc *bt_bmc = file_bt_bmc(file);
	__poll_t mask = 0;
	u8 ctrl;

	poll_wait(file, &bt_bmc->queue, wait);

	ctrl = bt_inb(bt_bmc, BT_CTRL);

	if (ctrl & BT_CTRL_H2B_ATN)
		mask |= EPOLLIN;

	return mask;
}

static const struct file_operations bt_bmc_fops = {
	.owner		= THIS_MODULE,
	.open		= bt_bmc_open,
	.read		= bt_bmc_read,
	.write		= bt_bmc_write,
	.release	= bt_bmc_release,
	.poll		= bt_bmc_poll,
	.unlocked_ioctl	= bt_bmc_ioctl,
};

static irqreturn_t bt_bmc_irq(int irq, void *arg)
{
	struct bt_bmc *bt_bmc = arg;
	u32 reg;

	reg = readl(bt_bmc->base + BT_INTMASK);

	if (!(reg & BT_INTMASK_H2B_IRQ))
		return IRQ_NONE;

	/* Clear H2B_IRQ */
	writel(BT_INTMASK_H2B_IRQ | BT_INTMASK_H2B_IRQEN,
	       bt_bmc->base + BT_INTMASK);

	wake_up(&bt_bmc->queue);
	return IRQ_HANDLED;
}

static int bt_bmc_config_irq(struct bt_bmc *bt_bmc,
			     struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	int rc;

	bt_bmc->irq = platform_get_irq_optional(pdev, 0);
	if (bt_bmc->irq < 0)
		return bt_bmc->irq;

	rc = devm_request_irq(dev, bt_bmc->irq, bt_bmc_irq, IRQF_SHARED,
			      DEVICE_NAME, bt_bmc);
	if (rc < 0) {
		dev_warn(dev, "Unable to request IRQ %d\n", bt_bmc->irq);
		bt_bmc->irq = rc;
		return rc;
	}

	/* Clear H2B_IRQ */
	writel(BT_INTMASK_H2B_IRQ, bt_bmc->base + BT_INTMASK);

	/* Enable H2B interrupt */
	writel(BT_INTMASK_H2B_IRQEN, bt_bmc->base + BT_INTMASK);

	return 0;
}

static int bt_bmc_probe(struct platform_device *pdev)
{
	struct bt_bmc *bt_bmc;
	struct device *dev;
	int rc;

	dev = &pdev->dev;
	dev_info(dev, "Found bt bmc device\n");

	bt_bmc = devm_kzalloc(dev, sizeof(*bt_bmc), GFP_KERNEL);
	if (!bt_bmc)
		return -ENOMEM;

	dev_set_drvdata(&pdev->dev, bt_bmc);

	bt_bmc->base = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(bt_bmc->base))
		return PTR_ERR(bt_bmc->base);

	mutex_init(&bt_bmc->mutex);
	init_waitqueue_head(&bt_bmc->queue);

	bt_bmc->miscdev.minor	= MISC_DYNAMIC_MINOR;
	bt_bmc->miscdev.name	= DEVICE_NAME;
	bt_bmc->miscdev.fops	= &bt_bmc_fops;
	bt_bmc->miscdev.parent = dev;
	rc = misc_register(&bt_bmc->miscdev);
	if (rc) {
		dev_err(dev, "Unable to register misc device\n");
		return rc;
	}

	rc = bt_bmc_config_irq(bt_bmc, pdev);
	if (rc) {
		dev_err(dev, "Unable to configure IRQ\n");
		return rc;
	}

	clr_b_busy(bt_bmc);

	return 0;
}

static int bt_bmc_remove(struct platform_device *pdev)
{
	struct bt_bmc *bt_bmc = dev_get_drvdata(&pdev->dev);

	misc_deregister(&bt_bmc->miscdev);
	return 0;
}

static const struct of_device_id npcm_bt_bmc_match[] = {
	{ .compatible = "nuvoton,npcm845-bt-bmc" },
	{ },
};

static struct platform_driver npcm_bt_bmc_driver = {
	.driver = {
		.name		= DEVICE_NAME,
		.of_match_table = npcm_bt_bmc_match,
	},
	.probe = bt_bmc_probe,
	.remove = bt_bmc_remove,
};

module_platform_driver(npcm_bt_bmc_driver);

MODULE_DEVICE_TABLE(of, npcm_bt_bmc_match);
MODULE_LICENSE("GPL");
MODULE_AUTHOR("Stanley Chu <yschu@nuvoton.com>");
MODULE_AUTHOR("Alistair Popple <alistair@popple.id.au>");
MODULE_DESCRIPTION("Linux device interface to the IPMI BT interface");
