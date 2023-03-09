// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2022, Microsoft Corporation

#define pr_fmt(fmt) "cerberus-on-TIP: " fmt

#include <linux/errno.h>
#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mailbox_client.h>
#include <linux/miscdevice.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/platform_device.h>
#include <linux/sysfs.h>
#include <linux/kfifo.h>
#include <linux/slab.h>

#define MSG_QUEUE_SIZE 32 // Max number of msgs in the queue
#define MAX_MSG_SIZE 2048 // Should be equal to rd shmem size

/**
 * struct chan_shmem - mbx channel's shmem info
 *
 * @off: IO memory offset address of shmem
 * @size: Size of the shmem region
 */
struct mbx_shmem {
	u8 __iomem *off;
	size_t size;
};

/*
 * struct msg - message node in the queue
 *
 * @len: length of the message
 * @buf: message buffer
 */
struct cerberus_msg {
	u8 buf[MAX_MSG_SIZE];
};

/*
 * struct cerberus_drvinfo - cerberus driver info
 *
 * @pdev: Reference to platform device
 * @cl: Cerberus mbox client info
 * @chan: Mailbox channel info
 * @rd_win: Mailbox shmem read window
 * @wr_win: Mailbox shmem write window
 * @mutex: Lock for serializing the writes
 * @lock: Spinlock for IO operations
 * @mq: Queue to hold the messages read from Cerberus TIP
 * @miscdev: Miscellaneous device structure
 */
struct cerberus_drvinfo {
	struct platform_device *pdev;
	struct mbox_client cl;
	struct mbox_chan *chan;
	struct mbx_shmem rd_win;
	struct mbx_shmem wr_win;
	struct mutex mutex;
	spinlock_t lock;
	DECLARE_KFIFO_PTR(mq, struct cerberus_msg);
	struct miscdevice miscdev;
};

static struct cerberus_drvinfo *to_cerberus_drvinfo(struct file *fp)
{
	return container_of(fp->private_data, struct cerberus_drvinfo, miscdev);
}

ssize_t cerberus_read(struct file *fp, char __user *buf, size_t count,
		      loff_t *ppos)
{
	ssize_t ret;
	struct cerberus_drvinfo *cerberus = to_cerberus_drvinfo(fp);
	unsigned long flags;
	struct cerberus_msg *rmsg;

	/* offset always should be 0 */
	*ppos = 0;

	if ((fp->f_flags & O_NONBLOCK) && kfifo_is_empty(&cerberus->mq)) {
		return -EAGAIN;
	}

	rmsg = kmalloc(sizeof(struct cerberus_msg), GFP_KERNEL);
	if (!rmsg)
		return 0;

	spin_lock_irqsave(&cerberus->lock, flags);
	ret = kfifo_len(&cerberus->mq);
	if (ret > 0)
		kfifo_out(&cerberus->mq, rmsg, 1);
	spin_unlock_irqrestore(&cerberus->lock, flags);

	if (ret <= 0) {
		ret = 0;
		goto fail_free_resources;
	}

	ret = simple_read_from_buffer(buf, count, ppos, rmsg->buf,
				      cerberus->rd_win.size);
fail_free_resources:
	kfree(rmsg);
	return ret;
}

/*
 * Callback handler for data received from Cerberus.
 * This function copies the message from read shmem to the next
 * message slot in the message queue, and wakes up one thread waiting
 * on the message
 *
 * @cl: mailbox client
 * @msg: message from cerberus (not used)
 */
static void msg_from_cerberus(struct mbox_client *cl, void *msg)
{
	unsigned long flags;
	struct cerberus_drvinfo *cerberus = dev_get_drvdata(cl->dev);

	spin_lock_irqsave(&cerberus->lock, flags);
	if (kfifo_is_full(&cerberus->mq)) {
		pr_err("Msg queue is full. Oldest message will be lost\n");
		kfifo_skip(&cerberus->mq);
	}
	kfifo_in(&cerberus->mq, (struct cerberus_msg *)cerberus->rd_win.off, 1);
	spin_unlock_irqrestore(&cerberus->lock, flags);
}

ssize_t cerberus_write(struct file *fp, const char __user *buf, size_t count,
		       loff_t *ppos)
{
	ssize_t ret;
	struct cerberus_drvinfo *cerberus = to_cerberus_drvinfo(fp);
	struct cerberus_msg *wmsg;

	/* Write size must be with in window limits */
	if (count > cerberus->wr_win.size)
		return -EINVAL;

	/* offset always should be 0 */
	*ppos = 0;

	wmsg = kmalloc(sizeof(struct cerberus_msg), GFP_KERNEL);
	if (!wmsg)
		return 0;

	memset(wmsg, 0, sizeof(struct cerberus_msg));

	if (copy_from_user(wmsg->buf, buf, count)) {
		ret = -EFAULT;
		goto fail_free_resources;
	}

	mutex_lock(&cerberus->mutex);

	memcpy_toio(cerberus->wr_win.off, wmsg->buf, count);

	/* Ring the doorbell (Blocking call) and wait for the data
	 * to be received by TIP.
	 */
	mbox_send_message(cerberus->chan, cerberus->wr_win.off);

	ret = count;

fail_unlock:
	mutex_unlock(&cerberus->mutex);
fail_free_resources:
	kfree(wmsg);
	return ret;
}

static const struct file_operations cerberus_fops = {
	.owner = THIS_MODULE,
	.read = cerberus_read,
	.write = cerberus_write,
};

static int cerberus_probe(struct platform_device *pdev)
{
	int ret;
	struct device *dev = &pdev->dev;
	struct device_node *np = dev->of_node;
	struct cerberus_drvinfo *cerberus;
	struct resource res;
	struct device_node *shmem;
	resource_size_t shmem_size;
	u8 __iomem *off;

	cerberus = devm_kzalloc(dev, sizeof(*cerberus), GFP_KERNEL);
	if (!cerberus) {
		dev_err(dev, "memory not allocated for cerberus drvinfo\n");
		return -ENOMEM;
	}

	/* extract shmem information from device node */
	shmem = of_parse_phandle(np, "shmem", 0);
	ret = of_address_to_resource(shmem, 0, &res);
	of_node_put(shmem);
	if (ret) {
		dev_err(dev, "Failed to get shared mem resource\n");
		return ret;
	}

	shmem_size = resource_size(&res);
	off = devm_ioremap_resource(dev, &res);
	if (IS_ERR(off)) {
		dev_err(dev, "device mem io remap failed\n");
		return PTR_ERR(off);
	}

	/* top half for write and bottom half for read */
	cerberus->wr_win.off = off;
	cerberus->wr_win.size = shmem_size >> 1;
	cerberus->rd_win.off = off + cerberus->wr_win.size;
	cerberus->rd_win.size = shmem_size - cerberus->wr_win.size;

	if (unlikely(cerberus->rd_win.size != MAX_MSG_SIZE)) {
		dev_err(dev, "Message size is not same as read shmem size\n");
		return -EINVAL;
	}

	mutex_init(&cerberus->mutex);
	spin_lock_init(&cerberus->lock);
	ret = kfifo_alloc(&cerberus->mq, MSG_QUEUE_SIZE, GFP_KERNEL);
	if (ret)
		return ret;

	/* set up and request mailbox channel */
	cerberus->cl.dev = dev;
	cerberus->cl.rx_callback = msg_from_cerberus; /* read non-blocking */
	cerberus->cl.tx_done = NULL;
	cerberus->cl.tx_block = true; /* write blocking */
	cerberus->cl.tx_tout = 500; /* msec */
	cerberus->chan = mbox_request_channel_byname(&cerberus->cl, "cerberus");
	if (IS_ERR(cerberus->chan)) {
		dev_err(dev, "mbox channel request failed\n");
		ret = PTR_ERR(cerberus->chan);
		goto fail_free_resources;
	}

	/* Register the driver as misc device */
	cerberus->miscdev.minor = MISC_DYNAMIC_MINOR;
	cerberus->miscdev.name = "cerberus";
	cerberus->miscdev.fops = &cerberus_fops;
	cerberus->miscdev.parent = dev;
	ret = misc_register(&cerberus->miscdev);
	if (ret) {
		dev_err(dev, "Unable to register misc device\n");
		goto fail_free_mbox_chan;
	}

	platform_set_drvdata(pdev, cerberus);

	dev_info(dev, "Cerberus mailbox client driver initialized\n");
	return 0;

fail_free_mbox_chan:
	dev_info(dev, "freeing mbox chan");
	mbox_free_channel(cerberus->chan);
fail_free_resources:
	dev_info(dev, "freeing kfifo");
	kfifo_free(&cerberus->mq);
	return ret;
}

static int cerberus_remove(struct platform_device *pdev)
{
	struct cerberus_drvinfo *cerberus = platform_get_drvdata(pdev);

	mbox_free_channel(cerberus->chan);
	kfifo_free(&cerberus->mq);
	misc_deregister(&cerberus->miscdev);

	return 0;
}

static const struct of_device_id cerberus_ids[] = {
	{
		.compatible = "nuvoton,cerberus",
	},
	{ /* sentinel */ }
};
MODULE_DEVICE_TABLE(of, cerberus_ids);

static struct platform_driver cerberus_mbox_client_driver = {
    .probe    =  cerberus_probe,
    .remove   =  cerberus_remove,
    .driver   = {
        .name  = "cerberus",
        .of_match_table = cerberus_ids,
        .owner = THIS_MODULE,
    },
};

module_platform_driver(cerberus_mbox_client_driver);

MODULE_LICENSE("GPL");
MODULE_AUTHOR("Parvathi Bhogaraju <pbhogaraju@microsoft.com>");
MODULE_DESCRIPTION("Mailbox client driver for cerberus on TIP");

