// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Intel Corporation.
 * Copyright (c) 2024 Nuvoton Technology corporation.
 */

#include <linux/fs.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <linux/sched/signal.h>
#include <linux/spinlock.h>
#include <linux/uaccess.h>
#include <linux/soc/nuvoton/npcm-espi.h>

#define DEVICE_NAME "espi-pltrstn"

struct npcm_espi_slave {
	struct regmap		*espi_regmap;
	struct device		*dev;
	int			irq;

	/* for PLTRST_N signal monitoring interface */
	struct miscdevice	miscdev;
	spinlock_t		pltrstn_lock; /* for PLTRST_N signal sampling */
	wait_queue_head_t	pltrstn_waitq;
	char			pltrstn;
	bool			pltrstn_in_avail;
};

/*
 * CFGUPD is shared with spi-npcm-flash, which is the only one that clears the
 * status when the flash channel is built in.  Its enable therefore belongs to
 * it, and @cfgupd lets the caller say whether this driver may switch the bit
 * on: never while spi-npcm-flash may still be waiting to probe, always once a
 * reset has cleared an enable it had already set.
 */
static void npcm_espi_slave_config_irq(struct npcm_espi_slave *priv,
				       bool cfgupd)
{
	u32 ie = NPCM_ESPI_IRQ_SLAVE;

	if (cfgupd)
		ie |= NPCM_ESPI_CFGUPD;

	regmap_update_bits(priv->espi_regmap, NPCM_ESPI_ESPIIE, ie, ie);
}

static irqreturn_t npcm_espi_slave_irq_handler(int irq, void *arg)
{
	struct npcm_espi_slave *priv = arg;
	u32 sts, evms1, handled = 0;

	regmap_read(priv->espi_regmap, NPCM_ESPI_ESPISTS, &sts);
	regmap_read(priv->espi_regmap, NPCM_ESPI_VWEVMS(1), &evms1);

	if (sts & NPCM_ESPI_PLTRST) {
		spin_lock(&priv->pltrstn_lock);
		priv->pltrstn = '0';
		priv->pltrstn_in_avail = true;
		spin_unlock(&priv->pltrstn_lock);
		wake_up_interruptible(&priv->pltrstn_waitq);

		regmap_write(priv->espi_regmap, NPCM_ESPI_ESPISTS, NPCM_ESPI_PLTRST);
		handled |= NPCM_ESPI_PLTRST;
	}

	if (sts & NPCM_ESPI_ESPIRST) {
		spin_lock(&priv->pltrstn_lock);
		priv->pltrstn = 'U';
		priv->pltrstn_in_avail = true;
		spin_unlock(&priv->pltrstn_lock);
		wake_up_interruptible(&priv->pltrstn_waitq);

		regmap_write(priv->espi_regmap, NPCM_ESPI_ESPISTS, NPCM_ESPI_ESPIRST);
		handled |= NPCM_ESPI_ESPIRST;

		npcm_espi_slave_config_irq(priv, true);
	}

	if (sts & NPCM_ESPI_CFGUPD) {
		spin_lock(&priv->pltrstn_lock);
		priv->pltrstn = (evms1 & NPCM_ESPI_VWEVMS_PLTRST) ? '1' : '0';
		priv->pltrstn_in_avail = true;
		spin_unlock(&priv->pltrstn_lock);
		wake_up_interruptible(&priv->pltrstn_waitq);

		/*
		 * Sampling the state is all this driver does with CFGUPD; the
		 * status belongs to spi-npcm-flash, which has more work to do
		 * before it may be cleared.  Leave it out of @handled so that
		 * a stuck bit reaches the spurious interrupt detector instead
		 * of being reported as serviced.
		 */
		if (!IS_ENABLED(CONFIG_SPI_NPCM_FLASH)) {
			regmap_write(priv->espi_regmap, NPCM_ESPI_ESPISTS,
				     NPCM_ESPI_CFGUPD);
			handled |= NPCM_ESPI_CFGUPD;
		}
	}

	return handled ? IRQ_HANDLED : IRQ_NONE;
}

static inline struct npcm_espi_slave *to_npcm_espi_slave(struct file *filp)
{
	return container_of(filp->private_data, struct npcm_espi_slave,
			    miscdev);
}

static int npcm_espi_slave_pltrstn_open(struct inode *inode, struct file *filp)
{
	struct npcm_espi_slave *priv = to_npcm_espi_slave(filp);

	if ((filp->f_flags & O_ACCMODE) != O_RDONLY)
		return -EACCES;

	/*Setting true returns first data after file open*/
	priv->pltrstn_in_avail = true;

	return 0;
}

static ssize_t npcm_espi_slave_pltrstn_read(struct file *filp, char __user *buf,
					    size_t count, loff_t *offset)
{
	struct npcm_espi_slave *priv = to_npcm_espi_slave(filp);
	DECLARE_WAITQUEUE(wait, current);
	char data, old_sample;
	int ret = 0;

	spin_lock_irq(&priv->pltrstn_lock);

	if (filp->f_flags & O_NONBLOCK) {
		if (!priv->pltrstn_in_avail) {
			ret = -EAGAIN;
			goto out_unlock;
		}
		data = priv->pltrstn;
		priv->pltrstn_in_avail = false;
	} else {
		add_wait_queue(&priv->pltrstn_waitq, &wait);
		set_current_state(TASK_INTERRUPTIBLE);

		old_sample = priv->pltrstn;

		do {
			if (old_sample != priv->pltrstn) {
				data = priv->pltrstn;
				priv->pltrstn_in_avail = false;
				break;
			}

			if (signal_pending(current)) {
				ret = -ERESTARTSYS;
			} else {
				spin_unlock_irq(&priv->pltrstn_lock);
				schedule();
				spin_lock_irq(&priv->pltrstn_lock);
			}
		} while (!ret);

		remove_wait_queue(&priv->pltrstn_waitq, &wait);
		set_current_state(TASK_RUNNING);
	}
out_unlock:
	spin_unlock_irq(&priv->pltrstn_lock);

	if (ret)
		return ret;

	ret = put_user(data, buf);
	if (!ret)
		ret = sizeof(data);

	return ret;
}

static unsigned int npcm_espi_slave_pltrstn_poll(struct file *file,
						 poll_table *wait)
{
	struct npcm_espi_slave *priv = to_npcm_espi_slave(file);
	unsigned int mask = 0;

	poll_wait(file, &priv->pltrstn_waitq, wait);
	if (priv->pltrstn_in_avail)
		mask |= POLLIN;
	return mask;
}

static const struct file_operations npcm_espi_slave_pltrstn_fops = {
	.owner	= THIS_MODULE,
	.open	= npcm_espi_slave_pltrstn_open,
	.read	= npcm_espi_slave_pltrstn_read,
	.poll	= npcm_espi_slave_pltrstn_poll,
};

static int npcm_espi_slave_probe(struct platform_device *pdev)
{
	struct device *dev = &pdev->dev;
	struct npcm_espi_slave *priv;
	int ret;

	priv = devm_kzalloc(dev, sizeof(struct npcm_espi_slave), GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->espi_regmap = syscon_node_to_regmap(dev->parent->of_node);
	if (IS_ERR(priv->espi_regmap)) {
		dev_err(dev, "Couldn't get espiregmap\n");
		return -ENODEV;
	}

	priv->miscdev.fops = &npcm_espi_slave_pltrstn_fops;
	priv->miscdev.parent = &pdev->dev;
	priv->miscdev.minor = MISC_DYNAMIC_MINOR;
	priv->miscdev.name = devm_kasprintf(dev, GFP_KERNEL, "%s", DEVICE_NAME);
	if (!priv->miscdev.name) {
		ret = -ENOMEM;
		return ret;
	}

	priv->irq = platform_get_irq(pdev, 0);
	if (priv->irq < 0)
		return priv->irq;

	spin_lock_init(&priv->pltrstn_lock);
	init_waitqueue_head(&priv->pltrstn_waitq);

	priv->pltrstn = 'U'; /* means it's not reported yet from master */
	priv->dev =  &pdev->dev;

	dev_set_drvdata(dev, priv);

	/*
	 * Drop what the bootloader left latched, but only for the status this
	 * driver owns.  A latched CFGUPD is what makes the interrupt fire once
	 * spi-npcm-flash enables it, and both drivers still need that first
	 * event: one to set the flash channel up, the other to learn the
	 * PLTRST_N level it reports as 'U' until then.
	 */
	regmap_write(priv->espi_regmap, NPCM_ESPI_ESPISTS,
		     NPCM_ESPI_IRQ_SLAVE);

	ret = devm_request_irq(dev, priv->irq, npcm_espi_slave_irq_handler,
			       IRQF_SHARED, "espi-slave", priv);
	if (ret) {
		dev_err(dev, "failed to request IRQ\n");
		return ret;
	}

	ret = misc_register(&priv->miscdev);
	if (ret) {
		dev_err(dev, "Unable to register device\n");
		return ret;
	}

	npcm_espi_slave_config_irq(priv, !IS_ENABLED(CONFIG_SPI_NPCM_FLASH));

	return 0;
}

static void npcm_espi_slave_remove(struct platform_device *pdev)
{
	struct npcm_espi_slave *priv = dev_get_drvdata(&pdev->dev);

	misc_deregister(&priv->miscdev);
}

static const struct of_device_id npcm_espi_slave_match[] = {
	{ .compatible = "nuvoton,npcm845-espi-slave" },
	{ }
};
MODULE_DEVICE_TABLE(of, npcm_espi_slave_match);

static struct platform_driver npcm_espi_slave_driver = {
	.driver	= {
		.name           = DEVICE_NAME,
		.owner		= THIS_MODULE,
		.of_match_table	= of_match_ptr(npcm_espi_slave_match),
	},
	.probe		= npcm_espi_slave_probe,
	.remove = npcm_espi_slave_remove,
};
module_platform_driver(npcm_espi_slave_driver);

MODULE_AUTHOR("Joseph Liu <kwliu@nuvoton.com>");
MODULE_AUTHOR("Ban Feng <kcfeng0@nuvoton.com>");
MODULE_DESCRIPTION("NPCM eSPI Slave driver");
MODULE_LICENSE("GPL v2");
