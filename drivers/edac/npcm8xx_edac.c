// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2019 Quanta Computer lnc.
 */

#include <linux/edac.h>
#include <linux/interrupt.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#include <linux/of_address.h>
#include <linux/of_device.h>

#include "edac_module.h"
#define PRIu64 "llu"

#define ECC_EVENT_ADDR_MASK            0x3FFFFFFFFLL
#define ECC_EN_INT_MASTER_MASK         0x7FFFFFFB //arbel ECC:bit1
#define ECC_EN_INT_ECC_MASK            0x1F0 //arbel ECC:bit3-0

#define INT_STATUS_ADDR                138//0x228 arbel 15-0 3:MU 2:AnU 1:MC 0:AC  135
#define INT_ACK_ADDR                   145//arbel 244
#define INT_MASK_MASTER_ADDR           136//arbel 136
#define INT_MASK_ECC_ADDR              152//arbel

#define ECC_EN_ADDR                    91 //arbel

#define ECC_C_LOWADDR_ADDR             99 //arbel 31-0
#define ECC_C_HIGHADDR_ADDR            100//arbel 1-0 0x3
#define ECC_C_LOWDATA_ADDR             101//arbel 31-0
#define ECC_C_HIGHDATA_ADDR            102//arbel 31-0
#define ECC_C_ID_ADDR                  103//arbel 29-16 0x3FFF
#define ECC_C_SYND_ADDR                100//arbel 15-8 0xFF

#define ECC_U_LOWADDR_ADDR             95 //arbel 31-0
#define ECC_U_HIGHADDR_ADDR            96 //arbrl 1-0 0x3
#define ECC_U_LOWDATA_ADDR             97 //arbel 31-0
#define ECC_U_HIGHDATA_ADDR            98 //arbel 31-0
#define ECC_U_ID_ADDR                  103//arbel 13-0 0x3FFF
#define ECC_U_SYND_ADDR                96 //arbel 15-8 0xFF

#define ECC_ERROR                      -1
#define EDAC_MSG_SIZE                  256
#define EDAC_MOD_NAME                  "npcm8xx-edac"

static struct class *edac_class = NULL;
static dev_t edac_devno;
struct device *edac_dev = NULL;
static unsigned int ecc_value=0;
static int ecc_ue = 0;

struct ecc_error_signature_info {
	u64 ecc_addr;
	u64 ecc_data;
	u32 ecc_id;
	u32 ecc_synd;
};

struct npcm8xx_ecc_int_status {
	u32 int_mask;
	u32 int_status;
	u32 int_ack;
	u32 ce_cnt;
	u32 ue_cnt;
	struct ecc_error_signature_info ceinfo;
	struct ecc_error_signature_info ueinfo;
};

struct npcm8xx_edac_priv {
	void __iomem *baseaddr;
	char message[EDAC_MSG_SIZE];
	struct npcm8xx_ecc_int_status stat;
};

/**
 * npcm8xx_edac_get_ecc_syndrom - Get the current ecc error info
 * @base:	Pointer to the base address of the ddr memory controller
 * @p:		Pointer to the Nuvoton ecc status structure
 *
 * Determines there is any ecc error or not
 *
 * Return: ECC detection status
 */
static int npcm8xx_edac_get_ecc_syndrom(void __iomem *base,
					struct npcm8xx_ecc_int_status *p)
{
	int status = 0;
	u32 int_status = 0;

	int_status = readl(base + 4*INT_STATUS_ADDR);
	writel(int_status, base + 4*INT_ACK_ADDR);
	edac_dbg(3, "int_status: %#08x\n", int_status);

	if ((int_status & (1 << 3)) == (1 << 3)) {
		edac_dbg(3, "3-Mult uncorrectable detected.\n");
		p->ue_cnt++;
		status = ECC_ERROR;
	}

	if ((int_status & (1 << 2)) == (1 << 2)) {
		edac_dbg(3, "2-An uncorrectable detected.\n");
		p->ue_cnt++;
		status = ECC_ERROR;
	}

	if ((int_status & (1 << 1)) == (1 << 1)) {
		edac_dbg(3, "1-mult correctable detected.\n");
		p->ce_cnt++;
		status = ECC_ERROR;
	}

	if ((int_status & (1 << 0)) == (1 << 0)) {
		edac_dbg(3, "0-A correctable detected.\n");
		p->ce_cnt++;
		status = ECC_ERROR;
	}

	if (status == ECC_ERROR) {
		u32 ecc_id;

		p->ceinfo.ecc_addr = readl(base + 4*ECC_C_LOWADDR_ADDR) | ((readl(base + 4*ECC_C_HIGHADDR_ADDR)&0x3) <<32);
		p->ceinfo.ecc_data = readl(base + 4*ECC_C_LOWDATA_ADDR) | (readl(base + 4*ECC_C_HIGHDATA_ADDR)<<32);
		p->ceinfo.ecc_synd = (readl(base + 4*ECC_C_SYND_ADDR)>>8) & 0xFF;

		p->ueinfo.ecc_addr = readl(base + 4*ECC_U_LOWADDR_ADDR) | ((readl(base + 4*ECC_U_HIGHADDR_ADDR)&0x3) <<32);
		p->ueinfo.ecc_data = readl(base + 4*ECC_U_LOWDATA_ADDR) | (readl(base + 4*ECC_U_HIGHDATA_ADDR)<<32);
		p->ueinfo.ecc_synd = (readl(base + 4*ECC_U_SYND_ADDR)>>8) & 0xFF;

		/* ECC_C_ID_ADDR has same value as ECC_U_ID_ADDR */
		ecc_id = readl(base + 4*ECC_C_ID_ADDR);
		p->ueinfo.ecc_id = ecc_id & 0x3FFF;
		p->ceinfo.ecc_id = (ecc_id >> 16) & 0x3FFF;
	}

	return status;
}

/**
 * npcm8xx_edac_handle_error - Handle controller error types CE and UE
 * @mci:	Pointer to the edac memory controller instance
 * @p:		Pointer to the Nuvoton ecc status structure
 *
 * Handles the controller ECC correctable and un correctable error.
 */
static void npcm8xx_edac_handle_error(struct mem_ctl_info *mci,
				    struct npcm8xx_ecc_int_status *p)
{
	struct npcm8xx_edac_priv *priv = mci->pvt_info;
	u32 page, offset;

	if (p->ce_cnt) {
		snprintf(priv->message, EDAC_MSG_SIZE,
			"DDR ECC: synd=%#08x addr=0x%" PRIu64 " data=0x%" PRIu64 " source_id=%#08x ",
			p->ceinfo.ecc_synd, p->ceinfo.ecc_addr&ECC_EVENT_ADDR_MASK,
			p->ceinfo.ecc_data, p->ceinfo.ecc_id);

		page = (p->ceinfo.ecc_addr&ECC_EVENT_ADDR_MASK) >> PAGE_SHIFT;
		offset = (p->ceinfo.ecc_addr&ECC_EVENT_ADDR_MASK) & ~PAGE_MASK;
		edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci,
				     p->ce_cnt, page, offset,
				     p->ceinfo.ecc_synd,
				     0, 0, -1,
				     priv->message, "");
	}

	if (p->ue_cnt) {
		snprintf(priv->message, EDAC_MSG_SIZE,
			"DDR ECC: synd=%#08x addr=0x%" PRIu64 " data=0x%" PRIu64 " source_id=%#08x ",
			p->ueinfo.ecc_synd, p->ueinfo.ecc_addr&ECC_EVENT_ADDR_MASK,
			p->ueinfo.ecc_data, p->ueinfo.ecc_id);

		page = (p->ueinfo.ecc_addr&ECC_EVENT_ADDR_MASK) >> PAGE_SHIFT;
		offset = (p->ueinfo.ecc_addr&ECC_EVENT_ADDR_MASK) & ~PAGE_MASK;
		edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci,
				     p->ue_cnt, page, offset,
				     p->ueinfo.ecc_synd,
				     0, 0, -1,
				     priv->message, "");
	}

	memset(p, 0, sizeof(*p));
}

/**
 * npcm8xx_edac_check - Check controller for ECC errors
 * @mci:	Pointer to the edac memory controller instance
 *
 * This routine is used to check and post ECC errors and is called by
 * this driver's CE and UE interrupt handler.
 */
static void npcm8xx_edac_check(struct mem_ctl_info *mci)
{
	struct npcm8xx_edac_priv *priv = mci->pvt_info;
	int status = 0;

	status = npcm8xx_edac_get_ecc_syndrom(priv->baseaddr, &priv->stat);
	if (status != ECC_ERROR)
		return;

	npcm8xx_edac_handle_error(mci, &priv->stat);
}

/**
 * npcm8xx_edac_isr - CE/UE interrupt service routine
 * @irq:    The virtual interrupt number being serviced.
 * @dev_id: A pointer to the EDAC memory controller instance
 *          associated with the interrupt being handled.
 *
 * This routine implements the interrupt handler for both correctable
 * (CE) and uncorrectable (UE) ECC errors for the Nuvoton Cadence DDR
 * controller. It simply calls through to the routine used to check,
 * report and clear the ECC status.
 *
 * Unconditionally returns IRQ_HANDLED.
 */
static irqreturn_t npcm8xx_edac_isr(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	npcm8xx_edac_check(mci);

	return IRQ_HANDLED;
}

static int npcm8xx_edac_register_irq(struct mem_ctl_info *mci,
					struct platform_device *pdev)
{
	int status = 0;
	int mc_irq;
	struct npcm8xx_edac_priv *priv = mci->pvt_info;

	/* Only enable MC interrupts with ECC - clear int_mask_master[1] */
	writel(ECC_EN_INT_MASTER_MASK, priv->baseaddr + 4*INT_MASK_MASTER_ADDR);

	/* Only enable ECC event - clear int_mask_ecc[3:0] */
	writel(ECC_EN_INT_ECC_MASK, priv->baseaddr + 4*INT_MASK_ECC_ADDR);

	mc_irq = platform_get_irq(pdev, 0);

	if (!mc_irq) {
		edac_printk(KERN_ERR, EDAC_MC, "Unable to map interrupts.\n");
		status = -ENODEV;
		goto fail;
	}

	edac_printk(KERN_ERR, EDAC_MC,
		      "request irq %d for EDAC",
		      mc_irq);

	status = devm_request_irq(&pdev->dev, mc_irq, npcm8xx_edac_isr, 0,
			       "npcm-memory-controller", mci);

	if (status < 0) {
		edac_printk(KERN_ERR, EDAC_MC,
				      "Unable to request irq %d for ECC",
				      mc_irq);
		status = -ENODEV;
		goto fail;
	}

	return 0;


fail:
	return status;
}

static const struct of_device_id npcm8xx_edac_of_match[] = {
	{ .compatible = "nuvoton,npcm8xx-sdram-edac"},
	{ /* end of table */ }
};

MODULE_DEVICE_TABLE(of, npcm8xx_edac_of_match);

/**
 * npcm8xx_edac_mc_init - Initialize driver instance
 * @mci:	Pointer to the edac memory controller instance
 * @pdev:	Pointer to the platform_device struct
 *
 * Performs initialization of the EDAC memory controller instance and
 * related driver-private data associated with the memory controller the
 * instance is bound to.
 *
 * Returns 0 if OK; otherwise, < 0 on error.
 */
static int npcm8xx_edac_mc_init(struct mem_ctl_info *mci,
				 struct platform_device *pdev)
{
	const struct of_device_id *id;

	id = of_match_device(npcm8xx_edac_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

	/* Initialize controller capabilities and configuration */
	mci->mtype_cap = MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_SECDED;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->scrub_cap = SCRUB_FLAG_HW_SRC;
	mci->scrub_mode = SCRUB_HW_SRC;
	mci->ctl_name = id->compatible;
	mci->dev_name = dev_name(&pdev->dev);
	mci->mod_name = EDAC_MOD_NAME;

	edac_op_state = EDAC_OPSTATE_INT;

	return 0;
}

/**
 * npcm8xx_edac_get_eccstate - Return the controller ecc enable/disable status
 * @base:	Pointer to the ddr memory controller base address
 *
 * Get the ECC enable/disable status for the controller
 *
 * Return: a ecc status boolean i.e true/false - enabled/disabled.
 */
static bool npcm8xx_edac_get_eccstate(void __iomem *base)
{
	u32 ecc_en;
	bool state = false;

	ecc_en = readl(base + 4*ECC_EN_ADDR);
	if ((ecc_en&0x30000) == 0x30000) {
		edac_printk(KERN_INFO, EDAC_MC, "ECC reporting and correcting on. ");
		state = true;
	}

	return state;
}

static ssize_t edac_read(struct device *dev, struct device_attribute *attr, char *buf)
{
	int ret = 0;
	void __iomem *baseaddr = dev_get_drvdata(dev);

	if( ecc_ue == 1 )
	{
		ecc_ue = 0;
		ecc_value = 0x030100;
		writel(ecc_value, baseaddr + 0x174);
		sprintf(buf, "0x%X\n",ecc_value);
		ret = strlen(buf) + 1;
	}
	else
	{
		ecc_ue = 1;
		ecc_value = 0xF40100;
		writel(ecc_value, baseaddr + 0x174);
		sprintf(buf, "0x%X\n",ecc_value);
		ret = strlen(buf) + 1;
	}

	return ret;
}
static ssize_t edac_write(struct device *dev, struct device_attribute *attr, const char *buf, size_t size)
{
	void __iomem *baseaddr = dev_get_drvdata(dev);
	ecc_value = simple_strtoul(buf, NULL, 0);
	printk("syndrome:0x%x\n",ecc_value);
	writel(ecc_value, baseaddr + 0x174);
	return size;
}
static DEVICE_ATTR(edac, 0664 , edac_read, edac_write);

/**
 * npcm8xx_edac_mc_probe - Check controller and bind driver
 * @pdev:	Pointer to the platform_device struct
 *
 * Probes a specific controller instance for binding with the driver.
 *
 * Return: 0 if the controller instance was successfully bound to the
 * driver; otherwise, < 0 on error.
 */
static int npcm8xx_edac_mc_probe(struct platform_device *pdev)
{
	struct mem_ctl_info *mci;
	struct edac_mc_layer layers[1];
	struct npcm8xx_edac_priv *priv;
	struct resource *res;
	void __iomem *baseaddr;
	int rc;

	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	baseaddr = devm_ioremap_resource(&pdev->dev, res);
	if (IS_ERR(baseaddr)) {
		edac_printk(KERN_ERR, EDAC_MOD_NAME,
			    "DDR controller regs not defined\n");
		return PTR_ERR(baseaddr);
	}

	/*
	 * Check if ECC is enabled.
	 * If not, there is no useful monitoring that can be done
	 * for this controller.
	 */
	if (!npcm8xx_edac_get_eccstate(baseaddr)) {
		edac_printk(KERN_INFO, EDAC_MC, "ECC disabled\n");
		return -ENXIO;
	}

	/*
	 * Allocate an EDA controller instance and perform the appropriate
	 * initialization.
	 */
	layers[0].type = EDAC_MC_LAYER_ALL_MEM;
	layers[0].size = 1;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct npcm8xx_edac_priv));
	if (!mci) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed memory allocation for mc instance\n");
		return -ENOMEM;
	}

	mci->pdev = &pdev->dev;
	priv = mci->pvt_info;
	priv->baseaddr = baseaddr;
	platform_set_drvdata(pdev, mci);

	rc = npcm8xx_edac_mc_init(mci, pdev);
	if (rc) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed to initialize instance\n");
		goto free_edac_mc;
	}

	/* Attempt to register it with the EDAC subsystem */
	rc = edac_mc_add_mc(mci);
	if (rc) {
		edac_printk(KERN_ERR, EDAC_MC,
			    "Failed to register with EDAC core\n");
		goto free_edac_mc;
	}

	/* Register interrupts */
	rc = npcm8xx_edac_register_irq(mci, pdev);
	if (rc)
		goto free_edac_mc;

	if( alloc_chrdev_region(&edac_devno, 0, 1,"edac") )
	{
		return -EAGAIN;
	}
	edac_class = class_create(THIS_MODULE, "edac");
	edac_dev = device_create(edac_class, NULL, edac_devno, NULL, "edac_dev");
	if(NULL == edac_dev)
	{
		return -EIO;
	}
	device_create_file(edac_dev, &dev_attr_edac);
	dev_set_drvdata(edac_dev, baseaddr);
	return 0;

free_edac_mc:
	edac_mc_free(mci);

	return rc;
}

/**
 * npcm8xx_edac_mc_remove - Unbind driver from controller
 * @pdev:	Pointer to the platform_device struct
 *
 * Return: Unconditionally 0
 */
static int npcm8xx_edac_mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);

	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);


	return 0;
}

static struct platform_driver npcm8xx_edac_driver = {
	.probe = npcm8xx_edac_mc_probe,
	.remove = npcm8xx_edac_mc_remove,
	.driver = {
		   .name = EDAC_MOD_NAME,
		   .of_match_table = npcm8xx_edac_of_match,
	},
};

module_platform_driver(npcm8xx_edac_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("Nuvoton npcm8xx EDAC Driver");
MODULE_LICENSE("GPL v2");
