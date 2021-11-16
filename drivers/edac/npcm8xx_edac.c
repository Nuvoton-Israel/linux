// SPDX-License-Identifier: GPL-2.0
/*
 * NPCM8XX Cadence DDR4-ECC EDAC Driver.
 *
 * Copyright (C) 2019-2020 Cadence Design Systems.
 */

#include <linux/init.h>
#include <linux/edac.h>
#include <linux/io.h>
#include <linux/interrupt.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/module.h>
#include <linux/uaccess.h>
#include <linux/delay.h>
#include "edac_module.h"


#define CDNS_EDAC_MOD_NAME "cadence-npcm8xx-edac"
#define FORCED_ECC_ERR_EVENT_SUPPORT	BIT(1)
//#define CONFIG_EDAC_DEBUG
#define EDAC_MSG_SIZE                  256
/* Granularity of reported error in bytes */
#define CDNS_EDAC_ERR_GRAIN		1

#define MEM_TYPE_DDR4			0xA
/* CDNS DDR4 Controller Registers */
#define DDR_CTL_MEM_TYPE_REG		0x000
#define DDR_CTL_MEM_WIDTH_REG		0x00c
#define DDR_CTL_CONTROLLER_BUSY		0x20C


/* ECC Controller Registers */
#define ECC_CTL_EN_ADDR				0x16C

#define ECC_CTL_CONTROL_93_REG		0x174

#define ECC_SIG_ECC_C_ADDR_L		0x18C
#define ECC_SIG_ECC_C_ADDR_H		0x190
#define ECC_SIG_ECC_C_DATA_L		0x194
#define ECC_SIG_ECC_C_DATA_H		0x198
#define ECC_SIG_ECC_C_ID		0x19C
#define ECC_SIG_ECC_C_SYND		0x190

#define ECC_SIG_ECC_U_ADDR_L		0x17C
#define ECC_SIG_ECC_U_ADDR_H		0x180
#define ECC_SIG_ECC_U_DATA_L		0x184
#define ECC_SIG_ECC_U_DATA_H		0x188
#define ECC_SIG_ECC_U_ID		0x19C
#define ECC_SIG_ECC_U_SYND		0x180

#define ECC_CTL_INT_MASK		0x350

#define ECC_CTL_INT_STATUS			0x228
#define ECC_CTL_INT_ACK				0x244
#define ECC_CTL_INT_MASK_MASTER		0x220
#define ECC_CTL_INT_MASK_ECC		0x260

/* Control register width definitions */
#define WDTH_16				(2)
#define WDTH_32				(1)
#define WDTH_64				(0)
#define CTL_REG_WIDTH_SHIFT		(32)
#define XOR_CHECK_BIT_SPLIT_WIDTH		(16)
#define CTL_CONTROLLER_BUSY_FLAG	BIT(0)
#define CTL_MEM_MAX_WIDTH_MASK		GENMASK(4, 0)

/* ECC Control Macros */
#define ECC_CTL_FORCE_WC		BIT(8)
#define ECC_CTL_AUTO_WRITEBACK_EN	BIT(24)
#define ECC_CTL_ECC_ENABLE		GENMASK(17, 16)
#define ECC_CTL_MTYPE_MASK		GENMASK(11, 8)
#define ECC_CTL_XOR_BITS_MASK		GENMASK(23, 16)
#define ECC_CTL_EN_INT_MASTER_MASK	(GENMASK(30, 3) | GENMASK(1, 0))
#define ECC_CTL_EN_INT_ECC_MASK		GENMASK(8, 4)


/* ECC IRQ Macros */
#define ECC_INT_CE_EVENT		BIT(0)
#define ECC_INT_SECOND_CE_EVENT		BIT(1)
#define ECC_INT_UE_EVENT		BIT(2)
#define ECC_INT_SECOND_UE_EVENT		BIT(3)
#define ECC_INT_WRITEBACK_UNHANDLED	BIT(6)
#define ECC_INT_SCRUB_DONE		BIT(7)
#define ECC_INT_SCRUB_CE_EVENT		BIT(8)
#define ECC_INT_MASK_ALL_H		BIT(8)
#define ECC_INT_CE_UE_MASK		GENMASK(3, 0)
#define ECC_CE_INTR_MASK		GENMASK(1, 0)
#define ECC_UE_INTR_MASK		GENMASK(3, 2)
#define ECC_CTL_INT_ENABLE_MASK		GENMASK(15, 0)
/* ECC Signature Macros */
#define ECC_SIG_ECC_C_ID_SHIFT		8
#define ECC_SIG_ECC_C_SYND_SHIFT	8
#define ECC_SIG_ECC_C_ADDR_H_MASK	GENMASK(1, 0)
#define ECC_SIG_ECC_C_ID_MASK		GENMASK(29, 16)
#define ECC_SIG_ECC_C_SYND_MASK		GENMASK(15, 8)

#define ECC_SIG_ECC_U_ID_SHIFT		0
#define ECC_SIG_ECC_U_SYND_SHIFT	8
#define ECC_SIG_ECC_U_ADDR_H_MASK	GENMASK(1, 0)
#define ECC_SIG_ECC_U_ID_MASK		GENMASK(13, 0)
#define ECC_SIG_ECC_U_SYND_MASK		GENMASK(15, 8)

/* Syndrome values */
#define ECC_DOUBLE_MULTI_ERR_SYND	0x03

static char data_synd[] = {
			0xf4, 0xf1, 0xec, 0xea, 0xe9, 0xe6, 0xe5, 0xe3,
			0xdc, 0xda, 0xd9, 0xd6, 0xd5, 0xd3, 0xce, 0xcb,
			0xb5, 0xb0, 0xad, 0xab, 0xa8, 0xa7, 0xa4, 0xa2,
			0x9d, 0x9b, 0x98, 0x97, 0x94, 0x92, 0x8f, 0x8a,
			0x75, 0x70, 0x6d, 0x6b, 0x68, 0x67, 0x64, 0x62,
			0x5e, 0x5b, 0x58, 0x57, 0x54, 0x52, 0x4f, 0x4a,
			0x34, 0x31, 0x2c, 0x2a, 0x29, 0x26, 0x25, 0x23,
			0x1c, 0x1a, 0x19, 0x16, 0x15, 0x13, 0x0e, 0x0b
		  };

static char check_synd[] = {0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80};

/**
 * struct cdns_platform_data -  cadence platform data structure.
 * @ip_features:	To differentiate IP features.
 */
struct cdns_platform_data {
	int ip_features;
};

/**
 * struct priv_data - edac device private instance data.
 * @reg:	        Base address of the DDR controller.
 * @ce_cnt:             Correctable Error count.
 * @ue_cnt:             Uncorrectable Error count.
 */
struct priv_data {
	void __iomem *reg;
	u32 ce_cnt;
	u32 ue_cnt;
	char message[EDAC_MSG_SIZE];
#ifdef CONFIG_EDAC_DEBUG
	struct cdns_platform_data *p;
#endif
};

/**
 * init_mem_layout -  Set address Map as per the mc design.
 * @mci:   memory controller information.
 *
 * Set Address Map as per mc instance .
 *
 * Return: none.
 */

static void init_mem_layout(struct mem_ctl_info *mci)
{
	struct priv_data *priv = mci->pvt_info;
	struct csrow_info *csi;
	struct dimm_info *dimm;
	struct sysinfo inf;
	enum mem_type mtype;
	u32 val, width;
	u32 size, row;
	u8 j;

	dimm = edac_get_dimm(mci, 0, 0, 0);
	if (!dimm) {
		si_meminfo(&inf);
		for (row = 0; row < mci->nr_csrows; row++) {
			csi = mci->csrows[row];
			size = inf.totalram * inf.mem_unit;

			for (j = 0; j < csi->nr_channels; j++) {
				dimm            = csi->channels[j]->dimm;
				dimm->edac_mode = EDAC_FLAG_SECDED;
				/* Get memory type by reading hw registers*/
				val = readl(priv->reg + DDR_CTL_MEM_TYPE_REG);
				mtype = val & ECC_CTL_MTYPE_MASK;

				if (mtype == MEM_TYPE_DDR4)
					dimm->mtype = MEM_DDR4;
				else
					dimm->mtype = MEM_EMPTY;

				/*Get EDAC devtype width for the current mc*/
				width = readl(priv->reg + DDR_CTL_MEM_WIDTH_REG)
					      & CTL_MEM_MAX_WIDTH_MASK;
				switch (width) {
				case WDTH_16:
					dimm->dtype  = DEV_X2;
					break;
				case WDTH_32:
					dimm->dtype  = DEV_X4;
					break;
				case WDTH_64:
					dimm->dtype  = DEV_X8;
					break;
				default:
					dimm->dtype = DEV_UNKNOWN;
				}

				dimm->nr_pages  = (size >> PAGE_SHIFT) /
					csi->nr_channels;
				dimm->grain     = CDNS_EDAC_ERR_GRAIN;
			}
		}
	}
}

/**
 * handle_ce - Handle corrected errors and notify to
 *             edac layer by reading appropriate hw registers
 * @mci:   memory controller information.
 * Return: void.
 */
static void handle_ce(struct mem_ctl_info *mci)
{
	struct priv_data *priv;
	u64 err_c_addr = 0x0;
	u64 err_c_data = 0x0;
	u32 err_c_synd, err_c_id;
	u32 sig_val_l, sig_val_h;

	priv = mci->pvt_info;

	sig_val_l = readl(priv->reg + ECC_SIG_ECC_C_ADDR_L);
	sig_val_h = (readl(priv->reg + ECC_SIG_ECC_C_ADDR_H) &
			ECC_SIG_ECC_C_ADDR_H_MASK);
	err_c_addr = (((err_c_addr | sig_val_h) <<
				CTL_REG_WIDTH_SHIFT) | sig_val_l);

	sig_val_l = readl(priv->reg + ECC_SIG_ECC_C_DATA_L);
	sig_val_h = readl(priv->reg + ECC_SIG_ECC_C_DATA_H);
	err_c_data = (((err_c_data | sig_val_h) <<
				CTL_REG_WIDTH_SHIFT) | sig_val_l);

	err_c_id = ((readl(priv->reg + ECC_SIG_ECC_C_ID) &
				ECC_SIG_ECC_C_ID_MASK) >>
			ECC_SIG_ECC_C_ID_SHIFT);

	err_c_synd = ((readl(priv->reg + ECC_SIG_ECC_C_SYND) &
				ECC_SIG_ECC_C_SYND_MASK) >>
			ECC_SIG_ECC_C_SYND_SHIFT);

	priv->ce_cnt = priv->ce_cnt + 1;

	snprintf(priv->message, EDAC_MSG_SIZE,
			"DDR ECC %s: addr=0x%llx data=0x%llx source_id=%#08x",
			mci->ctl_name, err_c_addr, err_c_data, err_c_id);

	edac_mc_handle_error(HW_EVENT_ERR_CORRECTED, mci,
			     1,
			     err_c_addr >> PAGE_SHIFT,
			     err_c_addr & ~PAGE_MASK,
			     err_c_synd, 0, 0, -1,
			     priv->message, "");
}

/**
 * handle_ue -	Handle uncorrected errors and notify to
 *		edac layer by reading appropriate hw registers
 * @mci:   memory controller information.
 * Return: void.
 */
static void handle_ue(struct mem_ctl_info *mci)
{
	struct priv_data *priv;
	u64 err_u_addr = 0x0;
	u64 err_u_data = 0x0;
	u32 err_u_synd, err_u_id;
	u32 sig_val_l, sig_val_h;

	priv = mci->pvt_info;
	sig_val_l = readl(priv->reg + ECC_SIG_ECC_U_ADDR_L);
	sig_val_h = (readl(priv->reg + ECC_SIG_ECC_U_ADDR_H) &
			ECC_SIG_ECC_U_ADDR_H_MASK);
	err_u_addr = (((err_u_addr | sig_val_h) <<
				CTL_REG_WIDTH_SHIFT) | sig_val_l);

	sig_val_l = readl(priv->reg + ECC_SIG_ECC_U_DATA_L);
	sig_val_h = readl(priv->reg + ECC_SIG_ECC_U_DATA_H);
	err_u_data = (((err_u_data | sig_val_h) <<
				CTL_REG_WIDTH_SHIFT) | sig_val_l);

	err_u_id = ((readl(priv->reg + ECC_SIG_ECC_U_ID) &
				ECC_SIG_ECC_U_ID_MASK) >>
			ECC_SIG_ECC_U_ID_SHIFT);

	err_u_synd = ((readl(priv->reg + ECC_SIG_ECC_U_SYND) &
				ECC_SIG_ECC_U_SYND_MASK) >>
			ECC_SIG_ECC_U_SYND_SHIFT);
	priv->ue_cnt = priv->ue_cnt + 1;

	snprintf(priv->message, EDAC_MSG_SIZE,
			"DDR ECC %s: addr=0x%llx data=0x%llx source_id=%#08x",
			mci->ctl_name, err_u_addr, err_u_data, err_u_id);

	edac_mc_handle_error(HW_EVENT_ERR_UNCORRECTED, mci,
			     1,
			     err_u_addr >> PAGE_SHIFT,
			     err_u_addr & ~PAGE_MASK,
			     err_u_synd, 0, 0, -1,
			     priv->message, "");
}

/**
 * edac_ecc_isr - Interrupt Handler for ECC interrupts.
 * @irq:        IRQ number.
 * @dev_id:     Device ID.
 *
 * Return: IRQ_NONE, if interrupt not set or IRQ_HANDLED otherwise.
 */
static irqreturn_t edac_ecc_isr(int irq, void *dev_id)
{
	struct mem_ctl_info *mci = dev_id;
	struct priv_data *priv;
	u32 intr_status;
	u32 val;

	priv = mci->pvt_info;

	/* Check the intr status and confirm ECC error intr */
	intr_status = readl(priv->reg + ECC_CTL_INT_STATUS);

	edac_dbg(3, "InterruptStatus : 0x%x\n", intr_status);
	val = intr_status & (ECC_INT_CE_UE_MASK);
	if (!((val & ECC_CE_INTR_MASK) || (val & ECC_UE_INTR_MASK)))
		return IRQ_NONE;

	if (val & ECC_CE_INTR_MASK) {
		handle_ce(mci);

		/* Clear the interrupt source */
		if (val & ECC_INT_CE_EVENT)
			writel(ECC_INT_CE_EVENT, priv->reg + ECC_CTL_INT_ACK);
		else if (val & ECC_INT_SECOND_CE_EVENT)
			writel(ECC_INT_SECOND_CE_EVENT,
			       priv->reg + ECC_CTL_INT_ACK);
		else
			edac_printk(KERN_ERR, EDAC_MC, "Failed to clear IRQ\n");
	}

	if (val & ECC_UE_INTR_MASK) {
		handle_ue(mci);

		/* Clear the interrupt source */
		if (val & ECC_INT_UE_EVENT)
			writel(ECC_INT_UE_EVENT, priv->reg + ECC_CTL_INT_ACK);
		else if (val & ECC_INT_SECOND_UE_EVENT)
			writel(ECC_INT_SECOND_UE_EVENT,
			       priv->reg + ECC_CTL_INT_ACK);
		else
			edac_printk(KERN_ERR, EDAC_MC, "Failed to clear IRQ\n");
	}

	edac_dbg(3, "Total error count CE %d UE %d\n",
		 priv->ce_cnt, priv->ue_cnt);

	return IRQ_HANDLED;
}

#ifdef CONFIG_EDAC_DEBUG
/**
 * forced_ecc_error_show/store - Sysfs atrribute functions.
 * @dev: Pointer to device structure.
 * @mattr: Pointer to device attributes.
 * @data : Data send by User space and stored in file.
 * Return: as SUCCESS,Total number of characters written otherwise
 *         negative value.
 */
static ssize_t forced_ecc_error_show(struct device *dev,
				     struct device_attribute *mattr,
				     char *data)
{
	return sprintf(data, "CDNS-DDR4 Force Injection Help:\n"
		       "CE: Corrected\n"
		       "checkcode/data:source\n"
		       "bit [0-63] for data [0-7] for checkcode:bit number\n"
		       "--------------------------------------------------\n"
		       "UE: Uncorrected\n");
}

static ssize_t forced_ecc_error_store(struct device *dev,
				      struct device_attribute *mattr,
				      const char *data, size_t count)
{
	struct mem_ctl_info *mci = to_mci(dev);
	struct priv_data *priv = mci->pvt_info;
	int	args_cnt;
	int	ret;
	char	**args;
	u32	regval;
	u8	bit_no;

	/* Split string buffer into separate parameters */
	args = argv_split(GFP_KERNEL, data, &args_cnt);

	/* Check ecc enabled */
	if (!readl(priv->reg + ECC_CTL_EN_ADDR) & ECC_CTL_ECC_ENABLE)
		return count;

	/* Check no write operation pending to controller*/
	while (readl(priv->reg + DDR_CTL_CONTROLLER_BUSY) &
			CTL_CONTROLLER_BUSY_FLAG) {
		usleep_range(1000, 10000);
	}

	/* Write appropriate syndrome to xor_check_bit*/
	if (!strcmp(args[0], "CE") && args_cnt == 3 ) {
		ret = kstrtou8(args[2], 0, &bit_no);
		if (ret)
			return ret;
		if (!strcmp(args[1], "checkcode")) {
			if(bit_no>7) {
				printk("bit_no for checkcode must be 0~7\n");
				return count;
			}
			regval = readl(priv->reg + ECC_CTL_CONTROL_93_REG);
			regval = (regval & ~(ECC_CTL_XOR_BITS_MASK)) |
				(check_synd[bit_no] << XOR_CHECK_BIT_SPLIT_WIDTH );
			writel(regval, priv->reg + ECC_CTL_CONTROL_93_REG);
		} else if (!strcmp(args[1], "data")) {
			if(bit_no>63) {
				printk("bit_no for data must be 0~63\n");
				return count;
			}
			regval = readl(priv->reg + ECC_CTL_CONTROL_93_REG);
			regval = (regval & ~(ECC_CTL_XOR_BITS_MASK)) |
				(data_synd[bit_no] << XOR_CHECK_BIT_SPLIT_WIDTH );
			writel(regval, priv->reg + ECC_CTL_CONTROL_93_REG);
		}
		/* Enable the ECC writeback_en for corrected error */
		regval = readl(priv->reg + ECC_CTL_CONTROL_93_REG);
		writel((regval | ECC_CTL_AUTO_WRITEBACK_EN),
		       priv->reg + ECC_CTL_CONTROL_93_REG);
	} else if (!strcmp(args[0], "UE")) {
		regval = readl(priv->reg + ECC_CTL_CONTROL_93_REG);
		regval = (regval & ~(ECC_CTL_XOR_BITS_MASK)) |
			(ECC_DOUBLE_MULTI_ERR_SYND << XOR_CHECK_BIT_SPLIT_WIDTH );
		writel(regval, priv->reg + ECC_CTL_CONTROL_93_REG);
	}

	/* Assert fwc */
	writel((ECC_CTL_FORCE_WC | readl(priv->reg + ECC_CTL_CONTROL_93_REG)),
	       priv->reg + ECC_CTL_CONTROL_93_REG);

	return count;
}

static DEVICE_ATTR_RW(forced_ecc_error);
static int create_sysfs_attributes(struct mem_ctl_info *mci)
{
	int rc;

	rc = device_create_file(&mci->dev, &dev_attr_forced_ecc_error);
	if (rc < 0)
		return rc;
	return 0;
}

static void remove_sysfs_attributes(struct mem_ctl_info *mci)
{
	device_remove_file(&mci->dev, &dev_attr_forced_ecc_error);
}

#endif

static const struct cdns_platform_data cdns_edac = {
#ifdef CONFIG_EDAC_DEBUG
	.ip_features = FORCED_ECC_ERR_EVENT_SUPPORT,
#endif
};

static const struct of_device_id cdns_edac_of_match[] = {
	{ .compatible = "nuvoton,npcm8xx-sdram-edac", .data = &cdns_edac },
	{},
};

MODULE_DEVICE_TABLE(of, cdns_edac_of_match);

/**
 * cdns_edac_mc_probe - bind cdns mc controller driver to framework.
 * @pdev:  platform device.
 *
 * Probe a memory controller for binding with the driver.
 *
 * Return: 0 if binding of the controller instance is successful;
 * otherwise, < 0 on error.
 */
static int cdns_edac_mc_probe(struct platform_device *pdev)
{
#ifdef CONFIG_EDAC_DEBUG
	const struct cdns_platform_data *p;
#endif
	struct device *dev = &pdev->dev;
	struct edac_mc_layer layers[1];
	const struct of_device_id *id;
	struct priv_data *priv_data;
	struct mem_ctl_info *mci;
	struct resource *res;
	void __iomem *reg;
	int ret = -ENODEV;
	int irq;

	id = of_match_device(cdns_edac_of_match, &pdev->dev);
	if (!id)
		return -ENODEV;

#ifdef CONFIG_EDAC_DEBUG
	p = of_device_get_match_data(&pdev->dev);
	if (!p)
		return -ENODEV;
#endif
	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	reg = devm_ioremap_resource(dev, res);
	if (IS_ERR(reg)) {
		edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
			    "cdns DDR4 mc regs are not defined\n");
		return PTR_ERR(reg);
	}
	else {
		/*
		* Check if ECC is enabled.
		* If not, there is no useful monitoring that can be done
		* for this controller.
		*/
		u32 ecc_en = readl(reg + ECC_CTL_EN_ADDR);
		if ((ecc_en&ECC_CTL_ECC_ENABLE) == ECC_CTL_ECC_ENABLE) {
			edac_printk(KERN_INFO, EDAC_MC, "ECC reporting and correcting on. ");
		} else {
			edac_printk(KERN_INFO, EDAC_MC, "ECC disabled\n");
			return -ENXIO;
		}
	}

	edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
		    "IO mapped reg addr: %p\n", reg);
	layers[0].type = EDAC_MC_LAYER_ALL_MEM;//EDAC_MC_LAYER_CHIP_SELECT;
	layers[0].size = 1;
	//layers[0].is_virt_csrow = true;

	mci = edac_mc_alloc(0, ARRAY_SIZE(layers), layers,
			    sizeof(struct priv_data));
	if (!mci) {
		edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
			    "Failed memory allocation for mc instance\n");
		return -ENOMEM;
	}
	mci->pdev = &pdev->dev;
	priv_data = mci->pvt_info;
	priv_data->reg = reg;
	priv_data->ce_cnt = 0;
	priv_data->ue_cnt = 0;
	platform_set_drvdata(pdev, mci);

	/* Initialize controller capabilities */
	mci->mtype_cap = MEM_FLAG_DDR4;
	mci->edac_ctl_cap = EDAC_FLAG_SECDED;
	mci->scrub_cap = SCRUB_FLAG_HW_SRC;
	mci->scrub_mode = SCRUB_HW_SRC;
	mci->edac_cap = EDAC_FLAG_SECDED;
	mci->ctl_name = id->compatible;
	mci->dev_name = dev_name(&pdev->dev);
	mci->mod_name = CDNS_EDAC_MOD_NAME;
	mci->ctl_page_to_phys = NULL;

	/* Interrupt feature is supported by cadence mc */
	edac_op_state = EDAC_OPSTATE_INT;
	init_mem_layout(mci);

	/* Setup Interrupt handler for ECC */
	irq = platform_get_irq(pdev, 0);
	if (!irq) {
		edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
			    "irq number not defined for ECC.\n");
		goto err;
	}
	ret = devm_request_irq(dev, irq, edac_ecc_isr, 0,
			       "cdns-edac-mc-ecc-irq", mci);
	if (ret) {
		edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
			    "request_irq fail for CDNS_EDAC irq\n");
		goto err;
	}
	ret = edac_mc_add_mc(mci);
	if (ret) {
		edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
			    "Failed to register with EDAC core\n");
		goto err;
	}

#ifdef CONFIG_EDAC_DEBUG
	if (p->ip_features & FORCED_ECC_ERR_EVENT_SUPPORT) {
		if (create_sysfs_attributes(mci)) {
			edac_printk(KERN_ERR, CDNS_EDAC_MOD_NAME,
				    "Failed to create sysfs entries\n");
			goto err1;
		}
	}
#endif
	/* Only enable MC interrupts with ECC - clear int_mask_master[1] */
	writel(ECC_CTL_EN_INT_MASTER_MASK, priv_data->reg + ECC_CTL_INT_MASK_MASTER);

	/* Only enable ECC event - clear int_mask_ecc[3:0] */
	writel(ECC_CTL_EN_INT_ECC_MASK, priv_data->reg + ECC_CTL_INT_MASK);

	return 0;

err1:
	edac_mc_del_mc(&pdev->dev);

err:
	edac_mc_free(mci);
	return ret;
}

/**
 * cdns_edac_mc_remove - Unbind driver from controller.
 * @pdev:  Platform device.
 *
 * Return: 0
 */
static int cdns_edac_mc_remove(struct platform_device *pdev)
{
	struct mem_ctl_info *mci = platform_get_drvdata(pdev);
	struct priv_data *priv = mci->pvt_info;

	/* Disable All ECC Interrupts for DDR4 Controller */
	writel(ECC_CTL_INT_ENABLE_MASK, priv->reg + ECC_CTL_INT_MASK);

	/* Disable ecc feature before removing driver by writing 0 */
	writel((unsigned int)(~(ECC_CTL_ECC_ENABLE)),
	       priv->reg + ECC_CTL_EN_ADDR);

#ifdef CONFIG_EDAC_DEBUG
	remove_sysfs_attributes(mci);
#endif
	edac_mc_del_mc(&pdev->dev);
	edac_mc_free(mci);

	return 0;
}

static struct platform_driver cdns_edac_mc_driver = {
	.driver = {
		   .name = "cadence-edac",
		   .of_match_table = cdns_edac_of_match,
	},
	.probe = cdns_edac_mc_probe,
	.remove = cdns_edac_mc_remove,
};

module_platform_driver(cdns_edac_mc_driver);

MODULE_AUTHOR("Nuvoton Technology Corporation");
MODULE_DESCRIPTION("Nuvoton npcm8xx EDAC Driver");
MODULE_LICENSE("GPL v2");