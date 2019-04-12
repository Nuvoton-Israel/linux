// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Nuvoton Technology corporation.
// Copyright (C) 2004-2007,2011 Freescale Semiconductor, Inc.

#include <linux/module.h>
#include <linux/kernel.h>
#include <linux/ioport.h>
#include <linux/types.h>
#include <linux/errno.h>
#include <linux/err.h>
#include <linux/delay.h>
#include <linux/sched.h>
#include <linux/slab.h>
#include <linux/init.h>
#include <linux/timer.h>
#include <linux/list.h>
#include <linux/interrupt.h>
#include <linux/proc_fs.h>
#include <linux/mm.h>
#include <linux/moduleparam.h>
#include <linux/device.h>
#include <linux/usb/ch9.h>
#include <linux/usb/gadget.h>
#include <linux/usb/otg.h>
#include <linux/dma-mapping.h>
#include <linux/platform_device.h>
#include <linux/dmapool.h>
#include <linux/delay.h>
#include <linux/of_device.h>
#include <linux/regmap.h>
#include <linux/mfd/syscon.h>


#include <asm/byteorder.h>
#include <asm/io.h>
#include <asm/irq.h>
#include <asm/unaligned.h>
#include <asm/dma.h>
#include <asm/cacheflush.h>

static struct regmap *gcr_regmap;

#define  INTCR3_OFFSET 0x9C
#define  NPCM_INTCR3_USBPHYSW	GENMASK(13, 12)

#define  USB1PHYCTL_OFFSET 0x140
#define  USB2PHYCTL_OFFSET 0x144

static struct regmap *rst_regmap;

#define  IPSRST1_OFFSET 0x20
#define  IPSRST2_OFFSET 0x24
#define  IPSRST3_OFFSET 0x34

//#include <mach/map.h>
//#include <mach/irqs.h>
//#include <mach/hal.h>
//#include <mach/module_init.h>

//#include <mach/regs_npcm750_gcr.h>
//#include <mach/regs_npcm750_clk.h>

#include "npcm_udc.h"

#define MINIMUM_NPCM_UDC_EPQ_DTD_SIZE                0x800

#undef VERBOSE

//#define USB_DEVICE_9_WA

#define	DRIVER_DESC	"Nuvoton High-Speed USB SOC Device Controller driver"
#define	DRIVER_VERSION	"Apr 25, 2014"

#define	DMA_ADDR_INVALID	(~(dma_addr_t)0)

static const char drv_20_name[] = "npcm-udc";
#ifndef CONFIG_OF
static const char driver_desc[] = DRIVER_DESC;
#endif

struct npcm_usb2_platform_data usb_data = {
        .operating_mode = NPCM_USB2_DR_DEVICE,
        .phy_mode = /*NPCM_USB2_PHY_SERIAL*/NPCM_USB2_PHY_UTMI_WIDE,
};

#if 0
#ifndef CONFIG_ARCH_MXC
static struct usb_sys_interface *usb_sys_regs;
#endif
#endif

static const struct usb_endpoint_descriptor
npcm_ep0_desc = {
	.bLength =		USB_DT_ENDPOINT_SIZE,
	.bDescriptorType =	USB_DT_ENDPOINT,
	.bEndpointAddress =	0,
	.bmAttributes =		USB_ENDPOINT_XFER_CONTROL,
	.wMaxPacketSize =	USB_MAX_CTRL_PAYLOAD,
};

static void npcm_ep_fifo_flush(struct usb_ep *_ep);
static void npcm_udc_release(struct device *dev);

#ifdef CONFIG_PPC32
/*
 * On some SoCs, the USB controller registers can be big or little endian,
 * depending on the version of the chip. In order to be able to run the
 * same kernel binary on 2 different versions of an SoC, the BE/LE decision
 * must be made at run time. _npcm_readl and npcm_writel are pointers to the
 * BE or LE readl() and writel() functions, and npcm_readl() and npcm_writel()
 * call through those pointers. Platform code for SoCs that have BE USB
 * registers should set pdata->big_endian_mmio flag.
 *
 * This also applies to controller-to-cpu accessors for the USB descriptors,
 * since their endianness is also SoC dependant. Platform code for SoCs that
 * have BE USB descriptors should set pdata->big_endian_desc flag.
 */
static u32 _npcm_readl_be(const unsigned __iomem *p)
{
	return in_be32(p);
}

static u32 _npcm_readl_le(const unsigned __iomem *p)
{
	return in_le32(p);
}

static void _npcm_writel_be(u32 v, unsigned __iomem *p)
{
	out_be32(p, v);
}

static void _npcm_writel_le(u32 v, unsigned __iomem *p)
{
	out_le32(p, v);
}

static u32 (*_npcm_readl)(const unsigned __iomem *p);
static void (*_npcm_writel)(u32 v, unsigned __iomem *p);

#define npcm_readl(p)		(*_npcm_readl)((p))
#define npcm_writel(v, p)	(*_npcm_writel)((v), (p))

static inline void npcm_set_accessors(struct npcm_usb2_platform_data *pdata)
{
	if (pdata->big_endian_mmio) {
		_npcm_readl = _npcm_readl_be;
		_npcm_writel = _npcm_writel_be;
	} else {
		_npcm_readl = _npcm_readl_le;
		_npcm_writel = _npcm_writel_le;
	}
}

static inline u32 cpu_to_hc32(const u32 x)
{
	return udc_controller->pdata->big_endian_desc
		? (__force u32)cpu_to_be32(x)
		: (__force u32)cpu_to_le32(x);
}

static inline u32 hc32_to_cpu(const u32 x)
{
	return udc_controller->pdata->big_endian_desc
		? be32_to_cpu((__force __be32)x)
		: le32_to_cpu((__force __le32)x);
}
#else /* !CONFIG_PPC32 */
static inline void npcm_set_accessors(struct npcm_usb2_platform_data *pdata) {}

#define npcm_readl(addr)		readl(addr)
#define npcm_writel(val32, addr) writel(val32, addr)
#define cpu_to_hc32(x)		cpu_to_le32(x)
#define hc32_to_cpu(x)		le32_to_cpu(x)
#endif /* CONFIG_PPC32 */

/********************************************************************
 *	Internal Used Function
********************************************************************/
/*-----------------------------------------------------------------
 * done() - retire a request; caller blocked irqs
 * @status : request status to be set, only works when
 *	request is still in progress.
 *--------------------------------------------------------------*/
static void done(struct npcm_ep *ep, struct npcm_req *req, int status)
__releases(ep->udc->lock)
__acquires(ep->udc->lock)
{
	unsigned char stopped = ep->stopped;
	struct ep_td_struct *curr_td, *next_td;
	int j;

	/* Removed the req from npcm_ep->queue */
	list_del_init(&req->queue);

	/* req.status should be set as -EINPROGRESS in ep_queue() */
	if (req->req.status == -EINPROGRESS)
		req->req.status = status;
	else
		status = req->req.status;

	/* Free dtd for the request */
	next_td = req->head;
	for (j = 0; j < req->dtd_count; j++) {
		curr_td = next_td;
		if (j != req->dtd_count - 1) {
			next_td = curr_td->next_td_virt;
		}
#ifdef NPCM_USB_DESC_PHYS_BASE_ADDR
		curr_td->res = DTD_IS_FREE; // curr_td is free
#else
		dma_pool_free(ep->udc->td_pool, curr_td, curr_td->td_dma);
#endif
	}

	usb_gadget_unmap_request(&ep->udc->gadget, &req->req, ep_is_in(ep));

	if (status && (status != -ESHUTDOWN))
		VDBG("complete %s req %p stat %d len %u/%u",
			ep->ep.name, &req->req, status,
			req->req.actual, req->req.length);

	ep->stopped = 1;

	spin_unlock(&ep->udc->lock);
	
	if (req->req.complete) 
		usb_gadget_giveback_request(&ep->ep, &req->req);

	spin_lock(&ep->udc->lock);
	ep->stopped = stopped;
}

/*-----------------------------------------------------------------
 * nuke(): delete all requests related to this ep
 * called with spinlock held
 *--------------------------------------------------------------*/
static void nuke(struct npcm_ep *ep, int status)
{
	ep->stopped = 1;

	/* Flush fifo */
	npcm_ep_fifo_flush(&ep->ep);

	/* Whether this eq has request linked */
	while (!list_empty(&ep->queue)) {
		struct npcm_req *req = NULL;

		req = list_entry(ep->queue.next, struct npcm_req, queue);
		done(ep, req, status);
	}
}

/*------------------------------------------------------------------
	Internal Hardware related function
 ------------------------------------------------------------------*/
static void dr_controller_stop(struct npcm_udc *udc);


static int dr_controller_setup(struct npcm_udc *udc)
{
	unsigned int tmp, portctrl, ep_num;
	unsigned int max_no_of_ep;
//	unsigned int ctrl;
	unsigned long timeout;
	struct usb_dr_device *dr_regs = udc->dr_regs;

#define NPCM_UDC_RESET_TIMEOUT 1000

	/* Config PHY interface */
	portctrl = npcm_readl(&dr_regs->portsc1);
	portctrl &= ~(PORTSCX_PHY_TYPE_SEL | PORTSCX_PORT_WIDTH);
	switch (udc->phy_mode) {
	case NPCM_USB2_PHY_ULPI:
		if (udc->pdata->have_sysif_regs) {
			if (udc->pdata->controller_ver) {
				/* controller version 1.6 or above */
				/*ctrl = __raw_readl(&usb_sys_regs->control);
				ctrl &= ~USB_CTRL_UTMI_PHY_EN;
				ctrl |= USB_CTRL_USB_EN;
				__raw_writel(ctrl, &usb_sys_regs->control);*/
				printk(KERN_INFO "udc debug: NPCM_USB2_PHY_ULPI\n");
			}
		}
		portctrl |= PORTSCX_PTS_ULPI;
		break;
	case NPCM_USB2_PHY_UTMI_WIDE:
		portctrl |= PORTSCX_PTW_16BIT;
		/* fall through */
	case NPCM_USB2_PHY_UTMI:
	case NPCM_USB2_PHY_UTMI_DUAL:
		if (udc->pdata->have_sysif_regs) {
			if (udc->pdata->controller_ver) {
				/* controller version 1.6 or above */
				/*ctrl = __raw_readl(&usb_sys_regs->control);
				ctrl |= (USB_CTRL_UTMI_PHY_EN |
					USB_CTRL_USB_EN);
				__raw_writel(ctrl, &usb_sys_regs->control);*/
				/*mdelay(npcm_UTMI_PHY_DLY);*/ /* Delay for UTMI
					PHY CLK to become stable - 10ms*/
				printk(KERN_INFO "udc debug: NPCM_USB2_PHY_UTMI\n");
			}
		}
		portctrl |= PORTSCX_PTS_UTMI;
		break;
	case NPCM_USB2_PHY_SERIAL:
		portctrl |= PORTSCX_PTS_FSLS;
		break;
	default:
		return -EINVAL;
	}
	npcm_writel(portctrl, &dr_regs->portsc1);

	/* Stop and reset the usb controller */
    dr_controller_stop(udc);

	tmp = npcm_readl(&dr_regs->usbcmd);
	tmp |= USB_CMD_CTRL_RESET;
	npcm_writel(tmp, &dr_regs->usbcmd);

	/* Wait for reset to complete */
	timeout = jiffies + NPCM_UDC_RESET_TIMEOUT;
	while (npcm_readl(&dr_regs->usbcmd) & USB_CMD_CTRL_RESET) {
		if (time_after(jiffies, timeout)) {
			NPCM_USB_ERR("udc reset timeout!\n");
			return -ETIMEDOUT;
		}
		cpu_relax();
	}

	/* Set the controller as device mode */
	tmp = npcm_readl(&dr_regs->usbmode);
	tmp &= ~USB_MODE_RESERVED_BITS_MASK;	/* Must clear reserved bits */
	tmp &= ~USB_MODE_CTRL_MODE_MASK;	/* clear mode bits */
	tmp |= USB_MODE_CTRL_MODE_DEVICE;
	/* Disable Setup Lockout */
	tmp |= USB_MODE_SETUP_LOCK_OFF;
	if (udc->pdata->es)
		tmp |= USB_MODE_ES;
	npcm_writel(tmp, &dr_regs->usbmode);

	/* Clear the setup status */
	npcm_writel(0, &dr_regs->usbsts);

	tmp = udc->ep_qh_dma;
	tmp &= USB_EP_LIST_ADDRESS_MASK;
	npcm_writel(tmp, &dr_regs->endpointlistaddr);

	VDBG("vir[qh_base] is %p phy[qh_base] is 0x%8x reg is 0x%8x",
		udc->ep_qh, (int)tmp,
		npcm_readl(&dr_regs->endpointlistaddr));

	max_no_of_ep = (0x0000001F & npcm_readl(&dr_regs->dccparams));
	for (ep_num = 1; ep_num < max_no_of_ep; ep_num++) {
		tmp = npcm_readl(&dr_regs->endptctrl[ep_num]);
		tmp &= ~(EPCTRL_TX_TYPE | EPCTRL_RX_TYPE);
		tmp |= (EPCTRL_EP_TYPE_BULK << EPCTRL_TX_EP_TYPE_SHIFT)
		| (EPCTRL_EP_TYPE_BULK << EPCTRL_RX_EP_TYPE_SHIFT);
		npcm_writel(tmp, &dr_regs->endptctrl[ep_num]);
	}
	
	return 0;
}

/* Enable DR irq and set controller to run state */
static void dr_controller_run(struct npcm_udc *udc)
{
    u32 temp;
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return;
    
    dr_regs = udc->dr_regs;

    /* Enable DR irq reg */
    temp = USB_INTR_INT_EN | USB_INTR_ERR_INT_EN
        | USB_INTR_PTC_DETECT_EN | USB_INTR_RESET_EN
        | USB_INTR_DEVICE_SUSPEND | USB_INTR_SYS_ERR_EN;

    npcm_writel(temp, &dr_regs->usbintr);

    /* Clear stopped bit */
    udc->stopped = 0;

    /* Set the controller as device mode */
    temp = npcm_readl(&dr_regs->usbmode);
    temp |= USB_MODE_CTRL_MODE_DEVICE;
    //temp |= USB_MODE_STREAM_DISABLE;
    npcm_writel(temp, &dr_regs->usbmode);

    /* Set controller to Run */
    temp = npcm_readl(&dr_regs->usbcmd);
    temp |= USB_CMD_RUN_STOP;
    npcm_writel(temp, &dr_regs->usbcmd);

    return;
}

static void dr_controller_stop(struct npcm_udc *udc)
{
    unsigned int tmp;
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return;

    dr_regs = udc->dr_regs;

    /* disable all INTR */
    npcm_writel(0, &dr_regs->usbintr);

    /* Set stopped bit for isr */
    udc->stopped = 1;

    /* disable IO output */
/*  usb_sys_regs->control = 0; */

    /* set controller to Stop */
    tmp = npcm_readl(&dr_regs->usbcmd);
    tmp &= ~USB_CMD_RUN_STOP;
    npcm_writel(tmp, &dr_regs->usbcmd);

    return;
}

static void dr_ep_setup(struct npcm_udc *udc, unsigned char ep_num, unsigned char dir, unsigned char ep_type)
{
    unsigned int tmp_epctrl = 0;
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return;

    dr_regs = udc->dr_regs;

    tmp_epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);
	if (dir) {
		if (ep_num)
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_TX_ENABLE;
		tmp_epctrl &= ~EPCTRL_TX_TYPE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_TX_EP_TYPE_SHIFT);
	} else {
		if (ep_num)
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		tmp_epctrl |= EPCTRL_RX_ENABLE;
		tmp_epctrl &= ~EPCTRL_RX_TYPE;
		tmp_epctrl |= ((unsigned int)(ep_type)
				<< EPCTRL_RX_EP_TYPE_SHIFT);
	}

    npcm_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

static void
dr_ep_change_stall(struct npcm_udc *udc, unsigned char ep_num, unsigned char dir, int value)
{
	u32 tmp_epctrl = 0;
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return;
    
    dr_regs = udc->dr_regs;

	tmp_epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);

	if (value) {
		/* set the stall bit */
		if (dir)
			tmp_epctrl |= EPCTRL_TX_EP_STALL;
		else
			tmp_epctrl |= EPCTRL_RX_EP_STALL;
	} else {
		/* clear the stall bit and reset data toggle */
		if (dir) {
			tmp_epctrl &= ~EPCTRL_TX_EP_STALL;
			tmp_epctrl |= EPCTRL_TX_DATA_TOGGLE_RST;
		} else {
			tmp_epctrl &= ~EPCTRL_RX_EP_STALL;
			tmp_epctrl |= EPCTRL_RX_DATA_TOGGLE_RST;
		}
	}
	npcm_writel(tmp_epctrl, &dr_regs->endptctrl[ep_num]);
}

/* Get stall status of a specific ep
   Return: 0: not stalled; 1:stalled */
static int dr_ep_get_stall(struct npcm_udc *udc, unsigned char ep_num, unsigned char dir)
{
	u32 epctrl;
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return -EINVAL;

    dr_regs = udc->dr_regs;

	epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);
	if (dir)
		return (epctrl & EPCTRL_TX_EP_STALL) ? 1 : 0;
	else
		return (epctrl & EPCTRL_RX_EP_STALL) ? 1 : 0;
}

/********************************************************************
	Internal Structure Build up functions
********************************************************************/

/*------------------------------------------------------------------
* struct_ep_qh_setup(): set the Endpoint Capabilites field of QH
 * @zlt: Zero Length Termination Select (1: disable; 0: enable)
 * @mult: Mult field
 ------------------------------------------------------------------*/
static void struct_ep_qh_setup(struct npcm_udc *udc, unsigned char ep_num,
		unsigned char dir, unsigned char ep_type,
		unsigned int max_pkt_len,
		unsigned int zlt, unsigned char mult)
{
	struct ep_queue_head *p_QH = &udc->ep_qh[2 * ep_num + dir];
	unsigned int tmp = 0;

	/* set the Endpoint Capabilites in QH */
	switch (ep_type) {
	case USB_ENDPOINT_XFER_CONTROL:
		/* Interrupt On Setup (IOS). for control ep  */
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| EP_QUEUE_HEAD_IOS;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		tmp = (max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS)
			| (mult << EP_QUEUE_HEAD_MULT_POS);
		break;
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		tmp = max_pkt_len << EP_QUEUE_HEAD_MAX_PKT_LEN_POS;
		break;
	default:
		pr_info("%s(): error ep type is %d\n", __func__, ep_type);
		return;
	}
	if (zlt)
		tmp |= EP_QUEUE_HEAD_ZLT_SEL;

	p_QH->max_pkt_length = cpu_to_hc32(tmp);
	p_QH->next_dtd_ptr = 1;
	p_QH->size_ioc_int_sts = 0;
}

/* Setup qh structure and ep register for ep0. */
static void ep0_setup(struct npcm_udc *udc)
{
	/* the initialization of an ep includes: fields in QH, Regs,
	 * npcm_ep struct */
	struct_ep_qh_setup(udc, 0, USB_RECV, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	struct_ep_qh_setup(udc, 0, USB_SEND, USB_ENDPOINT_XFER_CONTROL,
			USB_MAX_CTRL_PAYLOAD, 0, 0);
	dr_ep_setup(udc,0, USB_RECV, USB_ENDPOINT_XFER_CONTROL);
	dr_ep_setup(udc,0, USB_SEND, USB_ENDPOINT_XFER_CONTROL);

	return;

}

/***********************************************************************
		Endpoint Management Functions
***********************************************************************/

/*-------------------------------------------------------------------------
 * when configurations are set, or when interface settings change
 * for example the do_set_interface() in gadget layer,
 * the driver will enable or disable the relevant endpoints
 * ep0 doesn't use this routine. It is always enabled.
-------------------------------------------------------------------------*/
static int npcm_ep_enable(struct usb_ep *_ep,
		const struct usb_endpoint_descriptor *desc)
{
	struct npcm_udc *udc = NULL;
	struct npcm_ep *ep = NULL;
	unsigned short max = 0;
	unsigned char mult = 0, zlt;
	int retval = -EINVAL;
	unsigned long flags = 0;

	ep = container_of(_ep, struct npcm_ep, ep);

	/* catch various bogus parameters */
	if (!_ep || !desc
			|| (desc->bDescriptorType != USB_DT_ENDPOINT))
		return -EINVAL;

	udc = ep->udc;

	if (!udc->driver || (udc->gadget.speed == USB_SPEED_UNKNOWN))
		return -ESHUTDOWN;

	max = usb_endpoint_maxp(desc);

	/* Disable automatic zlp generation.  Driver is responsible to indicate
	 * explicitly through req->req.zero.  This is needed to enable multi-td
	 * request. */
	zlt = 1;

	/* Assume the max packet size from gadget is always correct */
	switch (desc->bmAttributes & 0x03) {
	case USB_ENDPOINT_XFER_CONTROL:
	case USB_ENDPOINT_XFER_BULK:
	case USB_ENDPOINT_XFER_INT:
		/* mult = 0.  Execute N Transactions as demonstrated by
		 * the USB variable length packet protocol where N is
		 * computed using the Maximum Packet Length (dQH) and
		 * the Total Bytes field (dTD) */
		mult = 0;
		break;
	case USB_ENDPOINT_XFER_ISOC:
		/* Calculate transactions needed for high bandwidth iso */
		mult = usb_endpoint_maxp_mult(desc);
		/* 3 transactions at most */
		if (mult > 3)
			goto en_done;
		break;
	}

	spin_lock_irqsave(&udc->lock, flags);
	ep->ep.maxpacket = max;
	ep->ep.desc = desc;
	ep->stopped = 0;

	/* Controller related setup */
	/* Init EPx Queue Head (Ep Capabilites field in QH
	 * according to max, zlt, mult) */
	struct_ep_qh_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					?  USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK),
			max, zlt, mult);

	/* Init endpoint ctrl register */
	dr_ep_setup(udc, (unsigned char) ep_index(ep),
			(unsigned char) ((desc->bEndpointAddress & USB_DIR_IN)
					? USB_SEND : USB_RECV),
			(unsigned char) (desc->bmAttributes
					& USB_ENDPOINT_XFERTYPE_MASK));

	spin_unlock_irqrestore(&udc->lock, flags);
	retval = 0;

	VDBG("enabled %s (ep%d%s) maxpacket %d",ep->ep.name,
			ep->ep.desc->bEndpointAddress & 0x0f,
			(desc->bEndpointAddress & USB_DIR_IN)
				? "in" : "out", max);
en_done:
	return retval;
}

/*---------------------------------------------------------------------
 * @ep : the ep being unconfigured. May not be ep0
 * Any pending and uncomplete req will complete with status (-ESHUTDOWN)
*---------------------------------------------------------------------*/
static int npcm_ep_disable(struct usb_ep *_ep)
{
	struct npcm_udc *udc = NULL;
	struct npcm_ep *ep = NULL;
	unsigned long flags = 0;
	u32 epctrl;
	int ep_num;
    struct usb_dr_device *dr_regs;

	ep = container_of(_ep, struct npcm_ep, ep);
	if (!_ep || !ep->ep.desc) {
		VDBG("%s not enabled", _ep ? ep->ep.name : NULL);
		return -EINVAL;
	}

    udc = (struct npcm_udc *)ep->udc;

   /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return -EINVAL;

    dr_regs = udc->dr_regs;

	/* disable ep on controller */
	ep_num = ep_index(ep);
	epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep)) {
		epctrl &= ~(EPCTRL_TX_ENABLE | EPCTRL_TX_TYPE);
		epctrl |= EPCTRL_EP_TYPE_BULK << EPCTRL_TX_EP_TYPE_SHIFT;
	} else {
		epctrl &= ~(EPCTRL_RX_ENABLE | EPCTRL_TX_TYPE);
		epctrl |= EPCTRL_EP_TYPE_BULK << EPCTRL_RX_EP_TYPE_SHIFT;
	}
	npcm_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	udc = (struct npcm_udc *)ep->udc;
	spin_lock_irqsave(&udc->lock, flags);

	/* nuke all pending requests (does flush) */
	nuke(ep, -ESHUTDOWN);

	ep->ep.desc = NULL;
	ep->stopped = 1;
	spin_unlock_irqrestore(&udc->lock, flags);

	VDBG("disabled %s OK", _ep->name);
	return 0;
}


/*---------------------------------------------------------------------
 * allocate a request object used by this endpoint
 * the main operation is to insert the req->queue to the eq->queue
 * Returns the request, or null if one could not be allocated
*---------------------------------------------------------------------*/
static struct usb_request *
npcm_alloc_request(struct usb_ep *_ep, gfp_t gfp_flags)
{
	struct npcm_req *req = NULL;

	req = kzalloc(sizeof(struct npcm_req), gfp_flags);
	if (!req)
		return NULL;

	req->req.dma = DMA_ADDR_INVALID;
	INIT_LIST_HEAD(&req->queue);

	return &req->req;
}

static void npcm_free_request(struct usb_ep *_ep, struct usb_request *_req)
{
	struct npcm_req *req = NULL;

	req = container_of(_req, struct npcm_req, req);

	if (_req)
		kfree(req);
}

/* Actually add a dTD chain to an empty dQH and let go */
static void npcm_prime_ep(struct npcm_ep *ep, struct ep_td_struct *td)
{
	struct ep_queue_head *qh = get_qh_by_ep(ep);
    struct usb_dr_device *dr_regs;

  /* before here, make sure dr_regs has been initialized */
    if (!ep->udc)
        return;

    dr_regs = ep->udc->dr_regs;

	/* Write dQH next pointer and terminate bit to 0 */
	qh->next_dtd_ptr = cpu_to_hc32(td->td_dma
			& EP_QUEUE_HEAD_NEXT_POINTER_MASK);

	/* Clear active and halt bit */
	qh->size_ioc_int_sts &= cpu_to_hc32(~(EP_QUEUE_HEAD_STATUS_ACTIVE
					| EP_QUEUE_HEAD_STATUS_HALT));

	/* Ensure that updates to the QH will occur before priming. */
	wmb();
	/* We add the read from qh->size_ioc_int_sts to make sure the previous
	   write to it indeed got into the mamory so when we prime the DMA
	   will read the updated data */
	if (qh->size_ioc_int_sts & 0x80000000)
		NPCM_USB_ERR("%s(): qh->size_ioc_int_sts=%08x\n", __func__, qh->size_ioc_int_sts);

	/* Prime endpoint by writing correct bit to ENDPTPRIME */
	npcm_writel(ep_is_in(ep) ? (1 << (ep_index(ep) + 16))
			: (1 << (ep_index(ep))), &dr_regs->endpointprime);
}

/* Add dTD chain to the dQH of an EP */
static int npcm_queue_td(struct npcm_ep *ep, struct npcm_req *req)
{
	u32 temp, bitmask, tmp_stat;
    struct usb_dr_device *dr_regs;
	unsigned int loops;
	int retval = 0;
	
	/* VDBG("QH addr Register 0x%8x", dr_regs->endpointlistaddr);
	VDBG("ep_qh[%d] addr is 0x%8x", i, (u32)&(ep->udc->ep_qh[i])); */



  /* before here, make sure dr_regs has been initialized */
    if (!ep->udc)
        return -EINVAL;

    dr_regs = ep->udc->dr_regs;

	bitmask = ep_is_in(ep)
		? (1 << (ep_index(ep) + 16))
		: (1 << (ep_index(ep)));

	/* check if the pipe is empty */
	if (!(list_empty(&ep->queue)) && !(ep_index(ep) == 0)) {
		/* Add td to the end */
		struct npcm_req *lastreq;
		lastreq = list_entry(ep->queue.prev, struct npcm_req, queue);
		lastreq->tail->next_td_ptr =
			cpu_to_hc32(req->head->td_dma & DTD_ADDR_MASK);
		/* Ensure dTD's next dtd pointer to be updated */
		wmb();
		/* Read prime bit, if 1 goto done */
		if (npcm_readl(&dr_regs->endpointprime) & bitmask)
			goto done;

		loops = 1000;
		while (1) {
			/* Set ATDTW bit in USBCMD */
			temp = npcm_readl(&dr_regs->usbcmd);
			npcm_writel(temp | USB_CMD_ATDTW, &dr_regs->usbcmd);

			/* Read correct status bit */
			tmp_stat = npcm_readl(&dr_regs->endptstatus) & bitmask;

			/*
			 * Reread the ATDTW semaphore bit to check if it is
			 * cleared. When hardware see a hazard, it will clear
			 * the bit or else we remain set to 1 and we can
			 * proceed with priming of endpoint if not already
			 * primed.
			 */
			if (npcm_readl(&dr_regs->usbcmd) & USB_CMD_ATDTW)
				break;

			loops--;
			if (loops == 0) {
				NPCM_USB_ERR("Timeout for ATDTW_TRIPWIRE...\n");
				retval = -ETIME;
				goto done;
			}
			udelay(1);
		}

		/* Write ATDTW bit to 0 */
		temp = npcm_readl(&dr_regs->usbcmd);
		npcm_writel(temp & ~USB_CMD_ATDTW, &dr_regs->usbcmd);

		if (tmp_stat)
			goto done;
	}

	npcm_prime_ep(ep, req->head);

done:
	return retval;
}

/* Fill in the dTD structure
 * @req: request that the transfer belongs to
 * @length: return actually data length of the dTD
 * @dma: return dma address of the dTD
 * @is_last: return flag if it is the last dTD of the request
 * return: pointer to the built dTD */
static struct ep_td_struct *npcm_build_dtd(struct npcm_req *req, unsigned *length,
		dma_addr_t *dma, int *is_last, gfp_t gfp_flags)
{
	u32 swap_temp;
	struct ep_td_struct *dtd;
    struct npcm_udc *udc = req->ep->udc;

	/* how big will this transfer be? */
	*length = min(req->req.length - req->req.actual,
			(unsigned)EP_MAX_LENGTH_TRANSFER);

#ifdef NPCM_USB_DESC_PHYS_BASE_ADDR
    {
        int td_count;
        
        for (td_count=0; td_count < udc->dtd_max_pool; td_count++)
        {
            dtd = (void *)(udc->dtd_virt_ba + 2*DTD_ALIGNMENT * td_count);
            if (dtd->res == DTD_IS_FREE)
            {
                dtd->res = DTD_IS_IN_USE;
                *dma = (dma_addr_t)(udc->dtd_phys_ba + 2*DTD_ALIGNMENT * td_count);
                break;
            }            
        }
        if (td_count == udc->dtd_max_pool)
            dtd = NULL;
    }
#else
    dtd = dma_pool_alloc(udc->td_pool, gfp_flags, dma);
#endif

	if (dtd == NULL)
		return dtd;

	dtd->td_dma = *dma;
	/* Clear reserved field */
	swap_temp = hc32_to_cpu(dtd->size_ioc_sts);
	swap_temp &= ~DTD_RESERVED_FIELDS;
	dtd->size_ioc_sts = cpu_to_hc32(swap_temp);

	/* Init all of buffer page pointers */
	swap_temp = (u32) (req->req.dma + req->req.actual);
	dtd->buff_ptr0 = cpu_to_hc32(swap_temp);
	dtd->buff_ptr1 = cpu_to_hc32(swap_temp + 0x1000);
	dtd->buff_ptr2 = cpu_to_hc32(swap_temp + 0x2000);
	dtd->buff_ptr3 = cpu_to_hc32(swap_temp + 0x3000);
	dtd->buff_ptr4 = cpu_to_hc32(swap_temp + 0x4000);

	req->req.actual += *length;

	/* zlp is needed if req->req.zero is set */
	if (req->req.zero) {
		if (*length == 0 || (*length % req->ep->ep.maxpacket) != 0)
			*is_last = 1;
		else
			*is_last = 0;
	} else if (req->req.length == req->req.actual)
		*is_last = 1;
	else
		*is_last = 0;

	if ((*is_last) == 0)
		VDBG("multi-dtd request!");
	/* Fill in the transfer size; set active bit */
	swap_temp = ((*length << DTD_LENGTH_BIT_POS) | DTD_STATUS_ACTIVE);

	/* Enable interrupt for the last dtd of a request */
	if (*is_last && !req->req.no_interrupt)
		swap_temp |= DTD_IOC;

	dtd->size_ioc_sts = cpu_to_hc32(swap_temp);

	mb();

	VDBG("length = %d address= 0x%x", *length, (int)*dma);

	return dtd;
}

/* Generate dtd chain for a request */
static int npcm_req_to_dtd(struct npcm_req *req, gfp_t gfp_flags)
{
	unsigned	count;
	int		is_last;
	int		is_first =1;
	struct ep_td_struct	*last_dtd = NULL, *dtd;
	dma_addr_t dma;

	do {
		dtd = npcm_build_dtd(req, &count, &dma, &is_last, gfp_flags);
		if (dtd == NULL)
			return -ENOMEM;

		if (is_first) {
			is_first = 0;
			req->head = dtd;
		} else {
			last_dtd->next_td_ptr = cpu_to_hc32(dma);
			last_dtd->next_td_virt = dtd;
		}
		last_dtd = dtd;

		req->dtd_count++;
	} while (!is_last);

	dtd->next_td_ptr = cpu_to_hc32(DTD_NEXT_TERMINATE);

	req->tail = dtd;

	return 0;
}

/* queues (submits) an I/O request to an endpoint */
static int
npcm_ep_queue(struct usb_ep *_ep, struct usb_request *_req, gfp_t gfp_flags)
{
	struct npcm_ep *ep;
	struct npcm_req *req;
	struct npcm_udc *udc;
	unsigned long flags;
	int ret;

	if (!_req)
	{
		pr_info("%s(): usb_request NULL\n", __func__);
		return -EINVAL;
	}

	ep = container_of(_ep, struct npcm_ep, ep);
	req = container_of(_req, struct npcm_req, req);
	
 	/* catch various bogus parameters */
	if (!req->req.complete || !req->req.buf
			|| !list_empty(&req->queue)) {
		pr_info("%s(): bad params\n", __func__);
		return -EINVAL;
	}
	if (unlikely(!_ep || !ep->ep.desc)) {
		pr_info("%s(): bad ep\n", __func__);
		return -EINVAL;
	}
	if (usb_endpoint_xfer_isoc(ep->ep.desc)) {
		if (req->req.length > ep->ep.maxpacket)
			return -EMSGSIZE;
	}

	udc = ep->udc;
	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	req->ep = ep;

	ret = usb_gadget_map_request(&ep->udc->gadget, &req->req, ep_is_in(ep));
	if (ret)
		return ret;

	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->dtd_count = 0;

	spin_lock_irqsave(&udc->lock, flags);

	/* build dtds and push them to device queue */
	if (!npcm_req_to_dtd(req, gfp_flags)) {
		ret = npcm_queue_td(ep, req);
		if (ret) {
			spin_unlock_irqrestore(&udc->lock, flags);
			NPCM_USB_ERR("%s(): Failed to queue dtd\n", __func__);
			goto err_unmap_dma;
		}
	} else {
		spin_unlock_irqrestore(&udc->lock, flags);
		NPCM_USB_ERR("%s(): Failed to dma_pool_alloc\n", __func__);
		ret = -ENOMEM;
		goto err_unmap_dma;
	}

	/* irq handler advances the queue */
	if (req != NULL)
		list_add_tail(&req->queue, &ep->queue);
	spin_unlock_irqrestore(&udc->lock, flags);

	return 0;
err_unmap_dma:
	usb_gadget_unmap_request(&ep->udc->gadget, &req->req, ep_is_in(ep));

	return ret;
}

/* dequeues (cancels, unlinks) an I/O request from an endpoint */
static int npcm_ep_dequeue(struct usb_ep *_ep, struct usb_request *_req)
{
	struct npcm_ep *ep;
	struct npcm_req *req;
	unsigned long flags;
	int ep_num, stopped, ret = 0;
	u32 epctrl;
    struct usb_dr_device *dr_regs;

    if (!_ep || !_req)
        return -EINVAL;

	ep = container_of(_ep, struct npcm_ep, ep);

    if (!ep->ep.desc)
        return -EINVAL;

  /* before here, make sure dr_regs has been initialized */
    if (!ep->udc)
        return -EINVAL;

    dr_regs = ep->udc->dr_regs;


	spin_lock_irqsave(&ep->udc->lock, flags);
	stopped = ep->stopped;

	/* Stop the ep before we deal with the queue */
	ep->stopped = 1;
	ep_num = ep_index(ep);
	epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl &= ~EPCTRL_TX_ENABLE;
	else
		epctrl &= ~EPCTRL_RX_ENABLE;
	npcm_writel(epctrl, &dr_regs->endptctrl[ep_num]);

	/* make sure it's actually queued on this endpoint */
	list_for_each_entry(req, &ep->queue, queue) {
		if (&req->req == _req)
			break;
	}
	if (&req->req != _req) {
		ret = -EINVAL;
		goto out;
	}

	/* The request is in progress, or completed but not dequeued */
	if (ep->queue.next == &req->queue) {
		_req->status = -ECONNRESET;
		npcm_ep_fifo_flush(_ep);	/* flush current transfer */

		/* The request isn't the last request in this ep queue */
		if (req->queue.next != &ep->queue) {
			struct npcm_req *next_req;

			next_req = list_entry(req->queue.next, struct npcm_req,
					queue);

			/* prime with dTD of next request */
			npcm_prime_ep(ep, next_req->head);
		}
	/* The request hasn't been processed, patch up the TD chain */
	} else {
		struct npcm_req *prev_req;

		prev_req = list_entry(req->queue.prev, struct npcm_req, queue);
		prev_req->tail->next_td_ptr = req->tail->next_td_ptr;
	}

	done(ep, req, -ECONNRESET);

	/* Enable EP */
out:	epctrl = npcm_readl(&dr_regs->endptctrl[ep_num]);
	if (ep_is_in(ep))
		epctrl |= EPCTRL_TX_ENABLE;
	else
		epctrl |= EPCTRL_RX_ENABLE;
	npcm_writel(epctrl, &dr_regs->endptctrl[ep_num]);
	ep->stopped = stopped;

	spin_unlock_irqrestore(&ep->udc->lock, flags);
	return ret;
}

/*-------------------------------------------------------------------------*/

/*-----------------------------------------------------------------
 * modify the endpoint halt feature
 * @ep: the non-isochronous endpoint being stalled
 * @value: 1--set halt  0--clear halt
 * Returns zero, or a negative error code.
*----------------------------------------------------------------*/
static int npcm_ep_set_halt(struct usb_ep *_ep, int value)
{
	struct npcm_ep *ep = NULL;
	unsigned long flags = 0;
	int status = -EOPNOTSUPP;	/* operation not supported */
	unsigned char ep_dir = 0, ep_num = 0;
	struct npcm_udc *udc = NULL;

	ep = container_of(_ep, struct npcm_ep, ep);
	udc = ep->udc;
	if (!_ep || !ep->ep.desc) {
		status = -EINVAL;
		goto out;
	}

	if (usb_endpoint_xfer_isoc(ep->ep.desc)) {
		status = -EOPNOTSUPP;
		goto out;
	}

	/* Attempt to halt IN ep will fail if any transfer requests
	 * are still queue */
	if (value && ep_is_in(ep) && !list_empty(&ep->queue)) {
		status = -EAGAIN;
		goto out;
	}

	status = 0;
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;
	ep_num = (unsigned char)(ep_index(ep));
	spin_lock_irqsave(&ep->udc->lock, flags);
	dr_ep_change_stall(udc, ep_num, ep_dir, value);
	spin_unlock_irqrestore(&ep->udc->lock, flags);

	if (ep_index(ep) == 0) {
		udc->ep0_state = WAIT_FOR_SETUP;
		udc->ep0_dir = 0;
	}

out:
	VDBG(" %s %s halt stat %d", ep->ep.name,
			value ?  "set" : "clear", status);

	return status;
}

static int npcm_ep_fifo_status(struct usb_ep *_ep)
{
	struct npcm_ep *ep;
    struct npcm_udc *udc = NULL;
	int size = 0;
	u32 bitmask;
	struct ep_queue_head *qh;
    struct usb_dr_device *dr_regs;
	
	if (!_ep)
		return -ENODEV;		

	ep = container_of(_ep, struct npcm_ep, ep);

	if (!ep->ep.desc && ep_index(ep) != 0)
		return -ENODEV;

  /* before here, make sure dr_regs has been initialized */
    if (!ep->udc)
        return -ENODEV;

    dr_regs = ep->udc->dr_regs;

	udc = (struct npcm_udc *)ep->udc;

	if (!udc->driver || udc->gadget.speed == USB_SPEED_UNKNOWN)
		return -ESHUTDOWN;

	qh = get_qh_by_ep(ep);

	bitmask = (ep_is_in(ep)) ? (1 << (ep_index(ep) + 16)) :
	    (1 << (ep_index(ep)));

	if (npcm_readl(&dr_regs->endptstatus) & bitmask)
		size = (qh->size_ioc_int_sts & DTD_PACKET_SIZE)
		    >> DTD_LENGTH_BIT_POS;

	pr_debug("%s %u\n", __func__, size);
	return size;
}

static void npcm_ep_fifo_flush(struct usb_ep *_ep)
{
	struct npcm_ep *ep;
	int ep_num, ep_dir;
	u32 bits;
	unsigned long timeout;
	struct usb_dr_device *dr_regs;
    struct npcm_udc *udc = NULL;

#define NPCM_UDC_FLUSH_TIMEOUT 1000

	if ((!_ep)||(!_ep->desc)) {
		return;
	} else {
		ep = container_of(_ep, struct npcm_ep, ep);
	}
	
	udc = (struct npcm_udc *)ep->udc;

   /* before here, make sure dr_regs has been initialized */
    if (!udc)
        return;

    dr_regs = udc->dr_regs;

	ep_num = ep_index(ep);
	ep_dir = ep_is_in(ep) ? USB_SEND : USB_RECV;

	if (ep_num == 0)
		bits = (1 << 16) | 1;
	else if (ep_dir == USB_SEND)
		bits = 1 << (16 + ep_num);
	else
		bits = 1 << ep_num;

	timeout = jiffies + NPCM_UDC_FLUSH_TIMEOUT;
	do {
		npcm_writel(bits, &dr_regs->endptflush);

		/* Wait until flush complete */
		while (npcm_readl(&dr_regs->endptflush)) {
			if (time_after(jiffies, timeout)) {
				NPCM_USB_ERR("ep flush timeout\n");
				return;
			}
			cpu_relax();
		}
		/* See if we need to flush again */
	} while (npcm_readl(&dr_regs->endptstatus) & bits);
}

static const struct usb_ep_ops npcm_ep_ops = {
	.enable = npcm_ep_enable,
	.disable = npcm_ep_disable,

	.alloc_request = npcm_alloc_request,
	.free_request = npcm_free_request,

	.queue = npcm_ep_queue,
	.dequeue = npcm_ep_dequeue,

	.set_halt = npcm_ep_set_halt,
	.fifo_status = npcm_ep_fifo_status,
	.fifo_flush = npcm_ep_fifo_flush,	/* flush fifo */
};

/*-------------------------------------------------------------------------
		Gadget Driver Layer Operations
-------------------------------------------------------------------------*/

/*----------------------------------------------------------------------
 * Get the current frame number (from DR frame_index Reg )
 *----------------------------------------------------------------------*/
static int npcm_get_frame(struct usb_gadget *gadget)
{
    struct npcm_udc *udc = container_of(gadget, struct npcm_udc, gadget);
    struct usb_dr_device *dr_regs = udc->dr_regs;

	return (int)(npcm_readl(&dr_regs->frindex) & USB_FRINDEX_MASKS);
}

/*-----------------------------------------------------------------------
 * Tries to wake up the host connected to this gadget
 -----------------------------------------------------------------------*/
static int npcm_wakeup(struct usb_gadget *gadget)
{
	struct npcm_udc *udc = container_of(gadget, struct npcm_udc, gadget);
	u32 portsc;
    struct usb_dr_device *dr_regs;

    dr_regs = udc->dr_regs;

    DBG("");
	/* Remote wakeup feature not enabled by host */
	if (!udc->remote_wakeup)
		return -ENOTSUPP;

	portsc = npcm_readl(&dr_regs->portsc1);
	/* not suspended? */
	if (!(portsc & PORTSCX_PORT_SUSPEND))
		return 0;

    /* trigger force resume */
    portsc |= PORTSCX_PORT_FORCE_RESUME;
    npcm_writel(portsc, &dr_regs->portsc1);
    
	return 0;
}

static int can_pullup(struct npcm_udc *udc)
{
	return udc->driver && udc->softconnect && udc->vbus_active;
}

/* Notify controller that VBUS is powered, Called by whatever
   detects VBUS sessions */
static int npcm_vbus_session(struct usb_gadget *gadget, int is_active)
{
	struct npcm_udc	*udc;
	unsigned long	flags;

	udc = container_of(gadget, struct npcm_udc, gadget);
	spin_lock_irqsave(&udc->lock, flags);
	VDBG("VBUS %s", is_active ? "on" : "off");
	udc->vbus_active = (is_active != 0);
	if (can_pullup(udc))
        dr_controller_run(udc);
	else
        dr_controller_stop(udc);
	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

/* constrain controller's VBUS power usage
 * This call is used by gadget drivers during SET_CONFIGURATION calls,
 * reporting how much power the device may consume.  For example, this
 * could affect how quickly batteries are recharged.
 *
 * Returns zero on success, else negative errno.
 */
static int npcm_vbus_draw(struct usb_gadget *gadget, unsigned mA)
{
	struct npcm_udc *udc;

	udc = container_of(gadget, struct npcm_udc, gadget);
	if (!IS_ERR_OR_NULL(udc->transceiver))
		return usb_phy_set_power(udc->transceiver, mA);
	return -ENOTSUPP;
}

/* Change Data+ pullup status
 * this func is used by usb_gadget_connect/disconnet
 */
static int npcm_pullup(struct usb_gadget *gadget, int is_on)
{
    struct npcm_udc *udc  = container_of(gadget, struct npcm_udc, gadget);

	udc = container_of(gadget, struct npcm_udc, gadget);

	if (!udc->vbus_active)
		return -EOPNOTSUPP;

	udc->softconnect = (is_on != 0);
	if (can_pullup(udc))
        dr_controller_run(udc);
	else
        dr_controller_stop(udc);

	return 0;
}

static int npcm_udc_start(struct usb_gadget *gadget,
                        struct usb_gadget_driver *driver);
static int npcm_udc_stop(struct usb_gadget *gadget);
/* defined in gadget.h */
static struct usb_gadget_ops npcm_gadget_ops = {
	.get_frame = npcm_get_frame,
	.wakeup = npcm_wakeup,
/*	.set_selfpowered = npcm_set_selfpowered,	*/ /* Always selfpowered */
	.vbus_session = npcm_vbus_session,
	.vbus_draw = npcm_vbus_draw,
	.pullup = npcm_pullup,
	.udc_start = npcm_udc_start,
	.udc_stop = npcm_udc_stop,
};

/*
 * Empty complete function used by this driver to fill in the req->complete
 * field when creating a request since the complete field is mandatory.
 */
static void npcm_noop_complete(struct usb_ep *ep, struct usb_request *req) { }

/* Set protocol stall on ep0, protocol stall will automatically be cleared
   on new transaction */
static void ep0stall(struct npcm_udc *udc)
{
	u32 tmp;
    struct usb_dr_device *dr_regs;

    if(!udc)
      return;

    dr_regs = udc->dr_regs;

	/* must set tx and rx to stall at the same time */
	tmp = npcm_readl(&dr_regs->endptctrl[0]);
	tmp |= EPCTRL_TX_EP_STALL | EPCTRL_RX_EP_STALL;
	npcm_writel(tmp, &dr_regs->endptctrl[0]);
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->ep0_dir = 0;
}

/* Prime a status phase for ep0 */
static int ep0_prime_status(struct npcm_udc *udc, int direction)
{
	struct npcm_req *req = udc->status_req;
	struct npcm_ep *ep;
	int ret;

	if (direction == EP_DIR_IN)
		udc->ep0_dir = USB_DIR_IN;
	else
		udc->ep0_dir = USB_DIR_OUT;

	ep = &udc->eps[0];
	udc->ep0_state = WAIT_FOR_OUT_STATUS;

	req->ep = ep;
	req->req.length = 0;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = npcm_noop_complete;
	req->dtd_count = 0;

	ret = usb_gadget_map_request(&ep->udc->gadget, &req->req, ep_is_in(ep));
	if (ret)
		return ret;

	if (npcm_req_to_dtd(req, GFP_ATOMIC) == 0) {
		ret = npcm_queue_td(ep, req);
		if (ret) {
			NPCM_USB_ERR("%s(): Failed to queue dtd when prime status\n", __func__);
			goto out;
		}
	} else{	/* no mem */
		ret = -ENOMEM;
		NPCM_USB_ERR("%s(): Failed to dma_pool_alloc when prime status\n", __func__);
		goto out;
	}

	list_add_tail(&req->queue, &ep->queue);

	return 0;
out:
	usb_gadget_unmap_request(&ep->udc->gadget, &req->req, ep_is_in(ep));

	return ret;
}

static void udc_reset_ep_queue(struct npcm_udc *udc, u8 pipe)
{
	struct npcm_ep *ep = get_ep_by_pipe(udc, pipe);

	if (ep->ep.name)
		nuke(ep, -ESHUTDOWN);
}

/*
 * ch9 Set address
 */
static void ch9setaddress(struct npcm_udc *udc, u16 value, u16 index, u16 length)
{
	/* Save the new address to device struct */
	udc->device_address = (u8) value;
	/* Update usb state */
	udc->usb_state = USB_STATE_ADDRESS;
	/* Status phase */
	if (ep0_prime_status(udc, EP_DIR_IN))
		ep0stall(udc);
}

/*
 * ch9 Get status
 */
static void ch9getstatus(struct npcm_udc *udc, u8 request_type, u16 value,
		u16 index, u16 length)
{
	u16 tmp = 0;		/* Status, cpu endian */
	struct npcm_req *req;
	struct npcm_ep *ep;
	int ret;

	ep = &udc->eps[0];

	if ((request_type & USB_RECIP_MASK) == USB_RECIP_DEVICE) {
		/* Get device status */
		tmp = udc->gadget.is_selfpowered;
		tmp |= udc->remote_wakeup << USB_DEVICE_REMOTE_WAKEUP;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_INTERFACE) {
		/* Get interface status */
		/* We don't have interface information in udc driver */
		tmp = 0;
	} else if ((request_type & USB_RECIP_MASK) == USB_RECIP_ENDPOINT) {
		/* Get endpoint status */
		struct npcm_ep *target_ep;

		target_ep = get_ep_by_pipe(udc, get_pipe_by_windex(index));

		/* stall if endpoint doesn't exist */
		if (!target_ep->ep.desc)
			goto stall;
		tmp = dr_ep_get_stall(udc, ep_index(target_ep), ep_is_in(target_ep))
				<< USB_ENDPOINT_HALT;
	}

	udc->ep0_dir = USB_DIR_IN;
	/* Borrow the per device status_req */
	req = udc->status_req;
	/* Fill in the reqest structure */
	*((u16 *) req->req.buf) = cpu_to_le16(tmp);

	req->ep = ep;
	req->req.length = 2;
	req->req.status = -EINPROGRESS;
	req->req.actual = 0;
	req->req.complete = npcm_noop_complete;
	req->dtd_count = 0;

	ret = usb_gadget_map_request(&ep->udc->gadget, &req->req, ep_is_in(ep));
	if (ret)
		goto stall;

	/* prime the data phase */
	if ((npcm_req_to_dtd(req, GFP_ATOMIC) == 0)) {
		ret = npcm_queue_td(ep, req);
		if (ret) {
			NPCM_USB_ERR("%s(): Failed to queue dtd\n", __func__);
			goto err_unmap_dma;
		}
	} else {
		NPCM_USB_ERR("%s(): Failed to dma_pool_alloc\n", __func__);
		goto err_unmap_dma;
	}

	list_add_tail(&req->queue, &ep->queue);
	udc->ep0_state = DATA_STATE_XMIT;

	return;
	
err_unmap_dma:
	usb_gadget_unmap_request(&ep->udc->gadget, &req->req, ep_is_in(ep));
	
stall:
	ep0stall(udc);
}

static void setup_received_irq(struct npcm_udc *udc,
		struct usb_ctrlrequest *setup)
__releases(udc->lock)
__acquires(udc->lock)
{
	u16 wValue = le16_to_cpu(setup->wValue);
	u16 wIndex = le16_to_cpu(setup->wIndex);
	u16 wLength = le16_to_cpu(setup->wLength);
    struct usb_dr_device *dr_regs;

    if(!udc)
      return;

    dr_regs = udc->dr_regs;

	udc_reset_ep_queue(udc, 0);

	/* We process some stardard setup requests here */
	switch (setup->bRequest) {
	case USB_REQ_GET_STATUS:
		/* Data+Status phase from udc */
		if ((setup->bRequestType & (USB_DIR_IN | USB_TYPE_MASK))
					!= (USB_DIR_IN | USB_TYPE_STANDARD))
			break;
		ch9getstatus(udc, setup->bRequestType, wValue, wIndex, wLength);
		return;

	case USB_REQ_SET_ADDRESS:
		/* Status phase from udc */
		if (setup->bRequestType != (USB_DIR_OUT | USB_TYPE_STANDARD
						| USB_RECIP_DEVICE))
			break;
		ch9setaddress(udc, wValue, wIndex, wLength);
		return;

	case USB_REQ_CLEAR_FEATURE:
	case USB_REQ_SET_FEATURE:
		/* Status phase from udc */
	{
		int rc;
		u16 ptc = 0;

		if ((setup->bRequestType & (USB_RECIP_MASK | USB_TYPE_MASK))
				== (USB_RECIP_ENDPOINT | USB_TYPE_STANDARD)) {
			int pipe = get_pipe_by_windex(wIndex);
			struct npcm_ep *ep;

			if (wValue != 0 || wLength != 0 || pipe >= udc->max_ep)
				break;
			ep = get_ep_by_pipe(udc, pipe);

			spin_unlock(&udc->lock);
			rc = npcm_ep_set_halt(&ep->ep,(setup->bRequest == USB_REQ_SET_FEATURE) ? 1 : 0);
			spin_lock(&udc->lock);

		} else if ((setup->bRequestType & (USB_RECIP_MASK
				| USB_TYPE_MASK)) == (USB_RECIP_DEVICE
				| USB_TYPE_STANDARD)) {
			/* Note: The driver has not include OTG support yet.
			 * This will be set when OTG support is added */
			if (wValue == USB_DEVICE_TEST_MODE)
				ptc = wIndex >> 8;
            else if (wValue == USB_DEVICE_REMOTE_WAKEUP)
                udc->remote_wakeup = (setup->bRequest == USB_REQ_CLEAR_FEATURE)? 0: 1;
			else if (gadget_is_otg(&udc->gadget)) {
				if (setup->bRequest ==
				    USB_DEVICE_B_HNP_ENABLE)
					udc->gadget.b_hnp_enable = 1;
				else if (setup->bRequest ==
					 USB_DEVICE_A_HNP_SUPPORT)
					udc->gadget.a_hnp_support = 1;
				else if (setup->bRequest ==
					 USB_DEVICE_A_ALT_HNP_SUPPORT)
					udc->gadget.a_alt_hnp_support = 1;
			}
			rc = 0;
		} else
			break;

		if (rc == 0) {
			if (ep0_prime_status(udc, EP_DIR_IN))
				ep0stall(udc);
		}
		if (ptc) {
			u32 tmp;

			mdelay(10);
			tmp = npcm_readl(&dr_regs->portsc1) | (ptc << 16);
			npcm_writel(tmp, &dr_regs->portsc1);
			printk(KERN_INFO "udc: switch to test mode %d.\n", ptc);
		}

		return;
	}

	default:
		break;
	}

	/* Requests handled by gadget */
	if (wLength) {
		/* Data phase from gadget, status phase from udc */
		udc->ep0_dir = (setup->bRequestType & USB_DIR_IN)
				?  USB_DIR_IN : USB_DIR_OUT;
		spin_unlock(&udc->lock);
		if (udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = (setup->bRequestType & USB_DIR_IN)
				?  DATA_STATE_XMIT : DATA_STATE_RECV;
	} else {
		/* No data phase, IN status from gadget */
		udc->ep0_dir = USB_DIR_IN;
		spin_unlock(&udc->lock);
		if (udc->driver->setup(&udc->gadget,
				&udc->local_setup_buff) < 0)
			ep0stall(udc);
		spin_lock(&udc->lock);
		udc->ep0_state = WAIT_FOR_OUT_STATUS;
	}
}

/* Process request for Data or Status phase of ep0
 * prime status phase if needed */
static void ep0_req_complete(struct npcm_udc *udc, struct npcm_ep *ep0,
		struct npcm_req *req)
{
    struct usb_dr_device *dr_regs;

    if(!udc)
      return;

    dr_regs = udc->dr_regs;

	if (udc->usb_state == USB_STATE_ADDRESS) {
		/* Set the new address */
		u32 new_address = (u32) udc->device_address;
		npcm_writel(new_address << USB_DEVICE_ADDRESS_BIT_POS,
				&dr_regs->deviceaddr);
	}

	done(ep0, req, 0);

	switch (udc->ep0_state) {
	case DATA_STATE_XMIT:
		/* already primed at setup_received_irq */
		udc->ep0_state = WAIT_FOR_OUT_STATUS;
        if (ep0_prime_status(udc, EP_DIR_OUT))
            ep0stall(udc);
		break;
	case DATA_STATE_RECV:
		/* send status phase */
		if (ep0_prime_status(udc, EP_DIR_IN))
			ep0stall(udc);
		break;
	case WAIT_FOR_OUT_STATUS:
		udc->ep0_state = WAIT_FOR_SETUP;
		break;
	case WAIT_FOR_SETUP:
		NPCM_USB_ERR("Unexpected ep0 packets\n");
		break;
	default:
		ep0stall(udc);
		break;
	}
}

/* Tripwire mechanism to ensure a setup packet payload is extracted without
 * being corrupted by another incoming setup packet */
static void tripwire_handler(struct npcm_udc *udc, u8 ep_num, u8 *buffer_ptr)
{
	u32 temp;
	struct ep_queue_head *qh;
	struct npcm_usb2_platform_data *pdata = udc->pdata;
    struct usb_dr_device *dr_regs = udc->dr_regs;

	qh = &udc->ep_qh[ep_num * 2 + EP_DIR_OUT];

	/* Clear bit in ENDPTSETUPSTAT */
	temp = npcm_readl(&dr_regs->endptsetupstat);
	npcm_writel(temp | (1 << ep_num), &dr_regs->endptsetupstat);

	/* while a hazard exists when setup package arrives */
	do {
		/* Set Setup Tripwire */
		temp = npcm_readl(&dr_regs->usbcmd);
		npcm_writel(temp | USB_CMD_SUTW, &dr_regs->usbcmd);

		/* Copy the setup packet to local buffer */
		if (pdata->le_setup_buf) {
			u32 *p = (u32 *)buffer_ptr;
			u32 *s = (u32 *)qh->setup_buffer;

			/* Convert little endian setup buffer to CPU endian */
			*p++ = le32_to_cpu(*s++);
			*p = le32_to_cpu(*s);
		} else {
			memcpy(buffer_ptr, (u8 *) qh->setup_buffer, 8);
		}
	} while (!(npcm_readl(&dr_regs->usbcmd) & USB_CMD_SUTW));

	/* Clear Setup Tripwire */
	temp = npcm_readl(&dr_regs->usbcmd);
	npcm_writel(temp & ~USB_CMD_SUTW, &dr_regs->usbcmd);
}

/* process-ep_req(): free the completed Tds for this req */
static int process_ep_req(struct npcm_udc *udc, int pipe,
		struct npcm_req *curr_req)
{
	struct ep_td_struct *curr_td;
	int	td_complete, actual, remaining_length, j, tmp;
	int	status = 0;
	int	errors = 0;
	struct  ep_queue_head *curr_qh = &udc->ep_qh[pipe];
	int direction = pipe % 2;

	curr_td = curr_req->head;
	td_complete = 0;
	actual = curr_req->req.length;

	for (j = 0; j < curr_req->dtd_count; j++) {
		remaining_length = (hc32_to_cpu(curr_td->size_ioc_sts)
					& DTD_PACKET_SIZE)
				>> DTD_LENGTH_BIT_POS;
		actual -= remaining_length;

		errors = hc32_to_cpu(curr_td->size_ioc_sts);
		if (errors & DTD_ERROR_MASK) {
			if (errors & DTD_STATUS_HALTED) {
				NPCM_USB_ERR("dTD error %08x QH=%d\n", errors, pipe);
				/* Clear the errors and Halt condition */
				tmp = hc32_to_cpu(curr_qh->size_ioc_int_sts);
				tmp &= ~errors;
				curr_qh->size_ioc_int_sts = cpu_to_hc32(tmp);
				status = -EPIPE;
				/* FIXME: continue with next queued TD? */

				break;
			}
			if (errors & DTD_STATUS_DATA_BUFF_ERR) {
				pr_info("%s(): Transfer overflow\n", __func__);
				status = -EPROTO;
				break;
			} else if (errors & DTD_STATUS_TRANSACTION_ERR) {
				pr_info("%s(): ISO error\n", __func__);
				status = -EILSEQ;
				break;
			} else
				NPCM_USB_ERR("Unknown error has occurred (0x%x)!\n",
					errors);

		} else if (hc32_to_cpu(curr_td->size_ioc_sts)
				& DTD_STATUS_ACTIVE) {
			VDBG("%s(): Request not complete\n", __func__);
			status = REQ_UNCOMPLETE;
			return status;
		} else if (remaining_length) {
			if (direction) {
				pr_info("%s(): Transmit dTD remaining length not zero\n", __func__);
				status = -EPROTO;
				break;
			} else {
				td_complete++;
				break;
			}
		} else {
			td_complete++;
			VDBG("dTD transmitted successful");
		}

		if (j != curr_req->dtd_count - 1)
			curr_td = (struct ep_td_struct *)curr_td->next_td_virt;
	}

	if (status)
		return status;

	curr_req->req.actual = actual;

	return 0;
}

/* Process a DTD completion interrupt */
static void dtd_complete_irq(struct npcm_udc *udc)
{
	u32 bit_pos;
	int i, ep_num, direction, bit_mask, status;
	struct npcm_ep *curr_ep;
	struct npcm_req *curr_req, *temp_req;
    struct usb_dr_device *dr_regs;

    if(!udc)
      return;

    dr_regs = udc->dr_regs;


	/* Clear the bits in the register */
	bit_pos = npcm_readl(&dr_regs->endptcomplete);
	npcm_writel(bit_pos, &dr_regs->endptcomplete);

	if (!bit_pos)
		return;

	for (i = 0; i < udc->max_ep; i++) {
		ep_num = i >> 1;
		direction = i % 2;

		bit_mask = 1 << (ep_num + 16 * direction);

		if (!(bit_pos & bit_mask))
			continue;

		curr_ep = get_ep_by_pipe(udc, i);

		/* If the ep is configured */
		if (!curr_ep->ep.name) {
			NPCM_USB_WARN("Invalid EP?");
			continue;
		}

		/* process the req queue until an uncomplete request */
		list_for_each_entry_safe(curr_req, temp_req, &curr_ep->queue,
				queue) {
			status = process_ep_req(udc, i, curr_req);

			VDBG("status of process_ep_req= %d, ep = %d",
					status, ep_num);
			if (status == REQ_UNCOMPLETE)
				break;
			/* write back status to req */
			curr_req->req.status = status;

			if (ep_num == 0) {
				ep0_req_complete(udc, curr_ep, curr_req);
				break;
			} else
				done(curr_ep, curr_req, status);
		}
	}
}

static inline enum usb_device_speed portscx_device_speed(u32 reg)
{
	switch (reg & PORTSCX_PORT_SPEED_MASK) {
	case PORTSCX_PORT_SPEED_HIGH:
		return USB_SPEED_HIGH;
	case PORTSCX_PORT_SPEED_FULL:
		return USB_SPEED_FULL;
	case PORTSCX_PORT_SPEED_LOW:
		return USB_SPEED_LOW;
	default:
		return USB_SPEED_UNKNOWN;
	}
}

/* Process a port change interrupt */
static void port_change_irq(struct npcm_udc *udc)
{
    struct usb_dr_device *dr_regs = udc->dr_regs;

	if (udc->bus_reset)
		udc->bus_reset = 0;

	/* Bus resetting is finished */
	if (!(npcm_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET))
		/* Get the speed */
		udc->gadget.speed =
			portscx_device_speed(npcm_readl(&dr_regs->portsc1));

	/* Update USB state */
	if (!udc->resume_state)
		udc->usb_state = USB_STATE_DEFAULT;
}

/* Process suspend interrupt */
static void suspend_irq(struct npcm_udc *udc)
{
	udc->resume_state = udc->usb_state;
	udc->usb_state = USB_STATE_SUSPENDED;

	/* report suspend to the driver, serial.c does not support this */
	if (udc->driver->suspend)
		udc->driver->suspend(&udc->gadget);
}

static void bus_resume(struct npcm_udc *udc)
{
	udc->usb_state = udc->resume_state;
	udc->resume_state = 0;

	/* report resume to the driver, serial.c does not support this */
	if (udc->driver->resume)
		udc->driver->resume(&udc->gadget);
}

/* Clear up all ep queues */
static int reset_queues(struct npcm_udc *udc, bool bus_reset)
{
	u8 pipe;

	for (pipe = 0; pipe < udc->max_pipes; pipe++)
		udc_reset_ep_queue(udc, pipe);

	/* report disconnect; the driver is already quiesced */
	spin_unlock(&udc->lock);
	if (bus_reset)
		usb_gadget_udc_reset(&udc->gadget, udc->driver);
	else
		udc->driver->disconnect(&udc->gadget);
	spin_lock(&udc->lock);

	return 0;
}

/* Process reset interrupt */
static void reset_irq(struct npcm_udc *udc)
{
	u32 temp;
	unsigned long timeout;
    struct usb_dr_device *dr_regs;

    if(!udc)
      return;

    dr_regs = udc->dr_regs;


	/* Clear the device address */
	temp = npcm_readl(&dr_regs->deviceaddr);
	npcm_writel(temp & ~USB_DEVICE_ADDRESS_MASK, &dr_regs->deviceaddr);

	udc->device_address = 0;

	/* Clear usb state */
	udc->resume_state = 0;
	udc->ep0_dir = 0;
	udc->ep0_state = WAIT_FOR_SETUP;
	udc->remote_wakeup = 0;	/* default to 0 on reset */
	udc->gadget.b_hnp_enable = 0;
	udc->gadget.a_hnp_support = 0;
	udc->gadget.a_alt_hnp_support = 0;

	/* Clear all the setup token semaphores */
	temp = npcm_readl(&dr_regs->endptsetupstat);
	npcm_writel(temp, &dr_regs->endptsetupstat);

	/* Clear all the endpoint complete status bits */
	temp = npcm_readl(&dr_regs->endptcomplete);
	npcm_writel(temp, &dr_regs->endptcomplete);

	timeout = jiffies + 100;
	while (npcm_readl(&dr_regs->endpointprime)) {
		/* Wait until all endptprime bits cleared */
		if (time_after(jiffies, timeout)) {
			NPCM_USB_ERR("Timeout for reset\n");
			break;
		}
		cpu_relax();
	}

	/* Write 1s to the flush register */
	npcm_writel(0xffffffff, &dr_regs->endptflush);

	if (npcm_readl(&dr_regs->portsc1) & PORTSCX_PORT_RESET) {
		VDBG("Bus reset %d", udc->id);
		/* Bus is reseting */
		udc->bus_reset = 1;
		/* Reset all the queues, include XD, dTD, EP queue
		 * head and TR Queue */
		reset_queues(udc, true);
		udc->usb_state = USB_STATE_DEFAULT;
	} else {
		VDBG("Controller reset %d", udc->id);
		/* initialize usb hw reg except for regs for EP, not
		 * touch usbintr reg */
		dr_controller_setup(udc);

		/* Reset all internal used Queues */
		reset_queues(udc, false);

		ep0_setup(udc);

		/* Enable DR IRQ reg, Set Run bit, change udc state */
		dr_controller_run(udc);
		udc->usb_state = USB_STATE_ATTACHED;
	}
}

/*
 * USB device controller interrupt handler
 */
static irqreturn_t npcm_udc_irq(int irq, void *_udc)
{
	struct npcm_udc *udc = _udc;
	u32 irq_src;
	irqreturn_t status = IRQ_NONE;
	unsigned long flags;
    struct usb_dr_device *dr_regs;

    if(!udc)
       return IRQ_NONE;

    dr_regs = udc->dr_regs;


	/* Disable ISR for OTG host mode */
	if (udc->stopped)
		return IRQ_NONE;
	spin_lock_irqsave(&udc->lock, flags);
	irq_src = npcm_readl(&dr_regs->usbsts) & npcm_readl(&dr_regs->usbintr);
	/* Clear notification bits */
	npcm_writel(irq_src, &dr_regs->usbsts);

	/* VDBG("irq_src [0x%8x]", irq_src); */

	/* Need to resume? */
	if (udc->usb_state == USB_STATE_SUSPENDED)
		if ((npcm_readl(&dr_regs->portsc1) & PORTSCX_PORT_SUSPEND) == 0)
			bus_resume(udc);

	/* USB Interrupt */
	if (irq_src & USB_STS_INT) {
		VDBG("Packet int");
		/* Setup package, we only support ep0 as control ep */
		if (npcm_readl(&dr_regs->endptsetupstat) & EP_SETUP_STATUS_EP0) {
			tripwire_handler(udc, 0,
					(u8 *) (&udc->local_setup_buff));
			setup_received_irq(udc, &udc->local_setup_buff);
			status = IRQ_HANDLED;
		}

		/* completion of dtd */
		if (npcm_readl(&dr_regs->endptcomplete)) {
			dtd_complete_irq(udc);
			status = IRQ_HANDLED;
		}
	}

	/* SOF (for ISO transfer) */
	if (irq_src & USB_STS_SOF) {
		status = IRQ_HANDLED;
	}

	/* Port Change */
	if (irq_src & USB_STS_PORT_CHANGE) {
		VDBG("Port Change");
		port_change_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Reset Received */
	if (irq_src & USB_STS_RESET) {
		VDBG("reset int");
		reset_irq(udc);
		status = IRQ_HANDLED;
	}

	/* Sleep Enable (Suspend) */
	if (irq_src & USB_STS_SUSPEND) {
		VDBG("Suspend int %d", udc->id);
		suspend_irq(udc);
		status = IRQ_HANDLED;
	}

	if (irq_src & (USB_STS_ERR | USB_STS_SYS_ERR)) {
		pr_info("%s(): Error IRQ %x\n", __func__, irq_src);
	}

	spin_unlock_irqrestore(&udc->lock, flags);
	return status;
}

/*----------------------------------------------------------------*
 * Hook to gadget drivers
 * Called by initialization code of gadget drivers
*----------------------------------------------------------------*/
static int npcm_udc_start(struct usb_gadget *gadget,
                        struct usb_gadget_driver *driver)
{
	int retval = 0;
	unsigned long flags = 0;
	struct npcm_udc *udc_controller = gadget_to_npcm(gadget);

	/* lock is needed but whether should use this lock or another */
	spin_lock_irqsave(&udc_controller->lock, flags);

	driver->driver.bus = NULL;
	/* hook up the driver */
	udc_controller->driver = driver;
	spin_unlock_irqrestore(&udc_controller->lock, flags);
	gadget->is_selfpowered = 1;

	if (!IS_ERR_OR_NULL(udc_controller->transceiver)) {
		/* Suspend the controller until OTG enable it */
		udc_controller->stopped = 1;
		printk(KERN_INFO "Suspend udc for OTG auto detect\n");

		/* connect to bus through transceiver */
		if (!IS_ERR_OR_NULL(udc_controller->transceiver)) {
			retval = otg_set_peripheral(
					udc_controller->transceiver->otg,
						    &udc_controller->gadget);
			if (retval < 0) {
				NPCM_USB_ERR("can't bind to transceiver\n");
				udc_controller->driver = NULL;
				return retval;
			}
		}
	}
#if 0
    // npcm_pullup starts the controller
    else {
		/* Enable DR IRQ reg and set USBCMD reg Run bit */
		dr_controller_run(udc_controller);
		udc_controller->usb_state = USB_STATE_ATTACHED;
		udc_controller->ep0_state = WAIT_FOR_SETUP;
		udc_controller->ep0_dir = 0;
	}
#endif

	printk(KERN_INFO "%s: bind to driver %s\n",
			udc_controller->gadget.name, driver->driver.name);

	if (retval)
		printk(KERN_WARNING "gadget driver register failed %d\n",
		       retval);
	return retval;
}

/* Disconnect from gadget driver */
static int npcm_udc_stop(struct usb_gadget *gadget)
{
	struct npcm_ep *loop_ep;
	unsigned long flags;
    struct npcm_udc *udc_controller = gadget_to_npcm(gadget);

if (!IS_ERR_OR_NULL(udc_controller->transceiver))
		otg_set_peripheral(udc_controller->transceiver->otg, NULL);

	/* stop DR, disable intr */
	dr_controller_stop(udc_controller);

	/* in fact, no needed */
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;

	/* stand operation */
	spin_lock_irqsave(&udc_controller->lock, flags);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
	nuke(&udc_controller->eps[0], -ESHUTDOWN);
	list_for_each_entry(loop_ep, &udc_controller->gadget.ep_list,
			ep.ep_list)
		nuke(loop_ep, -ESHUTDOWN);
	spin_unlock_irqrestore(&udc_controller->lock, flags);

	printk(KERN_WARNING "unregistered gadget driver '%s'\n",
	       udc_controller->driver->driver.name);

    udc_controller->driver = NULL;

	return 0;
}

/*-------------------------------------------------------------------------
		PROC File System Support
-------------------------------------------------------------------------*/
//#define CONFIG_USB_GADGET_DEBUG_FILES
#ifdef CONFIG_USB_GADGET_DEBUG_FILES

#include <linux/seq_file.h>

//static const char proc_filename[] = "driver/npcm_usb2_udc";
#define PROC_FILENAME "driver/npcm_udc"
char proc_filename[32];

static int npcm_proc_read(struct seq_file *m, void *v)
{
	unsigned long flags;
	int i;
	u32 tmp_reg;
	struct npcm_ep *ep = NULL;
	struct npcm_req *req;
    struct usb_dr_device *dr_regs;
    struct npcm_udc *udc = m->private;


	spin_lock_irqsave(&udc->lock, flags);
	dr_regs = udc->dr_regs;

	/* ------basic driver information ---- */
	seq_printf(m,
			DRIVER_DESC "\n"
			"%s version: %s\n"
			"Gadget driver: %s\n\n",
			drv_20_name, DRIVER_VERSION,
			udc->driver ? udc->driver->driver.name : "(none)");

	/* ------ DR Registers ----- */
    tmp_reg = npcm_readl(&dr_regs->sbscfg);
    seq_printf(m,
            "SBSCFG reg:\n"
            "AHBBRST: %d\n\n",
            tmp_reg);

	tmp_reg = npcm_readl(&dr_regs->usbcmd);
	seq_printf(m,
			"USBCMD reg:\n"
			"SetupTW: %d\n"
			"Run/Stop: %s\n\n",
			(tmp_reg & USB_CMD_SUTW) ? 1 : 0,
			(tmp_reg & USB_CMD_RUN_STOP) ? "Run" : "Stop");

	tmp_reg = npcm_readl(&dr_regs->usbsts);
	seq_printf(m,
			"USB Status Reg:\n"
			"Dr Suspend: %d\n"
			"Reset Received: %d\n"
			"System Error: %s\n"
			"USB Error Interrupt: %s\n\n",
			(tmp_reg & USB_STS_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_STS_RESET) ? 1 : 0,
			(tmp_reg & USB_STS_SYS_ERR) ? "Err" : "Normal",
			(tmp_reg & USB_STS_ERR) ? "Err detected" : "No err");

	tmp_reg = npcm_readl(&dr_regs->usbintr);
	seq_printf(m,
			"USB Intrrupt Enable Reg:\n"
			"Sleep Enable: %d\n"
			"SOF Received Enable: %d\n"
			"Reset Enable: %d\n"
			"System Error Enable: %d\n"
			"Port Change Dectected Enable: %d\n"
			"USB Error Intr Enable: %d\n"
			"USB Intr Enable: %d\n\n",
			(tmp_reg & USB_INTR_DEVICE_SUSPEND) ? 1 : 0,
			(tmp_reg & USB_INTR_SOF_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_RESET_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_SYS_ERR_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_PTC_DETECT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_ERR_INT_EN) ? 1 : 0,
			(tmp_reg & USB_INTR_INT_EN) ? 1 : 0);

	tmp_reg = npcm_readl(&dr_regs->frindex);
	seq_printf(m,
			"USB Frame Index Reg: Frame Number is 0x%x\n\n",
			(tmp_reg & USB_FRINDEX_MASKS));

	tmp_reg = npcm_readl(&dr_regs->deviceaddr);
	seq_printf(m,
			"USB Device Address Reg: Device Addr is 0x%x\n\n",
			(tmp_reg & USB_DEVICE_ADDRESS_MASK));

	tmp_reg = npcm_readl(&dr_regs->endpointlistaddr);
	seq_printf(m,
			"USB Endpoint List Address Reg: "
			"Device Addr is 0x%x\n\n",
			(tmp_reg & USB_EP_LIST_ADDRESS_MASK));

	tmp_reg = npcm_readl(&dr_regs->portsc1);
	seq_printf(m,
		"USB Port Status&Control Reg:\n"
		"Port Transceiver Type : %s\n"
		"Port Speed: %s\n"
		"PHY Low Power Suspend: %s\n"
		"Port Reset: %s \n"
		"Port Suspend Mode: %s\n"
		"Over-current Change: %s\n"
		"Port Enable/Disable Change: %s\n"
		"Port Enabled/Disabled: %s\n"
		"Current Connect Status: %s\n\n", ( {
			const char *s;
			switch (tmp_reg & PORTSCX_PTS_FSLS) {
			case PORTSCX_PTS_UTMI:
				s = "UTMI"; break;
			case PORTSCX_PTS_ULPI:
				s = "ULPI "; break;
			case PORTSCX_PTS_FSLS:
				s = "FS/LS Serial"; break;
			default:
				s = "None"; break;
			}
			s;} ),
		usb_speed_string(portscx_device_speed(tmp_reg)),
		(tmp_reg & PORTSCX_PHY_LOW_POWER_SPD) ?
		"Low power mode": "Normal PHY mode",
		(tmp_reg & PORTSCX_PORT_RESET) ? "In Reset" :
		"Not in Reset",
		(tmp_reg & PORTSCX_PORT_SUSPEND) ? "In " : "Not in",
		(tmp_reg & PORTSCX_OVER_CURRENT_CHG) ? "Dected" :
		"No",
		(tmp_reg & PORTSCX_PORT_EN_DIS_CHANGE) ? "Disable" :
		"Not change",
		(tmp_reg & PORTSCX_PORT_ENABLE) ? "Enable" :
		"Not correct",
		(tmp_reg & PORTSCX_CURRENT_CONNECT_STATUS) ?
		"Attached" : "Not-Att");

	tmp_reg = npcm_readl(&dr_regs->usbmode);
	seq_printf(m,
			"USB Mode Reg = 0x%08X: Controller Mode is: %s\n\n", tmp_reg, ( {
				const char *s;
				switch (tmp_reg & USB_MODE_CTRL_MODE_HOST) {
				case USB_MODE_CTRL_MODE_IDLE:
					s = "Idle"; break;
				case USB_MODE_CTRL_MODE_DEVICE:
					s = "Device Controller"; break;
				case USB_MODE_CTRL_MODE_HOST:
					s = "Host Controller"; break;
				default:
					s = "None"; break;
				}
				s;
			} ));

	tmp_reg = npcm_readl(&dr_regs->endptsetupstat);
	seq_printf(m,
			"Endpoint Setup Status Reg: SETUP on ep 0x%x\n\n",
			(tmp_reg & EP_SETUP_STATUS_MASK));

	for (i = 0; i < udc->max_ep / 2; i++) {
		tmp_reg = npcm_readl(&dr_regs->endptctrl[i]);
		seq_printf(m, "EP Ctrl Reg [0x%x]: = [0x%x]\n",
				i, tmp_reg);
	}
	tmp_reg = npcm_readl(&dr_regs->endpointprime);
	seq_printf(m, "EP Prime Reg = [0x%x]\n\n", tmp_reg);

#if 0
#ifndef CONFIG_ARCH_MXC
	if (udc->pdata->have_sysif_regs) {
		tmp_reg = usb_sys_regs->snoop1;
		seq_printf(m, "Snoop1 Reg : = [0x%x]\n\n", tmp_reg);

		tmp_reg = usb_sys_regs->control;
		seq_printf(m, "General Control Reg : = [0x%x]\n\n",
				tmp_reg);
	}
#endif
#endif

	/* ------npcm_udc, npcm_ep, npcm_request structure information ----- */
	ep = &udc->eps[0];
	seq_printf(m, "For %s Maxpkt is 0x%x index is 0x%x\n",
			ep->ep.name, ep_maxpacket(ep), ep_index(ep));

	if (list_empty(&ep->queue)) {
		seq_printf(m, "its req queue is empty\n\n");
	} else {
		list_for_each_entry(req, &ep->queue, queue) {
			seq_printf(m,
				"req %p actual 0x%x length 0x%x buf %p\n",
				&req->req, req->req.actual,
				req->req.length, req->req.buf);
		}
	}
	/* other gadget->eplist ep */
	list_for_each_entry(ep, &udc->gadget.ep_list, ep.ep_list) {
		if (ep) {
			seq_printf(m,
					"\nFor %s Maxpkt is 0x%x "
					"index is 0x%x\n",
					ep->ep.name, ep_maxpacket(ep),
					ep->ep.desc? ep_index(ep): -1);

			if (list_empty(&ep->queue)) {
				seq_printf(m,
						"its req queue is empty\n\n");
			} else {
				list_for_each_entry(req, &ep->queue, queue) {
					seq_printf(m,
						"req %p actual 0x%x length "
						"0x%x  buf %p\n",
						&req->req, req->req.actual,
						req->req.length, req->req.buf);
					}	/* end for each_entry of ep req */
				}	/* end for else */
			}	/* end for if(ep->queue) */
		}		/* end (ep->desc) */

	spin_unlock_irqrestore(&udc->lock, flags);
	return 0;
}

/*
 * seq_file wrappers for procfile show routines.
 */
static int npcm_proc_open(struct inode *inode, struct file *file)
{
	return single_open(file, npcm_proc_read, PDE_DATA(file_inode(file)));
}

#define create_proc_file() \
	proc_create_single(proc_filename, 0, NULL, npcm_proc_read)
#define remove_proc_file()	remove_proc_entry(proc_filename, NULL)

#else				/* !CONFIG_USB_GADGET_DEBUG_FILES */

#define create_proc_file(udc)	do {} while (0)
#define remove_proc_file()	do {} while (0)

#endif				/* CONFIG_USB_GADGET_DEBUG_FILES */

/*-------------------------------------------------------------------------*/

/* Release udc structures */
static void npcm_udc_release(struct device *dev)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(dev->parent);

    if(!udc_controller)
        return;

    complete(udc_controller->done);
#ifndef NPCM_USB_DESC_PHYS_BASE_ADDR
    dma_free_coherent(dev->parent, udc_controller->ep_qh_size,
            udc_controller->ep_qh, udc_controller->ep_qh_dma);
#endif
    kfree(udc_controller);
    dev_set_drvdata(dev->parent, NULL);

}

/******************************************************************
	Internal structure setup functions
*******************************************************************/
/*------------------------------------------------------------------
 * init resource for globle controller
 * Return the udc handle on success or NULL on failure
 ------------------------------------------------------------------*/
static int struct_udc_setup(struct npcm_udc *udc,
		struct platform_device *pdev)
{
	struct npcm_usb2_platform_data *pdata;
	size_t size;

	pdata = dev_get_platdata(&pdev->dev);
	udc->phy_mode = pdata->phy_mode;

	udc->eps = kcalloc(udc->max_ep, sizeof(struct npcm_ep), GFP_KERNEL);
	if (!udc->eps) {
		NPCM_USB_ERR("malloc npcm_ep failed\n");
		return -1;
	}

	/* initialized QHs, take care of alignment */
	size = udc->max_ep * sizeof(struct ep_queue_head);
	if (size < QH_ALIGNMENT)
		size = QH_ALIGNMENT;
	else if ((size % QH_ALIGNMENT) != 0) {
		size += QH_ALIGNMENT + 1;
		size &= ~(QH_ALIGNMENT - 1);
	}
#ifdef NPCM_USB_DESC_PHYS_BASE_ADDR
	{
		void __iomem *addr = NULL;
		struct resource *res = NULL;
	  	res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	  	if (!res) {
			pr_err("platform_get_resource dtd error\n");
			return -ENODEV;
	  	}

		if (MINIMUM_NPCM_UDC_EPQ_DTD_SIZE > resource_size(res)) {
			pr_err("Minimum UDC epq dtd size below 0x800\n");
			return -ENODEV;
		}

		addr = devm_ioremap_resource(&pdev->dev, res);

		udc->ep_qh_dma = (dma_addr_t)res->start;
		udc->ep_qh = (void *)addr;
		udc->dtd_size = resource_size(res);
		VDBG("NPCM USB DESC phy 0x%x virt 0x%x size 0x%x\n",(int)res->start,(int)addr,(int)resource_size(res));
	}
#else
	udc->ep_qh = dma_alloc_coherent(&pdev->dev, size,
					&udc->ep_qh_dma, GFP_KERNEL);
	VDBG("size=%d udc->ep_qh = %p %08x udc->ep_qh_dma = %08x\n", size, udc->ep_qh, (u32)udc->ep_qh, (u32)udc->ep_qh_dma);
	if (!udc->ep_qh) {
		NPCM_USB_ERR("malloc QHs for udc failed\n");
		kfree(udc->eps);
		return -1;
	}
#endif

	udc->ep_qh_size = size;

	/* Initialize ep0 status request structure */
	/* FIXME: npcm_alloc_request() ignores ep argument */
	udc->status_req = container_of(npcm_alloc_request(NULL, GFP_KERNEL),
			struct npcm_req, req);
	/* allocate a small amount of memory to get valid address */
	udc->status_req->req.buf = kmalloc(8, GFP_KERNEL);

	udc->resume_state = USB_STATE_NOTATTACHED;
	udc->usb_state = USB_STATE_POWERED;
	udc->ep0_dir = 0;
	udc->remote_wakeup = 0;	/* default to 0 on reset */

	return 0;
}

/*----------------------------------------------------------------
 * Setup the npcm_ep struct for eps
 * Link npcm_ep->ep to gadget->ep_list
 * ep0out is not used so do nothing here
 * ep0in should be taken care
 *--------------------------------------------------------------*/
static int struct_ep_setup(struct npcm_udc *udc, unsigned char index,
		char *name, int link)
{
	struct npcm_ep *ep = &udc->eps[index];

	ep->udc = udc;
	
	strncpy(ep->name, name, 13);
	ep->name[13]='\0';
	
	ep->ep.name = ep->name;

	ep->ep.ops = &npcm_ep_ops;
	ep->stopped = 0;

	if (index == 0) {
		ep->ep.caps.type_control = true;
	} else {
		ep->ep.caps.type_iso = true;
		ep->ep.caps.type_bulk = true;
		ep->ep.caps.type_int = true;
	}

	if (index & 1)
		ep->ep.caps.dir_in = true;
	else
		ep->ep.caps.dir_out = true;

	/* for ep0: maxP defined in desc
	 * for other eps, maxP is set by epautoconfig() called by gadget layer
	 */
	usb_ep_set_maxpacket_limit(&ep->ep, (unsigned short) ~0);

	/* the queue lists any req for this ep */
	INIT_LIST_HEAD(&ep->queue);

	/* gagdet.ep_list used for ep_autoconfig so no ep0 */
	if (link)
		list_add_tail(&ep->ep.ep_list, &udc->gadget.ep_list);
	ep->gadget = &udc->gadget;
	ep->qh = &udc->ep_qh[index];

	return 0;
}

/* Driver probe function
 * all initialization operations implemented here except enabling usb_intr reg
 * board setup should have been done in the platform code
 */
static int npcm_udc_probe(struct platform_device *pdev)
{
	struct npcm_usb2_platform_data *pdata = NULL;
	struct device *dev = &pdev->dev;
	void __iomem *addr = NULL;
	struct device_node *np = pdev->dev.of_node;
	int ret = -ENODEV;
	unsigned int i;
	u32 dccparams;
    struct npcm_udc *udc_controller;
    struct usb_dr_device *dr_regs;
	struct resource *res = NULL;
	static int first_init = 1;

#ifdef CONFIG_OF
	pdev->id = of_alias_get_id(np, "udc");
	if (pdev->id < 0)
		pdev->id = 0;
#endif

#ifdef USB_DEVICE_9_WA
    if (pdev->id == 0)
    {
        DBG("Replace with USB9");
		npcm_udc_replace_usb9();
        pdev->id = 9;
    }
    else if (pdev->id == 9)
        pdev->id = 0;
#endif
    printk(KERN_DEBUG "npcm_udc_probe start, device = %d\n", pdev->id);

 	udc_controller = kzalloc(sizeof(struct npcm_udc), GFP_KERNEL);
	if (udc_controller == NULL) {
		NPCM_USB_ERR("malloc udc failed\n");
		return -ENOMEM;
	}

	udc_controller->id = pdev->id;

	if (gcr_regmap == NULL) {
		gcr_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-gcr");
		if (IS_ERR(gcr_regmap)) {
			pr_err("%s: failed to find nuvoton,npcm750-gcr\n", __func__);
			return IS_ERR(gcr_regmap);
	    }
	}

	if (rst_regmap == NULL) {
		rst_regmap = syscon_regmap_lookup_by_compatible("nuvoton,npcm750-rst");
		if (IS_ERR(rst_regmap)) {
			pr_err("%s: failed to find nuvoton,npcm750-rst\n", __func__);
			return IS_ERR(rst_regmap);
	    }
	}

	if (first_init == 1)
	{
	     printk(KERN_INFO "usb phy init\n");
	     ret = regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 24),(0x1 << 24));
	     regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 23), (0x1 << 23));
	     regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 22), (0x1 << 22));
	     regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 25), (0x1 << 25));
	     regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 8), (0x1 << 8));
	     regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 5), (0x1 << 5));

	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 8), (0x1 << 8));
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 7), (0x1 << 7));
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 6), (0x1 << 6));
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 5), (0x1 << 5));
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 4), (0x1 << 4));

	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 24), (0x1 << 24));
	     regmap_update_bits(gcr_regmap, USB1PHYCTL_OFFSET, (0x1 << 28), 0);
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 24), 0);

	     udelay(50); // enable phy

	     regmap_update_bits(gcr_regmap, USB1PHYCTL_OFFSET, (0x1 << 28), (0x1 << 28));
	     regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 8), 0);
	     first_init = 0;
	}

	// enable devices
	switch(udc_controller->id)
	{
	 case 0:
		 regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 4), 0);
		 break;
	 case 1:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 5), 0);
		 break;
	 case 2:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 8), 0);
		 break;
	 case 3:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 25), 0);
		 break;
	 case 4:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 22), 0);
		 break;
	 case 5:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 23), 0);
		 break;
	 case 6:
		 regmap_update_bits(rst_regmap, IPSRST1_OFFSET, (0x1 << 24), 0);
		 break;
	 case 7:
		 regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 5), 0);
		 break;
	 case 8:
		 regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 6), 0);
		 break;
	 case 9:
		 regmap_update_bits(gcr_regmap, INTCR3_OFFSET, NPCM_INTCR3_USBPHYSW, NPCM_INTCR3_USBPHYSW);
		 regmap_update_bits(rst_regmap, IPSRST3_OFFSET, (0x1 << 7), 0);
		 break;
	 default :
		 ret = -ENODEV;
		 pr_err("npcm_udc_enable_devices failed\n");
		 goto err_iounmap;
	}

#ifdef CONFIG_OF
  	res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
  	if (!res) {
		pr_err("platform_get_resource error\n");
		ret = -ENODEV;
		goto err_iounmap;
  	}
	addr = devm_ioremap_resource(dev, res);
	udc_controller->dr_regs = (struct usb_dr_device *)addr;
	pdev->dev.platform_data = &usb_data; /*T.M. problematic*/

	/*
	* Right now device-tree probed devices don't get dma_mask set.
	* Since shared usb code relies on it, set it here for now.
	* Once we have dma capability bindings this can go away.
	*/
	ret = dma_coerce_mask_and_coherent(&pdev->dev, DMA_BIT_MASK(32));
	if (ret)
	{
		pr_err("dma_coerce_mask_and_coherent error\n");
		ret = -ENODEV;
		goto err_iounmap;
	}
#else
    udc_controller->dr_regs = (struct usb_dr_device *)USB_BASE_ADDR(udc_controller->id);
#endif

    dr_regs = udc_controller->dr_regs;

    dev_set_drvdata(&pdev->dev, udc_controller); 
	pdata = dev_get_platdata(&pdev->dev);
	udc_controller->pdata = pdata;
	spin_lock_init(&udc_controller->lock);
	udc_controller->stopped = 1;
	udc_controller->vbus_active = 1;
    udc_controller->gadget.name = drv_20_name;

#ifdef CONFIG_USB_OTG
	if (pdata->operating_mode == NPCM_USB2_DR_OTG) {
		udc_controller->transceiver = usb_get_phy(USB_PHY_TYPE_USB2);
		if (IS_ERR_OR_NULL(udc_controller->transceiver)) {
			ERR("Can't find OTG driver!\n");
			ret = -ENODEV;
			goto err_kfree;
		}
	}
#endif
	/* Set accessors only after pdata->init() ! */
	npcm_set_accessors(pdata);

	/* Initialize USB clocks */
	ret = npcm_udc_clk_init(pdev);
	if (ret < 0)
		goto err_iounmap_noclk;

	/* Read Device Controller Capability Parameters register */
	dccparams = npcm_readl(&dr_regs->dccparams);
	if (!(dccparams & DCCPARAMS_DC)) {
		NPCM_USB_ERR("This SOC doesn't support device role\n");
		ret = -ENODEV;
		goto err_iounmap;
	}
	/* Get max device endpoints */
	/* DEN is bidirectional ep number, max_ep doubles the number */
	udc_controller->max_ep = (dccparams & DCCPARAMS_DEN_MASK) * 2;
 
#ifdef CONFIG_OF
  	udc_controller->irq = platform_get_irq(pdev, 0);
  	if (udc_controller->irq < 0) {
		  pr_err("platform_get_irq error.");
		  return -ENODEV;
  	}
#else	
  	udc_controller->irq = USB_INTERRUPT(udc_controller->id);
#endif

	ret = request_irq(udc_controller->irq, npcm_udc_irq, IRQF_SHARED,
			udc_controller->gadget.name, udc_controller);
	if (ret != 0) {
		NPCM_USB_ERR("cannot request irq %d err %d\n",
				udc_controller->irq, ret);
		goto err_iounmap;
	}

	/* Initialize the udc structure including QH member and other member */
	if (struct_udc_setup(udc_controller, pdev)) {
		NPCM_USB_ERR("Can't initialize udc data structure\n");
		ret = -ENOMEM;
		goto err_free_irq;
	}

	if (IS_ERR_OR_NULL(udc_controller->transceiver)) {
		/* initialize usb hw reg except for regs for EP,
		 * leave usbintr reg untouched */
		dr_controller_setup(udc_controller);
	}

	npcm_udc_clk_finalize(pdev);
	
	/*ret = npcm_udc_clk_finalize(pdev);
	if (ret)
		goto err_free_irq;*/

	/* Setup gadget structure */
	udc_controller->gadget.ops = &npcm_gadget_ops;
	udc_controller->gadget.max_speed = USB_SPEED_HIGH;
	udc_controller->gadget.ep0 = &udc_controller->eps[0].ep;
	INIT_LIST_HEAD(&udc_controller->gadget.ep_list);
	udc_controller->gadget.speed = USB_SPEED_UNKNOWN;
    if(!strcmp(pdev->name, drv_20_name))
    {
	   udc_controller->gadget.name = drv_20_name;
    }

	/* Setup gadget.dev and register with kernel */
	dev_set_name(&udc_controller->gadget.dev, "gadget");
   	udc_controller->gadget.dev.of_node = pdev->dev.of_node;

	if (!IS_ERR_OR_NULL(udc_controller->transceiver))
		udc_controller->gadget.is_otg = 1;

	/* setup QH and epctrl for ep0 */
	ep0_setup(udc_controller);

	/* setup udc->eps[] for ep0 */
	struct_ep_setup(udc_controller, 0, "ep0", 0);
	/* for ep0: the desc defined here;
	 * for other eps, gadget layer called ep_enable with defined desc
	 */
	udc_controller->eps[0].ep.desc = &npcm_ep0_desc;
	usb_ep_set_maxpacket_limit(&udc_controller->eps[0].ep,
				   USB_MAX_CTRL_PAYLOAD);

	/* setup the udc->eps[] for non-control endpoints and link
	 * to gadget.ep_list */
	for (i = 1; i < (int)(udc_controller->max_ep / 2); i++) {
		char name[14];

		sprintf(name, "ep%dout", i);
		struct_ep_setup(udc_controller, i * 2, name, 1);
		sprintf(name, "ep%din", i);
		struct_ep_setup(udc_controller, i * 2 + 1, name, 1);
	}

	/* use dma_pool for TD management */
#ifdef NPCM_USB_DESC_PHYS_BASE_ADDR
    {
        int size_of_queue_heads;
        int td_count;
        struct ep_td_struct *dtd;
        
        size_of_queue_heads = sizeof(struct ep_queue_head) * udc_controller->max_ep;
        udc_controller->dtd_phys_ba = (u32)udc_controller->ep_qh_dma + size_of_queue_heads;
        udc_controller->dtd_virt_ba = (u32)udc_controller->ep_qh + size_of_queue_heads;
		udc_controller->dtd_max_pool = ((udc_controller->dtd_size - size_of_queue_heads) / (2 * DTD_ALIGNMENT));
		VDBG("udc_controller->dtd_phys_ba 0x%x , udc_controller->dtd_virt_ba 0x%x udc_controller->dtd_max_pool %d\n",udc_controller->dtd_phys_ba,
				udc_controller->dtd_virt_ba,udc_controller->dtd_max_pool);
        for (td_count=0; td_count < udc_controller->dtd_max_pool; td_count++)
        {
            dtd = (void *)(udc_controller->dtd_virt_ba + 2*DTD_ALIGNMENT * td_count);
            dtd->res = DTD_IS_FREE;          
        }
    }
#else
    {   
        char td_name[32];
        sprintf(td_name, "td_%s", udc_controller->gadget.name);
        udc_controller->td_pool = dma_pool_create(td_name, &pdev->dev,
                sizeof(struct ep_td_struct),
                DTD_ALIGNMENT, UDC_DMA_BOUNDARY);
        if (udc_controller->td_pool == NULL) {
            ret = -ENOMEM;
            goto err_free_irq;
        }
    }
#endif    

	ret = usb_add_gadget_udc_release(&pdev->dev, &udc_controller->gadget,
			npcm_udc_release);	
if (ret)
		goto err_del_udc;
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
    snprintf(proc_filename, sizeof(proc_filename), "%s.%d", PROC_FILENAME, udc_controller->id );
#endif
    create_proc_file(udc_controller);
    printk(KERN_DEBUG "npcm_udc_probe end\n");
	return 0;

err_del_udc:
#ifndef NPCM_USB_DESC_PHYS_BASE_ADDR
	dma_pool_destroy(udc_controller->td_pool);
#endif
err_free_irq:
	free_irq(udc_controller->irq, udc_controller);
err_iounmap:
	if (pdata && pdata->exit)
		pdata->exit(pdev);
	npcm_udc_clk_release();
err_iounmap_noclk:
	//iounmap(dr_regs);

	kfree(udc_controller);
	return ret;
}

/* Driver removal function
 * Free resources and finish pending transactions
 */
static int npcm_udc_remove(struct platform_device *pdev)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(&pdev->dev);
	struct npcm_usb2_platform_data *pdata = dev_get_platdata(&pdev->dev);

	DECLARE_COMPLETION_ONSTACK(done);

	if (!udc_controller)
		return -ENODEV;
	
	dr_controller_stop(udc_controller);

	udc_controller->done = &done;
	usb_del_gadget_udc(&udc_controller->gadget);


	npcm_udc_clk_release();

	/* DR has been stopped in usb_gadget_unregister_driver() */
#ifdef CONFIG_USB_GADGET_DEBUG_FILES
    snprintf(proc_filename, sizeof(proc_filename), "%s.%d", PROC_FILENAME, udc_controller->id );
#endif
	remove_proc_file();

	/* Free allocated memory */
	kfree(udc_controller->status_req->req.buf);
	kfree(udc_controller->status_req);
	kfree(udc_controller->eps);

#ifdef NPCM_USB_DESC_PHYS_BASE_ADDR
    {
        int td_count;
        struct ep_td_struct *dtd;
        
        for (td_count=0; td_count < udc_controller->dtd_max_pool; td_count++)
        {
            dtd = (void *)(udc_controller->dtd_virt_ba + 2*DTD_ALIGNMENT * td_count);
            dtd->res = DTD_IS_FREE;          
        }
    }

#else
    dma_pool_destroy(udc_controller->td_pool);
#endif

	free_irq(udc_controller->irq, udc_controller);

	/* free udc --wait for the release() finished */
	wait_for_completion(&done);

	/*
	 * do platform specific un-initialization:
	 * release iomux pins, etc.
	 */
	if (pdata->exit)
		pdata->exit(pdev);

	return 0;
}

/*-----------------------------------------------------------------
 * Modify Power management attributes
 * Used by OTG statemachine to disable gadget temporarily
 -----------------------------------------------------------------*/
static int npcm_udc_suspend(struct platform_device *pdev, pm_message_t state)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(&pdev->dev);

    if (!udc_controller)
       return -ENODEV;

    dr_controller_stop(udc_controller);
	return 0;
}

/*-----------------------------------------------------------------
 * Invoked on USB resume. May be called in_interrupt.
 * Here we start the DR controller and enable the irq
 *-----------------------------------------------------------------*/
static int npcm_udc_resume(struct platform_device *pdev)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(&pdev->dev);

    if (!udc_controller)
       return -ENODEV;

	/* Enable DR irq reg and set controller Run */
	if (udc_controller->stopped) {
		dr_controller_setup(udc_controller);
		dr_controller_run(udc_controller);
	}
	udc_controller->usb_state = USB_STATE_ATTACHED;
	udc_controller->ep0_state = WAIT_FOR_SETUP;
	udc_controller->ep0_dir = 0;
	return 0;
}

static int npcm_udc_otg_suspend(struct device *dev, pm_message_t state)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(dev);
	struct npcm_udc *udc;
	u32 mode, usbcmd;
    struct usb_dr_device *dr_regs;

    if (!udc_controller)
       return -ENODEV;

	udc = udc_controller;
	
    dr_regs = udc->dr_regs;

	mode = npcm_readl(&dr_regs->usbmode) & USB_MODE_CTRL_MODE_MASK;

	pr_debug("%s(): mode 0x%x stopped %d\n", __func__, mode, udc->stopped);

	/*
	 * If the controller is already stopped, then this must be a
	 * PM suspend.  Remember this fact, so that we will leave the
	 * controller stopped at PM resume time.
	 */
	if (udc->stopped) {
		pr_debug("gadget already stopped, leaving early\n");
		udc->already_stopped = 1;
		return 0;
	}

	if (mode != USB_MODE_CTRL_MODE_DEVICE) {
		pr_debug("gadget not in device mode, leaving early\n");
		return 0;
	}

	/* stop the controller */
	usbcmd = npcm_readl(&dr_regs->usbcmd) & ~USB_CMD_RUN_STOP;
	npcm_writel(usbcmd, &dr_regs->usbcmd);

	udc->stopped = 1;

	pr_info("USB Gadget suspended\n");

	return 0;
}

static int npcm_udc_otg_resume(struct device *dev)
{
    struct npcm_udc *udc_controller = dev_get_drvdata(dev);

	pr_debug("%s(): stopped %d  already_stopped %d\n", __func__,
		 udc_controller->stopped, udc_controller->already_stopped);

	/*
	 * If the controller was stopped at suspend time, then
	 * don't resume it now.
	 */
	if (udc_controller->already_stopped) {
		udc_controller->already_stopped = 0;
		pr_debug("gadget was already stopped, leaving early\n");
		return 0;
	}

	pr_info("USB Gadget resume\n");

	return npcm_udc_resume(NULL);
}

/*-------------------------------------------------------------------------
	Register entry point for the peripheral controller driver
--------------------------------------------------------------------------*/

struct bus_type usb_udc_bus_type = {
        .name           = "usb",
};

static const struct of_device_id nuvoton_udc_of_match[] = {
	{ .compatible = "nuvoton,npcm750-udc", },
};
MODULE_DEVICE_TABLE(of, nuvoton_udc_of_match);

static struct platform_driver udc_20_driver = {
	.remove		= npcm_udc_remove,
	/* Just for npcm i.mx SoC currently */
	/* these suspend and resume are not usb suspend and resume */
	.suspend	= npcm_udc_suspend,
	.resume		= npcm_udc_resume,
	#ifdef CONFIG_OF
	.probe      = npcm_udc_probe,
	#endif
	.driver		= {
			.name = drv_20_name,
			.owner = THIS_MODULE,
			/* udc suspend/resume called from OTG driver */
			.suspend = npcm_udc_otg_suspend,
			.resume  = npcm_udc_otg_resume,
			.of_match_table = nuvoton_udc_of_match,
	},
};

#ifndef CONFIG_OF

struct platform_device usb_platform_device[] = {
    {
        .name       = (char *) drv_20_name,
        .id         = 0,
        .dev = {
                .dma_mask              = &usb_platform_device[0].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 1,
        .dev = {
                .dma_mask              = &usb_platform_device[1].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 2,
        .dev = {
                .dma_mask              = &usb_platform_device[2].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 3,
        .dev = {
                .dma_mask              = &usb_platform_device[3].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 4,
        .dev = {
                .dma_mask              = &usb_platform_device[4].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 5,
        .dev = {
                .dma_mask              = &usb_platform_device[5].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 6,
        .dev = {
                .dma_mask              = &usb_platform_device[6].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 7,
        .dev = {
                .dma_mask              = &usb_platform_device[7].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 8,
        .dev = {
                .dma_mask              = &usb_platform_device[8].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
    {
        .name       = (char *) drv_20_name,
        .id         = 9,
        .dev = {
                .dma_mask              = &usb_platform_device[9].dev.coherent_dma_mask,
                .coherent_dma_mask = 0xffffffffUL, 
                .platform_data         = &usb_data,
                .release               = npcm_udc_release,
        },
    },
};

static int __init udc_init(void)
{
    volatile int result = 0;
    int i;
	
	printk(KERN_INFO "%s (%s)\n", driver_desc, DRIVER_VERSION);

    // register platform devices
    for (i = 0; i < NPCM_NUM_DEVICES; i++) {
        platform_device_register(&usb_platform_device[i]);

    }


//    result =  bus_register(&usb_udc_bus_type);

    udc_20_driver.driver.bus = &usb_udc_bus_type;

    result =  platform_driver_probe(&udc_20_driver, npcm_udc_probe);

    return result;
}

module_init(udc_init);

static void __exit udc_exit(void)
{
    int i;

    // unregister platform devices
    for (i = 0; i < NPCM_NUM_DEVICES; i++) {
        platform_device_unregister(&usb_platform_device[i]);
    }

    platform_driver_unregister(&udc_20_driver);

	printk(KERN_WARNING "%s unregistered\n", driver_desc);
}

module_exit(udc_exit);
#else
module_platform_driver(udc_20_driver);
#endif

MODULE_DESCRIPTION(DRIVER_DESC);
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:npcm-udc");
