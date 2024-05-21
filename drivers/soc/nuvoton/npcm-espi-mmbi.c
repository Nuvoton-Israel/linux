// SPDX-License-Identifier: GPL-2.0
/*
 * Copyright (c) 2021 Intel Corporation.
 * Copyright (c) 2024 Nuvoton Technology corporation.
 */

#include <linux/bitfield.h>
#include <linux/crc8.h>
#include <linux/dma-mapping.h>
#include <linux/interrupt.h>
#include <linux/io.h>
#include <linux/mfd/syscon.h>
#include <linux/miscdevice.h>
#include <linux/module.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#include <linux/platform_device.h>
#include <linux/poll.h>
#include <linux/regmap.h>
#include <dt-bindings/mmbi/protocols.h>

#define DEVICE_NAME "mmbi"
#define MAX_NO_OF_SUPPORTED_CHANNELS 1
#define MAX_NO_OF_SUPPORTED_PROTOCOLS 5

#define POLLING_INTERVAL_MS	50
#define POLLING_TIMEOUT_MS	1000
#define NPCM_ESPI_VWEVSM2	0x108
#define   WIRE(x)		BIT(x)
#define   HW_WIRE(x)		(BIT(x) << 24)
#define NPCM_MAX_WIN_SIZE	4096
#define SHM_SMC_STS	0x0
#define   SHM_ACC	BIT(6)
#define SHM_SMC_CTL	0x1
#define   SHM_ACC_IE	BIT(5)
#define SHM_WIN_SIZE	0x7
#define SHM_WIN_BASE1	0x20
#define SHM_HOFS_STS	0x48
#define   HOFS1W	BIT(1)
#define SHM_HOFS_CTL	0x49
#define   HOFS1W_IE	BIT(1)

/* 20 Bits for H2B/B2H Write/Read Pointers */
#define H2B_WRITE_POINTER_MASK GENMASK(19, 0)
#define B2H_READ_POINTER_MASK GENMASK(19, 0)
#define MMBI_HDR_LENGTH_MASK GENMASK(23, 0)
#define MMBI_HDR_TYPE_MASK GENMASK(31, 24)
#define HOST_RESET_REQUEST_BIT BIT(31)
#define HOST_READY_BIT BIT(31)
#define ESPI_SCI_STATUS_BIT BIT(16)

#define GET_H2B_WRITE_POINTER(x) ((x) & H2B_WRITE_POINTER_MASK)
#define GET_B2H_READ_POINTER(x) ((x) & B2H_READ_POINTER_MASK)
#define GET_HOST_RESET_REQ_BIT(x) ((x) & HOST_RESET_REQUEST_BIT)
#define GET_HOST_READY_BIT(x) ((x) & HOST_READY_BIT)

#define MMBI_CRC8_POLYNOMIAL 0x07
DECLARE_CRC8_TABLE(mmbi_crc8_table);

typedef u8 protocol_type;

struct host_rop {
	unsigned int
		b2h_wp : 20; /* Offset where BMC can write next data in B2H */
	unsigned int reserved1 : 11;
	unsigned int b_rdy : 1; /* BMC ready bit */
	unsigned int h2b_rp : 20; /* Offset till where bmc read data in H2B */
	unsigned int reserved2 : 11;
	unsigned int b_rst : 1; /* BMC reset request bit */
};

struct host_rwp {
	unsigned int
		h2b_wp : 20; /* Offset where HOST can write next data in H2B */
	unsigned int reserved1 : 11;
	unsigned int h_rdy : 1; /* Host ready bit */
	unsigned int b2h_rp : 20; /* Offset till where host read data in B2H */
	unsigned int reserved2 : 11;
	unsigned int h_rst : 1; /* host reset request bit */
};

struct buffer_type_desc {
	u32 host_rop_p;
	u32 host_rwp_p;
	u8 msg_protocol_type;
	u8 host_int_type;
	u16 global_sys_interrupt;
	u8 bmc_int_type;
	u32 bmc_int_a;
	u8 bmc_int_v;
} __packed;

struct mmbi_cap_desc {
	u8 signature[6];
	u8 version;
	u8 instance_num;
	u32 nex_inst_base_addr;
	u32 b2h_ba; /* B2H buffer base offset */
	u32 h2b_ba; /* H2B buffer base offset */
	u16 b2h_d; /* Multiple of 16 Bytes (Max 1MB) */
	u16 h2b_d; /* multiples of 16 bytes (Max 1MB) */
	u8 buffer_type; /* Type of buffer in B2H/H2B */
	u8 reserved1[7];
	struct buffer_type_desc bt_desc; /* 18 bytes */
	u8 reserved2[13];
	u8 crc8; /* CRC-8-CCITT of the whole data structure (bytes 0 to 62) */
} __packed;

struct mmbi_header {
	u32 data;
};

struct npcm_mmbi_protocol {
	struct miscdevice miscdev;
	struct npcm_mmbi_channel *chan_ref;
	protocol_type type;

	bool data_available;
	/*
	 * If user space application is opened for read, then only process
	 * the data and copy to userspace. Otherwise, discard the command and
	 * process the remaining commands (can be different protocol type)
	 */
	bool process_data;
	wait_queue_head_t queue;
};

struct npcm_mmbi_channel {
	struct npcm_mmbi_protocol protocol[MAX_NO_OF_SUPPORTED_PROTOCOLS];
	struct npcm_espi_mmbi *priv;

	u8 chan_num;
	u8 supported_protocols[MAX_NO_OF_SUPPORTED_PROTOCOLS];
	u32 b2h_cb_size;
	u32 h2b_cb_size;
	u8 *desc_vmem;
	u8 *hrop_vmem;
	u8 *b2h_cb_vmem;
	u8 *hrwp_vmem;
	u8 *h2b_cb_vmem;
	u32 cur_h2b_wp;
	bool enabled;
};

struct npcm_espi_mmbi {
	struct device *dev;
	void __iomem *regs;
	struct regmap *pmap;
	int irq;
	dma_addr_t mmbi_phys_addr;
	resource_size_t mmbi_size;
	u8 __iomem *shm_vaddr;
	struct delayed_work work;
	struct workqueue_struct *wq;
	struct mutex lock;

	struct npcm_mmbi_channel chan[MAX_NO_OF_SUPPORTED_CHANNELS];
	u32 poll_interval;
};

struct npcm_mmbi_get_empty_space {
	u32 length;
};

struct npcm_mmbi_get_config {
	bool h_rdy;
	u32 h2b_wp;
	u32 b2h_rp;
	u32 h2b_rp;
	u32 b2h_wp;
};

#define __NPCM_MMBI_CTRL_IOCTL_MAGIC 0xBB
/*
 * This IOCTL is meant to read empty space in B2H buffer
 * in a specific channel
 */
#define NPCM_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE	\
	_IOWR(__NPCM_MMBI_CTRL_IOCTL_MAGIC, 0x00,	\
	      struct npcm_mmbi_get_empty_space)

/* This IOCTL to send BMC reset request */
#define NPCM_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST		\
	_IOW(__NPCM_MMBI_CTRL_IOCTL_MAGIC, 0x01, int)

/* This IOCTL is to Get Config in HROP and HRWP */
#define NPCM_MMBI_CTRL_IOCTL_GET_CONFIG			\
	_IOWR(__NPCM_MMBI_CTRL_IOCTL_MAGIC, 0x02,	\
	      struct npcm_mmbi_get_config)

static void npcm_mmbi_enable_interrupt(struct npcm_espi_mmbi *priv,
				       bool enable)
{
	u8 offset = SHM_HOFS_CTL;
	u8 val;

	val = readb(priv->regs + offset);
	if (enable)
		val |= HOFS1W_IE;
	else
		val &= ~HOFS1W_IE;
	writeb(val, priv->regs + offset);
}

static void raise_sci_interrupt(struct npcm_mmbi_channel *channel)
{
	struct npcm_espi_mmbi *priv = channel->priv;
	struct regmap *espi_regmap = priv->pmap;
	u32 val;
	int ret;

	dev_dbg(priv->dev, "Raising SCI interrupt...\n");
	regmap_set_bits(espi_regmap, NPCM_ESPI_VWEVSM2, WIRE(0));
	regmap_clear_bits(espi_regmap, NPCM_ESPI_VWEVSM2, WIRE(0));

	ret = regmap_read_poll_timeout_atomic(priv->pmap, NPCM_ESPI_VWEVSM2,
					      val, !(val & ESPI_SCI_STATUS_BIT),
					      1, 30);
	/* Clear SCI interrupt */
	regmap_set_bits(espi_regmap, NPCM_ESPI_VWEVSM2, WIRE(0));
	if (ret)
		dev_dbg(priv->dev, "Host SCI handler not invoked after 30us\n");
}

static int read_host_rwp_val(struct npcm_mmbi_channel *channel, u32 offset,
			     u32 *val)
{
	*val = readl(channel->hrwp_vmem + offset);

	return 0;
}

static int get_b2h_avail_buf_len(struct npcm_mmbi_channel *channel,
				 ssize_t *avail_buf_len)
{
	struct host_rop hrop;
	u32 b2h_rp, h_rwp1;

	if (read_host_rwp_val(channel, 4, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	b2h_rp = GET_B2H_READ_POINTER(h_rwp1);
	dev_dbg(channel->priv->dev, "MMBI HRWP - b2h_rp: 0x%0x\n", b2h_rp);

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev, "HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x",
		hrop.b2h_wp, hrop.h2b_rp);

	if (hrop.b2h_wp >= b2h_rp)
		*avail_buf_len = channel->b2h_cb_size - hrop.b2h_wp + b2h_rp;
	else
		*avail_buf_len = b2h_rp - hrop.b2h_wp;

	return 0;
}

/*
 * data_length = multi-protocl packet length
 * unread_data_len = filled length in circular buffer
 */
static int get_mmbi_header(struct npcm_mmbi_channel *channel,
			   u32 *data_length, u8 *type, u32 *unread_data_len)
{
	u32 h2b_wp, b2h_rp, h_rwp0, h_rwp1;
	struct mmbi_header header;
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev,
		"MMBI HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", hrop.b2h_wp,
		hrop.h2b_rp);

	if (read_host_rwp_val(channel, 0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	if (read_host_rwp_val(channel, 4, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	h2b_wp = GET_H2B_WRITE_POINTER(h_rwp0);
	b2h_rp = GET_B2H_READ_POINTER(h_rwp1);
	dev_dbg(channel->priv->dev, "MMBI HRWP - h2b_wp: 0x%0x, b2h_rp: 0x%0x\n", h2b_wp, b2h_rp);

	if (h2b_wp >= hrop.h2b_rp)
		*unread_data_len = h2b_wp - hrop.h2b_rp;
	else
		*unread_data_len = channel->h2b_cb_size - hrop.h2b_rp + h2b_wp;

	if (*unread_data_len < sizeof(struct mmbi_header)) {
		dev_dbg(channel->priv->dev, "No data to read(%d -%d)\n", h2b_wp,
			hrop.h2b_rp);
		return -EAGAIN;
	}

	dev_dbg(channel->priv->dev, "READ MMBI header from: 0x%p\n",
		(channel->h2b_cb_vmem + hrop.h2b_rp));

	/* Extract MMBI protocol - protocol type and length */
	if ((hrop.h2b_rp + sizeof(header)) <= channel->h2b_cb_size) {
		memcpy(&header, channel->h2b_cb_vmem + hrop.h2b_rp,
		       sizeof(header));
	} else {
		ssize_t chunk_len = channel->h2b_cb_size - hrop.h2b_rp;

		memcpy(&header, channel->h2b_cb_vmem + hrop.h2b_rp, chunk_len);
		memcpy(((u8 *)&header) + chunk_len, channel->h2b_cb_vmem,
		       sizeof(header) - chunk_len);
	}

	*data_length = FIELD_GET(MMBI_HDR_LENGTH_MASK, header.data);
	*type = FIELD_GET(MMBI_HDR_TYPE_MASK, header.data);

	return 0;
}

static void raise_missing_sci(struct npcm_mmbi_channel *channel)
{
	struct host_rop hrop;
	u32 h_rwp0, h_rwp1, b2h_rp;

	/* Rise SCI only if Host is READY (h_rdy is 1). */
	if (read_host_rwp_val(channel, 0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return;
	}
	if (!GET_HOST_READY_BIT(h_rwp0)) {
		/* Host is not ready, no point in raising the SCI */
		return;
	}

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	if (read_host_rwp_val(channel, 4, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return;
	}
	b2h_rp = GET_B2H_READ_POINTER(h_rwp1);

	if (hrop.b2h_wp == b2h_rp) {
		/*
		 * Host has read all outstanding SCI data,
		 * Do not raise another SCI.
		 */
		return;
	}

	dev_dbg(channel->priv->dev,
		"Host not read the data yet, so rising SCI interrupt again...\n");
	raise_sci_interrupt(channel);
}

static void update_host_rop(struct npcm_mmbi_channel *channel,
			    unsigned int w_len, unsigned int r_len)
{
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	dev_dbg(channel->priv->dev,
		"MMBI HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n", hrop.b2h_wp,
		hrop.h2b_rp);

	/* Advance the B2H CB offset for next write */
	if ((hrop.b2h_wp + w_len) <= channel->b2h_cb_size)
		hrop.b2h_wp += w_len;
	else
		hrop.b2h_wp = hrop.b2h_wp + w_len - channel->b2h_cb_size;

	/* Advance the H2B CB offset till where BMC read data */
	if ((hrop.h2b_rp + r_len) <= channel->h2b_cb_size)
		hrop.h2b_rp += r_len;
	else
		hrop.h2b_rp = hrop.h2b_rp + r_len - channel->h2b_cb_size;

	/*
	 * Clear BMC reset request state its set:
	 * Set BMC reset request bit to 0
	 * Set BMC ready bit to 1
	 */
	if (hrop.b_rst) {
		dev_dbg(channel->priv->dev,
			"Clearing BMC reset request state\n");
		hrop.b_rst = 0;
		hrop.b_rdy = 1;
	}

	dev_dbg(channel->priv->dev,
		"Updating HROP - b2h_wp: 0x%0x, h2b_rp: 0x%0x\n",
		hrop.b2h_wp, hrop.h2b_rp);
	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));

	/*
	 * Raise SCI interrupt only if B2H buffer is updated
	 * Don't raise SCI, after BMC read the H2B buffer
	 */
	if (w_len != 0)
		raise_sci_interrupt(channel);
}

static int send_bmc_reset_request(struct npcm_mmbi_channel *channel)
{
	struct host_rop hrop;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	/*
	 * Send MMBI buffer reset request: First BMC should clear its own
	 * pointers, set Reset bit and reset BMC ready bit.
	 * B2H Write pointer - must be set to zero
	 * H2B read pointer - must be set to zero
	 * BMC ready bit - Set to 0
	 * BMC reset bit - Set to 1
	 */
	hrop.b2h_wp = 0;
	hrop.h2b_rp = 0;
	hrop.b_rdy = 0;
	hrop.b_rst = 1;

	dev_info(channel->priv->dev,
		 "Send BMC reset request on MMBI channel(%d)\n",
		 channel->chan_num);

	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));

	/* Raise SCI interrupt */
	raise_sci_interrupt(channel);

	return 0;
}

static bool check_host_reset_request(struct npcm_mmbi_channel *channel)
{
	struct host_rop hrop;
	u32 h_rwp1;

	if (read_host_rwp_val(channel, 4, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return false;
	}

	/* If its not host reset request, just discard */
	if (!GET_HOST_RESET_REQ_BIT(h_rwp1))
		return false;

	/*  Host requested for MMBI buffer reset */
	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	/*
	 * When host request for reset MMBI buffer:
	 * B2H Write pointer - must be set to zero
	 * H2B read pointer - must be set to zero
	 * BMC ready bit - No change (Set to 1)
	 * BMC reset bit - No change (Set to 0)
	 */
	hrop.b2h_wp = 0;
	hrop.h2b_rp = 0;
	channel->cur_h2b_wp = 0;

	dev_info(channel->priv->dev,
		 "Handle Host reset request on MMBI channel(%d)\n",
		 channel->chan_num);

	memcpy(channel->hrop_vmem, &hrop, sizeof(hrop));

	return true;
}

void wake_up_device(struct npcm_mmbi_channel *channel)
{
	u32 req_data_len, unread_data_len;
	u8 type;
	int i;

	if (0 !=
	    get_mmbi_header(channel, &req_data_len, &type, &unread_data_len)) {
		/* Bail out as we can't read header */
		return;
	}
	dev_dbg(channel->priv->dev, "%s: Length: 0x%0x, Protocol Type: %d\n",
		__func__, req_data_len, type);

	for (i = 0; channel->supported_protocols[i] != 0; i++) {
		if (type == channel->supported_protocols[i]) {
			/*
			 * MMBI supports multiple protocols on each channel
			 * If userspace application is not opened the device
			 * for read /write the data, discard the data and
			 * advance the HROP for processing next command.
			 */
			if (channel->protocol[i].process_data) {
				channel->protocol[i].data_available = true;
				wake_up(&channel->protocol[i].queue);
			} else {
				/* Discard data and advance the hrop */
				update_host_rop(channel, 0,
						req_data_len +
						sizeof(struct mmbi_header));
			}
			/*
			 * Raise the missing SCI's by checking pointer for host
			 * read acknowledgment. This will work around the Missing
			 * SCI bug on host side.
			 */
			dev_warn(channel->priv->dev,
				 "%s: Check and raise missing SCI\n", __func__);
			raise_missing_sci(channel);
		}
	}
}

static struct npcm_mmbi_protocol *file_npcm_espi_mmbi(struct file *file)
{
	return container_of(file->private_data, struct npcm_mmbi_protocol,
			    miscdev);
}

static int mmbi_open(struct inode *inode, struct file *filp)
{
	return 0;
}

static int mmbi_release(struct inode *inode, struct file *filp)
{
	return 0;
}

static unsigned int mmbi_poll(struct file *filp, poll_table *wait)
{
	struct npcm_mmbi_protocol *protocol = file_npcm_espi_mmbi(filp);

	poll_wait(filp, &protocol->queue, wait);

	return protocol->data_available ? POLLIN : 0;
}

static ssize_t mmbi_read(struct file *filp, char *buff, size_t count,
			 loff_t *offp)
{
	struct npcm_mmbi_protocol *protocol = file_npcm_espi_mmbi(filp);
	struct npcm_mmbi_channel *channel = protocol->chan_ref;
	struct npcm_espi_mmbi *priv = channel->priv;
	struct host_rop hrop;
	ssize_t rd_offset, rd_len;
	ssize_t ret;
	u32 unread_data_len, req_data_len;
	u8 type;

	protocol->process_data = true;

	if (!protocol->data_available && (filp->f_flags & O_NONBLOCK)) {
		dev_dbg(priv->dev, "%s: Non blocking file\n", __func__);
		/*
		 * Work around: The lack of response might be cause by missing SCI
		 * (host didn't consume the last message), check the buffer state
		 * and retry if it's needed
		 */
		raise_missing_sci(channel);
		return -EAGAIN;
	}
	dev_dbg(priv->dev, "%s: count:%ld, Type: %d\n", __func__, count,
		protocol->type);

	ret = wait_event_interruptible(protocol->queue,
				       protocol->data_available);

	mutex_lock(&priv->lock);
	if (ret == -ERESTARTSYS) {
		ret = -EINTR;
		goto err_out;
	}

	if (*offp >= channel->h2b_cb_size) {
		ret = -EINVAL;
		goto err_out;
	}

	if (*offp + count > channel->h2b_cb_size)
		count = channel->h2b_cb_size - *offp;

	ret = get_mmbi_header(channel, &req_data_len, &type, &unread_data_len);
	if (ret != 0) {
		/* Bail out as we can't read header. */
		goto err_out;
	}
	dev_dbg(priv->dev,
		"%s: Length: %d, Protocol Type: %d, Unread data: %d\n",
		__func__, req_data_len, type, unread_data_len);

	if (req_data_len > count) {
		dev_err(priv->dev, "Data exceeding user space limit: %ld\n", count);
		ret = -EFAULT;
		/* Discard data and advance the hrop */
		update_host_rop(channel, 0, req_data_len + sizeof(struct mmbi_header));
		goto err_out;
	}

	/* Check is data belongs to this device, if not wake_up corresponding device. */
	if (type != protocol->type) {
		dev_err(priv->dev, "type != protocol->type\n");
		ret = -EFAULT;
		goto err_out;
	}

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	if ((hrop.h2b_rp + sizeof(struct mmbi_header)) <=
	    channel->h2b_cb_size) {
		rd_offset = hrop.h2b_rp + sizeof(struct mmbi_header);
	} else {
		rd_offset = hrop.h2b_rp + sizeof(struct mmbi_header) -
			    channel->h2b_cb_size;
	}
	rd_len = req_data_len;

	/* Extract data and copy to user space application */
	dev_dbg(priv->dev, "READ MMBI Data from: offset 0x%lx and length: %ld\n",
		rd_offset, rd_len);
	if (unread_data_len < sizeof(struct mmbi_header) + rd_len) {
		dev_err(priv->dev, "Invalid H2B buffer (Read msg length: %ld)\n",
			rd_len);
		ret = -EFAULT;
		goto err_out;
	}

	if ((channel->h2b_cb_size - rd_offset) >= rd_len) {
		if (copy_to_user(buff, channel->h2b_cb_vmem + rd_offset,
				 rd_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		print_hex_dump_debug("mmbi H2B:", DUMP_PREFIX_NONE, 16, 1,
				     channel->h2b_cb_vmem + rd_offset,
				     rd_len, false);
		rd_offset += rd_len;
	} else {
		ssize_t chunk_len;

		chunk_len = channel->h2b_cb_size - rd_offset;
		if (copy_to_user(buff, channel->h2b_cb_vmem + rd_offset,
				 chunk_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		print_hex_dump_debug("mmbi H2B-1:", DUMP_PREFIX_NONE, 16, 1,
				     channel->h2b_cb_vmem + rd_offset,
				     chunk_len, false);
		rd_offset = 0;
		if (copy_to_user(buff + chunk_len,
				 channel->h2b_cb_vmem + rd_offset,
				 rd_len - chunk_len)) {
			dev_err(priv->dev,
				"Failed to copy data to user space\n");
			ret = -EFAULT;
			goto err_out;
		}
		print_hex_dump_debug("mmbi H2B-2:", DUMP_PREFIX_NONE, 16, 1,
				     channel->h2b_cb_vmem + rd_offset,
				     rd_len - chunk_len, false);
		rd_offset += (rd_len - chunk_len);
	}
	*offp += rd_len;
	ret = rd_len;

	update_host_rop(channel, 0, rd_len + sizeof(struct mmbi_header));

	dev_dbg(priv->dev, "%s: Return length: %ld\n", __func__, ret);
err_out:
	mutex_unlock(&priv->lock);
	/*
	 * Raise the missing SCI's by checking pointer for host
	 * read acknowledgment. This will work around the Missing
	 * SCI bug on host side.
	 */
	dev_warn(priv->dev, "%s: Check and raise missing SCI\n", __func__);
	raise_missing_sci(channel);

	protocol->data_available = false;

	wake_up_device(channel);

	return ret;
}

static ssize_t mmbi_write(struct file *filp, const char *buffer, size_t len,
			  loff_t *offp)
{
	struct npcm_mmbi_protocol *protocol = file_npcm_espi_mmbi(filp);
	struct npcm_mmbi_channel *channel = protocol->chan_ref;
	struct npcm_espi_mmbi *priv = channel->priv;
	struct mmbi_header header;
	struct host_rop hrop;
	ssize_t wt_offset;
	ssize_t avail_buf_len;
	ssize_t chunk_len;
	ssize_t end_offset;
	u32 h_rwp0;

	dev_dbg(priv->dev, "%s: length:%ld , type: %d\n", __func__, len,
		protocol->type);

	if (read_host_rwp_val(channel, 0, &h_rwp0)) {
		dev_err(priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	/* If Host READY bit is not set, Just discard the write. */
	if (!GET_HOST_READY_BIT(h_rwp0)) {
		dev_dbg(channel->priv->dev,
			"Host not ready, discarding request...\n");
		return -EAGAIN;
	}

	if (get_b2h_avail_buf_len(channel, &avail_buf_len)) {
		dev_dbg(priv->dev, "Failed to B2H empty buffer len\n");
		return -EAGAIN;
	}

	dev_dbg(priv->dev, "B2H buffer empty space: %ld\n", avail_buf_len);

	/* Empty space should be more than write request data size */
	if (len + sizeof(header) > avail_buf_len) {
		dev_err(priv->dev, "Not enough space(%ld) in B2H buffer\n",
			avail_buf_len);
		return -ENOSPC;
	}

	mutex_lock(&priv->lock);
	/* Fill multi-protocol header */
	header.data = ((protocol->type << 24) + len);

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));
	wt_offset = hrop.b2h_wp;
	end_offset = channel->b2h_cb_size;

	if ((end_offset - wt_offset) >= sizeof(header)) {
		memcpy(channel->b2h_cb_vmem + wt_offset, &header,
		       sizeof(header));
		wt_offset += sizeof(header);
	} else {
		chunk_len = end_offset - wt_offset;
		memcpy(channel->b2h_cb_vmem + wt_offset, &header, chunk_len);
		memcpy(channel->b2h_cb_vmem, &header + chunk_len,
		       (sizeof(header) - chunk_len));
		wt_offset = (sizeof(header) - chunk_len);
	}

	/* Write the data */
	if ((end_offset - wt_offset) >= len) {
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset], buffer,
				   len)) {
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}
		print_hex_dump_debug("mmbi B2H:", DUMP_PREFIX_NONE, 16, 1,
				     channel->b2h_cb_vmem + wt_offset,
				     len, false);
		wt_offset += len;
	} else {
		chunk_len = end_offset - wt_offset;
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset], buffer,
				   chunk_len)) {
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}
		print_hex_dump_debug("mmbi B2H-1:", DUMP_PREFIX_NONE, 16, 1,
				     channel->b2h_cb_vmem + wt_offset,
				     chunk_len, false);
		wt_offset = 0;
		if (copy_from_user(&channel->b2h_cb_vmem[wt_offset],
				   buffer + chunk_len, len - chunk_len)) {
			mutex_unlock(&priv->lock);
			return -EFAULT;
		}
		print_hex_dump_debug("mmbi B2H-2:", DUMP_PREFIX_NONE, 16, 1,
				     channel->b2h_cb_vmem + wt_offset,
				     len - chunk_len, false);
		wt_offset += len - chunk_len;
	}

	*offp += len;

	update_host_rop(channel, len + sizeof(struct mmbi_header), 0);
	mutex_unlock(&priv->lock);

	return len;
}

static int get_mmbi_config(struct npcm_mmbi_channel *channel, void __user *userbuf)
{
	bool h_ready;
	struct host_rop hrop;
	struct npcm_mmbi_get_config get_conf;
	u32 h2b_wptr, b2h_rptr, h_rwp0, h_rwp1;

	if (read_host_rwp_val(channel, 0, &h_rwp0)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}
	if (read_host_rwp_val(channel, 4, &h_rwp1)) {
		dev_err(channel->priv->dev, "Failed to read Host RWP\n");
		return -EAGAIN;
	}

	h2b_wptr = GET_H2B_WRITE_POINTER(h_rwp0);
	b2h_rptr = GET_B2H_READ_POINTER(h_rwp1);

	h_ready = GET_HOST_READY_BIT(h_rwp0) ? true : false;

	memcpy(&hrop, channel->hrop_vmem, sizeof(struct host_rop));

	get_conf.h_rdy = h_ready;
	get_conf.h2b_wp = h2b_wptr;
	get_conf.b2h_rp = b2h_rptr;
	get_conf.h2b_rp = hrop.h2b_rp;
	get_conf.b2h_wp = hrop.b2h_wp;

	if (copy_to_user(userbuf, &get_conf, sizeof(get_conf))) {
		dev_err(channel->priv->dev, "copy to user failed\n");
		return -EFAULT;
	}
	return 0;
}

static int get_b2h_empty_space(struct npcm_mmbi_channel *channel,
			       void __user *userbuf)
{
	struct npcm_mmbi_get_empty_space empty_space;
	ssize_t avail_buf_len;

	if (get_b2h_avail_buf_len(channel, &avail_buf_len)) {
		dev_dbg(channel->priv->dev, "Failed to B2H empty buffer len\n");
		return -EAGAIN;
	}

	dev_dbg(channel->priv->dev, "B2H buffer empty space: %ld\n",
		avail_buf_len);

	empty_space.length = avail_buf_len;

	if (copy_to_user(userbuf, &empty_space, sizeof(empty_space))) {
		dev_err(channel->priv->dev, "copy to user failed\n");
		return -EFAULT;
	}

	return 0;
}

static long mmbi_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct npcm_mmbi_protocol *protocol = file_npcm_espi_mmbi(filp);
	struct npcm_mmbi_channel *channel = protocol->chan_ref;
	void __user *userbuf = (void __user *)arg;
	int ret;

	switch (cmd) {
	case NPCM_MMBI_CTRL_IOCTL_GET_B2H_EMPTY_SPACE:
		ret = get_b2h_empty_space(channel, userbuf);
		break;

	case NPCM_MMBI_CTRL_IOCTL_SEND_RESET_REQUEST:
		ret = send_bmc_reset_request(channel);
		break;

	case NPCM_MMBI_CTRL_IOCTL_GET_CONFIG:
		ret = get_mmbi_config(channel, userbuf);
		break;

	default:
		dev_err(channel->priv->dev, "Command not found\n");
		ret = -ENOTTY;
	}

	return ret;
}

static const struct file_operations npcm_espi_mmbi_fops = {
	.owner = THIS_MODULE,
	.open = mmbi_open,
	.release = mmbi_release,
	.read = mmbi_read,
	.write = mmbi_write,
	.unlocked_ioctl = mmbi_ioctl,
	.poll = mmbi_poll
};

static char *get_protocol_suffix(protocol_type type)
{
	switch (type) {
	case MMBI_PROTOCOL_IPMI:
		return "ipmi";
	case MMBI_PROTOCOL_SEAMLESS:
		return "seamless";
	case MMBI_PROTOCOL_RAS_OFFLOAD:
		return "ras_offload";
	case MMBI_PROTOCOL_MCTP:
		return "mctp";
	case MMBI_PROTOCOL_NODE_MANAGER:
		return "nm";
	}

	return NULL;
}

static void mmbi_desc_init(struct npcm_mmbi_channel *channel,
			   struct mmbi_cap_desc *desc)
{
	struct mmbi_cap_desc ch_desc;
	u32 b2h_size = channel->hrwp_vmem - channel->desc_vmem;

	memset(&ch_desc, 0, sizeof(ch_desc));

	/* Per MMBI protoco spec, Set it to "$MMBI$" */
	strncpy(ch_desc.signature, "$MMBI$", sizeof(ch_desc.signature));
	ch_desc.version = 1;
	ch_desc.instance_num = channel->chan_num;
	/*
	 * TODO: Add multi-channel support. Handcoded H2B start offset
	 * to 0x8000 as we support single channel today.
	 */
	ch_desc.nex_inst_base_addr = 0;
	ch_desc.b2h_ba = sizeof(struct mmbi_cap_desc) + sizeof(struct host_rop);
	ch_desc.h2b_ba = b2h_size + sizeof(struct host_rwp);
	ch_desc.b2h_d = (b2h_size - (sizeof(struct mmbi_cap_desc) + sizeof(struct host_rop))) / 16;
	ch_desc.h2b_d = (b2h_size - sizeof(struct host_rwp)) / 16; /* 32KB = 0x800 * 16 */

	ch_desc.buffer_type = 0x01; /* VMSCB */
	ch_desc.bt_desc.host_rop_p = sizeof(struct mmbi_cap_desc);
	ch_desc.bt_desc.host_rwp_p = b2h_size;
	ch_desc.bt_desc.msg_protocol_type = 0x01; /* Multiple protocol type */
	ch_desc.bt_desc.host_int_type =
		0x01; /* SCI Triggered through eSPI VW */
	ch_desc.bt_desc.global_sys_interrupt = 0x00; /* Not used */
	ch_desc.bt_desc.bmc_int_type = 0x00; /* HW Interrupt */
	ch_desc.bt_desc.bmc_int_a = 0x00; /* Not used, set to zero */
	ch_desc.bt_desc.bmc_int_v = 0x00; /* Not used, set to zero */

	ch_desc.crc8 = crc8(mmbi_crc8_table, (u8 *)&ch_desc,
			    (size_t)(sizeof(ch_desc) - 1), 0);
	memcpy(desc, &ch_desc, sizeof(ch_desc));
}

static int mmbi_channel_init(struct npcm_espi_mmbi *priv, u8 idx)
{
	struct device *dev = priv->dev;
	int rc;
	u8 i;
	u8 *h2b_vaddr, *b2h_vaddr;
	struct mmbi_cap_desc ch_desc;
	struct host_rop hrop;
	struct device_node *node;
	int no_of_protocols_enabled;
	u8 mmbi_supported_protocols[MAX_NO_OF_SUPPORTED_PROTOCOLS];

	u32 b2h_size = (priv->mmbi_size / 2);
	u32 h2b_size = (priv->mmbi_size / 2);

	b2h_vaddr = priv->shm_vaddr;
	h2b_vaddr = priv->shm_vaddr + (priv->mmbi_size / 2);

	memset(&priv->chan[idx], 0, sizeof(struct npcm_mmbi_channel));
	priv->chan[idx].chan_num = idx;

	priv->chan[idx].desc_vmem = b2h_vaddr;
	priv->chan[idx].hrop_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc);
	priv->chan[idx].b2h_cb_vmem = b2h_vaddr + sizeof(struct mmbi_cap_desc) +
				      sizeof(struct host_rop);
	priv->chan[idx].b2h_cb_size = b2h_size - sizeof(struct mmbi_cap_desc) -
				      sizeof(struct host_rop);
	/* Set BMC ready bit */
	memcpy(&hrop, priv->chan[idx].hrop_vmem, sizeof(hrop));
	hrop.b_rdy = 1;
	memcpy(priv->chan[idx].hrop_vmem, &hrop, sizeof(hrop));

	priv->chan[idx].hrwp_vmem = h2b_vaddr;
	priv->chan[idx].h2b_cb_vmem = h2b_vaddr + sizeof(struct host_rwp);
	priv->chan[idx].h2b_cb_size = h2b_size - sizeof(struct host_rwp);

	dev_dbg(priv->dev,
		"B2H mapped addr - desc: 0x%0lx, hrop: 0x%0lx, b2h_cb: 0x%0lx\n",
		(size_t)priv->chan[idx].desc_vmem,
		(size_t)priv->chan[idx].hrop_vmem,
		(size_t)priv->chan[idx].b2h_cb_vmem);
	dev_dbg(priv->dev, "H2B mapped addr - hrwp: 0x%0lx, h2b_cb: 0x%0lx\n",
		(size_t)priv->chan[idx].hrwp_vmem,
		(size_t)priv->chan[idx].h2b_cb_vmem);

	dev_dbg(priv->dev, "B2H buffer size: 0x%0lx\n",
		(size_t)priv->chan[idx].b2h_cb_size);
	dev_dbg(priv->dev, "H2B buffer size: 0x%0lx\n",
		(size_t)priv->chan[idx].h2b_cb_size);

	/* Initialize the MMBI channel descriptor */
	mmbi_desc_init(&priv->chan[idx], &ch_desc);
	memcpy(priv->chan[idx].desc_vmem, &ch_desc, sizeof(ch_desc));

	priv->chan[idx].enabled = true;
	node = of_get_child_by_name(priv->dev->of_node, "instance");
	if (!node) {
		dev_err(priv->dev, "mmbi protocol : no instance found\n");
		goto err_destroy_channel;
	}
	no_of_protocols_enabled = of_property_count_u8_elems(node, "protocols");
	if (no_of_protocols_enabled <= 0 || no_of_protocols_enabled >
	    MAX_NO_OF_SUPPORTED_PROTOCOLS){
		dev_err(dev, "No supported mmbi protocol\n");
		of_node_put(node);
		goto err_destroy_channel;
	}
	memset(mmbi_supported_protocols, 0, sizeof(mmbi_supported_protocols));
	rc = of_property_read_u8_array(node, "protocols", mmbi_supported_protocols,
				       no_of_protocols_enabled);
	if (!rc) {
		memset(&priv->chan[idx].supported_protocols, 0,
		       sizeof(priv->chan[idx].supported_protocols));
		memcpy(&priv->chan[idx].supported_protocols, mmbi_supported_protocols,
		       sizeof(mmbi_supported_protocols));
	}
	of_node_put(node);

	for (i = 0; i < no_of_protocols_enabled; i++) {
		char *dev_name;
		u8 proto_type;

		proto_type = priv->chan[idx].supported_protocols[i];
		dev_name = get_protocol_suffix(proto_type);
		if (!dev_name) {
			dev_err(dev,
				"Unable to get MMBI protocol suffix name\n");
			goto err_destroy_channel;
		}
		priv->chan[idx].protocol[i].type = proto_type;
		priv->chan[idx].protocol[i].miscdev.name =
			devm_kasprintf(dev, GFP_KERNEL, "%s_%d_%s", DEVICE_NAME,
				       idx, dev_name);
		priv->chan[idx].protocol[i].miscdev.minor = MISC_DYNAMIC_MINOR;
		priv->chan[idx].protocol[i].miscdev.fops =
			&npcm_espi_mmbi_fops;
		priv->chan[idx].protocol[i].miscdev.parent = dev;
		rc = misc_register(&priv->chan[idx].protocol[i].miscdev);
		if (rc) {
			dev_err(dev, "Unable to register device\n");
			goto err_destroy_channel;
		}

		/* Hold the back reference of channel */
		priv->chan[idx].protocol[i].chan_ref = &priv->chan[idx];

		priv->chan[idx].protocol[i].data_available = false;
		priv->chan[idx].protocol[i].process_data = false;
		init_waitqueue_head(&priv->chan[idx].protocol[i].queue);
	}

	priv->chan[idx].priv = priv;

	/*
	 * When BMC goes for reset while host is in OS, SRAM memory will be
	 * remapped and the content in memory will be lost. This include
	 * host ready state which will block memory write transactions.
	 * Ideally this reset has to be done while mapping memory(u-boot).
	 * Since channel initialization (including descriptor) done at kernel,
	 * So added channel reset also during driver load. Future, when staged
	 * commands processing(IPMI commands for BIOS-BMC communication) is
	 * enabled, this check should be moved to u-boot.
	 */
	if (send_bmc_reset_request(&priv->chan[idx]))
		dev_info(dev, "MMBI channel(%d) reset failed\n", idx);

	dev_info(dev, "MMBI Channel(%d) initialized successfully\n", idx);

	return 0;

err_destroy_channel:
	if (b2h_vaddr)
		memunmap(b2h_vaddr);

	if (h2b_vaddr)
		memunmap(h2b_vaddr);

	priv->chan[idx].enabled = false;
	return -ENOMEM;
}

static void npcm_mmbi_check_h2b(struct work_struct *work)
{
	struct npcm_espi_mmbi *priv = container_of(to_delayed_work(work),
		struct npcm_espi_mmbi, work);
	struct npcm_mmbi_channel *channel = &priv->chan[0];
	u32 h2b_wp, h_rwp0;

	mutex_lock(&priv->lock);

	if (check_host_reset_request(channel))
		goto out;

	if (read_host_rwp_val(channel, 0, &h_rwp0)) {
		dev_dbg(channel->priv->dev, "Failed to read Host RWP0\n");
		goto out;
	}
	h2b_wp = GET_H2B_WRITE_POINTER(h_rwp0);
	if (h2b_wp != channel->cur_h2b_wp) {
		dev_dbg(priv->dev, "current h2b_wp 0x%x, new h2b_wp 0x%x\n",
			channel->cur_h2b_wp, h2b_wp);
		channel->cur_h2b_wp = h2b_wp;
		wake_up_device(channel);
	}
out:
	mutex_unlock(&priv->lock);
	if (priv->poll_interval)
		queue_delayed_work(priv->wq, &priv->work,
				   msecs_to_jiffies(priv->poll_interval));
	else
		npcm_mmbi_enable_interrupt(priv, true);
}

static irqreturn_t npcm_espi_mmbi_irq(int irq, void *arg)
{
	struct npcm_espi_mmbi *priv = arg;

	dev_dbg(priv->dev, "MMBI IRQ Status: 0x%02x\n",
		readb(priv->regs + SHM_HOFS_STS));
	npcm_mmbi_enable_interrupt(priv, false);
	/* Clear interrupt status */
	writeb(HOFS1W, priv->regs + SHM_HOFS_STS);

	queue_delayed_work(priv->wq, &priv->work, 0);

	return IRQ_HANDLED;
}

static int npcm_mmbi_setup_window(struct npcm_espi_mmbi *priv, struct resource *res)
{
	resource_size_t size = resource_size(res);
	resource_size_t phys_addr = res->start;
	int size_sel;
	u8 reg_val;

	if (size > NPCM_MAX_WIN_SIZE)
		size = NPCM_MAX_WIN_SIZE;
	size_sel = ilog2(size);
	reg_val = readb(priv->regs + SHM_WIN_SIZE) & 0xF0;
	reg_val |= size_sel & 0x0F;
	/* Map to access window 1 */
	writel(phys_addr, priv->regs + SHM_WIN_BASE1);
	writeb(reg_val, priv->regs + SHM_WIN_SIZE);

	return 0;
}

static int npcm_espi_mmbi_probe(struct platform_device *pdev)
{
	struct npcm_espi_mmbi *priv;
	struct device_node *node;
	struct resource resm;
	int rc, i;
	u32 val;

	dev_dbg(&pdev->dev, "MMBI: Probing MMBI devices...\n");

	priv = devm_kzalloc(&pdev->dev, sizeof(struct npcm_espi_mmbi),
			    GFP_KERNEL);
	if (!priv)
		return -ENOMEM;

	priv->dev = &pdev->dev;
	priv->regs = devm_platform_ioremap_resource(pdev, 0);
	if (IS_ERR(priv->regs)) {
		dev_err(priv->dev, "MMBI: Failed to get regmap!\n");
		return PTR_ERR(priv->regs);
	}

	/* ESPI register map */
	priv->pmap = syscon_regmap_lookup_by_phandle(priv->dev->of_node,
						     "nuvoton,espi");
	if (IS_ERR(priv->pmap)) {
		dev_err(priv->dev, "MMBI: Failed to find ESPI regmap\n");
		return PTR_ERR(priv->pmap);
	}

	/* Set SW control for SCI# VW(System Event Index 6 wire 0) */
	regmap_clear_bits(priv->pmap, NPCM_ESPI_VWEVSM2, HW_WIRE(0));

	priv->wq = alloc_workqueue("%s", 0, 0, dev_name(priv->dev));
	if (!priv->wq) {
		dev_err(priv->dev, "MMBI: Failed to allocate workqueue\n");
		return -ENOMEM;
	}
	/* If memory-region is described in device tree then store */
	node = of_parse_phandle(priv->dev->of_node, "memory-region", 0);
	if (node) {
		rc = of_address_to_resource(node, 0, &resm);
		of_node_put(node);
		if (!rc) {
			priv->mmbi_size = resource_size(&resm);
			if (priv->mmbi_size > NPCM_MAX_WIN_SIZE) {
				dev_err(priv->dev, "Too large memory region\n");
				return -EINVAL;
			}
			priv->mmbi_phys_addr = resm.start;
			priv->shm_vaddr = devm_ioremap_resource_wc(priv->dev, &resm);
			if (IS_ERR(priv->shm_vaddr)) {
				dev_err(priv->dev, "device mem io remap failed\n");
				return PTR_ERR(priv->shm_vaddr);
			}
			memset_io(priv->shm_vaddr, 0, priv->mmbi_size);
			npcm_mmbi_setup_window(priv, &resm);
		} else {
			dev_err(priv->dev, "No memory region\n");
			return -EINVAL;
		}
	} else {
		dev_dbg(priv->dev,
			"No DTS config, assign default MMBI Address\n");
		return -EINVAL;
	}
	dev_dbg(priv->dev, "MMBI: SramAddr:0x%llx, Size: 0x%llx\n",
		priv->mmbi_phys_addr, priv->mmbi_size);

	crc8_populate_msb(mmbi_crc8_table, MMBI_CRC8_POLYNOMIAL);

	dev_set_drvdata(priv->dev, priv);

	for (i = 0; i < MAX_NO_OF_SUPPORTED_CHANNELS; i++) {
		rc = mmbi_channel_init(priv, i);
		if (rc) {
			dev_err(priv->dev, "MMBI: Channel(%d) init failed\n",
				i);
			return rc;
		}
	}

	mutex_init(&priv->lock);
	INIT_DELAYED_WORK(&priv->work, npcm_mmbi_check_h2b);

	if (of_property_read_bool(priv->dev->of_node, "use-interrupt")) {
		dev_info(priv->dev, "use interrupt\n");
		/* Enable IRQ */
		priv->irq = platform_get_irq(pdev, 0);
		if (priv->irq < 0) {
			dev_err(priv->dev, "MMBI: No irq specified\n");
			return priv->irq;
		}

		rc = devm_request_irq(priv->dev, priv->irq, npcm_espi_mmbi_irq,
				      IRQF_SHARED, dev_name(priv->dev), priv);
		if (rc) {
			dev_err(priv->dev, "MMBI: Unable to get IRQ\n");
			return rc;
		}
		npcm_mmbi_enable_interrupt(priv, true);
	} else {
		if (!of_property_read_u32(priv->dev->of_node, "poll-interval-ms", &val))
			priv->poll_interval = val > POLLING_TIMEOUT_MS ?
				POLLING_TIMEOUT_MS : val;
		else
			priv->poll_interval = POLLING_INTERVAL_MS;
		dev_info(priv->dev, "use polling, interval=%u ms\n",
			 priv->poll_interval);
		queue_delayed_work(priv->wq, &priv->work,
				   msecs_to_jiffies(priv->poll_interval));
	}
	dev_info(priv->dev, "MMBI: npcm MMBI driver loaded successfully\n");

	return 0;
}

static int npcm_espi_mmbi_remove(struct platform_device *pdev)
{
	struct npcm_espi_mmbi *priv = dev_get_drvdata(&pdev->dev);
	int i, j;

	dev_dbg(priv->dev, "MMBI: Removing MMBI device\n");

	if (priv->wq)
		destroy_workqueue(priv->wq);
	for (i = 0; i < MAX_NO_OF_SUPPORTED_CHANNELS; i++) {
		if (!priv->chan[i].enabled)
			continue;
		for (j = 0; priv->chan[i].supported_protocols[j] != 0; j++)
			misc_deregister(&priv->chan[i].protocol[j].miscdev);
	}

	return 0;
}

static const struct of_device_id npcm_espi_mmbi_match[] = {
	{ .compatible = "nuvoton,npcm845-espi-mmbi" },
	{}
};
MODULE_DEVICE_TABLE(of, npcm_espi_mmbi_match);

static struct platform_driver npcm_espi_mmbi_driver = {
	.driver = {
		.name           = DEVICE_NAME,
		.of_match_table = npcm_espi_mmbi_match,
	},
	.probe  = npcm_espi_mmbi_probe,
	.remove = npcm_espi_mmbi_remove,
};
module_platform_driver(npcm_espi_mmbi_driver);

MODULE_AUTHOR("AppaRao Puli <apparao.puli@intel.com>");
MODULE_AUTHOR("Tomer Maimon <tomer.maimon@nuvoton.com>");
MODULE_AUTHOR("Stanley Chu <yschu@nuvoton.com>");
MODULE_AUTHOR("Ban Feng <kcfeng0@nuvoton.com>");
MODULE_DESCRIPTION("Nuvoton NPCM eSPI MMBI Driver");
MODULE_LICENSE("GPL v2");
