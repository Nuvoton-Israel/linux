/* SPDX-License-Identifier: GPL-2.0 */
/*
 * obmf.h - OBMF-ICP over USB driver header
 *
 * Copyright (C) 2025-2026 Nuvoton Technology Corp.
 *
 * OBMF-ICP (Open Boot and Management Framework - Interface Consolidation
 * Protocol) USB class driver.  Implements the virtual-adapter mux/demux
 * architecture per OBMF-ICP v0.9 + USB Device Class Spec v1.0.
 */

#ifndef __LINUX_USB_OBMF_H
#define __LINUX_USB_OBMF_H

#include <linux/usb.h>
#include <linux/mutex.h>
#include <linux/completion.h>
#include <linux/kref.h>
#include <linux/workqueue.h>
#include <linux/types.h>

/* Forward declarations for optional subsystem headers */
struct i2c_adapter;
struct gpio_chip;
struct spi_controller;
struct tty_driver;
struct tty_port;
struct miscdevice;

/*
 * USB class binding per OCP OBMF-ICP USB Spec v1.0
 */
#define USB_CLASS_OBMF			0xEF
#define USB_OBMF_SUBCLASS_ICP		0x09
#define USB_OBMF_PROTOCOL_V1		0x01

#define OBMF_IINTERFACE_STRING		"OCP OBMF"

/*
 * Class-specific descriptor constants
 * Note: USB_DT_CS_INTERFACE is already defined in <uapi/linux/usb/ch9.h>
 */
#define OBMF_SUBTYPE_FUNCTIONAL		0x01
#define OBMF_FUNCTIONAL_DESC_SIZE	14

/*
 * STALL recovery threshold — after this many consecutive STALLs without
 * a successful transfer, escalate to USB device reset.
 */
#define OBMF_STALL_THRESHOLD		3

/*
 * Default request timeout (ms)
 */
#define OBMF_DEFAULT_TIMEOUT_MS		5000

/* ---------- Common Header (5 bytes) ----------------------------------------
 *
 * Byte 0:   Channel [7:0]
 * Byte 1:   Channel Type [7:0]
 * Byte 2:   RqResp [0], Status [7:1]
 * Byte 3-4: Size [15:0]  (LE)
 * Byte 5+:  Payload
 */
struct obmf_common_hdr {
	u8	channel;
	u8	channel_type;
	u8	rqresp_status;
	__le16	size;
} __packed;

#define OBMF_COMMON_HDR_SIZE	sizeof(struct obmf_common_hdr)

/* Header byte 2 accessor macros */
#define OBMF_HDR_IS_RESPONSE(h)		((h)->rqresp_status & 0x01)
#define OBMF_HDR_STATUS(h)		(((h)->rqresp_status >> 1) & 0x7F)
#define OBMF_HDR_SET_REQUEST(h)		((h)->rqresp_status = 0)
#define OBMF_HDR_SET_RESPONSE(h, s)	((h)->rqresp_status = (((s) & 0x7F) << 1) | 0x01)

/* ---------- MMIO Sub-Header (2 bytes, Channel Type 01h only) ---------------
 *
 * Byte 0: Transaction [2:0], Reserved [7:3]
 * Byte 1: Tag [7:0]  (alternates 0 <-> 1)
 */
struct obmf_mmio_subhdr {
	u8	transaction;
	u8	tag;
} __packed;

#define OBMF_MMIO_SUBHDR_SIZE	sizeof(struct obmf_mmio_subhdr)

/* MMIO Transaction types (3-bit field, spec §3.4 Layer 2) */
#define OBMF_TRANS_SHORT_READ	0x00	/* 32-bit addr, Size u8 (up to 255B) */
#define OBMF_TRANS_SHORT_WRITE	0x01	/* 32-bit addr, Size u8 (up to 255B) */
#define OBMF_TRANS_LONG_READ	0x02	/* 64-bit addr, Size u16 */
#define OBMF_TRANS_LONG_WRITE	0x03	/* 64-bit addr, Size u16 */

/* MMIO channel-specific status codes */
#define OBMF_MMIO_STATUS_ADDR_OUT_OF_RANGE	0x40
#define OBMF_MMIO_STATUS_ACCESS_DENIED		0x41

/* ---------- Channel Type codes -------------------------------------------- */
#define OBMF_TYPE_CONFIG	0x00
#define OBMF_TYPE_MMIO		0x01
#define OBMF_TYPE_GPIO		0x02
#define OBMF_TYPE_SERIAL	0x03
#define OBMF_TYPE_I2C		0x04	/* I2C Controller */
#define OBMF_TYPE_I2C_TARGET	0x05	/* I2C Target */
#define OBMF_TYPE_I3C		0x06	/* I3C Controller (reserved) */
#define OBMF_TYPE_IPMI		0x07
#define OBMF_TYPE_SPI		0x08	/* SPI Controller */
#define OBMF_TYPE_OEM_MIN	0xF8
#define OBMF_TYPE_OEM_MAX	0xFF

/* ---------- Response Status Codes (Common Header byte 2[7:1]) ------------- */
#define OBMF_STATUS_SUCCESS		0x00
#define OBMF_STATUS_INVALID_CMD		0x01	/* Invalid command or parameter */
#define OBMF_STATUS_TIMEOUT		0x02	/* Request timeout */
#define OBMF_STATUS_NOT_READY		0x03	/* Channel not ready / busy */
#define OBMF_STATUS_PERMANENT_ERROR	0x04	/* Channel permanent error */
#define OBMF_STATUS_UNKNOWN_CHANNEL	0x05	/* Unknown/unsupported channel */
#define OBMF_STATUS_SIZE_NOT_SUPPORTED	0x06	/* Request size not supported */

/* ---------- GPIO Optimised Channel commands (v0.9) ----------------------- */
#define OBMF_GPIO_CMD_GET_VALUES		0x00
#define OBMF_GPIO_CMD_SET_VALUES		0x01
#define OBMF_GPIO_CMD_GET_IRQ_CFG	0x02
#define OBMF_GPIO_CMD_SET_IRQ_CFG	0x03
#define OBMF_GPIO_CMD_IRQ_NOTIFY		0x04
#define OBMF_GPIO_CMD_IRQ_NOTIFY_DG	0x05	/* Datagram (no response) */

/* GPIO Index/Data pair: u16 LE, [11:0]=index, [15:12]=data */
#define OBMF_GPIO_IDX_MASK		0x0FFF
#define OBMF_GPIO_DATA_SHIFT		12
#define OBMF_GPIO_DATA_MASK		0xF000
#define OBMF_GPIO_PACK(idx, data)	(((data) << 12) | ((idx) & 0x0FFF))
#define OBMF_GPIO_UNPACK_IDX(v)		((v) & 0x0FFF)
#define OBMF_GPIO_UNPACK_DATA(v)	(((v) >> 12) & 0x0F)

/* GPIO values for Get/Set Values (data nibble) */
#define OBMF_GPIO_VAL_HIGH		0x00
#define OBMF_GPIO_VAL_LOW		0x01

/* GPIO IRQ config values (data nibble for Get/Set IRQ Config) */
#define OBMF_GPIO_IRQ_DISABLE		0x00
#define OBMF_GPIO_IRQ_LEVEL_LOW		0x01
#define OBMF_GPIO_IRQ_LEVEL_HIGH	0x02
#define OBMF_GPIO_IRQ_RISING		0x03
#define OBMF_GPIO_IRQ_FALLING		0x04
#define OBMF_GPIO_IRQ_BOTH		0x05

/* GPIO channel-specific status codes (v0.9 spec) */
#define OBMF_STATUS_GPIO_IDX_NOT_SUPPORTED	0x40
#define OBMF_STATUS_GPIO_IRQ_NOT_SUPPORTED	0x41
#define OBMF_STATUS_GPIO_INVALID_OP		0x42

/* GPIO Configuration Entry (50 bytes per pin, spec §9.2.1) */
#define OBMF_GPIO_CONFIG_ENTRY_SIZE	50
#define OBMF_GPIO_CFG_INDEX		0x00	/* u16: GPIO index 0-4095 */
#define OBMF_GPIO_CFG_NAME		0x02	/* char[32]: GPIO name */
#define OBMF_GPIO_CFG_DIRECTION		0x22	/* u8: 0=Output, 1=Input */
#define OBMF_GPIO_CFG_DEFAULT_OUT	0x23	/* u8: 0=Low, 1=High */
#define OBMF_GPIO_CFG_DRIVE_CFG		0x24	/* u8: 0=PP, 1=OD, 2=OS */
#define OBMF_GPIO_CFG_PERSIST		0x25	/* u8: persist across reset */
#define OBMF_GPIO_CFG_BIAS_PULL		0x26	/* u8: 0=None, 1=Up, 2=Down */
#define OBMF_GPIO_DIR_OUTPUT		0x00
#define OBMF_GPIO_DIR_INPUT		0x01

/* GPIO channel-specific status codes */
#define OBMF_GPIO_STATUS_INDEX_NOT_SUPPORTED	0x40
#define OBMF_GPIO_STATUS_INT_NOT_SUPPORTED	0x41
#define OBMF_GPIO_STATUS_INVALID_OPERATION	0x42

/* ---------- I2C Controller Optimised Channel (v0.9) ----------------------- */
#define OBMF_I2C_CMD_READ		0x00
#define OBMF_I2C_CMD_WRITE		0x01
#define OBMF_I2C_CMD_SMBUS_BLOCK_READ	0x02
#define OBMF_I2C_CMD_SMBUS_WRITE_READ	0x03
#define OBMF_I2C_CMD_SMBUS_HOST_NOTIFY	0x04

/* I2C request byte 0[7]: 0=send STOP, 1=do NOT send STOP */
#define OBMF_I2C_NO_STOP		BIT(7)

/* I2C response header size: Command(1) + Address(1) + ReadLen(2) */
#define OBMF_I2C_RESP_HDR_SIZE		4

/* I2C channel-specific status codes */
#define OBMF_I2C_STATUS_TIMEOUT		0x40
#define OBMF_I2C_STATUS_ARB_LOST	0x41
#define OBMF_I2C_STATUS_BUS_BUSY	0x42
#define OBMF_I2C_STATUS_NACK		0x43

/* ---------- SPI Controller Optimised Channel (v0.9) ----------------------- */
#define OBMF_SPI_CMD_READ		0x01
#define OBMF_SPI_CMD_WRITE		0x02
#define OBMF_SPI_CMD_WRITE_READ		0x03
#define OBMF_SPI_CMD_POSTED_WRITE	0x04
#define OBMF_SPI_CMD_MASK		0x0F
#define OBMF_SPI_CS_DEASSERT		BIT(4)
#define OBMF_SPI_CS_ASSERT		BIT(5)
#define OBMF_SPI_CS_NUM_SHIFT		6
#define OBMF_SPI_CS_NUM_MASK		0xC0

/* SPI channel-specific status codes */
#define OBMF_SPI_STATUS_MODE_UNSUPPORTED		0x40
#define OBMF_SPI_STATUS_TRANSFER_ERROR		0x41

/* ---------- Serial Optimised Channel (v0.9) ------------------------------- */
/* Operation/Event bitfield (request byte 0) */
#define OBMF_SERIAL_EVT_BREAK		BIT(0)	/* Generate break */
#define OBMF_SERIAL_EVT_BREAK_DETECT	BIT(1)	/* Break detected */
#define OBMF_SERIAL_EVT_TX_OVERRUN	BIT(2)	/* TX overrun */
#define OBMF_SERIAL_EVT_CARRIER_DOWN	BIT(3)	/* Carrier down */

/* Response byte 0 */
#define OBMF_SERIAL_ACK			0x00
#define OBMF_SERIAL_NACK		0x01

/* Serial channel-specific status codes */
#define OBMF_SERIAL_STATUS_LINE_TIMEOUT	0x40

/* ---------- IPMI Optimised Channel ---------------------------------------- */
#define OBMF_IPMI_CMD_SEND_MESSAGE	0x00

/* ---------- MMIO misc device ioctl ---------------------------------------- */
struct obmf_mmio_xfer {
	__u8	transaction;	/* in:  OBMF_TRANS_SHORT_READ etc. */
	__u8	status;		/* out: MMIO response status byte */
	__u16	wr_len;		/* in:  write data length */
	__u16	rd_len;		/* in:  expected read data length */
	__u16	reserved;
	__u64	address;	/* in:  MMIO address (64-bit) */
	__u64	wr_data_ptr;	/* in:  userspace pointer to write data */
	__u64	rd_data_ptr;	/* out: userspace pointer to read buffer */
};

#define OBMF_MMIO_IOC_MAGIC	'O'
#define OBMF_MMIO_IOC_XFER	_IOWR(OBMF_MMIO_IOC_MAGIC, 1, struct obmf_mmio_xfer)

/* ---------- OCP_OBMF_FUNCTIONAL Descriptor -------------------------------- */
struct obmf_functional_desc {
	__u8	bLength;
	__u8	bDescriptorType;
	__u8	bDescriptorSubtype;
	__u8	bMultimessageSupport;
	__le16	wMaxWrTransferSize;
	__le16	wMaxRdTransferSize;
	__le16	wMaxWrInterruptSize;
	__le16	wMaxRdInterruptSize;
	__le16	bcdOCPOBMFVersion;
} __packed;

/* ---------- Channel 0 Discovery Register Map (v0.9) ---------------------- */
#define OBMF_DISC_OBMF_VER		0x00	/* R   16-bit BCD version */
#define OBMF_DISC_VENDOR_ID		0x02	/* R   16-bit PCI-SIG vendor */
#define OBMF_DISC_DEVICE_ID		0x04	/* R   16-bit */
#define OBMF_DISC_DEVICE_ROLE		0x06	/* R   32-bit */
#define OBMF_DISC_DEVICE_NAME		0x0A	/* R   32-byte UTF-8 */
#define OBMF_DISC_NUM_CHANNELS		0x2A	/* R    8-bit */
#define OBMF_DISC_CONFIG_STATUS		0x2B	/* R    8-bit */
#define OBMF_DISC_VENDOR_CFG_OFF	0x2C	/* R   32-bit */
#define OBMF_DISC_CHANNEL_OFFSET_BASE	0x30	/* R   4B × N */

#define OBMF_DISC_DEVICE_NAME_LEN	32

/* DEVICE_ROLE values */
#define OBMF_ROLE_UNSPECIFIED		0
#define OBMF_ROLE_HOST			1
#define OBMF_ROLE_DEVICE		2

/* ---------- Channel Config Common Header ---------------------------------- */
#define OBMF_CHCFG_TYPE			0x00	/* 1B */
#define OBMF_CHCFG_NUMBER		0x01	/* 1B */
#define OBMF_CHCFG_NAME			0x02	/* 16B */
#define OBMF_CHCFG_STATUS		0x12	/* 1B */
#define OBMF_CHCFG_CONTROL		0x13	/* 1B */
#define OBMF_CHCFG_SPECIFIC_STATUS	0x14	/* 1B */
#define OBMF_CHCFG_SPECIFIC_CONTROL	0x15	/* 1B */
#define OBMF_CHCFG_CONFIG_SIZE		0x18	/* 4B */
#define OBMF_CHCFG_CONFIG_DATA		0x1C	/* variable */

#define OBMF_CHCFG_NAME_LEN		16

/* CHANNEL_CONTROL bits */
#define OBMF_CHCTL_ENABLE		BIT(0)

/* Minimum spec version we support */
#define OBMF_MIN_SPEC_VERSION		0x0090	/* v0.9.0 */

/* ---------- Maximum device-request queue depth ----------------------------- */
#define OBMF_DEV_REQ_QUEUE_DEPTH	8

/* ---------- Per-channel state --------------------------------------------- */
struct obmf_channel {
	u8			channel_id;
	u8			channel_type;
	u8			channel_cfg;
	u8			tag;		/* MMIO only: host→device tag, alternates 0/1 */
	u8			dev_tag;	/* MMIO only: device→host expected tag, alternates 0/1 */

	struct mutex		lock;		/* One outstanding request per ch */
	struct completion	done;		/* Signalled by RX demux */

	u8			*resp_buf;	/* Response payload written by RX */
	int			resp_len;
	int			status;		/* Completion code */

	void			*priv;		/* Subsystem-specific data (optimised channels) */

	u32			config_offset;	/* CH0 offset to channel config header */
	u32			config_size;	/* Size of channel config data area */
	u16			gpio_count;	/* GPIO channels: number of GPIO lines */

	struct kobject		*kobj;		/* sysfs: /obmf/channel/<N> */
	struct device		*sysfs_dev;	/* device for sysfs "device" symlink */
	struct obmf_device	*odev;		/* Back-pointer */

	/* RX segment reassembly state (per channel) */
	u8			*reasm_buf;
	int			reasm_total;	/* expected total payload bytes */
	int			reasm_offset;	/* bytes accumulated so far */
	struct obmf_common_hdr	reasm_hdr;	/* saved header from first segment */
	bool			reasm_active;
};

/* ---------- Device-initiated request work item ---------------------------- */
struct obmf_dev_req {
	struct work_struct	work;
	struct obmf_device	*odev;
	u8			channel_id;
	u8			channel_type;
	u8			transaction;	/* MMIO only */
	u8			tag;		/* MMIO only */
	int			data_len;
	u8			data[];
};

/* ---------- STALL recovery flags ------------------------------------------ */
#define OBMF_STALL_BULK_IN		0
#define OBMF_STALL_BULK_OUT		1
#define OBMF_STALL_INT_IN		2
#define OBMF_STALL_INT_OUT		3
#define OBMF_STALL_RESET_PENDING	4

/* ---------- Main device structure ----------------------------------------- */
struct obmf_device {
	struct usb_device	*udev;
	struct usb_interface	*intf;
	struct kref		kref;
	bool			disconnected;
	int			device_index;	/* Global obmf device number */

	/* Endpoints */
	u8			bulk_in_ep;
	u8			bulk_out_ep;
	u8			int_in_ep;
	u8			int_out_ep;
	unsigned int		bulk_out_maxp;
	unsigned int		bulk_in_maxp;

	/* Functional descriptor values (from device, read-only) */
	u16			max_wr_transfer_size;	/* device: max Bulk OUT recv size */
	u16			max_rd_transfer_size;	/* device: max Bulk IN  send size */
	u16			max_wr_int_size;
	u16			max_rd_int_size;
	u16			bcd_version;
	bool			has_int_in;
	bool			has_int_out;

	/* Host (BMC) capabilities — used for tx_buf allocation and
	 * READ_SIZE.PRI / WRITE_SIZE.PRI advertisement in discovery.
	 * Capped at device limits: min(host_max, device_max).
	 */
	u16			host_tx_size;		/* actual tx_buf allocation size */
	u16			host_rx_size;		/* actual rx_buf allocation size */

	/* Interrupt endpoint details */
	unsigned int		int_in_ep_size;
	unsigned int		int_out_ep_size;
	unsigned int		int_in_interval;
	unsigned int		int_out_interval;

	/* Channels (discovered via Channel 0) */
	struct obmf_channel	*channels;
	int			num_channels;

	/* Transport: TX */
	u8			*tx_buf;
	struct mutex		tx_lock;

	/* Transport: RX (continuous Bulk IN) */
	struct urb		*rx_urb;
	u8			*rx_buf;

	/* Transport: Interrupt IN (optional) */
	struct urb		*int_in_urb;
	u8			*int_in_buf;

	/* Transport: Interrupt OUT (optional) */
	u8			*int_out_buf;

	/* Device-initiated request workqueue (high priority) */
	struct workqueue_struct	*dev_req_wq;

	/* STALL recovery */
	unsigned int		bulk_in_stall_count;
	unsigned int		bulk_out_stall_count;
	struct work_struct	stall_work;
	unsigned long		stall_flags;

	/* sysfs: /sys/bus/usb/devices/<intf>/obmf/channel/ */
	struct kobject		*obmf_kobj;
	struct kobject		*channel_kobj;

	/* Subsystem registrations (indexed by channel_id) */
#if IS_ENABLED(CONFIG_USB_OBMF_I2C)
	int			num_i2c;
#endif
#if IS_ENABLED(CONFIG_USB_OBMF_GPIO)
	int			num_gpio;
#endif
#if IS_ENABLED(CONFIG_USB_OBMF_SPI)
	int			num_spi;
#endif
#if IS_ENABLED(CONFIG_USB_OBMF_SERIAL)
	struct tty_driver	*tty_drv;
	void			*tty_ports;	/* struct obmf_serial_port[] */
	int			num_serial;
	char			tty_drv_name[16];
#endif
	int			num_misc;	/* IPMI + OEM misc devices */
};

/* ---------- STALL recovery work (obmf-core.c) ----------------------------- */
void obmf_stall_recovery_work(struct work_struct *work);

/* ---------- Transport layer (obmf-transport.c) ---------------------------- */
int  obmf_transport_init(struct obmf_device *odev);
void obmf_transport_exit(struct obmf_device *odev);

int  obmf_send_request(struct obmf_device *odev, struct obmf_channel *ch,
		       u8 channel_type, const void *payload, int payload_len,
		       void *resp_buf, int resp_buf_len,
		       unsigned long timeout_ms);

int  obmf_send_mmio_request(struct obmf_device *odev, struct obmf_channel *ch,
			    u8 transaction, u64 address,
			    const void *wr_data, int wr_len,
			    void *rd_data, int rd_len);

int  obmf_send_response(struct obmf_device *odev, u8 channel_id,
			u8 channel_type, u8 status,
			const void *payload, int payload_len);

/* ---------- Discovery (obmf-discovery.c) ---------------------------------- */
int  obmf_discover_channels(struct obmf_device *odev);
void obmf_free_channels(struct obmf_device *odev);

/* ---------- I2C (obmf-i2c.c) ---------------------------------------------- */
#if IS_ENABLED(CONFIG_USB_OBMF_I2C)
int  obmf_i2c_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_i2c_unregister(struct obmf_channel *ch);
#else
static inline int obmf_i2c_register(struct obmf_device *odev,
				    struct obmf_channel *ch) { return 0; }
static inline void obmf_i2c_unregister(struct obmf_channel *ch) {}
#endif

/* ---------- GPIO (obmf-gpio.c) -------------------------------------------- */
#if IS_ENABLED(CONFIG_USB_OBMF_GPIO)
int  obmf_gpio_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_gpio_unregister(struct obmf_channel *ch);
void obmf_gpio_handle_dev_request(struct obmf_channel *ch,
				  const u8 *data, int len);
#else
static inline int obmf_gpio_register(struct obmf_device *odev,
				     struct obmf_channel *ch) { return 0; }
static inline void obmf_gpio_unregister(struct obmf_channel *ch) {}
static inline void obmf_gpio_handle_dev_request(struct obmf_channel *ch,
						const u8 *data, int len) {}
#endif

/* ---------- SPI (obmf-spi.c) ---------------------------------------------- */
#if IS_ENABLED(CONFIG_USB_OBMF_SPI)
int  obmf_spi_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_spi_unregister(struct obmf_channel *ch);
void obmf_spi_handle_dev_request(struct obmf_channel *ch,
				 const u8 *data, int len);
#else
static inline int obmf_spi_register(struct obmf_device *odev,
				    struct obmf_channel *ch) { return 0; }
static inline void obmf_spi_unregister(struct obmf_channel *ch) {}
static inline void obmf_spi_handle_dev_request(struct obmf_channel *ch,
					       const u8 *data, int len) {}
#endif

/* ---------- Serial (obmf-serial.c) ---------------------------------------- */
#if IS_ENABLED(CONFIG_USB_OBMF_SERIAL)
int  obmf_serial_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_serial_unregister(struct obmf_channel *ch);
int  obmf_serial_init(struct obmf_device *odev);
void obmf_serial_exit(struct obmf_device *odev);
void obmf_serial_rx(struct obmf_channel *ch, const u8 *data, int len);
void obmf_serial_handle_dev_request(struct obmf_channel *ch,
				    const u8 *data, int len);
#else
static inline int obmf_serial_register(struct obmf_device *odev,
				       struct obmf_channel *ch) { return 0; }
static inline void obmf_serial_unregister(struct obmf_channel *ch) {}
static inline int obmf_serial_init(struct obmf_device *odev) { return 0; }
static inline void obmf_serial_exit(struct obmf_device *odev) {}
static inline void obmf_serial_rx(struct obmf_channel *ch,
				  const u8 *data, int len) {}
static inline void obmf_serial_handle_dev_request(struct obmf_channel *ch,
						  const u8 *data,
						  int len) {}
#endif

/* ---------- IPMI (obmf-ipmi.c) -------------------------------------------- */
#if IS_ENABLED(CONFIG_USB_OBMF_IPMI)
int  obmf_ipmi_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_ipmi_unregister(struct obmf_channel *ch);
void obmf_ipmi_handle_dev_request(struct obmf_channel *ch,
				  const u8 *data, int len);
#else
static inline int obmf_ipmi_register(struct obmf_device *odev,
				     struct obmf_channel *ch) { return 0; }
static inline void obmf_ipmi_unregister(struct obmf_channel *ch) {}
static inline void obmf_ipmi_handle_dev_request(struct obmf_channel *ch,
						const u8 *data, int len) {}
#endif

/* ---------- MMIO misc device (obmf-mmio-misc.c) --------------------------- */
int  obmf_mmio_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_mmio_unregister(struct obmf_channel *ch);
void obmf_mmio_handle_dev_request(struct obmf_channel *ch,
				 u8 transaction, u8 tag,
				 const u8 *data, int len);

/* ---------- OEM (obmf-oem.c) ---------------------------------------------- */
int  obmf_oem_register(struct obmf_device *odev, struct obmf_channel *ch);
void obmf_oem_unregister(struct obmf_channel *ch);
void obmf_oem_handle_dev_request(struct obmf_channel *ch,
				 const u8 *data, int len);

#endif /* __LINUX_USB_OBMF_H */
