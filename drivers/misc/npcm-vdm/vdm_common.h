/********************************************************
 *
 * file : vdm_common.h
 *
 *
 ****************************************************/

#ifndef _VDM_COMMON_H_
#define _VDM_COMMON_H_

#define _1KB_ 1024
#define _64KB_ (64*1024)

/******** vdm module defines **************/
#define PHY_SELECT_FOR_PCIE_BRIDGE_REG		0x64
#define PHY_SELECT_FOR_PCIE_BRIDGE_FIELD_POS   (17)

#ifndef CONFIG_ARM64
#define PHYS_VDM_STAT_REG_ADDR			0xE0800000
#else
#define PHYS_VDM_STAT_REG_ADDR			0xE0800000
#endif

#define VDM_STAT_REG_ADDR			0x00

#define VDM_STAT_REG				VDM_STAT_REG_ADDR
#define VDM_TX_DONE_BIT_POS 			0
#define VDM_RX_DONE_BIT_POS 			1
#define VDM_RX_FULL_BIT_POS 			2
#define VDM_RXNDW_FIELD_POS 			16
#define VDM_RXNDW_FIELD_MASK 			0xff0000

#ifndef CONFIG_ARM64
#define VDM_INT_EN_REG				0x04
#else
#define VDM_INT_EN_REG				0x100
#endif

#define VDM_INT_EN_REG_RX_INT_BIT_POS		1

#ifndef CONFIG_ARM64
#define VDM_RXF_REG_ADDR			0x08
#define PHYS_VDM_RXF_REG_ADDR			0xE0800008
#else
#define VDM_RXF_REG_ADDR			0x200
#define PHYS_VDM_RXF_REG_ADDR			0xE0800200
#endif

#ifndef CONFIG_ARM64
#define VDM_TXF_REG				0x0C
#else
#define VDM_TXF_REG				0x300
#endif

#ifndef CONFIG_ARM64
#define VDM_CNT_REG_ADDR			0x10
#else
#define VDM_CNT_REG_ADDR			0x400
#endif

#define VDM_CNT_REG				VDM_CNT_REG_ADDR

#define VDM_CNT_REG_START_TX_BIT		0
#define VDM_CNT_REG_VDM_ENABLE_BIT_POS		1
#define VDM_RX_TIMEOUT_FIELD_POS 		4
#define VDM_RX_TIMEOUT_FIELD_MASK 		0x70
#define VDM_ENABLE_FIELD_POS			1

#ifndef CONFIG_ARM64
#define VDM_FLT_REG				0x14
#else
#define VDM_FLT_REG				0x500
#endif
#define VDM_FLT_REG_FLT_ENABLE_BIT_POS		31

/******** vdma module defines **************/
#define VDMA_CNT_REG				0x00
#define VDMA_CNT_REG_BUS_LOCK_BIT_POS		17
#define VDMA_CNT_REG_BURST_BIT_POS		9
#define VDMA_CNT_REG_ENABLE_POS			0


#define VDMA_SRCB_REG				0x04
#define VDMA_DSTB_REG				0x08
#define VDMA_CDST_REG				0x14
#define VDMA_ERDPNT_REG				0x48
#define VDMA_ECTL_REG				0x40

#define VDMA_ECTL_REG_DRDY_EN_BIT_POS		29
#define VDMA_ECTL_REG_DRDY_BIT_POS		28
#define VDMA_ECTL_REG_HALT_INT_EN_BIT_POS	25
#define VDMA_ECTL_REG_HALT_BIT_POS		24
#define VDMA_ECTL_REG_BUFF_SIZE_POS		16
#define VDMA_ECTL_REG_RETRIGGER_IF_BUFF_NOT_EMPTY_BIT_POS	11
#define VDMA_ECTL_REG_AUTO_STATUS_UPDATE_BIT_POS		9
#define VDMA_ECTL_REG_SIZE_MODIFIER_POS				4
#define VDMA_ECTL_REG_SIZE_CYCLIC_BUF_EN_BIT_POS		1

#define VDMA_ESRCSZ_REG				0x44
#define VDMA_EST0AD_REG				0x50
#define VDMA_EST0MK_REG				0x54
#define VDMA_EST0DT_REG				0x58

#define PCIe_MAX_TX_BUFFER_SIZE		128
#define PCIe_MSG_HEADER_SIZE_INT		(4)
#define PCIe_MSG_HEADER_SIZE_WITHOUT_MCTP_INT	(3)
#define PCIe_MAX_PAYLOAD_SIZE_BYTES  (PCIe_MAX_TX_BUFFER_SIZE-(PCIe_MSG_HEADER_SIZE_WITHOUT_MCTP_INT*4))

//#define  VDM_VENDOR_ID     0xb41a
#define  VDM_VENDOR_ID     0x1ab4
//#define  VDM_VENDOR_ID_LSB_IN_U32     	0x1ab40000
#define  VDM_VENDOR_ID_LSB_IN_U32     		( ((VDM_VENDOR_ID & 0xff)<<24) + ((VDM_VENDOR_ID & 0xff00)<<8))
#define  VDM_VENDOR_ID_MASK_LSB_IN_U32    	0xffff0000

#define  PCIe_HEADER_FMT_FIELD_MASK     	0x00000060
#define  PCIe_HEADER_FMT_MSG_NO_PAYLOAD  	0x00000020
#define  PCIe_HEADER_FMT_MSG_WITH_PAYLOAD  	0x00000060

//ROUTE_FIELD icludes tlp type and route mechanism
#define  PCIe_HEADER_ROUTE_FIELD_MASK     	0x0000001f

#define  PCIe_HEADER_LENGTH_FIELD_MASK     	0xFF030000

#define  PCIe_HEADER_ATTR_MASK  		0x00300000
#define  PCIe_HEADER_ATTR_TEST  		0x00100000


#define  PCIe_HEADER_DEST_BDF_FIELD_MASK	0x0000FFFF

#define  PCIe_HEADER_TAG_FIELD_MASK		0x00FF0000

#define  PCIe_TC0     			0x00
#define  PCIe_NO_IDO    		0x00
#define  PCIe_NO_TH    			0x00
#define  PCIe_NO_TD    			0x00
#define  PCIe_NO_EP    			0x00
#define  PCIe_NO_RLX_ORDER_NO_SNOOP	0x00
#define  PCIe_NO_AT			0x00
#define  PCIe_TAG     			0x00
#define  PCIe_MSG_VDM_TYPE1     	0x7f

#define	MAX_PACKET_LENGTH		128
#define SEND_TIMEOUT			100

typedef enum
{
	VDMA_SIZE_MODIFIER_NONE=0,
	VDMA_SIZE_MODIFIER_BITS_0_7_USED,
	VDMA_SIZE_MODIFIER_BITS_8_15_USED,
	VDMA_SIZE_MODIFIER_BITS_16_23_USED,
	VDMA_SIZE_MODIFIER_BITS_24_31_USED,
} VDMA_SIZE_MODIFIER_t;

#else
#pragma message( "warning : this header file had already been included" )
#endif
