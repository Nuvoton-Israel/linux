/********************************************************
 *
 * file : vdm_common.h
 *
 *
 ****************************************************/

#ifndef _VDM_API_H_
#define _VDM_API_H_

#define  PCIe_HEADER_ROUTE_BY_ID			0x12
#define  PCIe_HEADER_ROUTE_TO_RC			0x10
#define  PCIe_HEADER_ROUTE_FROM_RC			0x13

#define VDM_ERR_FIFO_OVERFLOW				0x00000001

typedef enum
{
	VDMA_STATUS_NONE						=	0,
	VDMA_STATUS_DATA_READY					=	1,
	VDMA_STATUS_OVERFLOWED					=	2,
	VDMA_STATUS_DATA_READY_WITH_OVERFLOWED	=	3
} VDMA_STATUS_t;


typedef enum
{
	VDM_RX_TIMEOUT_NONE=0,
	VDM_RX_TIMEOUT_0u5,
	VDM_RX_TIMEOUT_1u,
	VDM_RX_TIMEOUT_2u,
	VDM_RX_TIMEOUT_4u,
	VDM_RX_TIMEOUT_8u,
} VDM_RX_TIMEOUT_t;

typedef void (*receive_packet_func_t)(uint16_t aBDF, uint32_t *data ,
										uint32_t NumOfWords ,uint8_t isThisLastDataInPacket);


extern int vdm_exit_common(void);
extern int vdm_init_common(uint32_t *apVdma_rx_buff,uint32_t *apVdma_rx_buff_virt_addr,uint32_t buff_bytes_len);
extern int vdm_SendMessage(uint8_t route_type, uint16_t aBDF,uint8_t  *apData,uint32_t aLength);

extern VDMA_STATUS_t vdma_is_data_ready(void);
extern void vdma_copy_packets_from_buffer_with_overflow(receive_packet_func_t receive_packet_func);
extern void vdma_copy_packets_from_buffer(receive_packet_func_t receive_packet_func);
extern uint8_t vdm_is_ready_for_write(void);
extern uint8_t vdm_set_timeout(VDM_RX_TIMEOUT_t timeout);
extern void vdm_reset(void);
extern void vdm_disable_rx(void);
extern void vdm_enable_rx(void);
extern uint8_t vdm_is_in_reset(void) ;
extern uint32_t vdm_get_errors(void);
extern void vdm_clear_errors(uint32_t clear_errors_bitmap) ;

#endif
