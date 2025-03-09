/********************************************************
 *
 * file : vdm_module.h
 *
 *
 ****************************************************/

#ifndef _VDM_MODULE_H_
#define _VDM_MODULE_H_

typedef struct
{
    uint8_t bus, device, function;
} bdf_arg_t;

#define PCIE_VDM_IOC_MAGIC       0xe8
#define PCIE_VDM_SET_BDF 					_IOW(PCIE_VDM_IOC_MAGIC, 1, bdf_arg_t *)
#define PCIE_VDM_SET_TRANSMIT_BUFFER_SIZE   _IOW(PCIE_VDM_IOC_MAGIC, 2 , uint32_t )
#define PCIE_VDM_SET_RECEIVE_BUFFER_SIZE    _IOW(PCIE_VDM_IOC_MAGIC, 3 , uint32_t )
#define PCIE_VDM_STOP_VDM_TX			    _IO(PCIE_VDM_IOC_MAGIC, 4 )
#define PCIE_VDM_STOP_VDM_RX			    _IO(PCIE_VDM_IOC_MAGIC, 5 )
#define PCIE_VDM_GET_BDF 					_IOR(PCIE_VDM_IOC_MAGIC, 6, bdf_arg_t *)
#define PCIE_VDM_RESET 						_IO(PCIE_VDM_IOC_MAGIC, 7 )
#define PCIE_VDM_SET_RX_TIMEOUT 			_IOW(PCIE_VDM_IOC_MAGIC, 8 , uint32_t )
#define PCIE_VDM_REINIT						_IOW(PCIE_VDM_IOC_MAGIC, 9 , uint32_t )
#define PCIE_VDM_SET_RESET_DETECT_POLL		_IOW(PCIE_VDM_IOC_MAGIC, 10 , uint32_t )
#define PCIE_VDM_GET_ERRORS					_IOW(PCIE_VDM_IOC_MAGIC, 11 , uint32_t *)
#define PCIE_VDM_CLEAR_ERRORS				_IOW(PCIE_VDM_IOC_MAGIC, 12 , uint32_t )

#define PCIE_VDM_ERR_HW_FIFO_OVERFLOW				0x00000001
#define PCIE_VDM_ERR_DMA_BUFFER_OVERFLOW			0x00000002
#define PCIE_VDM_ERR_USER_BUFFER_OVERFLOW			0x00000004
#define PCIE_VDM_ERR_BUS_RESET_OCCURED				0x00000008

#endif
