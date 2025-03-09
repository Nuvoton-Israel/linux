
Nuvoton VDM implementation notes:

1) ioctl() commands:
	For all ioctl commands in case that VDM is in reset (due to PCIe reset) then -EIO is returned.
	
	PCIE_VDM_SET_BDF
		argument of ioctl is pointer to variable of following type:
		typedef struct
		{
			u8_t bus, device, function;
		} bdf_arg_t;

	PCIE_VDM_SET_TRANSMIT_BUFFER_SIZE
		set transmit buffer size
		argument of ioctl is u32_t

	PCIE_VDM_SET_RECEIVE_BUFFER_SIZE
		set receive buffer size
		argument of ioctl is u32_t

	PCIE_VDM_STOP_VDM_TX
		enable/disable transmit (argument of ioctl : 0 - disable / 1-enable)

	PCIE_VDM_STOP_VDM_RX
		enable/disable receive
		argument of ioctl : 0 - disable / 1-enable

	PCIE_VDM_GET_BDF
		argument of ioctl is pointer to variable of following type:
		typedef struct
		{
			u8_t bus, device, function;
		} bdf_arg_t;

		in case of invalid BDF -ENOENT is returned
																	
	PCIE_VDM_RESET
		reset vdm hardware
													
	PCIE_VDM_SET_RX_TIMEOUT
		set timeout on PCIe bus
		argument of ioctl : 0 - 0us , 1 -0.5us , 2- 1us , 3- 2us , 4- 4us , 5- 8us
									
	PCIE_VDM_REINIT
		re-init hw: mark all open BDF as invalid and re-init all buffers
																					
	PCIE_VDM_SET_RESET_DETECT_POLL
		enable/disable PCIe reset detection polling (argument of ioctl : 0 - disable / 1-enable)
		polling is enabled on start

2) read()
	Read the requested number of bytes from VDM.
	in case of invalid BDF -ENOENT is returned.
	in case that VDM is in reset (due to PCIe reset) then -EIO is returned.
	in case of a blocking read (default read() in linux) and reset detection poll is
			enabled (see ioctl PCIE_VDM_SET_RESET_DETECT_POLL) then every 100ms VDM is
			checked for reset (due to PCIe reset), if reset detected then -EIO is returned

3) write()
	in case of invalid BDF -ENOENT is returned .
	in case that VDM is in reset (due to PCIe reset) then -EIO is returned.

4) poll()
	0 - when timeout expires and data is not ready and VDM was not reset
	1 and POLLIN set in revents parameter - when data is ready.
	1 and POLLHUP (Hang up) set in revents - when VDM was reset.
	Note:
		if poll() timeout parameter is greater then 100ms and reset detection poll is
		enabled (see ioctl PCIE_VDM_SET_RESET_DETECT_POLL) then every 100ms VDM is
		checked for reset (due to PCIe reset)
