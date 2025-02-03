// SPDX-License-Identifier: GPL-2.0-only
#ifndef __LINUX_DMA_NPCM_GDMA_H
#define __LINUX_DMA_NPCM_GDMA_H

#include <linux/types.h>

struct npcm_gdma_peripheral_config {
	/*
	 * DMA request and acknowledge connectivity.
	 *  0: eSPI or FIU3 (default)
	 *  1: FIU0
	 *  2: AES Write FIFO
	 *  3: AES Read FIFO
	 *  4: Virtual UART1 RX
	 *  5: Virtual UART2 RX
	 *  6: I3C0 TX FIFO
	 *  7: I3C0 RX FIFO
	 *  8: I3C1 TX FIFO
	 *  9: I3C1 RX FIFO
	 * 10: I3C2 TX FIFO
	 * 11: I3C2 RX FIFO
	 * 12: I3C3 TX FIFO
	 * 13: I3C3 RX FIFO
	 * 14: I3C4 TX FIFO
	 * 15: I3C4 RX FIFO
	 * 16: I3C5 TX FIFO
	 * 17: I3C5 RX FIFO
	 */
	unsigned int connectivity;
};

#endif /* __LINUX_DMA_NPCM_GDMA_H */
