// SPDX-License-Identifier: GPL-2.0
// Copyright (c) 2018 Nuvoton Technology corporation.
// Copyright 2018 Google, Inc.

#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/types.h>
#include <asm/mach/arch.h>
#include <asm/mach-types.h>
#include <asm/mach/map.h>
#include <asm/hardware/cache-l2x0.h>

#define NPCM7XX_SPSWC_REG 0x038	/* Serial ports switch control register */

static const char *const npcm7xx_dt_match[] = {
	"nuvoton,npcm750",
	"nuvoton,npcm730",
	NULL
};

void __init npcm7xx_init_late(void)
{
	struct device_node *gcr_np;
	void __iomem *gcr_base;
	u32 spswc;

	gcr_np = of_find_compatible_node(NULL, NULL, "nuvoton,npcm750-gcr");
	if (!gcr_np) {
		pr_err("no gcr device node\n");
		return;
	}
	gcr_base = of_iomap(gcr_np, 0);
	if (!gcr_base) {
		pr_err("could not iomap gcr");
		return;
	}

	/* Set serial port connectivity mode if specified */
	if (of_property_read_u32(of_root, "nuvoton,npcm750-spswc", &spswc) == 0)
		iowrite32(spswc, gcr_base + NPCM7XX_SPSWC_REG);

	iounmap(gcr_base);
}

DT_MACHINE_START(NPCM7XX_DT, "NPCM7XX Chip family")
	.atag_offset	= 0x100,
	.dt_compat	= npcm7xx_dt_match,
	.l2c_aux_val	= 0x0,
	.l2c_aux_mask	= ~0x0,
	.init_late	= npcm7xx_init_late,
MACHINE_END
