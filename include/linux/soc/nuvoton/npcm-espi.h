/* SPDX-License-Identifier: GPL-2.0 */
/*
 * Register definitions for the Nuvoton NPCM eSPI block.
 *
 * Copyright (c) 2026 Nuvoton Technology corporation.
 */

#ifndef __SOC_NUVOTON_NPCM_ESPI_H
#define __SOC_NUVOTON_NPCM_ESPI_H

#include <linux/bitfield.h>
#include <linux/bits.h>
#include <linux/build_bug.h>

/*
 * The eSPI block is one register window shared by several drivers, each
 * bound to its own child of the espi node and living in a different
 * subsystem.  They all reach it through the same syscon regmap, so the
 * offsets below are the single definition for all of them.
 */

#define NPCM_ESPI_ESPICFG		0x004
#define  NPCM_ESPI_ESPICFG_VWCHANEN	BIT(1)
#define  NPCM_ESPI_ESPICFG_FLASHCHANEN	BIT(3)
#define  NPCM_ESPI_ESPICFG_HFLASHCHANEN	BIT(7)
#define  NPCM_ESPI_ESPICFG_VWCHN_SUPP	BIT(25)

/*
 * Status and interrupt enable use the same bit numbering, so one set of
 * names serves both.  Status is write-one-to-clear.
 */
#define NPCM_ESPI_ESPISTS		0x008
#define NPCM_ESPI_ESPIIE		0x00C
#define  NPCM_ESPI_CFGUPD		BIT(1)
#define  NPCM_ESPI_FLASHRX		BIT(4)
#define  NPCM_ESPI_SFLASHRD		BIT(5)
#define  NPCM_ESPI_VWUPD		BIT(8)
#define  NPCM_ESPI_ESPIRST		BIT(9)
#define  NPCM_ESPI_PLTRST		BIT(10)

#define NPCM_ESPI_ESPIERR		0x03C
#define  NPCM_ESPI_ESPIERR_VWERR	BIT(11)

#define NPCM_ESPI_VWEVMS(n)		(0x140 + (4 * (n)))
#define  NPCM_ESPI_VWEVMS_PLTRST	BIT(1)
#define  NPCM_ESPI_VWEVMS_INDEX_EN	BIT(15)
#define  NPCM_ESPI_VWEVMS_MODIFIED	BIT(16)
#define  NPCM_ESPI_VWEVMS_IE		BIT(18)

/*
 * Which driver owns which status bit.  Owning a bit means being the one
 * that clears it and the one that decides when its enable may be set; no
 * other driver may write either register for that bit.  A bit claimed
 * twice is an interrupt storm waiting to happen, because each owner
 * assumes the other cleared it.
 */
#define NPCM_ESPI_IRQ_SLAVE	(NPCM_ESPI_PLTRST | NPCM_ESPI_ESPIRST)
#define NPCM_ESPI_IRQ_FLASH	(NPCM_ESPI_SFLASHRD | NPCM_ESPI_FLASHRX | \
				 NPCM_ESPI_CFGUPD)
#define NPCM_ESPI_IRQ_VWGPIO	(NPCM_ESPI_VWUPD)

static_assert(!(NPCM_ESPI_IRQ_SLAVE & NPCM_ESPI_IRQ_FLASH),
	      "espi-slave and espi-flash claim the same status bit");
static_assert(!(NPCM_ESPI_IRQ_SLAVE & NPCM_ESPI_IRQ_VWGPIO),
	      "espi-slave and espi-vwgpio claim the same status bit");
static_assert(!(NPCM_ESPI_IRQ_FLASH & NPCM_ESPI_IRQ_VWGPIO),
	      "espi-flash and espi-vwgpio claim the same status bit");

/*
 * Interrupt lines the eSPI block offers its children.  These are the
 * numbers a child names in its "interrupts" property; they are an index
 * into the controller's domain and are unrelated to the bit positions
 * above.  Keep them in sync with the bindings.
 */
#define NPCM_ESPI_IRQ_LINE_CFGUPD	0
#define NPCM_ESPI_IRQ_LINE_FLASHRX	1
#define NPCM_ESPI_IRQ_LINE_SFLASHRD	2
#define NPCM_ESPI_IRQ_LINE_VWUPD	3
#define NPCM_ESPI_IRQ_LINE_ESPIRST	4
#define NPCM_ESPI_IRQ_LINE_PLTRST	5

/* Events broadcast by the eSPI block to its children. */
#define NPCM_ESPI_EVENT_CFGUPD		0

struct device;
struct notifier_block;

#if IS_ENABLED(CONFIG_NPCM_ESPI)
int devm_npcm_espi_register_notifier(struct device *dev,
				     struct notifier_block *nb);
#else
static inline int devm_npcm_espi_register_notifier(struct device *dev,
						   struct notifier_block *nb)
{
	return -ENODEV;
}
#endif

#endif /* __SOC_NUVOTON_NPCM_ESPI_H */
