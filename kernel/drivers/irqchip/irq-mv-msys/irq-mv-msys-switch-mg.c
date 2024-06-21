/*
 * Marvell MSYS MG internal (cascaded to one of switch irq) IRQ handling
 *
 * Copyright (C) 2012 Marvell
 *
 * Grzegorz Jaszczyk <jaz@semihalf.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */


#include <linux/io.h>
#include <linux/of_address.h>
#include <linux/of_irq.h>
#include <linux/irq.h>
#include <linux/interrupt.h>
#include <linux/irqchip/chained_irq.h>
#include <linux/syscore_ops.h>
#include <linux/irqdomain.h>
#include "../irqchip.h"

/* MG (inside the switching core irq 1) interrupt registers */
#define MG_INT_IRQ_CAUSE_OFFSET_1	(0x0)
#define MG_INT_IRQ_MASK_OFFSET_1	(0x4)

/* Global definitions */
#define MG_SUM_IRQ_NR	32

static void __iomem *mg_int_base;
static struct irq_domain *mg_swic_domain;

static void mg_sw_irq_mask(struct irq_data *d)
{
	u32 reg;
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	reg = readl(mg_int_base + MG_INT_IRQ_MASK_OFFSET_1);
	writel(reg & ~(1 << hwirq), mg_int_base + MG_INT_IRQ_MASK_OFFSET_1);
}

static void mg_sw_irq_unmask(struct irq_data *d)
{
	u32 reg;
	irq_hw_number_t hwirq = irqd_to_hwirq(d);

	reg = readl(mg_int_base + MG_INT_IRQ_MASK_OFFSET_1);
	writel(reg | (1 << hwirq), mg_int_base + MG_INT_IRQ_MASK_OFFSET_1);
}

static struct irq_chip mg_sw_irq_chip = {
	.name		= "mg_sw_irq",
	.irq_mask       = mg_sw_irq_mask,
	.irq_mask_ack   = mg_sw_irq_mask,
	.irq_unmask     = mg_sw_irq_unmask,
#ifdef CONFIG_SMP
	/* XXX: There is no information in the documentation about affinity of
	 * those interrupts
	 */
#endif
};

static int mg_swic_irq_map(struct irq_domain *h, unsigned int virq, irq_hw_number_t hw)
{
	mg_sw_irq_mask(irq_get_irq_data(virq));

	irq_set_status_flags(virq, IRQ_LEVEL);

	irq_set_chip_and_handler(virq, &mg_sw_irq_chip, handle_level_irq);

	set_irq_flags(virq, IRQF_VALID | IRQF_PROBE);

	return 0;
}

static void mg_swic_handle_cascade_irq(unsigned int irq, struct irq_desc *desc)
{
	struct irq_chip *chip = irq_get_chip(irq);
	unsigned long irqmap, irqn;
	unsigned int cascade_irq;

	chained_irq_enter(chip, desc);

	irqmap = readl_relaxed(mg_int_base + MG_INT_IRQ_CAUSE_OFFSET_1);

	for_each_set_bit(irqn, &irqmap, MG_SUM_IRQ_NR) {

		/* Check if the interrupt is not masked on current SWITCH IRQ
		 * (there are 4 SWITCH IRQ which points to the same interrupt
		 * tree, but can be used/mapped to e.g.: MSYS, CM3, PCI. We use
		 * MG IRQ, which is SWITCH IRQ 1, for cascade handling in MSYS.
		 * SWITCH IRQ 0 is used for SWITCH, SWITCH IRQ 2 is currently
		 * unused, SWITCH IRQ 3 is used for CM3 (in FreeRTOS)
		 */
		if (!((1 << irqn) & readl_relaxed(mg_int_base + MG_INT_IRQ_MASK_OFFSET_1)))
			continue;

		cascade_irq = irq_find_mapping(mg_swic_domain, irqn);

		generic_handle_irq(cascade_irq);
	}

	chained_irq_exit(chip, desc);
}

static struct irq_domain_ops mg_swic_irq_ops = {
	.map = mg_swic_irq_map,
	.xlate = irq_domain_xlate_onecell,
};

static int mg_swic_suspend(void)
{
	/* TODO */
	return 0;
}

static void mg_swic_resume(void)
{
	/* TODO */
}

struct syscore_ops mg_swic_syscore_ops = {
	.suspend = mg_swic_suspend,
	.resume	= mg_swic_resume,
};

static int __init mg_swic_of_init(struct device_node *node,
					     struct device_node *parent)
{
	int parent_irq;

	mg_int_base = of_iomap(node, 0);
	BUG_ON(!mg_int_base);

	mg_swic_domain = irq_domain_add_linear(node, MG_SUM_IRQ_NR, &mg_swic_irq_ops, NULL);
	BUG_ON(!mg_swic_domain);

	parent_irq = irq_of_parse_and_map(node, 0);
	BUG_ON(!parent_irq);

	irq_set_chained_handler(parent_irq, mg_swic_handle_cascade_irq);

	register_syscore_ops(&mg_swic_syscore_ops);

	return 0;
}

IRQCHIP_DECLARE(mg_swic, "marvell,mg_swic_irq", mg_swic_of_init);
