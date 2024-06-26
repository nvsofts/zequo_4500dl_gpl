/*
 * Marvell MVEBU clock gating control.
 *
 * Sebastian Hesselbarth <sebastian.hesselbarth@gmail.com>
 * Andrew Lunn <andrew@lunn.ch>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */
#include <linux/kernel.h>
#include <linux/bitops.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/clkdev.h>
#include <linux/clk-provider.h>
#include <linux/clk/mvebu.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/syscore_ops.h>

struct mvebu_gating_ctrl {
	spinlock_t lock;
	struct clk **gates;
	int num_gates;
	void __iomem *base;
	u32 saved_reg;
};

struct mvebu_soc_descr {
	const char *name;
	const char *parent;
	int bit_idx;
};

#define to_clk_gate(_hw) container_of(_hw, struct clk_gate, hw)

static struct mvebu_gating_ctrl *ctrl;

static struct clk *mvebu_clk_gating_get_src(
	struct of_phandle_args *clkspec, void *data)
{
	int n;

	if (clkspec->args_count < 1)
		return ERR_PTR(-EINVAL);

	for (n = 0; n < ctrl->num_gates; n++) {
		struct clk_gate *gate =
			to_clk_gate(__clk_get_hw(ctrl->gates[n]));
		if (clkspec->args[0] == gate->bit_idx)
			return ctrl->gates[n];
	}
	return ERR_PTR(-ENODEV);
}

static int mvebu_clk_gating_suspend(void)
{
	ctrl->saved_reg = readl(ctrl->base);
	return 0;
}

static void mvebu_clk_gating_resume(void)
{
	writel(ctrl->saved_reg, ctrl->base);
}

static struct syscore_ops clk_gate_syscore_ops = {
	.suspend = mvebu_clk_gating_suspend,
	.resume = mvebu_clk_gating_resume,
};

static void __init mvebu_clk_gating_setup(
	struct device_node *np, const struct mvebu_soc_descr *descr)
{
	struct clk *clk;
	void __iomem *base;
	const char *default_parent = NULL;
	int n;

	if (ctrl) {
		pr_err("mvebu-clk-gating: cannot instantiate more than one gatable clock device\n");
		return;
	}

	base = of_iomap(np, 0);

	clk = of_clk_get(np, 0);
	if (!IS_ERR(clk)) {
		default_parent = __clk_get_name(clk);
		clk_put(clk);
	}

	ctrl = kzalloc(sizeof(struct mvebu_gating_ctrl), GFP_KERNEL);
	if (WARN_ON(!ctrl))
		return;

	spin_lock_init(&ctrl->lock);

	ctrl->base = base;

	/*
	 * Count, allocate, and register clock gates
	 */
	for (n = 0; descr[n].name;)
		n++;

	ctrl->num_gates = n;
	ctrl->gates = kzalloc(ctrl->num_gates * sizeof(struct clk *),
			      GFP_KERNEL);
	if (WARN_ON(!ctrl->gates)) {
		kfree(ctrl);
		return;
	}

	for (n = 0; n < ctrl->num_gates; n++) {
		u8 flags = 0;
		const char *parent =
			(descr[n].parent) ? descr[n].parent : default_parent;

		/*
		 * On Armada 370, the DDR clock is a special case: it
		 * isn't taken by any driver, but should anyway be
		 * kept enabled, so we mark it as IGNORE_UNUSED for
		 * now.
		 */
		if (!strcmp(descr[n].name, "ddr"))
			flags |= CLK_IGNORE_UNUSED;

		ctrl->gates[n] = clk_register_gate(NULL, descr[n].name, parent,
				   flags, base, descr[n].bit_idx, 0, &ctrl->lock);
		WARN_ON(IS_ERR(ctrl->gates[n]));
	}
	of_clk_add_provider(np, mvebu_clk_gating_get_src, ctrl);

	register_syscore_ops(&clk_gate_syscore_ops);
}

/*
 * SoC specific clock gating control
 */

#ifdef CONFIG_MACH_ARMADA_370
static const struct mvebu_soc_descr __initconst armada_370_gating_descr[] = {
	{ "audio", NULL, 0 },
	{ "pex0_en", NULL, 1 },
	{ "pex1_en", NULL,  2 },
	{ "ge1", NULL, 3 },
	{ "ge0", NULL, 4 },
	{ "pex0", "pex0_en", 5 },
	{ "pex1", "pex1_en", 9 },
	{ "sata0", NULL, 15 },
	{ "sdio", NULL, 17 },
	{ "tdm", NULL, 25 },
	{ "ddr", NULL, 28 },
	{ "sata1", NULL, 30 },
	{ }
};
#endif


#ifdef CONFIG_MACH_ARMADA_375
static const struct mvebu_soc_descr __initconst armada_375_gating_descr[] = {
	{ "tdmmc", NULL, 0 },
	{ "xpon", NULL, 1 },
	{ "mu", NULL, 2 },
	{ "pp", NULL, 3 },
	{ "ptp", NULL, 4 },
	{ "pex0", NULL, 5 },
	{ "pex1", NULL, 6 },
	{ "audio", NULL, 8 },
	{ "isi_slic", NULL, 9 },
	{ "zsi_slic", NULL, 10 },
	{ "nd_clk", "nand", 11 },
	{ "switch", NULL, 12 },
	{ "ssi_slic", NULL, 13 },
	{ "sata0_link", "sata0_core", 14 },
	{ "sata0_core", NULL, 15 },
	{ "usb3", NULL, 16 },
	{ "sdio", NULL, 17 },
	{ "usb", NULL, 18 },
	{ "gop", NULL, 19 },
	{ "sata1_link", "sata1_core", 20 },
	{ "sata1_core", NULL, 21 },
	{ "xor0", NULL, 22 },
	{ "xor1", NULL, 23 },
	{ "copro", NULL, 24 },
	{ "tdm", NULL, 25 },
	{ "usb_p1", NULL, 26 },
	{ "crypto0_enc", NULL, 28 },
	{ "crypto0_core", NULL, 29 },
	{ "crypto1_enc", NULL, 30 },
	{ "crypto1_core", NULL, 31 },
	{ }
};
#endif

#ifdef CONFIG_MACH_ARMADA_380
static const struct mvebu_soc_descr __initconst armada_380_gating_descr[] = {
	{ "audio", NULL, 0 },
	{ "ge2", NULL, 2 },
	{ "ge1", NULL, 3 },
	{ "ge0", NULL, 4 },
	{ "pex1", NULL, 5 },
	{ "pex2", NULL, 6 },
	{ "pex3", NULL, 7 },
	{ "pex0", NULL, 8 },
	{ "usb3h0", NULL, 9 },
	{ "usb3h1", NULL, 10 },
	{ "usb3d", NULL, 11 },
	{ "bm", NULL, 13 },
	{ "crypto0z", NULL, 14 },
	{ "sata0", NULL, 15 },
	{ "crypto1z", NULL, 16 },
	{ "sdio", NULL, 17 },
	{ "usb2", NULL, 18 },
	{ "crypto1", NULL, 21 },
	{ "xor0", NULL, 22 },
	{ "crypto0", NULL, 23 },
	{ "tdm", NULL, 25 },
	{ "xor1", NULL, 28 },
	{ "pnc", NULL, 29 },
	{ "sata1", NULL, 30 },
	{ }
};
#endif

#ifdef CONFIG_MACH_ARMADA_XP
static const struct mvebu_soc_descr __initconst armada_xp_gating_descr[] = {
	{ "audio", NULL, 0 },
	{ "ge3", NULL, 1 },
	{ "ge2", NULL,  2 },
	{ "ge1", NULL, 3 },
	{ "ge0", NULL, 4 },
	{ "pex00", NULL, 5 },
	{ "pex01", NULL, 6 },
	{ "pex02", NULL, 7 },
	{ "pex03", NULL, 8 },
	{ "pex10", NULL, 9 },
	{ "pex11", NULL, 10 },
	{ "pex12", NULL, 11 },
	{ "pex13", NULL, 12 },
	{ "bp", NULL, 13 },
	{ "sata0lnk", NULL, 14 },
	{ "sata0", "sata0lnk", 15 },
	{ "lcd", NULL, 16 },
	{ "sdio", NULL, 17 },
	{ "usb0", NULL, 18 },
	{ "usb1", NULL, 19 },
	{ "usb2", NULL, 20 },
	{ "xor0", NULL, 22 },
	{ "crypto", NULL, 23 },
	{ "tdm", NULL, 25 },
	{ "pex20", NULL, 26 },
	{ "pex30", NULL, 27 },
	{ "xor1", NULL, 28 },
	{ "sata1lnk", NULL, 29 },
	{ "sata1", "sata1lnk", 30 },
	{ }
};
#endif

#ifdef CONFIG_MACH_MSYS
static const struct mvebu_soc_descr __initconst msys_gating_descr[] = {
	{ "ge1", NULL, 3 },
	{ "ge0", NULL, 4 },
	{ "pex00", NULL, 5 },
	{ "sdio", NULL, 17 },
	{ "usb0", NULL, 18 },
	{ "xor0", NULL, 22 },
	{ }
};
#endif

#ifdef CONFIG_ARCH_DOVE
static const struct mvebu_soc_descr __initconst dove_gating_descr[] = {
	{ "usb0", NULL, 0 },
	{ "usb1", NULL, 1 },
	{ "ge", "gephy", 2 },
	{ "sata", NULL, 3 },
	{ "pex0", NULL, 4 },
	{ "pex1", NULL, 5 },
	{ "sdio0", NULL, 8 },
	{ "sdio1", NULL, 9 },
	{ "nand", NULL, 10 },
	{ "camera", NULL, 11 },
	{ "i2s0", NULL, 12 },
	{ "i2s1", NULL, 13 },
	{ "crypto", NULL, 15 },
	{ "ac97", NULL, 21 },
	{ "pdma", NULL, 22 },
	{ "xor0", NULL, 23 },
	{ "xor1", NULL, 24 },
	{ "gephy", NULL, 30 },
	{ }
};
#endif

#ifdef CONFIG_ARCH_KIRKWOOD
static const struct mvebu_soc_descr __initconst kirkwood_gating_descr[] = {
	{ "ge0", NULL, 0 },
	{ "pex0", NULL, 2 },
	{ "usb0", NULL, 3 },
	{ "sdio", NULL, 4 },
	{ "tsu", NULL, 5 },
	{ "runit", NULL, 7 },
	{ "xor0", NULL, 8 },
	{ "audio", NULL, 9 },
	{ "powersave", "cpuclk", 11 },
	{ "sata0", NULL, 14 },
	{ "sata1", NULL, 15 },
	{ "xor1", NULL, 16 },
	{ "crypto", NULL, 17 },
	{ "pex1", NULL, 18 },
	{ "ge1", NULL, 19 },
	{ "tdm", NULL, 20 },
	{ }
};
#endif

static const __initdata struct of_device_id clk_gating_match[] = {
#ifdef CONFIG_MACH_ARMADA_370
	{
		.compatible = "marvell,armada-370-gating-clock",
		.data = armada_370_gating_descr,
	},
#endif

#ifdef CONFIG_MACH_ARMADA_375
	{
		.compatible = "marvell,armada-375-gating-clock",
		.data = armada_375_gating_descr,
	},
#endif

#ifdef CONFIG_MACH_ARMADA_380
	{
		.compatible = "marvell,armada-380-gating-clock",
		.data = armada_380_gating_descr,
	},
#endif

#ifdef CONFIG_MACH_ARMADA_XP
	{
		.compatible = "marvell,armada-xp-gating-clock",
		.data = armada_xp_gating_descr,
	},
#endif

#ifdef CONFIG_MACH_MSYS
	{
		.compatible = "marvell,msys-gating-clock",
		.data = msys_gating_descr,
	},
#endif

#ifdef CONFIG_ARCH_DOVE
	{
		.compatible = "marvell,dove-gating-clock",
		.data = dove_gating_descr,
	},
#endif

#ifdef CONFIG_ARCH_KIRKWOOD
	{
		.compatible = "marvell,kirkwood-gating-clock",
		.data = kirkwood_gating_descr,
	},
#endif

	{ }
};

void __init mvebu_gating_clk_init(void)
{
	struct device_node *np;

	for_each_matching_node(np, clk_gating_match) {
		const struct of_device_id *match =
			of_match_node(clk_gating_match, np);
		mvebu_clk_gating_setup(np,
		       (const struct mvebu_soc_descr *)match->data);
	}
}
