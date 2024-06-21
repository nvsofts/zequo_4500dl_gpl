/*
 * ID and revision information for mvebu SoCs
 *
 * Copyright (C) 2014 Marvell
 *
 * Gregory CLEMENT <gregory.clement@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2.  This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 *
 * All the mvebu SoCs have information related to their variant and
 * revision that can be read from the PCI control register. This is
 * done before the PCI initialization to avoid any conflict. Once the
 * ID and revision are retrieved, the mapping is freed.
 */

#define pr_fmt(fmt) "mvebu-soc-id: " fmt

#include <linux/i2c.h>
#include <linux/clk.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/kernel.h>
#include <linux/of.h>
#include <linux/of_address.h>
#include "mvebu-soc-id.h"

#define PCIE_DEV_ID_OFF		0x0
#define PCIE_DEV_REV_OFF	0x8
#define A38X_DEV_ID_OFF		0x38
#define A38X_DEV_REV_OFF	0x3C

#define SOC_ID_MASK		0xFFFF0000
#define SOC_REV_MASK		0xFF
#define A38X_REV_MASK		0xF
#define MSYS_DEV_ID_MASK	0xFF00
#define MTS_MSYS_DEV_ID_MASK   0xFFFF
#define MSYS_REV_MASK		0xF

static u32 mts_dev_id;
static u32 soc_dev_id;
static u32 soc_rev;
static bool is_id_valid;


static const struct of_device_id mvebu_pcie_of_match_table[] = {
	{ .compatible = "marvell,armada-xp-pcie", },
	{ .compatible = "marvell,armada-370-pcie", },
	{},
};

static const struct of_device_id mvebu_a38x_of_match_table[] = {
	{ .compatible = "marvell,armada-380-system-controller", },
	{ .compatible = "marvell,msys-soc-id", },
	{},
};


int mvebu_get_mts_PP_id(u32 *dev, u32 *rev)
{
	if (is_id_valid && mts_dev_id) {
		*dev = mts_dev_id;
		*rev = soc_rev;
		return 0;
	} else
		return -1;
}

int mvebu_get_soc_id(u32 *dev, u32 *rev)
{
	if (is_id_valid) {
		*dev = soc_dev_id;
		*rev = soc_rev;
		return 0;
	} else
		return -1;
}

static int __init mvebu_soc_id_init(void)
{
	struct device_node *np;
	int ret = 0;
	void __iomem *reg_base;
	struct clk *clk;
	struct device_node *child;
	bool is_pcie_id;

	np = of_find_matching_node(NULL, mvebu_pcie_of_match_table);
	if (!np) {/* If no pcie for soc-id, try A38x deicated register */
		np = of_find_matching_node(NULL, mvebu_a38x_of_match_table);
		if (!np)
			return ret;
		is_pcie_id = false;
	} else {
		is_pcie_id = true;

		/*
		 * ID and revision are available from any port, so we
		 * just pick the first one
		 */
		child = of_get_next_child(np, NULL);
		if (child == NULL) {
			pr_err("cannot get pci node\n");
			ret = -ENOMEM;
			goto clk_err;
		}

		clk = of_clk_get_by_name(child, NULL);
		if (IS_ERR(clk)) {
			pr_err("cannot get clock\n");
			ret = -ENOMEM;
			goto clk_err;
		}

		ret = clk_prepare_enable(clk);
		if (ret) {
			pr_err("cannot enable clock\n");
			goto clk_err;
		}
	}
	if (is_pcie_id == true)
		reg_base = of_iomap(child, 0);
	else
		reg_base = of_iomap(np, 0);
	if (IS_ERR(reg_base)) {
		pr_err("cannot map registers\n");
		ret = -ENOMEM;
		goto res_ioremap;
	}

	if (is_pcie_id == true) {
		/* SoC ID */
		mts_dev_id = soc_dev_id = readl(reg_base + PCIE_DEV_ID_OFF) >> 16;
		/* SoC revision */
		soc_rev = readl(reg_base + PCIE_DEV_REV_OFF) & SOC_REV_MASK;
	} else if (of_device_is_compatible(np, "marvell,msys-soc-id")) {
		/* SoC ID */
		soc_dev_id = (readl(reg_base) >> 12) & MSYS_DEV_ID_MASK;
		mts_dev_id = (readl(reg_base) >> 12) & MTS_MSYS_DEV_ID_MASK;
		/* SoC revision */
		soc_rev = (readl(reg_base) >> 28) & MSYS_REV_MASK;
	} else {
		/* SoC ID */
		mts_dev_id = soc_dev_id = readl(reg_base + A38X_DEV_ID_OFF) >> 16;
		/* SoC revision */
		soc_rev = (readl(reg_base + A38X_DEV_REV_OFF) >> 8) & A38X_REV_MASK;
	}

	is_id_valid = true;

	pr_info("MVEBU SoC ID=0x%X, Rev=0x%X\n", soc_dev_id, soc_rev);

	iounmap(reg_base);

res_ioremap:
	/*
	 * If the PCIe unit is actually enabled and we have PCI
	 * support in the kernel, we intentionally do not release the
	 * reference to the clock. We want to keep it running since
	 * the bootloader does some PCIe link configuration that the
	 * kernel is for now unable to do, and gating the clock would
	 * make us loose this precious configuration.
	 */
	if (is_pcie_id == true && (!of_device_is_available(child) ||
				   !IS_ENABLED(CONFIG_PCI_MVEBU))) {
		clk_disable_unprepare(clk);
		clk_put(clk);
	}

clk_err:
	if (is_pcie_id == true)
		of_node_put(child);
	of_node_put(np);

	return ret;
}
core_initcall(mvebu_soc_id_init);

/************************************************************************
 *
 * mvBoardIdGet: Obtain board ID from on-board EEPROM
 */
int mvBoardIdGet(void)
{
	struct i2c_adapter	*adapter;
	struct i2c_msg		msg[2];
	u8			msgbuf[2], reg;
	int			ret;
	u8			i2c_bus_num = MV_EEPROM_I2C_BUS_NUM;
	u8			i2c_device_addr = MV_EEPROM_I2C_DEV_ADDR;
	u16			board_id_reg_num = MV_EEPROM_I2C_BOARD_ID_REG;
	u32			regs[2];
	struct device_node	*eeprom_node;
	struct device_node	*prestera_node;

	/* Check if device tree requests Board ID enforcement */
	prestera_node = of_find_node_by_name(NULL, "prestera");
	if (prestera_node != NULL) {
		/* No further processing if the Board ID forced to a fixed value by device tree */
		ret = of_property_read_u32(prestera_node, "cpss_force_board_id", &regs[0]);
		if (ret >= 0)
			return regs[0];
	}

	/* Try to get board ID register, i2c bus nymber and EEPROM address from DTB */
	eeprom_node = of_find_node_by_name(NULL, "eeprom");
	if (eeprom_node != NULL) {

		/* EEPROM i2c bus address */
		ret = of_property_read_u32(eeprom_node, "reg", &regs[0]);
		if (ret >= 0)
			i2c_device_addr = regs[0];

		/* Board ID register number inside EEPROM */
		ret = of_property_read_u32(eeprom_node, "marvell,board_id_reg", &regs[0]);
		if (ret >= 0)
			board_id_reg_num = regs[0];

		/* parent is i2c adapter */
		ret = of_property_read_u32_array(eeprom_node->parent, "reg", regs, 2);
		if (ret >= 0)
			i2c_bus_num = (regs[0] >> 8) & 0xF; /* 0x11000 for i2c0 or 0x11100 for i2c1 */
	}

	/* The Board Id is normally taken from the first address-value pair of the EEPROM initalization sequence
	   In order to support normal TWSI init sequence flow, the first pair of DWORDS on EEPROM
	   should contain an address (bytes 0-3) of some scratch pad register (for instance an UART SCR)
	   and a value (bytes 4-7), which will be partially interpreted as Board ID (bits[2:0] of byte 7)
	*/
	adapter = i2c_get_adapter(i2c_bus_num);
	if (!adapter) {
		printk(KERN_ERR "failed to get i2c%d adapter\n", i2c_bus_num);
		return -ENODEV;
	}

	/* Build the 16-bit register address since the EEPROM uses 16-bit addressing mode */
	msgbuf[0] = board_id_reg_num >> 8;	/* MSB */
	msgbuf[1] = board_id_reg_num;		/* LSB */

	/* First send the register address */
	msg[0].addr = i2c_device_addr;
	msg[0].flags = 0;
	msg[0].len = 2;
	msg[0].buf = msgbuf;

	/* Second read the register value  */
	msg[1].addr = i2c_device_addr;
	msg[1].flags = I2C_M_RD;
	msg[1].len = 1;
	msg[1].buf = &reg;

	ret = i2c_transfer(adapter, msg, 2);
	if (ret != 2) {
		printk(KERN_ERR "Bad i2c message count %d", ret);
		return -EIO;
	}

	i2c_put_adapter(adapter);

	return reg & 0x7;
}
