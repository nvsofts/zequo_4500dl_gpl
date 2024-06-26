/*
 * Driver for the MDIO interface of Marvell network interfaces.
 *
 * Since the MDIO interface of Marvell network interfaces is shared
 * between all network interfaces, having a single driver allows to
 * handle concurrent accesses properly (you may have four Ethernet
 * ports, but they in fact share the same SMI interface to access the
 * MDIO bus). Moreover, this MDIO interface code is similar between
 * the mv643xx_eth driver and the mvneta driver. For now, it is only
 * used by the mvneta driver, but it could later be used by the
 * mv643xx_eth driver as well.
 *
 * Copyright (C) 2012 Marvell
 *
 * Thomas Petazzoni <thomas.petazzoni@free-electrons.com>
 *
 * This file is licensed under the terms of the GNU General Public
 * License version 2. This program is licensed "as is" without any
 * warranty of any kind, whether express or implied.
 */

#include <linux/init.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/mutex.h>
#include <linux/phy.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/delay.h>
#include <linux/io.h>
#include <linux/clk.h>
#include <linux/of_device.h>
#include <linux/of_mdio.h>
#include <linux/sched.h>
#include <linux/wait.h>
#include <linux/mv_addr_compl.h>


#define MVMDIO_SMI_DATA_SHIFT              0
#define MVMDIO_SMI_PHY_ADDR_SHIFT          16
#define MVMDIO_SMI_PHY_REG_SHIFT           21
#define MVMDIO_SMI_READ_OPERATION          BIT(26)
#define MVMDIO_SMI_WRITE_OPERATION         0
#define MVMDIO_SMI_READ_VALID              BIT(27)
#define MVMDIO_SMI_BUSY                    BIT(28)
#define MVMDIO_ERR_INT_CAUSE		   0x007C
#define  MVMDIO_ERR_INT_SMI_DONE	   0x00000010
#define MVMDIO_ERR_INT_MASK		   0x0080

/* PP SMI Registers */
#define PP_SMI_MNGT                        0x0
#define PP_SMI_MISC_CONF_BC2               0x4
#define PP_SMI_MISC_CONF_BOBK              0x4
#define PP_SMI_MISC_CONF_AC3               0x1AC

/* Bits definitions: PP_SMI_MISC_CONF */
#define PP_SMI_INV_BOBCAT2                 BIT(10)
#define PP_SMI_INV_BOBK                    BIT(11)
#define PP_SMI_INV_AC3                     BIT(20)


struct orion_mdio_dev {
	struct mutex lock;
	void __iomem *regs;
	struct clk *clk;
	/*
	 * If we have access to the error interrupt pin (which is
	 * somewhat misnamed as it not only reflects internal errors
	 * but also reflects SMI completion), use that to wait for
	 * SMI access completion instead of polling the SMI busy bit.
	 */
	int err_interrupt;
	/* The parameter is only indicate whether SMI is indirect accessed for MSYS platform. */
	bool smi_indirect_access;
	wait_queue_head_t smi_busy_wait;
};

static int orion_mdio_smi_is_done(struct orion_mdio_dev *dev)
{
	return !(readl(dev->regs) & MVMDIO_SMI_BUSY);
}

/* Wait for the SMI unit to be ready for another operation
 */
static int orion_mdio_wait_ready(struct mii_bus *bus)
{
	struct orion_mdio_dev *dev = bus->priv;
	int count;

	if (dev->err_interrupt <= 0) {
		count = 0;
		while (1) {
			if (orion_mdio_smi_is_done(dev))
				break;

			if (count > 100) {
				dev_err(bus->parent,
					"Timeout: SMI busy for too long\n");
				return -ETIMEDOUT;
			}

			udelay(10);
			count++;
		}
	} else {
		if (!orion_mdio_smi_is_done(dev)) {
			wait_event_timeout(dev->smi_busy_wait,
				orion_mdio_smi_is_done(dev),
				msecs_to_jiffies(100));
			if (!orion_mdio_smi_is_done(dev))
				return -ETIMEDOUT;
		}
	}

	return 0;
}

static int orion_mdio_read(struct mii_bus *bus, int mii_id,
			   int regnum)
{
	struct orion_mdio_dev *dev = bus->priv;
	int count;
	u32 val;
	int ret;

	mutex_lock(&dev->lock);

	ret = orion_mdio_wait_ready(bus);
	if (ret < 0) {
		mutex_unlock(&dev->lock);
		return ret;
	}

	writel(((mii_id << MVMDIO_SMI_PHY_ADDR_SHIFT) |
		(regnum << MVMDIO_SMI_PHY_REG_SHIFT)  |
		MVMDIO_SMI_READ_OPERATION),
	       dev->regs);

	/* Wait for the value to become available */
	count = 0;
	while (1) {
		val = readl(dev->regs);
		if (val & MVMDIO_SMI_READ_VALID)
			break;

		if (count > 100) {
			dev_err(bus->parent, "Timeout when reading PHY\n");
			mutex_unlock(&dev->lock);
			return -ETIMEDOUT;
		}

		udelay(10);
		count++;
	}

	mutex_unlock(&dev->lock);

	return val & 0xFFFF;
}

static int orion_mdio_write(struct mii_bus *bus, int mii_id,
			    int regnum, u16 value)
{
	struct orion_mdio_dev *dev = bus->priv;
	int ret;

	mutex_lock(&dev->lock);

	ret = orion_mdio_wait_ready(bus);
	if (ret < 0) {
		mutex_unlock(&dev->lock);
		return ret;
	}

	writel(((mii_id << MVMDIO_SMI_PHY_ADDR_SHIFT) |
		(regnum << MVMDIO_SMI_PHY_REG_SHIFT)  |
		MVMDIO_SMI_WRITE_OPERATION            |
		(value << MVMDIO_SMI_DATA_SHIFT)),
	       dev->regs);

	mutex_unlock(&dev->lock);

	return 0;
}

static int orion_mdio_reset(struct mii_bus *bus)
{
	struct orion_mdio_dev *dev = bus->priv;
	u32 smi_conf, pp_smi_misc_off;

	mutex_lock(&dev->lock);

	if (dev->smi_indirect_access) {
		/* Disable MDC inversion */
		if (of_machine_is_compatible("marvell,msys-bc2")) {
			pp_smi_misc_off = PP_SMI_MISC_CONF_BC2;
			smi_conf = readl(dev->regs + pp_smi_misc_off);
			smi_conf &= ~PP_SMI_INV_BOBCAT2;
		} else if (of_machine_is_compatible("marvell,msys-bobk")) {
			pp_smi_misc_off = PP_SMI_MISC_CONF_BOBK;
			smi_conf = readl(dev->regs + pp_smi_misc_off);
			smi_conf &= ~PP_SMI_INV_BOBK;
		} else if (of_machine_is_compatible("marvell,msys-ac3")) {
			pp_smi_misc_off = PP_SMI_MISC_CONF_AC3;
			smi_conf = readl(dev->regs + pp_smi_misc_off);
			smi_conf &= ~PP_SMI_INV_AC3;
		} else {
			mutex_unlock(&dev->lock);
			dev_err(&bus->dev, "SMI indirect access is enabled while mdio reset is not supported!\n");
			return -EPERM;
		}

		writel(smi_conf, dev->regs + pp_smi_misc_off);
	}

	mutex_unlock(&dev->lock);

	return 0;
}

static irqreturn_t orion_mdio_err_irq(int irq, void *dev_id)
{
	struct orion_mdio_dev *dev = dev_id;

	if (readl(dev->regs + MVMDIO_ERR_INT_CAUSE) &
			MVMDIO_ERR_INT_SMI_DONE) {
		writel(~MVMDIO_ERR_INT_SMI_DONE,
				dev->regs + MVMDIO_ERR_INT_CAUSE);
		wake_up(&dev->smi_busy_wait);
		return IRQ_HANDLED;
	}

	return IRQ_NONE;
}

static const struct of_device_id orion_mdio_match[] = {
	{ .compatible = "marvell,orion-mdio" },
	{ }
};
MODULE_DEVICE_TABLE(of, orion_mdio_match);

static int orion_mdio_probe(struct platform_device *pdev)
{
	struct resource *r;
	struct mii_bus *bus;
	struct orion_mdio_dev *dev;
	const struct of_device_id *of_id;
	phys_addr_t smi_base_addr;
	int i, ret;

	of_id = of_match_device(orion_mdio_match, &pdev->dev);
	if (!of_id) {
		dev_err(&pdev->dev, "no device found\n");
		return -ENODEV;
	}

	r = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (!r) {
		dev_err(&pdev->dev, "No SMI register address given\n");
		return -ENODEV;
	}

	bus = mdiobus_alloc_size(sizeof(struct orion_mdio_dev));
	if (!bus) {
		dev_err(&pdev->dev, "Cannot allocate MDIO bus\n");
		return -ENOMEM;
	}

	bus->name = "orion_mdio_bus";
	bus->read = orion_mdio_read;
	bus->write = orion_mdio_write;
	bus->reset = orion_mdio_reset;
	snprintf(bus->id, MII_BUS_ID_SIZE, "%s-mii",
		 dev_name(&pdev->dev));
	bus->parent = &pdev->dev;

	bus->irq = kmalloc(sizeof(int) * PHY_MAX_ADDR, GFP_KERNEL);
	if (!bus->irq) {
		mdiobus_free(bus);
		return -ENOMEM;
	}

	for (i = 0; i < PHY_MAX_ADDR; i++)
		bus->irq[i] = PHY_POLL;

	dev = bus->priv;

	/* The parameter 'smi_indirect_access' indicates wthether MSYS
	  * access the PP SMI registers indirectly. In general, the external
	  * PHY is supposed to connect to Gbe MAC's SMI interface
	  * directly, 'smi_indirect_access' is false by default.
	  */
	if (of_find_property(pdev->dev.of_node, "smi-indirect-access", NULL)) {
		unsigned int compl_region_id;

		dev->smi_indirect_access = true;

		/* indirect access SMI requires ADdr/Compl */
		ret = mv_addr_compl_request_region(r->start, &compl_region_id, &smi_base_addr);
		if (ret) {
			dev_err(&pdev->dev, "Failed to request address completion region for SMI indirect access\n");
			ret = -ENODEV;
			goto out_mdio;
		}
	} else {
		dev->smi_indirect_access = false;
		smi_base_addr = r->start;
	}

	dev->regs = devm_ioremap(&pdev->dev, smi_base_addr, resource_size(r));
	if (!dev->regs) {
		dev_err(&pdev->dev, "Unable to remap SMI register\n");
		ret = -ENODEV;
		goto out_mdio;
	}

	init_waitqueue_head(&dev->smi_busy_wait);

	/*
	  * Notes:
	  * In SMI indirect-access mode, clock gate control is managed
	  * by switch core. SMI interrupt is not supposed to be routined
	  * to MSYS. Therefore, both are not supported.
	  */
	dev->clk = devm_clk_get(&pdev->dev, NULL);
	if (!IS_ERR(dev->clk) && !dev->smi_indirect_access)
		clk_prepare_enable(dev->clk);
	else
		dev->clk = NULL;

	dev->err_interrupt = platform_get_irq(pdev, 0);
	if (dev->err_interrupt != -ENXIO && !dev->smi_indirect_access) {
		ret = devm_request_irq(&pdev->dev, dev->err_interrupt,
					orion_mdio_err_irq,
					IRQF_SHARED, pdev->name, dev);
		if (ret)
			goto out_mdio;

		writel(MVMDIO_ERR_INT_SMI_DONE,
			dev->regs + MVMDIO_ERR_INT_MASK);
	}
	mutex_init(&dev->lock);

	if (pdev->dev.of_node)
		ret = of_mdiobus_register(bus, pdev->dev.of_node);
	else
		ret = mdiobus_register(bus);
	if (ret < 0) {
		dev_err(&pdev->dev, "Cannot register MDIO bus (%d)\n", ret);
		goto out_mdio;
	}

	platform_set_drvdata(pdev, bus);

	return 0;

out_mdio:
	if (!dev->clk)
		clk_disable_unprepare(dev->clk);
	kfree(bus->irq);
	mdiobus_free(bus);
	return ret;
}

static int orion_mdio_remove(struct platform_device *pdev)
{
	struct mii_bus *bus = platform_get_drvdata(pdev);
	struct orion_mdio_dev *dev = bus->priv;

	if (!dev->smi_indirect_access)
		writel(0, dev->regs + MVMDIO_ERR_INT_MASK);
	mdiobus_unregister(bus);
	kfree(bus->irq);
	mdiobus_free(bus);
	if (!dev->clk)
		clk_disable_unprepare(dev->clk);

	return 0;
}

static struct platform_driver orion_mdio_driver = {
	.probe = orion_mdio_probe,
	.remove = orion_mdio_remove,
	.driver = {
		.name = "orion-mdio",
		.of_match_table = orion_mdio_match,
	},
};

module_platform_driver(orion_mdio_driver);

MODULE_DESCRIPTION("Marvell MDIO interface driver");
MODULE_AUTHOR("Thomas Petazzoni <thomas.petazzoni@free-electrons.com>");
MODULE_LICENSE("GPL");
MODULE_ALIAS("platform:orion-mdio");
