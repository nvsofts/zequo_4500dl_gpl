/*******************************************************************************
   Copyright (C) Marvell International Ltd. and its affiliates

   This software file (the "File") is owned and distributed by Marvell
   International Ltd. and/or its affiliates ("Marvell") under the following
   alternative licensing terms.  Once you have made an election to distribute the
   File under one of the following license alternatives, please (i) delete this
   introductory statement regarding license alternatives, (ii) delete the two
   license alternatives that you have not elected to use and (iii) preserve the
   Marvell copyright notice above.

********************************************************************************
   Marvell Commercial License Option

   If you received this File from Marvell and you have entered into a commercial
   license agreement (a "Commercial License") with Marvell, the File is licensed
   to you under the terms of the applicable Commercial License.

********************************************************************************
   Marvell GPL License Option

   If you received this File from Marvell, you may opt to use, redistribute and/or
   modify this File in accordance with the terms and conditions of the General
   Public License Version 2, June 1991 (the "GPL License"), a copy of which is
   available along with the File in the license.txt file.

   THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
   WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
   DISCLAIMED.  The GPL License provides additional details about this warranty
   disclaimer.
********************************************************************************
   Marvell BSD License Option

   If you received this File from Marvell, you may opt to use, redistribute and/or
   modify this File under the following licensing terms.
   Redistribution and use in source and binary forms, with or without modification,
   are permitted provided that the following conditions are met:

*   Redistributions of source code must retain the above copyright notice,
    this list of conditions and the following disclaimer.

*   Redistributions in binary form must reproduce the above copyright
    notice, this list of conditions and the following disclaimer in the
    documentation and/or other materials provided with the distribution.

*   Neither the name of Marvell nor the names of its contributors may be
    used to endorse or promote products derived from this software without
    specific prior written permission.

   THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
   ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
   WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
   DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
   ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
   (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
   LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
   ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
   (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
   SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
********************************************************************************
* mv_addr_compl.c
*
* DESCRIPTION:
*	address completion support for MSYS devices (BobK/BC2).
*
* DEPENDENCIES:
*
*******************************************************************************/

#include <linux/init.h>
#include <linux/io.h>
#include <linux/err.h>
#include <linux/kernel.h>
#include <linux/module.h>
#include <linux/platform_device.h>
#ifdef CONFIG_OF
#include <linux/of.h>
#include <linux/of_address.h>
#include <linux/of_device.h>
#ifdef CONFIG_MV_INCLUDE_PRESTERA_PCI
#include <linux/pci.h>
#include "mv_prestera_pci.h"
#endif
#endif
#include <linux/mv_addr_compl.h>

/* MG Registers */
#define PP_ADDCOMPL_LEGACY                 0x0
#define PP_ADDCOMPL_NORMAL(i)              (0x120 + (i << 2))
#define PP_ADDCOMPL_CTRL                   0x140

/* Bits definitions: PP_ADDCOMPL_CTRL */
#define PP_LEGACY_MODE                     BIT(16)

/* Address Region IDs */
#define MSYS_REGION_0		0
#define MSYS_REGION_1		1
#define MSYS_REGION_2		2
#define MSYS_REGION_3		3
#define MSYS_REGION_4		4
#define MSYS_REGION_5		5
#define MSYS_REGION_6		6
#define MSYS_REGION_7		7
#define MSYS_REGION_NUM		8

/* Legacy Address Completion */
#define MSYS_INTR_ADDR_BITS_LEGACY	24
#define MSYS_REGION_BITS_LEGACY		2
#define MSYS_REGION_MASK_LEGACY		((1 << MSYS_REGION_BITS_LEGACY) - 1)
#define MSYS_REGION_SHIFT_LEGACY	MSYS_INTR_ADDR_BITS_LEGACY
#define MSYS_REMAP_ADDR_BITS_LEGACY	8
#define MSYS_REMAP_MASK_LEGACY		((1 << MSYS_REMAP_ADDR_BITS_LEGACY) - 1)
#define MSYS_REMAP_SHIFT_LEGACY		MSYS_INTR_ADDR_BITS_LEGACY

/* Normal Address Completion */
#define MSYS_INTR_ADDR_BITS_NORMAL	19
#define MSYS_REGION_BITS_NORMAL		3
#define MSYS_REGION_MASK_NORMAL		((1 << MSYS_REGION_BITS_NORMAL) - 1)
#define MSYS_REGION_SHIFT_NORMAL	MSYS_INTR_ADDR_BITS_NORMAL
#define MSYS_REMAP_ADDR_BITS_NORMAL	13
#define MSYS_REMAP_MASK_NORMAL		((1 << MSYS_REMAP_ADDR_BITS_NORMAL) - 1)
#define MSYS_REMAP_SHIFT_NORMAL		MSYS_INTR_ADDR_BITS_NORMAL

struct msys_region_reg {
	unsigned int offset;
	unsigned int region_shift;
};

struct msys_addr_compl {
	unsigned int           max_region;
	unsigned int           region_offset;
	unsigned int           region_mask;
	unsigned int           remap_offset;
	unsigned int           remap_mask;
	struct msys_region_reg *region_regs;
};

struct msys_region {
	unsigned int ref_cnt;
	phys_addr_t phys_addr;
	phys_addr_t compl_addr;
};

static struct msys_region_reg msys_region_regs_legacy[] = {
	{
		.offset       = PP_ADDCOMPL_LEGACY,
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_LEGACY,
		.region_shift = 8,
	},
	{
		.offset       = PP_ADDCOMPL_LEGACY,
		.region_shift = 16,
	},
	{
		.offset       = PP_ADDCOMPL_LEGACY,
		.region_shift = 24,
	},
};

static struct msys_region_reg msys_region_regs_normal[] = {
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_0),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_1),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_2),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_3),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_4),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_5),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_6),
		.region_shift = 0,
	},
	{
		.offset       = PP_ADDCOMPL_NORMAL(MSYS_REGION_7),
		.region_shift = 0,
	},
};

static struct msys_addr_compl msys_addr_compl_tbl[ADDRCOMPL_MAX] = {
	{
		.max_region    = ARRAY_SIZE(msys_region_regs_legacy),
		.region_offset = MSYS_REGION_SHIFT_LEGACY,
		.region_mask   = MSYS_REGION_MASK_LEGACY,
		.remap_offset  = MSYS_REMAP_SHIFT_LEGACY,
		.remap_mask    = MSYS_REMAP_MASK_LEGACY,
		.region_regs   = msys_region_regs_legacy,
	},
	{
		.max_region    = ARRAY_SIZE(msys_region_regs_normal),
		.region_offset = MSYS_REGION_SHIFT_NORMAL,
		.region_mask   = MSYS_REGION_MASK_NORMAL,
		.remap_offset  = MSYS_REMAP_SHIFT_NORMAL,
		.remap_mask    = MSYS_REMAP_MASK_NORMAL,
		.region_regs   = msys_region_regs_normal,
	},
};

static void __iomem *addr_compl_reg_base_virtual;
static phys_addr_t addr_compl_reg_base_physical;
static enum msys_addr_compl_mode addr_compl_mode;
static struct msys_region msys_regions[MSYS_REGION_NUM];
static spinlock_t addr_compl_lock;

static const struct of_device_id msys_addr_compl_of_match_table[] = {
	{ .compatible = "marvell,msys-addr-compl", },
	{},
};

/*******************************************************************************
* mv_addr_compl_reg_base_set
* DESCRIPTION: set physical and virtual address of address completion register base
*
* INPUT:  reg_base_physical - address completion register base physical address
*             reg_base_virtual   - address completion register base vitual address
* OUTPUT: None.
* RETURN: None
*******************************************************************************/
void mv_addr_compl_reg_base_set(phys_addr_t reg_base_physical, void __iomem *reg_base_virtual)
{
	addr_compl_reg_base_physical = reg_base_physical;
	addr_compl_reg_base_virtual = reg_base_virtual;

	return;
}

/*******************************************************************************
* mv_addr_compl_mode_set
* DESCRIPTION: set address completion mode
*
* INPUT:  mode - address completion mode, legacy(0) and normal(1)
* OUTPUT: None.
* RETURN: None
*******************************************************************************/
void mv_addr_compl_mode_set(enum msys_addr_compl_mode mode)
{
	addr_compl_mode = mode;
	return;
}

/*******************************************************************************
* mv_addr_compl_init
* DESCRIPTION: address completion initialization
*
* INPUT:  mode - address completion mode, 0 - legacy 1 - normal
* OUTPUT: None.
* RETURN: 0:succes, otherwise fail
*******************************************************************************/
int mv_addr_compl_init(void)
{
	struct msys_addr_compl *addr_compl;
	u32 ctrl;
	int i;

	if (!addr_compl_reg_base_virtual)
		return -EPERM;

	spin_lock_init(&addr_compl_lock);

	ctrl = readl(addr_compl_reg_base_virtual + PP_ADDCOMPL_CTRL);
	/*set address completion mode*/
	if (addr_compl_mode == ADDRCOMPL_LEGACY) {
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_LEGACY];
		ctrl |= PP_LEGACY_MODE;
	} else {
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_NORMAL];
		ctrl &= ~PP_LEGACY_MODE;
	}

#if 0
	/*In address completion initialization, we should deactive all regions except region0(for MG),
	   yet we find an issue that once we mask the bit2, we can not access the MG register(region0
	   should be corresponding to bit0), so now the deactiving codes are commented and all regions
	   are active by default*/
	/*
	 * 1. During initialization, all addr-compl regions except region 0
	 *    are supposed to be disabled.
	 * 2. The region should be actived/deactived on demand by API call
	 *    - mv_addr_compl_request_region
	 *    - mv_addr_compl_release_region
	 * 3. Since region 0 is used for addr-compl register access, it should
	 *     be never disabled.
	 */
	for (i = 1; i < addr_compl->max_region; i++)
		ctrl |= (1 << i);
#else
	/* In address completion initialization, we find an issue that once we mask the bit2,
	  * we can not access the MG register(region0 should be corresponding to bit0),
	  * so now we active all regions in hardware by default and enable/disable regions
	  * in software logic */
	for (i = 1; i < addr_compl->max_region; i++)
		ctrl &= ~(1 << i);
#endif
	writel(ctrl, addr_compl_reg_base_virtual + PP_ADDCOMPL_CTRL);
	return 0;
}

/*******************************************************************************
* mv_addr_compl_request_region
* DESCRIPTION: requestion a address completion region and do the the address completion
*
* INPUT:  addr - the physcial address offset in cpss, which will be transfered by address completion
* OUTPUT: compl_region_id - address completion region ID
*            : compl_addr - the transfered physcial address by addrees completion
* RETURN: 0:succes, otherwise fail
*******************************************************************************/
int mv_addr_compl_request_region(phys_addr_t addr, unsigned int *compl_region_id, phys_addr_t *compl_addr)
{
	unsigned int		region_id = 0;
	unsigned int		free_region_id = MSYS_REGION_NUM;
	struct msys_addr_compl	*addr_compl;
	struct msys_region_reg	*region_reg;
	u32			region;
	phys_addr_t		tmp_addr = addr;

	if (!addr_compl_reg_base_virtual)
		return -EPERM;

	if (addr_compl_mode == ADDRCOMPL_LEGACY)
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_LEGACY];
	else
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_NORMAL];

	spin_lock(&addr_compl_lock);
	for (region_id = addr_compl->max_region - 1; region_id > 0; region_id--) {
		if (msys_regions[region_id].ref_cnt && (msys_regions[region_id].phys_addr == addr)) {
			/* reuse the existing region */
			*compl_region_id = region_id;
			*compl_addr = msys_regions[region_id].compl_addr;
			msys_regions[region_id].ref_cnt++;
			spin_unlock(&addr_compl_lock);
			return 0;
		}
	}

	/* find a free region, since the region from the beginning is occupied by other modules,
	we start from the last to aviod conflict. */
	for (region_id = addr_compl->max_region - 1; region_id > 0; region_id--) {
		if (0 == msys_regions[region_id].ref_cnt) {
			free_region_id = region_id;
			break;
		}
	}

	if (MSYS_REGION_NUM == free_region_id) {
		pr_err("Could not find a free region!!!\n");
		spin_unlock(&addr_compl_lock);
		return -EPERM;
	}

	/* address completion request in a free region */
	region_id = free_region_id;
	region_reg = &addr_compl->region_regs[region_id];

	/* Configure address completion region for PP SMI register space */
	region = readl(addr_compl_reg_base_virtual + region_reg->offset);
	region &= (~(addr_compl->remap_mask) << region_reg->region_shift);
	tmp_addr >>= addr_compl->remap_offset;
	region |= (tmp_addr << region_reg->region_shift);
	writel(region, addr_compl_reg_base_virtual + region_reg->offset);

	*compl_region_id = region_id;
	*compl_addr = addr_compl_reg_base_physical +
			((region_id & addr_compl->region_mask) << addr_compl->region_offset);

	msys_regions[region_id].phys_addr = addr;
	msys_regions[region_id].compl_addr = *compl_addr;
	msys_regions[region_id].ref_cnt++;

	spin_unlock(&addr_compl_lock);
	return 0;
}

/*******************************************************************************
* mv_addr_compl_release_region
* DESCRIPTION: release a address completion region
*
* INPUT:  region_id - address completion region ID
* OUTPUT: None
* RETURN: 0:succes, otherwise fail
*******************************************************************************/
int mv_addr_compl_release_region(unsigned int region_id)
{
	struct msys_addr_compl	*addr_compl;

	if (!addr_compl_reg_base_virtual)
		return -EPERM;

	if (addr_compl_mode == ADDRCOMPL_LEGACY)
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_LEGACY];
	else
		addr_compl = &msys_addr_compl_tbl[ADDRCOMPL_NORMAL];

	if (region_id >= addr_compl->max_region)
		return -EPERM;

	spin_lock(&addr_compl_lock);
	if (!msys_regions[region_id].ref_cnt) {
		/* the region has been released by others*/
		spin_unlock(&addr_compl_lock);
		return 0;
	}

	if (msys_regions[region_id].ref_cnt)
		msys_regions[region_id].ref_cnt--;

	spin_unlock(&addr_compl_lock);

	return 0;
}

#ifdef CONFIG_OF
/*for DT kernels, addr_compl_reg_base_physical can be read out from fdt files
   while addr_compl_reg_base_virtual can be iomapped then, so address completion
   initialization can be done before drivers' initialization in core_initcall way*/
static int __init mv_addr_compl_probe(void)
{
	struct device_node *np;
	struct resource res;
	void __iomem *addr_virtual;

	np = of_find_matching_node(NULL, msys_addr_compl_of_match_table);
	if (!np)
		return -EPERM;

	if (of_address_to_resource(np, 0, &res))
		return -EPERM;

	addr_virtual = of_iomap(np, 0);

	if (IS_ERR(addr_virtual)) {
		pr_err("cannot map registers\n");
		addr_compl_reg_base_virtual = NULL;
		return -ENOMEM;
	}

	mv_addr_compl_reg_base_set(res.start, addr_virtual);
	mv_addr_compl_mode_set(ADDRCOMPL_NORMAL);

	return mv_addr_compl_init();
}

#ifdef CONFIG_MV_INCLUDE_PRESTERA_PCI
/* This function works in the prestera PCI mode that AMC card does address completion
  * through the PCI endpoint - msys.
  * Since address completion probe through PCI should wait after all PCI BARs are
  * configured properly and reassigned by prestera, so this probe function is in the
  * late_initcall way
  */
static int __init mv_addr_compl_pci_probe(void)
{
	/* address completion is implemented in PCI end point */
	if (!prestera_pci_addr_compl_reg_base_get(&addr_compl_reg_base_physical, &addr_compl_reg_base_virtual))
		return -EPERM;

	mv_addr_compl_mode_set(ADDRCOMPL_NORMAL);
	return mv_addr_compl_init();
}

late_initcall(mv_addr_compl_pci_probe);
#endif

core_initcall(mv_addr_compl_probe);
#else
/*for Non-DT kernels, cpss switch register base has been mapped by iotable_init()
   in system initialization, so address completion initialization can be done in machine
   initilization*/
#endif

