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
Marvell GPL License Option

If you received this File from Marvell, you may opt to use, redistribute and/or
modify this File in accordance with the terms and conditions of the General
Public License Version 2, June 1991 (the "GPL License"), a copy of which is
available along with the File in the license.txt file.

THE FILE IS DISTRIBUTED AS-IS, WITHOUT WARRANTY OF ANY KIND, AND THE IMPLIED
WARRANTIES OF MERCHANTABILITY OR FITNESS FOR A PARTICULAR PURPOSE ARE EXPRESSLY
DISCLAIMED.  The GPL License provides additional details about this warranty
disclaimer.
*******************************************************************************/
#ifndef __mv_addr_compl_h__
#define __mv_addr_compl_h__
#include <linux/types.h>

enum msys_addr_compl_mode {
	ADDRCOMPL_LEGACY,
	ADDRCOMPL_NORMAL,
	ADDRCOMPL_MAX,
};

#ifdef CONFIG_MSYS_ADDR_COMPL
int mv_addr_compl_request_region(phys_addr_t addr, unsigned int *compl_region_id, phys_addr_t *compl_addr);
int mv_addr_compl_release_region(unsigned int region_id);
#ifndef CONFIG_OF
void mv_addr_compl_reg_base_set(phys_addr_t reg_base_physical, void __iomem *reg_base_virtual);
void mv_addr_compl_mode_set(enum msys_addr_compl_mode mode);
int mv_addr_compl_init(void);
#endif
#else
static inline int mv_addr_compl_request_region(phys_addr_t addr,
							unsigned int *compl_region_id,
							phys_addr_t *compl_addr)
{
	return -1;
}

static inline int mv_addr_compl_release_region(unsigned int region_id)
{
	return -1;
}
#ifndef CONFIG_OF
static inline void mv_addr_compl_reg_base_set(phys_addr_t reg_base_physical, void __iomem *reg_base_virtual)
{
	return;
}
static inline void mv_addr_compl_mode_set(enum msys_addr_compl_mode mode)
{
	return;
}
static inline int mv_addr_compl_init(void)
{
	return -1;
}
#endif
#endif

#endif /* __mv_addr_compl_h__ */
