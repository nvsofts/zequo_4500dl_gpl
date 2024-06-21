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
* servicecpu.h
*
* DESCRIPTION:
*	Service CPU support for BobCat2.
*
* DEPENDENCIES:
*
*******************************************************************************/

#ifndef __SERVICECPU_H
#define __SERVICECPU_H

#ifndef TRUE
#define TRUE 1
#endif
#ifndef FALSE
#define FALSE 0
#endif

#define SERVICECPU_HOSTMEM_INFO_ITEMS 4

/* There are 6 parameters to congigure SRAM(cache size, free area size,hostmem size
	, oamdb area size, share memory size and firmware size),
	also 4 parameter to configure DRAM(shmem offset, shmem size,
	hostmem offset and hostmem size), it's too complicated if we set all parameters
	in one command. So we make the conguration of SRAM and DRAM seperately,
	then apply together, this flag will be passed first to indicate which part we
	are going to set, the apply operation only disable IPC and set memory sections'
	size/base in software, but not setup memory in hardware, the memory hardware
	set up work will be done in device open*/
/* For external mode with firmware in SRAM, only CONFIG_SRAM works, CONFIG_SRAM will
	disable IPC, set memory sections' size/base in software, the memory hardware
	set up work will be done in device open*/
/* For external mode with firmware in SRAM, CONFIG_SRAM/CONFIG_DRAM/CONFIG_APPLY works,
	CONFIG_SRAM only stores SRAM configuration and CONFIG_DRAM only stores DRAM configuration,
	CONFIG_APPLY will disable IPC, set memory sections' size/base in software, the memory hardware
	set up work will be done in device open*/
enum scpu_config_flag {
	CONFIG_SRAM = 0,/* set SRAM configuration items, valid when firmware is in either SRAM or DRAM */
	CONFIG_DRAM,	/* set DRAM configuration items, valid only when firmware is in DRAM */
	CONFIG_APPLY,	/* apply RAM configurations, valid only when firmware is in DRAM */
	CONFIG_MAX
};

struct servicecpu_sram_config {
	unsigned int seg_cache_size;
	unsigned int seg_free_size;
	unsigned int seg_hostmem_size;
	unsigned int seg_oamdb_size;
	unsigned int seg_shmem_size;
	unsigned int seg_code_size;
};

struct servicecpu_dram_config {
	unsigned int seg_shmem_in_dram_offset;
	unsigned int seg_shmem_in_dram_size;
	unsigned int seg_hostmem_in_dram_offset;
	unsigned int seg_hostmem_in_dram_size;
};

struct servicecpu_config {
	enum scpu_config_flag config_flag;
	union {
		struct servicecpu_sram_config sram_config;
		struct servicecpu_dram_config dram_config;
	};
};

struct servicecpu_hostmem_info {
	unsigned int items[SERVICECPU_HOSTMEM_INFO_ITEMS];
};

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
struct servicecpu_ipcchn_set {
	unsigned int	chn_id;
	unsigned int	chn_en;
	char		*dev_name;
};
#endif

/* now IPC_FREERTOS_CPU_ID is only valid for external mode, in external mode, the serviceCPU works in cpu#0*/
#define IPC_FREERTOS_CPU_ID		0
#define IPC_FREERTOS_CHANNEL_ID		1

#define SERVICECPU_MINOR		243
#define SERVICECPU_IOC_MAGIC		's'
#define SERVICECPU_IOC_UNRESET		_IOW(SERVICECPU_IOC_MAGIC, 1, unsigned int)
#define SERVICECPU_IOC_SENDIRQ		_IO(SERVICECPU_IOC_MAGIC, 2)
#define SERVICECPU_IOC_CONFIG		_IOW(SERVICECPU_IOC_MAGIC, 3, struct servicecpu_config)
#define SERVICECPU_IOC_MMAPINFO		_IOR(SERVICECPU_IOC_MAGIC, 4, struct servicecpu_hostmem_info)
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
#define SERVICECPU_IOC_SET_IPCCHN	_IOW(SERVICECPU_IOC_MAGIC, 5, struct servicecpu_ipcchn_set)
#endif

#define SERVICECPU_SRAM_WAY_SIZE	0x10000		/* 64kB */

#ifndef CONFIG_OF
#define SERVICECPU_FW_IN_SRAM_CACHE_SIZE_DEF	(2 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_SRAM_FREE_SIZE_DEF	(0 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_SRAM_HOSTMEM_SIZE_DEF	(2 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_SRAM_OAMDB_SIZE_DEF	(1 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_SRAM_SHMEM_SIZE_DEF	(2 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_SRAM_CODE_SIZE_DEF	(1 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_DRAM_CACHE_SIZE_DEF	(24 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_DRAM_FREE_SIZE_DEF	(0 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_DRAM_HOSTMEM_SIZE_DEF	(4 * SERVICECPU_SRAM_WAY_SIZE)
#define SERVICECPU_FW_IN_DRAM_OAMDB_SIZE_DEF	(4 * SERVICECPU_SRAM_WAY_SIZE)

#define SERVICECPU_FW_IN_DRAM_SHMEM_OFFSET_DEF	(0x180000)
#define SERVICECPU_FW_IN_DRAM_SHMEM_SIZE_DEF	(_256K)
#define SERVICECPU_FW_IN_DRAM_HOSTMEM_OFFSET_DEF	(0x2c0000)
#define SERVICECPU_FW_IN_DRAM_HOSTMEM_SIZE_DEF	(_1M)
#endif

/* Location of internal MSYS registers */
#define MSYS_BRIDGE_BASE		0x20000
#define MSYS_CPU_RESUME_ADDR_REG	(MSYS_BRIDGE_BASE + 0x984)
#define MSYS_CPU_RESUME_CTRL_REG	(MSYS_BRIDGE_BASE + 0x988)
#define MSYS_CPU_RESET_REG(cpu)		(MSYS_BRIDGE_BASE + (0x800+(cpu)*8))
#define MSYS_SW_RESET_ENABLE_MASK	0x1
#define MSYS_CIB_CONTROL_REG		0x20280
#define MSYS_L2_DATA_LOCK_CPU0_REG	0x8900
#define MSYS_L2_INSTR_LOCK_CPU0_REG	0x8904
#define MSYS_L2_IO_BRIDGE_LOCK_REG	0x8984
#define MSYS_L2_CONTROL_REG		0x8100
#define MSYS_L2_CONTROL_ENABLE_MASK	0x1
#define MSYS_L2_AUX_CTRL_REG		0x8104
#define MSYS_L2_BLOCK_ALLOCATION	0x878C
#define MSYS_SRAM_WINDOW_CTRL_REG(n)	(0x20240 + 4 * (n))
#define MSYS_SRAM_WINDOW_BASE_MASK	0xFFFF0000
#define MSYS_SRAM_WINDOW_SIZE_512KB	(7 << 8)
#define MSYS_SRAM_WINDOW_512KB_MASK	0xFFF80000
#define MSYS_SRAM_WINDOW_ENABLE		0x1
#define MSYS_MBUS_WINDOW_13_CTRL	0x200B8

#define IPC_PCI_DRBL_BASE		(12)
#define IPC_PCI_DRBL_MASK		(1 << IPC_PCI_DRBL_BASE)
#define IPC_PCI_RX_MAX_MSGS_PER_ISR	(50)
#define IPC_PCI_OUTDRBL_CPU0_CAUSE_REG	(0x21870)
#define IPC_PCI_OUTDRBL_CPU0_MASK_REG	(0x21874)
#define IPC_PCI_SW_TRIG_INTERRUPT_REG	(0x20A04)

/* external mode with ServiceCPU firmware run in SRAM
	The firmware of ServiceCPU will be put under the file system of HostCPU(running Linux),
	the servicecpu(/dev/servicecpu) driver can load the firmware into the SRAM memory
	and boot the first core of Internal CPU into that firmware. */
/* external mode with ServiceCPU firmware run in DRAM
	HostCPU firmware (Linux) and ServiceCPU firmware will boot separately from their own Uboot,
	we reserve areas for HostCPU and ServiceCPU in both SRAM and DRAM.  */
enum servicecpu_mode {
	EXTERNAL_FW_IN_SRAM  = 0,	/* external mode with ServiceCPU firmware run in SRAM */
	EXTERNAL_FW_IN_DRAM,		/* external mode with ServiceCPU firmware run in DRAM */
	INTERNAL_FW_IN_DRAM,		/* internal mode, with this mode ServiceCPU firmware runs in DRAM */

	SCPU_MODE_MAX
};

struct servicecpu_info {
	struct platform_device	*pdev;

	enum servicecpu_mode	scpu_mode;
	int pci_win_size;
	int seg_cache_size;
	int seg_free_size;
	int seg_hostmem_size;
	int seg_oamdb_size;

	void __iomem *seg_free_base;
	void __iomem *seg_hostmem_base;
	void __iomem *seg_oamdb_base;
	void __iomem *seg_hostmem_phy_base;

	int seg_shmem_size;
	int seg_code_size;
	void __iomem *seg_shmem_base;
	void __iomem *seg_code_base;

	/*belows are in DRAM*/
	int seg_shmem_in_dram_offset;
	int seg_shmem_in_dram_size;
	int seg_hostmem_in_dram_offset;
	int seg_hostmem_in_dram_size;
	void __iomem *seg_hostmem_in_dram_phy_base;
	void __iomem *pci_win_dram_phys_base;

	void __iomem *inter_regs_base;
	void __iomem *pci_win_virt_base;
	void __iomem *pci_win_phys_base;

	void __iomem *sram_virt_base;
	unsigned int sram_phys_base;
	unsigned int sram_size;
	void __iomem *dram_virt_base;
	unsigned int dram_phys_base;
	unsigned int dram_size;

	/* this flag is set in prestera pci driver, in LK3.10, servicecpu is
	enabled not only by dts file, but also need to check the PCI deviceID
	when run with AMC card. */
#ifdef CONFIG_OF
	int servicecpu_enabled;
#endif

	int servicecpu_initialized;
};

extern struct servicecpu_info servicecpu_data;
#define scpu_inter_regs (servicecpu_data.inter_regs_base)

#ifndef CONFIG_OF
int servicecpu_init(void);
void servicecpu_deinit(void);
#endif

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
/***********************************************************************************
 * servicecpu_enable_channel
 *
 * DESCRIPTION:
 *              enable serviceCPU IPC link's channel
 *
 * INPUT:
 *		chn_id - The IPC channel ID
 *		dev_name - The IPC channel device name
 * OUTPUT:
 *       None
 * RETURN:
 *		0 successful; others fail
 *
 ************************************************************************************/
int servicecpu_enable_channel(unsigned int chn_id, char *dev_name);

/***********************************************************************************
 * servicecpu_disable_channel
 *
 * DESCRIPTION:
 *              disable serviceCPU IPC link's channel
 *
 * INPUT:
 *		chn_id - The channel ID
 * OUTPUT:
 *       None
 * RETURN:
 *		0 successful; others fail
 *
 ************************************************************************************/
int servicecpu_disable_channel(unsigned int chn_id);

/***********************************************************************************
 * servicecpu_msys_reserve_memory
 *
 * DESCRIPTION:
 *              reserve dram memory for internal Service CPU
 *
 * INPUT:
 *		None
 * OUTPUT:
 *		None
 * RETURN:
 *		None
 *
 ************************************************************************************/
void servicecpu_msys_reserve_memory(void);
#endif

#endif /* __SERVICECPU_H */
