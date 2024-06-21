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
* servicecpu.c
*
* DESCRIPTION:
*	serviceCPU support for msys CPU.
*
* DEPENDENCIES:
*
*******************************************************************************/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/memblock.h>
#include <linux/firmware.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/of_device.h>
#include <linux/of_fdt.h>
#include <linux/of_address.h>
#include <linux/of.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/mm.h>

#ifdef CONFIG_OF
#include "mach/mvCommon.h"
#else
#include "common/mvCommon.h"
#endif
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
#include "mv_ipc.h"
#include "mv_ipc_wrapper.h"
#endif
#include "servicecpu.h"

#undef MV_SERVICECPU_DBG
#ifdef MV_SERVICECPU_DBG
#define dprintk(a...) printk(a)
#else
#define dprintk(a...)
#endif

typedef int (*scpu_callback)(struct servicecpu_info *info);

/* FDT memory configurations or IOCTL memory configurations will be stored in the below structure */
/* Later servicecpu driver will out read memory configurations from it and recalculate out the */
/* memory physical& virtual base addresses */
struct servicecpu_memory_init {
	/*sram base offset in service cpu sram pci window, only valid in external mode */
	unsigned int sram_base_offset;
	unsigned int sram_size;
	/*dram base offset in service cpu dram pci window, , only valid in external mode */
	unsigned int dram_base_offset;
	unsigned int dram_size;
	/* below bases are sram/dram sections' relative address offsets in the whole sram/dram area */
	/* sram cache section always starts from the sram base, its base offset is always 0 */
	/* so here we only need seg_cache_size */
	/* all other sram sections excpet cache section need to do sram hardware configuration */
	unsigned int seg_cache_size;
	unsigned int seg_free_base;
	unsigned int seg_free_size;
	unsigned int seg_hostmem_base;
	unsigned int seg_hostmem_size;
	unsigned int seg_oamdb_base;
	unsigned int seg_oamdb_size;
	unsigned int seg_shmem_base;
	unsigned int seg_shmem_size;
	unsigned int seg_code_base;
	unsigned int seg_code_size;
	unsigned int seg_shmem_in_dram_offset;
	unsigned int seg_shmem_in_dram_size;
	unsigned int seg_hostmem_in_dram_offset;
	unsigned int seg_hostmem_in_dram_size;
	bool sram_configured;
	bool dram_configured;
};

struct scpu_mode_setup_cb {
	scpu_callback init_memmap;
	scpu_callback config_memmap;
	scpu_callback setup_mem;
	scpu_callback setup_ipc;
	scpu_callback disable_cpu;
	scpu_callback kickoff_cpu;
	scpu_callback load_fw;
};

static int servicecpu_init_memmap_ext_sram_mode(struct servicecpu_info *info);
static int servicecpu_init_memmap_ext_dram_mode(struct servicecpu_info *info);
static int servicecpu_init_memmap_inter_dram_mode(struct servicecpu_info *info);
static int servicecpu_config_memmap_ext_sram_mode(struct servicecpu_info *info);
static int servicecpu_config_memmap_ext_dram_mode(struct servicecpu_info *info);
static int servicecpu_setup_mem_ext_sram_mode(struct servicecpu_info *info);
static int servicecpu_setup_mem_ext_dram_mode(struct servicecpu_info *info);
static int servicecpu_setup_mem_inter_dram_mode(struct servicecpu_info *info);
static int servicecpu_setup_ipc_ext_mode(struct servicecpu_info *info);
static int servicecpu_setup_ipc_inter_mode(struct servicecpu_info *info);
static int servicecpu_disable_ext_sram_mode(struct servicecpu_info *info);
static int servicecpu_kickoff_ext_sram_mode(struct servicecpu_info *info);
static int servicecpu_load_fw_inter_dram_mode(struct servicecpu_info *info);
static int servicecpu_kickoff_inter_mode(struct servicecpu_info *info);
static int servicecpu_ext_unreset(bool enable);

struct servicecpu_info servicecpu_data;
struct servicecpu_info *info;

static const char drv_name_internal[] = "serviceCPU";
static struct servicecpu_memory_init servicecpu_mem_init;
static bool servicecpu_reserve_mem_ok;

static const char *scpu_mode[SCPU_MODE_MAX] = {
	"external mode with ServiceCPU firmware run in SRAM",
	"external mode with ServiceCPU firmware run in DRAM",
};

static const char *scpu_config[CONFIG_MAX] = {
	"SRAM configuration",
	"DRAM configuration",
	"RAM configuration APPLY",
};

static struct scpu_mode_setup_cb scpu_mode_setup_cb[SCPU_MODE_MAX] = {
	/* external mode with ServiceCPU firmware run in SRAM */
	{
		.init_memmap   = servicecpu_init_memmap_ext_sram_mode,
		.config_memmap = servicecpu_config_memmap_ext_sram_mode,
		.setup_mem     = servicecpu_setup_mem_ext_sram_mode,
		.setup_ipc     = servicecpu_setup_ipc_ext_mode,
		.disable_cpu   = servicecpu_disable_ext_sram_mode,
		.kickoff_cpu   = servicecpu_kickoff_ext_sram_mode,
		/* in this mode, servicecpu image is written to sram directly by device write operation */
		/* so here we do not need a special load firmware callback function */
		.load_fw       = NULL,
	},
	/* external mode with ServiceCPU firmware run in DRAM */
	{
		.init_memmap   = servicecpu_init_memmap_ext_dram_mode,
		.config_memmap = servicecpu_config_memmap_ext_dram_mode,
		.setup_mem     = servicecpu_setup_mem_ext_dram_mode,
		.setup_ipc     = servicecpu_setup_ipc_ext_mode,
		/* in this mode, the servicecpu firmware was loaded directly from UBOOT */
		/* so here we do not need special disable_cpu/kickoff_cpu/load_fw callback functions */
		.disable_cpu   = NULL,
		.kickoff_cpu   = NULL,
		.load_fw       = NULL,
	},
	/* internal mode with ServiceCPU firmware run in DRAM */
	{
		.init_memmap   = servicecpu_init_memmap_inter_dram_mode,
		/* internal mode does not support ioctl config memory map function*/
		.config_memmap = NULL,
		.setup_mem     = servicecpu_setup_mem_inter_dram_mode,
		.setup_ipc     = servicecpu_setup_ipc_inter_mode,
		/* internal mode does not use disable_cpu callback function*/
		.disable_cpu   = NULL,
		.kickoff_cpu   = servicecpu_kickoff_inter_mode,
		.load_fw       = servicecpu_load_fw_inter_dram_mode,
	}
};

static int servicecpu_init_memmap(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].init_memmap)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].init_memmap(info);
}

static int servicecpu_config_memmap(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].config_memmap)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].config_memmap(info);
}

static int servicecpu_setup_mem(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].setup_mem)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].setup_mem(info);
}

static int servicecpu_setup_ipc(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].setup_ipc)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].setup_ipc(info);
}

static int servicecpu_disable_cpu(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].disable_cpu)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].disable_cpu(info);
}

static int servicecpu_kickoff_cpu(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].kickoff_cpu)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].kickoff_cpu(info);
}

static int servicecpu_load_fw(struct servicecpu_info *info)
{
	if (!scpu_mode_setup_cb[info->scpu_mode].load_fw)
		return -1;
	return scpu_mode_setup_cb[info->scpu_mode].load_fw(info);
}

struct scpu_callflow {
	char         *stage;
	scpu_callback func;
};

static struct scpu_callflow scpu_callflow_ext_sram_mode_probe[] = {
	{"init_memmap",	servicecpu_init_memmap},
	{"disable_cpu",	servicecpu_disable_cpu},
	{NULL, NULL},
};

static struct scpu_callflow scpu_callflow_ext_dram_mode_probe[] = {
	{"init_memmap",	servicecpu_init_memmap},
	{NULL, NULL},
};

/* In internal mode, ServiceCPU firmware will be compiled into the Linux kernel of HostCPU*/
/* and load to the predefined firmware dram address during the boot stage of Linux. */
static struct scpu_callflow scpu_callflow_inter_dram_mode_probe[] = {
	{"init_memmap",		servicecpu_init_memmap},
	{"setup_mem",		servicecpu_setup_mem},
	{"load_firmware",	servicecpu_load_fw},
	{"kickoff_cpu",		servicecpu_kickoff_cpu},
	{"setup_ipc",		servicecpu_setup_ipc},
	{NULL, NULL},
};

static struct scpu_callflow *scpu_callflow_tbl_probe[SCPU_MODE_MAX] = {
	scpu_callflow_ext_sram_mode_probe,
	scpu_callflow_ext_dram_mode_probe,
	scpu_callflow_inter_dram_mode_probe
};

static struct scpu_callflow scpu_callflow_ext_mode_open[] = {
	{"setup_mem",	servicecpu_setup_mem},
	{"setup_ipc",	servicecpu_setup_ipc},
	{NULL, NULL},
};

static struct scpu_callflow scpu_callflow_inter_mode_open[] = {
	{NULL, NULL},
};

static struct scpu_callflow *scpu_callflow_tbl_open[SCPU_MODE_MAX] = {
	scpu_callflow_ext_mode_open,
	scpu_callflow_ext_mode_open,
	scpu_callflow_inter_mode_open
};

#define do_funcs(_name, _info)					\
{								\
	int func_i, func_ret = 0;				\
	struct scpu_callflow *cf = NULL;			\
	if (_info->scpu_mode >= SCPU_MODE_MAX) {		\
		pr_err("do_funcs: scpu_mode %d is larger than %d!\n", _info->scpu_mode, SCPU_MODE_MAX);	\
		return -1;					\
	}							\
	cf = scpu_callflow_tbl_##_name[_info->scpu_mode];	\
	if (!cf) {						\
		pr_err("do_funcs: call flows for mode %d is not found!\n", _info->scpu_mode);		\
		return -1;					\
	}							\
	for (func_i = 0; cf[func_i].func; func_i++) {		\
		dprintk("do %s()\n", cf[func_i].stage);		\
		func_ret = cf[func_i].func(_info);		\
		if (func_ret) {					\
			pr_err("do_funcs: mode %d stage %s ret %d!\n", _info->scpu_mode, cf[func_i].stage, func_ret);\
			return func_ret;					\
		}						\
	}							\
}

static irqreturn_t servicecpu_interrupt(int irq, void *dev_id)
{
	unsigned int reg;

	reg = readl(scpu_inter_regs + IPC_PCI_OUTDRBL_CPU0_CAUSE_REG);
	if ((reg & IPC_PCI_DRBL_MASK) == 0)
		return IRQ_NONE;
	writel(reg & (~IPC_PCI_DRBL_MASK), scpu_inter_regs + IPC_PCI_OUTDRBL_CPU0_CAUSE_REG);

	dprintk("IPC PCIDrbl: RX callback. got irq no = %d\n", irq);

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	/* Process IPC msg */
	ipc_wrapper_link_rx_process(IPC_SCPU_FREERTOS_LINK_ID);
#endif

	return IRQ_HANDLED;
}


#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
/* Platform and application specific function, which will trigger host2serviceCPU
 * interrupt, used to signal the other side of IPC tunnel (serviceCPU), that the msg
 * is prepared and ready for process. It may be empty, if the other IPC side
 * operates in polling mode.
 */
static void servicecpu_ipc_trigger_irq(unsigned int link_id, unsigned int chn_id, void *dev_id)
{
	/* Currently slave is configured to pooling mode - do nothing */

	/* TODO: Send interrupt to firmware CPU */
}

/* Platform specific, user call-back used for masking/unmasking doorbell */
static void servicecpu_ipc_enable_doorbell(unsigned int link_id, unsigned int chn_id, bool enable, void *dev_id)
{
	/* Because now all channels share one doorbell - IPC_PCI_DRBL_BASE,
	 * do not mask/unmask it per-channel
	 */
}

/* Platform specific, user call-back used for interrupt registering */
static int servicecpu_ipc_request_irq(unsigned int link_id, void *dev_id)
{
	struct servicecpu_info *info = dev_id;
	struct platform_device *pdev = info->pdev;
	const char *drv_name;
	int irq, ret;

	drv_name = drv_name_internal;

	irq = platform_get_irq(pdev, 0);

	if (irq >= 0) {
		ret = request_irq(irq, servicecpu_interrupt, IRQF_SHARED, drv_name, (void *)info);
		if (ret) {
			dev_err(&pdev->dev, "error during irq request %d\n", ret);
			return ret;
		}
	}

	return 0;
}


static int servicecpu_set_ipc_linkinfo(struct ipc_link_info *link_info)
{
	struct servicecpu_info *info = (struct servicecpu_info *)link_info->dev_id;
	unsigned int shmem_size;
	void *shmem_virt_addr;

	shmem_virt_addr = info->seg_shmem_base;
	shmem_size = info->seg_shmem_size;

	link_info->shmem_size          = shmem_size;
	link_info->shmem_virt_addr     = shmem_virt_addr;
	link_info->send_trigger        = servicecpu_ipc_trigger_irq;
	link_info->enable_chn_doorbell = servicecpu_ipc_enable_doorbell;
	link_info->irq_init            = servicecpu_ipc_request_irq;

	return 0;
}

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
int servicecpu_enable_channel(unsigned int chn_id, char *dev_name)
{
	sprintf(dev_name, "/dev/ipc-lnk%d/chn%d", IPC_SCPU_FREERTOS_LINK_ID, chn_id);

	return ipc_wrapper_channel_register(IPC_SCPU_FREERTOS_LINK_ID, chn_id);
}

/***********************************************************************************
 * servicecpu_disable_channel
 *
 * DESCRIPTION:
 *              disable serviceCPU IPC link's channel
 *
 * INPUT:
 *		chn_id - The IPC channel ID
 * OUTPUT:
 *       None
 * RETURN:
 *		0 successful; others fail
 *
 ************************************************************************************/
int servicecpu_disable_channel(unsigned int chn_id)
{
	return ipc_wrapper_channel_unregister(IPC_SCPU_FREERTOS_LINK_ID, chn_id);
}
#endif

#ifndef CONFIG_MV_SERVICECPU_IPC_SUPPORT
/***********************************************************************************
 * servicecpu_ipc_send_pci_drbl
 *
 * DESCRIPTION:
 *              Send an IPC pcidrbl to target CPU
 *
 * INPUT:
 *		cpu_id - the remote cpu id
 *		chn_id - The channel ID
 * OUTPUT:
 *       None
 * RETURN:
 *		MV_OK or MV_ERROR
 *
 ************************************************************************************/
static void servicecpu_ipc_send_pci_drbl(MV_U32 cpu_id, MV_U32 chn_id)
{
	MV_U32 pci_drbl_Num;
	MV_U32 cpu_bit_mask;

	cpu_bit_mask = ((1 << cpu_id) << 8);
	/* now all IPC channels only share one doorbell - IPC_PCI_DRBL_BASE */
	pci_drbl_Num = IPC_PCI_DRBL_BASE;

	/* Trigger private doorbell on remote CPU */
	writel(cpu_bit_mask | pci_drbl_Num, scpu_inter_regs + IPC_PCI_SW_TRIG_INTERRUPT_REG);

	return;
}
#endif

static int servicecpu_disable_ext_sram_mode(struct servicecpu_info *info)
{
	return servicecpu_ext_unreset(false);
}

static int servicecpu_kickoff_ext_sram_mode(struct servicecpu_info *info)
{
	return servicecpu_ext_unreset(true);
}

static int servicecpu_kickoff_inter_mode(struct servicecpu_info *info)
{
	unsigned long reg;
	unsigned int cpu;

	/* Boot Secondary CPU */
	cpu = 1;

	/* Set resume control and address */
	writel(0x0, scpu_inter_regs + MSYS_CPU_RESUME_CTRL_REG);
	writel(info->dram_phys_base, scpu_inter_regs + MSYS_CPU_RESUME_ADDR_REG);

	dsb();

	/* Kick secondary CPUs */
	reg = readl(scpu_inter_regs + MSYS_CPU_RESET_REG(cpu));
	reg = reg & ~MSYS_SW_RESET_ENABLE_MASK;
	writel(reg, scpu_inter_regs + MSYS_CPU_RESET_REG(cpu));

	mb();

	info->servicecpu_initialized = 1;

	return 0;
}

static int servicecpu_load_fw_inter_dram_mode(struct servicecpu_info *info)
{
	const struct firmware *fw = NULL;

	/* Load Firmware */
	if (request_firmware(&fw, "service_cpu.bin", NULL)) {
		pr_err("servicecpu: Failed requesting firmware!\n");
		return -ENOENT;
	}

	if (fw->size > info->seg_code_size) {
		pr_err("Firmware image size 0x%x is too large! There are only 0x%x bytes.\n",
			fw->size, info->seg_code_size);
		return -ENOMEM;
	}

	memcpy_toio(info->seg_code_base, fw->data, fw->size);
	release_firmware(fw);

	return 0;
}

/***********************************************************************************
 * servicecpu_setup_ipc_ext_mode
 *
 * DESCRIPTION:
 *			set up IPC for external mode
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_setup_ipc_ext_mode(struct servicecpu_info *info)
{
	int ret;
	unsigned int reg;
#ifndef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	int irq;
#endif

#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	/* Initialize IPC and start links */
	/* Enable the pcidrbl, now all channels share one doorbell - IPC_PCI_DRBL_BASE */
	reg = readl(scpu_inter_regs + IPC_PCI_OUTDRBL_CPU0_MASK_REG);
	reg |= (1 << IPC_PCI_DRBL_BASE);
	writel(reg, scpu_inter_regs + IPC_PCI_OUTDRBL_CPU0_MASK_REG);

	/* Initialize IPC and start links */
	ret = ipc_wrapper_link_bind(IPC_SCPU_FREERTOS_LINK_ID, info, servicecpu_set_ipc_linkinfo);
	if (ret) {
		dev_err(&info->pdev->dev, "error during bind ipc link %d\n", ret);
		return ret;
	}
#else
	irq = platform_get_irq(info->pdev, 0);

	ret = request_irq(irq, servicecpu_interrupt, IRQF_DISABLED, drv_name_internal,
								(void *)info);
	if (ret) {
		dev_err(&info->pdev->dev, "error during irq request %d\n", ret);
		return ret;
	}
#endif
	return ret;
}

/***********************************************************************************
 * servicecpu_setup_ipc_inter_mode
 *
 * DESCRIPTION:
 *			set up IPC for internal mode
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_setup_ipc_inter_mode(struct servicecpu_info *info)
{
	int ret = 0;
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	/* in internal service cpu mode, both host cpu(0) and slave cpu(1) work in polling mode*/
	/* so internal service cpu mode does not need interrupt */
	/* Initialize IPC and start links */
	ret = ipc_wrapper_link_bind(IPC_SCPU_FREERTOS_LINK_ID, info, servicecpu_set_ipc_linkinfo);
	if (ret)
		return ret;

	ipc_wrapper_enable_link_poll(IPC_SCPU_FREERTOS_LINK_ID);
#endif
	return ret;
}

/***********************************************************************************
 * servicecpu_config_sram
 *
 * DESCRIPTION:
 *			Configure the L2-SRAM according to the memory settings
 *
 * INPUT:
 *			base -sram physical base offset
 *			size -sram size
 * OUTPUT:
 *			None
 * RETURN:
 *			None
 *
 **********************************************************************************/
static void servicecpu_config_sram(unsigned int base, unsigned int size)
{
	unsigned int reg, way, mask, i;

	/* Enable access to L2 configuration registers */
	reg = readl(scpu_inter_regs + MSYS_CIB_CONTROL_REG);
	writel(reg & ~BIT12, scpu_inter_regs + MSYS_CIB_CONTROL_REG);

	/* Lock L2 ways for Code, Shared Memory and OAM DB segments */
	way = info->seg_cache_size / SERVICECPU_SRAM_WAY_SIZE;

	mask = ((1 << (size / SERVICECPU_SRAM_WAY_SIZE)) - 1) << way;

	writel(mask, scpu_inter_regs + MSYS_L2_DATA_LOCK_CPU0_REG);
	writel(mask, scpu_inter_regs + MSYS_L2_INSTR_LOCK_CPU0_REG);
	writel(mask, scpu_inter_regs + MSYS_L2_IO_BRIDGE_LOCK_REG);

	/* Allocate SRAM ways and open decode windows */
	for (reg = base; reg != base + size; reg += SERVICECPU_SRAM_WAY_SIZE, way++)
		writel(reg | way, scpu_inter_regs + MSYS_L2_BLOCK_ALLOCATION);
	i = 0;
	while (size > 0) {
		reg = base & MSYS_SRAM_WINDOW_512KB_MASK;
		/* Must check if window located at the end of the space, like
		 * base=0xFFE00000, size=0x200000 */
		if ((reg + SZ_512K == 0x0) || ((reg + SZ_512K >= base + size) && (base + size != 0x0)))
			size = 0;
		else {
			size -= reg + SZ_512K - base;
			base = reg + SZ_512K;
		}
		reg |= MSYS_SRAM_WINDOW_SIZE_512KB | MSYS_SRAM_WINDOW_ENABLE;
		writel(reg, scpu_inter_regs + MSYS_SRAM_WINDOW_CTRL_REG(i));
		i++;
	}

	pr_err("Initializing MSYS SRAM finished\n");

	return;
}

/***********************************************************************************
 * servicecpu_config_ext_sram
 *
 * DESCRIPTION:
 *			Configure the L2-SRAM according to the memory settings for external mode
 *
 * INPUT:
 *			base -sram physical base offset
 *			size -sram size
 * OUTPUT:
 *			None
 * RETURN:
 *			None
 *
 **********************************************************************************/
static int servicecpu_config_ext_sram(unsigned int base, unsigned int size)
{
	if (size > info->sram_size) {
		pr_err("The total size 0x%x is larger than sram size 0x%x\n", size, info->sram_size);
		return -ENOMEM;
	}

	/*
	 * Configure ServiceCPU's SRAM
	 */
	servicecpu_config_sram(base, size);

	/* Disable window 13 used by BootROM */
	writel(0x0, scpu_inter_regs + MSYS_MBUS_WINDOW_13_CTRL);

	dev_info(&info->pdev->dev, "Initializing MSYS SRAM finished\n");

	return 0;
}

/***********************************************************************************
 * servicecpu_setup_mem_ext_sram_mode
 *
 * DESCRIPTION:
 *			Configure the L2-SRAM according to the memory settings
 *                                      for external mode with firmware in sram
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_setup_mem_ext_sram_mode(struct servicecpu_info *info)
{
	unsigned int size, base;

	size = info->seg_free_size + info->seg_hostmem_size +
			info->seg_code_size + info->seg_shmem_size + info->seg_oamdb_size;
	base = 0x0 - info->pci_win_size + info->seg_cache_size;

	/*
	 * Configure ServiceCPU's SRAM
	 */
	return servicecpu_config_ext_sram(base, size);

}

 /***********************************************************************************
 * servicecpu_setup_mem_ext_dram_mode
 *
 * DESCRIPTION:
 *			Change the configuration of the memory areas
 *                                      for external mode with firmware in dram
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_setup_mem_ext_dram_mode(struct servicecpu_info *info)
{
	unsigned int size, base;

	/* run in DRAM, there is no code and shmem areas in SRAM */
	size = info->seg_free_size + info->seg_hostmem_size + info->seg_oamdb_size;
	base = 0x0 - info->pci_win_size + info->seg_cache_size;

	/*
	 * Configure ServiceCPU's SRAM
	 */
	return servicecpu_config_ext_sram(base, size);
}

/***********************************************************************************
 * servicecpu_setup_mem_inter_dram_mode
 *
 * DESCRIPTION:
 *			Change the configuration of the memory areas
 *			for internal mode with firmware in dram
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_setup_mem_inter_dram_mode(struct servicecpu_info *info)
{
	unsigned int size;

	size = info->seg_free_size + info->seg_hostmem_size + info->seg_oamdb_size;

	/* if all sram are used for L2 cache, no need do sram setup*/
	if (!size)
		return 0;

	if (size > info->sram_size) {
		pr_err("The total size 0x%x is larger than sram size 0x%x\n", size, info->sram_size);
		return -ENOMEM;
	}

	/*
	 * Configure ServiceCPU's SRAM
	 */
	servicecpu_config_sram(info->sram_phys_base + info->seg_cache_size, size);

	return 0;
}

/***********************************************************************************
 * servicecpu_config_memmap_ext_sram_mode
 *
 * DESCRIPTION:
 *			Configure the L2-SRAM according to the memory settings
 *                                      for external mode with firmware in sram
 *
 * INPUT:
 *			 info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_config_memmap_ext_sram_mode(struct servicecpu_info *info)
{
	int ret = 0;
	int reg;
	unsigned int size;

	/* servicecpu of external mode is cpu#0*/
	reg = readl((void *)(scpu_inter_regs + MSYS_CPU_RESET_REG(0)));

	/* prohibit configuration of memory areas after serviceCPU starts up*/
	if (!(reg & MSYS_SW_RESET_ENABLE_MASK)) {
		pr_err("ServiceCPU has already started up, configuration of memory areas is prohibited!\n");
		return -EPERM;
	}

	if (info->servicecpu_initialized) {
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		/* serviceCPU setup includes the IPC link bind work, */
		/* so we need to unbind IPC link to reconfig IPC share memory later*/
		ret = ipc_wrapper_link_unbind(IPC_SCPU_FREERTOS_LINK_ID);
		if (ret)
			return ret;
#endif
	}

	/* Need reconfiguration in next open */
	info->servicecpu_initialized = 0;

	info->seg_cache_size = servicecpu_mem_init.seg_cache_size;
	info->seg_free_size = servicecpu_mem_init.seg_free_size;
	info->seg_hostmem_size = servicecpu_mem_init.seg_hostmem_size;
	info->seg_oamdb_size = servicecpu_mem_init.seg_oamdb_size;
	info->seg_shmem_size = servicecpu_mem_init.seg_shmem_size;
	info->seg_code_size = servicecpu_mem_init.seg_code_size;

	size = info->seg_free_size + info->seg_hostmem_size +
			info->seg_code_size + info->seg_shmem_size + info->seg_oamdb_size;
	if (size > info->sram_size) {
		pr_err("The total size 0x%x is larger than sram size 0x%x\n", size, info->sram_size);
		return -ENOMEM;
	}

	info->seg_free_base = (void *)((unsigned int)info->sram_virt_base +
				info->sram_size - size);
	info->seg_hostmem_base = (void *)((unsigned int)info->seg_free_base +
				info->seg_free_size);
	info->seg_oamdb_base = (void *)((unsigned int)info->seg_hostmem_base +
				info->seg_hostmem_size);

	/* init the base address of each area, we have shmem and code area in SRAM mode*/
	info->seg_shmem_base = (void *)((unsigned int)info->seg_oamdb_base +
				info->seg_oamdb_size);
	info->seg_code_base = (void *)((unsigned int)info->seg_shmem_base +
				info->seg_shmem_size);

	info->seg_hostmem_phy_base = (void *)((unsigned int)info->sram_phys_base +
				info->seg_cache_size + info->seg_free_size);

	return ret;
}

/***********************************************************************************
 * servicecpu_config_memmap_ext_dram_mode
 *
 * DESCRIPTION:
 *			Change the configuration of the memory areas for external mode with firmware in dram
 *
 * INPUT:
 *			info - servicecpu info
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_config_memmap_ext_dram_mode(struct servicecpu_info *info)
{
	int ret = 0;
	int reg;
	unsigned int size;

	/* if there is no sram or dram configuration, then do nothing*/
	if (!servicecpu_mem_init.sram_configured && !servicecpu_mem_init.dram_configured)
		return 0;

	/* servicecpu of external mode is cpu#0*/
	reg = readl((void *)(scpu_inter_regs + MSYS_CPU_RESET_REG(0)));

	/* prohibit configuration of memory areas after serviceCPU starts up*/
	if (!(reg & MSYS_SW_RESET_ENABLE_MASK)) {
		pr_err("ServiceCPU has already started up, configuration of memory areas is prohibited!\n");
		return -EPERM;
	}

	if (info->servicecpu_initialized) {
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		/* serviceCPU setup includes the IPC link bind work, */
		/* so we need to unbind IPC link to reconfig IPC share memory later*/
		ret = ipc_wrapper_link_unbind(IPC_SCPU_FREERTOS_LINK_ID);
		if (ret)
			return ret;
#endif
	}

	/* Need reconfiguration in next open */
	info->servicecpu_initialized = 0;

	if (servicecpu_mem_init.sram_configured) {
		info->seg_cache_size = servicecpu_mem_init.seg_cache_size;
		info->seg_free_size = servicecpu_mem_init.seg_free_size;
		info->seg_hostmem_size = servicecpu_mem_init.seg_hostmem_size;
		info->seg_oamdb_size = servicecpu_mem_init.seg_oamdb_size;
		servicecpu_mem_init.sram_configured = false;
	}

	if (servicecpu_mem_init.dram_configured) {
		info->seg_shmem_in_dram_offset = servicecpu_mem_init.seg_shmem_in_dram_offset;
		info->seg_shmem_in_dram_size = servicecpu_mem_init.seg_shmem_in_dram_size;
		info->seg_hostmem_in_dram_offset = servicecpu_mem_init.seg_hostmem_in_dram_offset;
		info->seg_hostmem_in_dram_size = servicecpu_mem_init.seg_hostmem_in_dram_size;
		servicecpu_mem_init.dram_configured = false;
	}

	size = info->seg_free_size + info->seg_hostmem_size + info->seg_oamdb_size;
	if (size > info->sram_size) {
		pr_err("The total size 0x%x is larger than sram size 0x%x\n", size, info->sram_size);
		return -ENOMEM;
	}

	info->seg_free_base = (void *)((unsigned int)info->sram_virt_base +
				info->sram_size - size);
	info->seg_hostmem_base = (void *)((unsigned int)info->seg_free_base +
				info->seg_free_size);
	info->seg_oamdb_base = (void *)((unsigned int)info->seg_hostmem_base +
				info->seg_hostmem_size);

	/* When run in DRAM mode, shmem and code areas are defined via offset based on
	the dram_phy_base, only need to init the dram_phy_base here */
	size = info->seg_shmem_in_dram_size + info->seg_hostmem_in_dram_size;
	if (size > info->dram_size) {
		pr_err("The total size 0x%x is larger than dram size 0x%x\n", size, info->dram_size);
		return -ENOMEM;
	}
	info->seg_hostmem_in_dram_phy_base = (void *)((unsigned int)info->dram_phys_base +
			 info->seg_hostmem_in_dram_offset);

	info->seg_hostmem_phy_base = (void *)((unsigned int)info->sram_phys_base +
				info->seg_cache_size + info->seg_free_size);
	/* needs to set dram virtual address in the future */
	info->seg_shmem_base = info->dram_virt_base + info->seg_shmem_in_dram_offset;
	info->seg_shmem_size = info->seg_shmem_in_dram_size;

	return ret;
}

static ssize_t servicecpu_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	size_t mem_size;
	char *src_ptr;
	int i;

	dprintk("%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	mem_size = info->seg_code_size;
	src_ptr = info->seg_code_base + *f_pos;

	if (*f_pos >= mem_size || !count)
		return 0;
	if (count > mem_size - *f_pos)
		count = mem_size - *f_pos;

	for (i = 0; i < count; i++) {
		if (copy_to_user(buf + i, src_ptr + i, 1)) {
			pr_err("%s: copy_to_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_SERVICECPU_DBG
		if (i < 50)
			dprintk("0x%x ", *(src_ptr + i));
#endif
	}

	dprintk("\n%s: read 0x%x, from SRAM f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, *f_pos, src_ptr);

	return i;
}

static ssize_t servicecpu_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	size_t mem_size;
	char *dst_ptr;
	int i;

	dprintk("%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	mem_size = info->seg_code_size;
	dst_ptr = info->seg_code_base + *f_pos;

	if (*f_pos + count > mem_size) {
		pr_err("Not enough space in SRAM for writing %d bytes.",
			count);
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		if (copy_from_user(dst_ptr + i, buf + i, 1)) {
			pr_err("%s: copy_from_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_SERVICECPU_DBG
		if (i < 50)
			dprintk("copy from %p to %p: 0x%x 0x%x\n",
				buf + i, dst_ptr + i, readb(buf + i),
				readb(dst_ptr + i));
#endif
	}

	dprintk("\n%s: writen 0x%x, to SRAM f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, *f_pos, dst_ptr);

	return i;
}


/***********************************************************************************
 * servicecpu_ext_unreset
 *
 * DESCRIPTION:
 *			kick off or disable servicecpu in external mode
 *
 * INPUT:
 *			enable - true: kick off servicecpu in external mode
 *                                                  false: disable servicecpu in external mode
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_ext_unreset(bool enable)
{
	int reg;

	/* servicecpu of external mode is cpu#0*/
	reg = readl((void *)(scpu_inter_regs + MSYS_CPU_RESET_REG(0)));

	if (enable == TRUE)
		reg &= ~MSYS_SW_RESET_ENABLE_MASK;
	else if (enable == FALSE) {
		reg |= MSYS_SW_RESET_ENABLE_MASK;
		info->servicecpu_initialized = 0;
	} else {
		pr_err("ERROR unknown value\n");
		return -EINVAL;
	}

	writel(reg, (void *)(scpu_inter_regs + MSYS_CPU_RESET_REG(0)));

	dprintk("%s: Service CPU SW Reset Control register: 0x%x\n",
		__func__, reg);

	return 0;
}

static inline void servicecpu_mem_init_sram_config(struct servicecpu_config *config)
{
	servicecpu_mem_init.seg_cache_size = config->sram_config.seg_cache_size;
	servicecpu_mem_init.seg_free_size = config->sram_config.seg_free_size;
	servicecpu_mem_init.seg_hostmem_size = config->sram_config.seg_hostmem_size;
	servicecpu_mem_init.seg_oamdb_size = config->sram_config.seg_oamdb_size;
	servicecpu_mem_init.seg_shmem_size = config->sram_config.seg_shmem_size;
	servicecpu_mem_init.seg_code_size = config->sram_config.seg_code_size;
}

static inline void servicecpu_mem_init_dram_config(struct servicecpu_config *config)
{
	servicecpu_mem_init.seg_shmem_in_dram_offset = config->dram_config.seg_shmem_in_dram_offset;
	servicecpu_mem_init.seg_shmem_in_dram_size = config->dram_config.seg_shmem_in_dram_size;
	servicecpu_mem_init.seg_hostmem_in_dram_offset = config->dram_config.seg_hostmem_in_dram_offset;
	servicecpu_mem_init.seg_hostmem_in_dram_size = config->dram_config.seg_hostmem_in_dram_size;
}

static long servicecpu_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	int ret;
	unsigned int reset;
	struct servicecpu_config config;
	struct servicecpu_hostmem_info hostmem_info;
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	struct servicecpu_ipcchn_set ipcchn_set;
	char dev_name[32];
#endif
	bool config_apply = false;

	dprintk("%s: cmd 0x%x, arg 0x%lx\n", __func__, cmd, arg);

	/* Don't even decode wrong cmds */
	if (_IOC_TYPE(cmd) != SERVICECPU_IOC_MAGIC) {
		pr_err("wrong ioctl magic key\n");
		return -ENOTTY;
	}
	switch (cmd) {
	case SERVICECPU_IOC_UNRESET:
		if (EXTERNAL_FW_IN_SRAM == info->scpu_mode) {
			if (copy_from_user(&reset, (unsigned int *)arg, sizeof(unsigned int))) {
				pr_err("copy_from_user failed\n");
				return -EFAULT;
			}

			if (reset)
				ret = servicecpu_kickoff_cpu(info);
			else
				ret = servicecpu_disable_cpu(info);
			if (ret)
				return ret;
		} else {
			pr_warn("ioctl(%x) is not supported in %s.\n", cmd, scpu_mode[info->scpu_mode]);
			return 0;
		}
		break;

#ifndef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	/* Send interrupt to firmware CPU (generate FIQ) */
	case SERVICECPU_IOC_SENDIRQ:
		/* Send interrupt to firmware CPU (generate Doorbell IRQ) */
		/* now this function only works in external mode */
		servicecpu_ipc_send_pci_drbl(IPC_FREERTOS_CPU_ID, IPC_FREERTOS_CHANNEL_ID);
		break;
#endif
	case SERVICECPU_IOC_CONFIG:
		if (copy_from_user(&config, (struct servicecpu_config *)arg,
				   sizeof(struct servicecpu_config))) {
			pr_err("copy_from_user failed\n");
			return -EFAULT;
		}

		if (EXTERNAL_FW_IN_SRAM == info->scpu_mode) {
			if (config.config_flag == CONFIG_SRAM) { /* flag for SRAM */
				servicecpu_mem_init_sram_config(&config);
				config_apply = true;
			} else {
				pr_err("%s is not supported in %s!\n",
					scpu_config[config.config_flag], scpu_mode[info->scpu_mode]);
				return -EPERM;
			}
		} else if (EXTERNAL_FW_IN_DRAM == info->scpu_mode) {
			if (config.config_flag == CONFIG_SRAM) {
				servicecpu_mem_init_sram_config(&config);
				servicecpu_mem_init.sram_configured = true;
			} else if (config.config_flag == CONFIG_DRAM) {
				servicecpu_mem_init_dram_config(&config);
				servicecpu_mem_init.dram_configured = true;
			} else if (config.config_flag == CONFIG_APPLY)
				config_apply = true;
			else
				return -EPERM;
		} else
			return -EPERM;

		if (config_apply == true) {
			ret = servicecpu_config_memmap(info);
			if (ret)
				return ret;
			/* do memory setup and ipc setup work */
			if (!info->servicecpu_initialized) {
				do_funcs(open, info);
				info->servicecpu_initialized = 1;
			}
		}
		break;
	case SERVICECPU_IOC_MMAPINFO:
		memset(&hostmem_info, 0, sizeof(struct servicecpu_hostmem_info));
		hostmem_info.items[0] = (unsigned int)info->seg_hostmem_phy_base;
		hostmem_info.items[1] = info->seg_hostmem_size;
		if (info->scpu_mode == EXTERNAL_FW_IN_DRAM) {
			hostmem_info.items[2] = (unsigned int)info->seg_hostmem_in_dram_phy_base;
			hostmem_info.items[3] = info->seg_hostmem_in_dram_size;
		}

		if (copy_to_user((struct servicecpu_hostmem_info *)arg, &hostmem_info,
				 sizeof(struct servicecpu_hostmem_info))) {
			pr_err("copy_from_user failed\n");
			return -EFAULT;
		}
		break;
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
	case SERVICECPU_IOC_SET_IPCCHN:
		if (copy_from_user(&ipcchn_set, (struct servicecpu_ipcchn_set *)arg,
			sizeof(struct servicecpu_ipcchn_set))) {
			pr_err("copy_from_user failed\n");
			return -EFAULT;
		}

		dprintk("%s: set channel %d status %d\n", __func__, ipcchn_set.chn_id, ipcchn_set.chn_en);

		if (ipcchn_set.chn_en) {
			memset(dev_name, 0, sizeof(dev_name));
			servicecpu_enable_channel(ipcchn_set.chn_id, dev_name);
			if (copy_to_user(ipcchn_set.dev_name, dev_name, sizeof(dev_name))) {
				pr_err("copy_to_user failed\n");
				return -EFAULT;
			}
		} else
			servicecpu_disable_channel(ipcchn_set.chn_id);

		break;
#endif
	default:
		pr_warn("Unknown ioctl (%x).\n", cmd);
		break;
	}
	return 0;
}

static loff_t servicecpu_lseek(struct file *filp, loff_t offset, int whence)
{
	size_t mem_size = info->seg_code_size;

	dprintk("%s(whence=0x%x, offset=0x%llx)\n", __func__, (whence), (offset));

	switch (whence) {
	case SEEK_SET:
		break;

	case SEEK_CUR:
		offset += filp->f_pos;
		break;

	case SEEK_END:
		offset += mem_size;
		break;

	default:
		return -EINVAL;
	}

	if (offset >= 0 && offset <= mem_size)
		return filp->f_pos = offset;

	dprintk("file pos 0x%llx\n", filp->f_pos);

	return -EINVAL;
}

static int servicecpu_mmap(struct file *file, struct vm_area_struct *vma)
{
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long phys = (unsigned int)(info->seg_hostmem_phy_base + off);
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = info->seg_hostmem_size - off;

	dprintk("%s off=%lx phys=%lx vsize=%lx psize=%lx\n", __func__, off, phys, vsize, psize);

	/* Allow only host mem mapping */
	if (vsize > psize) {
		pr_err("Trying to span too high during host mem mmap\n");
		return -EINVAL;
	}

	if (remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT, vsize,
							vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int servicecpu_open(struct inode *inode, struct file *file)
{
	if (info->servicecpu_initialized)
		return 0;

	/* do open funcs */
	do_funcs(open, info);

	info->servicecpu_initialized = 1;

	return 0;
}

static int servicecpu_release(struct inode *inode, struct file *file)
{
	dprintk("%s\n", __func__);

	return 0;
}

/* When run in SRAM, the ServiceCPU firmware was loaded by HOST cpu,
	so we need these functions to read data form SRAM and check the firmware content.
	When run in DRAM, the serviceCPU firmware was loaded directly from UBOOT,
	so the lseek, read and write functions do nothing for external mode with firmware run in SRAM.*/
static const struct file_operations servicecpu_fops_ex_fw_sram = {
	.llseek			= servicecpu_lseek,
	.read			= servicecpu_read,
	.write			= servicecpu_write,
	.unlocked_ioctl		= servicecpu_ioctl,
	.mmap			= servicecpu_mmap,
	.open			= servicecpu_open,
	.release		= servicecpu_release
};

static const struct file_operations servicecpu_fops_ex_fw_dram = {
	.unlocked_ioctl		= servicecpu_ioctl,
	.mmap			= servicecpu_mmap,
	.open			= servicecpu_open,
	.release		= servicecpu_release
};

static const struct file_operations servicecpu_fops_inter_fw_dram = {
	.llseek			= servicecpu_lseek,
	.read			= servicecpu_read,
	.unlocked_ioctl		= servicecpu_ioctl,
	.open			= servicecpu_open,
	.release		= servicecpu_release
};

static const struct file_operations *servicecpu_fops[SCPU_MODE_MAX] = {
	&servicecpu_fops_ex_fw_sram,
	&servicecpu_fops_ex_fw_dram,
	&servicecpu_fops_inter_fw_dram
};

static struct miscdevice servicecpu_dev = {
	SERVICECPU_MINOR,
	"servicecpu",
};

#ifdef CONFIG_OF
static struct of_device_id mv_servicecpu_match[] = {
	{ .compatible = "marvell,msys-servicecpu", },
	{},
};
#endif


#ifdef CONFIG_OF
static int of_servicecpu_init_common(struct device_node *np)
{
	struct resource		*res;
	struct device_node	*sections_node;
	struct device_node	*section_node;
	const char		*label;
	unsigned int		sum_sections_size = 0;
	const __be32		*reg;
	int			len;
	int			a_cells, s_cells;

	if (of_machine_is_compatible("marvell,msys")) {
		/* for msys, service cpu mode can only be internal mode */
		/* in this mode ServiceCPU firmware runs in DRAM*/
		info->scpu_mode = INTERNAL_FW_IN_DRAM;

		/* set scpu intel register base for internal mode*/
		res = platform_get_resource(info->pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&info->pdev->dev, "internal register resource missing\n");
			return -ENODEV;
		}
		scpu_inter_regs = devm_ioremap(&info->pdev->dev, res->start, resource_size(res));
		if (!scpu_inter_regs) {
			dev_err(&info->pdev->dev, "scpu_inter_regs ioremap failed\n");
			return -EBUSY;
		}
	} else
		/* set serviceCPU default mode to be external mode with firmware in DRAM */
		/* later if we can find "firmware" node in sram sections, */
		/* then serviceCPU mode will be updated to EXTERNAL_FW_IN_SRAM */
		info->scpu_mode = EXTERNAL_FW_IN_DRAM;

	sections_node = of_get_child_by_name(np, "sram_sections");
	if (sections_node) {
		if (info->scpu_mode == INTERNAL_FW_IN_DRAM)
			/* sram_phy_address is the total sram area physical address */
			/* for external modes, sram base address is assigned by prestera PCI sram window */
			/* so here we only need to consider internal mode */
			if (of_property_read_u32(sections_node, "sram_phy_address", &info->sram_phys_base)) {
				pr_err("could not get service cpu sram physical address in %s\n",
						scpu_mode[info->scpu_mode]);
				return -ENXIO;
			}

		/*get sram base offset and size in service cpu pci winow */
		reg = of_get_property(sections_node, "sram_base", &len);
		if (!reg) {
			pr_err("%s read sram_section %s reg failed!!!\n", __func__, sections_node->name);
			return -ENXIO;
		}
		a_cells = of_n_addr_cells(sections_node);
		s_cells = of_n_size_cells(sections_node);

		servicecpu_mem_init.sram_base_offset = (unsigned int)of_read_number(reg, a_cells);
		servicecpu_mem_init.sram_size = (unsigned int)of_read_number(reg + a_cells, s_cells);

		for_each_node_by_name(section_node, "sram_section") {
			/*get sram section label */
			if (0 != of_property_read_string(section_node, "label", &label)) {
				pr_err("%s read sram_section %s label failed!!!\n", __func__, section_node->name);
				return -ENXIO;
			}

			reg = of_get_property(section_node, "reg", &len);
			if (!reg) {
				pr_err("%s read sram_section %s reg failed!!!\n", __func__, section_node->name);
				return -ENXIO;
			}

			a_cells = of_n_addr_cells(section_node);
			s_cells = of_n_size_cells(section_node);

			if (0 == strcmp(label, "L2 Cache")) {
				servicecpu_mem_init.seg_cache_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "free area")) {
				servicecpu_mem_init.seg_free_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_free_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "host memory")) {
				servicecpu_mem_init.seg_hostmem_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_hostmem_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "OAM DB")) {
				servicecpu_mem_init.seg_oamdb_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_oamdb_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "share memory")) {
				servicecpu_mem_init.seg_shmem_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_shmem_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "firmware")) {
				if (info->scpu_mode == INTERNAL_FW_IN_DRAM) {
					pr_err("Firmware should be in dram sections for internal mode!!!\n");
					return -EPERM;
				}
				servicecpu_mem_init.seg_code_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_code_size = of_read_number(reg + a_cells, s_cells);
				info->scpu_mode = EXTERNAL_FW_IN_SRAM;
			}

			sum_sections_size += of_read_number(reg + a_cells, s_cells);
		}

		/* the configured sram size in dts should be equal to the sum of all sections' size*/
		if (sum_sections_size != servicecpu_mem_init.sram_size) {
			pr_err("%s the configured sram size is 0x%x while the sum of all sections's size is 0x%x!!!\n",
				__func__, servicecpu_mem_init.sram_size, sum_sections_size);
			return -EPERM;
		}
	}

	sections_node = of_get_child_by_name(np, "dram_sections");
	if (sections_node) {
		/*get dram base offset and size in service cpu pci winow */
		sum_sections_size = 0;

		if (info->scpu_mode == INTERNAL_FW_IN_DRAM) {
			/* In internal mode, ServiceCPU firmware will be compiled into the Linux kernel of HostCPU*/
			/* and load to the predefined firmware dram address during the boot stage of Linux. */
			/* Internal servicecpu dram area includes firmware and ipc share memory */
			/* dram_phy_address is the total dram area physical address */
			/* for external modes, dram base address is assigned by prestera PCI dram window */
			/* so here we only need to consider internal mode */
			if (of_property_read_u32(sections_node, "dram_phy_address", &info->dram_phys_base)) {
				pr_err("could not get service cpu dram physical address in %s\n",
						scpu_mode[info->scpu_mode]);
				return -EPERM;
			}
		}

		reg = of_get_property(sections_node, "dram_base", &len);
		if (!reg) {
			pr_err("%s read dram_section %s reg failed!!!\n", __func__, sections_node->name);
			return -ENXIO;
		}
		a_cells = of_n_addr_cells(sections_node);
		s_cells = of_n_size_cells(sections_node);

		servicecpu_mem_init.dram_base_offset = (unsigned int)of_read_number(reg, a_cells);
		servicecpu_mem_init.dram_size = (unsigned int)of_read_number(reg + a_cells, s_cells);

		for_each_node_by_name(section_node, "dram_section") {
			/*get dram section label */
			if (0 != of_property_read_string(section_node, "label", &label)) {
				pr_err("%s read dram_section %s label failed!!!\n", __func__, section_node->name);
				return -ENXIO;
			}

			reg = of_get_property(section_node, "reg", &len);
			if (!reg) {
				pr_err("%s read dram_section %s reg failed!!!\n", __func__, section_node->name);
				return -ENXIO;
			}

			a_cells = of_n_addr_cells(section_node);
			s_cells = of_n_size_cells(section_node);

			if (0 == strcmp(label, "host memory")) {
				servicecpu_mem_init.seg_hostmem_in_dram_offset = of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_hostmem_in_dram_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "share memory")) {
				servicecpu_mem_init.seg_shmem_in_dram_offset = of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_shmem_in_dram_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "firmware")) {
				if (info->scpu_mode == EXTERNAL_FW_IN_SRAM) {
					pr_err("Firmware is in sram sections already, can not be in dram again!!!\n");
					return -EPERM;
				}
				servicecpu_mem_init.seg_code_base = (unsigned int)of_read_number(reg, a_cells);
				servicecpu_mem_init.seg_code_size = of_read_number(reg + a_cells, s_cells);
			}
			sum_sections_size += of_read_number(reg + a_cells, s_cells);
		}

		/* the configured dram size in dts should be equal to the sum of all sections' size*/
		if (sum_sections_size != servicecpu_mem_init.dram_size) {
			pr_err("%s the configured dram size is 0x%x while the sum of all sections's size is 0x%x!!!\n",
				__func__, servicecpu_mem_init.dram_size, sum_sections_size);
			return -EPERM;
		}
	}

	return 0;
}
#endif

static int servicecpu_init_memmap_ext_sram_mode(struct servicecpu_info *info)
{
#ifdef CONFIG_OF
	if ((servicecpu_mem_init.sram_base_offset + servicecpu_mem_init.sram_size) > info->pci_win_size) {
		pr_err("the sum of base 0x%x and size 0x%x is larger than pci window size 0x%x\n",
			servicecpu_mem_init.sram_base_offset, servicecpu_mem_init.sram_size, info->pci_win_size);
		return -ENOMEM;
	}

	/* servicecpu_mem_init.sram_base_offset is the offset in service cpu pci window */
	info->sram_virt_base = info->pci_win_virt_base + servicecpu_mem_init.sram_base_offset;
	info->sram_phys_base = (unsigned int)info->pci_win_phys_base + servicecpu_mem_init.sram_base_offset;
	info->sram_size = servicecpu_mem_init.sram_size;

	/* adjust sram sections base to be virtual address */
	info->seg_cache_size = servicecpu_mem_init.seg_cache_size;
	info->seg_free_base = servicecpu_mem_init.seg_free_base + info->sram_virt_base;
	info->seg_free_size = servicecpu_mem_init.seg_free_size;
	info->seg_hostmem_base = servicecpu_mem_init.seg_hostmem_base + info->sram_virt_base;
	info->seg_hostmem_size = servicecpu_mem_init.seg_hostmem_size;
	info->seg_oamdb_base = servicecpu_mem_init.seg_oamdb_base + info->sram_virt_base;
	info->seg_oamdb_size = servicecpu_mem_init.seg_oamdb_size;
	info->seg_shmem_base = servicecpu_mem_init.seg_shmem_base + info->sram_virt_base;
	info->seg_shmem_size = servicecpu_mem_init.seg_shmem_size;
	info->seg_code_base = servicecpu_mem_init.seg_code_base + info->sram_virt_base;
	info->seg_code_size = servicecpu_mem_init.seg_code_size;
	pr_err("sram_virt_base %p sram_phys_base 0x%x\n", info->sram_virt_base, info->sram_phys_base);
	pr_err("seg_free_base %p seg_hostmem_base %p seg_oamdb_base %p seg_shmem_base %p seg_code_base %p\n",
			info->seg_free_base, info->seg_hostmem_base,
			info->seg_oamdb_base, info->seg_shmem_base, info->seg_code_base);
#else
	info->sram_virt_base = info->pci_win_virt_base;
	info->sram_phys_base = (unsigned int)info->pci_win_phys_base;
	info->sram_size = info->pci_win_size;
	info->seg_shmem_size = SERVICECPU_FW_IN_SRAM_SHMEM_SIZE_DEF;
	info->seg_code_size = SERVICECPU_FW_IN_SRAM_CODE_SIZE_DEF;
	info->seg_cache_size = SERVICECPU_FW_IN_SRAM_CACHE_SIZE_DEF;
	info->seg_free_size = SERVICECPU_FW_IN_SRAM_FREE_SIZE_DEF;
	info->seg_hostmem_size = SERVICECPU_FW_IN_SRAM_HOSTMEM_SIZE_DEF;
	info->seg_oamdb_size = SERVICECPU_FW_IN_SRAM_OAMDB_SIZE_DEF;

	info->seg_free_base = (void *)((unsigned int)info->sram_virt_base + info->seg_cache_size);
	info->seg_hostmem_base = (void *)((unsigned int)info->seg_free_base + info->seg_free_size);
	info->seg_oamdb_base = (void *)((unsigned int)info->seg_hostmem_base + info->seg_hostmem_size);
	info->seg_shmem_base = (void *)((unsigned int)info->seg_oamdb_base + info->seg_oamdb_size);
	info->seg_code_base = (void *)((unsigned int)info->seg_shmem_base + info->seg_shmem_size);
	info->seg_hostmem_phy_base = (void *)((unsigned int)info->sram_phys_base
				+ info->seg_cache_size + info->seg_free_size);
#endif

	return 0;
}

static int servicecpu_init_memmap_ext_dram_mode(struct servicecpu_info *info)
{
#ifdef CONFIG_OF
	if ((servicecpu_mem_init.sram_base_offset + servicecpu_mem_init.sram_size) > info->pci_win_size) {
		pr_err("the sum of base 0x%x and size 0x%x is larger than pci window size 0x%x\n",
			servicecpu_mem_init.sram_base_offset, servicecpu_mem_init.sram_size, info->pci_win_size);
		return -ENOMEM;
	}

	/* servicecpu_mem_init.sram_base_offset is the offset in service cpu pci window */
	info->sram_virt_base = info->pci_win_virt_base + servicecpu_mem_init.sram_base_offset;
	info->sram_phys_base = (unsigned int)info->pci_win_phys_base + servicecpu_mem_init.sram_base_offset;
	info->sram_size = servicecpu_mem_init.sram_size;

	/* servicecpu_mem_init.dram_base_offset is the offset in service cpu dram pci window*/
	info->dram_phys_base = (unsigned int)info->pci_win_dram_phys_base + servicecpu_mem_init.dram_base_offset;
	info->dram_size = servicecpu_mem_init.dram_size;

	/* adjust sram sections base to be virtual address */
	info->seg_cache_size = servicecpu_mem_init.seg_cache_size;
	info->seg_free_base = servicecpu_mem_init.seg_free_base + info->sram_virt_base;
	info->seg_free_size = servicecpu_mem_init.seg_free_size;
	info->seg_hostmem_base = servicecpu_mem_init.seg_hostmem_base + info->sram_virt_base;
	info->seg_hostmem_size = servicecpu_mem_init.seg_hostmem_size;
	info->seg_oamdb_base = servicecpu_mem_init.seg_oamdb_base + info->sram_virt_base;
	info->seg_oamdb_size = servicecpu_mem_init.seg_oamdb_size;

	/* adjust dram sections base to be read physical address */
	info->seg_hostmem_in_dram_offset = servicecpu_mem_init.seg_hostmem_in_dram_offset;
	info->seg_hostmem_in_dram_size = servicecpu_mem_init.seg_hostmem_in_dram_size;
	info->seg_shmem_in_dram_offset = servicecpu_mem_init.seg_shmem_in_dram_offset;
	info->seg_shmem_in_dram_size = servicecpu_mem_init.seg_shmem_in_dram_size;
	info->seg_hostmem_in_dram_phy_base = (void *)(servicecpu_mem_init.seg_hostmem_in_dram_offset
						+ info->dram_phys_base);
	/* needs to set dram virtual address in the future */
	info->seg_shmem_base = info->dram_virt_base + info->seg_shmem_in_dram_offset;
	info->seg_shmem_size = info->seg_shmem_in_dram_size;
#else
	info->sram_virt_base = info->pci_win_virt_base;
	info->sram_phys_base = (unsigned int)info->pci_win_phys_base;
	info->sram_size = info->pci_win_size;
	info->dram_phys_base = (unsigned int)info->pci_win_dram_phys_base;
	info->dram_size = SERVICECPU_FW_IN_DRAM_HOSTMEM_SIZE_DEF + SERVICECPU_FW_IN_DRAM_SHMEM_SIZE_DEF;
	info->seg_shmem_in_dram_offset = SERVICECPU_FW_IN_DRAM_SHMEM_OFFSET_DEF;
	info->seg_shmem_in_dram_size = SERVICECPU_FW_IN_DRAM_SHMEM_SIZE_DEF;
	info->seg_hostmem_in_dram_offset = SERVICECPU_FW_IN_DRAM_HOSTMEM_OFFSET_DEF;
	info->seg_hostmem_in_dram_size = SERVICECPU_FW_IN_DRAM_HOSTMEM_SIZE_DEF;
	info->seg_cache_size = SERVICECPU_FW_IN_DRAM_CACHE_SIZE_DEF;
	info->seg_free_size = SERVICECPU_FW_IN_DRAM_FREE_SIZE_DEF;
	info->seg_hostmem_size = SERVICECPU_FW_IN_DRAM_HOSTMEM_SIZE_DEF;
	info->seg_oamdb_size = SERVICECPU_FW_IN_DRAM_OAMDB_SIZE_DEF;

	info->seg_free_base = (void *)((unsigned int)info->sram_virt_base + info->seg_cache_size);
	info->seg_hostmem_base = (void *)((unsigned int)info->seg_free_base + info->seg_free_size);
	info->seg_oamdb_base = (void *)((unsigned int)info->seg_hostmem_base + info->seg_hostmem_size);
	info->seg_hostmem_in_dram_phy_base = (void *)(info->dram_phys_base + info->seg_hostmem_in_dram_offset);
	info->seg_hostmem_phy_base = (void *)((unsigned int)info->sram_phys_base +
				info->seg_cache_size + info->seg_free_size);
	/* needs to set dram virtual address in the future */
	info->seg_shmem_base = info->dram_virt_base + info->seg_shmem_in_dram_offset;
	info->seg_shmem_size = info->seg_shmem_in_dram_size;
#endif
	return 0;
}

static int servicecpu_init_memmap_inter_dram_mode(struct servicecpu_info *info)
{
	/* Calculate base addresses of segments  */
	info->sram_virt_base = ioremap_nocache(info->sram_phys_base, servicecpu_mem_init.sram_size);
	if (info->sram_virt_base == NULL)
		return -ENOMEM;
	info->sram_size = servicecpu_mem_init.sram_size;

	info->dram_virt_base = ioremap_nocache(info->dram_phys_base, servicecpu_mem_init.dram_size);
	if (info->dram_virt_base == NULL) {
		iounmap(info->sram_virt_base);
		return -ENOMEM;
	}
	info->dram_size = servicecpu_mem_init.dram_size;

	pr_err("ServiceCPU L2-SRAM Configuration: Cache_size=0x%x Sram_size=0x%x\n"
		"Sram_base=0x%x free_size=0x%x hostmem_size=0x%x oamdb_size=0x%x\n",
		info->seg_cache_size, info->seg_free_size + info->seg_hostmem_size + info->seg_oamdb_size,
		info->sram_phys_base, info->seg_free_size, info->seg_hostmem_size, info->seg_oamdb_size);

	/* adjust sram sections base to be virtual address */
	info->seg_free_base = servicecpu_mem_init.seg_free_base + info->sram_virt_base;
	info->seg_free_size = servicecpu_mem_init.seg_free_size;
	info->seg_hostmem_base = servicecpu_mem_init.seg_hostmem_base + info->sram_virt_base;
	info->seg_hostmem_size = servicecpu_mem_init.seg_hostmem_size;
	info->seg_oamdb_base = servicecpu_mem_init.seg_oamdb_base + info->sram_virt_base;
	info->seg_oamdb_size = servicecpu_mem_init.seg_oamdb_size;

	/* adjust dram sections base to be read physical address */
	info->seg_code_base = servicecpu_mem_init.seg_code_base + info->dram_virt_base;
	info->seg_code_size = servicecpu_mem_init.seg_code_size;
	info->seg_shmem_in_dram_offset = servicecpu_mem_init.seg_shmem_in_dram_offset;
	info->seg_shmem_in_dram_size = servicecpu_mem_init.seg_shmem_in_dram_size;
	info->seg_shmem_base = info->dram_virt_base + info->seg_shmem_in_dram_offset;
	info->seg_shmem_size = info->seg_shmem_in_dram_size;

	return 0;
}

/***********************************************************************************
 * servicecpu_reserve_mem
 *
 * DESCRIPTION:
 *			reserve memory for service cpu
 *
 * INPUT:
 *			servicecpu_mem_base - reserve memory base
 *			servicecpu_mem_size  - reserve memory size
 * OUTPUT:
 *			None
 * RETURN:
 *			0: OK
 *			other: ERROR
 *
 **********************************************************************************/
static int servicecpu_reserve_mem(unsigned int servicecpu_mem_base,
					unsigned int servicecpu_mem_size)
{
	int error;

	/* no need reserve */
	if ((servicecpu_mem_base == 0) || (servicecpu_mem_size == 0))
		return 0;

	if ((servicecpu_mem_base & ~PAGE_MASK) || (servicecpu_mem_size & ~PAGE_MASK)) {
		pr_err("servicecpu: Specified memory region [0x%08X-0x%08X] is not page aligned!\n",
			 servicecpu_mem_base, servicecpu_mem_base + servicecpu_mem_size - 1);

		return -EPERM;
	}

	if (!memblock_is_region_memory(servicecpu_mem_base, servicecpu_mem_size)) {
		pr_err("servicecpu: Specified memory region [0x%08X-0x%08X] is invalid!\n",
			 servicecpu_mem_base, servicecpu_mem_base + servicecpu_mem_size - 1);

		return -ENOMEM;
	}

	if (memblock_is_region_reserved(servicecpu_mem_base, servicecpu_mem_size)) {
		pr_err("servicecpu: Specified memory region [0x%08X-0x%08X] is reserved!\n",
			 servicecpu_mem_base, servicecpu_mem_base + servicecpu_mem_size - 1);

		return -EPERM;
	}

	error = memblock_remove(servicecpu_mem_base, servicecpu_mem_size);
	if (error) {
		pr_err("servicecpu: Cannot reserve memory [0x%08X-0x%08X] (error %i)\n",
			servicecpu_mem_base, servicecpu_mem_base + servicecpu_mem_size - 1, error);
		return -ENOMEM;
	}

	return 0;
}

#ifdef CONFIG_OF
static int servicecpu_scan_dram_reserve(unsigned long node, const char *uname, int depth, void *data)
{
	int error;
	__be32 *reg;
	unsigned int dram_phy_address, dram_base, dram_size;
	unsigned long l;

	reg = of_get_flat_dt_prop(node, "dram_phy_address", &l);
	if (reg == NULL)
		return 0;

	dram_phy_address = dt_mem_next_cell(1, &reg);

	reg = of_get_flat_dt_prop(node, "dram_base", &l);
	if (reg == NULL)
		return 0;

	dram_base = dt_mem_next_cell(1, &reg);
	dram_size = dt_mem_next_cell(1, &reg);

	error = servicecpu_reserve_mem(dram_phy_address, dram_size);
	if (error) {
		pr_err("servicecpu: Cannot reserve dram memory [0x%08X-0x%08X] (error %i)\n",
			dram_phy_address, dram_phy_address + dram_size - 1, error);
		servicecpu_data.servicecpu_enabled = 0;
		return -1;
	}

	servicecpu_reserve_mem_ok = true;
	return 0;
}

/***********************************************************************************
 * servicecpu_msys_reserve_memory
 *
 * DESCRIPTION:
 *              reserve dram memory for msys in internal Service CPU mode
 *
 * INPUT:
 *		None
 * OUTPUT:
 *		None
 * RETURN:
 *		None
 *
 ************************************************************************************/
void servicecpu_msys_reserve_memory(void)
{
	/* Reserve dram memory for internal Service CPU */
	of_scan_flat_dt(servicecpu_scan_dram_reserve, NULL);

	return;
}
#endif

static int servicecpu_probe(struct platform_device *pdev)
{
	int ret;
#ifdef CONFIG_OF
	const struct of_device_id *match;
	struct device_node	*np = pdev->dev.of_node;
#endif

	dev_info(&pdev->dev, "Initializing Service CPU...\n");

	match = of_match_device(mv_servicecpu_match, &pdev->dev);
	if (!match)
		return -EINVAL;

#ifdef CONFIG_OF
	info = &servicecpu_data;
	pdev->dev.platform_data = info;
#else
	info = dev_get_platdata(&pdev->dev);
	if (!info) {
		dev_err(&pdev->dev, "Missing servicecpu data structure\n");
		return -ENOMEM;
	}
#endif

	info->pdev = pdev;

#ifdef CONFIG_OF
	ret = of_servicecpu_init_common(np);
	if (ret) {
		dev_info(&pdev->dev, "of_servicecpu_init_common failed, ret %d\n", ret);
		return ret;
	}
#endif

	/* do probe funcs */
	do_funcs(probe, info);

	servicecpu_dev.fops = servicecpu_fops[info->scpu_mode];
	ret = misc_register(&servicecpu_dev);
	if (ret)
		return ret;

	dev_info(&pdev->dev, "Initializing Co-processor finished\n");

	return 0;
}

static int servicecpu_remove(struct platform_device *pdev)
{
	if (info->servicecpu_initialized) {
#ifdef CONFIG_MV_SERVICECPU_IPC_SUPPORT
		ipc_wrapper_link_unbind(IPC_SCPU_FREERTOS_LINK_ID);
#endif
	}

	info->servicecpu_initialized  = 0;

	return 0;
}

static struct platform_driver servicecpu_driver = {
	.probe		= servicecpu_probe,
	.remove		= servicecpu_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "servicecpu",
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mv_servicecpu_match),
#endif
	},
};

#ifndef CONFIG_OF
static struct platform_device servicecpu_device = {
	.name		= "servicecpu",
	.id		= -1,
	.dev		= {
		.platform_data  = &servicecpu_data,
	},
};
#endif

#ifdef CONFIG_OF
static int __init servicecpu_init(void)
#else
int servicecpu_init(void)
#endif
{
	int rc;

#ifdef CONFIG_OF
	/* for AMC host(external mode), service cpu only initialized when run with MSYS CPUs */
	if (!of_machine_is_compatible("marvell,msys") && !servicecpu_data.servicecpu_enabled) {
		pr_err("NOT run with proper PCI device, skip servicecpu_init\n");
		return 0;
	}

	/* for msys host(internal mode), service cpu only initialized after dram memory of servicecpu is reserved */
	if (of_machine_is_compatible("marvell,msys") && !servicecpu_reserve_mem_ok) {
		pr_err("Memory for servicecpu is not reserved, skip servicecpu_init\n");
		return 0;
	}
#endif

	rc = platform_driver_register(&servicecpu_driver);

#ifndef CONFIG_OF
	if (rc)
		return rc;
	rc = platform_device_register(&servicecpu_device);
	if (rc)
		platform_driver_unregister(&servicecpu_driver);
#endif

	return rc;
}
#ifdef CONFIG_OF
late_initcall(servicecpu_init);
#endif

#ifdef CONFIG_OF
void __exit servicecpu_deinit(void)
#else
void servicecpu_deinit(void)
#endif
{
#ifdef CONFIG_OF
	if (!of_machine_is_compatible("marvell,msys") && !servicecpu_data.servicecpu_enabled) {
		pr_err("NOT run with proper PCI device, skip servicecpu_deinit\n");
		return;
	}

	if (of_machine_is_compatible("marvell,msys") && !servicecpu_reserve_mem_ok) {
		pr_err("Memory for servicecpu is not reserved, skip servicecpu_deinit\n");
		return;
	}
#endif

#ifndef CONFIG_OF
	platform_device_unregister(&servicecpu_device);
#endif
	platform_driver_unregister(&servicecpu_driver);
}
#ifdef CONFIG_OF
module_exit(servicecpu_deinit);
#endif
