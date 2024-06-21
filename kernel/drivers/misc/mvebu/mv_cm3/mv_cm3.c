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
   available along with the File in the license.txt file or by writing to the Free
   Software Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA 02111-1307 or
   on the worldwide web at http://www.gnu.org/licenses/gpl.txt.

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
* mv_cm3.c
*
* DESCRIPTION:
*	Cortex-M3 (POE) support for MSYS devices (BobK/BC2).
*
* DEPENDENCIES:
*
*******************************************************************************/

/* #define DEBUG */

#include <linux/init.h>
#include <linux/interrupt.h>
#include <linux/platform_device.h>
#include <linux/of.h>
#include <linux/mutex.h>
#include <linux/miscdevice.h>
#include <linux/fs.h>
#include <linux/uaccess.h>
#include <linux/poll.h>
#include <linux/io.h>
#include <linux/mm.h>
#include <linux/sched.h>
#ifdef CONFIG_OF
#include <linux/of_device.h>
#endif
#ifdef CONFIG_MV_CM3_IPC_SUPPORT
#include "mv_ipc.h"
#include "mv_ipc_wrapper.h"
#endif

#include "mv_cm3.h"

#define _4K         0x00001000

/* ctrl reg fields */
#define CM3_CPU_EN_BIT		BIT(28)
#define CM3_INIT_RAM		BIT(20)
#define CM3_PROC_CLK_DIS	BIT(19)

/* metal fixed reg fields*/
#define CM3_METAL_FIX_JTAG_EN	BIT(0)

/* TODO: after moving interrupt configuration to irq driver this will be
 * removed
 */
/* poe_cause_irq_reg fields */
#define MG_INTERNAL_SUM_MASK	(1 << 7)
/* poe_mg_cause_irq_reg fields */
#define MG_DOORBELL_FROM_CM3	(1 << 14)

#define _4B_ALIGN 4

#undef MV_CM3_DBG

static const char drv_name_internal[] = "cm3";

static ssize_t cm3_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	struct cm3_info *info = filp->private_data;
	struct device *dev = &info->pdev->dev;
	size_t mem_size;
	char *src_ptr;
	int i;

	dev_dbg(dev, "%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	if (info->mem_direction == PROC_MEM) {
		mem_size = info->proc_mem_size;
		src_ptr = info->proc_mem_base + *f_pos;
	} else /* info->mem_direction == PROC_SRAM */ {
		mem_size = info->proc_sram_size;
		src_ptr = info->proc_sram_base + *f_pos;
	}

	if (*f_pos >= mem_size || !count)
		return 0;
	if (count > mem_size - *f_pos)
		count = mem_size - *f_pos;

	for (i = 0; i < count; i++) {
		if (copy_to_user(buf + i, src_ptr + i, 1)) {
			dev_err(dev, "%s: copy_to_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_CM3_DBG
		if (i < 500)
			dev_dbg(dev, "0x%x ", *(src_ptr + i));
#endif
	}

	dev_dbg(dev, "\n%s: read 0x%x, from %s f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, info->mem_direction ? "PROC_SRAM" : "PROC_MEM",
		*f_pos, src_ptr);

	return i;
}

static ssize_t cm3_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct cm3_info *info = filp->private_data;
	struct device *dev = &info->pdev->dev;
	size_t mem_size;
	char *dst_ptr;
	int i;

	dev_dbg(dev, "%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	if (info->mem_direction == PROC_MEM) {
		mem_size = info->proc_mem_size;
		dst_ptr = info->proc_mem_base + *f_pos;
	} else /* info->mem_direction == PROC_SRAM */ {
		mem_size = info->proc_sram_size;
		dst_ptr = info->proc_sram_base + *f_pos;
	}

	if (*f_pos + count > mem_size) {
		dev_err(dev, "Not enough space in %s for writing %d",
			info->mem_direction ? "PROC_SRAM" : "PROC_MEM", count);
		return -ENOMEM;
	}

	for (i = 0; i < count; i = i + _4B_ALIGN) {
		if (copy_from_user(dst_ptr + i, buf + i, _4B_ALIGN)) {
			dev_err(dev, "%s: copy_from_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_CM3_DBG
		/* The jtag_reg is present only in BobK SoC. This allows us to
		 * indicate if we are working with BobK or BC2. Becasue of BC2
		 * hw bug any 8/16 bit access hangs the board - so this debug
		 * can be enabled only for BobK and not for BC2
		 */
		if ((info->jtag_reg) && (i < 500))
			dev_dbg(dev, "0x%x ", readb(dst_ptr + i));
#endif
	}

	dev_dbg(dev, "\n%s: writen 0x%x, to %s f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, info->mem_direction ? "PROC_SRAM" : "PROC_MEM",
		*f_pos, dst_ptr);

	return i;
}


static unsigned int cm3_poll(struct file *filp, poll_table *poll_table_p)
{
	struct cm3_info *info = filp->private_data;
	int mask = 0;

	poll_wait(filp, &info->irq_queue, poll_table_p);

	if (info->irq_get > 0) {
		mask |= POLLPRI;
		info->irq_get--;
	}

	return mask;
}

static int protect_tcm(struct cm3_info *info)
{
	struct device *dev = &info->pdev->dev;

	/* After releasing the cm3 from reset do not allow to access any
	 * memory instead of 4KB of PROC_SRAM starting at address 0. This protection
	 * is provided by unmaping PROC_MEM and ioremap only first 4KB of PROC_SRAM
	 */
	dev_dbg(dev, "proc_sram_base %p, proc_mem_base %p\n",
		info->proc_sram_base, info->proc_mem_base);

	if (info->proc_mem_base) {
		iounmap(info->proc_mem_base);
		info->proc_mem_base = NULL;
	}

	info->proc_mem_size = 0;

	info->proc_sram_base = devm_ioremap_nocache(dev, info->proc_sram_phys,
						    _4K);
	if (!info->proc_sram_base) {
		dev_dbg(dev, "Failed with re-mapping switch address space\n");
		return -EFAULT;
	}
	info->proc_sram_size = _4K;
	dev_dbg(dev, "proc_sram_base %p, proc_mem_base %p\n",
		info->proc_sram_base, info->proc_mem_base);

	return 0;
}

static int mv_unreset_cm3(uint32_t cm3_state, struct cm3_info *info)
{
	struct device *dev = &info->pdev->dev;
	int reg, ret;

	reg = readl(info->ctrl_reg);
	dev_dbg(dev, "%s, BF cm3 configuration read: 0x%x\n", __func__, reg);

	switch (cm3_state) {
	case D_RESET:
		reg &= ~(CM3_CPU_EN_BIT);
		break;
	/* protect and un-reset cm3: missing break by purpose */
	case D_UNRESET_AND_PROTECT_MEM:
		ret = protect_tcm(info);
		if (ret)
			return ret;
	case D_UNRESET:
		reg |= CM3_CPU_EN_BIT;
		break;
	default:
		dev_err(dev, "%s: ERROR unknown value\n", __func__);
		return -EINVAL;
	}

	writel(reg, info->ctrl_reg);
	reg = readl(info->ctrl_reg);

	dev_dbg(dev, "%s, cm3 configuration read: 0x%x from addr 0x%p\n",
		__func__, reg, info->ctrl_reg);

	return 0;
}

static loff_t cm3_lseek(struct file *filp, loff_t offset, int whence)
{
	struct cm3_info *info = filp->private_data;
	struct device *dev = &info->pdev->dev;
	size_t mem_size;

	dev_dbg(dev, "%s(whence=0x%x, offset=0x%llx)\n",
		__func__, (whence), (offset));

	if (info->mem_direction == PROC_MEM)
		mem_size = info->proc_mem_size;
	else /* info->mem_direction == PROC_SRAM */
		mem_size = info->proc_sram_size;

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

	dev_dbg(dev, "file pos 0x%llx\n", filp->f_pos);

	return -EINVAL;
}

static long cm3_do_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct cm3_info *info = filp->private_data;
	struct device *dev = &info->pdev->dev;
	int ret;
	unsigned int reset;
	struct cm3_filep cm3_wfp, cm3_rfp;

	dev_dbg(dev, "%s: cmd 0x%x, arg 0x%lx\n", __func__, cmd, arg);

	/* Don't even decode wrong cmds */
	if (_IOC_TYPE(cmd) != CM3_IOC_MAGIC) {
		dev_err(dev, "wrong ioctl magic key\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case CM3_IOC_SETMEM_TYPE:
		if (copy_from_user(&info->mem_direction, (uint32_t *)arg, sizeof(unsigned int))) {
			dev_err(dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		if (info->mem_direction != PROC_MEM && info->mem_direction != PROC_SRAM) {
			dev_err(dev, "wrong memory direction\n");
			return -EINVAL;
		}

		dev_dbg(dev, "%s: mem direction(%d): %s\n", __func__,
			info->mem_direction,
			info->mem_direction ? "PROC_SRAM" : "PROC_MEM");
		break;

	case CM3_IOC_UNRESET:
		if (copy_from_user(&reset, (uint32_t *)arg, sizeof(unsigned int))) {
			dev_err(dev, "copy_from_user failed\n");
			return -EFAULT;
		}

		ret = mv_unreset_cm3(reset, info);
		if (ret)
			return ret;
		break;

#ifndef CONFIG_MV_CM3_IPC_SUPPORT
	case CM3_IOC_SENDIRQ:
		/* Send interrupt to firmware CPU */
		writel(0x1, info->host2poe_irq_reg);
		dev_dbg(dev, "host2poe_irq_reg: %x from addr %p\n",
			readl(info->host2poe_irq_reg), info->host2poe_irq_reg);

		break;
#endif
	case CM3_IOC_READ:
		if (copy_from_user(&cm3_rfp, (struct cm3_filep *)arg,
				   sizeof(cm3_rfp))) {
			dev_err(dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		info->mem_direction = cm3_rfp.mem_direction;

		ret = cm3_lseek(filp, cm3_rfp.offset, SEEK_SET);
		if (ret < 0)
			return ret;

		ret = cm3_read(filp, cm3_rfp.buf, cm3_rfp.count, &filp->f_pos);
		if (ret < 0)
			return ret;

		break;

	case CM3_IOC_WRITE:
		if (copy_from_user(&cm3_wfp, (struct cm3_filep *)arg, sizeof(cm3_wfp))) {
			dev_err(dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		info->mem_direction = cm3_wfp.mem_direction;

		ret = cm3_lseek(filp, cm3_wfp.offset, SEEK_SET);
		if (ret < 0)
			return ret;

		ret = cm3_write(filp, cm3_wfp.buf, cm3_wfp.count, &filp->f_pos);
		if (ret < 0)
			return ret;

		break;

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
	case CM3_IOC_SET_IPCCHN:
	{
		struct cm3_ipcchn_set ipcchn_set;
		char ipc_dev_name[32];

		if (copy_from_user(&ipcchn_set, (struct cm3_ipcchn_set *)arg,
			sizeof(struct cm3_ipcchn_set))) {
			dev_err(dev, "copy_from_user failed\n");
			return -EFAULT;
		}

		dev_dbg(dev, "set ipc channel %d status %d\n",
			ipcchn_set.chn_id, ipcchn_set.chn_en);

		if (ipcchn_set.chn_en) {
			memset(ipc_dev_name, 0, sizeof(ipc_dev_name));
			ret = cm3_enable_ipc_channel(ipcchn_set.chn_id,
						     ipc_dev_name);
			if (ret) {
				dev_err(dev, "enabling chn failed\n");
				return ret;
			}

			if (copy_to_user(ipcchn_set.dev_name, ipc_dev_name,
				sizeof(ipc_dev_name))) {
				dev_err(dev, "copy_to_user failed\n");
				return -EFAULT;
			}
		} else {
			ret = cm3_disable_ipc_channel(ipcchn_set.chn_id);
			if (ret) {
				dev_err(dev, "disabling chn failed\n");
				return ret;
			}
		}

		break;
	}
#endif
	default:
		dev_warn(dev, "Unknown ioctl (%x).\n", cmd);
		return -EINVAL;
	}

	return 0;
}

static long cm3_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct cm3_info *info = filp->private_data;
	int ret;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	ret = cm3_do_ioctl(filp, cmd, arg);

	mutex_unlock(&info->lock);

	return ret;
}

static int cm3_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct cm3_info *info = file->private_data;
	struct device *dev = &info->pdev->dev;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long phys = info->proc_sram_phys + off;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = info->proc_sram_size - off;

	/* Allow only proc_sram mapping */
	if (vsize > psize) {
		dev_err(dev, "Trying to span too high during proc_sram\n");
		return -EINVAL;
	}

	/* VM_IO for I/O memory */
	vma->vm_flags |= VM_IO;

	/* Disable caching on mapped memory */
	vma->vm_page_prot = pgprot_noncached(vma->vm_page_prot);

	if (remap_pfn_range(vma, vma->vm_start, phys >> PAGE_SHIFT, vsize,
							vma->vm_page_prot))
		return -EAGAIN;

	return 0;
}

static int cm3_open(struct inode *inode, struct file *file)
{
	int ret;
	u32 reg;
	struct cm3_info *info;
	struct device *dev;

	/*
	 * The miscdevice layer puts our struct miscdevice into the
	 * filp->private_data field. We use this to find our private data
	 * which is a container for the miscdev. Then we overwrite
	 * file->private_data with our own private structure.
	 */
	info = container_of(file->private_data, struct cm3_info, miscdev);

	file->private_data = info;
	dev = &info->pdev->dev;

	dev_dbg(dev, "%s\n", __func__);

	if (info->cm3_initialized)
		return 0;

	/* Disable cm3 */
	ret = mv_unreset_cm3(D_RESET, info);
	if (ret)
		return ret;

	/* Configure cm3 */
	reg = readl(info->ctrl_reg);
	dev_dbg(dev, "%s, BR cm3 configuration read: 0x%x\n", __func__, reg);
	writel(((reg | CM3_INIT_RAM) & (~CM3_PROC_CLK_DIS)), info->ctrl_reg);
	reg = readl(info->ctrl_reg);

	dev_dbg(dev, "%s, cm3 configuration read: 0x%x\n", __func__, reg);
	/* Enable JTAG (Metal fix) - presented only on BobK and not in BC2 */
	if (!info->jtag_reg)
		goto skip_jtag;

	reg = readl(info->jtag_reg);
	dev_dbg(dev, "%s, BR cm3 metal-fix read: 0x%x\n", __func__, reg);
	writel(reg | CM3_METAL_FIX_JTAG_EN, info->jtag_reg);

	reg = readl(info->jtag_reg);
	dev_dbg(dev, "%s, cm3 metal-fix read: 0x%x\n", __func__, reg);

skip_jtag:
	info->mem_direction = PROC_MEM;

	info->cm3_initialized = 1;

	return 0;
}

static int cm3_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations cm3_fops = {
	.llseek			= cm3_lseek,
	.read			= cm3_read,
	.write			= cm3_write,
	.poll			= cm3_poll,
	.unlocked_ioctl		= cm3_ioctl,
	.mmap			= cm3_mmap,
	.open			= cm3_open,
	.release		= cm3_release
};

#ifdef CONFIG_OF
static struct of_device_id mv_cm3_dt_ids[] = {
	{ .compatible = "marvell,msys-bc2-cm3", },
	{ .compatible = "marvell,msys-bobk-cm3", },
	{},
};
#endif

static irqreturn_t cm3_interrupt(int irq, void *dev_id)
{
	struct cm3_info *info = dev_id;

	/* Doorbell from cm3 arrives */

	/* if nobody cares about irq in user-space, stop
	 * indicating them
	 */
	if (info->irq_get < 10000) {
		info->irq_get++;
		wake_up_interruptible(&info->irq_queue);
	}

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
	return IRQ_WAKE_THREAD;
#else
	return IRQ_HANDLED;
#endif
}

/* XXX: remove whole switch irq connected part, when swic and mg_swic will be
 * supported in LSPv2.6 and in cm3 over pcie
 */
static irqreturn_t cm3_interrupt_no_swic(int irq, void *dev_id)
{
	struct cm3_info *info = dev_id;

	int reg;
	reg = readl(info->poe_cause_irq_reg);
	if (reg & MG_INTERNAL_SUM_MASK) {
		reg = readl(info->poe_mg_cause_irq_reg);
		if (reg & MG_DOORBELL_FROM_CM3) {
			/* Doorbell from cm3 arrives */

			/* if nobody cares about irq in user-space, stop
			 * indicating them
			 */
			if (info->irq_get < 10000) {
				info->irq_get++;
				wake_up_interruptible(&info->irq_queue);
			}

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
			return IRQ_WAKE_THREAD;
#else
			return IRQ_HANDLED;
#endif
		}
	}

	return IRQ_NONE;
}

/* XXX: remove whole switch irq connected part, when swic and mg_swic will be
 * supported in LSPv2.6 and in cm3 over pcie
 */
static void cm3_swic_unmask_irq(struct cm3_info *info)
{
	struct platform_device *pdev = info->pdev;
	int reg;

	/* Unmask global switch irq  in MG */
	reg = readl(info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq mask read %x\n", reg);
	writel(reg | MG_INTERNAL_SUM_MASK, info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq unmask read %x\n", readl(info->poe_mask_irq_reg));

	/* Unmask mg_sum switch irq in MG */
	reg = readl(info->poe_mg_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq mask read %x\n", reg);
	writel(reg | MG_DOORBELL_FROM_CM3, info->poe_mg_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq unmask read %x\n", readl(info->poe_mg_mask_irq_reg));
}

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
static irqreturn_t ipc_thread_irq(int irq, void *data)
{
	/* Process IPC msg */
	ipc_wrapper_link_rx_process(IPC_CM3_FREERTOS_LINK_ID);

	return IRQ_HANDLED;
}

/* Platform and application specific function, which will trigger MSYS2CM3
 * interrupt, used to signal the other side of IPC tunnel (CM3), that the msg
 * is prepared and ready for process. It may be empty, if the other IPC side
 * operates in polling mode.
 */
static void cm3_ipc_trigger_irq(unsigned int linkId, unsigned int chnId, void *dev_id)
{
	/* Currently slave is configured to pooling mode - do nothing */

	/* TODO: Send interrupt to firmware CPU */
}

/* Platform specific, user call-back used for masking/unmasking doorbell */
static void cm3_ipc_enable_doorbell(unsigned int linkId, unsigned int chnId, bool enable, void *dev_id)
{
	/* Because there is only one doorbell irq from cm3 to msys, do not
	 * mask/unmask it per-channel
	 */
}

/* Platform specific, user call-back used for interrupt registering */
static int cm3_ipc_request_irq(unsigned int link_id, void *dev_id)
{
	struct cm3_info *info = dev_id;
	struct platform_device *pdev = info->pdev;
	const char *drv_name;
	int irq, ret;

	drv_name = drv_name_internal;

	irq = platform_get_irq(pdev, 0);

	if (info->mg_swic_support == true) {
		ret = request_threaded_irq(irq, cm3_interrupt, ipc_thread_irq,
					   IRQF_SHARED, drv_name, (void *)info);
	} else {
		/* XXX: remove whole switch irq connected part, when swic and
		 * mg_swic will be supported in LSPv2.6 in cm3 over pcie
		 */
		cm3_swic_unmask_irq(info);
		ret = request_threaded_irq(irq, cm3_interrupt_no_swic,
			   ipc_thread_irq, IRQF_SHARED, drv_name, (void *)info);
	}

	if (ret) {
		dev_err(&pdev->dev, "error during irq request %d\n", ret);
		return ret;
	}

	return 0;
}


int cm3_set_ipc_linkinfo(struct ipc_link_info *link_info)
{
	struct cm3_info *info = (struct cm3_info *)link_info->dev_id;

	if (info->proc_sram_size == 0)
		return -EINVAL;

	link_info->shmem_size          = info->proc_sram_size;
	link_info->shmem_virt_addr     = info->proc_sram_base;
	link_info->send_trigger        = cm3_ipc_trigger_irq;
	link_info->enable_chn_doorbell = cm3_ipc_enable_doorbell;
	link_info->irq_init            = cm3_ipc_request_irq;

	return 0;
}

int cm3_enable_ipc_channel(unsigned int chn_id, char *dev_name)
{
	sprintf(dev_name, "/dev/ipc-lnk%d/chn%d", IPC_CM3_FREERTOS_LINK_ID, chn_id);

	return ipc_wrapper_channel_register(IPC_CM3_FREERTOS_LINK_ID, chn_id);
}

int cm3_disable_ipc_channel(unsigned int chn_id)
{
	return ipc_wrapper_channel_unregister(IPC_CM3_FREERTOS_LINK_ID, chn_id);
}
#endif

static int mv_cm3_probe(struct platform_device *pdev)
{
	struct resource *res;
	struct cm3_info *info;
	const char *drv_name;
	int ret;
#ifndef CONFIG_MV_CM3_IPC_SUPPORT
	int irq;
#endif
#ifdef CONFIG_OF
	const struct of_device_id *of_id = of_match_device(mv_cm3_dt_ids, &pdev->dev);
#else
	const struct of_device_id *of_id = NULL;
#endif

	dev_info(&pdev->dev, "Initializing CM3 Co-processor...\n");

	drv_name = drv_name_internal;

	if (of_id) {
		/* We are here when running from LSPv3.10 cm3 internal (Not
		 * when running from cm3 over PCI either cm3 on LSPv2.6)
		 */
		info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
		if (!info)
			return -ENOMEM;

		info->mg_swic_support = true;

		/* proc_mem - place for code in cm3 */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 0);
		if (!res) {
			dev_err(&pdev->dev, "proc_mem memory resource missing\n");
			return -ENODEV;
		}

		info->proc_mem_size = resource_size(res);
		info->proc_mem_base = devm_ioremap(&pdev->dev, res->start,
						info->proc_mem_size);
		if (!info->proc_mem_base) {
			dev_err(&pdev->dev, "proc_mem ioremap failed\n");
			return -EBUSY;
		}

		/* proc_sram - cm3 sram */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 1);
		if (!res) {
			dev_err(&pdev->dev, "proc_sram memory resource missing\n");
			return -ENODEV;
		}

		info->proc_sram_size = resource_size(res);
		info->proc_sram_base = devm_ioremap(&pdev->dev, res->start,
						info->proc_sram_size);
		if (!info->proc_sram_base) {
			dev_err(&pdev->dev, "proc_sram ioremap failed\n");
			return -EBUSY;
		}

		info->proc_sram_phys = res->start;

		/* config reg */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (!res) {
			dev_err(&pdev->dev, "proc_config resource missing\n");
			return -ENODEV;
		}

		info->ctrl_reg = devm_ioremap(&pdev->dev, res->start,
						resource_size(res));
		if (!info->ctrl_reg) {
			dev_err(&pdev->dev, "ctrl_reg ioremap failed\n");
			return -EBUSY;
		}

		/* host to poe irq */
		res = platform_get_resource(pdev, IORESOURCE_MEM, 3);
		if (!res) {
			dev_err(&pdev->dev, "host2poe resource missing\n");
			return -ENODEV;
		}

		info->host2poe_irq_reg = devm_ioremap(&pdev->dev, res->start,
						resource_size(res));
		if (!info->host2poe_irq_reg) {
			dev_err(&pdev->dev, "host2poe_irq_reg ioremap failed\n");
			return -EBUSY;
		}

		/* JTAG - MG Metal Fix Register - missing in BC2 */
		if (of_device_is_compatible(pdev->dev.of_node, "marvell,msys-bobk-cm3")) {

			/* config reg */
			res = platform_get_resource(pdev, IORESOURCE_MEM, 4);
			if (!res) {
				dev_err(&pdev->dev, "matl fix resource missing\n");
				return -ENODEV;
			}

			info->jtag_reg = devm_ioremap(&pdev->dev, res->start,
							resource_size(res));
			if (!info->ctrl_reg) {
				dev_err(&pdev->dev, "ctrl_reg ioremap failed\n");
				return -EBUSY;
			}
		}
	} else {
		/* We are here when running from LSPv2.6 or when running from
		 * LSPv3.10 over PCI (no dts node about cm3, 'pdata' is
		 * statically filled-out in core.c (LSPv2.6) or in
		 * mv_prestera_pci.c (cm3 over PCI for both LSPv2.6 and
		 * LSPv3.10)
		 */
		info = dev_get_platdata(&pdev->dev);
		if (!info) {
			dev_err(&pdev->dev, "Missing cm3 data structure\n");
			return -ENOMEM;
		}

		info->mg_swic_support = false;
	}

	/* Prepare the miscdevice */
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = drv_name;
	info->miscdev.fops = &cm3_fops;

	ret = misc_register(&info->miscdev);
	if (ret) {
		dev_err(&pdev->dev, "registering miscdev failed\n");
		return ret;
	}

	info->pdev = pdev;

#ifndef CONFIG_MV_CM3_IPC_SUPPORT
	irq = platform_get_irq(pdev, 0);

	if (info->mg_swic_support == true)
		ret = request_irq(irq, cm3_interrupt, IRQF_SHARED, drv_name,
								(void *)info);
	else {
		cm3_swic_unmask_irq(info);
		ret = request_irq(irq, cm3_interrupt_no_swic, IRQF_SHARED,
							drv_name, (void *)info);
	}
	if (ret) {
		dev_err(&pdev->dev, "error during irq request %d\n", ret);
		goto fail_deregister;
	}
#else
	ret = ipc_wrapper_link_bind(IPC_CM3_FREERTOS_LINK_ID, info, cm3_set_ipc_linkinfo);
	if (ret)
		goto fail_deregister;
#endif

	dev_info(&pdev->dev, "Initializing Co-processor finished\n");

	dev_dbg(&pdev->dev, "proc_mem_base %p\n", info->proc_mem_base);
	dev_dbg(&pdev->dev, "proc_mem_size 0x%x\n", info->proc_mem_size);
	dev_dbg(&pdev->dev, "proc_sram_base %p\n", info->proc_sram_base);
	dev_dbg(&pdev->dev, "proc_sram_size 0x%x\n", info->proc_sram_size);
	dev_dbg(&pdev->dev, "proc_sram_phys 0x%x\n", info->proc_sram_phys);
	dev_dbg(&pdev->dev, "ctrl_reg %p\n", info->ctrl_reg);

	mutex_init(&info->lock);
	info->cm3_initialized = 0;
	init_waitqueue_head(&info->irq_queue);
	info->irq_get = 0;

	return 0;

fail_deregister:
	misc_deregister(&info->miscdev);

	return ret;
}

static int mv_cm3_remove(struct platform_device *pdev)
{
#ifdef CONFIG_MV_CM3_IPC_SUPPORT
	ipc_wrapper_link_unbind(IPC_CM3_FREERTOS_LINK_ID);
#endif
	return 0;
}

static struct platform_driver mv_cm3_driver = {
	.probe		= mv_cm3_probe,
	.remove		= mv_cm3_remove,
	.driver		= {
		.owner	        = THIS_MODULE,
		.name	        = MV_CM3_NAME,
#ifdef CONFIG_OF
		.of_match_table = of_match_ptr(mv_cm3_dt_ids),
#endif
	},
};

static int __init cm3_pltfm_init(void)
{
	return platform_driver_register(&mv_cm3_driver);
}
module_init(cm3_pltfm_init);

static void __exit cm3_pltfm_exit(void)
{
	platform_driver_unregister(&mv_cm3_driver);
}
module_exit(cm3_pltfm_exit);
