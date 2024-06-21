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
* dragonite_xcat.c
*
* DESCRIPTION:
*	Dragonite (POE) support for AlleyCat3.
*
* DEPENDENCIES:
*
*******************************************************************************/

#include <linux/module.h>
#include <linux/types.h>
#include <linux/kernel.h>
#include <linux/fs.h>
#include <linux/of_device.h>
#include <linux/of_address.h>
#include <linux/miscdevice.h>
#include <linux/platform_device.h>
#include <linux/errno.h>
#include <linux/init.h>
#include <linux/io.h>
#include <linux/uaccess.h>
#include <linux/slab.h>
#include <linux/interrupt.h>
#include <linux/of.h>
#include <linux/of_irq.h>
#include <linux/poll.h>
#include <linux/sched.h>
#include <linux/mutex.h>
#include <linux/mm.h>
#include <linux/mv_addr_compl.h>
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
#include "mv_ipc.h"
#include "mv_ipc_wrapper.h"
#endif


#include "dragonite_xcat.h"

static const char drv_name_internal[] = "dragonite";
static const char drv_name_pci[] = "dragonite_pci";

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
static int dragonite_set_ipc_linkinfo(struct ipc_link_info *link_info);
#endif


static ssize_t dragonite_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	struct dragonite_info *info = filp->private_data;
	size_t mem_size;
	char *src_ptr;
	int i;

	dev_dbg(&info->pdev->dev, "%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	if (info->mem_direction == ITCM_DIR) {
		mem_size = info->itcm_size;
		src_ptr = info->itcm_base + *f_pos;
	} else /* info->mem_direction == DTCM_DIR */ {
		mem_size = info->dtcm_size;
		src_ptr = info->dtcm_base + *f_pos;
	}

	if (*f_pos >= mem_size || !count)
		return 0;
	if (count > mem_size - *f_pos)
		count = mem_size - *f_pos;

	for (i = 0; i < count; i++) {
		if (copy_to_user(buf + i, src_ptr + i, 1)) {
			dev_err(&info->pdev->dev, "%s: copy_to_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_DRAGONITE_DBG
		if (i < 500)
			dev_dbg(&info->pdev->dev, "0x%x ", *(src_ptr + i));
#endif
	}

	dev_dbg(&info->pdev->dev, "\n%s: read 0x%x, from %s f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, info->mem_direction ? "DTCM" : "ITCM", *f_pos, src_ptr);

	return i;
}

static ssize_t dragonite_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct dragonite_info *info = filp->private_data;
	size_t mem_size;
	char *dst_ptr;
	int i;

	dev_dbg(&info->pdev->dev, "%s: count 0x%x, f_pos 0x%llx\n", __func__, count, *f_pos);

	if (info->mem_direction == ITCM_DIR) {
		mem_size = info->itcm_size;
		dst_ptr = info->itcm_base + *f_pos;
	} else /* info->mem_direction == DTCM_DIR */ {
		mem_size = info->dtcm_size;
		dst_ptr = info->dtcm_base + *f_pos;
	}

	if (*f_pos + count > mem_size) {
		dev_err(&info->pdev->dev, "Not enough space in %s for writing %d",
			info->mem_direction ? "DTCM" : "ITCM", count);
		return -ENOMEM;
	}

	for (i = 0; i < count; i++) {
		if (copy_from_user(dst_ptr + i, buf + i, 1)) {
			dev_err(&info->pdev->dev, "%s: copy_from_user failed.\n", __func__);
			return -EFAULT;
		}

#ifdef MV_DRAGONITE_DBG
		if (i < 500)
			dev_dbg(&info->pdev->dev, "0x%x ", readb(dst_ptr + i));
#endif
	}

	dev_dbg(&info->pdev->dev, "\n%s: writen 0x%x, to %s f_pos 0x%llx, mem_base_addr 0x%p\n",
		__func__, i, info->mem_direction ? "DTCM" : "ITCM", *f_pos, dst_ptr);

	return i;
}

static unsigned int dragonite_poll(struct file *filp, poll_table *poll_table_p)
{
	struct dragonite_info *info = filp->private_data;
	int mask = 0;

	poll_wait(filp, &info->irq_queue, poll_table_p);

	if (info->irq_get > 0) {
		mask |= POLLPRI;
		info->irq_get--;
	}

	return mask;
}

static int protect_tcm(struct dragonite_info *info)
{
	/* After releasing the Dragonite from reset do not allow to access any
	 * memory instead of the protected dtcm section. This protection
	 * is provided by unmaping ITCM and ioremap the protected dtcm section.
	 * The protected dtcm section is generally 4KB of DTCM starting at address 0.
	 */
	dev_dbg(&info->pdev->dev, "dtcm %p, itcm %p\n", info->dtcm_base, info->itcm_base);
	iounmap(info->itcm_base);
	info->itcm_size = 0;

	info->dtcm_base = devm_ioremap_nocache(&info->pdev->dev,
						info->dtcm_phys + info->protected_dtcm_offset,
						info->protected_dtcm_size);
	if (!info->dtcm_base) {
		dev_dbg(&info->pdev->dev, "Failed with re-mapping DTCM space\n");
		return -EFAULT;
	}

	info->dtcm_size = info->protected_dtcm_size;
	dev_dbg(&info->pdev->dev, "dtcm %p, itcm %p\n", info->dtcm_base, info->itcm_base);

	return 0;
}

static int mv_unreset_dragonite(uint32_t drag_state, struct dragonite_info *info)
{
	int reg, ret;

	reg = readl(info->ctrl_reg);
	dev_dbg(&info->pdev->dev, "%s, BF dragonite configuration read: 0x%x\n", __func__, reg);

	switch (drag_state) {
	case D_RESET:
		reg &= ~(DRAGONITE_CPU_EN_BIT);
		break;
	/* protect and un-reset dragonite: missing break by purpose */
	case D_UNRESET_AND_PROTECT_TCM:
		ret = protect_tcm(info);
		if (ret)
			return ret;
	case D_UNRESET:
		reg |= DRAGONITE_CPU_EN_BIT;
		break;
	default:
		dev_err(&info->pdev->dev, "%s: ERROR unknown value\n", __func__);
		return -EINVAL;
	}

	writel(reg, info->ctrl_reg);
	reg = readl(info->ctrl_reg);

	dev_dbg(&info->pdev->dev, "%s, dragonite configuration read: 0x%x from addr 0x%p\n",
		__func__, reg, info->ctrl_reg);

	return 0;
}

static loff_t dragonite_lseek(struct file *filp, loff_t offset, int whence)
{
	struct dragonite_info *info = filp->private_data;
	size_t mem_size;
	dev_dbg(&info->pdev->dev, "%s(whence=0x%x, offset=0x%llx)\n", __func__, (whence), (offset));

	if (info->mem_direction == ITCM_DIR)
		mem_size = info->itcm_size;
	else /* info->mem_direction == DTCM */
		mem_size = info->dtcm_size;

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

	dev_dbg(&info->pdev->dev, "file pos 0x%llx\n", filp->f_pos);

	return -EINVAL;
}

static long dragonite_do_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dragonite_info *info = filp->private_data;
	int ret;
	enum dragonite_state reset;
	struct dragontie_filep drag_wfp, drag_rfp;
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	struct dragonite_ipcchn_set ipcchn_set;
	char dev_name[32];
#endif

	dev_dbg(&info->pdev->dev, "%s: cmd 0x%x, arg 0x%lx\n", __func__, cmd, arg);

	/* Don't even decode wrong cmds */
	if (_IOC_TYPE(cmd) != DRAGONITE_IOC_MAGIC) {
		dev_err(&info->pdev->dev, "wrong ioctl magic key\n");
		return -ENOTTY;
	}

	switch (cmd) {
	case DRAGONITE_IOC_SETMEM_TYPE:
		if (copy_from_user(&info->mem_direction, (uint32_t *)arg, sizeof(unsigned int))) {
			dev_err(&info->pdev->dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		if (info->mem_direction != ITCM_DIR && info->mem_direction != DTCM_DIR) {
			dev_err(&info->pdev->dev, "wrong memory direction\n");
			return -EINVAL;
		}

		dev_dbg(&info->pdev->dev, "%s: mem direction(%d): %s\n", __func__,
			info->mem_direction, info->mem_direction ? "DTCM" : "ITCM");
		break;
	case DRAGONITE_IOC_UNRESET:
		if (copy_from_user(&reset, (enum dragonite_state *)arg, sizeof(enum dragonite_state))) {
			dev_err(&info->pdev->dev, "copy_from_user failed\n");
			return -EFAULT;
		}

		ret = mv_unreset_dragonite(reset, info);
		if (ret)
			return ret;
		break;
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	case DRAGONITE_IOC_SENDIRQ:
		/* Send interrupt to firmware CPU (generate FIQ) */
		writel(0x1, info->host2poe_irq_reg);
		dev_dbg(&info->pdev->dev, "host2poe_irq_reg: %x from addr %p\n",
			readl(info->host2poe_irq_reg), info->host2poe_irq_reg);

		break;
#endif
	case DRAGONITE_IOC_READ:
		if (copy_from_user(&drag_rfp, (struct dragontie_filep *)arg, sizeof(drag_rfp))) {
			dev_err(&info->pdev->dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		info->mem_direction = drag_rfp.mem_direction;

		ret = dragonite_lseek(filp, drag_rfp.offset, SEEK_SET);
		if (ret < 0)
			return ret;

		ret = dragonite_read(filp, drag_rfp.buf, drag_rfp.count, &filp->f_pos);
		if (ret < 0)
			return ret;

		break;
	case DRAGONITE_IOC_WRITE:
		if (copy_from_user(&drag_wfp, (struct dragontie_filep *)arg, sizeof(drag_wfp))) {
			dev_err(&info->pdev->dev, "copy_from_user failed\n");
			return -EFAULT;
		}
		info->mem_direction = drag_wfp.mem_direction;

		ret = dragonite_lseek(filp, drag_wfp.offset, SEEK_SET);
		if (ret < 0)
			return ret;

		ret = dragonite_write(filp, drag_wfp.buf, drag_wfp.count, &filp->f_pos);
		if (ret < 0)
			return ret;

		break;
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	case DRAGONITE_IOC_SET_IPCCHN:
		if (copy_from_user(&ipcchn_set, (struct dragonite_ipcchn_set *)arg,
			sizeof(struct dragonite_ipcchn_set))) {
			dev_err(&info->pdev->dev, "copy_from_user failed\n");
			return -EFAULT;
		}

		dev_dbg(&info->pdev->dev, "set ipc channel %d status %d\n",
			ipcchn_set.chn_id, ipcchn_set.chn_en);

		if (ipcchn_set.chn_en) {
			memset(dev_name, 0, sizeof(dev_name));
			dragonite_enable_ipc_channel(ipcchn_set.chn_id, dev_name);
			if (copy_to_user(ipcchn_set.dev_name, dev_name,
				sizeof(dev_name))) {
				dev_err(&info->pdev->dev, "copy_to_user failed\n");
				return -EFAULT;
			}
		} else
			dragonite_disable_ipc_channel(ipcchn_set.chn_id);

		break;
#endif

	default:
		dev_warn(&info->pdev->dev, "Unknown ioctl (%x).\n", cmd);
		break;
	}

	return 0;
}

static long dragonite_ioctl(struct file *filp, unsigned int cmd, unsigned long arg)
{
	struct dragonite_info *info = filp->private_data;
	int ret;

	ret = mutex_lock_interruptible(&info->lock);
	if (ret)
		return ret;

	ret = dragonite_do_ioctl(filp, cmd, arg);

	mutex_unlock(&info->lock);

	return ret;
}

static int dragonite_mmap(struct file *file, struct vm_area_struct *vma)
{
	struct dragonite_info *info = file->private_data;
	unsigned long off = vma->vm_pgoff << PAGE_SHIFT;
	unsigned long phys = info->dtcm_phys + off;
	unsigned long vsize = vma->vm_end - vma->vm_start;
	unsigned long psize = info->dtcm_size - off;

	/* Allow only dtcm mapping */
	if (vsize > psize) {
		dev_err(&info->pdev->dev, "Trying to span too high during dtcm mmap\n");
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

static int dragonite_open(struct inode *inode, struct file *file)
{
	int ret;
	u32 reg;
	struct dragonite_info *info;
	struct dragonite_fw_config	fwcfg;

	/*
	 * The miscdevice layer puts our struct miscdevice into the
	 * filp->private_data field. We use this to find our private data
	 * which is a container for the miscdev. Then we overwrite
	 * file->private_data with our own private structure.
	 */
	info = container_of(file->private_data, struct dragonite_info, miscdev);
	file->private_data = info;
	dev_dbg(&info->pdev->dev, "%s\n", __func__);

	if (info->dragonite_initialized)
		return 0;

#ifdef CONFIG_DRAGONITE_JTAG_ENABLE
	/* Enable JTAG (DFX sections) */
	reg = readl(info->jtag_reg);
	dev_dbg(&info->pdev->dev, "%s, BF DFX jtag reg read: 0x%x\n", __func__, reg);
	writel(reg | DFX_JTAG_ENABLE_VAL, info->jtag_reg);
	reg = readl(info->jtag_reg);
	dev_dbg(&info->pdev->dev, "%s, DFX jtag reg read: 0x%x\n", __func__, reg);
#endif

	/* In BOBK, the SPI(for spi2uart) should be configured by reg: Device General Control 19 (DFX sections)*/
	reg = readl(info->poe_spi_enable_reg);
	dev_dbg(&info->pdev->dev, "%s, BF dragonite spi config reg read: 0x%x\n", __func__, reg);
	writel(reg | DFX_SPI_ENABLE_VAL, info->poe_spi_enable_reg);
	reg = readl(info->poe_spi_enable_reg);
	dev_dbg(&info->pdev->dev, "%s, dragonite spi config reg read: 0x%x\n", __func__, reg);


	/* Disable Dragonite */
	ret = mv_unreset_dragonite(D_RESET, info);
	if (ret)
		return ret;

	/* Configure Dragonite */
	reg = readl(info->ctrl_reg);
	dev_dbg(&info->pdev->dev, "%s, BF dragonite configuration read: 0x%x\n", __func__, reg);
	writel(reg | DRAGONITE_EN_BIT | DRAGONITE_CPU_INITRAM_BIT | DRAGONITE_CPU_GPP0FUNC,
	       info->ctrl_reg);
	reg = readl(info->ctrl_reg);

	dev_dbg(&info->pdev->dev, "%s, dragonite configuration read: 0x%x\n", __func__, reg);

	info->mem_direction = ITCM_DIR;

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	/* set up mipc */
	ret = ipc_wrapper_link_bind(IPC_DRG_FREERTOS_LINK_ID, info, dragonite_set_ipc_linkinfo);
	if (ret)
		return ret;
#endif

	/* Prepare firmware configuration */
	fwcfg.magic = DRAGONITE_FW_CONFIG_MAGIC;
	fwcfg.ipc_shm_offset = info->shmem_base - info->dtcm_base;
	fwcfg.ipc_shm_size = info->shmem_size;

	/* Write firmware configuration block at the end of the DTCM memory */
	memcpy_toio(info->dtcm_base + info->dtcm_size - sizeof(fwcfg), &fwcfg, sizeof(fwcfg));

	info->dragonite_initialized = 1;

	return 0;
}

static int dragonite_release(struct inode *inode, struct file *file)
{
	return 0;
}

static const struct file_operations dragonite_fops = {
	.llseek			= dragonite_lseek,
	.read			= dragonite_read,
	.write			= dragonite_write,
	.poll			= dragonite_poll,
	.unlocked_ioctl		= dragonite_ioctl,
	.mmap			= dragonite_mmap,
	.open			= dragonite_open,
	.release		= dragonite_release
};

static irqreturn_t dragonite_interrupt(int irq, void *dev_id)
{
	struct dragonite_info *info = dev_id;
	int reg;

	reg = readl(info->poe_cause_irq_reg);

	/* if nobody cares about irq in user-space, stop indicating them */
	if (info->irq_get < 10000) {
		info->irq_get++;
		wake_up_interruptible(&info->irq_queue);
	}

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	/* Process IPC msg */
	ipc_wrapper_link_rx_process(IPC_DRG_FREERTOS_LINK_ID);
#endif

	return IRQ_HANDLED;
}

static irqreturn_t dragonite_pci_handler(int irq, void *dev_id)
{
#ifdef DRAGONITE_OVER_PCIE /* TODO dragonite over pcie */
	struct dragonite_info *info = dev_id;
	int reg;

	reg = readl(info->msys_iregs_base + MSYS_CAUSE_VEC1_REG_OFFS);
	dev_dbg(NULL, "msys cause reg 0x%x, dragonite_mask 0x%x\n",
						      reg, IRQ_DRAGONITE_MASK);

	if ((reg & IRQ_DRAGONITE_MASK) == 0)
		return IRQ_NONE;

	reg = readl(info->poe_cause_irq_reg);

	/* if nobody cares about irq in user-space, stop indicating them */
	if (info->irq_get < 10000) {
		info->irq_get++;
		wake_up_interruptible(&info->irq_queue);
	}
#endif

	return IRQ_HANDLED;
}

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT

/* Platform and application specific function, which will trigger MSYS2DRAGONITE
 * interrupt, used to signal the other side of IPC tunnel (DRAGONITE), that the msg
 * is prepared and ready for process. It may be empty, if the other IPC side
 * operates in polling mode.
 */
static void dragonite_ipc_trigger_irq(unsigned int linkId, unsigned int chnId, void *dev_id)
{
	/* Currently slave is configured to pooling mode - do nothing */

	/* TODO: Send interrupt to firmware CPU */
}

/* Platform specific, user call-back used for masking/unmasking doorbell */
static void dragonite_ipc_enable_doorbell(unsigned int linkId, unsigned int chnId, bool enable, void *dev_id)
{
	/* Because there is only one doorbell irq from dragonite to msys, do not
	 * mask/unmask it per-channel
	 */
}

/* Platform specific, user call-back used for interrupt registering */
static int dragonite_ipc_request_irq(unsigned int link_id, void *dev_id)
{
	struct dragonite_info *info = dev_id;
	struct platform_device *pdev = info->pdev;
	const char *drv_name;
	int irq, ret, reg;

	drv_name = drv_name_internal;

	irq = platform_get_irq(pdev, 0);

	ret = request_irq(irq, dragonite_interrupt, IRQF_SHARED, drv_name,
								(void *)info);
	if (ret) {
		dev_err(&pdev->dev, "error during irq request %d\n", ret);
		return ret;
	}

	/* Unmask global switch irq  in MG */
	reg = readl(info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq mask read %x\n", reg);
	writel(reg | DRAGONITE_POE_IRQ_BIT, info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq unmask read %x\n", readl(info->poe_mask_irq_reg));
	return 0;
}


static int dragonite_set_ipc_linkinfo(struct ipc_link_info *link_info)
{
	struct dragonite_info *info = (struct dragonite_info *)link_info->dev_id;

	if (info->shmem_base == 0)
		return -EINVAL;

	link_info->shmem_size          = info->shmem_size;
	link_info->shmem_virt_addr     = info->shmem_base;
	link_info->send_trigger        = dragonite_ipc_trigger_irq;
	link_info->enable_chn_doorbell = dragonite_ipc_enable_doorbell;
	link_info->irq_init            = dragonite_ipc_request_irq;

	return 0;
}

int dragonite_enable_ipc_channel(unsigned int chn_id, char *dev_name)
{
	sprintf(dev_name, "/dev/ipc-lnk%d/chn%d", IPC_DRG_FREERTOS_LINK_ID, chn_id);

	return ipc_wrapper_channel_register(IPC_DRG_FREERTOS_LINK_ID, chn_id);
}

int dragonite_disable_ipc_channel(unsigned int chn_id)
{
	return ipc_wrapper_channel_unregister(IPC_DRG_FREERTOS_LINK_ID, chn_id);
}
#endif

static struct of_device_id mv_drag_dt_ids[] = {
	{ .compatible = "marvell,msys-bobk-dragonite", },
	{},
};

static int dragonite_xcat_probe(struct platform_device *pdev)
{
	struct resource *itcm, *dtcm, *msys_iregs;
	struct dragonite_info *info;
	int ret, reg_val, irq = 0;
	bool poe_on_pci = false;
	unsigned int addr;
	struct resource *res;
	const char *drv_name;
	const struct of_device_id *match;
	unsigned int compl_region_id;
	phys_addr_t poe_base_addr;
	unsigned int config_regs[2];
	struct device_node	*dtcm_sections_node;
	struct device_node	*dtcm_section_node;
	const char		*label;
	const __be32		*reg;
	int			len;
	int			a_cells, s_cells;
	unsigned int		sum_dtcm_sections_size = 0;

	dev_info(&pdev->dev, "Initializing Dragonite Co-processor...\n");

	match = of_match_device(mv_drag_dt_ids, &pdev->dev);
	if (!match)
		return -EINVAL;

	info = devm_kzalloc(&pdev->dev, sizeof(*info), GFP_KERNEL);
	if (!info)
		return -ENOMEM;

	pdev->dev.platform_data = info;

	irq = irq_of_parse_and_map(pdev->dev.of_node, 0);
	if (irq == NO_IRQ)
		return -ENXIO;

	/* For dragonite over pcie, remap pcie endpoint iregs and with use of it
	 * setup dragonite interrupts as pcie endpoint
	 */
	if (irq == IRQ_AURORA_PCIE0) {
		dev_err(&pdev->dev, "Unsupported feature: dragonite over pcie\n");
#ifdef DRAGONITE_OVER_PCIE
		dev_dbg(&pdev->dev, "dragontie over pci irq %d\n", irq);
		poe_on_pci = true;
		drv_name = drv_name_pci;

		msys_iregs = platform_get_resource(pdev, IORESOURCE_MEM, 2);
		if (msys_iregs == NULL) {
			dev_err(&pdev->dev, "MSYS iregs memory resource missing\n");
			return -ENODEV;
		}

		info->msys_iregs_size = resource_size(msys_iregs);
		info->msys_iregs_base = devm_ioremap_nocache(&pdev->dev,
				     msys_iregs->start, info->msys_iregs_size);
		if (!info->msys_iregs_base) {
			dev_dbg(&pdev->dev, "Failed mapping MSYS iregs space\n");
			return -EFAULT;
		}

		/* Prepare interrupts: set as pci endpoint and enable */
		addr = CPU_INT_SOURCE_CONTROL_REG(IRQ_POE_DRAGONITE);
		reg_val = readl(info->msys_iregs_base + addr);
		reg_val |= (PEX_IRQ_EN) | (PEX_IRQ_EP);
		writel(reg_val, info->msys_iregs_base + addr);

		dev_dbg(&pdev->dev, "irq status and ctrl(0x%x) = 0x%x\n",
			addr, readl(info->msys_iregs_base + addr));

		/* Clear irq */
		writel(IRQ_POE_DRAGONITE,
			info->msys_iregs_base + CPU_INT_CLEAR_MASK_LOCAL_REG);
		dev_dbg(&pdev->dev, "clear reg(0x%x)\n", CPU_INT_CLEAR_MASK_LOCAL_REG);
#endif
	} else
		drv_name = drv_name_internal;

	info->pdev = pdev;

	/* Map the dragonite tightly coupled memory (TCM) to host cpu */
	itcm = platform_get_resource(pdev, IORESOURCE_MEM, 0);
	if (itcm == NULL) {
		dev_err(&pdev->dev, "ITCM memory resource missing\n");
		ret = -ENODEV;
		goto fail_unmap_msys_iregs;
	}

	info->itcm_size = resource_size(itcm);
	if (!poe_on_pci)
		if (!request_mem_region(itcm->start, info->itcm_size, pdev->name)) {
			dev_dbg(&pdev->dev, "Failed requesting ITCM region\n");
			ret = -EBUSY;
			goto fail_unmap_msys_iregs;
		}

	info->itcm_base = devm_ioremap_nocache(&pdev->dev, itcm->start, info->itcm_size);
	if (!info->itcm_base) {
		dev_dbg(&pdev->dev, "Failed mapping ITCM space\n");
		ret = -EFAULT;
		goto fail_release_itcm;
	}

	dtcm = platform_get_resource(pdev, IORESOURCE_MEM, 1);
	if (dtcm == NULL) {
		dev_err(&pdev->dev, "DTCM memory resource missing\n");
		ret = -EBUSY;
		goto fail_unmap_itcm;
	}

	info->dtcm_phys = dtcm->start;

	info->dtcm_size = resource_size(dtcm);
	if (!poe_on_pci)
		if (!request_mem_region(dtcm->start, info->dtcm_size, pdev->name)) {
			dev_dbg(&pdev->dev, "Failed requesting DTCM region\n");
			ret = -EBUSY;
			goto fail_unmap_itcm;
		}

	info->dtcm_base = devm_ioremap_nocache(&pdev->dev, dtcm->start, info->dtcm_size);
	if (!info->dtcm_base) {
		dev_dbg(&pdev->dev, "Failed mapping DTCM space\n");
		ret = -EFAULT;
		goto fail_release_dtcm;
	}

	/* In BOBK, the SPI(for spi2uart) should be configured by reg: Device General Control 19 (DFX sections)*/
	res = platform_get_resource(pdev, IORESOURCE_MEM, 2);
	if (!res) {
		dev_err(&pdev->dev, "spi reg resource missing\n");
		return -ENODEV;
	}

	info->poe_spi_enable_reg = devm_ioremap(&pdev->dev, res->start,
					resource_size(res));
	if (!info->poe_spi_enable_reg) {
		dev_err(&pdev->dev, "poe_spi_enable_reg ioremap failed\n");
		return -EBUSY;
	}

	/* set default values */
	/* The protected dtcm section is generally 4KB of DTCM starting at address 0 */
	info->protected_dtcm_offset = 0;
	info->protected_dtcm_size = DRAGONITE_PROTECTED_DTCM_SIZE_DEFAULT;
	info->shmem_size = DRAGONITE_SHARED_DTCM_SIZE_DEFAULT;
	info->shmem_base = info->dtcm_base + info->dtcm_size - info->shmem_size;

	dtcm_sections_node = of_get_child_by_name(pdev->dev.of_node, "dtcm_sections");
	if (dtcm_sections_node) {
		for_each_node_by_name(dtcm_section_node, "dtcm_section") {
			/*get dtcm section label */
			if (0 != of_property_read_string(dtcm_section_node, "label", &label)) {
				pr_err("%s read dtcm_section %s label failed!!!\n", __func__, dtcm_section_node->name);
				return -ENXIO;
			}

			reg = of_get_property(dtcm_section_node, "reg", &len);
			if (!reg) {
				pr_err("%s read dtcm_section %s reg failed!!!\n", __func__, dtcm_section_node->name);
				return -ENXIO;
			}

			a_cells = of_n_addr_cells(dtcm_section_node);
			s_cells = of_n_size_cells(dtcm_section_node);

			if ((of_read_number(reg, a_cells) + of_read_number(reg + a_cells, s_cells))
				> info->dtcm_size) {
				pr_err("%s the configured dtcm size is 0x%x while section %s end is 0x%llx!!!\n",
				__func__, info->dtcm_size, label,
				of_read_number(reg, a_cells) + of_read_number(reg + a_cells, s_cells));
				return -EPERM;
			}

			if (0 == strcmp(label, "user memory")) {
				/* this piece of memory is only used in protected mode*/
				info->protected_dtcm_offset = of_read_number(reg, a_cells);
				info->protected_dtcm_size = of_read_number(reg + a_cells, s_cells);
			} else if (0 == strcmp(label, "shared memory")) {
				info->shmem_base = info->dtcm_base + of_read_number(reg, a_cells);
				info->shmem_size = of_read_number(reg + a_cells, s_cells);
			}

			sum_dtcm_sections_size += of_read_number(reg + a_cells, s_cells);
		}

		if (sum_dtcm_sections_size > info->dtcm_size) {
			pr_err("%s the configured dtcm size is 0x%x while the sum of all sections's size is 0x%x!!!\n",
				__func__, info->dtcm_size, sum_dtcm_sections_size);
			return -EPERM;
		}
	}

	/* Dragonite config regs */

	/* init the addr completion region, config_reg[0] is the config reg base address,
	config_reg[1] is the region size */
	ret = of_property_read_u32_array(pdev->dev.of_node, "config_reg", config_regs, 2);
	ret = mv_addr_compl_request_region(config_regs[0], &compl_region_id, &poe_base_addr);
	info->ctrl_reg = devm_ioremap(&pdev->dev, poe_base_addr, config_regs[1]);
	if (!info->ctrl_reg) {
		dev_err(&pdev->dev, "ctrl_reg ioremap failed\n");
		return -EBUSY;
	}

	info->poe_cause_irq_reg = info->ctrl_reg + POE_IRQ_CAUSE_OFFSET;
	info->poe_mask_irq_reg = info->ctrl_reg + POE_IRQ_MASK_OFFSET;
	info->host2poe_irq_reg = info->ctrl_reg + POE_IRQ_HOST2POE_OFFSET;

	info->jtag_reg = info->poe_spi_enable_reg; /*jtag reg is the same with spi enable reg */

	/* Prepare the miscdev miscdevice */
	info->miscdev.minor = MISC_DYNAMIC_MINOR;
	info->miscdev.name = drv_name;
	info->miscdev.fops = &dragonite_fops;

	ret = misc_register(&info->miscdev);
	if (ret)
		goto fail_unmap_dtcm;


#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	if (!poe_on_pci)
		ret = request_irq(irq, dragonite_interrupt, IRQF_DISABLED,
			drv_name_internal, (void *)info);
	else
		ret = request_irq(irq, dragonite_pci_handler, IRQF_SHARED,
			drv_name_pci, (void *)info);
	if (ret) {
		dev_err(&pdev->dev, "error requesting irq %d\n", ret);
		goto fail_deregister;
	}

	/* Unmask irq */
	reg_val = readl(info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq mask read %x\n", readl(info->poe_mask_irq_reg));
	writel(reg_val | DRAGONITE_POE_IRQ_BIT, info->poe_mask_irq_reg);
	dev_dbg(&pdev->dev, "irq unmask read %x\n", readl(info->poe_mask_irq_reg));
#endif
	mutex_init(&info->lock);
	info->dragonite_initialized = 0;
	init_waitqueue_head(&info->irq_queue);
	info->irq_get = 0;

	dev_info(&pdev->dev, "Initializing Co-processor finished\n");
	return 0;

fail_deregister:
	misc_deregister(&info->miscdev);
fail_unmap_dtcm:
	iounmap(info->dtcm_base);
fail_release_dtcm:
	if (!poe_on_pci)
		release_mem_region(dtcm->start, resource_size(dtcm));
fail_unmap_itcm:
	iounmap(info->itcm_base);
fail_release_itcm:
	if (!poe_on_pci)
		release_mem_region(itcm->start, resource_size(itcm));
fail_unmap_msys_iregs:
	if (poe_on_pci)
		iounmap(info->msys_iregs_base);

	return ret;
}

static int dragonite_xcat_remove(struct platform_device *pdev)
{
	struct dragonite_info *info;
	info = (struct dragonite_info *)pdev->dev.platform_data;

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	if (info && info->dragonite_initialized)
		ipc_wrapper_link_unbind(IPC_DRG_FREERTOS_LINK_ID);
#endif
	devm_kfree(&pdev->dev, info);
	return 0;
}

static struct platform_driver dragonite_xcat_driver = {
	.probe		= dragonite_xcat_probe,
	.remove		= dragonite_xcat_remove,
	.driver		= {
		.owner	= THIS_MODULE,
		.name	= "dragonite_xcat",
		.of_match_table = of_match_ptr(mv_drag_dt_ids),
	},
};

static int __init dragonite_xcat_init(void)
{
	return platform_driver_register(&dragonite_xcat_driver);
}

static void __exit dragonite_xcat_deinit(void)
{
	platform_driver_unregister(&dragonite_xcat_driver);
}


module_init(dragonite_xcat_init);
module_exit(dragonite_xcat_deinit);
