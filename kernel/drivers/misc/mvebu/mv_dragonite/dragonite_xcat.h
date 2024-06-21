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
* dragonite_xcat.h
*
* DESCRIPTION:
*	Dragonite (POE) support for AlleyCat3.
*
* DEPENDENCIES:
*
*******************************************************************************/

#ifndef __DRAGONITE_XCAT_H
#define __DRAGONITE_XCAT_H

#include <linux/wait.h>

#define ITCM_DIR 0
#define DTCM_DIR 1

#define _4K		0x00001000
#define _16K		0x00004000

#define IRQ_AURORA_PCIE0 58

#define DRAGONITE_PROTECTED_DTCM_SIZE_DEFAULT		_4K
#define DRAGONITE_SHARED_DTCM_SIZE_DEFAULT		(4*_16K)

#define DRAGONITE_EN_BIT		BIT(0)
#define DRAGONITE_CPU_EN_BIT		BIT(1)
#define DRAGONITE_CPU_GPP0FUNC		BIT(2)
#define DRAGONITE_CPU_INITRAM_BIT	BIT(3)

#define DFX_JTAG_ENABLE_VAL		(BIT(13) | BIT(14))
#define DFX_SPI_ENABLE_VAL		(BIT(9) | BIT(10))

#define DRAGONITE_POE_IRQ_BIT		BIT(1)

#define IRQ_POE_DRAGONITE		56
#define IRQ_DRAGONITE_MASK		(1 << (IRQ_POE_DRAGONITE - (CPU_INT_SOURCE_CONTROL_IRQ_OFFS + 1)))

#define POE_IRQ_CAUSE_OFFSET	(0x4)
#define POE_IRQ_MASK_OFFSET		(0x8)
#define POE_IRQ_HOST2POE_OFFSET	(0xC)


struct dragontie_filep {
	unsigned int mem_direction;
	loff_t	offset;
	char	*buf;
	size_t	count;
};

enum dragonite_state {
	D_RESET,
	D_UNRESET,
	D_UNRESET_AND_PROTECT_TCM
};

#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
struct dragonite_ipcchn_set {
	unsigned int chn_id;
	unsigned int chn_en;
	char        *dev_name;
};
#endif

#define DRAGONITE_IOC_MAGIC 'd'
#define DRAGONITE_IOC_SETMEM_TYPE	_IOW(DRAGONITE_IOC_MAGIC, 0, unsigned int)
#define DRAGONITE_IOC_UNRESET		_IOW(DRAGONITE_IOC_MAGIC, 1, enum dragonite_state)
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
#define DRAGONITE_IOC_SENDIRQ		_IO(DRAGONITE_IOC_MAGIC, 2)
#endif
#define DRAGONITE_IOC_READ		_IOW(DRAGONITE_IOC_MAGIC, 3, struct dragontie_filep)
#define DRAGONITE_IOC_WRITE		_IOW(DRAGONITE_IOC_MAGIC, 4, struct dragontie_filep)
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
#define DRAGONITE_IOC_SET_IPCCHN	_IOWR(DRAGONITE_IOC_MAGIC, 5, struct dragonite_ipcchn_set)
#endif

#define DRAGONITE_FW_CONFIG_MAGIC	0x46574346 /* 'FWCF' */
struct dragonite_fw_config {
	unsigned int	magic;
	unsigned int	ipc_shm_offset;	/* ipc share memory offset in dragonite DTCM */
	unsigned int	ipc_shm_size;	/* ipc share memory size*/
};


#ifdef __KERNEL__
struct dragonite_info {
	struct mutex		lock;
	struct platform_device	*pdev;

	void			*ctrl_reg;
	void			*jtag_reg;
	void			*poe_cause_irq_reg;
	void			*poe_mask_irq_reg;
	void			*host2poe_irq_reg;

	void			*poe_spi_enable_reg;

	void			*itcm_base;
	size_t			itcm_size;

	phys_addr_t		dtcm_phys;
	void			*dtcm_base;
	size_t			dtcm_size;

	/* The protected dtcm section is generally 4KB of DTCM starting at address 0 */
	/* this piece of memory is only used in protected mode */
	phys_addr_t		protected_dtcm_offset;
	size_t			protected_dtcm_size;

	void			*shmem_base;
	size_t			shmem_size;

	void			*msys_iregs_base;
	size_t			msys_iregs_size;

	int			dragonite_initialized;

	wait_queue_head_t	irq_queue;
	int			irq_get;
	uint32_t		mem_direction;
	struct miscdevice	miscdev;
};

int dragonite_xcat_pci_probe(struct dragonite_info *info_pci);
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
int dragonite_enable_ipc_channel(unsigned int chn_id, char *dev_name);
int dragonite_disable_ipc_channel(unsigned int chn_id);
#endif

#endif /* __KERNEL__ */

#endif /* __DRAGONITE_XCAT_H */
