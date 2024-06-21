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
* mv_cm3.h
*
* DESCRIPTION:
*	Cortex-M3 (POE) support for MSYS devices (BobK/BC2).
*
* DEPENDENCIES:
*
*******************************************************************************/

#ifndef __MV_CM3_H
#define __MV_CM3_H

#include <linux/wait.h>

#define PROC_MEM 0
#define PROC_SRAM 1

#define MV_CM3_NAME	"mv_cm3"

struct cm3_filep {
	unsigned int	mem_direction;
	loff_t		offset;
	char		*buf;
	size_t		count;
};

enum cm3_state {
	D_RESET,
	D_UNRESET,
	D_UNRESET_AND_PROTECT_MEM
};

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
struct cm3_ipcchn_set {
	unsigned int chn_id;
	unsigned int chn_en;
	char        *dev_name;
};
#endif

#define CM3_IOC_MAGIC 'c'
#define CM3_IOC_SETMEM_TYPE	_IOW(CM3_IOC_MAGIC, 0, unsigned int)
#define CM3_IOC_UNRESET		_IOW(CM3_IOC_MAGIC, 1, enum cm3_state)

#ifndef CONFIG_MV_CM3_IPC_SUPPORT
#define CM3_IOC_SENDIRQ		_IO(CM3_IOC_MAGIC, 2)
#endif

#define CM3_IOC_READ		_IOW(CM3_IOC_MAGIC, 3, struct cm3_filep)
#define CM3_IOC_WRITE		_IOW(CM3_IOC_MAGIC, 4, struct cm3_filep)
#ifdef CONFIG_MV_CM3_IPC_SUPPORT
#define CM3_IOC_SET_IPCCHN	_IOWR(CM3_IOC_MAGIC, 5, struct cm3_ipcchn_set)
#endif

#ifdef __KERNEL__
struct cm3_info {
	struct mutex		lock;
	struct platform_device	*pdev;

	/* proc mem */
	void __iomem		*proc_mem_base;
	size_t			proc_mem_size;

	/* proc sram */
	void __iomem		*proc_sram_base;
	size_t			proc_sram_size;
	phys_addr_t		proc_sram_phys;

	void __iomem		*ctrl_reg;
	void __iomem		*jtag_reg; /* called 'metal fix' in BobK
					      documentation */

/* XXX: remove whole switch irq connected part, when swic and mg_swic will be
 * supported in LSPv2.6
 */
	void			*poe_cause_irq_reg;
	void			*poe_mask_irq_reg;
	void			*poe_mg_cause_irq_reg;
	void			*poe_mg_mask_irq_reg;

	void			*host2poe_irq_reg;

	int			cm3_initialized;

	wait_queue_head_t	irq_queue;
	int			irq_get;
	uint32_t		mem_direction;
	struct miscdevice	miscdev;

	bool			mg_swic_support;
};

#ifdef CONFIG_MV_CM3_IPC_SUPPORT
int cm3_enable_ipc_channel(unsigned int chn_id, char *dev_name);
int cm3_disable_ipc_channel(unsigned int chn_id);
#endif

#endif /* __KERNEL__ */

#endif
