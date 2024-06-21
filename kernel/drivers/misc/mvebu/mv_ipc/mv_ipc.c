/*
 * Copyright (C) 2014 Marvell
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation version 2 of the License.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/******************************************************************************
* mv_ipc.c
*
* DESCRIPTION:
*	IPC is a shared library (mv_ipc.c and mv_ipc.h) for both LSP and
*	FreeRTOS. It provides API which is used in user application/driver
*	(e.g. mv_ipc_demo.c). The IPC should not contain platform specific part.
*
* DEPENDENCIES:
*	All platform specific part should be hooked to the IPC library, by
*	filling some fields of struct ipc_link_info.
*
******************************************************************************/

#include "mv_ipc.h"

#undef MV_IPC_DEBUG

#ifdef MV_IPC_DEBUG
#define ATTACH_BACKOFF 1000
#define ATTACH_TIMEOUT 100
#define ipc_debug mv_os_printf
#else /* MV_IPC_DEBUG */
#define ATTACH_BACKOFF 10
#define ATTACH_TIMEOUT 10
#define ipc_debug(...)
#endif /* MV_IPC_DEBUG */

#define _4B_ALIGN 4

#define IPCERROR 1

/* The msg index 0 is reserved for the control msgs */
#define IPC_CTRL_MSG_IDX 0
#define IPC_HEAD_MSG_IDX 1

/* Main data structure - links array */
struct ipc_link mv_ipc_links[MV_IPC_LINKS_NUM];

struct ipc_link_info ipc_lnks_info[MV_IPC_LINKS_NUM];

bool ipc_is_master(unsigned int link_id)
{
	return ipc_lnks_info[link_id].is_master;
}

unsigned int ipc_get_remote_node_id(unsigned int link_id)
{
	return ipc_lnks_info[link_id].remote_node_id;
}

unsigned int ipc_who_am_i(unsigned int link_id)
{
	return mv_ipc_links[link_id].node_id;
}

void *get_shared_mem_offset(unsigned int link_id, void *virt_addr)
{
	int sh_mem_virt_addr = (int)ipc_lnks_info[link_id].shmem_virt_addr;

	/* check if virt_addr is from shared memory region */
	if (((int)virt_addr >= sh_mem_virt_addr) && ((int)virt_addr <
		 (int)(sh_mem_virt_addr + ipc_lnks_info[link_id].shmem_size)))
		return (void *)((int)virt_addr - sh_mem_virt_addr);
	else
		return NULL;
}

void *get_shared_mem_virt_addr(unsigned int link_id, void *sh_mem_offset)
{
	/* Check if sh_mem_offset not cross shared memory region */
	if ((unsigned int)sh_mem_offset < ipc_lnks_info[link_id].shmem_size)
		return (void *)((int)sh_mem_offset +
				(int)ipc_lnks_info[link_id].shmem_virt_addr);
	else
		return NULL;
}

/* This add base address to all addresses in link and channel structures */
static void ipc_channel_offset_fix(struct ipc_link *link, unsigned int base)
{
	unsigned int chn_idx;

	/* Fixup all offsets to shmem to local addresses */
	for (chn_idx = 0; chn_idx < link->num_of_chn; chn_idx++) {
		link->channels[chn_idx].rx_msg_que_va = (struct ipc_message *)
		   (base + (unsigned int)link->channels[chn_idx].rx_msg_que_va);
		link->channels[chn_idx].tx_msg_que_va = (struct ipc_message *)
		   (base + (unsigned int)link->channels[chn_idx].tx_msg_que_va);

		link->channels[chn_idx].rx_ctrl_msg =
					  link->channels[chn_idx].rx_msg_que_va;
		link->channels[chn_idx].tx_ctrl_msg =
					  link->channels[chn_idx].tx_msg_que_va;

		link->channels[chn_idx].tx_msg_flag += base;
		link->channels[chn_idx].rx_msg_flag += base;
	}

	link->tx_shared_heap_addr += base;
	link->rx_shared_heap_addr += base;
}

/* This routine read configuration from shared memory and fill local
 * structures with data configured by Master. Can be called from ipc_link_start
 * or postponed by and called from ipc_open_chn
 */
static int ipc_slave_config(unsigned int link_id, struct ipc_link *link)
{
	unsigned int chn_idx;
	unsigned int tmp_addr;
	struct ipc_channel_common chn_common;

	/* Override local parameters for link */
	link->node_id = MV_IPC_NODE_ID_SLAVE;
	link->shmem_base_addr = (int)ipc_lnks_info[link_id].shmem_virt_addr;
	link->remote_node_id = ipc_lnks_info[link_id].remote_node_id;
	link->channels =
		mv_os_malloc(sizeof(struct ipc_channel) * link->num_of_chn);
	if (!link->channels)
		return -IPCERROR;

	/* Swap rx and tx fields for Heap region partition */
	tmp_addr = link->tx_shared_heap_addr;
	link->tx_shared_heap_addr = link->rx_shared_heap_addr;
	link->rx_shared_heap_addr = tmp_addr;
	tmp_addr = link->tx_shared_heap_size;
	link->tx_shared_heap_size = link->rx_shared_heap_size;
	link->rx_shared_heap_size = tmp_addr;

	/* Initialize all channels */
	for (chn_idx = 0; chn_idx < link->num_of_chn; chn_idx++) {
		/* Read channel structure from shared mem */
		memcpy(&chn_common, (void *)(link->shmem_base_addr +
		       sizeof(*link) + (chn_idx * sizeof(chn_common))),
		       sizeof(chn_common));

		link->channels[chn_idx].state = MV_CHN_CLOSED;
		link->channels[chn_idx].tx_enable = false;
		link->channels[chn_idx].rx_enable = false;
		link->channels[chn_idx].next_rx_msg_idx = IPC_HEAD_MSG_IDX;
		link->channels[chn_idx].next_tx_msg_idx = IPC_HEAD_MSG_IDX;

		/* Channel common struct copy and swap TX and RX queue start */
		link->channels[chn_idx].rx_msg_que_va =
			chn_common.tx_msg_que_va;
		link->channels[chn_idx].tx_msg_que_va =
			chn_common.rx_msg_que_va;
		link->channels[chn_idx].msg_rxque_size =
			chn_common.msg_txque_size;
		link->channels[chn_idx].msg_txque_size =
			chn_common.msg_rxque_size;
		link->channels[chn_idx].rxbuf_size = chn_common.txbuf_size;
		link->channels[chn_idx].txbuf_size = chn_common.rxbuf_size;
		link->channels[chn_idx].tx_msg_flag = chn_common.rx_msg_flag;
		link->channels[chn_idx].rx_msg_flag = chn_common.tx_msg_flag;

		ipc_debug("IPC HAL: Init ch %d, RxQ = 0x%08x; TxQ = 0x%08x\n",
		   chn_idx, (unsigned int)link->channels[chn_idx].rx_msg_que_va,
		   (unsigned int)link->channels[chn_idx].tx_msg_que_va);

		/* Set rx and tx functions */
		link->channels[chn_idx].send_trigger =
			ipc_lnks_info[link_id].send_trigger;
		link->channels[chn_idx].enable_chn_doorbell =
			ipc_lnks_info[link_id].enable_chn_doorbell;
	}

	/* Fixup all offsets to shmem to local addresses */
	ipc_channel_offset_fix(link, link->shmem_base_addr);

	return 0;
}

/* Initialize the IPC mechanism. If called by the IPC master, then it
 * allocates required resources in the shared memory used for the IPC
 * communication. It also copies configuration to the shared memory and fills
 * local structures with data.
 * If called by slave, it calls subroutine for slave configuration or
 * postpones the slave init, depends on master state.
 * It is called from ipc_shmem_link_setup() after the ipc_lnks_info is
 * filled-out.
 */
static int ipc_link_start(unsigned int link_id)
{
	unsigned int chn_idx;
	struct ipc_link *link;
	struct ipc_channel_common chn_common;
	/* The runningOffset is offset in shared memory,
	 * used to compute addresses of queues and heap
	 */
	unsigned int runningOffset = 0, flagsOffset;
	unsigned int heapSize;
	int txbuf_size, rxbuf_size;
	int ret;

	/* Verify parameters */
	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: IPC Init: Bad link id %d\n", link_id);
		return -IPCERROR;
	}

	link = &mv_ipc_links[link_id];

	if (true == ipc_lnks_info[link_id].is_master) {
		/* Master configuration*/

		link->node_id = MV_IPC_NODE_ID_MASTER;
		link->shmem_base_addr =
			(int)ipc_lnks_info[link_id].shmem_virt_addr;
		link->shmem_size = ipc_lnks_info[link_id].shmem_size;
		link->num_of_chn = ipc_lnks_info[link_id].num_of_chn;
		link->remote_node_id = ipc_lnks_info[link_id].remote_node_id;
		link->channels =
		    mv_os_malloc(sizeof(struct ipc_channel) * link->num_of_chn);
		if (!link->channels)
			return -IPCERROR;

		/* Skip the control structures in Shared mem.
		 * Note: all pointers to shmem will be offsets, after control
		 * structures will be copied ti shmem, them will be fixed to
		 * addresses
		 * */
		runningOffset += sizeof(*link);
		runningOffset += sizeof(chn_common) * link->num_of_chn;
		/* Skip the RX/TX flags in Shared mem */
		flagsOffset = runningOffset;
		runningOffset += 2 * sizeof(unsigned int) * link->num_of_chn;

		/* Initialize all channels */
		for (chn_idx = 0; chn_idx < link->num_of_chn; chn_idx++) {
			txbuf_size =
			    ipc_lnks_info[link_id].ch_txbuf_info[chn_idx] * 4;
			rxbuf_size =
			    ipc_lnks_info[link_id].ch_rxbuf_info[chn_idx] * 4;

			link->channels[chn_idx].state = MV_CHN_CLOSED;
			link->channels[chn_idx].tx_enable = false;
			link->channels[chn_idx].rx_enable = false;
			link->channels[chn_idx].msg_rxque_size =
			    ipc_lnks_info[link_id].chn_rx_info_array[chn_idx];
			link->channels[chn_idx].msg_txque_size =
			    ipc_lnks_info[link_id].chn_tx_info_array[chn_idx];
			link->channels[chn_idx].rxbuf_size = rxbuf_size;
			link->channels[chn_idx].txbuf_size = txbuf_size;
			link->channels[chn_idx].next_rx_msg_idx =
				IPC_HEAD_MSG_IDX;
			link->channels[chn_idx].next_tx_msg_idx =
				IPC_HEAD_MSG_IDX;

			/*
			 * Set RX queue start and move offset to
			 * (queue size * message size)
			 */
			link->channels[chn_idx].rx_msg_que_va =
					    (struct ipc_message *)runningOffset;
			runningOffset +=
				link->channels[chn_idx].msg_rxque_size *
				      (sizeof(struct ipc_message) + rxbuf_size);

			/*
			 * Set TX queue start and move offset to
			 * (queue size * message size)
			 */
			link->channels[chn_idx].tx_msg_que_va =
					    (struct ipc_message *)runningOffset;
			runningOffset +=
				link->channels[chn_idx].msg_txque_size *
				      (sizeof(struct ipc_message) + txbuf_size);

			memset((void *)(link->shmem_base_addr +
			   (unsigned int)link->channels[chn_idx].rx_msg_que_va),
			   0, link->channels[chn_idx].msg_rxque_size *
			   (sizeof(struct ipc_message) + rxbuf_size));
			memset((void *)(link->shmem_base_addr +
			   (unsigned int)link->channels[chn_idx].tx_msg_que_va),
			   0, link->channels[chn_idx].msg_txque_size *
			   (sizeof(struct ipc_message) + txbuf_size));

			ipc_debug("IPC HAL: Init ch %d with RxQ %p; TxQ %p\n",
				 chn_idx, link->channels[chn_idx].rx_msg_que_va,
				 link->channels[chn_idx].tx_msg_que_va);

			ipc_debug("IPC HAL: RxQ size %d with %dB buffers ",
			    link->channels[chn_idx].msg_rxque_size, rxbuf_size);

			ipc_debug("TxQ size %d with %dB buffers,\n",
			    link->channels[chn_idx].msg_txque_size, txbuf_size);

			/* Set rx and tx functions */
			link->channels[chn_idx].send_trigger =
				ipc_lnks_info[link_id].send_trigger;
			link->channels[chn_idx].enable_chn_doorbell =
				ipc_lnks_info[link_id].enable_chn_doorbell;

			link->channels[chn_idx].tx_msg_flag = flagsOffset + 2 *
				chn_idx * sizeof(unsigned int);
			link->channels[chn_idx].rx_msg_flag = flagsOffset +
				(2 * chn_idx + 1) * sizeof(unsigned int);

			/* Prepare channel common struct */
			chn_common.rx_msg_que_va =
				link->channels[chn_idx].rx_msg_que_va;
			chn_common.tx_msg_que_va =
				link->channels[chn_idx].tx_msg_que_va;
			chn_common.msg_rxque_size =
				link->channels[chn_idx].msg_rxque_size;
			chn_common.msg_txque_size =
				link->channels[chn_idx].msg_txque_size;
			chn_common.rxbuf_size = rxbuf_size;
			chn_common.txbuf_size = txbuf_size;
			chn_common.tx_msg_flag =
				link->channels[chn_idx].tx_msg_flag;
			chn_common.rx_msg_flag =
				link->channels[chn_idx].rx_msg_flag;

			/* Check if we have enough shared mem to copy channel */
			if (runningOffset > (ipc_lnks_info[link_id].shmem_size -
					     sizeof(chn_common))) {
				ipc_debug("IPC HAL: Init channels allocated 0x%X bytes, shmem is 0x%X bytes\n",
					  runningOffset,
					  ipc_lnks_info[link_id].shmem_size);
				mv_os_free(link->channels);
				return -IPCERROR;
			}

			/* Copy channels first to shared mem */
			memcpy((void *)(link->shmem_base_addr + sizeof(*link) +
				(sizeof(chn_common) * chn_idx)),
				&chn_common, sizeof(chn_common));
		}

		/* Heap region partition */
		heapSize = ipc_lnks_info[link_id].shmem_size - runningOffset;
		link->tx_shared_heap_addr = runningOffset;
		link->tx_shared_heap_size = (heapSize *
			ipc_lnks_info[link_id].master_free_region_percent)/100;
		runningOffset += link->tx_shared_heap_size;
		link->rx_shared_heap_addr = runningOffset;
		link->rx_shared_heap_size =
			ipc_lnks_info[link_id].shmem_size - runningOffset;

		ipc_debug("run off %d 0x%x, sizeof chn_com %d, sizeof *lk %d\n",
			  runningOffset, runningOffset, sizeof(chn_common),
			  sizeof(*link));

		/* Set magic value in link structures and copy to shared memory,
		 * this is ready state for client
		 */
		link->master_conf_done = MV_IPC_MASTER_CONFIG_MAGIC;
		memcpy((void *)link->shmem_base_addr, link, sizeof(*link));

		/* Fix-up all offsets to shmem to local addresses */
		ipc_channel_offset_fix(link, link->shmem_base_addr);

		ipc_debug("IPC HAL: Initialized interface as Master\n");
	} else { /* Slave configuration */

		/* Read link structure from shared mem */
		memcpy((void *)link, ipc_lnks_info[link_id].shmem_virt_addr,
		       sizeof(*link));
		if (link->master_conf_done == MV_IPC_MASTER_CONFIG_MAGIC) {
			/* Master initialized - configure slave */
			ret = ipc_slave_config(link_id, link);
			if (ret) {
				mv_os_printf("Ipc slave config failed\n");
				return ret;
			}

			/* Clear magic */
			link->master_conf_done = 0;
			memcpy((void *)link->shmem_base_addr, link,
			       sizeof(*link));
			link->slave_link_initialized = 0;

			ipc_debug("IPC HAL: Initialized interface as Slave\n");
		} else {
			/* Postpone the Slave init, will be done in
			 * ipc_open_chn
			 */
			link->slave_link_initialized =
				MV_IPC_MASTER_CONFIG_MAGIC;
			ipc_debug("IPC HAL: Initialized interface as Slave, ");
			ipc_debug("config postponed\n");
		}
	}

	return 0;
}

static int ipc_msg_offset(unsigned int msg_idx, unsigned int buf_size)
{
	return msg_idx * (sizeof(struct ipc_message) + buf_size);
}

/* Get pointer of a channel basing on link id and channel id */
static struct ipc_channel *get_chn_pointer(unsigned int link_id,
							    unsigned int chn_id)
{
	struct ipc_link *link;

	/* Verify parameters */
	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: Ack attach: Bad link id %d\n", chn_id);
		return NULL;
	}

	link = &mv_ipc_links[link_id];

	if (chn_id >= link->num_of_chn) {
		mv_os_printf("IPC ERROR:Ack attach: Bad chan id %d\n", chn_id);
		return NULL;
	}

	return &link->channels[chn_id];
}

/* Get pointer to the next free tx buffer */
char *ipc_getnext_txbuf(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;
	unsigned int msg_offset;
	struct ipc_message *curr_msg;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return NULL;

	msg_offset = ipc_msg_offset(chn->next_tx_msg_idx, chn->txbuf_size);

	curr_msg = (struct ipc_message *)((int)chn->tx_msg_que_va + msg_offset);

	if (mvU32Ipc2Host(curr_msg->is_used) != false) {
		ipc_debug("IPC HAL: No free Tx buff. Msg %d used flag = %d\n",
			chn->next_tx_msg_idx, curr_msg->is_used);
		return NULL;
	}

	return (char *)((int)curr_msg + sizeof(struct ipc_message));
}

/* Calculate pointer of rx buffer */
char *ipc_get_rxbuf_of_msg(struct ipc_message *msg)
{
	return (char *)msg + sizeof(struct ipc_message);
}

/* Get tx buffer size of particular channel */
int ipc_chn_txbuf_max_size(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	return chn->txbuf_size;
}

/* Closes an ipc channel and prepares it for receiving messages */
int ipc_close(unsigned int link_id)
{
	struct ipc_link *link;
	unsigned int chn_idx;

	/* Verify parameters */
	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: IPC close: Bad link id %d\n", link_id);
		return -IPCERROR;
	}

	link = &mv_ipc_links[link_id];

	/* De-activate all channels */
	for (chn_idx = 0; chn_idx < link->num_of_chn; chn_idx++) {
		ipc_disable_chn_rx(link_id, chn_idx);

		if (link->channels[chn_idx].state == MV_CHN_ATTACHED)
			ipc_dettach_chn(link_id, chn_idx);

		if (link->channels[chn_idx].state == MV_CHN_OPEN)
			ipc_close_chn(link_id, chn_idx);
	}

	ipc_debug("IPC HAL: Closed IPC interface\n");

	return 0;
}

/* Opens an ipc channel and prepares it for receiving messages */
int ipc_open_chn(unsigned int link_id, unsigned int chn_id, void *dev,
	 MV_IPC_RX_CLBK rx_clbk, MV_IPC_SET_CHN_STAT set_chn_status)
{
	struct ipc_link *link;
	struct ipc_channel *chn;
	struct ipc_message *curr_msg;
	unsigned int msg_id, msg_offset;
	int status;

	/* Verify parameters */
	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: Open Chn: Bad link id %d\n", chn_id);
		return -IPCERROR;
	}

	link = &mv_ipc_links[link_id];

	/* Check if postponed Slave init needed */
	if (link->slave_link_initialized == MV_IPC_MASTER_CONFIG_MAGIC) {
		/* Read link structure from shared mem */
		memcpy((void *)link, ipc_lnks_info[link_id].shmem_virt_addr,
		       sizeof(*link));
		if (link->master_conf_done == MV_IPC_MASTER_CONFIG_MAGIC) {
			/* Master initialized - configure slave */
			status = ipc_slave_config(link_id, link);
			if (status) {
				mv_os_printf("Ipc slave config failed\n");
				return status;
			}
			mv_os_printf("IPC MESSG: %s: Postponed init done\n",
				     __func__);

			/* Clear magic */
			link->master_conf_done = 0;
			memcpy((void *)link->shmem_base_addr, link,
				sizeof(*link));
			link->slave_link_initialized = 0;
		} else {
			/* Master still not wake, cannot open the channel */
			mv_os_printf("IPC WARNG: Open Chn: Master not ready\n");
			link->slave_link_initialized =
						     MV_IPC_MASTER_CONFIG_MAGIC;
			return MV_NOT_STARTED;
		}
	}

	if (chn_id >= link->num_of_chn) {
		mv_os_printf("IPC ERROR: Open Chn: Bad chan id %d\n", chn_id);
		return -IPCERROR;
	}

	chn = &link->channels[chn_id];

	if (chn->state != MV_CHN_CLOSED) {
		mv_os_printf("IPC ERROR: chan %d is already opened %d\n",
			     chn_id, chn->state);
		return -IPCERROR;
	}

	/* Initialize the transmit queue */
	for (msg_id = 0; msg_id < chn->msg_txque_size; msg_id++) {
		msg_offset = ipc_msg_offset(msg_id, chn->txbuf_size);
		curr_msg =
		   (struct ipc_message *)((int)chn->tx_msg_que_va + msg_offset);
		curr_msg->is_used = mvU32Host2Ipc(false);
	}

	/* Initialize channel members */
	chn->state = MV_CHN_OPEN;
	chn->next_rx_msg_idx = IPC_HEAD_MSG_IDX;
	chn->next_tx_msg_idx = IPC_HEAD_MSG_IDX;
	chn->rx_enable = true;
	chn->rx_callback = rx_clbk;
	chn->set_chn_status = set_chn_status;
	chn->private_data = dev;

	ipc_debug("IPC HAL: Opened channel %d successfully\n", chn_id);

	return 0;
}

/* Acknowledges and Attach request from receiver. */
static int ipc_ack_attach(unsigned int link_id, unsigned int chn_id,
			  int acknowledge)
{
	struct ipc_link *link;
	struct ipc_channel *chn;
	struct ipc_message attach_msg;
	int status;

	/* Verify parameters */
	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: Ack attach: Bad link id %d\n", chn_id);
		return -IPCERROR;
	}

	link = &mv_ipc_links[link_id];

	if (chn_id >= link->num_of_chn) {
		mv_os_printf("IPC ERROR:Ack attach: Bad chan id %d\n", chn_id);
		return -IPCERROR;
	}

	chn = &link->channels[chn_id];

	/* Cannot acknowledge remote attach until local attach was requested */
	if ((chn->state != MV_CHN_ATTACHED) && (chn->state != MV_CHN_LINKING)) {
		ipc_debug("IPC HAL: Can't ack attach. Channel in state %d\n",
			  chn->state);
		return -IPCERROR;
	}

	if (acknowledge == true) {
		/* Check that channel is not already coupled to another CPU */
		if (chn->remote_node_id != link->remote_node_id) {
			ipc_debug("IPC HAL: Can't ack attach. CPU %d != %d\n",
				  chn->remote_node_id, link->remote_node_id);
			return -IPCERROR;
		}

		ipc_debug("IPC HAL: Acknowledging attach from CPU %d\n",
			  link->remote_node_id);

		/* Send the attach acknowledge message */
		attach_msg.type = IPC_MSG_ATTACH_ACK;
		attach_msg.value = link->remote_node_id;
		attach_msg.size = 0;
		status = ipc_tx_ctrl_msg(link_id, chn_id, &attach_msg);
		if (status != 0) {
			mv_os_printf("IPC ERROR: Cannot Send attach ack msg\n");
			return -IPCERROR;
		}
	}

	/* Now change my own state to attached */
	chn->state = MV_CHN_ATTACHED;

	if (chn->set_chn_status)
		chn->set_chn_status(1, chn->private_data);

	ipc_debug("IPC HAL: Attached channel %d\n", chn_id);

	return 0;
}

/* Acknowledges detach request from receiver. This closes the channel for
 * transmission and resets the queues
 */
static int ipc_ack_detach(unsigned int link_id, unsigned int chn_id,
								int acknowledge)
{
	struct ipc_channel *chn;
	struct ipc_message dettach_msg;
	int status;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	/* Cannot acknowledge remote detach until local attach was requested */
	if ((chn->state != MV_CHN_ATTACHED) &&
					     (chn->state != MV_CHN_UNLINKING)) {
		ipc_debug("IPC HAL: Can't ack detach. Channel in state %d\n",
			  chn->state);
		return -IPCERROR;
	}

	if (acknowledge == true) {
		/* Send the attach acknowledge message */
		dettach_msg.type = IPC_MSG_DETACH_ACK;
		dettach_msg.size = 0;
		dettach_msg.value = 0;

		status = ipc_tx_ctrl_msg(link_id, chn_id, &dettach_msg);
		if (status != 0) {
			mv_os_printf("IPC ERROR: Cannot Send detach ack msg\n");
			return -IPCERROR;
		}
	}

	/* Now change my own state to linking */
	chn->state = MV_CHN_LINKING;

	if (chn->set_chn_status)
		chn->set_chn_status(0, chn->private_data);

	ipc_debug("IPC HAL: Acknowledging detach message\n");

	return 0;
}

/* Ask receiver to acknowledge attach request. To verify reception, message
 * transmission is possible only after receiver acknowledges the attach
 */
static int ipc_req_attach(unsigned int link_id, struct ipc_channel *chn,
							    unsigned int chn_id)
{
	struct ipc_message attach_msg;
	int status;
	int backoff = ATTACH_BACKOFF, timeout = ATTACH_TIMEOUT;

	ipc_debug("IPC HAL: Requesting attach from cpu %d\n",
		  chn->remote_node_id);

	/* Send the attach message */
	attach_msg.type = IPC_MSG_ATTACH_REQ;
	attach_msg.value = ipc_who_am_i(link_id);
	status = ipc_tx_ctrl_msg(link_id, chn_id, &attach_msg);
	if (status != 0) {
		mv_os_printf("IPC ERROR: Cannot Send attach req message\n");
		return -IPCERROR;
	}

	/* Give the receiver 100us to reply (or 100ms if debugs enabled) */
	while ((chn->state != MV_CHN_ATTACHED) && timeout) {
		usleep_range(backoff, 2 * backoff);
		timeout--;
	}

	if (chn->state != MV_CHN_ATTACHED) {
		ipc_debug("IPC ERROR: %s: no reply from receiver after %d us\n",
			  __func__, ATTACH_BACKOFF * ATTACH_TIMEOUT);
		return -IPCERROR;
	}

	ipc_debug("IPC HAL: Attached channel %d\n", chn_id);

	return 0;
}

/* Attempts to attach the TX queue to a remote CPU by sending a ATTACH ACK
 * messages to receiver. If the message is acknowledged, the channel state
 * becomes attached and message transmission is enabled.
 */
int ipc_attach_chn(unsigned int link_id, unsigned int chn_id,
				     unsigned int remote_node_id, int *attached)
{
	struct ipc_channel *chn;
	struct ipc_message *curr_msg;
	unsigned int msg_id, msg_offset;
	int status;

	(*attached) = false;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->state == MV_CHN_CLOSED) {
		mv_os_printf("IPC ERROR: Can't attach chan %d. It is closed\n",
			     chn_id);
		return -IPCERROR;
	}

	if (chn->state == MV_CHN_ATTACHED) {
		(*attached) = true;
		return 0;
	}

	chn->state = MV_CHN_LINKING;
	chn->remote_node_id = remote_node_id;
	chn->tx_enable = true;

	/* Initialize the transmit queue */
	for (msg_id = 0; msg_id < chn->msg_txque_size; msg_id++) {
		msg_offset = ipc_msg_offset(msg_id, chn->txbuf_size);
		curr_msg =
		   (struct ipc_message *)((int)chn->tx_msg_que_va + msg_offset);
		curr_msg->is_used = mvU32Host2Ipc(false);
	}

	/* Send req for attach to other side */
	status = ipc_req_attach(link_id, chn, chn_id);
	if (status == 0) {
		(*attached) = true;
		ipc_debug("IPC HAL: Attached channel %d to link %d\n",
			   chn_id, link_id);
	}

	return 0;
}

/* Detaches the channel from remote CPU. It notifies remote CPU, by sending
 * control message and waits for acknowledge. After calling this function data
 * messages cannot be sent anymore.
 */
int ipc_dettach_chn(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;
	struct ipc_message msg;
	int status;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->state != MV_CHN_ATTACHED) {
		mv_os_printf("IPC ERROR: Detach: channel %d is not attached\n",
			     chn_id);
		return -IPCERROR;
	}

	msg.type = IPC_MSG_DETACH_REQ;
	msg.size = 0;
	msg.value = 0;

	status = ipc_tx_ctrl_msg(link_id, chn_id, &msg);
	if (status != 0) {
		mv_os_printf("IPC ERROR: Cannot Send detach request message\n");
		return -IPCERROR;
	}

	chn->remote_node_id = 0;
	chn->state = MV_CHN_UNLINKING;

	return 0;
}

/* Closes the IPC channels. This disables the channels ability to receive msg */
int ipc_close_chn(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->state == MV_CHN_CLOSED) {
		mv_os_printf("IPC ERROR: Channel %d is already closed\n",
			     chn_id);
		return -IPCERROR;
	}

	chn->state = MV_CHN_CLOSED;
	chn->tx_enable = false;
	chn->rx_enable = false;
	chn->remote_node_id = 0;

	ipc_debug("IPC HAL: Closed channel %d successfully\n", chn_id);

	return 0;
}

/* Checks if the channel is ready to transmit */
bool ipc_tx_ready(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;
	struct ipc_message *curr_msg;
	unsigned int msg_offset;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return false;

	if (chn->state != MV_CHN_ATTACHED) {
		mv_os_printf("IPC ERROR: %s: chan not attached, state is %d\n",
			     __func__, chn->state);
		return false;
	}

	/* If next message is used by receiver it means a full queue or a bug */
	msg_offset = ipc_msg_offset(chn->next_tx_msg_idx, chn->txbuf_size);
	curr_msg = (struct ipc_message *)((int)chn->tx_msg_que_va + msg_offset);
	if (mvU32Ipc2Host(curr_msg->is_used) != false) {
		ipc_debug("IPC HAL: %s: Can't send, Msg %d used flag = %d\n",
			  __func__, chn->next_tx_msg_idx, curr_msg->is_used);
		return false;
	}

	return true;
}

/*
 * Sends a control message to other side. These messages are not forwarded to
 * the user
 */
int ipc_tx_ctrl_msg(unsigned int link_id, unsigned int chn_id,
						     struct ipc_message *in_msg)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->tx_enable == false) {
		mv_os_printf("IPC ERROR: Tx Ctrl msg: Tx not enabled\n");
		return -IPCERROR;
	}

	/* Write the message and pass */
	chn->tx_ctrl_msg->type = mvU32Host2Ipc(in_msg->type);
	chn->tx_ctrl_msg->size = mvU32Host2Ipc(in_msg->size);
	chn->tx_ctrl_msg->value = mvU32Host2Ipc(in_msg->value);

	/* Make sure the msg values are written before the used flag
	 * to ensure the polling receiver will get valid message once
	 * it detects is_used == true.
	 */
	dmb();

	chn->tx_ctrl_msg->is_used = mvU32Host2Ipc(true);

	ipc_debug("IPC HAL: Sent control message 0x%8x on chan %d to link %d\n",
			(int)chn->tx_ctrl_msg, chn_id, link_id);

	/* Raise the TX ready flag and send the trigger */
	*((unsigned int *)chn->tx_msg_flag) = mvU32Host2Ipc(true);

	/* If the send_trigger function registered, call it */
	if (chn->send_trigger != NULL)
		chn->send_trigger(chn->remote_node_id, chn_id,
				  ipc_lnks_info[link_id].dev_id);

	return 0;
}

/* Main transmit function */
int ipc_tx_msg(unsigned int link_id, unsigned int chn_id,
	       struct ipc_message *in_msg)
{
	struct ipc_channel *chn;
	struct ipc_message *curr_msg;
	unsigned int msg_offset;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	/* Test if TX ready to send */
	if (chn->state != MV_CHN_ATTACHED) {
		mv_os_printf("IPC ERROR: %s: chan not attached, state is %d\n",
			     __func__, chn->state);
		return -IPCERROR;
	}

	msg_offset = ipc_msg_offset(chn->next_tx_msg_idx, chn->txbuf_size);
	curr_msg = (struct ipc_message *)((int)chn->tx_msg_que_va + msg_offset);
	/* If next message is used by receiver it means a full queue or a bug */
	if (mvU32Ipc2Host(curr_msg->is_used) != false) {
		ipc_debug("IPC HAL: %s: Can't send, Msg %d used flag = %d\n",
			  __func__, chn->next_tx_msg_idx, curr_msg->is_used);
		return -IPCERROR;
	}

	/* Write the message */
	curr_msg->type = mvU32Host2Ipc(in_msg->type);
	curr_msg->size = mvU32Host2Ipc(in_msg->size);
	curr_msg->value = mvU32Host2Ipc(in_msg->value);

	/* Make sure the msg values are written before the used flag
	 * to ensure the polling receiver will get valid message once
	 * it detects is_used == true.
	 */
	dmb();

	/* Pass ownership to remote cpu */
	curr_msg->is_used = mvU32Host2Ipc(true);

	chn->next_tx_msg_idx++;
	if (chn->next_tx_msg_idx == chn->msg_txque_size)
		chn->next_tx_msg_idx = IPC_HEAD_MSG_IDX;

	ipc_debug("IPC HAL: Sent message %d on channel %d to link %d\n",
			chn->next_tx_msg_idx - 1, chn_id, link_id);

	/* Raise the TX ready flag and send the trigger */
	*((unsigned int *)chn->tx_msg_flag) = mvU32Host2Ipc(true);

	/* There is no need to register send_trigger */
	if (chn->send_trigger != NULL)
		chn->send_trigger(chn->remote_node_id, chn_id,
				  ipc_lnks_info[link_id].dev_id);

	return 0;
}

/*
 * This routine initializes IPC channel: setup receive queue and enable data
 * receiving
 */
static int ipc_rx_ctrl_msg(unsigned int link_id, unsigned int chn_id,
			   struct ipc_message *msg)
{
	int ret = 0;

	ipc_debug("IPC HAL: Processing control message %d\n", msg->type);

	switch (mvU32Ipc2Host(msg->type)) {
	case IPC_MSG_ATTACH_REQ:
		ret = ipc_ack_attach(link_id, chn_id, true);
		break;

	case IPC_MSG_ATTACH_ACK:
		ret = ipc_ack_attach(link_id, chn_id, false);
		break;

	case IPC_MSG_DETACH_REQ:
		ret = ipc_ack_detach(link_id, chn_id, true);
		break;

	case IPC_MSG_DETACH_ACK:
		ret = ipc_ack_detach(link_id, chn_id, false);
		break;

	default:
		ipc_debug("IPC HAL: Unknown internal message type %d\n",
			  msg->type);
	}

	if (ret)
		mv_os_printf("IPC ERROR: %s: processing ctrl msg failed\n",
			      __func__);

	ret = ipc_release_msg(link_id, chn_id, msg);
	if (ret) {
		mv_os_printf("IPC: %s: release msg failed\n", __func__);
		return ret;
	}

	return 0;
}

/* Masks the given channel in ISR */
void ipc_disable_chn_rx(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return;

	if (chn->enable_chn_doorbell)
		chn->enable_chn_doorbell(link_id, chn_id, false,
					 ipc_lnks_info[link_id].dev_id);

	ipc_debug("IPC HAL: Disabled ISR for link %d, channel %d\n",
		  link_id, chn_id);
	return;
}

/* Unmasks the given channel in ISR */
void ipc_enable_chn_rx(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return;

	if (chn->enable_chn_doorbell)
		chn->enable_chn_doorbell(link_id, chn_id, true,
					 ipc_lnks_info[link_id].dev_id);

	ipc_debug("IPC HAL: Enabled ISR for link %d, channel %d\n",
		  link_id, chn_id);
	return;
}

/* Main Rx routine.
 * With usage of ipc_rx_msg function two paths of rx processing are allowed:
 * 1) if user callback (rx_callback) registered, process the msg using the
 * user callback.
 * 2) if user callback not registered, IPC layer will copy the msg buffer
 * to the indicated address (*msg_data).
 * User should be aware that 2) option requires memcpy, while using 1) one,
 * memcpy is not a must - user can operate on msg buffer, which is placed in
 * shared memory.
 *
 * The ipc layer is responsible for releasing msgs: after calling rx_callback
 * or after copying msg buffer to the indicated address.
 */
int ipc_rx_msg(unsigned int link_id, unsigned int chn_id,
	       char *msg_data, size_t msg_size)
{
	struct ipc_channel *chn;
	struct ipc_message *curr_msg;
	unsigned int msg_offset;
	int ret;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->state == MV_CHN_CLOSED)
		return -IPCERROR;

	/* First process control messages like attach, detach, close */
	if (mvU32Ipc2Host(chn->rx_ctrl_msg->is_used) == true) {
		ret = ipc_rx_ctrl_msg(link_id, chn_id, chn->rx_ctrl_msg);
		if (ret)
			mv_os_printf("IPC ERROR: %s: process rx msg failed\n",
				     __func__);
		return ret;
	}

	msg_offset = ipc_msg_offset(chn->next_rx_msg_idx, chn->rxbuf_size);
	curr_msg = (struct ipc_message *)((int)chn->rx_msg_que_va + msg_offset);

	/* Check for unread data messages in queue */
	if (mvU32Ipc2Host(curr_msg->is_used) != true) {
		/* No more messages, disable RX ready flag */
		*((unsigned int *)chn->rx_msg_flag) = mvU32Host2Ipc(false);
		return MV_NO_MORE;
	}

	/* Increment msg idx to keep in sync with sender */
	chn->next_rx_msg_idx++;
	if (chn->next_rx_msg_idx == chn->msg_rxque_size)
		chn->next_rx_msg_idx = IPC_HEAD_MSG_IDX;

	/* Check if channel is ready to receive messages */
	if (chn->state < MV_CHN_OPEN) {
		mv_os_printf("IPC ERROR: Rx msg: Chan not ready, state = %d\n",
			     chn->state);
		return -IPCERROR;
	}

	/* Now process user messages */
	ipc_debug("IPC HAL: Received message %d on channel %d\n",
			chn->next_rx_msg_idx - 1, chn_id);

	/* Call user function to care the message, if function missing, copy
	 * data to the msg_data pointer. Please note that the 1 solution (call
	 * rx_callback) msg can avoid copying */
	if (chn->rx_callback)
		chn->rx_callback(curr_msg, chn->private_data);
	else {
		if (!msg_data) {
			mv_os_printf("IPC ERROR: %s: msg_data missing\n",
				     __func__);
			return -IPCERROR;
		}

		memcpy(msg_data, ipc_get_rxbuf_of_msg(curr_msg),
		       min(curr_msg->size, msg_size));

		ipc_debug("IPC HAL: %dB copied to the msg_data, from %dB msg\n",
			   msg_size, curr_msg->size);
	}

	ret = ipc_release_msg(link_id, chn_id, curr_msg);
	if (ret != 0) {
		mv_os_printf("IPC: %s: release msg failed\n", __func__);
		return ret;
	}

	return 0;
}

/* Check if RX flag raised */
bool ipc_rx_msg_flag_check(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return false;

	if (chn->state == MV_CHN_CLOSED)
		return false;

	if (mvU32Ipc2Host(*((unsigned int *)chn->rx_msg_flag)) != true)
		return false;

	return true;
}

/* Return ownership on message to transmitter */
int ipc_release_msg(unsigned int link_id, unsigned int chn_id,
							struct ipc_message *msg)
{
	struct ipc_channel *chn;

	chn = get_chn_pointer(link_id, chn_id);
	if (!chn)
		return -IPCERROR;

	if (chn->state == MV_CHN_CLOSED) {
		mv_os_printf("IPC ERROR: Msg release: Inactive channel id %d\n",
			     chn_id);
		return -IPCERROR;
	}

	if (mvU32Ipc2Host(msg->is_used == false)) {
		mv_os_printf("IPC ERROR: Msg release: Msg %d owned by %d\n",
			chn->next_rx_msg_idx, msg->is_used);
		return -IPCERROR;
	}

	msg->is_used = mvU32Host2Ipc(false);

	ipc_debug("IPC HAL: Released message 0x%8x on channel %d\n", (int)msg,
		  chn_id);

	return 0;
}

/* Malloc buffer in shared memory heap for TX buffers
 * (Sequential malloc, no free allowed)
 */
void *ipc_sh_malloc(unsigned int link_id, unsigned int size)
{
	struct ipc_link *link;
	void *ptr;

	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: Tx Msg: Bad link id %d\n", link_id);
		return NULL;
	}

	link = &mv_ipc_links[link_id];

	if (size > link->tx_shared_heap_size)
		return NULL;

	ptr = (void *)link->tx_shared_heap_addr;

	link->tx_shared_heap_addr += size;
	link->tx_shared_heap_size -= size;

	return ptr;
}

/* Process all Rx msg in all links and channels */
void ipc_rx_process(unsigned int link_id, char *enabled_chn,
		    char *msg_data, size_t msg_size)
{
	unsigned int chn_id;
	int read_msgs = IPC_RX_MAX_MSGS_PER_ISR;

	/* NOTE: This ISR may be customised by user application requirements to
	 * make it more efficient
	 */

	/* Scan all rx flags */
	for (chn_id = 0; chn_id < ipc_lnks_info[link_id].num_of_chn; chn_id++) {
		/* Check if RX flag raised */
		if ((enabled_chn[chn_id] == 1) &&
		    (ipc_rx_msg_flag_check(link_id, chn_id) == true)) {
			/* If ready to RX, start get the messages */
			ipc_debug("Got message in channel %d\n", chn_id);
			while (read_msgs) {
				if (ipc_rx_msg(link_id, chn_id, msg_data,
				    msg_size) != 0)
					break;
				read_msgs--;
			}
		}
	}
}

/* Process Rx msg basing on link id and channel id */
int ipc_rx_queue_process(unsigned int link_id, unsigned int chn_id,
			 char *msg_data, size_t msg_size)
{
	if (ipc_rx_msg_flag_check(link_id, chn_id) == true) {
		/* If ready to RX, start get the messages */
		ipc_debug("Got message in channel %d\n", chn_id);
		if (ipc_rx_msg(link_id, chn_id, msg_data, msg_size) != 0)
			return -IPCERROR;
		else
		/* Return 1 on receiving the new message
		   successfully. */
			return 1;
	}

	/* If the channel is not in illigal status
	   or there is no pending messages in the
	   rx queue, return '0' sliencely */
	return 0;
}

/* Send 'data' using link id and channel id. The 'data' will be copy to the next
 * free tx buffer, required fields of msg header will be filled and ipc_tx_msg
 * subroutine will be called
 */
int ipc_tx_queue_send(unsigned int link_id, unsigned int chn_id,
					  const char *data, unsigned int length)
{
	int status, txbuf_size;
	struct ipc_message msg;

	char *tx_buf = ipc_getnext_txbuf(link_id, chn_id);
	if (!tx_buf) {
		mv_os_printf("IPC ERROR: No free tx_buf in link %d, chn %d.\n",
			link_id, chn_id);
		return -IPCERROR;
	}

	txbuf_size = ipc_chn_txbuf_max_size(link_id, chn_id);
	if (txbuf_size < 0) {
		mv_os_printf("IPC ERROR: unable to get txbuf max size in link %d, chn %d.\n",
			     link_id, chn_id);
		return -IPCERROR;
	}
	if (length > (unsigned int)txbuf_size) {
		mv_os_printf("IPC ERROR: TX length %d beyond TX Buffer size %d.\n",
			     length, txbuf_size);
		return -IPCERROR;
	}

	memcpy(tx_buf, data, roundup(length, _4B_ALIGN));

	msg.type = 0;
	msg.size = length;
	msg.value = ipc_who_am_i(link_id);

	ipc_debug("IPC: msg prepare to send in buf %p(shmem offset), sh_virt_addr %p\n",
		  get_shared_mem_offset(link_id, tx_buf), tx_buf);

	status = ipc_tx_msg(link_id, chn_id, &msg);

	if (status == -IPCERROR)
		mv_os_printf("IPC ERROR: TX queue busy\n");

	return status;
}

/* Fill the local structures basing on link_info data, call platform-specific
 * irq_init subroutine and then initialize link by calling ipc_link_start
 */
int ipc_shmem_link_setup(unsigned char link_id, struct ipc_link_info *link_info)
{
	int ret;

	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: %s: Bad link id %d\n", __func__,
			     link_id);
		return -IPCERROR;
	}

	ipc_lnks_info[link_id] = *link_info;

	if (link_info->irq_init) {
		ret = link_info->irq_init(link_id, link_info->dev_id);
		if (ret != 0) {
			mv_os_printf("IPC: IPC HAL init failed (irq init)\n");
			return ret;
		}
	}

	ret = ipc_link_start(link_id);
	if (ret != 0) {
		mv_os_printf("IPC: IPC HAL initialization failed\n");
		return ret;
	}

	return 0;
}

int ipc_shmem_link_cleanup(unsigned char link_id)
{
	struct ipc_link *link;

	if (link_id >= MV_IPC_LINKS_NUM) {
		mv_os_printf("IPC ERROR: %s: Bad l_id %d\n", __func__, link_id);
		return -IPCERROR;
	}

	link = &mv_ipc_links[link_id];

	mv_os_free(link->channels);

	return 0;
}

