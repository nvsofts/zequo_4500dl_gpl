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
/********************************************************************************
* mv_ipc_demo.c
*
* DESCRIPTION:
*	The IPC DEMO is provided to demonstration the IPC library usage, but
*	this is only an example. When IPC DEMO is used on both IPC master and
*	slave platforms, it establish IPC tunnel on IPC_DEMO_LINK_ID, and
*	IPC_DEMO_CHANNEL_ID. In this example, ipc slave indicate message
*	reception by responding to the master: "Received message from remote
*	node: " together with contents, gathered from ipc master.
*
* DEPENDENCIES:
*	The IPC DEMO can be used only together with the IPC library.
*
******************************************************************************/
#include "mv_ipc.h"

#undef MV_IPCDEMO_DEBUG

#ifdef MV_IPCDEMO_DEBUG
#define mv_ipcdbg_printf mv_os_printf
#else
#define mv_ipcdbg_printf(x ...)
#endif /* MV_IPCDEMO_DEBUG */

struct mv_ipc_demo_chn {
	unsigned int link;
	unsigned int chn;
};

static struct mv_ipc_demo_chn chn_info;

#define rc_msg "ECHO:"
/* Demo receive function: called from IPC driver on msg arrival */
static int ipc_demo_rx(struct ipc_message *msg, void *dev)
{
	char *ptr_virt;
	int res_pos, txbuf_size, resp_len;
	char response[IPC_MAX_MSG_SIZE] = rc_msg;
	struct mv_ipc_demo_chn *info = dev;

	ptr_virt = ipc_get_rxbuf_of_msg(msg);
	if (!ptr_virt)
		mv_os_printf("IPC DEMO: Unable to get rxbuf of msg %p\n", msg);

	mv_ipcdbg_printf("IPC DEMO: addr of received msg buf %p(shmem offset), ptr_virt %p\n",
		get_shared_mem_offset(info->link, ptr_virt), ptr_virt);

	mv_os_printf("IPC DEMO: Message from CPU%d \"%s\"\n", mvU32Ipc2Host(msg->value), ptr_virt);

	/* Only slave sends resonse */
	if (ipc_is_master(info->link))
		return 0;

	res_pos = sizeof(rc_msg)/sizeof(char) - 1;

	txbuf_size = ipc_chn_txbuf_max_size(info->link, info->chn);

	/* Assume demo message is pure string. */
	resp_len = msg->size + res_pos;
	if (resp_len > txbuf_size)
		resp_len = txbuf_size;

	if (resp_len) {
		strncpy(&response[res_pos], ptr_virt, resp_len - res_pos);
		response[resp_len - 1] = 0;

		ipc_tx_queue_send(info->link, info->chn, response, resp_len);
	} else
		mv_os_printf("IPC DEMO: Unable to echo received msg on RX Only channel\n");

	return 0;
}

/* To demonstrate the IPC library usage, open/attach/enable one
 * channel(IPC_DEMO_CHANNEL_ID) in particular link(IPC_DEMO_LINK_ID). Also
 * register ipc_demo_rx subroutine, which will be used as a user call-back in
 * the Rx path of IPC layer (ipc_rx_msg).
 */
int ipc_demo_init(unsigned int link, unsigned int chn)
{
	int status;
	int attached;

	chn_info.link = link;
	chn_info.chn  = chn;

	/* Initialize IPC driver */
	status = ipc_open_chn(link, chn, &chn_info, ipc_demo_rx, NULL);
	if (status != 0) {
		mv_os_printf("IPC DEMO: Failed to open IPC channel %d", chn);
		return -1;
	}

	status = ipc_attach_chn(link, chn,
			  ipc_get_remote_node_id(link), &attached);
	/* Don't need to confirm the "attached" during the init, the ipc implementation allows that.
	In case the MSYS is up and CM3(or other CPU) is down. The msys will try to attached, but it
	will not be able because the CM3 is not up yet*/
	if (status != 0) {
		mv_os_printf("IPC DEMO: Attach returned error for target CPU %d\n",
				ipc_get_remote_node_id(link));
		return -1;
	}

	/* Enable channel for RX */
	ipc_enable_chn_rx(link, chn);

	if (attached) {
		mv_os_printf("IPC DEMO: channel %d to cpu %d attached succesfully.\n",
				chn, ipc_get_remote_node_id(link));
	}

	return 0;
}

/* Disable/dettach/close demo channel(IPC_DEMO_CHANNEL_ID) in particular
 * link(IPC_DEMO_LINK_ID).
 */
int ipc_demo_deinit(unsigned int link, unsigned int chn)
{
	int status;

	ipc_disable_chn_rx(link, chn);

	status = ipc_dettach_chn(link, chn);
	if (status != 0) {
		mv_os_printf("IPC DEMO: Dettach returned error for target CPU %d\n",
				ipc_get_remote_node_id(link));
		return -1;
	}

	status = ipc_close_chn(link, chn);
	if (status != 0) {
		mv_os_printf("IPC DEMO: Failed to close IPC channel %d", chn);
		return -1;
	}

	return 0;
}

