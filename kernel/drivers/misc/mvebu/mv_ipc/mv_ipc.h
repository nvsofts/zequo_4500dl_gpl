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

#ifndef __mv_ipc_h
#define __mv_ipc_h

#ifdef __KERNEL__
#include <linux/slab.h>
#include <linux/kernel.h>
#include <linux/io.h>
#include <linux/delay.h>
#include <linux/device.h>
#else /* __KERNEL__ - FreeRTOS */
#include <printf.h>
#include <stdlib.h>
#include <common.h>
#include <string.h>
#endif /* __KERNEL__ - FreeRTOS */

/* OS definition (Linux and FreeRTOS) */
#ifdef __KERNEL__
#define mv_os_malloc(_size_)	kmalloc(_size_, GFP_ATOMIC)
#define mv_os_free		kfree
#define mv_os_printf	printk
#define MV_NO_MORE	(12)	/* No more items found */
#define MV_NOT_STARTED	(29)	/* Not started yet */
#else /* __KERNEL__ - FreeRTOS */
/* Use the same bool definition as Linux uses - it is required to ensure the
 * same struct size copied to the shared memory
 */
typedef _Bool			bool;
enum {
	false	= 0,
	true	= 1
};
#define mv_os_malloc(x)		pvPortMalloc(x)
#define mv_os_free		vPortFree
#define memset(p, v, s)		memset((void *)(p), (long)v, (size_t)s)
#define mv_os_printf		mvOsPrintf
#define roundup(x, y)		((x + ((y)-1)) & ~((y)-1))
#define usleep_range(x, y)	udelay(x)
#define min(a, b)		((a) < (b) ? (a) : (b))
#endif

/* Unique ID of the IPC node, all nodes should be numbered sequentially from 0 */
#define MV_IPC_NODE_ID_MASTER	0
#define MV_IPC_NODE_ID_SLAVE	1

#define IPC_CM3_FREERTOS_LINK_ID	0
#define IPC_DRG_FREERTOS_LINK_ID	1
#define IPC_SCPU_FREERTOS_LINK_ID	2
#define IPC_MAX_FREERTOS_LINK_ID	3

/* Number of links established with other nodes */
#define MV_IPC_LINKS_NUM	IPC_MAX_FREERTOS_LINK_ID

#define MV_IPC_CHN_NUM_MAX	8

/* Division of free malloc area between master and slave */
#define MV_IPC_MASTER_FREE_REGION_PERCENT 50

#define IPC_RX_MAX_MSGS_PER_ISR	(50)

/* Magic for master_conf_done, wrote by master and clean by slave */
#define MV_IPC_MASTER_CONFIG_MAGIC      0x12345678

#define IPC_CPU_D_CACHE_LINE_SIZE 32

/* Control messages types */
enum {
	IPC_MSG_ATTACH_REQ = 0,
	IPC_MSG_ATTACH_ACK,
	IPC_MSG_DETACH_REQ,
	IPC_MSG_DETACH_ACK
};

/* Message struct(channel queue entry). Size is align to cache line size */
struct ipc_message {
	unsigned int type;
	unsigned int size;	/* Buffer size */
	unsigned int value;	/* User data */
	unsigned int is_used;	/* CPU Id and optional oob message */
} __attribute__((__aligned__(IPC_CPU_D_CACHE_LINE_SIZE)));

/* Function types */
typedef int (*MV_IPC_RX_CLBK)(struct ipc_message *msg, void *dev);
typedef int (*MV_IPC_SET_CHN_STAT)(int attached, void *dev);
typedef void (*MV_IPC_SEND_TRIGGER)(unsigned int link_id, unsigned int chn_id, void *dev_id);
typedef void (*mv_ipc_rx_channel_register)(unsigned int link_id, unsigned int chn_id, bool enable, void *dev_id);
typedef int (*mv_ipc_interrupt_init)(unsigned int link_id, void *dev_id);

/* Channel statuses */
enum ipc_chn_state {
	MV_CHN_CLOSED =  0,
	MV_CHN_OPEN,
	MV_CHN_LINKING,
	MV_CHN_UNLINKING,
	MV_CHN_ATTACHED
};

/* Channel struct */
struct ipc_channel {
	void *rx_msg_que_va;	/* Buffer virtual address for Rx side */
	void *tx_msg_que_va;	/* Buffer virtual address for Tx side */
	struct ipc_message *rx_ctrl_msg;	/* Buffer virtual address for Rx side */
	struct ipc_message *tx_ctrl_msg;	/* Buffer virtual address for Tx side */
	unsigned int next_rx_msg_idx;
	unsigned int next_tx_msg_idx;
	unsigned int msg_rxque_size;	/* Size of Rx queue */
	unsigned int msg_txque_size;	/* Size of Tx queue */
	unsigned int rxbuf_size;	/* Rx buffer size in bytes */
	unsigned int txbuf_size;	/* Tx buffer size in bytes */
	unsigned int remote_node_id;
	int tx_enable;
	int rx_enable;
	enum ipc_chn_state state;

	unsigned int tx_msg_flag;	/* Shared memory flag raised for message in queue */
	unsigned int rx_msg_flag;	/* Shared memory flag raised for message in queue */

	void *private_data;		/* Channel user data */

	MV_IPC_RX_CLBK rx_callback;			/* Called for each RX */
	MV_IPC_SET_CHN_STAT set_chn_status;
	MV_IPC_SEND_TRIGGER send_trigger;		/* Trigger to remote node to start RX */
	mv_ipc_rx_channel_register enable_chn_doorbell;	/* Register the channel in RX ISR/Timer */
};

/* Channel common struct */
struct ipc_channel_common {
	void *rx_msg_que_va;	/* Buffer virtual address for Rx side*/
	void *tx_msg_que_va;	/* Buffer virtual address for Tx side*/
	unsigned int msg_rxque_size;	/* Size of Rx queue */
	unsigned int msg_txque_size;	/* Size of Tx queue */

	unsigned int rxbuf_size;	/* Rx buffer size in bytes */
	unsigned int txbuf_size;	/* Tx buffer size in bytes */

	unsigned int tx_msg_flag;	/* Shared memory flag raised for message in queue*/
	unsigned int rx_msg_flag;	/* Shared memory flag raised for message in queue*/
};

struct ipc_link {
	struct ipc_channel *channels;	/* Array of channels */
	unsigned int num_of_chn;	/* Number of channels */
	unsigned int shmem_base_addr;	/* Shared mem addr (virtual on Linux side, physical on FRTOS side */
	unsigned int shmem_size;		/* Shared mem physical addr */
	unsigned int node_id;			/* I node ID */
	unsigned int remote_node_id;		/* Remote node ID */
	unsigned int tx_shared_heap_addr;	/* Offset of heap node memory */
	unsigned int tx_shared_heap_size;	/* Size of heap node memory */
	unsigned int rx_shared_heap_addr;	/* Offset of heap for remote node memory */
	unsigned int rx_shared_heap_size;	/* Size of heap node memory */
	unsigned int master_conf_done;		/* If master finished the configuration */
	unsigned int slave_link_initialized;	/* If master not finished the configuration
							and configuration was postponed by slave */
};

struct ipc_link_info {
	unsigned int num_of_chn;
	bool is_master;
	unsigned int remote_node_id;
	unsigned int shmem_size;
	void *shmem_virt_addr;
	void *dev_id;
	MV_IPC_SEND_TRIGGER send_trigger;		/* Trigger to remote node to start RX */
	mv_ipc_rx_channel_register enable_chn_doorbell;	/* Register the channel in RX ISR/Timer */
	mv_ipc_interrupt_init irq_init;

	/* Define only when master */
	unsigned int master_free_region_percent;	/* Division of free malloc area between master and slave */
	/* Sizes of TX/RX queues for each channel (slave node doesn't know the sizes).
	 * For slave set queue size to 0, the value will be ignored hw Layer indexes
	 */
	unsigned int chn_rx_info_array[MV_IPC_CHN_NUM_MAX];
	unsigned int chn_tx_info_array[MV_IPC_CHN_NUM_MAX];

	/* Describes the buffer size per-queue (4B granulation, e.g. 8 means 32B) */
	unsigned int ch_rxbuf_info[MV_IPC_CHN_NUM_MAX];
	unsigned int ch_txbuf_info[MV_IPC_CHN_NUM_MAX];
};

/* Assume that all RTOS machines are in LE (Dragonite, SCPU and so on) */
#if defined(CONFIG_CPU_BIG_ENDIAN) && defined(__KERNEL__)
#define __le_to_be32(x) ((__force __be32)__swab32((x)))
#define __be32_to_le(x) (__swab32((__force __u32)(__be32)(x)))

#define mvU32Host2Ipc(x) __be32_to_le(x)
#define mvU32Ipc2Host(x) __le_to_be32(x)
#else
#define mvU32Host2Ipc(x) (x)
#define mvU32Ipc2Host(x) (x)
#endif

#define IPC_MAX_MSG_SIZE		(128)

/* ipc api */
bool ipc_is_master(unsigned int link_id);
unsigned int ipc_get_remote_node_id(unsigned int link_id);
unsigned int ipc_who_am_i(unsigned int link_id);
void *get_shared_mem_offset(unsigned int link_id, void *virt_addr);
void *get_shared_mem_virt_addr(unsigned int link_id, void *sh_mem_offset);

char *ipc_getnext_txbuf(unsigned int link_id, unsigned int chn_id);
char *ipc_get_rxbuf_of_msg(struct ipc_message *msg);
int ipc_chn_txbuf_max_size(unsigned int link_id, unsigned int chn_id);

int ipc_close(unsigned int link_id);
int ipc_open_chn(unsigned int link_id, unsigned int chn_id, void *dev, MV_IPC_RX_CLBK rx_clbk, MV_IPC_SET_CHN_STAT set_chn_status);
int ipc_attach_chn(unsigned int link_id, unsigned int chn_id, unsigned int remote_node_id, int *attached);
int ipc_dettach_chn(unsigned int link_id, unsigned int chn_id);
int ipc_close_chn(unsigned int link_id, unsigned int chn_id);
bool ipc_tx_ready(unsigned int link_id, unsigned int chn_id);
int ipc_tx_ctrl_msg(unsigned int link_id, unsigned int chn_id, struct ipc_message *inMsg);
int ipc_tx_msg(unsigned int link_id, unsigned int chn_id, struct ipc_message *inMsg);
void ipc_disable_chn_rx(unsigned int link_id, unsigned int chn_id);
void ipc_enable_chn_rx(unsigned int link_id, unsigned int chn_id);
int ipc_rx_msg(unsigned int link_id, unsigned int chn_id,
	       char *msg_data, size_t msg_size);
bool ipc_rx_msg_flag_check(unsigned int link_id, unsigned int chn_id);
int ipc_release_msg(unsigned int link_id, unsigned int chn_id, struct ipc_message *msg);
void *ipc_sh_malloc(unsigned int link_id, unsigned int size);
void ipc_rx_process(unsigned int link_id, char *enabled_chn,
		    char *msg_data, size_t msg_size);
int ipc_rx_queue_process(unsigned int link_id, unsigned int chn_id,
			 char *msg_data, size_t msg_size);
int ipc_tx_queue_send(unsigned int link_id, unsigned int chn_id, const char *data, unsigned int length);
int ipc_shmem_link_setup(unsigned char link_id,
			 struct ipc_link_info *link_info);
int ipc_shmem_link_cleanup(unsigned char link_id);

#ifdef CONFIG_MV_IPC_FREERTOS_DEMO
int ipc_demo_init(unsigned int link, unsigned int chn);
int ipc_demo_deinit(unsigned int link, unsigned int chn);
#endif

#endif /* __mv_ipc_h */
