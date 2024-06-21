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
* mv_ipc_wrapper.c
*
* DESCRIPTION:
*	Wrapper layer of IPC support in Linux Kernel.
*	In this wrapper, link class will be created and each channel is an individual
*	device, IPC msg can be sent/received by write/read the device.
*
* DEPENDENCIES:
*
*******************************************************************************/
#include <linux/kernel.h>
#include <linux/init.h>
#include <linux/module.h>
#include <linux/device.h>
#include <linux/cdev.h>
#include <linux/fs.h>
#include <linux/poll.h>
#include <linux/slab.h>
#include <linux/wait.h>
#include <linux/sched.h>
#include <linux/list.h>
#include <linux/spinlock.h>

#include "mv_ipc.h"
#include "mv_ipc_wrapper.h"

#undef MV_IPC_WRAPPER_DEBUG

#ifdef MV_IPC_WRAPPER_DEBUG
#define ipc_wrapper_debug mv_os_printf
#else
#define ipc_wrapper_debug(...)
#endif /* MV_IPC_WRAPPER_DEBUG */


#define MV_IPC_CHNDEV_MAJOR        280
#define MV_IPC_CHNDEV_MINOR(l, c)  (((l) << MV_IPC_CHNDEV_NUM_BITS) | (c))

#define MV_IPC_CHNDEV_NUM_BITS     3
#define MV_IPC_CHNDEV_NUM_MASK     ((1 << MV_IPC_CHNDEV_NUM_BITS) - 1)

#define MV_IPC_MINOR_2_LINK(m)     ((m) >> MV_IPC_CHNDEV_NUM_BITS)
#define MV_IPC_MINOR_2_CHN(m)      ((m) & MV_IPC_CHNDEV_NUM_MASK)

#define MV_IPC_CLASS_NAME_SIZE          16

#define MV_IPC_CHN_QUEUE_BUFFER_NUM     16
#define MV_IPC_CHN_QUEUE_BUFFER_SIZE    (IPC_MAX_MSG_SIZE/4)

#define MV_IPC_POLL_PERIOD 1

struct ipc_rx_msg {
	struct list_head  node;
	char              buff[IPC_MAX_MSG_SIZE];
	unsigned int      size;
};

/* channel device struct */
struct ipc_channel_device {
	unsigned int      chn_id;
	unsigned int      chn_rdy;
	struct cdev       dev;
	struct ipc_link_class  *link;
	spinlock_t        rx_lock;
	wait_queue_head_t rx_wait;
	unsigned int      rx_todo;
	struct list_head  rx_msg_list;
	char              tx_buff[IPC_MAX_MSG_SIZE];
};

/* link class struct */
struct ipc_link_class {
	unsigned int      link_id;
	char              name[MV_IPC_CLASS_NAME_SIZE];
	struct class     *class;
	char              chn_en[MV_IPC_CHN_NUM_MAX];
	struct ipc_channel_device *chns[MV_IPC_CHN_NUM_MAX];
};

/* the struct for IPC links which work in polling mode */
struct ipc_links_poll {
	unsigned int poll_links_num;		/* the IPC links number in polling mode */
	bool links_poll[MV_IPC_LINKS_NUM];	/* whether IPC links work in polling mode */
	struct timer_list poll_timer;		/* the polling timer for all ipc links in polling mode */
};

static struct ipc_link_class ipc_link_classes[MV_IPC_LINKS_NUM];

static struct ipc_links_poll ipc_links_poller = {
	.poll_links_num = 0,
};

/* This is the main structure for the IPC wrapper, the user
can update the settings here accodring to the demand */
static struct ipc_link_info ipc_links_cfg[MV_IPC_LINKS_NUM] = {
	[IPC_CM3_FREERTOS_LINK_ID] = {
		.num_of_chn           = 6,
		.is_master            = true,
		.remote_node_id       = MV_IPC_NODE_ID_SLAVE,

		/* tx/rx queue size, i.e., buffer number */
		.chn_tx_info_array[0] = 4,
		.chn_tx_info_array[1] = 4,
		.chn_tx_info_array[2] = 1,
		.chn_tx_info_array[3] = 1,
		.chn_tx_info_array[4] = 1,
		.chn_tx_info_array[5] = 1,

		.chn_rx_info_array[0] = 1,
		.chn_rx_info_array[1] = 1,
		.chn_rx_info_array[2] = 2,
		.chn_rx_info_array[3] = 2,
		.chn_rx_info_array[4] = 2,
		.chn_rx_info_array[5] = 2,

		/* tx/rx buffer size */
		/* Because of 4B granularity, 16 refers to 64B */
		.ch_txbuf_info[0] = 16,
		.ch_txbuf_info[1] = 16,
		.ch_txbuf_info[2] = 0,
		.ch_txbuf_info[3] = 0,
		.ch_txbuf_info[4] = 0,
		.ch_txbuf_info[5] = 0,

		.ch_rxbuf_info[0] = 0,
		.ch_rxbuf_info[1] = 0,
		.ch_rxbuf_info[2] = 16,
		.ch_rxbuf_info[3] = 16,
		.ch_rxbuf_info[4] = 16,
		.ch_rxbuf_info[5] = 16,
	},
	[IPC_DRG_FREERTOS_LINK_ID] = {
		.num_of_chn           = 3,
		.is_master            = true,
		.remote_node_id       = MV_IPC_NODE_ID_SLAVE,

		/* three channels in all: chn0 RX only, chn1 TX only, chn2 DGB channel RX/TX */
		/* chn0&1 are used for normal messages */
		/* chn0&1 buffer size is 32 bytes, 4 buffers for each rx/tx queue. */
		/* chn1 is for IPC request, chn0 is for IPC reply. */
		/* chn2 is used for debug messages */
		/* ch2 buffer size is 128 bytes: 2 buffers for tx queue while 12 buffers for rx queue. */

		/* tx/rx queue size, i.e., buffer number */
		.chn_tx_info_array[0] = 4,
		.chn_tx_info_array[1] = 4,
		.chn_tx_info_array[2] = 2,

		.chn_rx_info_array[0] = 4,
		.chn_rx_info_array[1] = 4,
		.chn_rx_info_array[2] = 12,

		/* tx/rx buffer size */
		/* Because of 4B granularity, 16 refers to 64B */
		.ch_txbuf_info[0] = 0,
		.ch_txbuf_info[1] = 8,
		.ch_txbuf_info[2] = 32,

		.ch_rxbuf_info[0] = 8,
		.ch_rxbuf_info[1] = 0,
		.ch_rxbuf_info[2] = 32,
	},
	[IPC_SCPU_FREERTOS_LINK_ID] = {
		.num_of_chn           = 6,
		.is_master            = true,
		.remote_node_id       = MV_IPC_NODE_ID_SLAVE,

		/* tx/rx queue size, i.e., buffer number */
		.chn_tx_info_array[0] = 4,
		.chn_tx_info_array[1] = 4,
		.chn_tx_info_array[2] = 1,
		.chn_tx_info_array[3] = 1,
		.chn_tx_info_array[4] = 1,
		.chn_tx_info_array[5] = 1,

		.chn_rx_info_array[0] = 1,
		.chn_rx_info_array[1] = 1,
		.chn_rx_info_array[2] = 2,
		.chn_rx_info_array[3] = 2,
		.chn_rx_info_array[4] = 2,
		.chn_rx_info_array[5] = 2,

		/* tx/rx buffer size */
		/* Because of 4B granularity, 16 refers to 64B */
		.ch_txbuf_info[0] = 16,
		.ch_txbuf_info[1] = 16,
		.ch_txbuf_info[2] = 0,
		.ch_txbuf_info[3] = 0,
		.ch_txbuf_info[4] = 0,
		.ch_txbuf_info[5] = 0,

		.ch_rxbuf_info[0] = 0,
		.ch_rxbuf_info[1] = 0,
		.ch_rxbuf_info[2] = 16,
		.ch_rxbuf_info[3] = 16,
		.ch_rxbuf_info[4] = 16,
		.ch_rxbuf_info[5] = 16,
	},
};


/***************************************/
/*             channel cdev            */
/***************************************/

static int filp_to_link_chn_id(struct file *filp, unsigned int *link_id, unsigned int *chn_id)
{
#ifdef CONFIG_OF
	unsigned int minor = MINOR(filp->f_inode->i_rdev);
#else
	unsigned int minor = MINOR(filp->f_dentry->d_inode->i_rdev);
#endif

	*link_id = MV_IPC_MINOR_2_LINK(minor);
	*chn_id  = MV_IPC_MINOR_2_CHN(minor);

	return 0;
}
static ssize_t ipc_wrapper_chndev_read(struct file *filp, char *buf, size_t count, loff_t *f_pos)
{
	struct ipc_channel_device *chn_dev = filp->private_data;
	struct ipc_rx_msg *msg;

	ipc_wrapper_debug("%s(): rx_todo=%d\n", __func__, chn_dev->rx_todo);

	if (!chn_dev->chn_rdy)
		return -EAGAIN;

	if (!chn_dev->rx_todo) {
		if (filp->f_flags & O_NONBLOCK)
			return -EAGAIN;
		ipc_wrapper_debug("waiting for messages...\n");
		wait_event_interruptible(chn_dev->rx_wait, chn_dev->rx_todo);
	}

	spin_lock_irq(&chn_dev->rx_lock);
	if (list_empty(&chn_dev->rx_msg_list)) {
		chn_dev->rx_todo = 0;
		spin_unlock_irq(&chn_dev->rx_lock);
		return -EFAULT;
	}

	msg = list_first_entry(&chn_dev->rx_msg_list, struct ipc_rx_msg, node);
	list_del(&msg->node);
	spin_unlock_irq(&chn_dev->rx_lock);

	ipc_wrapper_debug("Read %d bytes\n", msg->size);

	/* Assuming the buffer is big enough, the whole
	message should be retrieved in one read syscall.*/
	if (msg->size > count)
		return -EMSGSIZE;

	/* Each read syscall is supposed to get one and
	only one message.*/
	count = msg->size;

	if (copy_to_user(buf, msg->buff, count))
		return -EFAULT;

	kfree(msg);
	chn_dev->rx_todo--;

	return count;
}

static ssize_t ipc_wrapper_chndev_write(struct file *filp, const char __user *buf, size_t count, loff_t *f_pos)
{
	struct ipc_channel_device *chn_dev = filp->private_data;
	int ret;

	ipc_wrapper_debug("Write %zu bytes\n", count);

	if (!chn_dev->chn_rdy)
		return -EAGAIN;

	/* The whole message should be transmitted in one write syscall.*/
	if (count > ipc_links_cfg[chn_dev->link->link_id].ch_txbuf_info[chn_dev->chn_id] * 4)
		return -EMSGSIZE;

	memset(chn_dev->tx_buff, 0, sizeof(chn_dev->tx_buff));
	if (copy_from_user(chn_dev->tx_buff, buf, count))
		return -EFAULT;

	ipc_wrapper_debug("Write %s\n", chn_dev->tx_buff);

	ret = ipc_tx_queue_send(chn_dev->link->link_id, chn_dev->chn_id,
			(const char *)chn_dev->tx_buff, (unsigned int)count);
	if (ret)
		return -EBUSY;

	return count;
}

static unsigned int ipc_wrapper_chndev_poll(struct file *filp, poll_table *poll_table_p)
{
	struct ipc_channel_device *chn_dev = filp->private_data;
	int mask = 0;

	poll_wait(filp, &chn_dev->rx_wait, poll_table_p);

	if (chn_dev->rx_todo) {
		ipc_wrapper_debug("Receive one IPC message ...\n");
		mask |= POLLRDNORM | POLLIN;
	}

	return mask;
}

static int ipc_wrapper_chn_open(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel_device *chn_dev;
	unsigned int remote_nid;
	int ret;

	if (ipc_link_classes[link_id].chn_en[chn_id]) {
		mv_os_printf("IPC Dev: link %d channel %d is in use\n",
			     link_id, chn_id);
		return -EBUSY;
	}

	chn_dev = ipc_link_classes[link_id].chns[chn_id];
	ipc_wrapper_debug("Open link %d channel %d\n", link_id, chn_id);

	/* Enable channel before attach - since during the attach some ctrl msg
	 * will be send/receive on both directions
	 */
	ipc_enable_chn_rx(link_id, chn_id);

	ipc_link_classes[link_id].chn_en[chn_id] = 1;
	chn_dev->rx_todo = 0;
	INIT_LIST_HEAD(&chn_dev->rx_msg_list);

	remote_nid = ipc_get_remote_node_id(link_id);
	ret = ipc_attach_chn(link_id, chn_id, remote_nid, &chn_dev->chn_rdy);
	if (ret) {
		mv_os_printf("IPC Dev: Attach failed for target CPU %d\n",
			      remote_nid);
		return ret;
	}

	if (chn_dev->chn_rdy) {
		ipc_wrapper_debug("IPC Dev: link %d chan %d to cpu %d attached\n",
			link_id, chn_id, remote_nid);
	}

	return 0;
}

static int ipc_wrapper_chn_close(unsigned int link_id, unsigned int chn_id)
{
	struct ipc_channel_device *chn_dev;
	struct ipc_rx_msg *entry, *next;
	int ret;

	chn_dev = ipc_link_classes[link_id].chns[chn_id];
	ipc_wrapper_debug("Close link %d channel %d\n", link_id, chn_id);

	if (!ipc_link_classes[link_id].chn_en[chn_id])
		return 0;

	chn_dev->rx_todo = 0;
	chn_dev->chn_rdy = false;

	ipc_link_classes[link_id].chn_en[chn_id] = 0;

	ipc_disable_chn_rx(link_id, chn_id);

	ret = ipc_dettach_chn(link_id, chn_id);
	if (ret != 0) {
		mv_os_printf("IPC Dev: Detach failed for target CPU %d\n",
			ipc_get_remote_node_id(link_id));
		return ret;
	}

	if (list_empty(&chn_dev->rx_msg_list))
		return 0;

	/* flush the rx message list */
	list_for_each_entry_safe(entry, next, &chn_dev->rx_msg_list, node) {
		list_del(&entry->node);
		kfree(entry);
	}

	return 0;
}

static long ipc_wrapper_chndev_ioctl(struct file *filp, unsigned int cmd,
							unsigned long arg)
{
	struct ipc_channel_device *chn_dev = filp->private_data;
	unsigned int link_id, chn_id;
	int ret = 0;

	/* Don't even decode wrong cmds */
	if (_IOC_TYPE(cmd) != IPCW_IOC_MAGIC) {
		mv_os_printf("wrong ioctl magic key\n");
		return -ENOTTY;
	}

	filp_to_link_chn_id(filp, &link_id, &chn_id);

	switch (cmd) {
	case IPCW_IOC_GET_STAT:
		if (copy_to_user((void __user *)arg, &chn_dev->chn_rdy,
							sizeof(unsigned int))) {
			mv_os_printf("copy_to_user failed\n");
			return -EFAULT;
		}
		break;

	case IPCW_IOC_CHN_OPEN:
		ret = ipc_wrapper_chn_open(link_id, chn_id);
		break;
	case IPCW_IOC_CHN_CLOSE:
		ret = ipc_wrapper_chn_close(link_id, chn_id);
		break;
	default:
		mv_os_printf("%s: unknown ioctl (%x)\n", __func__, cmd);
		return -EINVAL;
	}

	if (ret)
		return ret;

	return 0;
}

static int ipc_wrapper_chndev_open(struct inode *inode, struct file *file)
{
	struct ipc_channel_device *chn_dev;
	unsigned int link_id, chn_id;

	filp_to_link_chn_id(file, &link_id, &chn_id);

	chn_dev = ipc_link_classes[link_id].chns[chn_id];

	file->private_data = chn_dev;

	return 0;
}

static int ipc_wrapper_chndev_release(struct inode *inode, struct file *file)
{
	file->private_data = NULL;

	return 0;
}

static const struct file_operations ipc_wrapper_chndev_fops = {
	.owner          = THIS_MODULE,
	.read           = ipc_wrapper_chndev_read,
	.write          = ipc_wrapper_chndev_write,
	.poll           = ipc_wrapper_chndev_poll,
	.unlocked_ioctl = ipc_wrapper_chndev_ioctl,
	.open           = ipc_wrapper_chndev_open,
	.release        = ipc_wrapper_chndev_release,
};

/* Description:
 * This fuction is to copy the IPC message from shared memory
 * to the receiver's memory space, i.e., Host CPU's own buffer
 * for each channel. Then, it awakes the poll function to
 * indicates user application to retrieve the message via
 * read() API.
 */
static int ipc_wrapper_channel_rx(struct ipc_message *msg, void *dev)
{
	struct ipc_channel_device *chn_dev = dev;
	struct ipc_rx_msg *entry;
	char *ptr_virt;
	unsigned long flags;

	if (!chn_dev)
		return -EINVAL;

	ipc_wrapper_debug("%s: %d, %d\n", __func__, chn_dev->link->link_id, chn_dev->chn_id);

	spin_lock_irqsave(&chn_dev->rx_lock, flags);

	ptr_virt = ipc_get_rxbuf_of_msg(msg);
	if (!ptr_virt)
		ipc_wrapper_debug("IPC Dev: Unable to get rxbuf of msg %p\n", msg);

	ipc_wrapper_debug("IPC Dev: addr of received msg buf %p(shmem offset), ptr_virt %p\n",
		get_shared_mem_offset(chn_dev->link->link_id, ptr_virt), ptr_virt);

	entry = kmalloc(sizeof(struct ipc_rx_msg), GFP_ATOMIC);
	if (!entry)
		return -ENOMEM;

	/* Copy message from shared memory to host memory */
	memcpy(entry->buff, ptr_virt, msg->size);
	entry->size = msg->size;

	list_add_tail(&entry->node, &chn_dev->rx_msg_list);

	/* Wake up blocking READ syscall */
	chn_dev->rx_todo++;
	wake_up(&chn_dev->rx_wait);

	spin_unlock_irqrestore(&chn_dev->rx_lock, flags);

	return 0;
}

static int ipc_wrapper_channel_set_status(int attached, void *dev)
{
	struct ipc_channel_device *chn_dev = dev;

	if (!chn_dev)
		return -EINVAL;

	chn_dev->chn_rdy = attached;

	mv_os_printf("Status: Link %d Channel %d %s\n",
		chn_dev->link->link_id, chn_dev->chn_id,
		attached ? "attached" : "de-attached");

	return 0;
}
int ipc_wrapper_channel_register(unsigned char link_id, unsigned int chn_id)
{
	struct ipc_channel_device *chn_dev;
	char dev_name[16];
	dev_t dev = MKDEV(MV_IPC_CHNDEV_MAJOR, MV_IPC_CHNDEV_MINOR(link_id, chn_id));
	int ret = 0;

	if (link_id >= MV_IPC_LINKS_NUM || chn_id >= MV_IPC_CHN_NUM_MAX)
		return -EINVAL;

	if (ipc_link_classes[link_id].chns[chn_id])
		return 0;

	chn_dev = kmalloc(sizeof(struct ipc_channel_device), GFP_KERNEL);
	if (!chn_dev)
		return -ENOMEM;

	spin_lock_init(&chn_dev->rx_lock);
	init_waitqueue_head(&chn_dev->rx_wait);
	INIT_LIST_HEAD(&chn_dev->rx_msg_list);
	chn_dev->chn_id      = chn_id;
	chn_dev->chn_rdy     = 0;
	chn_dev->rx_todo     = 0;
	chn_dev->link        = &ipc_link_classes[link_id];

	/* Initialize IPC driver */
	ret = ipc_open_chn(link_id, chn_id, chn_dev,
		 ipc_wrapper_channel_rx, ipc_wrapper_channel_set_status);
	if (ret)
		goto fail_dev_cleanup;

	memset(dev_name, 0, sizeof(dev_name));
	sprintf(dev_name, "chn%d", chn_id);

	ret = register_chrdev_region(dev, 1, dev_name);
	if (ret < 0) {
		mv_os_printf("error: register dev region [%d:%d]\n", link_id, chn_id);
		goto fail_chn_close;
	}

	cdev_init(&chn_dev->dev, &ipc_wrapper_chndev_fops);
	ret = cdev_add(&chn_dev->dev, dev, 1);
	if (ret) {
		mv_os_printf("error: add dev [%d:%d]\n", link_id, chn_id);
		goto fail_chrdev_unreg;
	}

	device_create(ipc_link_classes[link_id].class, NULL, dev, NULL, dev_name);

	ipc_link_classes[link_id].chns[chn_id] = chn_dev;

	ipc_wrapper_debug("Register chn #%d on link #%d\n", chn_id, link_id);

	ret = ipc_wrapper_chn_open(link_id, chn_id);

out:
	return ret;
fail_chrdev_unreg:
	unregister_chrdev_region(dev, 1);
fail_chn_close:
	ipc_close_chn(link_id, chn_id);
fail_dev_cleanup:
	kfree(chn_dev);
	goto out;
}

int ipc_wrapper_channel_unregister(unsigned char link_id, unsigned int chn_id)
{
	dev_t dev = MKDEV(MV_IPC_CHNDEV_MAJOR, MV_IPC_CHNDEV_MINOR(link_id, chn_id));
	struct ipc_channel_device *chn_dev = ipc_link_classes[link_id].chns[chn_id];

	if (link_id >= MV_IPC_LINKS_NUM || chn_id >= MV_IPC_CHN_NUM_MAX)
		return -EINVAL;

	if (!chn_dev)
		return 0;

	if (ipc_wrapper_chn_close(link_id, chn_id))
		return -EBUSY;

	ipc_link_classes[link_id].chns[chn_id] = NULL;

	device_destroy(ipc_link_classes[link_id].class, dev);
	cdev_del(&chn_dev->dev);
	unregister_chrdev_region(dev, 1);

	ipc_close_chn(link_id, chn_id);

	kfree(chn_dev);

	ipc_wrapper_debug("Unregister chn #%d on link #%d\n", chn_id, link_id);

	return 0;
}

/***************************************/
/*              link class             */
/***************************************/

static char *ipc_wrapper_link_devnode(struct device *dev, umode_t *mode)
{
	unsigned int minor;

	/* RW permissions*/
	if (mode)
		*mode = 0666;
	if (!dev)
		return NULL;
	minor = MINOR(dev->devt);

	/* device path: /dev/ipc-lnkX/chnY*/
	return kasprintf(GFP_KERNEL, "ipc-lnk%d/%s", MV_IPC_MINOR_2_LINK(minor), dev_name(dev));
}

int ipc_wrapper_link_rx_process(unsigned int link_id)
{
	if (link_id >= MV_IPC_LINKS_NUM)
		return -EINVAL;

	ipc_rx_process(link_id, ipc_link_classes[link_id].chn_en, NULL, 0);

	return 0;
}
/* register the link class and setup the link info */
int ipc_wrapper_link_bind(unsigned int link_id, void *dev_id, int (*set_shmem)(struct ipc_link_info *link_info))
{
	struct ipc_link_info link_info;
	struct ipc_link_class *ipc_cls;
	int ret = 0, i;

	if (link_id >= MV_IPC_LINKS_NUM)
		return -EINVAL;

	if (!dev_id || !set_shmem)
		return -EINVAL;

	memcpy(&link_info, &ipc_links_cfg[link_id], sizeof(struct ipc_link_info));

	link_info.dev_id = dev_id;
	ret = set_shmem(&link_info);
	if (ret)
		return ret;

	ret = ipc_shmem_link_setup(link_id, &link_info);
	if (ret)
		return ret;

	ipc_cls = &ipc_link_classes[link_id];
	ipc_cls->link_id = link_id;

	for (i = 0; i < MV_IPC_CHN_NUM_MAX; i++) {
		ipc_cls->chn_en[i] = 0;
		ipc_cls->chns[i]   = NULL;
	}
	memset(ipc_cls->name, 0, MV_IPC_CLASS_NAME_SIZE * sizeof(char));
	sprintf(ipc_cls->name, "ipc-lnk%d", link_id);

	ipc_cls->class = class_create(THIS_MODULE, ipc_cls->name);
	if (IS_ERR(ipc_cls->class)) {
		mv_os_printf("error: create link #%d class", link_id);
		ret = PTR_ERR(ipc_cls->class);
		goto fail_link_cleanup;
	}
	ipc_cls->class->devnode = ipc_wrapper_link_devnode;

	/* by default, the ipc polling mode is disabled */
	ipc_links_poller.links_poll[link_id] = false;

	ipc_wrapper_debug("Bind link #%d\n", link_id);

out:
	return ret;
fail_link_cleanup:
	ipc_shmem_link_cleanup(link_id);
	goto out;
}

/* unregister the link class and cleanup the link info */
int ipc_wrapper_link_unbind(unsigned int link_id)
{
	struct ipc_link_class *ipc_cls;
	int i;

	if (link_id >= MV_IPC_LINKS_NUM)
		return -EINVAL;

	ipc_cls = &ipc_link_classes[link_id];

	for (i = 0; i < MV_IPC_CHN_NUM_MAX; i++)
		if (ipc_cls->chns[i])
			return -EFAULT;

	class_destroy(ipc_cls->class);
	ipc_shmem_link_cleanup(link_id);

	ipc_wrapper_debug("Unbind link #%d\n", link_id);

	return 0;
}

/* rx polling service routine */
static void ipc_wrapper_do_rx_poll(unsigned long data)
{
	unsigned int link_id;

	if (!ipc_links_poller.poll_links_num)
		return;

	/* Scan all ipc links*/
	for (link_id = 0; link_id < MV_IPC_LINKS_NUM; link_id++)
		if (ipc_links_poller.links_poll[link_id])
			ipc_wrapper_link_rx_process(IPC_SCPU_FREERTOS_LINK_ID);

	ipc_links_poller.poll_timer.expires = jiffies + MV_IPC_POLL_PERIOD;
	add_timer(&ipc_links_poller.poll_timer);
}

/* enable an ipc link to work in polling mode */
void ipc_wrapper_enable_link_poll(unsigned int link_id)
{
	ipc_links_poller.links_poll[link_id] = true;

	/*Start timer event*/
	if (!ipc_links_poller.poll_links_num) {
		init_timer(&ipc_links_poller.poll_timer);
		ipc_links_poller.poll_timer.function = ipc_wrapper_do_rx_poll;
		ipc_links_poller.poll_timer.expires = jiffies + MV_IPC_POLL_PERIOD;
		add_timer(&ipc_links_poller.poll_timer);
	}

	ipc_links_poller.poll_links_num++;

	return;
}

/* disable an ipc link in polling mode */
void ipc_wrapper_disable_link_poll(unsigned int link_id)
{
	ipc_links_poller.links_poll[link_id] = false;

	/*Start timer event*/
	if (ipc_links_poller.poll_links_num) {
		ipc_links_poller.poll_links_num--;
		if (!ipc_links_poller.poll_links_num)
			del_timer(&ipc_links_poller.poll_timer);
	}

	return;
}

