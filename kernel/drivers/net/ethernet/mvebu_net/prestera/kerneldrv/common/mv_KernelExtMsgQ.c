/*******************************************************************************
* mv_KervelExtMsgQ.c
*
* DESCRIPTION:
*       Message queues
*
* DEPENDENCIES:
*
********************************************************************************/


/************* Defines ********************************************************/

#define MV_MSGQ_STAT

/************ Internal Typedefs ***********************************************/
struct mvMsgQSTC {
	int                     flags;
	char                    name[MV_MSGQ_NAME_LEN+1];
	struct mv_waitqueue_t   rxWaitQueue;
	struct mv_waitqueue_t   txWaitQueue;
	int                     maxMsgs;
	int                     maxMsgSize;
	int                     messages;
	char                    *buffer;
	int                     head;
	int                     tail;
	int                     waitRx;
	int                     waitTx;
};

static struct mvMsgQSTC *mvMsgQs;
static int              mv_num_queues = MV_QUEUES_DEF;

module_param(mv_num_queues, int, S_IRUGO);

#ifdef CONFIG_PROC_FS
/*******************************************************************************
* mvKernelExtMsgQ_read_proc_mem
*
* DESCRIPTION:
*       proc read data rooutine.
*       Use cat /proc/mvKernelExtMsgQ to show message queue list
*
* INPUTS:
*
* OUTPUTS:
*       None
*
* RETURNS:
*       Data length
*
* COMMENTS:
*
*******************************************************************************/
static int mvKernelExtMsgQ_proc_status_show(struct seq_file *m, void *v)
{
	int k;

	seq_puts(m, "id msgs waitRx waitTx");
#ifdef MV_MSGQ_STAT
	/*
	seq_printf(m, " tcount gcount wcount");
	*/
#endif
	seq_puts(m, " name\n");
	for (k = 1; k < mv_num_queues; k++) {
		struct mvMsgQSTC *q;
		struct mv_task *p;

		if (!mvMsgQs[k].flags)
			continue;
		q = mvMsgQs + k;

		seq_printf(m, "%d %d %d %d",
				k, q->messages, q->waitRx, q->waitTx);
#ifdef MV_MSGQ_STAT
		/*
	seq_printf(m, " %d %d %d", sem->tcount, sem->gcount, sem->wcount);
		*/
#endif
		if (q->name[0])
			seq_printf(m, " %s", q->name);
		seq_puts(m, "\n");

		for (p = q->rxWaitQueue.first; p; p = p->wait_next)
			seq_printf(m, "  rq=%d\n", p->task->pid);
		for (p = q->txWaitQueue.first; p; p = p->wait_next)
			seq_printf(m, "  tq=%d\n", p->task->pid);
	}

	return 0;
}
static int mvKernelExtMsgQ_proc_status_open(struct inode *inode, struct file *file)
{
	return single_open(file, mvKernelExtMsgQ_proc_status_show, NULL);
}
static const struct file_operations mvKernelExtMsgQ_read_proc_operations = {
	.open		= mvKernelExtMsgQ_proc_status_open,
	.read		= seq_read,
	.llseek		= seq_lseek,
	.release	= seq_release,
};
#endif /* CONFIG_PROC_FS */

/*******************************************************************************
* mvKernelExt_MsgQInit
*
* DESCRIPTION:
*       Initialize message queues support, create /proc for queues info
*
* INPUTS:
*       None
*
* OUTPUTS:
*       None
*
* RETURNS:
*       Non zero if successful
*       Zero if failed
*
* COMMENTS:
*
*******************************************************************************/
static int mvKernelExt_MsgQInit(void)
{
	if (mv_num_queues < MV_QUEUES_MIN)
		mv_num_semaphores = MV_QUEUES_MIN;

	mvMsgQs = kmalloc(mv_num_queues * sizeof(struct mvMsgQSTC), GFP_KERNEL);

	if (mvMsgQs == NULL)
		return 0;

	memset(mvMsgQs, 0, mv_num_queues * sizeof(struct mvMsgQSTC));

#ifdef CONFIG_PROC_FS
	if (!proc_create("mvKernelExtMsgQ", S_IRUGO, NULL, &mvKernelExtMsgQ_read_proc_operations))
		return -ENOMEM;
#endif

	return 1;
}

/*******************************************************************************
* mvKernelExt_DeleteAllMsgQ
*
* DESCRIPTION:
*       Destroys all message queues
*       This is safety action which is executed when all tasks closed
*
* INPUTS:
*       None
*
* OUTPUTS:
*       None
*
* RETURNS:
*       None
*
* COMMENTS:
*
*******************************************************************************/
static void mvKernelExt_DeleteAllMsgQ(void)
{
	int k;

	for (k = 1; k < mv_num_queues; k++) {
		if (mvMsgQs[k].flags) {
			mv_waitqueue_wake_all(&(mvMsgQs[k].rxWaitQueue));
			mv_waitqueue_wake_all(&(mvMsgQs[k].txWaitQueue));
			mv_waitqueue_cleanup(&(mvMsgQs[k].rxWaitQueue));
			mv_waitqueue_cleanup(&(mvMsgQs[k].txWaitQueue));
		}
		mvMsgQs[k].flags = 0;
	}
}

/*******************************************************************************
* mvKernelExt_MsgQCleanup
*
* DESCRIPTION:
*       Perform message queues cleanup actions before module unload
*
* INPUTS:
*       None
*
* OUTPUTS:
*       None
*
* RETURNS:
*       None
*
* COMMENTS:
*
*******************************************************************************/
static void mvKernelExt_MsgQCleanup(void)
{
	MV_GLOBAL_LOCK();

	if (mvMsgQs) {
		mvKernelExt_DeleteAllMsgQ();
		kfree(mvMsgQs);
	}

	mvMsgQs = NULL;

	MV_GLOBAL_UNLOCK();

#ifdef CONFIG_PROC_FS
	remove_proc_entry("mvKernelExtMsgQ", NULL);
#endif
}

/*******************************************************************************
* mvKernelExt_MsgQCreate
*
* DESCRIPTION:
*       Create a new message queue
*
* INPUTS:
*       arg   - pointer to structure with creation params and queue name
*
* OUTPUTS:
*       None
*
* RETURNS:
*       Positive value         - queue ID
*       -MVKERNELEXT_EINVAL    - invalid parameter passed
*       -MVKERNELEXT_ENOMEM    - queue array is full
*
*
* COMMENTS:
*
*******************************************************************************/
int mvKernelExt_MsgQCreate(
	const char *name,
	int maxMsgs,
	int maxMsgSize
)
{
	int k;
	struct mvMsgQSTC *q = NULL;

	MV_GLOBAL_LOCK();

	/* create queue */
	for (k = 1; k < mv_num_queues; k++) {
		if (mvMsgQs[k].flags == 0)
			break;
	}
	if (k >= mv_num_queues) {
		MV_GLOBAL_UNLOCK();
		return -MVKERNELEXT_ENOMEM;
	}
	q = mvMsgQs + k;

	memset(q, 0, sizeof(*q));
	q->flags = 3;
	MV_GLOBAL_UNLOCK();

	/* align max message size by 4 bytes */
	maxMsgSize = (maxMsgSize+3) & ~3;
	q->maxMsgs = maxMsgs;
	q->maxMsgSize = maxMsgSize;
	q->buffer = kmalloc((maxMsgSize + sizeof(int))*maxMsgs, GFP_KERNEL);
	if (q->buffer == NULL) {
		q->flags = 0;
		return -MVKERNELEXT_ENOMEM;
	}

	MV_GLOBAL_LOCK();
	q->flags = 1;
	mv_waitqueue_init(&(q->rxWaitQueue));
	mv_waitqueue_init(&(q->txWaitQueue));

	strncpy(q->name, name, MV_MSGQ_NAME_LEN);
	q->name[MV_MSGQ_NAME_LEN] = 0;

	MV_GLOBAL_UNLOCK();
	return k;
}
EXPORT_SYMBOL(mvKernelExt_MsgQCreate);

#define MSGQ_BY_ID(msgId) { \
	MV_GLOBAL_LOCK(); \
	if (unlikely(msgqId == 0 || msgqId >= mv_num_queues)) { \
		MV_GLOBAL_UNLOCK(); \
		return -MVKERNELEXT_EINVAL; \
	} \
	q = mvMsgQs + msgqId; \
	if (unlikely(q->flags != 1)) { \
		MV_GLOBAL_UNLOCK(); \
		return -MVKERNELEXT_EINVAL; \
	} \
}

#define CHECK_MSGQ() { \
	if (unlikely(q->flags != 1)) { \
		MV_GLOBAL_UNLOCK(); \
		return -MVKERNELEXT_EDELETED; \
	} \
}

/*******************************************************************************
* mvKernelExt_MsgQDelete
*
* DESCRIPTION:
*       Destroys semaphore
*
* INPUTS:
*       msgqId   - queue ID
*
* OUTPUTS:
*       None
*
* RETURNS:
*       Zero if successful
*       -MVKERNELEXT_EINVAL if bad ID passed
*
* COMMENTS:
*
*******************************************************************************/
int mvKernelExt_MsgQDelete(int msgqId)
{
	struct mvMsgQSTC *q;
	int timeOut;

	MSGQ_BY_ID(msgqId);

	q->flags = 2; /* deleting */

	for (timeOut = HZ; q->waitRx && timeOut; timeOut--) {
		mv_waitqueue_wake_all(&(q->rxWaitQueue));
		if (q->waitRx) {
			MV_GLOBAL_UNLOCK();
			schedule_timeout(1);
			MV_GLOBAL_LOCK();
		}
	}
	for (timeOut = HZ; q->waitTx && timeOut; timeOut--) {
		mv_waitqueue_wake_all(&(q->txWaitQueue));
		if (q->waitTx) {
			MV_GLOBAL_UNLOCK();
			schedule_timeout(1);
			MV_GLOBAL_LOCK();
		}
	}

	mv_waitqueue_cleanup(&(q->rxWaitQueue));
	mv_waitqueue_cleanup(&(q->txWaitQueue));

	MV_GLOBAL_UNLOCK();
	kfree(q->buffer);

	q->flags = 0;

	return 0;
}
EXPORT_SYMBOL(mvKernelExt_MsgQDelete);

/*******************************************************************************
* mvKernelExt_MsgQSend
*
* DESCRIPTION:
*       Send message to queue
*
* INPUTS:
*       msgqId       - Message queue Id
*       message      - message data pointer
*       messageSize  - message size
*       timeOut      - time out in miliseconds or
*                      -1 for WAIT_FOREVER or 0 for NO_WAIT
*       userspace    - called from userspace
*
* OUTPUTS:
*       None
*
* RETURNS:
*       Zero if successful
*       -MVKERNELEXT_EINVAL    - bad ID passed
*       -MVKERNELEXT_ETIMEOUT  - on timeout
*       -MVKERNELEXT_ENOMEM    - full and no wait
*       -MVKERNELEXT_EDELETED  - deleted
*
* COMMENTS:
*
*******************************************************************************/
int mvKernelExt_MsgQSend(
	int     msgqId,
	void    *message,
	int     messageSize,
	int     timeOut,
	int     userspace
)
{
	char    *msg;
	struct mvMsgQSTC *q;

	MSGQ_BY_ID(msgqId);

	while (q->messages == q->maxMsgs) {
		/* queue full */
		if (timeOut == 0) {
			MV_GLOBAL_UNLOCK();
			return -MVKERNELEXT_EFULL; /* ??? -MVKERNELEXT_ETIMEOUT */
		} else {
			struct mv_task *p;
			TASK_WILL_WAIT(current);
			q->waitTx++;
			if (timeOut != -1) {
#if HZ != 1000
				timeOut += 1000 / HZ - 1;
				timeOut /= 1000 / HZ;
#endif
				timeOut = mv_do_wait_on_queue_timeout(&(q->txWaitQueue), p, timeOut);
				CHECK_MSGQ();
				if (timeOut == 0) {
					q->waitTx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_ETIMEOUT;
				}
				if (timeOut == (unsigned long)(-1)) {
					q->waitTx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_EINTR;
				}
			} else { /* timeOut == -1, wait forever */
				if (unlikely(mv_do_wait_on_queue(&(q->txWaitQueue), p))) {
					q->waitTx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_EINTR;
				}
				CHECK_MSGQ();
			}
			q->waitTx--;
		}
	}

	/* put message */
	msg = q->buffer + q->head * (q->maxMsgSize + sizeof(int));
	if (messageSize > q->maxMsgSize)
		messageSize = q->maxMsgSize;

	*((int *)msg) = messageSize;
	if (userspace) {
		if (copy_from_user(msg+sizeof(int), message, messageSize)) {
			MV_GLOBAL_UNLOCK();
			return -MVKERNELEXT_EINVAL;
		}
	} else {
		memcpy(msg+sizeof(int), message, messageSize);
	}
	q->head++;
	if (q->head >= q->maxMsgs) /* round up */
		q->head = 0;
	q->messages++;

	/* signal to Recv thread if any */
	if (q->waitRx) {
		mv_waitqueue_wake_first(&(q->rxWaitQueue));
		/*
		if (unlikely(!q->rxWaitQueue.first))
			q->waitRx = 0;
		*/
	}

	MV_GLOBAL_UNLOCK();
	return 0;
}
EXPORT_SYMBOL(mvKernelExt_MsgQSend);

/*******************************************************************************
* mvKernelExt_MsgQRecv
*
* DESCRIPTION:
*       Receive message from queue
*
* INPUTS:
*       msgqId       - Message queue Id
*       messageSize  - size of buffer pointed by message
*       timeOut      - time out in miliseconds or
*                      -1 for WAIT_FOREVER or 0 for NO_WAIT
*       userspace    - called from userspace
*
* OUTPUTS:
*       message      - message data pointer
*
* RETURNS:
*       message size if successful
*       -MVKERNELEXT_EINVAL    - bad ID passed
*       -MVKERNELEXT_ETIMEOUT  - on timeout
*       -MVKERNELEXT_ENOMEM    - empty and no wait
*       -MVKERNELEXT_EDELETED  - deleted
*
* COMMENTS:
*
*******************************************************************************/
int mvKernelExt_MsgQRecv(
	int     msgqId,
	void    *message,
	int     messageSize,
	int     timeOut,
	int     userspace
)
{
	char    *msg;
	int  msgSize;
	struct mvMsgQSTC *q;

	MSGQ_BY_ID(msgqId);

	while (q->messages == 0) {
		/* queue empty */
		if (timeOut == 0) {
			MV_GLOBAL_UNLOCK();
			return -MVKERNELEXT_EEMPTY; /* ??? -MVKERNELEXT_ETIMEOUT */
		} else {
			struct mv_task *p;
			TASK_WILL_WAIT(current);
			q->waitRx++;
			if (timeOut != -1) {
#if HZ != 1000
				timeOut += 1000 / HZ - 1;
				timeOut /= 1000 / HZ;
#endif
				timeOut = mv_do_wait_on_queue_timeout(&(q->rxWaitQueue), p, timeOut);
				CHECK_MSGQ();
				if (timeOut == 0) {
					q->waitRx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_ETIMEOUT;
				}
				if (timeOut == (unsigned long)(-1)) {
					q->waitRx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_EINTR;
				}
			} else { /* timeOut == -1, wait forever */
				if (unlikely(mv_do_wait_on_queue(&(q->rxWaitQueue), p))) {
					q->waitRx--;
					MV_GLOBAL_UNLOCK();
					return -MVKERNELEXT_EINTR;
				}
				CHECK_MSGQ();
			}
			q->waitRx--;
		}
	}
	/* get message */
	msg = q->buffer + q->tail * (q->maxMsgSize + sizeof(int));
	msgSize = *((int *)msg);
	if (msgSize > messageSize)
		msgSize = messageSize;

	if (userspace) {
		if (copy_to_user(message, msg+sizeof(int), msgSize))
			msgSize = 0;
	} else {
		memcpy(message, msg+sizeof(int), msgSize);
	}
	q->tail++;
	if (q->tail >= q->maxMsgs) /* round up */
		q->tail = 0;
	q->messages--;

	/* signal to Recv thread if any */
	if (q->waitTx) {
		mv_waitqueue_wake_first(&(q->txWaitQueue));
		/*
		if (unlikely(!q->txWaitQueue.first))
			q->waitTx = 0;*/
	}

	MV_GLOBAL_UNLOCK();
	return msgSize;
}
EXPORT_SYMBOL(mvKernelExt_MsgQRecv);

/*******************************************************************************
* mvKernelExt_MsgQNumMsgs
*
* DESCRIPTION:
*       Return number of messages pending in queue
*
* INPUTS:
*       msgqId       - Message queue Id
*
* OUTPUTS:
*       None
*
* RETURNS:
*       numMessages  - number of messages pending in queue
*       -MVKERNELEXT_EINVAL    - bad ID passed
*
* COMMENTS:
*       None
*
*******************************************************************************/
int mvKernelExt_MsgQNumMsgs(int msgqId)
{
	int numMessages;
	struct mvMsgQSTC *q;

	MSGQ_BY_ID(msgqId);
	numMessages = q->messages;
	MV_GLOBAL_UNLOCK();

	return numMessages;
}
EXPORT_SYMBOL(mvKernelExt_MsgQNumMsgs);
