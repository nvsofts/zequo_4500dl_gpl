
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

*******************************************************************************/

#include <stdio.h>
#include <string.h>
#include <errno.h>
#include <fcntl.h>
#include <stdlib.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <sys/poll.h>
#include <stdbool.h>
#include <err.h>
#include <getopt.h>
#include <sys/times.h>
#include <sys/select.h>

#include "mv_ipc_wrapper.h"

#define IPC_MAX_MSG_SIZE 128 /* must align with mv_ipc.h */

void show_usage(void)
{
	fprintf(stderr,
		"Usage:\n"
		" mv_ipc_tool -h\n"
		"   Display this help.\n"
		"\n"
		"   Synopsis:\n"
		"	./mv_ipc_tool /dev/ipc-lnkX/chnY -<options>\n\n"
		"   Options:\n"

		"	-s, --status       :\n"
		"	    check status of the channel\n"
		"	-r, --read timeout :\n"
		"	    read one message from ipc channel\n"
		"	-w, --write msg    :\n"
		"	    write one message to ipc channel\n"
		"	-o, --open chn     :\n"
		"	    open the ipc channel\n"
		"	-c, --close chn    :\n"
		"	    close the ipc channel\n"
		"\n\n"
		"   Example:\n"
		"	./mv_ipc_tool /dev/ipc-lnk0/chn1 --write hello\n"
		"	./mv_ipc_tool /dev/ipc-lnk0/chn1 --read 5\n"
		"	./mv_ipc_tool /dev/ipc-lnk0/chn1 --status\n"
		"	./mv_ipc_tool /dev/ipc-lnk0/chn1 --open\n"
		"	./mv_ipc_tool /dev/ipc-lnk0/chn1 --close\n"
	);
}

static int dump_to_stdout(char *buf, ssize_t nr)
{
	int wfd, rc;

	wfd = fileno(stdout);
	rc = write(wfd, buf, nr);

	if (rc < 1)
		fprintf(stderr, "Cannot write to stdout.\n");

	return rc;
}

static int write_to_chn(int fd, char *msg)
{
	if (write(fd, msg, (size_t)(strlen(msg)+1)) < 0) {
		fprintf(stderr, "Failed to write message to channel.\n");
		return -1;
	}

	fprintf(stdout, "Write msg succeed\n");
	return 0;
}

static int read_from_chn(int fd, char *arg)
{
	char buf[IPC_MAX_MSG_SIZE];
	fd_set rd_fds;
	struct timeval timeout;
	int msg_size, ret;

	memset(buf, 0, sizeof(buf));

	timeout.tv_sec  = atoi(arg);
	timeout.tv_usec = 0;

	FD_ZERO(&rd_fds);
	FD_SET(fd, &rd_fds);

	ret = select(fd+1, &rd_fds, NULL, NULL, timeout.tv_sec ? &timeout : NULL);
	if (ret < 0)
		return -1;

	if (FD_ISSET(fd, &rd_fds)) {
		msg_size = read(fd, buf, IPC_MAX_MSG_SIZE);
		if (msg_size < 0) {
			fprintf(stderr, "Failed to read message from channel.\n");
			return -1;
		}
		fprintf(stdout, "Got msg from channel:\n");
		dump_to_stdout(buf, msg_size);
		fprintf(stdout, "\n");
	} else
		fprintf(stderr, "No message recieved.\n");

	return 0;
}


/* Purpose of below description is to improve readability
 * argv[1] - ipc file name: /dev/ipc-lnkX/chnY
 * optarg - first argument after options
 * argv[optind] - next argument after optarg
 * argv[optind + n] - the n+1 argument after options
 *
 * Example:
 * ./mv_ipc_tool /dev/ipc-lnk0/chn1 -w "hello"
 * argv[optind]="hello"
 */
int main(int argc, char *argv[])
{
	char *name;
	int fd, rc = 0;
	unsigned int state;
	int long_index = 0, opt = 0;

	/* Specifying the expected options */
	static struct option long_options[] = {
		{"help",	no_argument,		0,	'h' },
		{"status",	no_argument,		0,	's' },
		{"open",	no_argument,		0,	'o' },
		{"close",	no_argument,		0,	'c' },
		{"read",	required_argument,	0,	'r' },
		{"write",	required_argument,	0,	'w' },
		{0,		0,			0,	0}
	};

	if (argc < 3) {
		fprintf(stderr, "missing arguments\n");
		show_usage();
		exit(2);
	}

	name = argv[1];
	fprintf(stdout, "opening %s device.\n", name);
	fd = open(name, O_RDWR);
	if (fd <= 0) {
		fprintf(stderr, "Cannot open %s device.\n", name);
		exit(1);
	}

	opt = getopt_long(argc, argv, "chor:sw:", long_options, &long_index);
	if (opt == -1)
		goto fail_close;

	switch (opt) {
	case 'h':
		show_usage();
		break;
	case 's': /* get ipc channel status */
		rc = ioctl(fd, IPCW_IOC_GET_STAT, &state);
		fprintf(stdout, "channel status %s\n",
			state ? "attached" : "not attached");
		break;
	case 'o': /* open ipc channel */
		rc = ioctl(fd, IPCW_IOC_CHN_OPEN);
		if (!rc)
			fprintf(stdout, "channel opened\n");
		break;
	case 'c': /* close ipc channel */
		rc = ioctl(fd, IPCW_IOC_CHN_CLOSE);
		if (!rc)
			fprintf(stdout, "channel closed\n");
		break;
	case 'r': /* read from ipc channel */
		if (argc < 4) {
			fprintf(stderr, "missing read arguments\n");
			goto fail_close;
		}
		rc = read_from_chn(fd, optarg);
		break;
	case 'w': /* write to ipc channel */
		if (argc < 4) {
			fprintf(stderr, "missing write arguments\n");
			goto fail_close;
		}
		rc = write_to_chn(fd, optarg);
		break;
	default:
		show_usage();
		break;
	}

	if (rc < 0) {
		fprintf(stderr, "IPC Tool failed to perform action!\n");
		goto fail_close;
	}

	close(fd);

	return 0;

fail_close:
	close(fd);
	exit(2);
}
