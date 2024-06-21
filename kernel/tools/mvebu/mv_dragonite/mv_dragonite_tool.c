
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
#include <sys/mman.h>

#include "dragonite_xcat.h"

void show_usage(void)
{
	fprintf(stderr,
		"Usage:\n"
		" mv_dragonite_tool -h\n"
		"   Display this help.\n"
		"\n"
		"   Synopsis:\n"
		"	./mv_dragonite_tool /dev/<dragonite_name> -<options>\n\n"
		"   Options:\n"
		"	-m, --memtype m_type	: set memory direction to m_type (itcm or dtcm)\n"
		"	-t, --reset		: put dragonite into reset\n"
		"	-u, --unreset=unprotected : un-reset dragonite; If optional argument\n"
		"	    \"--unreset=unprotected\" is used access to whole dragonite memory is\n"
		"	    allowed. Otherwise all dragonite memory is protected instead of\n"
		"	    first 4KB of DTCM\n"
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		"	-s, --sendirq		: send irq to dragonite\n"
#endif
		"	-p, --irqpoll		: infinite poll for dragonite irq\n"
		"	-a, --mmap		: mmap first 4KB of DTCM, dumps first 32byte and munmap\n"
		"	-r, --read f_off m_size :\n"
		"	    read m_size from dragonite(ITCM or DTCM) from f_off offset\n"
		"	-w, --write f_off f_name f_size :\n"
		"	    write f_size bytes from f_name to dragonite(ITCM or DTCM) in f_off offset\n"
		"	-d, --uread m_type f_off m_size	:\n"
		"	    unify read m_size from m_type(ITCM/DTCM) at f_off offset\n"
		"	    This reach more 'atomic' read by unifying into DRAGONITE_IOC_READ ioctl\n"
		"	    following features: set memory direction, llsek and read syscall.\n"
		"	    This allows to use two processes which call the driver simultaneously.\n"
		"	-e, --uwrite m_type f_off buf	:\n"
		"	    unify write buf to m_type(ITCM/DTCM) at f_off offset\n"
		"	    This reach more 'atomic' write by unifying into DRAGONITE_IOC_WRITE ioctl\n"
		"	    following features: set memory direction, llsek and write syscall.\n"
		"	    This allows to use two processes which call the driver simultaneously.\n"
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		"	-c, --setipcchn chn_id status :\n"
		"	    enable/disable dragonite ipc channel #id\n"
#endif

		"\n\n"
		"   Example:\n"
		"	./mv_dragonite_tool /dev/dragonite --memtype itcm\n"
		"	./mv_dragonite_tool /dev/dragonite --write 0 RTOSDemo.bin 15312\n"
		"	./mv_dragonite_tool /dev/dragonite --read 0 15312 > itcm_read.bin\n"
		"	./mv_dragonite_tool /dev/dragonite --uread itcm 0 15312 > itcm_read.bin\n"
		"	./mv_dragonite_tool /dev/dragonite --uwrite dtcm 0 \"abcde\"\n"
		"	./mv_dragonite_tool /dev/dragonite --unreset\n"
		"	./mv_dragonite_tool /dev/dragonite --unreset=unprotected\n"
		"	./mv_dragonite_tool /dev/dragonite --reset\n"
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		"	./mv_dragonite_tool /dev/dragonite --sendirq\n"
#endif
		"	./mv_dragonite_tool /dev/dragonite --irqpoll\n"
		"	./mv_dragonite_tool /dev/dragonite --mmap\n"
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		"	./mv_dragonite_tool /dev/dragonite --setipcchn 1 1\n"
#endif

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

static int write_to_dragonite(int fd_dragonite, char *off, char *f_name, char *size)
{
	int rfd;
	ssize_t nr, nw, bsize;
	char *buf = NULL;

	/* open file with firmware */
	rfd = open(f_name, O_RDONLY);
	if (rfd <= 0) {
		fprintf(stderr, "Cannot open %s file.\n", f_name);
		return -1;
	}

	bsize = atoi(size);
	buf = malloc(bsize);
	if (buf == NULL) {
		fprintf(stderr, "Cannot malloc buf.\n");
		goto fail_write_close;
	}

	if (lseek(fd_dragonite, atoi(off), SEEK_SET) < 0)
		goto fail_write_free;

	nr = read(rfd, buf, bsize);
	if (nr < 0) {
		fprintf(stderr, "Cannot read from %s.\n", f_name);
		goto fail_write_free;
	}

	nw = write(fd_dragonite, buf, (size_t)nr);
	if (nw < 0) {
		fprintf(stderr, "Cannot write to %s.\n", f_name);
		goto fail_write_free;
	}

	free(buf);
	close(rfd);

	fprintf(stdout, "File name %s, size 0x%x\n", f_name, bsize);

	return 0;

fail_write_free:
	free(buf);
fail_write_close:
	close(rfd);
	return -1;
}

static int read_from_dragonite(int fd_dragonite, char *off, char *size)
{
	ssize_t nr, bsize;
	char *buf = NULL;

	bsize = atoi(size);
	buf = malloc(bsize);
	if (buf == NULL) {
		fprintf(stderr, "Cannot malloc buf.\n");
		return -1;
	}

	if (lseek(fd_dragonite, atoi(off), SEEK_SET) < 0)
		goto fail_read_free;

	nr = read(fd_dragonite, buf, bsize);
	if (nr < 0) {
		fprintf(stderr, "Cannot read from dragonite.\n");
		goto fail_read_free;
	}

	if (!dump_to_stdout(buf, nr))
		goto fail_read_free;

	free(buf);

	return 0;

fail_read_free:
	free(buf);
	return -1;
}

static int unify_write(int fd_dragonite, unsigned int dir, char *off, char *buf)
{
	int rc;
	ssize_t bsize;
	struct dragontie_filep drag_fp;

	bsize = strlen(buf);

	drag_fp.mem_direction = dir;
	drag_fp.offset = atoi(off);
	drag_fp.buf = buf;
	drag_fp.count = bsize;

	rc = ioctl(fd_dragonite, DRAGONITE_IOC_WRITE, &drag_fp);
	if (rc < 0) {
		fprintf(stderr, "unify_write failed.\n");
		goto fail_write;
	}

	return 0;

fail_write:
	return -1;
}

static int unify_read(int fd_dragonite, unsigned int dir, char *off, char *size)
{
	int rc;
	char *buf = NULL;
	ssize_t bsize;
	struct dragontie_filep drag_fp;

	bsize = atoi(size);
	buf = malloc(bsize);
	if (buf == NULL) {
		fprintf(stderr, "Cannot malloc buf.\n");
		return -1;
	}

	drag_fp.mem_direction = dir;
	drag_fp.offset = atoi(off);
	drag_fp.buf = buf;
	drag_fp.count = bsize;

	rc = ioctl(fd_dragonite, DRAGONITE_IOC_READ, &drag_fp);
	if (rc < 0) {
		fprintf(stderr, "unify_read failed.\n");
		goto fail_uread;
	}

	if (!dump_to_stdout(buf, bsize))
		goto fail_uread;

	free(buf);
	return 0;

fail_uread:
	free(buf);
	return -1;
}

static int poll_for_irq(int fd_dragonite)
{
	fd_set exc_fds;
	static int irq_count;

	FD_ZERO(&exc_fds);
	FD_SET(fd_dragonite, &exc_fds);

wait:
	/* Wait for event - can block indefinitely */
	if (select(fd_dragonite + 1, NULL, NULL, &exc_fds, NULL) < 0) {
		printf("Error while polling for irq\n");
		return -1;
	}

	if (FD_ISSET(fd_dragonite, &exc_fds)) {
		fprintf(stdout, "dbg: Got %d irqs.\n", ++irq_count);
		goto wait;
	}

	/* shouldn't get here */
	return -1;
}

static int mem_dir(const char *mem, unsigned int *dir)
{
	if (!strcmp(mem, "itcm"))
		*dir = ITCM_DIR;
	else if (!strcmp(mem, "dtcm"))
		*dir = DTCM_DIR;
	else {
		fprintf(stderr, "Unknown memory: %s\n", mem);
		return -1;
	}

	return 0;
}

/* Purpose of below description is to improve readability
 * argv[1] - dragonite file name: /dev/dragonite or /dev/dragonite_pci
 * optarg - first argument after options
 * argv[optind] - next argument after optarg
 * argv[optind + n] - the n+1 argument after options
 *
 * Example:
 * ./mv_dragonite_tool /dev/dragonite -w 0 RT2.bin 16000
 * optarg="0", argv[optind]="RT2.bin", argv[optind+1]="16000"
 */
int main(int argc, char *argv[])
{
	char *name;
	int fd, i, rc = 0;
	unsigned int mem_direction;
	char *mapped = NULL;
	enum dragonite_state state;
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	struct dragonite_ipcchn_set ipcchn_set;
	char dev_name[32];
#endif

	/* Specifying the expected options */
	static struct option long_options[] = {
		{"help",	no_argument,		0,	'h' },
		{"reset",	no_argument,		0,	't' },
		{"unreset",	optional_argument,	0,	'u' },
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		{"sendirq",	no_argument,		0,	's' },
#endif
		{"irqpoll",	no_argument,		0,	'p' },
		{"mmap",	no_argument,		0,	'a' },
		{"uread",	required_argument,	0,	'd' },
		{"uwrite",	required_argument,	0,	'e' },
		{"memtype",	required_argument,	0,	'm' },
		{"read",	required_argument,	0,	'r' },
		{"write",	required_argument,	0,	'w' },
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
		{"setipcchn",	required_argument,	0,	'c' },
#endif
		{0,		0,			0,	0}
	};
	int long_index = 0, opt = 0;

	if (argc < 3) {
		fprintf(stderr, "missing arguments\n");
		show_usage();
		exit(2);
	}

	/* Get dragonite file name: it should be dragontie or dragontie_pci */
	name = argv[1];
	if (!(strstr(name, "/dev/dragonite"))) {
		fprintf(stderr, "Trying to open not a dragonite device: \"%s\".\n", name);
		exit(1);
	}

	fprintf(stdout, "opening %s device.\n", name);
	fd = open(name, O_RDWR);
	if (fd <= 0) {
		fprintf(stderr, "Cannot open %s device.\n", name);
		exit(1);
	}

	opt = getopt_long(argc, argv, "ac:d:e:hm:n:pr:stu::w:",
					   long_options, &long_index);
	if (opt == -1)
		goto fail_close;

	switch (opt) {
	case 'h':
		show_usage();
		break;
	case 'm': /* set mem type (ITCM/DTCM) */
		rc = mem_dir(optarg, &mem_direction);
		if (rc < 0)
			goto fail_close;

		rc = ioctl(fd, DRAGONITE_IOC_SETMEM_TYPE, &mem_direction);
		break;
	case 't': /* put Dragonite into reset */
		state = D_RESET;
		rc = ioctl(fd, DRAGONITE_IOC_UNRESET, &state);
		break;
	case 'u': /* take Dragonite from reset */
		if ((optarg) && (!strcmp(optarg, "unprotected"))) {
			state = D_UNRESET;
			fprintf(stdout, "warning: running in unprotected mode\n");
		} else {
			/* set protection for whole tcm instead of 4KB of dtcm */
			state = D_UNRESET_AND_PROTECT_TCM;
			fprintf(stdout, "running in protected mode\n");
		}

		rc = ioctl(fd, DRAGONITE_IOC_UNRESET, &state);
		break;
#ifndef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	case 's': /* send irq to Dragonite */
		rc = ioctl(fd, DRAGONITE_IOC_SENDIRQ, NULL);
		break;
#endif
	case 'p': /* poll for Dragonite irq */
		rc = poll_for_irq(fd);
		break;
	case 'r': /* read from Dragontie */
		if (argc < 5) {
			fprintf(stderr, "missing read arguments\n");
			goto fail_close;
		}
		rc = read_from_dragonite(fd, optarg, argv[optind]);
		break;
	case 'w': /* write to Dragonite */
		if (argc < 6) {
			fprintf(stderr, "missing write arguments\n");
			goto fail_close;
		}
		rc = write_to_dragonite(fd, optarg, argv[optind], argv[optind+1]);
		break;
	case 'd': /* unify_read */
		if (argc < 6) {
			fprintf(stderr, "missing uread arguments\n");
			goto fail_close;
		}

		rc = mem_dir(optarg, &mem_direction);
		if (rc < 0)
			goto fail_close;

		rc = unify_read(fd, mem_direction, argv[optind], argv[optind+1]);

		break;
	case 'e': /* unify_write */
		if (argc < 6) {
			fprintf(stderr, "missing uwrite arguments\n");
			goto fail_close;
		}

		rc = mem_dir(optarg, &mem_direction);
		if (rc < 0)
			goto fail_close;

		rc = unify_write(fd, mem_direction, argv[optind], argv[optind+1]);

		break;
	case 'a': /* mmap */
		mapped = mmap(NULL, _4K, PROT_READ | PROT_WRITE, MAP_SHARED, fd, 0x0);

		for (i = 0; i < 32; i++) {
			if (!(i % 16))
				fprintf(stdout, "\n%p: ", mapped);
			else if (!(i % 4))
				fprintf(stdout, "\t");

			fprintf(stdout, "%02x", *mapped++);
		}
		fprintf(stdout, "\n");

		munmap(mapped, _4K);

		break;
#ifdef CONFIG_MV_DRAGONITE_IPC_SUPPORT
	case 'c': /* set DRAGONITE IPC channel */
		if (argc < 5) {
			fprintf(stderr, "missing set ipc channel arguments\n");
			goto fail_close;
		}

		memset(dev_name, 0, sizeof(dev_name));
		ipcchn_set.chn_id = atoi(optarg);
		ipcchn_set.chn_en = atoi(argv[optind]);
		ipcchn_set.dev_name = dev_name;

		rc = ioctl(fd, DRAGONITE_IOC_SET_IPCCHN, &ipcchn_set);

		fprintf(stdout, "DRAGONITE: %s ipc channel %d (%s)\n",
			ipcchn_set.chn_en ? "enable" : "disable",
			ipcchn_set.chn_id, ipcchn_set.dev_name);
		break;
#endif

	default:
		show_usage();
		break;
	}

	if (rc < 0) {
		fprintf(stderr, "Dragonite Tool failed to perform action!\n");
		goto fail_close;
	}

	close(fd);

	return 0;

fail_close:
	close(fd);
	exit(2);
}
